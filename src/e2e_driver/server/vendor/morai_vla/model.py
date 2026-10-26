"""Teacher T* (MODEL_PLAN.md 3, 4.2-4.3): Alpamayo 1.5 (alpamayo_r1 classes, as the recipe does) + a LiDAR ViT that is a
copy of the camera ViT with its merger and 3 DeepStack mergers. One class for every run and for inference; the config
only decides which parts train and which losses run (recipe alpamayo1_5_sft @2e907da):

  step 2:     train lidar                              recipe Stage-1 loss (traj CE + the answer's end token)
  Stage 1-T:  train camera_vit, lidar, llm, lm_head    same, CoC off
  Stage 1-C:  same parts                               same + CoC LM, keyframes only
  Stage 2:    train expert                             flow matching on the stop-gradient KV of the prefix (no CoC)
"""
import copy, json, logging, time
from contextlib import nullcontext
from pathlib import Path

import hydra.utils as hyu, torch
from alpamayo.utils.checkpoint_utils import remap_targets
from alpamayo1_5_sft.models.sft_alpamayo_r1 import TrainableAlpamayoR1
from alpamayo1_5_sft.models.sft_base_model import ReasoningVLAOutput, TrainableReasoningVLA, TrajectoryFusionWithFutureMixin
from alpamayo_r1.config import AlpamayoR1Config
from alpamayo_r1.models.alpamayo_r1 import AlpamayoR1
from alpamayo_r1.models.base_model import IGNORE_INDEX
from transformers import Qwen3VLConfig, Qwen3VLForConditionalGeneration
from transformers.models.qwen3_vl.modeling_qwen3_vl import Qwen3VLModel
from transformers.trainer_callback import TrainerCallback

log = logging.getLogger(__name__)

A15_TO_A1 = {  # scripts/convert_checkpoint.py (recipe 2e907da), applied in memory instead of writing a converted copy
    "alpamayo1_5.models.action_in_proj.": "alpamayo_r1.models.action_in_proj.",
    "alpamayo1_5.models.delta_tokenizer.": "alpamayo_r1.models.delta_tokenizer.",
    "alpamayo1_5.action_space.": "alpamayo_r1.action_space.",
    "alpamayo1_5.diffusion.": "alpamayo_r1.diffusion.",
}
PARTS = {  # name -> module paths
    "lidar": ["vlm.model.lidar_visual"],
    "camera_vit": ["vlm.model.visual"],
    "llm": ["vlm.model.language_model"],            # includes the token embedding
    "lm_head": ["vlm.lm_head"],
    "expert": ["expert", "action_in_proj", "action_out_proj"],
}


class LidarQwen3VLModel(Qwen3VLModel):
    """Qwen3VLModel whose image encoder routes images flagged in `_image_is_lidar` to `lidar_visual`."""
    _image_is_lidar = None

    def get_image_features(self, pixel_values, image_grid_thw=None):
        m = self._image_is_lidar
        if m is None or not bool(m.any()):
            out = super().get_image_features(pixel_values, image_grid_thw)
            if not (self.training and any(p.requires_grad for p in self.lidar_visual.parameters())):
                return out
            # all-dropped batch while the LiDAR ViT trains: give it a zero-weighted pass so every rank has the same graph
            # (ZeRO-2/DDP reduce gradients in hook order; a rank skipping lidar_visual hung all-reduce on 8 GPUs, 10/25)
            n = int(image_grid_thw[0].prod())
            e, ds = self.lidar_visual(pixel_values[:n].type(self.visual.dtype), grid_thw=image_grid_thw[:1])
            zero = 0 * (e.sum() + sum(d.sum() for d in ds))
            return (out[0][0] + zero.to(out[0][0].dtype), *out[0][1:]), out[1]
        m = m.to(image_grid_thw.device).bool()
        if len(m) < len(image_grid_thw):
            # shortcut: generate() repeats each sample's images num_return_sequences times; exact for batch 1 only
            assert len(image_grid_thw) % len(m) == 0, (len(m), len(image_grid_thw))
            m = m.repeat(len(image_grid_thw) // len(m))
        n_patch = image_grid_thw.prod(-1); n_tok = (n_patch // self.visual.spatial_merge_size ** 2).tolist()
        pv = pixel_values.type(self.visual.dtype).split(n_patch.tolist())
        embeds, deep = [None] * len(n_tok), None
        for flag, enc in ((False, self.visual), (True, self.lidar_visual)):
            idx = torch.nonzero(m == flag).flatten().tolist()
            if not idx: continue
            e, ds = enc(torch.cat([pv[i] for i in idx]), grid_thw=image_grid_thw[idx])
            sizes = [n_tok[i] for i in idx]
            for i, x in zip(idx, e.split(sizes)): embeds[i] = x
            ds = [d.split(sizes) for d in ds]
            if deep is None: deep = [[None] * len(n_tok) for _ in ds]
            for layer, chunks in zip(deep, ds):
                for i, x in zip(idx, chunks): layer[i] = x
        return tuple(embeds), [torch.cat(layer) for layer in deep]


def _add_lidar_vit(vlm):
    vlm.model.__class__ = LidarQwen3VLModel
    vlm.model.lidar_visual = copy.deepcopy(vlm.model.visual)


def load_config(path, vlm_name_or_path, attn_implementation=None, vlm_overrides=None, expert_overrides=None, config_extra=None):
    """AlpamayoR1Config from a release/checkpoint config.json: targets remapped to alpamayo_r1, the public VLM, overrides.
    The tokenizer of vlm_name_or_path must give the release's trajectory token ids."""
    cfg = json.load(open(Path(path) / "config.json"))
    remap_targets(cfg, A15_TO_A1)
    for k in ("model_type", "architectures", "transformers_version", "dtype", "torch_dtype"): cfg.pop(k, None)
    cfg["vlm_name_or_path"] = vlm_name_or_path
    if attn_implementation: cfg["attn_implementation"] = attn_implementation
    if vlm_overrides is not None: cfg["vlm_overrides"] = vlm_overrides
    if expert_overrides: cfg["expert_cfg"] = {**cfg["expert_cfg"], **expert_overrides}
    cfg.update(config_extra or {})
    want_ids = cfg.pop("traj_token_ids", None)
    config = AlpamayoR1Config(**cfg)
    assert want_ids is None or config.traj_token_ids == want_ids, (config.traj_token_ids, want_ids)   # same tokenizer as the release
    return config


def exact_action_space(model):
    """(a, kappa) normalisation stats live in buffers built from config.json; a bf16 load rounds them (std 0.6810 -> 0.6797).
    Keep them exact in fp32: the student inherits them with the expert."""
    c = model.config.action_space_cfg
    model.action_space = hyu.instantiate(c).to(model.device)
    for k in ("accel_mean", "accel_std", "curvature_mean", "curvature_std"):
        assert float(getattr(model.action_space, k)) == torch.tensor(c[k]).item(), k


class TeacherVLA(TrajectoryFusionWithFutureMixin, AlpamayoR1):
    accepts_loss_kwargs = False      # our loss is a per-batch mean: let the HF Trainer divide by gradient_accumulation_steps
    PARTS = PARTS
    # borrowed from the recipe as is (history + future discrete token fusion comes from the mixin)
    _compute_next_token_loss = TrainableReasoningVLA._compute_next_token_loss
    gradient_checkpointing_enable = TrainableReasoningVLA.gradient_checkpointing_enable
    gradient_checkpointing_disable = TrainableReasoningVLA.gradient_checkpointing_disable
    tie_weights = TrainableReasoningVLA.tie_weights
    _process_traj_future_training = TrainableAlpamayoR1._process_traj_future_training
    _process_position_ids_qwen2_5_vl = TrainableAlpamayoR1._process_position_ids_qwen2_5_vl

    def __init__(self, config, pretrained_modules=None, original_vocab_size=None):
        super().__init__(config, pretrained_modules, original_vocab_size)
        _add_lidar_vit(self.vlm)
        self.loss_weights = dict(traj_ce=1.0, lm=1.0, fm=1.0)
        self.loss_sums = {}

    def _initialize_qwenvl3_vlm(self, config):
        """as upstream, plus `vlm_overrides` (a tiny same-architecture config for local tests)."""
        ov = getattr(config, "vlm_overrides", None)
        if not ov:
            return super()._initialize_qwenvl3_vlm(config)
        vc = Qwen3VLConfig.from_pretrained(config.vlm_name_or_path, dtype=config.model_dtype, attn_implementation=config.attn_implementation)
        for sub, kv in ov.items():
            for k, v in kv.items(): setattr(getattr(vc, sub), k, v)
        self.original_vocab_size = vc.text_config.vocab_size
        vc.text_config.vocab_size = vc.vocab_size = config.vocab_size
        vc.text_config._attn_implementation = vc.vision_config._attn_implementation = config.attn_implementation
        self.vlm = Qwen3VLForConditionalGeneration(vc)

    # ------------------------------------------------------------------ construction
    @classmethod
    def build(cls, init, vlm_name_or_path="Qwen/Qwen3-VL-8B-Instruct", train_parts=(), loss_weights=None,
              attn_implementation=None, dtype="bfloat16", vlm_overrides=None, expert_overrides=None, strict=True, master_fp32=False,
              config_extra=None, **load_kwargs):
        """init: an Alpamayo-1.5 release dir (lidar ViT copied from the camera ViT), one of our checkpoints, or
        'random:<release config dir>' for a random-init model from that config (+ overrides).
        master_fp32: keep the trainable parts in fp32 (bf16 autocast computes) so small updates are not rounded away when
        no DeepSpeed fp32 master copy exists (Stage 2). config_extra: more config.json keys (the student's)."""
        dtype = getattr(torch, dtype)
        random_init = init.startswith("random:")
        path = Path(init.removeprefix("random:"))
        config = load_config(path, vlm_name_or_path, attn_implementation, vlm_overrides, expert_overrides, config_extra)
        if random_init:
            torch.manual_seed(0)
            model = cls(config).to(dtype)
        else:
            model, info = cls.from_pretrained(path, config=config, dtype=dtype, output_loading_info=True, **load_kwargs)
            if not getattr(model, "is_quantized", False) and any(p.dtype != dtype for p in model.parameters() if p.is_floating_point()):
                model.to(dtype)                                  # fp32 load: LLM + expert came up bf16 (their sub-configs say so)
            params = {n for n, _ in model.named_parameters()}
            lidar_missing = sum(".lidar_visual." in k for k in info["missing_keys"])
            missing = [k for k in info["missing_keys"] if ".lidar_visual." not in k and k in params]
            # buffers the release does not store (Fourier freqs, (a, kappa) stats) are rebuilt from config.json: check them
            buffers = [k for k in info["missing_keys"] if ".lidar_visual." not in k and k not in params]
            assert all(torch.isfinite(model.get_buffer(k)).all() and model.get_buffer(k).abs().sum() > 0 for k in buffers), buffers
            log.info("loaded %s: missing params %d (lidar %d), config-derived buffers %d, unexpected %d, mismatched %d", path,
                     len(missing), lidar_missing, len(buffers), len(info["unexpected_keys"]), len(info["mismatched_keys"]))
            if strict and (missing or info["unexpected_keys"] or info["mismatched_keys"]):
                raise RuntimeError(f"non-strict load from {path}: missing={missing[:10]} unexpected={info['unexpected_keys'][:10]} "
                                   f"mismatched={info['mismatched_keys'][:10]}")
            if lidar_missing:                                         # release checkpoint: LiDAR path starts as the camera path
                model.vlm.model.lidar_visual.load_state_dict(model.vlm.model.visual.state_dict())
                log.info("lidar_visual initialised from the camera ViT")
        exact_action_space(model)
        model.set_stage(train_parts, loss_weights, master_fp32)
        return model

    def set_stage(self, train_parts, loss_weights=None, master_fp32=False):
        self.loss_weights.update(loss_weights or {})
        self.requires_grad_(False)
        for part in train_parts:
            for path in self.PARTS[part]:
                self.get_submodule(path).requires_grad_(True)
                if master_fp32: self.get_submodule(path).float()
        self.vlm_trains = any(p.requires_grad for p in self.vlm.parameters())
        n = sum(p.numel() for p in self.parameters() if p.requires_grad)
        log.info("stage %s: %.3fB trainable, losses %s", list(train_parts), n / 1e9, self.loss_weights)

    # ------------------------------------------------------------------ training forward
    def forward(self, tokenized_data, ego_history_xyz=None, ego_history_rot=None, ego_future_xyz=None, ego_future_rot=None,
                labels_mask=None, **kwargs):
        w = self.loss_weights
        self.vlm.model._image_is_lidar = tokenized_data.pop("image_is_lidar", None)
        input_ids = tokenized_data.pop("input_ids")
        traj_data = dict(ego_history_xyz=ego_history_xyz, ego_history_rot=ego_history_rot, ego_future_xyz=ego_future_xyz, ego_future_rot=ego_future_rot)
        input_ids = self.fuse_traj_tokens(input_ids, traj_data)
        labels = torch.where(labels_mask, input_ids, IGNORE_INDEX)
        need_ce, need_fm = w["traj_ce"] > 0 or w["lm"] > 0, w["fm"] > 0
        with nullcontext() if self.vlm_trains else torch.no_grad():
            out = self.vlm(input_ids=input_ids, use_cache=need_fm, logits_to_keep=0 if need_ce else 1, **tokenized_data)
        losses = {}
        if need_ce:
            traj = ((labels >= self.future_token_start_idx) & (labels < self.future_token_start_idx + self.config.traj_vocab_size)
                    | (labels == self.special_token_ids["traj_future_start"]) | (labels == self.special_token_ids["traj_future_end"]))
            losses["traj_ce"] = self._compute_next_token_loss(out, labels, traj)
            other = labels.masked_fill(traj, IGNORE_INDEX)
            losses["lm"] = self._compute_next_token_loss(out, other, other != IGNORE_INDEX)
        if need_fm:
            losses["fm"] = self._flow_matching(out, input_ids, tokenized_data["attention_mask"], traj_data)
        for k, v in losses.items():
            s = self.loss_sums.setdefault(k, [0.0, 0]); s[0] += float(v.detach()); s[1] += 1
        return ReasoningVLAOutput(loss=sum(w[k] * v for k, v in losses.items()))

    def _flow_matching(self, out, input_ids, attention_mask, traj_data):
        """recipe Stage-2 expert loss on the KV cropped after the last <traj_future_start>, with stop-gradient (AR1 5.1)."""
        fut = self._process_traj_future_training(traj_data)
        pred = self._velocity(out, input_ids, attention_mask, fut["noisy_x"], fut["timesteps"])
        return self.diffusion.compute_loss_from_pred(training_data=fut, pred=pred)

    def _velocity(self, out, input_ids, attention_mask, noisy_x, timesteps, detach=True):
        """expert velocity at (x_t, t) reading the prefix KV of `out`, cropped after the last <traj_future_start>.
        detach=False lets the loss reach the VLM through the KV (the student)."""
        start = (input_ids == self.config.traj_token_ids["future_start"]).nonzero()[-1, 1] + 1
        x = self.action_in_proj(noisy_x, timesteps)
        kv = out.past_key_values
        assert kv is not None, "flow matching needs the KV cache: run this stage with gradient_checkpointing: false"
        kv.crop(int(start))
        if detach:
            for layer in kv.layers: layer.keys, layer.values = layer.keys.detach(), layer.values.detach()
        B, n = x.shape[:2]
        pos = self._process_position_ids_qwen2_5_vl(out, B, n, x.device)
        mask = None
        if not bool(attention_mask[:, :start].all()):                    # left padding inside the prefix (batch > 1)
            if self.config.attn_implementation == "flash_attention_2":
                raise ValueError("padded batches with the expert need sdpa/eager attention (or per_device_train_batch_size 1)")
            keep = torch.cat([attention_mask[:, :start], attention_mask.new_ones(B, n)], 1).bool()
            mask = torch.zeros(B, 1, n, keep.shape[1], dtype=x.dtype, device=x.device).masked_fill(~keep[:, None, None], torch.finfo(x.dtype).min)
        h = self.expert(inputs_embeds=x, position_ids=pos, past_key_values=kv, attention_mask=mask, use_cache=True,
                        **({"is_causal": False} if self.config.expert_non_causal_attention else {})).last_hidden_state
        return self.action_out_proj(h[:, -n:]).view(-1, *self.action_space.get_action_space_dims())

    # ------------------------------------------------------------------ inference (no CoC: prefix -> <traj_future_start> -> expert)
    @torch.no_grad()
    @torch.autocast("cuda", dtype=torch.bfloat16)                                  # as upstream inference; Euler state stays fp32
    def sample_trajectories(self, tokenized_data, ego_history_xyz, ego_history_rot, num_samples=1, num_steps=10, velocity_at=None, **_):
        """MODEL_PLAN 1: no text generation; prefill the prefix (ending in <traj_future_start>) once, Euler flow matching.
        velocity_at=(x_t (B*num_samples, 64, 2), t (B*num_samples,)): also return the velocity there, read off the same
        prefill (soft targets: one prefill for the K trajectories and the fixed (x_t, t) points)."""
        self.vlm.model._image_is_lidar = tokenized_data.pop("image_is_lidar", None)
        input_ids = self.fuse_traj_tokens(tokenized_data.pop("input_ids"), dict(ego_history_xyz=ego_history_xyz, ego_history_rot=ego_history_rot))
        assert (input_ids[:, -1] == self.config.traj_token_ids["future_start"]).all(), "build the prompt with generation_mode=True"
        rep = lambda t: t.repeat_interleave(num_samples, 0)
        out = self.vlm(input_ids=input_ids, use_cache=True, logits_to_keep=1, **tokenized_data)
        kv, B = out.past_key_values, input_ids.shape[0] * num_samples
        kv.batch_repeat_interleave(num_samples)
        prefix = kv.get_seq_length(); n = self.action_space.get_action_space_dims()[0]
        pos = (torch.arange(n, device=input_ids.device)[None, None] + rep(out.rope_deltas).view(1, B, 1) + prefix).expand(3, B, n)
        attn = rep(tokenized_data["attention_mask"]).bool()
        mask = torch.zeros(B, 1, n, prefix + n, dtype=self.expert.dtype, device=input_ids.device)
        mask[:, :, :, :prefix] = mask[:, :, :, :prefix].masked_fill(~attn[:, None, None], torch.finfo(mask.dtype).min)

        def step_fn(x, t):
            e = self.action_in_proj(x, t)
            h = self.expert(inputs_embeds=e, position_ids=pos, past_key_values=kv, attention_mask=mask, use_cache=True,
                            **({"is_causal": False} if self.config.expert_non_causal_attention else {})).last_hidden_state
            kv.crop(prefix)
            return self.action_out_proj(h[:, -n:]).view(-1, *self.action_space.get_action_space_dims())

        cfg, impl = self.expert.config, self.expert.config._attn_implementation
        cfg._attn_implementation = "sdpa"                          # FA2 rejects the 4D mask over K repeated samples (t2 10/26)
        try:
            v = None if velocity_at is None else step_fn(velocity_at[0], velocity_at[1].view(B, 1, 1)).float().view(-1, num_samples, n, 2)
            action = self.diffusion.sample(batch_size=B, step_fn=step_fn, device=input_ids.device, return_all_steps=False, inference_step=num_steps)
        finally:
            cfg._attn_implementation = impl
        xyz, rot = self.action_space.action_to_traj(action, rep(ego_history_xyz[:, 0]), rep(ego_history_rot[:, 0]))
        xyz, rot = xyz.view(-1, num_samples, n, 3), rot.view(-1, num_samples, n, 3, 3)
        return (xyz, rot) if v is None else (xyz, rot, v)


class LossComponentsCallback(TrainerCallback):
    """per log line: mean of each loss component since the previous line (HF logs only the weighted sum), seconds per
    optimizer step and peak GPU memory (for batch/memory profiling)."""
    _t = _step = None

    def on_log(self, args, state, control, logs=None, model=None, **kw):
        m = getattr(model, "module", model)
        if m is None or not getattr(m, "loss_sums", None): return
        tag = "eval_" if any(k.startswith("eval_") for k in (logs or {})) else ""      # evaluation forward passes
        parts = {tag + k: round(s / max(n, 1), 5) for k, (s, n) in m.loss_sums.items()}
        m.loss_sums.clear()
        now = time.time()
        if not tag and self._t is not None and state.global_step > self._step:
            parts["s_per_step"] = round((now - self._t) / (state.global_step - self._step), 3)
        if not tag: self._t, self._step = now, state.global_step
        if torch.cuda.is_available():
            parts["peak_mem_gb"] = round(torch.cuda.max_memory_allocated() / 2**30, 2)
        if state.is_world_process_zero:
            print(json.dumps({"step": state.global_step, **parts}), flush=True)
            with open(Path(args.output_dir) / "loss_components.jsonl", "a") as f:
                f.write(json.dumps({"step": state.global_step, **parts}) + "\n")
