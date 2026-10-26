"""Student (MODEL_PLAN.md 3, 4 steps 5-6): Qwen3-VL-4B + a LiDAR ViT copied from its camera ViT + the action expert,
action_in/out_proj and (a, kappa) stats transplanted from T*. It is a TeacherVLA (same processor and tokens, history
fusion, LiDAR routing, expert reading the LLM KV, inference), distilled from a frozen T* that runs in the same step on
the same batch (same tokens, same LiDAR mask). Losses (loss_weights keys):

  kv      MSE(K_s, K_T) + MSE(V_s, V_T) per layer, averaged over the 36 layers (= the plan's sum / 36; at init ~13.5,
          the order of CE and FM), on the prefix the expert reads (no projection: same layer count, 8 KV heads x 128,
          same RoPE, same token positions)
  vit     camera and LiDAR ViT features per patch (main merger LayerNorm output, before the 2x2 merge; token grids are 1:1),
          MSE(P(f_s), f_T) with P a train-only linear 1024 -> 1152 (student -> teacher: a teacher -> student map would let
          P and f_s collapse together, the teacher target is fixed)
  fm      GT flow matching through the expert, with the gradient INTO the VLM KV (the teacher's Stage 2 detaches it):
          stage 1 trains the VLM to feed the frozen expert
  vkd     MSE(v_s, v_T) at the same (x_t, t), only on rows whose T* trajectories passed the rule check (`vkd_ok`)
  traj_ce discrete-trajectory CE; lm: the other answer tokens (CoC when present, the end token)
  coc_kl  KL(T || S) of the next-token distributions on the CoC answer tokens (optional)
Rows with CoC in the prefix skip fm and vkd: the expert only knows CoC-free prefixes (T* Stage 2 = inference format).
Stage 1 freezes the expert; stage 2 trains it at a low lr (lr_multiplier) and adds vkd with lambda_v 1 -> 0.2.
"""
import json, logging, zlib
from pathlib import Path

import torch, torch.distributed as dist, torch.nn as nn, torch.nn.functional as F
from alpamayo1_5_sft.models.sft_base_model import ReasoningVLAOutput
from alpamayo_r1.models.base_model import IGNORE_INDEX
from safetensors import safe_open
from transformers import Qwen3VLConfig, Qwen3VLForConditionalGeneration
from transformers.trainer_callback import TrainerCallback

from morai_vla.model import PARTS, TeacherVLA, exact_action_space, load_config

log = logging.getLogger(__name__)
TRANSPLANT = ("expert", "action_in_proj", "action_out_proj")


def transplant(model, ckpt):
    """expert.*, action_in_proj.*, action_out_proj.* of a teacher checkpoint into `model` (expert built from ITS text_config
    + the teacher's expert_cfg), strictly: every tensor loads with its exact shape and every parameter of those modules is
    loaded; only config-derived buffers (Fourier freqs) may be absent from the file, as in TeacherVLA.build. Streams one
    tensor at a time (the expert is 4.6 GB). Returns the tensor count."""
    mine = {k: v for k, v in model.state_dict(keep_vars=True).items() if k.split(".")[0] in TRANSPLANT}
    params = {n for n, _ in model.named_parameters() if n.split(".")[0] in TRANSPLANT}
    done, unexpected = set(), []
    for f in sorted(Path(ckpt).glob("*.safetensors")):
        with safe_open(f, "pt") as h:
            for k in h.keys():
                if k.split(".")[0] not in TRANSPLANT: continue
                if k not in mine: unexpected.append(k); continue
                t = h.get_tensor(k)
                if t.shape != mine[k].shape: raise RuntimeError(f"size mismatch for {k}: {tuple(t.shape)} in {ckpt}, {tuple(mine[k].shape)} here")
                with torch.no_grad(): mine[k].copy_(t)
                done.add(k)
    if params - done or unexpected:
        raise RuntimeError(f"non-strict transplant from {ckpt}: missing={sorted(params - done)[:5]} unexpected={unexpected[:5]}")
    log.info("transplanted %d tensors (%s) from %s", len(done), ", ".join(TRANSPLANT), ckpt)
    return len(done)


def hook_vit_features(model):
    """{camera, lidar}: per-patch features (main merger LayerNorm output) of the last call of each ViT (one per forward)."""
    feats, m = {}, model.vlm.model
    for name, enc in (("camera", m.visual), ("lidar", m.lidar_visual)):
        enc.merger.norm.register_forward_hook(lambda mod, args, out, name=name: feats.__setitem__(name, out))
    return feats


def kv_mse(kv_s, kv_t, attention_mask, start):
    """per layer MSE(K) + MSE(V) over the real (unpadded) tokens of the prefix [0, start); returns (layer mean, per-layer list)."""
    m = attention_mask[:, None, :start, None].float()                                 # (B, 1, S, 1)
    per = []
    for ls, lt in zip(kv_s.layers, kv_t.layers):
        n = m.sum() * ls.keys.shape[1] * ls.keys.shape[-1]
        per.append(sum(((a[:, :, :start].float() - b[:, :, :start].float()).pow(2) * m).sum() / n
                       for a, b in ((ls.keys, lt.keys), (ls.values, lt.values))))
    return sum(per) / len(per), per


def masked_mean(x, keep):
    keep = keep.to(x.dtype)
    return (x * keep).sum() / keep.sum().clamp(min=1)


class StudentVLA(TeacherVLA):
    PARTS = {**PARTS, "kd_proj": ["vit_kd"]}
    teacher = None                  # frozen T*: a plain attribute, not a submodule (not saved, optimised, cast or wrapped)

    def __init__(self, config, pretrained_modules=None, original_vocab_size=None):
        super().__init__(config, pretrained_modules, original_vocab_size)
        d = self.vlm.config.vision_config.hidden_size
        self.vit_kd = nn.ModuleDict({k: nn.Linear(d, config.teacher_vit_hidden) for k in ("camera", "lidar")})
        self.loss_weights = dict(kv=1.0, vit=1.0, fm=1.0, traj_ce=1.0, lm=1.0, vkd=0.0, coc_kl=0.0)
        self.vit_feats = hook_vit_features(self)
        self.eval_sums = {}                     # loss components of the current evaluation (EvalComponentMetric)

    # ------------------------------------------------------------------ construction
    @classmethod
    def build(cls, teacher=None, init=None, vlm_name_or_path="Qwen/Qwen3-VL-4B-Instruct", teacher_vlm="Qwen/Qwen3-VL-8B-Instruct",
              distill=True, train_parts=(), loss_weights=None, attn_implementation=None, dtype="bfloat16", vlm_overrides=None,
              strict=True, master_fp32=False, **load_kwargs):
        """teacher: T* checkpoint dir (until T* exists: the 1.5 release or the step-2 checkpoint; a tiny teacher in tests).
        It runs frozen in every step (distill) and is the source of a new student's expert.
        init: None = new student from the pretrained vlm_name_or_path (4B weights, LiDAR ViT copy, new rows for the
        3,761 added tokens, T*'s expert); 'random' = random VLM of that config + vlm_overrides (tests), T*'s expert;
        a directory = a saved student (stage 2, evaluation), loaded strictly. teacher=None: no distillation (evaluation)."""
        if init and init != "random":
            if (Path(init) / "modelopt_state.pth").exists():
                import morai_vla.qad  # noqa: F401  (ModelOpt restores the quantizers inside from_pretrained)
            model = super().build(init, vlm_name_or_path, train_parts, loss_weights, attn_implementation, dtype,
                                  strict=strict, master_fp32=master_fp32, **load_kwargs)
        else:
            assert teacher, "a new student takes its expert from the teacher checkpoint"
            extra = {"teacher_vit_hidden": _teacher_vit_hidden(teacher, teacher_vlm)}
            if init == "random":
                model = super().build("random:" + str(teacher), vlm_name_or_path, attn_implementation=attn_implementation, dtype=dtype,
                                      vlm_overrides=vlm_overrides, config_extra=extra)
            else:
                model = cls._from_pretrained_vlm(teacher, vlm_name_or_path, attn_implementation, getattr(torch, dtype), extra)
            transplant(model, teacher)
            model.set_stage(train_parts, loss_weights, master_fp32)
        if teacher and distill:
            T = TeacherVLA.build(teacher, vlm_name_or_path=teacher_vlm, attn_implementation=model.config.attn_implementation, dtype=dtype)
            model.attach_teacher(T)
        return model

    @classmethod
    def _from_pretrained_vlm(cls, teacher, vlm_name_or_path, attn_implementation, dtype, extra):
        config = load_config(teacher, vlm_name_or_path, attn_implementation, {}, None, extra)
        vlm = Qwen3VLForConditionalGeneration.from_pretrained(vlm_name_or_path, dtype=dtype, attn_implementation=config.attn_implementation)
        n0 = vlm.config.text_config.vocab_size
        vlm.resize_token_embeddings(config.vocab_size)                                # trajectory + special token rows: new
        vlm.config.text_config.vocab_size = vlm.config.vocab_size = config.vocab_size
        prev = torch.get_default_dtype(); torch.set_default_dtype(dtype)              # expert etc. built in bf16 (RAM), and like
        try:                                                                          # from_pretrained: fp32 rotary inv_freq kept
            model = cls(config, pretrained_modules={"vlm": vlm}, original_vocab_size=n0)
        finally:
            torch.set_default_dtype(prev)
        assert model.vlm.lm_head.weight is model.vlm.model.language_model.embed_tokens.weight     # 4B ties them
        exact_action_space(model)
        log.info("student VLM %s: %d -> %d token rows", vlm_name_or_path, n0, config.vocab_size)
        return model

    def attach_teacher(self, T):
        T.eval().requires_grad_(False)
        T.vit_feats = hook_vit_features(T)
        for name in TRANSPLANT:                     # config-derived buffers (rotary, Fourier freqs) must equal T*'s too; their
            for k, b in T.get_submodule(name).named_buffers():           # dtype may differ (fresh build vs reload): both are
                mine = self.get_submodule(name).get_buffer(k)            # promoted to fp32 where they are used
                assert torch.equal(mine.float().cpu(), b.float().cpu()), f"{name}.{k} differs from the teacher's"
        object.__setattr__(self, "teacher", T)

    # ------------------------------------------------------------------ training forward
    def forward(self, tokenized_data, ego_history_xyz=None, ego_history_rot=None, ego_future_xyz=None, ego_future_rot=None,
                labels_mask=None, vkd_ok=None, key=None, **kwargs):
        w, T = self.loss_weights, self.teacher
        assert T is not None or not any(w[k] for k in ("kv", "vit", "vkd", "coc_kl")), "distillation needs the teacher"
        td = dict(tokenized_data)
        is_lidar = td.pop("image_is_lidar", None)
        traj_data = dict(ego_history_xyz=ego_history_xyz, ego_history_rot=ego_history_rot, ego_future_xyz=ego_future_xyz, ego_future_rot=ego_future_rot)
        input_ids = self.fuse_traj_tokens(td.pop("input_ids"), traj_data)
        labels = torch.where(labels_mask, input_ids, IGNORE_INDEX)
        attn = td["attention_mask"]
        start = int((input_ids == self.config.traj_token_ids["future_start"]).nonzero()[-1, 1]) + 1
        has_coc = (input_ids == self.special_token_ids["cot_start"]).any(1)
        need_logits = w["traj_ce"] > 0 or w["lm"] > 0 or w["coc_kl"] > 0
        keep = input_ids.shape[1] - int(labels_mask.float().argmax(1)[labels_mask.any(1)].min()) + 1   # logits from the first
        labels = labels[:, -keep:]                                                                    # answer token on only
        losses = {}
        with torch.autocast("cuda", torch.bfloat16, enabled=input_ids.is_cuda):      # fp32 action inputs, bf16 weights (DeepSpeed)
            self.vit_feats.clear()
            if T is not None:
                with torch.no_grad():
                    if T.device != input_ids.device: T.to(input_ids.device)
                    T.vit_feats.clear()
                    T.vlm.model._image_is_lidar = is_lidar
                    t_out = T.vlm(input_ids=input_ids, use_cache=True, logits_to_keep=keep if w["coc_kl"] > 0 else 1, **td)
            self.vlm.model._image_is_lidar = is_lidar
            out = self.vlm(input_ids=input_ids, use_cache=True, logits_to_keep=keep if need_logits else 1, **td)
            if w["kv"] > 0:
                losses["kv"] = kv_mse(out.past_key_values, t_out.past_key_values, attn, start)[0]
            if w["vit"] > 0:
                vit = F.mse_loss(self.vit_kd["camera"](self.vit_feats["camera"]).float(), T.vit_feats["camera"].float())
                if "lidar" in self.vit_feats:   # a LiDAR-free batch has only the zero-weighted pass LidarQwen3VLModel runs while
                    p = self.vit_kd["lidar"](self.vit_feats["lidar"]).float()          # the LiDAR ViT trains: keep the same
                    vit = vit + (F.mse_loss(p, T.vit_feats["lidar"].float()) if "lidar" in T.vit_feats else 0 * p.sum())   # graph
                losses["vit"] = vit
            if need_logits:
                traj = ((labels >= self.future_token_start_idx) & (labels < self.future_token_start_idx + self.config.traj_vocab_size)
                        | (labels == self.special_token_ids["traj_future_start"]) | (labels == self.special_token_ids["traj_future_end"]))
                losses["traj_ce"] = self._compute_next_token_loss(out, labels, traj)
                other = labels.masked_fill(traj, IGNORE_INDEX)
                losses["lm"] = self._compute_next_token_loss(out, other, other != IGNORE_INDEX)
                sel = ((other != IGNORE_INDEX) & has_coc[:, None])[:, 1:]               # position p predicts answer token p + 1
                if w["coc_kl"] > 0 and bool(sel.any()):
                    ls, lt = out.logits[:, :-1][sel].float().log_softmax(-1), t_out.logits[:, :-1][sel].float().log_softmax(-1)
                    losses["coc_kl"] = F.kl_div(ls, lt, log_target=True, reduction="batchmean")
            fm_rows = ~has_coc
            if (w["fm"] > 0 or w["vkd"] > 0) and bool(fm_rows.any()):
                if self.training or key is None:
                    fut = self._process_traj_future_training(traj_data)
                else:                                   # evaluation: noise and t fixed by the sample keys, so checkpoints are
                    with torch.random.fork_rng(devices=[input_ids.device] if input_ids.is_cuda else []):   # compared on the
                        torch.manual_seed(zlib.crc32("|".join(key).encode()))                             # same draws
                        fut = self._process_traj_future_training(traj_data)
                v = self._velocity(out, input_ids, attn, fut["noisy_x"], fut["timesteps"], detach=False).float()
                per = lambda d: d.pow(2).mean((1, 2))
                losses["fm"] = masked_mean(per(v - (fut["x"] - fut["noise"]).float()), fm_rows)
                if w["vkd"] > 0 and (vkd_ok is not None or self.training):
                    assert vkd_ok is not None, "vkd needs the soft-target rule check: set data.train_dataset.rule_ok"
                    with torch.no_grad():
                        v_t = T._velocity(t_out, input_ids, attn, fut["noisy_x"], fut["timesteps"]).float()
                    losses["vkd"] = masked_mean(per(v - v_t), fm_rows & torch.as_tensor(vkd_ok, device=fm_rows.device))
        for k, x in losses.items():
            s = self.loss_sums.setdefault(k, [0.0, 0]); s[0] += float(x.detach()); s[1] += 1
            if not self.training:
                e = self.eval_sums.setdefault(k, [0.0, 0]); e[0] += float(x.detach()); e[1] += 1
        return ReasoningVLAOutput(loss=sum(w[k] * x for k, x in losses.items()))


def _teacher_vit_hidden(teacher, teacher_vlm):
    """the teacher's ViT width (1152 for the 1.5 release; a tiny teacher's vlm_overrides change it)."""
    ov = json.load(open(Path(teacher) / "config.json")).get("vlm_overrides") or {}
    return ov.get("vision_config", {}).get("hidden_size") or Qwen3VLConfig.from_pretrained(teacher_vlm).vision_config.hidden_size


class EvalComponentMetric(TrainerCallback):
    """adds eval_<key>, the mean of one loss component over the evaluation (all ranks), to the evaluation metrics, so the
    Trainer can keep the best checkpoint on it (metric_for_best_model): the weighted total moves with lambda_v (stage 2)."""

    def __init__(self, key="fm"):
        self.key = key

    def on_evaluate(self, args, state, control, metrics=None, model=None, **kw):
        m = getattr(model, "module", model)
        s, n = m.eval_sums.get(self.key, (0.0, 0)); m.eval_sums.clear()
        t = torch.tensor([s, n], dtype=torch.float64, device="cuda" if torch.cuda.is_available() else "cpu")
        if dist.is_available() and dist.is_initialized(): dist.all_reduce(t)
        metrics[f"eval_{self.key}"] = float(t[0] / t[1]) if t[1] > 0 else float("inf")


class LinearLossWeight(TrainerCallback):
    """loss weight `key` goes linearly from `start` to `end` over the run's optimizer steps (MODEL_PLAN 4.5: lambda_v 1 -> 0.2).
    Set from global_step at every step, so a resumed run continues the schedule."""

    def __init__(self, key="vkd", start=1.0, end=0.2):
        self.key, self.start, self.end = key, start, end

    def on_step_begin(self, args, state, control, model=None, **kw):
        m = getattr(model, "module", model)
        m.loss_weights[self.key] = self.start + (self.end - self.start) * state.global_step / max(state.max_steps, 1)
