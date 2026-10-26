"""Expert-free Stage 1 monitoring. The frozen expert reads a KV that step 2 / Stage 1 keep moving, so its trajectories are
no Stage 1 metric until Stage 2 (README "단계 B 결과"). On fixed val + val_asset samples:
  ce_on / ce_off   teacher-forced trajectory CE with LiDAR present / forced off (camera-only stalling -> raise dropout)
  ade / fde        the 128 discrete trajectory tokens generated greedily from the deployment prefix (no CoC), decoded by the
                   model's DiscreteTrajectoryTokenizer to (a, kappa) and integrated; 6.4 s, vs GT and constant velocity
Rows are the first n of check_release's 1,000-sample set (same seed/stride), so BF16 references there match per sample.
Used by scripts/eval_stage1.py and, at every eval interval, by Stage1EvalCallback (configs/stage1t.yaml, stage1c.yaml)."""
import json, logging
from pathlib import Path

import numpy as np, torch
from transformers import LogitsProcessor, LogitsProcessorList
from transformers.trainer_callback import TrainerCallback

from morai_vla.data import MoraiDataset
from morai_vla.processor import collate_fn, get_preprocess_fn

log = logging.getLogger(__name__)
DROP = ("camera_frames", "bev_frames", "camera_times", "bev_times")


class OnlyTrajBins(LogitsProcessor):
    """greedy decoding restricted to the future-trajectory bins [lo, lo + n)."""
    def __init__(self, lo, n): self.lo, self.n = lo, n

    def __call__(self, input_ids, scores):
        out = torch.full_like(scores, float("-inf")); out[:, self.lo:self.lo + self.n] = scores[:, self.lo:self.lo + self.n]
        return out


def tokens_to_xyz(tokenizer, ids, start, hist_xyz, hist_rot):
    """vocabulary ids (B, 128) -> future xyz (B, 64, 3) in the ego frame; hist_* (B, T, ...) of one trajectory group.
    The tokenizer's action space is a plain fp32 CPU object (not moved with the model)."""
    return tokenizer.decode(hist_xyz.float().cpu(), hist_rot.float().cpu(), (ids - start).cpu())[0]


@torch.no_grad()
def discrete_trajectory(model, tokenized_data, ego_history_xyz, ego_history_rot):
    """prompt from get_preprocess_fn(generation_mode=True) (ends in <traj_future_start>) -> (B, 64, 3)."""
    model.vlm.model._image_is_lidar = tokenized_data.pop("image_is_lidar", None)
    ids = model.fuse_traj_tokens(tokenized_data.pop("input_ids"), dict(ego_history_xyz=ego_history_xyz, ego_history_rot=ego_history_rot))
    assert (ids[:, -1] == model.config.traj_token_ids["future_start"]).all(), "build the prompt with generation_mode=True"
    n, lo = model.config.tokens_per_future_traj, model.future_token_start_idx
    with torch.autocast("cuda", dtype=torch.bfloat16):
        out = model.vlm.generate(input_ids=ids, **tokenized_data, max_new_tokens=n, do_sample=False, use_cache=True,   # GC configs turn the cache off
                                 logits_processor=LogitsProcessorList([OnlyTrajBins(lo, model.traj_tokenizer.vocab_size)]),
                                 pad_token_id=model.tokenizer.pad_token_id)
    return tokens_to_xyz(model.traj_tokenizer, out[:, -n:], lo, ego_history_xyz[:, 0], ego_history_rot[:, 0])


def const_vel(hist_xyz, n=64):
    v = (hist_xyz[-1, :2] - hist_xyz[-6, :2]) / 0.5                               # last 0.5 s of history (check_release)
    return torch.arange(1, n + 1)[:, None] * 0.1 * v[None]


class EvalSamples(torch.utils.data.Dataset):
    """per row: [generation prompt, teacher-forced with LiDAR, teacher-forced without LiDAR], decoded once."""
    def __init__(self, root, model_config, n=200, keep=None):
        self.ds = MoraiDataset(root, ["val", "val_asset"], frame_stride=10, max_samples=1000, augment=False, seed=1)
        self.ds.rows = self.ds.rows.iloc[:n].reset_index(drop=True)              # rows are in random order: a random subset
        self.idx = list(range(len(self.ds.rows)))[keep] if keep else list(range(len(self.ds.rows)))
        self.gen, self.tf = get_preprocess_fn(model_config, generation_mode=True), get_preprocess_fn(model_config)

    def __len__(self):
        return len(self.idx)

    def __getitem__(self, j):
        k = self.idx[j]; s = self.ds.sample(k); out = []
        for fn, d in ((self.gen, dict(s)), (self.tf, dict(s)), (self.tf, {**s, "bev_frames": None})):
            td = fn(data=d)                                                       # also sets d's label_components, generation_mode
            out.append({key: v for key, v in d.items() if key not in DROP} | {"tokenized_data": td})
        return k, str(self.ds.rows.cell.iloc[k]), out


def evaluate(model, samples, num_workers=4):
    """rows of {key, cell, ce_on, ce_off, ade, fde, cv_ade, cv_fde}. Leaves model.loss_sums as it found them."""
    m = getattr(model, "module", model); saved, m.loss_sums = m.loss_sums, {}; was_training = m.training; m.eval()
    dev = lambda x: {k: dev(v) if isinstance(v, dict) else v.cuda() if torch.is_tensor(v) else v for k, v in x.items()}
    rows = []
    try:
        for k, cell, (g, on, off) in torch.utils.data.DataLoader(samples, batch_size=None, num_workers=num_workers, collate_fn=lambda s: s):
            r = {"key": g["key"], "cell": cell}
            for name, item in (("ce_on", on), ("ce_off", off)):
                m.loss_sums = {}
                with torch.no_grad(), torch.autocast("cuda", dtype=torch.bfloat16):
                    m(**dev(collate_fn([item], model_config=m.config)))
                r[name] = m.loss_sums["traj_ce"][0]
            b = dev(collate_fn([g], model_config=m.config))
            hist, gt = b["ego_history_xyz"][0, 0].float().cpu(), b["ego_future_xyz"][0, 0, :, :2].float().cpu()
            xy = discrete_trajectory(m, b["tokenized_data"], b["ego_history_xyz"], b["ego_history_rot"])[0, :, :2]
            for tag, p in (("", xy), ("cv_", const_vel(hist))):
                e = (p - gt).norm(dim=-1); r[tag + "ade"], r[tag + "fde"] = float(e.mean()), float(e[-1])
            rows.append(r)
            if len(rows) % 25 == 0: log.info("stage1_eval: %d / %d samples", len(rows), len(samples))
    finally:
        m.loss_sums = saved; m.train(was_training)
    return rows


def summarize(rows):
    keys = ("ce_on", "ce_off", "ade", "fde", "cv_ade", "cv_fde")
    mean = lambda rs: {k: round(float(np.mean([r[k] for r in rs])), 4) for k in keys} | {"n": len(rs)}
    return mean(rows) | {"by_cell": {c: mean([r for r in rows if r["cell"] == c]) for c in sorted({r["cell"] for r in rows})}}


class Stage1EvalCallback(TrainerCallback):
    """after each HF evaluation: evaluate() on n fixed samples, sharded over ranks (ZeRO-2 keeps full weights on every
    rank), gathered on rank 0 -> <output_dir>/stage1_eval.jsonl."""
    def __init__(self, root, n=200, num_workers=4):
        self.root, self.n, self.num_workers, self.samples = root, n, num_workers, None

    def on_evaluate(self, args, state, control, model=None, **kw):
        dist = torch.distributed.is_available() and torch.distributed.is_initialized()
        rank, world = (torch.distributed.get_rank(), torch.distributed.get_world_size()) if dist else (0, 1)
        m = getattr(model, "module", model)
        if self.samples is None:
            self.samples = EvalSamples(self.root, m.config, self.n, keep=slice(rank, None, world))
        rows = evaluate(m, self.samples, self.num_workers)
        if dist:
            parts = [None] * world; torch.distributed.all_gather_object(parts, rows); rows = [r for p in parts for r in p]
        if state.is_world_process_zero:
            res = {"step": state.global_step, **summarize(rows)}
            print(json.dumps({k: v for k, v in res.items() if k != "by_cell"}), flush=True)
            with open(Path(args.output_dir) / "stage1_eval.jsonl", "a") as f: f.write(json.dumps(res) + "\n")
