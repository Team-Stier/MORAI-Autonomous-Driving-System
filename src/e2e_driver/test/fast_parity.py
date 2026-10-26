"""fast_engine (CUDA graphs) vs the eager engine (engine.load + sample_trajectories): same val samples, same initial
noise, then the step time when each step brings 6 new images (newest frame of the 4 cameras + the 2 LiDAR BEVs; the
older 12 frames come from the ViT cache), as at 10 Hz.

  winpc (cwd src/e2e_driver/server): HF_HUB_OFFLINE=1 ~/claude-bench/venv-teacher/bin/python ../test/fast_parity.py \
      <deliver dir> <a run config.yaml for the data section> <step-0 data root> [n] [iters]
"""
import json
import os
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE.parent / "server"), str(HERE.parent / "server" / "vendor")]
sys.path += [p for p in os.environ.get("EXTRA_SITE", "").split(":") if p]   # pure-python deps from another venv (tests only)

import hydra.utils as hyu  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402
from omegaconf import OmegaConf  # noqa: E402

import engine  # noqa: E402
from fast_engine import FastEngine, fingerprints, layout  # noqa: E402
from morai_vla.processor import collate_fn  # noqa: E402


def main():
    deliver, cfg_path, root = sys.argv[1:4]
    n = int(sys.argv[4]) if len(sys.argv) > 4 else 4
    iters = int(sys.argv[5]) if len(sys.argv) > 5 else 30
    m = engine.load(deliver)
    fe = FastEngine(m, compile=os.environ.get("FAST_COMPILE", "1") == "1", use_graphs=os.environ.get("FAST_GRAPHS", "1") == "1")   # 0: debug
    cfg = OmegaConf.load(cfg_path)
    cfg.data.root = root
    dcfg = OmegaConf.to_container(cfg.data.val_dataset, resolve=True)
    dcfg.update(augment=False, max_samples=n, vla_preprocess_args={"_target_": "morai_vla.processor.get_preprocess_fn", "generation_mode": True})
    ds = hyu.instantiate(dcfg, _convert_="partial", model_config=m.config)
    cap = {}
    orig = m.diffusion.sample
    m.diffusion.sample = lambda *a, **k: cap.setdefault("a", orig(*a, **k))
    gpu = lambda b: {k: ({kk: vv.cuda() for kk, vv in v.items()} if isinstance(v, dict) else v.cuda() if torch.is_tensor(v) else v) for k, v in b.items()}
    rows = []
    with torch.inference_mode():
        for i in range(min(n, len(ds))):
            b = collate_fn([ds[i]], model_config=m.config)
            keys = fingerprints(b["tokenized_data"]["pixel_values"], b["tokenized_data"]["image_grid_thw"])
            b = gpu(b)
            ids = m.fuse_traj_tokens(b["tokenized_data"]["input_ids"], dict(ego_history_xyz=b["ego_history_xyz"], ego_history_rot=b["ego_history_rot"]))
            pos_ref, d_ref = fe.vlm.get_rope_index(ids, b["tokenized_data"]["image_grid_thw"], None, b["tokenized_data"]["attention_mask"])
            pos, d, _ = layout(b["tokenized_data"]["input_ids"], b["tokenized_data"]["image_grid_thw"], fe.image_token, fe.vlm.visual.spatial_merge_size)
            assert torch.equal(pos.cuda(), pos_ref) and torch.equal(d.cuda(), d_ref.reshape(1, 1)), "layout != get_rope_index"
            torch.manual_seed(i); cap.clear()
            xyz, _ = m.sample_trajectories(dict(b["tokenized_data"]), b["ego_history_xyz"], b["ego_history_rot"], num_samples=1, num_steps=10)
            ref = cap["a"].float()
            torch.manual_seed(i)
            x0 = torch.randn(1, *ref.shape[1:], device="cuda")
            act, st = fe.run(b["tokenized_data"], b["ego_history_xyz"], b["ego_history_rot"], x0, keys)
            xyz2, _ = m.action_space.action_to_traj(act, b["ego_history_xyz"][:, 0], b["ego_history_rot"][:, 0])
            r = dict(action_rel=float((act - ref).norm() / ref.norm()),
                     traj_m=float((xyz2[0, :, :2].float() - xyz[0, 0, :, :2].float()).norm(dim=-1).mean()), **st)
            rows.append(r); print(json.dumps(r), flush=True)
        # timing: 6 new images per step
        b = collate_fn([ds[0]], model_config=m.config)
        keys = fingerprints(b["tokenized_data"]["pixel_values"], b["tokenized_data"]["image_grid_thw"])
        b = gpu(b)
        lidar = b["tokenized_data"]["image_is_lidar"].bool().tolist()
        cams = [i for i, f in enumerate(lidar) if not f]
        fr = len(cams) // 4
        new = [cams[(c + 1) * fr - 1] for c in range(4)] + [i for i, f in enumerate(lidar) if f]
        ts, parts = [], []
        for it in range(iters):
            for i in new:
                fe.vit_cache.pop((lidar[i], keys[i]), None)
            torch.cuda.synchronize()
            s, e = torch.cuda.Event(enable_timing=True), torch.cuda.Event(enable_timing=True)
            s.record()
            fe.run(b["tokenized_data"], b["ego_history_xyz"], b["ego_history_rot"], torch.randn(1, 64, 2, device="cuda"), keys)
            e.record(); torch.cuda.synchronize()
            ts.append(s.elapsed_time(e)); parts.append(dict(fe.stats))
    med = lambda xs: round(float(np.median(xs[3:])), 1)
    print(json.dumps(dict(parity_traj_m_mean=round(float(np.mean([r["traj_m"] for r in rows])), 5),
                          parity_action_rel_max=round(max(r["action_rel"] for r in rows), 5),
                          step_ms=med(ts), p90=round(float(np.percentile(ts[3:], 90)), 1),
                          vit_ms=med([p["vit_ms"] for p in parts]), assemble_ms=med([p["gpu_assemble_ms"] for p in parts]),
                          llm_ms=med([p["llm_ms"] for p in parts]), expert_ms=med([p["expert_ms"] for p in parts]),
                          new_images=parts[-1]["new_images"],
                          prefix=parts[-1]["prefix_tokens"], padded=parts[-1]["padded"],
                          peak_gb=round(torch.cuda.max_memory_allocated() / 2**30, 2), engine=m.engine_info), indent=1))


if __name__ == "__main__":
    main()
