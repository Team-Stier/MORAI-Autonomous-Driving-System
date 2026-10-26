"""Engine (real NVFP4/FP8 GEMMs, server/engine.py) vs the QAD student it was delivered as (ModelOpt fake quantization)
and vs the same weights in plain BF16: the same val samples, the same initial noise, inference format (no CoC, Euler 10,
one trajectory). eng_vs_qad should be far below bf16_vs_qad (the size of the quantization effect itself).

  winpc (cwd src/e2e_driver/server):
  MORAI_VLA_PYLIB=~/morai-student/pylib ~/claude-bench/venv-teacher/bin/python ../test/engine_parity.py \
      <deliver dir> <the QAD run's config.yaml> <step-0 data root> [n] [vlm]
"""
import json
import os
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE.parent / "server"), str(HERE.parent / "server" / "vendor"), os.path.expanduser(os.environ["MORAI_VLA_PYLIB"])]

import hydra.utils as hyu  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402
from omegaconf import OmegaConf  # noqa: E402

import engine  # noqa: E402
from fast_engine import FastEngine, fingerprints  # noqa: E402
from morai_vla.processor import collate_fn  # noqa: E402
from morai_vla.student import StudentVLA  # noqa: E402


def main():
    deliver, cfg_path, root = sys.argv[1:4]
    n = int(sys.argv[4]) if len(sys.argv) > 4 else 16
    cfg = OmegaConf.load(cfg_path)
    vlm = sys.argv[5] if len(sys.argv) > 5 else cfg.model.vlm_name_or_path
    qad = StudentVLA.build(init=str(Path(deliver) / "final"), vlm_name_or_path=vlm, attn_implementation="sdpa", dtype="bfloat16",
                           device_map={"": 0}).eval()
    bf16 = engine.load(deliver, vlm, quantize=False)
    eng = engine.load(deliver, vlm)
    fast = FastEngine(eng)
    print(json.dumps(eng.engine_info))
    ref, got = qad.state_dict(), bf16.state_dict()                                   # the streamed load = the HF load
    diff = [k for k in got if k in ref and not torch.equal(got[k].cpu(), ref[k].cpu())]
    assert not diff and len(set(got) - set(ref)) == 0, (diff[:10], sorted(set(got) - set(ref))[:10])
    print("streamed BF16 load == StudentVLA.build:", len(got), "tensors")
    cfg.data.root = root
    dcfg = OmegaConf.to_container(cfg.data.val_dataset, resolve=True)
    dcfg.update(augment=False, max_samples=n, vla_preprocess_args={"_target_": "morai_vla.processor.get_preprocess_fn", "generation_mode": True})
    ds = hyu.instantiate(dcfg, _convert_="partial", model_config=qad.config)
    rows = []
    with torch.inference_mode():
        for i in range(min(n, len(ds))):
            item = ds[i]
            gt = item["ego_future_xyz"][0, :, :2]
            pred = {}
            for name, m in (("qad", qad), ("bf16", bf16), ("eng", eng), ("fast", eng)):
                b = collate_fn([item], model_config=m.config)
                keys = fingerprints(b["tokenized_data"]["pixel_values"], b["tokenized_data"]["image_grid_thw"])
                b = {k: ({kk: vv.cuda() for kk, vv in v.items()} if isinstance(v, dict) else v.cuda() if torch.is_tensor(v) else v) for k, v in b.items()}
                torch.manual_seed(i)                                                   # the same initial noise for all
                if name == "fast":                                                     # the served path (CUDA graphs)
                    x0 = torch.randn(1, *m.action_space.get_action_space_dims(), device="cuda")
                    act, _ = fast.run(b["tokenized_data"], b["ego_history_xyz"], b["ego_history_rot"], x0, keys)
                    xyz = m.action_space.action_to_traj(act, b["ego_history_xyz"][:, 0], b["ego_history_rot"][:, 0])[0][:, None]
                else:
                    xyz, _ = m.sample_trajectories(b["tokenized_data"], b["ego_history_xyz"], b["ego_history_rot"], num_samples=1)
                pred[name] = xyz[0, 0, :, :2].float().cpu()
            d = lambda a, b: float((pred[a] - pred[b]).norm(dim=-1).mean())
            r = dict(key=item["key"], fast_vs_qad=d("fast", "qad"), eng_vs_qad=d("eng", "qad"), bf16_vs_qad=d("bf16", "qad"),
                     eng_max=float((pred["eng"] - pred["qad"]).norm(dim=-1).max()),
                     **{f"{k}_ade": float((pred[k] - gt).norm(dim=-1).mean()) for k in pred})
            rows.append(r)
            print(json.dumps(r), flush=True)
    keys = [k for k in rows[0] if k != "key"]
    s = {k: round(float(np.mean([r[k] for r in rows])), 4) for k in keys} | {"n": len(rows), **eng.engine_info}
    print(json.dumps(s, indent=1))


if __name__ == "__main__":
    main()
