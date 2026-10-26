"""Kernel time of the fast_engine graphs (LLM prefill, expert Euler steps, the camera ViT batch) on one val sample.

  winpc (cwd src/e2e_driver/server): HF_HUB_OFFLINE=1 ~/morai-eval/venv-engine/bin/python ../test/engine_profile.py \
      <deliver dir> <a run config.yaml for the data section> <step-0 data root> [top]
"""
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE.parent / "server"), str(HERE.parent / "server" / "vendor")]

import hydra.utils as hyu  # noqa: E402
import torch  # noqa: E402
from omegaconf import OmegaConf  # noqa: E402
from torch.profiler import ProfilerActivity, profile  # noqa: E402

import engine  # noqa: E402
from fast_engine import FastEngine, fingerprints  # noqa: E402
from morai_vla.processor import collate_fn  # noqa: E402


def main():
    deliver, cfg_path, root = sys.argv[1:4]
    top = int(sys.argv[4]) if len(sys.argv) > 4 else 12
    m = engine.load(deliver)
    fe = FastEngine(m)
    cfg = OmegaConf.load(cfg_path)
    cfg.data.root = root
    d = OmegaConf.to_container(cfg.data.val_dataset, resolve=True)
    d.update(augment=False, max_samples=1, vla_preprocess_args={"_target_": "morai_vla.processor.get_preprocess_fn", "generation_mode": True})
    ds = hyu.instantiate(d, _convert_="partial", model_config=m.config)
    b = collate_fn([ds[0]], model_config=m.config)
    keys = fingerprints(b["tokenized_data"]["pixel_values"], b["tokenized_data"]["image_grid_thw"])
    b = {k: ({kk: vv.cuda() for kk, vv in v.items()} if isinstance(v, dict) else v.cuda() if torch.is_tensor(v) else v) for k, v in b.items()}
    fe.run(b["tokenized_data"], b["ego_history_xyz"], b["ego_history_rot"], torch.randn(1, 64, 2, device="cuda"), keys)
    Lp = list(fe.exp)[0]
    graphs = [("llm", fe.llm[Lp][2]), ("expert", fe.exp[Lp][2])] + [(f"vit {k[0] and 'lidar' or 'camera'} x{len(k[1])}", v[-1])
                                                                     for k, v in fe.vit_graphs.items()]
    for name, g in graphs:
        for _ in range(3):
            g.replay()
        torch.cuda.synchronize()
        with profile(activities=[ProfilerActivity.CUDA]) as p:
            g.replay(); torch.cuda.synchronize()
        ev = sorted(((e.key, e.self_device_time_total, e.count) for e in p.key_averages()), key=lambda r: -r[1])
        print("==", name, "kernel total ms", round(sum(r[1] for r in ev) / 1e3, 2))
        for k, t, c in ev[:top]:
            print(f"{t / 1e3:7.2f} ms {c:5d}  {k[:110]}")


if __name__ == "__main__":
    main()
