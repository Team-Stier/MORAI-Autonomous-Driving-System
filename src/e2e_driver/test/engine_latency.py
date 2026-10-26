"""Engine step time on a real sample (val step-0 data, the delivery's own config for tokenization): one
sample_trajectories call = ViT (cameras + LiDAR BEVs) + LLM prefill + 10 Euler steps of the expert, split by section.

  winpc (cwd src/e2e_driver/server): HF_HUB_OFFLINE=1 ~/claude-bench/venv-teacher/bin/python ../test/engine_latency.py \
      <deliver dir> <a run config.yaml for the data section> <step-0 data root> [iters] [--bf16]
"""
import json
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE.parent / "server"), str(HERE.parent / "server" / "vendor")]

import hydra.utils as hyu  # noqa: E402
import numpy as np  # noqa: E402
import torch  # noqa: E402
from omegaconf import OmegaConf  # noqa: E402

import engine  # noqa: E402
from morai_vla.processor import collate_fn  # noqa: E402


def timed(obj, name, log):
    f = getattr(obj, name)

    def wrap(*a, **k):
        torch.cuda.synchronize(); t = time.perf_counter()
        out = f(*a, **k)
        torch.cuda.synchronize(); log.setdefault(name, []).append(1e3 * (time.perf_counter() - t))
        return out
    setattr(obj, name, wrap)


def main():
    deliver, cfg_path, root = sys.argv[1:4]
    iters = int(sys.argv[4]) if len(sys.argv) > 4 and sys.argv[4].isdigit() else 20
    t = time.perf_counter()
    m = engine.load(deliver, quantize="--bf16" not in sys.argv)
    load_s = time.perf_counter() - t
    cfg = OmegaConf.load(cfg_path)
    cfg.data.root = root
    dcfg = OmegaConf.to_container(cfg.data.val_dataset, resolve=True)
    dcfg.update(augment=False, max_samples=4, vla_preprocess_args={"_target_": "morai_vla.processor.get_preprocess_fn", "generation_mode": True})
    ds = hyu.instantiate(dcfg, _convert_="partial", model_config=m.config)
    b = collate_fn([ds[0]], model_config=m.config)
    b = {k: ({kk: vv.cuda() for kk, vv in v.items()} if isinstance(v, dict) else v.cuda() if torch.is_tensor(v) else v) for k, v in b.items()}
    log = {}
    timed(m.vlm.model, "get_image_features", log)
    timed(m.vlm, "forward", log)
    timed(m.diffusion, "sample", log)
    total = []
    with torch.inference_mode():
        for i in range(iters + 3):
            td = dict(b["tokenized_data"])
            torch.cuda.synchronize(); t = time.perf_counter()
            m.sample_trajectories(td, b["ego_history_xyz"], b["ego_history_rot"], num_samples=1, num_steps=10)
            torch.cuda.synchronize(); total.append(1e3 * (time.perf_counter() - t))
    med = lambda xs: round(float(np.median(xs[3:])), 1)
    vit, vlm, exp = med(log["get_image_features"]), med(log["forward"]), med(log["sample"])
    tok = b["tokenized_data"]
    print(json.dumps(dict(engine=m.engine_info, load_s=round(load_s, 1), prefix_tokens=int(tok["input_ids"].shape[1]),
                          images=int(tok["image_grid_thw"].shape[0]), step_ms=med(total), vit_ms=vit, llm_ms=round(vlm - vit, 1),
                          expert_10_steps_ms=exp, peak_gb=round(torch.cuda.max_memory_allocated() / 2**30, 2),
                          gpu=torch.cuda.get_device_name(), torch=torch.__version__), indent=1))


if __name__ == "__main__":
    main()
