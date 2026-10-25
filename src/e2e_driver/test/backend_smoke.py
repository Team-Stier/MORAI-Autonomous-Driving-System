"""Backend smoke test without ROS: dumped requests (offline_replay.py or model_server --dump) -> assemble -> backend.
Usage: MORAI_VLA_TRAIN=... python backend_smoke.py --backend hf --ckpt DIR [--n 20] DUMP_DIR"""
import argparse, json, sys, time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "server"))
import model_server                                   # noqa: E402  (sys.path: server, src, training code, pylib)
from assemble import build_sample                     # noqa: E402
from e2e_driver import wire                           # noqa: E402

import numpy as np                                    # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--backend", default="hf"); ap.add_argument("--ckpt"); ap.add_argument("--model-class", default="student")
    ap.add_argument("--vlm", default="Qwen/Qwen3-VL-4B-Instruct"); ap.add_argument("--attn", default="sdpa")
    ap.add_argument("--dtype", default="bfloat16"); ap.add_argument("--steps", type=int, default=10); ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--preprocess-config"); ap.add_argument("--fake-speed", type=float, default=4.0); ap.add_argument("--fake-kappa", type=float, default=0.0)
    ap.add_argument("--n", type=int, default=20); ap.add_argument("dump")
    a = ap.parse_args()
    t = time.perf_counter(); be = model_server.make_backend(a); load_s = time.perf_counter() - t
    files = sorted(Path(a.dump).glob("req_*.bin"))[:a.n]
    rows = []
    for f in files:
        out = be.predict(build_sample(wire.unpack(f.read_bytes())))
        assert np.isfinite(out["accel"]).all() and np.isfinite(out["kappa"]).all() and out["accel"].shape == (64,)
        rows.append(dict(out["timing"], a0=float(out["accel"][0]), k0=float(out["kappa"][0]), v0=out["v0"]))
    import torch
    med = lambda k: float(np.median([r[k] for r in rows[1:] or rows if k in r])) if any(k in r for r in rows) else None
    print(json.dumps(dict(backend=a.backend, ckpt=a.ckpt, load_s=round(load_s, 1), n=len(rows), preprocess_ms=med("preprocess_ms"),
                          model_ms=med("model_ms"), prefix_tokens=rows[0].get("prefix_tokens"),
                          gpu_peak_gb=round(torch.cuda.max_memory_allocated() / 2**30, 2) if torch.cuda.is_available() else None,
                          first=rows[0], accel_range=[min(r["a0"] for r in rows), max(r["a0"] for r in rows)])))


if __name__ == "__main__":
    main()
