"""FastEngine under a jittery camera stream: before each stored request, random ViT-cache evictions leave 0-18 images
new (a late or dropped camera frame changes the picks; the fake host showed a p50 of 10). Checks that no new ViT graph
appears after the warm-up (one slot pattern per encoder), that the action equals an every-image-new pass with the same
noise (ViT outputs and (a, kappa) max |diff|), and prints the step time by the number of new images.

  winpc (cwd src/e2e_driver/server): HF_HUB_OFFLINE=1 MORAI_VLA_PYLIB=~/morai-student/pylib \
      ~/morai-eval/venv-engine/bin/python ../test/vit_slots.py <deliver dir> <dir of model_server --dump req_*.bin> [n]
"""
import glob
import os
import random
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE.parent / "server"), str(HERE.parent / "src"), str(HERE.parent / "server" / "vendor")]
sys.path += [os.path.expanduser(p) for p in [os.environ.get("MORAI_VLA_PYLIB", "")] if p]

import numpy as np  # noqa: E402

from assemble import build_sample  # noqa: E402
from backends import EngineBackend  # noqa: E402
from e2e_driver import wire  # noqa: E402


def main():
    deliver, reqdir = sys.argv[1:3]
    n = int(sys.argv[3]) if len(sys.argv) > 3 else 40
    be = EngineBackend(deliver, seed=0, warmup=sorted(glob.glob(str(HERE.parent / "assets/warmup/req_*.bin")))[:2])
    print("warm-up", be.warmup_timing, flush=True)
    fe, rng, rows = be.fast, random.Random(0), []
    graphs = set(fe.vit_graphs)
    for i, f in enumerate(sorted(glob.glob(f"{reqdir}/req_*.bin"))[:n]):
        sample = build_sample(wire.unpack(open(f, "rb").read()), key=f"slots/{i}")
        for c in rng.sample(sorted(fe.vit_cache, key=str), rng.randint(0, len(fe.vit_cache))):
            del fe.vit_cache[c]
        out = be.run(sample, be.prepare(sample))
        be.n -= 1                                                     # same noise again
        cached = dict(fe.vit_cache)
        be.reset()
        ref = be.run(sample, be.prepare(sample))
        vit_err = max(max((e - fe.vit_cache[c][0]).abs().max().item(), *[(d - d2).abs().max().item() for d, d2 in zip(ds, fe.vit_cache[c][1])])
                      for c, (e, ds) in cached.items())
        err = max(np.abs(out["accel"] - ref["accel"]).max(), np.abs(out["kappa"] - ref["kappa"]).max() * 100)
        t = out["timing"]
        rows.append((t["new_images"], t["vit_ms"], t["model_ms"], err, vit_err))
        print(i, rows[-1], flush=True)
        assert set(fe.vit_graphs) == graphs, f"new ViT graph {set(fe.vit_graphs) - graphs}"
    print("ViT graphs", [(k[0], len(k[1])) for k in graphs], "requests", len(rows))
    for k in sorted({r[0] for r in rows}):
        g = [r for r in rows if r[0] == k]
        print("new %2d: n %2d  vit_ms %5.1f  model_ms %5.1f  max |d accel|, 100 |d kappa| %.2g  max |d ViT out| %.2g" % (
            k, len(g), np.median([r[1] for r in g]), np.median([r[2] for r in g]), max(r[3] for r in g), max(r[4] for r in g)))
    assert max(r[3] for r in rows) < 0.05, "cached path differs from the every-image-new pass"


if __name__ == "__main__":
    main()
