"""--prep process gives the same batch as the thread path, bit for bit: stored requests in order (twice, so the JPEG /
image caches are hit as live), build_sample + cpu_prepare here vs prep_proc.PrepProcess; every tensor, the image keys
and the M-RoPE layout compared exactly. CPU only (no model weights: config.json + the base VLM's processor).

    python prep_proc_check.py DELIVER_DIR [REQ_DIR] [ALPAMAYO_PKG_DIR]      (DELIVER_DIR/final/config.json)
"""
import glob
import os
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE.parent / "server"), str(HERE.parent / "src"), str(HERE.parent / "server" / "vendor")] + sys.argv[3:4]
os.environ["PYTHONPATH"] = os.pathsep.join(sys.path[:4])                     # the spawned worker imports the same modules

import torch  # noqa: E402
from transformers import AutoConfig  # noqa: E402

from assemble import build_sample  # noqa: E402
from e2e_driver import wire  # noqa: E402
from prep_proc import PrepProcess, cpu_prepare  # noqa: E402


def same(a, b, path="batch"):
    if torch.is_tensor(a):
        assert torch.is_tensor(b) and a.dtype == b.dtype and a.shape == b.shape and torch.equal(a, b), path
    elif isinstance(a, dict):
        assert set(a) == set(b), (path, set(a) ^ set(b))
        for k in a:
            same(a[k], b[k], f"{path}.{k}")
    elif isinstance(a, (list, tuple)):
        assert len(a) == len(b), path
        for i, (x, y) in enumerate(zip(a, b)):
            same(x, y, f"{path}[{i}]")
    else:
        assert a == b, (path, a, b)


def main():
    deliver = sys.argv[1]
    reqs = sorted(glob.glob(os.path.join((sys.argv[2:3] or [""])[0] or str(HERE.parent / "assets" / "warmup"), "req_*.bin")))
    assert reqs, "no req_*.bin"
    from morai_vla.model import load_config
    vlm = "Qwen/Qwen3-VL-4B-Instruct"
    config = load_config(Path(deliver) / "final", vlm, "sdpa")
    vc = AutoConfig.from_pretrained(vlm)
    image_token, merge = vc.image_token_id, vc.vision_config.spatial_merge_size
    proc = PrepProcess(deliver, vlm, "sdpa", image_token, merge)
    n = 0
    for f in reqs * 2:
        req, key = wire.unpack(open(f, "rb").read()), f"check/{n}"
        here = cpu_prepare(build_sample(req, key=key), config, image_token, merge)
        there, timing, _ = proc(req, key)
        same(here[0], there, f"req {Path(f).name}")
        n += 1
    proc.close()
    print("prep process == thread on %d requests (%d files x 2)" % (n, len(reqs)))


if __name__ == "__main__":
    main()
