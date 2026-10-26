"""Engine request preprocessing on the CPU, in the server's prepare thread or in a worker process (model_server
--prep process): assemble.build_sample + tokenize + fast_engine.fingerprints / layout + the bf16 pixel copy. In the
thread, ~35-60 ms of this Python holds the GIL while the run thread launches the GPU graphs and post-processes: the
10/27 rehearsal's 100 ms overruns at <= 6 new images had post 5.9 ms instead of 1.5. The worker runs the same calls, so
the batch is bit-identical (test/prep_proc_check.py); tensors come back through shared memory (torch.multiprocessing)
and the parent only pins the pixels (CUDA stays out of the worker).
"""
import os
import time
from pathlib import Path

import torch

from assemble import build_sample, tokenize


def cpu_prepare(sample, config, image_token, merge):
    """-> ((batch, image keys, M-RoPE layout), timing): EngineBackend.prepare without the pinning."""
    from fast_engine import fingerprints, layout
    t = time.perf_counter()
    b = tokenize(sample, config)
    timing = dict(preprocess_ms=round(1e3 * (time.perf_counter() - t), 1), prefix_tokens=int(b["tokenized_data"]["input_ids"].shape[1]))
    t = time.perf_counter()
    td = b["tokenized_data"]
    keys = fingerprints(td["pixel_values"], td["image_grid_thw"])
    lay = layout(td["input_ids"], td["image_grid_thw"], image_token, merge)
    td["pixel_values"] = td["pixel_values"].to(torch.bfloat16)                # the ViT input dtype
    return (b, keys, lay), dict(timing, fingerprint_ms=round(1e3 * (time.perf_counter() - t), 1))


def _worker(qin, qout, ckpt, vlm, attn, image_token, merge, threads):
    torch.set_num_threads(threads)
    from morai_vla.model import load_config
    config = load_config(Path(ckpt) / "final", vlm, attn)                     # as engine.load builds it
    while True:
        job = qin.get()
        if job is None:
            return
        req, key = job
        t = time.perf_counter()
        sample = build_sample(req, key=key)
        assemble_s = time.perf_counter() - t
        prepared, timing = cpu_prepare(sample, config, image_token, merge)
        qout.put((prepared, timing, assemble_s))


class PrepProcess:
    """one worker process; call(req, key) -> (prepared without pinning, timing, build_sample seconds)."""

    def __init__(self, ckpt, vlm, attn, image_token, merge, threads=int(os.environ.get("PREP_THREADS", "4"))):
        import torch.multiprocessing as mp
        ctx = mp.get_context("spawn")
        self.qin, self.qout = ctx.Queue(2), ctx.Queue(2)
        self.p = ctx.Process(target=_worker, args=(self.qin, self.qout, str(ckpt), vlm, attn, image_token, merge, threads), daemon=True)
        self.p.start()

    def __call__(self, req, key):
        self.qin.put((req, key))
        return self.qout.get()

    def close(self):
        self.qin.put(None); self.p.join(10)
