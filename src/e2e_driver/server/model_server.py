"""Model server for e2e_driver_node (GPU, Python 3.12 venv from server/requirements.lock), one client at a time; run.sh
starts it (config/e2e_runtime.env), or by hand:

  <venv>/bin/python model_server.py --backend hf --ckpt <student dir>
  <venv>/bin/python model_server.py --backend engine --ckpt <QAD deliver dir>       (RTX 5080 NVFP4/FP8)
  ... --backend fake [--preprocess-config <any checkpoint/release dir with config.json>]

The training code it needs is the pinned copy in server/vendor (tools/vendor_sync.sh, test/test_vendor.py);
MORAI_VLA_TRAIN=<train dir> overrides it for development only.

Each request (e2e_driver.wire frame, with an "id") -> assemble.build_sample + backend.prepare (CPU thread) ->
backend.run (GPU, this thread) -> {id, accel, kappa, v0, xyz, timing}. The node keeps up to two requests in flight, so
the next request is prepared while the model runs the current one: the step rate is set by the slower stage, not by
their sum. --dump DIR keeps every request (wire frames) for offline checks.
"""
import argparse, logging, os, queue, socket, sys, threading, time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE), str(HERE.parent / "src"), os.path.expanduser(os.environ.get("MORAI_VLA_TRAIN", str(HERE / "vendor")))]
PYLIB = os.environ.get("MORAI_VLA_PYLIB")                     # dev only: nvidia-modelopt outside the venv (QAD checkpoints)
if PYLIB and os.path.isdir(os.path.expanduser(PYLIB)): sys.path.append(os.path.expanduser(PYLIB))
os.environ.setdefault("HF_HUB_OFFLINE", "1")

from assemble import build_sample                     # noqa: E402
from e2e_driver import wire                           # noqa: E402

log = logging.getLogger("model_server")


def gpu_mem():
    """this process's CUDA memory (GiB): allocated now, peak allocated, peak reserved by the caching allocator."""
    import torch
    if not torch.cuda.is_available():
        return {}
    g = lambda x: round(x / 2**30, 2)
    return dict(alloc_gb=g(torch.cuda.memory_allocated()), alloc_max_gb=g(torch.cuda.max_memory_allocated()),
                reserved_max_gb=g(torch.cuda.max_memory_reserved()))


def make_backend(a):
    import backends
    if a.backend == "fake":
        return backends.FakeBackend(v_target=a.fake_speed, kappa=a.fake_kappa, preprocess_config=a.preprocess_config, vlm=a.vlm)
    if a.backend == "engine":
        warm = sorted(Path(a.warmup).glob("req_*.bin"))[:2] if a.warmup else []
        b = backends.EngineBackend(a.ckpt, vlm=a.vlm, num_steps=a.steps, seed=a.seed, warmup=warm, want_xyz=bool(a.dump))
        log.info("engine %s loaded in %.0f s, warm-up %s, gpu %s", b.model.engine_info, b.load_s, getattr(b, "warmup_timing", None), gpu_mem())
        return b
    return backends.HFBackend(a.ckpt, vlm=a.vlm, model_class=a.model_class, attn=a.attn, dtype=a.dtype, num_steps=a.steps, seed=a.seed)


def serve(a, backend):
    proc = None
    if a.prep == "process":
        from prep_proc import PrepProcess
        assert a.backend == "engine", "--prep process is for the engine backend"
        proc = PrepProcess(a.ckpt, a.vlm, "sdpa", backend.fast.image_token, backend.fast.vlm.visual.spatial_merge_size)
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((a.host, a.port)); srv.listen(1)
    log.info("listening on %s:%d (backend %s)", a.host, a.port, a.backend)
    dump = Path(a.dump) if a.dump else None
    if dump: dump.mkdir(parents=True, exist_ok=True)
    n = 0
    while True:
        conn, peer = srv.accept()
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        log.info("client %s", peer)
        inbox, ready = queue.Queue(4), queue.Queue(1)

        def receive():
            try:
                while True:
                    inbox.put((wire.recv(conn), time.perf_counter()))
            except (ConnectionError, OSError) as e:
                log.info("client gone: %s", e)
            inbox.put(None)

        def prepare():
            while True:
                job = inbox.get()
                if job is None:
                    ready.put(None); return
                req, t_in = job
                t = time.perf_counter()
                if dump: (dump / f"req_{int(req.get('id', 0)):06d}.bin").write_bytes(wire.pack(req))
                if proc is not None:                   # --prep process: build + tokenize in the worker, pin here
                    prepared, timing, assemble_s = proc(req, f"live/{req.get('id', 0)}")
                    sample, t1, prepared = {}, t + assemble_s, backend.pin((prepared, timing))
                    ready.put((req, sample, prepared, t_in, t, t1, time.perf_counter()))
                    continue
                sample = build_sample(req, key=f"live/{req.get('id', 0)}")
                t1 = time.perf_counter()
                prepared = backend.prepare(sample)
                ready.put((req, sample, prepared, t_in, t, t1, time.perf_counter()))

        threading.Thread(target=receive, daemon=True).start()
        threading.Thread(target=prepare, daemon=True).start()
        try:
            while True:
                job = ready.get()
                if job is None:
                    break
                req, sample, prepared, t_in, t, t1, t2 = job
                t3 = time.perf_counter()
                if req.get("reset") and hasattr(backend, "reset"):   # a teleport: nothing cached belongs to the new place
                    backend.reset()
                if "route_kappa" in req:                  # load-test steering for the fake backend; after prepare, so
                    sample["route_kappa"] = req["route_kappa"]   # the tokenized batch never sees it
                out = backend.run(sample, prepared)
                t4 = time.perf_counter()
                ms = lambda x: round(1e3 * x, 1)
                out["timing"] = dict(out.get("timing", {}), wait_prep_ms=ms(t - t_in), assemble_ms=ms(t1 - t), prep_ms=ms(t2 - t),
                                     wait_run_ms=ms(t3 - t2), run_ms=ms(t4 - t3), total_ms=ms(t4 - t_in))
                out["id"] = req.get("id")
                if out.get("xyz") is None: out.pop("xyz", None)
                wire.send(conn, out)
                n += 1
                if n % 100 == 1: log.info("request %d: %s gpu %s", n, out["timing"], gpu_mem() if a.backend != "fake" else {})
        except (ConnectionError, OSError) as e:
            log.info("send failed: %s", e)
        finally:
            conn.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--backend", choices=["hf", "engine", "fake"], default="hf")
    ap.add_argument("--ckpt", help="hf: student (or teacher) checkpoint dir; engine: QAD deliver dir (final/ + quant_manifest.json)")
    ap.add_argument("--model-class", choices=["student", "teacher"], default="student")
    ap.add_argument("--vlm", default="Qwen/Qwen3-VL-4B-Instruct", help="VLM id the checkpoint was built on (8B for the teacher)")
    ap.add_argument("--attn", default="sdpa"); ap.add_argument("--dtype", default="bfloat16")
    ap.add_argument("--steps", type=int, default=10, help="Euler steps (MODEL_PLAN: 10)")
    ap.add_argument("--seed", type=int, default=None, help="fixed noise seed per step (default: fresh noise)")
    ap.add_argument("--preprocess-config", help="fake: build the real model batch with this config.json dir")
    ap.add_argument("--fake-speed", type=float, default=4.0); ap.add_argument("--fake-kappa", type=float, default=0.0)
    ap.add_argument("--host", default="127.0.0.1"); ap.add_argument("--port", type=int, default=47011)
    ap.add_argument("--dump")
    ap.add_argument("--prep", choices=["thread", "process"], default="thread",
                    help="engine: request preprocessing in the server's thread or in a worker process (no GIL contention "
                         "with the run thread; prep_proc.py)")
    ap.add_argument("--warmup", default=str(HERE.parent / "assets" / "warmup"),
                    help="engine: dumped requests (req_*.bin) to compile and capture the graphs on before listening")
    a = ap.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(message)s")
    if a.backend in ("hf", "engine") and not a.ckpt: ap.error("--backend %s needs --ckpt" % a.backend)
    try:
        backend = make_backend(a)
    except Exception as e:
        if type(e).__name__ != "DeliveryError":
            raise
        log.error("delivery %s is not usable: %s", a.ckpt, e)            # run.sh: model_server exited unexpectedly (exit 2)
        sys.exit(2)
    serve(a, backend)


if __name__ == "__main__":
    main()
