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
import argparse, contextlib, logging, os, queue, socket, sys, threading, time
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
        b = backends.EngineBackend(a.ckpt, vlm=a.vlm, num_steps=a.steps, seed=a.seed, warmup=warm, want_xyz=bool(a.dump) and not a.sims,
                                   sims=a.sims, expert2=a.expert2, lora_watch=a.lora_watch)
        log.info("engine %s loaded in %.0f s, warm-up %s, gpu %s", b.model.engine_info, b.load_s, getattr(b, "warmup_timing", None), gpu_mem())
        return b
    return backends.HFBackend(a.ckpt, vlm=a.vlm, model_class=a.model_class, attn=a.attn, dtype=a.dtype, num_steps=a.steps, seed=a.seed)


class Poked(queue.Queue):
    """a Queue(1) that sets a shared event on put (serve_batch waits for any simulator's request); t = the last put."""

    def __init__(self, poke):
        super().__init__(1); self.poke, self.t = poke, -1e9

    def put(self, item, *a, **k):
        super().put(item, *a, **k); self.t = time.monotonic(); self.poke.set()


def client_threads(conn, backend, proc, dump, ready, lock=None):
    """receive + prepare threads of one client: prepared jobs (req, sample, prepared, t_in, t, t1, t2) into `ready` (one
    request prepared ahead of the GPU thread), then None when the client is gone. dump: req -> the directory to keep it
    in (None: no dump). lock: serializes the in-thread preparation of several clients (tokenize's per-image cache queue
    is shared)."""
    inbox = queue.Queue(4)

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
            if dump: (dump(req) / f"req_{int(req.get('id', 0)):06d}.bin").write_bytes(wire.pack(req))
            if proc is not None:                   # --prep process: build + tokenize in the worker, pin here
                prepared, timing, assemble_s = proc(req, f"live/{req.get('id', 0)}")
                sample, t1, prepared = {}, t + assemble_s, backend.pin((prepared, timing))
                ready.put((req, sample, prepared, t_in, t, t1, time.perf_counter()))
                continue
            with lock or contextlib.nullcontext():
                sample = build_sample(req, key=f"live/{req.get('id', 0)}")
                t1 = time.perf_counter()
                prepared = backend.prepare(sample)
            ready.put((req, sample, prepared, t_in, t, t1, time.perf_counter()))

    threading.Thread(target=receive, daemon=True).start()
    threading.Thread(target=prepare, daemon=True).start()


def to_sample(req, sample):
    """request fields the backend reads besides the model inputs; after prepare, so the tokenized batch never sees them
    (route_kappa: load-test steering for the fake backend; expert: the expert set, RL_PLAN 5 A)."""
    for k in ("route_kappa", "expert"):
        if k in req:
            sample[k] = req[k]


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
        ready = queue.Queue(1)
        client_threads(conn, backend, proc, (lambda req: dump) if dump else None, ready)
        try:
            while True:
                job = ready.get()
                if job is None:
                    break
                req, sample, prepared, t_in, t, t1, t2 = job
                t3 = time.perf_counter()
                if req.get("reset") and hasattr(backend, "reset"):   # a teleport: nothing cached belongs to the new place
                    backend.reset()
                to_sample(req, sample)
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


def serve_batch(a, backend):
    """--sims N, the RL actor (RL_PLAN 3.1): N nodes at once, one connection each; a connection's slot is its
    simulator (its own ViT cache, dropped when the client goes). The GPU thread takes one ready request per simulator
    and runs them as one batch (backend.run_batch) without waiting for the others: while the GPU is the bottleneck
    every simulator has one ready. Exploration std: --rl-sigma, or a request's own "sigma" (0 for an evaluation lap).
    --lora-watch: a new LoRA version (latest.json) is merged in between batches. --dump DIR keeps DIR/worldW/req_ID.bin
    (the request) and out_ID.bin (the response: actions, chain, logp, expert, lora_version) per decision, W = the
    request's "world" (the node's ~world, the episode manager's), or DIR/simK/ without one. --gather S (synchronous
    evaluation, the worlds tick in step): after the first ready request, wait up to S s until every simulator that sent
    one in the last 2 s has one, so they run as one batch (a world loading a scene, or an idle stack, is not waited for)."""
    sims = a.sims
    poke, lock = threading.Event(), threading.Lock()
    readies, slots = [Poked(poke) for _ in range(sims)], [None] * sims
    procs = [None] * sims
    if a.prep == "process":
        from prep_proc import PrepProcess
        assert a.backend == "engine", "--prep process is for the engine backend"
        procs = [PrepProcess(a.ckpt, a.vlm, "sdpa", backend.fast.image_token, backend.fast.vlm.visual.spatial_merge_size) for _ in range(sims)]
    from backends import sigmas
    sigma = [float(x) for x in a.rl_sigma.split(",")]
    sigma = sigma[0] if len(sigma) == 1 else sigma                          # one std, or one per Euler step
    sigmas(sigma, a.steps)                                                  # all > 0 or all 0, else ValueError
    dump = Path(a.dump) if a.dump else None

    def dump_dir(req, k):
        d = dump / (f"world{int(req['world'])}" if "world" in req else f"sim{k}")
        d.mkdir(parents=True, exist_ok=True)
        return d
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((a.host, a.port)); srv.listen(sims)
    prep_lock = threading.Lock() if a.prep == "thread" else None

    def accept():
        while True:
            conn, peer = srv.accept()
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            with lock:
                k = next((i for i, c in enumerate(slots) if c is None), None)
                if k is not None:
                    slots[k] = conn
            if k is None:
                log.warning("client %s refused: %d simulators connected", peer, sims); conn.close(); continue
            log.info("client %s = simulator %d", peer, k)
            client_threads(conn, backend, procs[k], (lambda req, k=k: dump_dir(req, k)) if dump else None, readies[k], prep_lock)

    threading.Thread(target=accept, daemon=True).start()
    log.info("listening on %s:%d (backend %s, %d simulators, sigma %s, LoRA %s)", a.host, a.port, a.backend, sims, sigma, a.lora_watch)
    n, t_poll, ms = 0, 0.0, lambda x: round(1e3 * x, 1)
    while True:
        poke.wait(); poke.clear()
        end = time.monotonic() + a.gather
        while a.gather and time.monotonic() < end and any(c is not None and rq.empty() and end - rq.t < 2.0 + a.gather
                                                          for c, rq in zip(slots, readies)):
            poke.wait(max(end - time.monotonic(), 0.0)); poke.clear()
        jobs = []
        for k, rq in enumerate(readies):
            try:
                job = rq.get_nowait()
            except queue.Empty:
                continue
            if job is None:                                   # client gone: its slot and ViT cache are free
                if hasattr(backend, "reset"): backend.reset(k)
                with lock:
                    slots[k].close(); slots[k] = None
                continue
            jobs.append((k, job))
        if not jobs:
            continue
        if a.lora_watch and time.monotonic() - t_poll > 1.0:
            t_poll = time.monotonic()
            try:
                took = backend.poll_lora()
                if took is not None: log.info("LoRA %s (version %s) merged in %.2f s", a.lora_watch, backend.lora_version, took)
            except Exception as e:                            # a bad file: keep driving on the weights we have
                log.error("LoRA %s not loaded: %s", a.lora_watch, e)
        t3 = time.perf_counter()
        batch = []
        for k, (req, sample, prepared, *_) in jobs:
            if req.get("reset") and hasattr(backend, "reset"):
                backend.reset(k)
            to_sample(req, sample)
            try:
                sig = sigmas(req["sigma"], a.steps) if "sigma" in req else sigma
            except ValueError as e:                           # the episode manager's request: keep the server's own
                log.error("simulator %d request %s: %s", k, req.get("id"), e); sig = sigma
            batch.append((k, sample, prepared, sig))
        outs = backend.run_batch(batch) if hasattr(backend, "run_batch") else [backend.run(s, p) for _, s, p, _ in batch]
        t4 = time.perf_counter()
        for (k, (req, sample, prepared, t_in, t, t1, t2)), out in zip(jobs, outs):
            out["timing"] = dict(out.get("timing", {}), wait_prep_ms=ms(t - t_in), assemble_ms=ms(t1 - t), prep_ms=ms(t2 - t),
                                 wait_run_ms=ms(t3 - t2), run_ms=ms(t4 - t3), total_ms=ms(t4 - t_in))
            out.update(id=req.get("id"), sim=k, world=req.get("world"), t_wall=req.get("t_wall"))   # rl_reward joins on t_wall
            if out.get("xyz") is None: out.pop("xyz", None)
            if dump: (dump_dir(req, k) / f"out_{int(req.get('id', 0)):06d}.bin").write_bytes(wire.pack(out))
            try:
                wire.send(slots[k], out)
            except (ConnectionError, OSError) as e:           # the receive thread reports the client gone
                log.info("simulator %d: send failed: %s", k, e)
            n += 1
            if n % 100 == 1: log.info("request %d (simulator %d, batch %d): %s gpu %s", n, k, len(jobs), out["timing"], gpu_mem() if a.backend != "fake" else {})


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
    ap.add_argument("--sims", type=int, default=0,
                    help="RL actor: serve N simulators at once, batched (serve_batch); 0 = the deployment server (one client)")
    ap.add_argument("--gather", type=float, default=0.0, help="--sims: wait up to this many s for every connected "
                    "simulator's request before a batch (synchronous evaluation)")
    ap.add_argument("--rl-sigma", default="0", help="--sims: exploration std per Euler step (one value or one per step; "
                    "0 = deterministic); a request's own 'sigma' overrides it")
    ap.add_argument("--expert2", nargs="?", const="", default=None, metavar="LORA",
                    help="engine: a second expert set picked by the request's 'expert' (RL_PLAN 5 A): the delivery's expert "
                         "+ this LoRA file (train/rl LoRA.export); no file = a copy of the base")
    ap.add_argument("--lora-watch", metavar="LATEST_JSON", help="engine: the learner's <out>/lora/latest.json; each new "
                    "version is merged into the last expert set (--sims servers check between batches, once a second)")
    ap.add_argument("--warmup", default=str(HERE.parent / "assets" / "warmup"),
                    help="engine: dumped requests (req_*.bin) to compile and capture the graphs on before listening")
    a = ap.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(message)s")
    if a.backend in ("hf", "engine") and not a.ckpt: ap.error("--backend %s needs --ckpt" % a.backend)
    if (a.expert2 is not None or a.lora_watch) and a.backend != "engine": ap.error("--expert2 / --lora-watch need --backend engine")
    try:
        backend = make_backend(a)
    except Exception as e:
        if type(e).__name__ != "DeliveryError":
            raise
        log.error("delivery %s is not usable: %s", a.ckpt, e)            # run.sh: model_server exited unexpectedly (exit 2)
        sys.exit(2)
    serve_batch(a, backend) if a.sims else serve(a, backend)


if __name__ == "__main__":
    main()
