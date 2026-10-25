"""Model server for e2e_driver_node: runs outside the Noetic chroot in the training venv (GPU), one client at a time.

  MORAI_VLA_TRAIN=~/morai-student/train ~/claude-bench/venv-teacher/bin/python model_server.py --backend hf --ckpt <student dir>
  ... --backend fake [--preprocess-config <any checkpoint/release dir with config.json>]

Each request (e2e_driver.wire frame) -> assemble.build_sample -> backend.predict -> {accel, kappa, v0, xyz, timing}.
--dump DIR keeps every request (wire frames) for offline checks.
"""
import argparse, logging, os, socket, sys, time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE), str(HERE.parent / "src"), os.path.expanduser(os.environ.get("MORAI_VLA_TRAIN", "~/morai-student/train"))]
PYLIB = os.path.expanduser(os.environ.get("MORAI_VLA_PYLIB", "~/morai-student/pylib"))   # nvidia-modelopt for QAD
if os.path.isdir(PYLIB): sys.path.append(PYLIB)                                          # checkpoints (modelopt_state.pth)
os.environ.setdefault("HF_HUB_OFFLINE", "1")

from assemble import build_sample                     # noqa: E402
from e2e_driver import wire                           # noqa: E402

log = logging.getLogger("model_server")


def make_backend(a):
    import backends
    if a.backend == "fake":
        return backends.FakeBackend(v_target=a.fake_speed, kappa=a.fake_kappa, preprocess_config=a.preprocess_config, vlm=a.vlm)
    return backends.HFBackend(a.ckpt, vlm=a.vlm, model_class=a.model_class, attn=a.attn, dtype=a.dtype, num_steps=a.steps, seed=a.seed)


def serve(a, backend):
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
        try:
            while True:
                req = wire.recv(conn)
                t = time.perf_counter()
                if dump: (dump / f"req_{n:06d}.bin").write_bytes(wire.pack(req))
                sample = build_sample(req, key=f"live/{n}")
                t1 = time.perf_counter()
                out = backend.predict(sample)
                out["timing"] = dict(out.get("timing", {}), assemble_ms=round(1e3 * (t1 - t), 1),
                                     total_ms=round(1e3 * (time.perf_counter() - t), 1))
                if out.get("xyz") is None: out.pop("xyz", None)
                wire.send(conn, out)
                n += 1
                if n % 100 == 1: log.info("request %d: %s", n, out["timing"])
        except (ConnectionError, OSError) as e:
            log.info("client gone: %s", e)
        finally:
            conn.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--backend", choices=["hf", "fake"], default="hf")
    ap.add_argument("--ckpt", help="hf: student (or teacher) checkpoint dir")
    ap.add_argument("--model-class", choices=["student", "teacher"], default="student")
    ap.add_argument("--vlm", default="Qwen/Qwen3-VL-4B-Instruct", help="VLM id the checkpoint was built on (8B for the teacher)")
    ap.add_argument("--attn", default="sdpa"); ap.add_argument("--dtype", default="bfloat16")
    ap.add_argument("--steps", type=int, default=10, help="Euler steps (MODEL_PLAN: 10)")
    ap.add_argument("--seed", type=int, default=None, help="fixed noise seed per step (default: fresh noise)")
    ap.add_argument("--preprocess-config", help="fake: build the real model batch with this config.json dir")
    ap.add_argument("--fake-speed", type=float, default=4.0); ap.add_argument("--fake-kappa", type=float, default=0.0)
    ap.add_argument("--host", default="127.0.0.1"); ap.add_argument("--port", type=int, default=47011)
    ap.add_argument("--dump")
    a = ap.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(message)s")
    if a.backend == "hf" and not a.ckpt: ap.error("--backend hf needs --ckpt")
    backend = make_backend(a)
    serve(a, backend)


if __name__ == "__main__":
    main()
