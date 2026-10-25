"""Model-server throughput bench: dumped requests (offline_replay.py, consecutive rows) sent at RATE Hz with up to
INFLIGHT outstanding, as e2e_driver_node does; prints achieved step rate and per-stage timings (server timing fields).
Usage: python3 server_bench.py [--rate 10] [--inflight 2] [--seconds 20] DUMP_DIR"""
import argparse, json, socket, sys, threading, time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from e2e_driver import wire  # noqa: E402

import numpy as np  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1"); ap.add_argument("--port", type=int, default=47011)
    ap.add_argument("--rate", type=float, default=10.0); ap.add_argument("--inflight", type=int, default=2)
    ap.add_argument("--seconds", type=float, default=20.0); ap.add_argument("dump")
    a = ap.parse_args()
    reqs = [wire.unpack(f.read_bytes()) for f in sorted(Path(a.dump).glob("req_*.bin"))]
    s = socket.create_connection((a.host, a.port)); s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    pending, done, lock = {}, [], threading.Lock()

    def receive():
        try:
            while True:
                r = wire.recv(s); t = time.perf_counter()
                with lock:
                    done.append((t - pending.pop(r["id"]), r["timing"], t))
        except Exception:
            pass
    threading.Thread(target=receive, daemon=True).start()
    t_start, n, skipped, nxt = time.perf_counter(), 0, 0, time.perf_counter()
    while time.perf_counter() - t_start < a.seconds:
        time.sleep(max(0.0, nxt - time.perf_counter())); nxt += 1.0 / a.rate
        with lock:
            busy = len(pending) >= a.inflight
        if busy:
            skipped += 1; continue
        r = dict(reqs[n % len(reqs)]); r["id"] = n
        with lock:
            pending[n] = time.perf_counter()
        wire.send(s, r); n += 1
    time.sleep(2.0)
    warm = done[5:]
    span = warm[-1][2] - warm[0][2] if len(warm) > 1 else 1e-9
    med = lambda k: round(float(np.median([d[1][k] for d in warm if k in d[1]])), 1) if any(k in d[1] for d in warm) else None
    print(json.dumps(dict(sent=n, replies=len(done), skipped_ticks=skipped, rate_hz=round((len(warm) - 1) / span, 2),
                          roundtrip_ms_median=round(1e3 * float(np.median([d[0] for d in warm])), 1),
                          roundtrip_ms_p95=round(1e3 * float(np.percentile([d[0] for d in warm], 95)), 1),
                          **{k: med(k) for k in ("assemble_ms", "preprocess_ms", "prep_ms", "wait_run_ms", "run_ms", "model_ms", "total_ms")})))


if __name__ == "__main__":
    main()
