"""Model-server throughput bench: dumped requests (offline_replay.py, consecutive rows) sent at RATE Hz with up to
INFLIGHT outstanding, as e2e_driver_node does; prints achieved step rate and per-stage timings (server timing fields).
Usage: python3 server_bench.py [--rate 10] [--inflight 2] [--seconds 20] [--clients N] DUMP_DIR
(--clients N against model_server --sims N: rate_hz is the sum over the nodes, decisions/s)"""
import argparse, json, socket, sys, threading, time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from e2e_driver import wire  # noqa: E402

import numpy as np  # noqa: E402


def client(a, reqs, offset):
    """one node: requests from `offset` on at RATE Hz with up to INFLIGHT outstanding -> [(round trip s, timing, t)]."""
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
        r = dict(reqs[(offset + n) % len(reqs)]); r["id"] = n
        with lock:
            pending[n] = time.perf_counter()
        wire.send(s, r); n += 1
    time.sleep(2.0)
    return done, n, skipped


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1"); ap.add_argument("--port", type=int, default=47011)
    ap.add_argument("--rate", type=float, default=10.0); ap.add_argument("--inflight", type=int, default=2)
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--clients", type=int, default=1, help="nodes at once (model_server --sims N), each on its own stretch of the dump")
    ap.add_argument("dump")
    a = ap.parse_args()
    reqs = [wire.unpack(f.read_bytes()) for f in sorted(Path(a.dump).glob("req_*.bin"))]
    res = [None] * a.clients

    def run(k):
        res[k] = client(a, reqs, k * len(reqs) // a.clients)
    ts = [threading.Thread(target=run, args=(k,)) for k in range(a.clients)]
    [t.start() for t in ts]; [t.join() for t in ts]
    warm = [d for done, _, _ in res for d in done[5:]]
    rate = lambda w: (len(w) - 1) / max(w[-1][2] - w[0][2], 1e-9) if len(w) > 1 else 0.0
    med = lambda k: round(float(np.median([d[1][k] for d in warm if k in d[1]])), 1) if any(k in d[1] for d in warm) else None
    print(json.dumps(dict(sent=sum(r[1] for r in res), replies=sum(len(r[0]) for r in res), skipped_ticks=sum(r[2] for r in res),
                          rate_hz=round(sum(rate(r[0][5:]) for r in res), 2), per_client_hz=[round(rate(r[0][5:]), 2) for r in res],
                          roundtrip_ms_median=round(1e3 * float(np.median([d[0] for d in warm])), 1),
                          roundtrip_ms_p95=round(1e3 * float(np.percentile([d[0] for d in warm], 95)), 1),
                          **{k: med(k) for k in ("assemble_ms", "preprocess_ms", "prep_ms", "wait_run_ms", "run_ms", "model_ms", "total_ms",
                                                 "prefill_ms", "expert_ms")})))


if __name__ == "__main__":
    main()
