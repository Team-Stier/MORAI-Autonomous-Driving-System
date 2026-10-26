"""control.py mode map vs mode mlp (the package weights) on the same inputs: rows from control_mlp.py data, the
episode-held-out validation episodes; per speed bin the median / p95 of |map - mlp| and of each against the applied
values recorded 0.1 s later. numpy only (the node's code path).

    python control_map_vs_mlp.py ROWS.npz [control_mlp.npz | control_mlp_nohist.npz]
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))
from e2e_driver.control import Controller, Plan  # noqa: E402

BINS = [("stop", 0.0, 0.3), ("low", 0.3, 4.5), ("mid", 4.5, 7.5), ("cruise", 7.5, 99.0)]


def main():
    d = np.load(sys.argv[1]); weights = sys.argv[2] if len(sys.argv) > 2 else "control_mlp.npz"
    X, Y, ep, v = d["x"], d["y"], d["ep"], d["v"]
    eps = np.unique(ep); val = set(np.random.default_rng(0).choice(eps, len(eps) // 5, replace=False))   # control_mlp.py's split
    rows = np.nonzero([e in val for e in ep])[0]
    ctl = {"map": Controller(), "mlp": Controller(mode="mlp", mlp_weights=weights)}
    out = {k: [] for k in ctl}
    for i in rows:
        x = X[i]
        a, k = np.r_[x[:10], np.full(54, x[9])], np.r_[x[10:20], np.full(54, x[19])]
        for name, c in ctl.items():
            acc, brk, st, _ = c.command(Plan(0.0, a, k, x[20]), 0.0, x[20], x[21:27] if name == "mlp" else None)
            out[name].append((acc, brk, 40.0 * st))
    P = {k: np.array(o) for k, o in out.items()}
    q = lambda e: "%.3f/%.3f" % (np.median(np.abs(e)), np.percentile(np.abs(e), 95))
    print("%d validation rows, weights %s; columns: accel, brake, steer deg (p50/p95)" % (len(rows), weights))
    for name, lo, hi in BINS + [("all", 0.0, 99.0)]:
        m = (np.abs(v[rows]) >= lo) & (np.abs(v[rows]) < hi)
        diff = P["map"][m] - P["mlp"][m]
        print("%-6s n %5d | map-mlp %s %s %s | map-applied %s %s %s | mlp-applied %s %s %s" % (
            name, m.sum(), *(q(diff[:, c]) for c in range(3)), *(q(P["map"][m][:, c] - Y[rows][m][:, c]) for c in range(3)),
            *(q(P["mlp"][m][:, c] - Y[rows][m][:, c]) for c in range(3))))
    assert np.all((P["mlp"][:, :2] >= 0) & (P["mlp"][:, :2] <= 1)) and np.all(np.abs(P["mlp"][:, 2]) <= 40.0)


if __name__ == "__main__":
    main()
