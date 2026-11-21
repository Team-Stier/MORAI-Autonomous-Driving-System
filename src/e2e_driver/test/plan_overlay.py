"""Planned paths over the lanes for a recorded drive (delivery_drive.sh output with the node's plan_a / plan_k debug):
each plan = its 64 (a, kappa) integrated from the Ego GT pose at the request time t0 (unicycle, 0.1 s steps, from the
reply's v0), drawn over the lanelet drivable area, lane lines and the route (lane centre) with the driven path. One
figure for the whole lap (a plan every 5 s) and one per curve section (roundabout, turns, sharp curves, lane penalties;
a plan every 0.5 s, 0-2 s bold). Mac: uv run --with numpy,pandas,matplotlib + Alpamayo-Cosmos tools/scorer's
requirements.

    python plan_overlay.py DRIVE_DIR NAME SCORER_DIR [OFFSET_S]      (NAME e.g. wowmayo2_map; DRIVE_DIR/eval_map)
"""
import gzip
import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd

D, NAME, SCORER = Path(sys.argv[1]), sys.argv[2], sys.argv[3]
OFF = float(sys.argv[4]) if len(sys.argv) > 4 else 0.04
sys.path.insert(0, SCORER)
from scorer import KMap, load_config  # noqa: E402

cfg = load_config(Path(SCORER) / "thresholds.yaml")
km = KMap.from_files(cfg)
ev = D / "eval_map"
lap = json.loads(open(ev / "laps.jsonl").read().splitlines()[-1])
rows = json.load(gzip.open(ev / "lap000.rows.json.gz", "rt"))
clock = np.array(json.load(open(ev / "lap000.clock.json")))
ego = np.array([(ns / 1e9, *v) for ns, v in rows["ego_status"]])
T0 = ego[0, 0]; te = ego[:, 0] - T0
clk = pd.read_csv(D / f"clk_{NAME}.csv")
w, ts = clk["field.data0"].to_numpy(float), clk["field.data1"].to_numpy(float)
o = np.argsort(ts); w, ts = w[o], ts[o]
to_eval = lambda tau: np.interp(np.interp(tau, ts, w) + OFF, clock[:, 0], clock[:, 1]) - T0
dbg = [json.loads(l.split(",", 1)[1]) for l in open(D / f"dbg_{NAME}.csv").read().splitlines()[1:]]
dbg = [r for r in dbg if r.get("plan_k")]
assert dbg, "no plan_a / plan_k in the debug: the node predates the plan logging"
td = to_eval(np.array([r["t0"] for r in dbg]))
pose = lambda t: [np.interp(t, te, ego[:, k]) for k in (1, 2)] + [np.interp(t, te, np.unwrap(ego[:, 3]))]


def plan_xy(r, t):
    x, y, h = pose(t)
    v, a, k = r["v0"], np.asarray(r["plan_a"]), np.asarray(r["plan_k"])
    pts = [(x, y)]
    for ai, ki in zip(a, k):
        v1 = max(v + ai * 0.1, 0.0); vm = 0.5 * (v + v1)
        h += ki * vm * 0.1; x += vm * np.cos(h) * 0.1; y += vm * np.sin(h) * 0.1; v = v1
        pts.append((x, y))
    return np.array(pts)


s_e = np.array([km.project((x, y)) for x, y in ego[:, 1:3]])
Hs = np.unwrap(km.heading); ks = np.gradient(Hs, km.s[:-1]); ks = np.convolve(ks, np.ones(41) / 41, mode="same")
# sections: lane penalties, the roundabout (sample spawn point 4), sharp curves (|k| > 0.03 for >= 10 m) -> t windows
secs = [("lane %.0f s" % e["t_start"], e["t_start"] - 4, min(e["t_end"], e["t_start"] + 10) + 2)
        for e in lap["scorer"]["events"] if e["type"] == "lane" and e["penalty_s"] > 0]
rb = np.hypot(ego[:, 1] + 111.3, ego[:, 2] - 358.3) < 45
if rb.any():
    secs.append(("roundabout", te[rb].min() - 2, te[rb].max() + 2))
sharp = np.abs(np.interp(s_e, km.s[:-1], ks)) > 0.03
edges = np.flatnonzero(np.diff(np.r_[0, sharp.astype(int), 0]))
for a, b in zip(edges[::2], edges[1::2]):
    if s_e[b - 1] - s_e[a] >= 10 and not any(lo <= te[a] <= hi for _, lo, hi in secs):
        secs.append(("curve s %.0f" % s_e[a], te[a] - 2, te[b - 1] + 2))

import matplotlib  # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.collections import LineCollection, PolyCollection  # noqa: E402


def base(ax, box):
    (x0, y0), (x1, y1) = box
    inb = lambda p: (p[:, 0].max() >= x0) & (p[:, 0].min() <= x1) & (p[:, 1].max() >= y0) & (p[:, 1].min() <= y1)
    ax.add_collection(PolyCollection([p for p in km.lanelets if inb(p)], facecolor="0.9", edgecolor="none"))
    m = (np.maximum(km.seg_a[:, 0], km.seg_b[:, 0]) >= x0) & (np.minimum(km.seg_a[:, 0], km.seg_b[:, 0]) <= x1) & \
        (np.maximum(km.seg_a[:, 1], km.seg_b[:, 1]) >= y0) & (np.minimum(km.seg_a[:, 1], km.seg_b[:, 1]) <= y1)
    ax.add_collection(LineCollection(np.stack([km.seg_a[m], km.seg_b[m]], 1), colors="0.55", linewidths=0.6))
    ax.plot(km.path[:, 0], km.path[:, 1], "k--", lw=0.8, label="route (lane centre)")
    ax.set_xlim(x0, x1); ax.set_ylim(y0, y1); ax.set_aspect("equal")


out = D / "plans"; out.mkdir(exist_ok=True)
fig, ax = plt.subplots(figsize=(12, 12))
lo, hi = ego[:, 1:3].min(0) - 30, ego[:, 1:3].max(0) + 30
base(ax, (lo, hi)); ax.plot(ego[:, 1], ego[:, 2], "r-", lw=1, label="driven")
last = -1e9
for r, t in zip(dbg, td):
    if t - last >= 5.0:
        p = plan_xy(r, t); ax.plot(p[:, 0], p[:, 1], "b-", lw=0.8, alpha=0.7); last = t
ax.legend(loc="upper right"); ax.set_title("%s: planned paths every 5 s (blue) over the lanes" % NAME)
fig.savefig(out / "00_full_map.png", dpi=80); plt.close(fig)
files = ["00_full_map.png"]
for i, (title, a, b) in enumerate(sorted(secs, key=lambda x: x[1]), 1):
    m = (te >= a) & (te <= b)
    if m.sum() < 5:
        continue
    sel, last = [], -1e9
    for r, t in zip(dbg, td):
        if a <= t <= b and t - last >= 0.5:
            sel.append((t, plan_xy(r, t))); last = t
    allp = np.vstack([ego[m, 1:3]] + [p[:21] for _, p in sel])
    c, half = allp.mean(0), max(np.ptp(allp, 0).max() / 2 + 8, 20)
    fig, ax = plt.subplots(figsize=(9, 9))
    base(ax, ((c[0] - half, c[1] - half), (c[0] + half, c[1] + half)))
    ax.plot(ego[m, 1], ego[m, 2], "r-", lw=2, label="driven")
    for t, p in sel:
        ax.plot(p[:21, 0], p[:21, 1], "b-", lw=1.6, alpha=0.8)          # 0-2 s
        ax.plot(p[20:, 0], p[20:, 1], "b-", lw=0.5, alpha=0.35)         # 2-6.4 s
    ax.plot([], [], "b-", lw=1.6, label="plan 0-2 s (every 0.5 s)")
    ax.legend(loc="upper right"); ax.set_title("%s  %s  t %.0f-%.0f s" % (NAME, title, a, b))
    f = "%02d_%s.png" % (i, title.replace(" ", "_")); fig.savefig(out / f, dpi=80); plt.close(fig); files.append(f)
print("figures in", out, files)
