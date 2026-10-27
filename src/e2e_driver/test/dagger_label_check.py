"""dagger_label.py on a synthetic episode along the real route (no simulator): GT rows, shadow records and steps as the
pod writes them; a car at 10 m/s on the route lane, the plan = the lane ahead at 10 m/s, a pedestrian crossing ahead,
then the car 6 m off the lane. Checks the npz contract (step/ak/valid/source), source 0 vs 1, the yield firing, and
that the labels of the plain stretch read a ~ 0 and kappa ~ the lane's.
Mac: uv run --no-project --with torch,numpy,pandas,pillow,hydra-core,einops,scipy,colorlog,pyyaml python dagger_label_check.py GT_DIR ALPAMAYO_SRC
"""
import gzip
import json
import math
import sys
import tempfile
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "tools"))
import dagger_label as dl  # noqa: E402

gt_dir, alp = sys.argv[1], sys.argv[2]
lane = dl.Lane(HERE.parent / "assets" / "route.json")
tmp = Path(tempfile.mkdtemp()); ep = tmp / "ep00000"; ep.mkdir(); rec = tmp / "rec"; rec.mkdir()
W0, V0, S0 = 1.8e9, 10.0, 100.0                       # wall clock start, speed, route s start
xy = lambda s: np.array([np.interp(s, lane.S, lane.P[:, 0]), np.interp(s, lane.S, lane.P[:, 1])])
pose = lambda s: (*xy(s), math.atan2(*(xy(s + 1.0) - xy(s - 1.0))[::-1]))   # route.json has a 0.1 m kink at s 160
T = np.arange(0, 30.0, 0.05)                          # 30 s of simulator time, RTF 1 (wall = W0 + tau)
off = np.where(T > 20.0, 6.0, 0.0)                    # after 20 s: 6 m left of the lane (teleported, a "big deviation")
rows, lines = [], []
for t, o in zip(T, off):
    x, y, h = pose(S0 + V0 * t); x, y = x - o * math.sin(h), y + o * math.cos(h)
    rows.append([int(t * 1e9), [x, y, h, V0]])
    lines.append(["ego", W0 + t, x, y, h, V0])
    lines.append(["comp", W0 + t, V0, 0.0, 0.2, 0.0])
    if round(t / 0.05) % 2 == 0:                       # plan and objects at 10 Hz
        P = lane.ahead(S0 + V0 * t - 2.0, 80.0, 1.0)
        lines.append(["vp", W0 + t, W0 + t, [round(p, 2) for p in P[:, 0]], [round(p, 2) for p in P[:, 1]], [V0] * len(P), [x, y, h]])
        px, py, ph = pose(S0 + 120.0)                  # a pedestrian crossing the lane at s 220 from the right, 1.4 m/s
        tc = t - 10.0
        lines.append(["obj", W0 + t, [[7, "pedestrian", px + math.sin(ph) * (4 - 1.4 * tc), py - math.cos(ph) * (4 - 1.4 * tc),
                                       0.0, 1.4 * -math.sin(ph), 1.4 * math.cos(ph), 1, 0.6, 0.6]] if 0 <= tc <= 8 else []])
        lines.append(["sec", W0 + t, 2])
json.dump({"ego_status": rows}, gzip.open(ep / "rows.json.gz", "wt"))
(rec / "rec_1.jsonl").write_text("".join(json.dumps(l) + "\n" for l in lines))
with open(ep / "steps.jsonl", "w") as f:
    for t in np.arange(2.0, 23.0, 0.1):
        f.write(json.dumps(dict(k=0, t_wall=W0 + t, t_sim=t, s=S0 + V0 * t)) + "\n")
conv, gt = dl.Converter(dict(accel_mean=0.02902694707164455, accel_std=0.6810426736454882, curvature_mean=0.0002692167976330542,
                             curvature_std=0.026148280660833106, a_lambda=1e-4, a_ridge=1e-4, kappa_lambda=1e-4, kappa_ridge=1e-4,
                             theta_lambda=1e-6, theta_ridge=1e-8, v_lambda=1e-6, v_ridge=1e-4, dt=0.1, n_waypoints=64), alp), dl.load_gt(gt_dir)
summ = dl.label_episode(ep, rec, lane, gt, conv, actual=True, jobs=4)
print(json.dumps(summ))
z, d = np.load(ep / "dagger_labels.npz"), np.load(ep / "dagger_debug.npz")
N = len(z["step"])
assert z["step"].dtype == np.int32 and z["ak"].dtype == np.float32 and z["ak"].shape == (N, 64, 2)
assert z["valid"].dtype == bool and z["valid"].shape == (N, 64) and z["source"].dtype == np.int8 and N == 210
t = d["t_sim"]
early = (t < 6.0)                                      # before the pedestrian matters: plain lane following
print("source 1 at", t[z["source"] == 1].round(1), "reasons", d["reason"][z["source"] == 1]); assert (z["source"][t < 19.5] == 0).all() and (z["source"][t > 20.5] == 1).all()
assert z["valid"][t > 20.5].all()
hs = np.unwrap([pose(q)[2] for q in np.arange(90.0, 420.0, 0.5)]); ks = np.gradient(hs, 0.5)
kl = np.array([np.interp(S0 + V0 * tt + V0 * 0.1 * np.arange(1, 65), np.arange(90.0, 420.0, 0.5), ks) for tt in t[early]])
print("plain stretch: |a| max", np.abs(z["ak"][early, :, 0]).max().round(3), " kappa err rms", np.sqrt(np.mean((z["ak"][early, :, 1] - kl) ** 2)).round(4))
assert np.abs(z["ak"][early, :40, 0]).max() < 0.5
yl = d["flags"][:, 0]
print("yield at t0 from", t[yl].min() if yl.any() else None, "to", t[yl].max() if yl.any() else None)
assert yl.any() and 6.0 < t[yl].min() < 14.0
dev = np.nanmax(np.abs(np.diff(d["rollout"][t > 20.5][:, :, :2], axis=1)))
ra = d["rollout"][(t > 20.5) & (t < 21)][0]
print("deviation case: rollout end offset from the lane", round(lane.project(ra[-1, 0], ra[-1, 1], None)[1], 2), "m")
assert abs(lane.project(ra[-1, 0], ra[-1, 1], None)[1]) < 1.0
# the driven future of the plain stretch, through the same conversion, agrees with the label there
e = np.hypot(*(d["fxy"][early] - d["fxy_actual"][early]).transpose(2, 0, 1)).mean()
print("plain stretch ADE label vs driven", round(float(e), 3), "m")
assert e < 1.0
print("dagger_label_check ok")
