"""Does control.py's standstill hold delay launches? On step-0 launches (the last standstill before the car passes
1 m/s), with the training target (a, kappa) as the plan (label_control_check.py's path, no inference):
  current rule  hold while |v| < stop_speed and the plan's speed over the next hold_horizon_s stays < stop_speed
  candidate     the same, but no hold when the plan's mean a over the next 0.5 s is > 0.05 m/s^2
Reports, per rule: share of standstill rows with recorded applied accel > 0.1 (the driver going) that are held, the launch
delay (first release - first applied accel > 0.1, s), and the share of true standstill rows (applied accel <= 0.02,
applied brake > 0) released (the car would creep). Mac CPU.

    python launch_hold_check.py EGO_DIR LABELS_DIR CONFIG.json ALPAMAYO_PKG_DIR [episodes]
"""
import glob
import json
import os
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ego_dir, labels_dir, cfg_path, pkg = sys.argv[1:5]
N_EP = int(sys.argv[5]) if len(sys.argv) > 5 else 400
sys.path[:0] = [pkg, str(HERE.parent / "server" / "vendor"), str(HERE.parent / "src")]
import pandas as pd  # noqa: E402
import torch  # noqa: E402
from alpamayo_r1.action_space import UnicycleAccelCurvatureActionSpace  # noqa: E402
from e2e_driver.control import DEFAULTS  # noqa: E402
from morai_vla.data import N_FUT, N_HIST, ego_trajectory  # noqa: E402

cfg = {k: v for k, v in json.load(open(cfg_path))["action_space_cfg"].items() if k != "_target_"}
space = UnicycleAccelCurvatureActionSpace(**cfg)
STOP, H = DEFAULTS["stop_speed"], int(round(DEFAULTS["hold_horizon_s"] / 0.1))


RULES = ("current rule", "no hold if mean a next 0.5 s > 0.05", "stop_speed 0.1", "hold_horizon 0.5 s")


def holds(v0, a):
    """hold under each of RULES for a plan of accelerations a (0.1 s steps) from speed v0."""
    ahead = v0 + np.r_[0.0, np.cumsum(a[:H] * 0.1)]
    cur = abs(v0) < STOP and ahead.max() < STOP
    return (cur, cur and not (a[:5].mean() > 0.05), abs(v0) < 0.1 and ahead.max() < 0.1, abs(v0) < STOP and ahead[:6].max() < STOP)


files = sorted(glob.glob(os.path.join(ego_dir, "*.npz")))
files = [files[i] for i in np.random.default_rng(0).choice(len(files), min(N_EP, len(files)), replace=False)]
go_rows, stay_rows, delays, gt_lead = ({k: [] for k in RULES} for _ in range(4))
events, crawl = 0, []
for f in files:
    ep = Path(f).stem
    lf = glob.glob(os.path.join(labels_dir, "*", ep + ".labels.parquet"))
    if not lf:
        continue
    lab = pd.read_parquet(lf[0], columns=["bag_time_ns", "ctrl_u", "ctrl_ok"])
    bt, cok = lab.bag_time_ns.to_numpy(np.int64), lab.ctrl_ok.to_numpy(bool)
    z = np.load(f); v, yaw, ns = z["v"].astype(np.float64), z["yaw"], z["bag_ns"]

    def applied_u(i):
        j = int(np.searchsorted(bt, ns[i], side="right") - 1)
        if j < 0 or not cok[j]:
            return None
        k = int(round((ns[i] - bt[j]) / 5e7)); u = lab.ctrl_u.iat[j]
        return float(u[k]) if 0 <= k < len(u) else None
    # launches: last standstill row before |v| passes 1 m/s; the standstill segment before it (<= 8 s)
    moving = np.abs(v) >= 1.0
    starts = np.nonzero(moving[1:] & ~moving[:-1])[0] + 1
    for s in starts:
        seg = np.arange(s - 1, max(s - 81, N_HIST), -1)
        seg = seg[np.abs(v[seg]) < STOP]
        if len(seg) < 3 or s + N_FUT + 1 >= len(v):
            continue
        rows = np.sort(seg)
        rows = rows[rows >= rows.max() - (len(seg) - 1)]                  # the contiguous standstill before the launch
        trajs = [ego_trajectory(v, yaw, int(i)) for i in rows]
        with torch.no_grad():
            act = space.traj_to_action(*(torch.cat([t[q] for t in trajs]).double() for q in range(4))).numpy()
        A = act[..., 0] * cfg["accel_std"] + cfg["accel_mean"]
        u = [applied_u(int(i) + 1) for i in rows]                           # the command at t0 is applied 0.1 s later
        if any(x is None for x in u):
            continue
        u = np.array(u); hc = np.array([holds(v[i], A[r]) for r, i in enumerate(rows)])
        go, stay = u > 0.1, (u <= 0.02) & (u < 0)                          # u < 0: brake applied, no accel
        if not go.any():
            continue
        events += 1
        t_go = np.argmax(go)
        crawl.append(float(np.abs(v[rows[t_go:]]).max()))                  # speed reached while the driver presses
        for c, key in enumerate(RULES):
            go_rows[key] += list(hc[go, c]); stay_rows[key] += list(~hc[stay, c])
            rel = np.nonzero(~hc[:, c])[0]
            first = rel[0] if len(rel) else len(rows)
            delays[key].append(0.1 * (first - t_go)); gt_lead.append(0) if False else gt_lead[key].append(0.1 * (len(rows) - first))
print("launch events %d (episodes %d)" % (events, len(files)))
d_cur = np.array(delays[RULES[0]]); cr = np.array(crawl)
print("long delays (> 0.3 s, current rule): %d; of them crawling (0.05 < max |v| < 0.3 before the launch row) %d" % (
    (d_cur > 0.3).sum(), ((d_cur > 0.3) & (cr > 0.05)).sum()))
for key in RULES:
    name, d = key, np.array(delays[key])
    print("  release before the launch row (s) p50 %.1f p10 %.1f" % (np.median(gt_lead[key]), np.percentile(gt_lead[key], 10)))
    print("%s: held while the driver accelerates %.1f %% of %d rows; launch delay s p50 %.2f p90 %.2f max %.2f (> 0.3 s: %.1f %%); "
          "true standstill rows released %.1f %% of %d" % (name, 100 * np.mean(go_rows[key]), len(go_rows[key]), np.median(d),
                                                            np.percentile(d, 90), d.max(), 100 * np.mean(d > 0.3),
                                                            100 * np.mean(stay_rows[key]), len(stay_rows[key])))
