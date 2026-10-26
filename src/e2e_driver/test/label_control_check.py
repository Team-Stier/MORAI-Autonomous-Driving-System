"""Do the training targets drive the car the way it was driven? At step-0 rows t0 (tau grid), the target (a, kappa) is
built as training builds it (morai_vla.data.ego_trajectory + the release action space's traj_to_action, de-normalised);
control.py turns it into (accel, brake, steering) as the node does at t0 (step under t0 + 0.1 s, measured speed v(t0));
that command is applied 0.1 s later, so it is compared with the applied values recorded at the next tau row
(competition_status steering_angle, accel, brake: the label tables' ctrl_steer / ctrl_u = accel - brake, 20 Hz).
No inference. Mac: uv run --with torch,numpy,pandas,pyarrow,scipy,einops,colorlog,hydra-core,pillow,pyyaml.

    python label_control_check.py EGO_DIR LABELS_DIR CONFIG.json ALPAMAYO_PKG_DIR [per_bin]
"""
import glob
import json
import math
import os
import sys
from pathlib import Path

import numpy as np
import pandas as pd

HERE = Path(__file__).resolve().parent
ego_dir, labels_dir, cfg_path, pkg = sys.argv[1:5]
PER_BIN = int(sys.argv[5]) if len(sys.argv) > 5 else 6
sys.path[:0] = [pkg, str(HERE.parent / "server" / "vendor"), str(HERE.parent / "src")]
import torch  # noqa: E402
from alpamayo_r1.action_space import UnicycleAccelCurvatureActionSpace  # noqa: E402
from e2e_driver.control import DEFAULTS, Controller, Plan  # noqa: E402
from morai_vla.data import N_FUT, N_HIST, ego_trajectory  # noqa: E402

cfg = {k: v for k, v in json.load(open(cfg_path))["action_space_cfg"].items() if k != "_target_"}
space = UnicycleAccelCurvatureActionSpace(**cfg)
BINS = [("stop", 0.0, 0.3), ("low", 0.3, 4.5), ("mid", 4.5, 7.5), ("cruise", 7.5, 99.0)]


def labels_of(ep):
    f = glob.glob(os.path.join(labels_dir, "*", ep + ".labels.parquet"))
    return pd.read_parquet(f[0], columns=["bag_time_ns", "ctrl_steer", "ctrl_u", "ctrl_ok", "traj_ok"]) if f else None


def applied(lab, ns):
    """(steer right + rad, accel - brake) recorded at bag time ns (20 Hz arrays of the label row at or before it)."""
    bt = lab.bag_time_ns.to_numpy(np.int64)
    j = int(np.searchsorted(bt, ns, side="right") - 1)
    if j < 0 or not lab.ctrl_ok.iat[j]:
        return None
    k = int(round((ns - bt[j]) / 5e7))
    s, u = lab.ctrl_steer.iat[j], lab.ctrl_u.iat[j]
    return (float(s[k]), float(u[k])) if 0 <= k < len(s) else None


rng, rows = np.random.default_rng(0), []
for f in sorted(glob.glob(os.path.join(ego_dir, "*.npz"))):
    ep = Path(f).stem
    lab = labels_of(ep)
    if lab is None:
        continue
    z = np.load(f)
    v, yaw, ns = z["v"].astype(np.float64), z["yaw"], z["bag_ns"]
    ok = np.zeros(len(v), bool); ok[N_HIST:len(v) - N_FUT - 1] = True
    ok &= z["clean"] if "clean" in z.files else True
    for name, lo, hi in BINS:
        cand = np.nonzero(ok & (np.abs(v) >= lo) & (np.abs(v) < hi))[0]
        for i in rng.choice(cand, min(PER_BIN, len(cand)), replace=False) if len(cand) else []:
            app = applied(lab, int(ns[i + 1]))                        # the command at t0 is applied at t0 + 0.1 s
            if app is None:
                continue
            hx, hr, fx, fr = ego_trajectory(v, yaw, int(i))
            with torch.no_grad():
                act = space.traj_to_action(hx.double(), hr.double(), fx.double(), fr.double())[0]
            a = act[:, 0].numpy() * cfg["accel_std"] + cfg["accel_mean"]
            kap = act[:, 1].numpy() * cfg["curvature_std"] + cfg["curvature_mean"]
            rows.append(dict(ep=ep, bin=name, v=float(v[i]), a1=float(a[1]), k1=float(kap[1]), a_plan=a, k_plan=kap,
                             steer_app=-app[0], u_app=app[1], fog=bool(z["fog"][i]), hour=int(z["hour"][i])))
print("samples", len(rows), "episodes", len({r["ep"] for r in rows}), flush=True)


def evaluate(params):
    c = Controller(**params)
    out = []
    for r in rows:
        acc, brk, st, info = c.command(Plan(0.0, r["a_plan"], r["k_plan"], r["v"]), 0.0, r["v"])
        delta = st * math.radians(c.p["max_steer_deg"])
        out.append((math.degrees(delta) - math.degrees(r["steer_app"]), acc - brk, r["u_app"], info["hold"]))
    return np.array(out, dtype=object)


def stats(x):
    x = np.asarray(x, float)
    return "p50 %.3f p95 %.3f bias %+.3f (n %d)" % (np.median(np.abs(x)), np.percentile(np.abs(x), 95), x.mean(), len(x))


res = evaluate({})
bins = np.array([r["bin"] for r in rows]); steer_err = res[:, 0].astype(float)
u_pred, u_app, hold = res[:, 1].astype(float), res[:, 2].astype(float), res[:, 3].astype(bool)
moving = bins != "stop"
print("steering err deg (moving):", stats(steer_err[moving]))
for name, *_ in BINS:
    m = bins == name
    print("  %-6s steering %s | pedal (accel - brake) %s" % (name, stats(steer_err[m]), stats(u_pred[m] - u_app[m])))
dead = 0.02
sp, sa = np.sign(np.where(np.abs(u_pred) < dead, 0, u_pred)), np.sign(np.where(np.abs(u_app) < dead, 0, u_app))
print("accel/brake side agreement (|u| < %.2f = none): %.1f %%" % (dead, 100 * np.mean(sp == sa)))
for name, *_ in BINS:
    m = bins == name
    print("  %-6s %.1f %%  (pred brake %d / accel %d / none %d; applied brake %d / accel %d / none %d)" % (
        name, 100 * np.mean(sp[m] == sa[m]), *(int(np.sum(sp[m] == s)) for s in (-1, 1, 0)), *(int(np.sum(sa[m] == s)) for s in (-1, 1, 0))))
st = bins == "stop"
print("hold: predicted at %d of %d stop rows; applied brake >= %.2f there: %.1f %%, applied u p50 %.3f" % (
    hold[st].sum(), st.sum(), DEFAULTS["hold_brake"], 100 * np.mean(-u_app[st & hold] >= DEFAULTS["hold_brake"]), np.median(u_app[st & hold]) if (st & hold).any() else float("nan")))
# refit candidates (reported, not applied): steering gain g; a scale on the accel / brake table accelerations
k1, s_app = np.array([r["k1"] for r in rows]), np.degrees([r["steer_app"] for r in rows])
m = moving & (np.abs(k1) > 0.002)
g_best = min(np.arange(0.6, 1.0, 0.005), key=lambda g: np.median(np.abs(np.degrees(np.arctan(3.0 * k1[m] / g)) - s_app[m])))
r2 = evaluate(dict(steer_gain=float(g_best)))
print("candidate steer_gain %.3f: steering err (moving) %s" % (g_best, stats(r2[moving, 0].astype(float))))
best = None
for sa_ in np.arange(0.6, 1.61, 0.05):
    for sb_ in np.arange(0.6, 1.61, 0.05):
        p = dict(accel_map_low=[[q, y * sa_ if q > 0.06 else y] for q, y in DEFAULTS["accel_map_low"]],
                 accel_map_cruise=[[q, y * sa_ if q > 0.06 else y] for q, y in DEFAULTS["accel_map_cruise"]],
                 brake_map=[[q, y * sb_] for q, y in DEFAULTS["brake_map"]])
        e = evaluate(p)
        err = np.median(np.abs(e[moving, 1].astype(float) - e[moving, 2].astype(float)))
        if best is None or err < best[0]:
            best = (err, sa_, sb_, e)
e = best[3]
print("candidate table scale accel x%.2f brake x%.2f: pedal err (moving) %s" % (best[1], best[2], stats(e[moving, 1].astype(float) - e[moving, 2].astype(float))))
print("cells: fog %d / clear %d, hours %s" % (sum(r["fog"] for r in rows), sum(not r["fog"] for r in rows), sorted({r["hour"] for r in rows})))
