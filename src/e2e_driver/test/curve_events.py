"""Lane events of a recorded drive (delivery_drive.sh output): trajectory (model) or tracking (control) problem?
Mac only. Per event window (scorer lane events, -3 s .. +1 s) on the evaluation clock:
  lateral offset from the route (lane centre), road curvature, the model's planned curvature (kappa0, and the commanded
  step under t + 0.1 s), the actual curvature (yaw rate / v of the Ego GT), commanded vs applied steering, and the
  engine's delays (model_ms > 100, skipped ticks). The node logged only the plan's first steps, so the plan's path is
  approximated by holding the commanded kappa for 1 s (offset predicted from the current offset and heading error).
Judgement: plan kappa already off the road's (|kappa_cmd - kappa_road| large and the held plan leaves the lane) ->
trajectory; plan near the road but the actual kappa lags or falls short -> tracking. Drives whose node logged the whole
plan (plan_a / plan_k, 34df5b9): the plans of the 2 s before the event are integrated from the Ego GT pose at their t0
(as plan_overlay.py) and their lateral offset 1 s ahead replaces the held-kappa estimate. Tracking constants: actual kappa vs
commanded kappa (best lag) per speed bin -> steer_gain candidates. Writes OUT_DIR/curve_events.png.

    python curve_events.py DRIVE_DIR ROUTE_TXT [OFFSET_S]
"""
import gzip
import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd

D, route_txt = Path(sys.argv[1]), sys.argv[2]
OFF = float(sys.argv[3]) if len(sys.argv) > 3 else 0.04
mode = "map"
lap = json.loads(open(D / f"eval_{mode}" / "laps.jsonl").read().splitlines()[-1])
rows = json.load(gzip.open(D / f"eval_{mode}" / "lap000.rows.json.gz", "rt"))
clock = np.array(json.load(open(D / f"eval_{mode}" / "lap000.clock.json")))          # Mac wall, tau_eval
ego = np.array([(ns / 1e9, *v) for ns, v in rows["ego_status"]])                       # tau, x, y, yaw, v
T0 = ego[0, 0]
t_e = ego[:, 0] - T0

# route: arc length, heading, curvature (smoothed over +-5 m)
P = np.array([list(map(float, l.split()[:2])) for l in open(route_txt) if l.strip()])
seg = np.hypot(*np.diff(P, axis=0).T); S = np.r_[0, np.cumsum(seg)]
H = np.unwrap(np.arctan2(*np.diff(P, axis=0).T[::-1])); H = np.r_[H, H[-1]]
Sg = np.arange(0, S[-1], 0.5); Hg = np.interp(Sg, S, H)
Kg = np.gradient(Hg, Sg); Kg = np.convolve(Kg, np.ones(21) / 21, mode="same")         # +-5 m


def project(x, y, i0=None):
    w = slice(max(i0 - 80, 0), min(i0 + 80, len(P))) if i0 is not None else slice(None)
    j = np.argmin(np.hypot(P[w, 0] - x, P[w, 1] - y)) + (w.start or 0)
    j = min(j, len(P) - 2)
    h = np.arctan2(P[j + 1, 1] - P[j, 1], P[j + 1, 0] - P[j, 0])
    dx, dy = x - P[j, 0], y - P[j, 1]
    return S[j] + dx * np.cos(h) + dy * np.sin(h), -dx * np.sin(h) + dy * np.cos(h), h, j


lat, s_e, herr, j = [], [], [], None
for _, x, y, yaw, v in ego:
    s, e, h, j = project(x, y, j)
    s_e.append(s); lat.append(e); herr.append(np.angle(np.exp(1j * (yaw - h))))
s_e, lat, herr = map(np.array, (s_e, lat, herr))
k_road = np.interp(s_e, Sg, Kg)
yaw_u = np.unwrap(ego[:, 3]); v_e = ego[:, 4]
dt = np.gradient(t_e)
yr = np.convolve(np.gradient(yaw_u) / np.maximum(dt, 1e-3), np.ones(5) / 5, mode="same")   # 20 Hz, 0.25 s smoothing
k_act = np.where(np.abs(v_e) > 1.0, yr / np.maximum(np.abs(v_e), 1.0), np.nan)

# node debug (stack tau) -> winpc wall (sim clock state) -> Mac wall (+OFF) -> tau_eval
clk = pd.read_csv(D / f"clk_{D.name}_{mode}.csv")
wall_w, tau_st = clk["field.data0"].to_numpy(float), clk["field.data1"].to_numpy(float)
o = np.argsort(tau_st); wall_w, tau_st = wall_w[o], tau_st[o]
to_eval = lambda tau_stack: np.interp(np.interp(tau_stack, tau_st, wall_w) + OFF, clock[:, 0], clock[:, 1]) - T0
dbg = [json.loads(l.split(",", 1)[1]) for l in open(D / f"dbg_{D.name}_{mode}.csv").read().splitlines()[1:]]
t_d = to_eval(np.array([r["t0"] for r in dbg]))
k0 = np.array([r.get("kappa0", np.nan) for r in dbg]); kc = np.array([(r.get("cmd") or {}).get("kappa", np.nan) for r in dbg])
mms = np.array([r["server"]["model_ms"] for r in dbg]); skip = np.r_[0, np.diff([r["skipped"] for r in dbg])]
tel = pd.read_csv(D / f"tel_{D.name}_{mode}.csv")
tel["t"] = np.interp(tel.wall.to_numpy(float) + OFF, clock[:, 0], clock[:, 1]) - T0
ts, tc = tel[tel.kind == "s"].sort_values("t"), tel[tel.kind == "c"].sort_values("t")

# tracking: actual kappa vs commanded kappa delayed by lag, moving rows, per speed bin
kc_at = lambda t: np.interp(t, t_d, kc)
best = {}
for name, lo, hi in (("all", 1.0, 99), ("<7 m/s", 1.0, 7.0), ("7-12 m/s", 7.0, 12.0), (">12 m/s", 12.0, 99)):
    m = (np.abs(v_e) >= lo) & (np.abs(v_e) < hi) & np.isfinite(k_act) & (np.abs(k_road) > 0.005)
    if m.sum() < 50:
        continue
    fits = []
    for lag in np.arange(0.0, 0.81, 0.05):
        x = kc_at(t_e[m] - lag); y = k_act[m]
        r = float(np.dot(x, y) / np.dot(x, x)); fits.append((float(np.mean((y - r * x) ** 2)), lag, r))
    mse, lag, r = min(fits); best[name] = (lag, r, m.sum())
print("tracking actual kappa = r x commanded kappa(t - lag), curves only (|road kappa| > 0.005):")
for k, (lag, r, n) in best.items():
    print("  %-9s n %5d  lag %.2f s  r %.3f  -> steer_gain candidate %.3f (now 0.77)" % (k, n, lag, r, 0.77 * r))

events = [e for e in lap["scorer"]["events"] if e["type"] == "lane" and e["penalty_s"] > 0]
print("\nevent        road k   plan k0  cmd k   act k  | lat start->max m | held-plan lat 1 s | steer cmd/applied deg | >100ms skipped | verdict")
summ = []
for e in events:
    a, b = e["t_start"] - 3.0, min(e["t_end"], e["t_start"] + 6.0)
    m = (t_e >= a) & (t_e <= b); md = (t_d >= a) & (t_d <= b)
    pre = (t_e >= e["t_start"] - 2.0) & (t_e <= e["t_start"])
    kr, ka = np.nanmean(k_road[pre]), np.nanmean(k_act[pre])
    pre_d = (t_d >= e["t_start"] - 2.0) & (t_d <= e["t_start"])
    kp0, kcm = np.nanmean(k0[pre_d]), np.nanmean(kc[pre_d])
    # plan held 1 s from 2 s before the event: e(1) = e0 + v herr + 0.5 v^2 (kcmd - kroad)
    i = np.nonzero(pre)[0][0]; vv = abs(v_e[i])
    held = lat[i] + vv * np.sin(herr[i]) * 1.0 + 0.5 * vv ** 2 * (kcm - kr) * 1.0
    full = [r for r, t in zip(dbg, t_d) if e["t_start"] - 2.0 <= t <= e["t_start"] and r.get("plan_k")]
    if full:                                          # the logged plans themselves, 1 s (10 steps) ahead
        ends = []
        for r in full:
            t0 = to_eval(r["t0"]); x, y, h = (np.interp(t0, t_e, ego[:, 1]), np.interp(t0, t_e, ego[:, 2]),
                                              np.interp(t0, t_e, yaw_u)); v = r["v0"]
            for ai, ki in list(zip(r["plan_a"], r["plan_k"]))[:10]:
                v1 = max(v + ai * 0.1, 0.0); vm = 0.5 * (v + v1)
                h += ki * vm * 0.1; x += vm * np.cos(h) * 0.1; y += vm * np.sin(h) * 0.1; v = v1
            ends.append(project(x, y)[1])
        held = float(np.array(ends)[np.argmax(np.abs(ends))])
    sw = (ts.t >= e["t_start"] - 2) & (ts.t <= e["t_start"]); cw = (tc.t >= e["t_start"] - 2) & (tc.t <= e["t_start"])
    late = int((mms[md] > 100).sum()); sk = int(skip[md].sum())
    traj = abs(held) > 1.3 or abs(kcm - kr) > 0.5 * max(abs(kr), 0.01)
    track = abs(ka - kcm) > 0.3 * max(abs(kcm), 0.01)
    verdict = "trajectory" if traj and not track else "tracking" if track and not traj else "both" if traj else "neither"
    if late or sk:
        verdict += " + delay"
    summ.append((e, verdict))
    print("%5.1f-%5.1f  %+.4f  %+.4f  %+.4f  %+.4f | %+.2f -> %+.2f   | %+.2f              | %+5.1f / %+5.1f          | %3d / %2d        | %s" % (
        e["t_start"], e["t_end"], kr, kp0, kcm, ka, lat[i], lat[m][np.argmax(np.abs(lat[m]))], held,
        tc.steer_deg[cw].mean(), ts.steer_deg[sw].mean(), late, sk, verdict))

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
pick = events[:3] if len(events) <= 3 else [events[0], events[2], events[-1]]
fig, ax = plt.subplots(len(pick), 2, figsize=(13, 3.6 * len(pick)))
for row, e in zip(np.atleast_2d(ax), pick):
    a, b = e["t_start"] - 4.0, min(e["t_end"], e["t_start"] + 6.0) + 1.0
    m = (t_e >= a) & (t_e <= b); md = (t_d >= a) & (t_d <= b)
    row[0].plot(t_e[m], k_road[m], "k-", label="road kappa")
    row[0].plot(t_d[md], k0[md], "b.", ms=3, label="plan kappa0")
    row[0].plot(t_d[md], kc[md], "c-", label="cmd kappa (t+0.1)")
    row[0].plot(t_e[m], k_act[m], "r-", label="actual kappa")
    row[0].axvspan(e["t_start"], e["t_end"], color="orange", alpha=0.2)
    row[0].set_title("lane event %.1f-%.1f s: curvature (1/m)" % (e["t_start"], e["t_end"])); row[0].legend(fontsize=7)
    row[1].plot(ego[m, 1], ego[m, 2], "r-", label="Ego path")
    w = (S >= s_e[m].min() - 10) & (S <= s_e[m].max() + 10)
    row[1].plot(P[w, 0], P[w, 1], "k--", label="route (lane centre)")
    i = np.nonzero(m & (t_e >= e["t_start"] - 2.0))[0][0]
    x, y, yaw, vv = ego[i, 1], ego[i, 2], ego[i, 3], abs(ego[i, 4])
    kh = float(np.nanmean(kc[(t_d >= t_e[i] - 0.2) & (t_d <= t_e[i] + 0.2)]))
    tt = np.linspace(0, 2.0, 41); hh = yaw + kh * vv * tt
    row[1].plot(x + np.cumsum(vv * np.cos(hh)) * 0.05, y + np.cumsum(vv * np.sin(hh)) * 0.05, "c-", label="cmd kappa held 2 s")
    row[1].set_aspect("equal"); row[1].legend(fontsize=7); row[1].set_title("top view, t = %.1f s" % t_e[i])
fig.tight_layout(); fig.savefig(D / "curve_events.png", dpi=90)
print("\nfigure", D / "curve_events.png")
print("verdicts", [v for _, v in summ])
