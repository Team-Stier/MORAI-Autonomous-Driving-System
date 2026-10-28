#!/usr/bin/env python3
"""DAgger expert labels from the shadow R2 collection driver (pod/shadow.sh): for every decision of an rl_episode
episode (steps.jsonl line j, policy driving), what the collection driver would have driven for the next 6.4 s, as the
training targets are made (morai_vla.data.ego_trajectory + the release action space's traj_to_action, de-normalised).

    dagger_label.py EP_DIR [EP_DIR ...] --rec REC_DIR --gt GT_NODE_DIR [--config DELIVERY/config.json] [--alpamayo SRC]
    dagger_label.py --selftest

Per decision (t_wall, t_sim from steps.jsonl; the shadow's records by host wall clock, shadow_rec.py):
  - path: the planner's /local_path/velocity_injected (control's input, back in map) newer than PLAN_AGE_S, used when it
    starts within PLAN_D0_M / PLAN_HE_RAD of the car (path_planner's own max_initial_distance_m / heading_error limits)
    and the car is within LANE_D_M of the official route lane: source 0. Otherwise source 1, the official route lane
    centreline (assets/route.json, ft_synth_hist.py's projection) from the car's projection on, with the plan's point
    speeds by distance ahead (no plan within NO_PLAN_S: no label). A plan shorter than the drive goes on along the lane.
  - rollout, 0.05 s steps for 6.4 s from the car's GT pose (shadow GT Ego status, the planners' frame), speed (rows,
    the GT the scorer reads) and steering (competition status): control's pp core as in control/cores/pp.hpp with
    pp.yaml (lookahead clamp(0.6 v, 2, 15) m, wheelbase 3.0, steering rate 1.047 rad/s, PI speed loop kp 1.2 ki 0.25
    kd 0.05, accel gain 5.74, brake gain 18.76, max accel 0.7, max brake 0.6, hold brake 0.2), the speed filter (full
    brake above 60 km/h outside sections 11-12), and the collection's GT overrides with their own code (gt-yield-node.py
    conflict/brake_for every 0.2 s, gt-aeb-node.py danger every 0.1 s with its 0.5 s hold) on the GT objects moved at
    constant velocity; kinematic bicycle, a = 5.74 accel - min(18.76 brake, A_BRAKE_MAX).
  - roundabout (section 9): roundabout_merge's own schedule by time (Schedule), not its point speeds.
  - (a, kappa): the 16-point GT history (rows, simulator time) + the rollout's speed and heading at 0.1 s.
Out per episode (the runner's contract, do not change): EP_DIR/dagger_labels.npz step int32 (N,) = steps.jsonl line,
ak float32 (N, 64, 2) a m/s^2 and kappa 1/m at 0.1 s, valid bool (N, 64), source int8 (N,) 0 planner / 1 official lane;
optional: signal int8 (N,) t0 light for the next judged stop line (0 none/unknown, 1 red, 2 yellow, 3 green), stop_d
float32 (N,) front axle to that line along the route (m), intervened bool (N,) inside a takeover.py start intervention.
valid is False where the label checker (tools/scorer/label_check.py) says fail or unrecoverable, and where no label
could be rolled out. No future information: the plan, light and objects of t0 only (user 10/28).
Debug: EP_DIR/dagger_debug.npz (rollout xy, v, flags, reasons) for dagger_eval.py.
"""
import argparse
import gzip
import importlib.util
import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
N_HIST, N_FUT, DT, SUB = 16, 64, 0.1, 2                # SUB rollout steps of 0.05 s per label step
PP = dict(wheelbase_m=3.0, lookahead_min_m=2.0, lookahead_max_m=15.0, lookahead_time_sec=0.6, steering_rate_limit_radps=1.0471975512,
          control_period_sec=0.05, speed_kp=1.2, speed_ki=0.25, speed_kd=0.05, integral_limit_mps_sec=4.0,
          acceleration_gain_mps2=5.74, braking_gain_mps2=18.76, maximum_accel=0.70, maximum_brake=0.60, stop_speed_mps=0.10, hold_brake=0.20)
STEER_LIMIT = 0.6981317007977318                      # the bridge's, pp.hpp
A_BRAKE_MAX = 9.0                                     # shortcut: tyre limit guess for brake 1.0 (no AEB stop in the fit); fit it from AEB stops
# the car's response, fitted on the collection driver's own drive (10/28 shadow run, 1,941 decisions: bridge commands vs GT
# speed, least squares over 0-0.4 s delays): a = 3.56 accel - 14.1 brake - 0.15 - 0.029 v, 0.05 s late. pp.yaml's gains
# (5.74, 18.76) made the labels accelerate too hard (+2.5 m/s at 6.4 s).
CAR = dict(accel=3.56, brake=14.1, a0=0.15, drag=0.029, delay_steps=1)
# beyond the plan (it ends ~50 m ahead): velocity_planner's curvature speed (curvature_velocity60/100.yaml) on the lane
CURVE = dict(a_lat=2.6, v_min=4.166666666666667, approach=1.70, cap60=16.666666666666668, cap100=27.77777777777778)
SPEED_LIMIT_MPS, NO_LIMIT_SECTIONS = 60 / 3.6, (11, 12)   # speed_filter selection.yaml
ROUNDABOUT = (9,)            # path_planner selection.yaml roundabout_merge: the yield node ignores vehicles there (shadow_objects.py)
# roundabout_merge (roundabout_merge.yaml, shadow_patches/roundabout_lead.patch): it commands its schedule
# command_step_sec ahead; a schedule ending at least this fast (or still speeding up) has merged and goes on past its
# searched horizon; schedule and plan stamps within RB_STAMP_S; with no merge for kinematic reasons only (speed,
# curvature: not the ring's objects) the newest schedule within RB_REUSE_S before goes on
RB_LEAD_S, RB_GO_MPS, RB_STAMP_S, RB_REUSE_S = 0.8, 0.5, 0.15, 1.0
# the ring's traffic circle (Object Info vehicles of 10/28 runs b and d, 8,532 positions: r p5-p95 18.29-18.47 m) and the
# band counted as ring traffic; label/vehicle boxes grow by RING_PAD_M. roundabout_merge's own predictions let its
# schedule cut in (10/28 run d expert: struck at s 889.1 by a ring car 11 m away 2 s before; 40 % of its ring labels
# met a ring car within 6.4 s): before the car enters the band, a label that meets a ring car moved along the circle at
# its t0 speed yields: the schedule capped to stop RING_STOP_M before the band at RING_DEC_MPS2, if that keeps out of
# the band and of the ring cars
RING_C, RING_R, RING_BAND_M, RING_PAD_M, RING_STOP_M, RING_DEC_MPS2 = (-102.36, 343.88), 18.4, 2.5, 0.5, 5.0, 2.5   # 5 m: stop_line_state x_m (pp overshoots ~3 m)
EGO_BOX = (1.55, 2.35, 0.95)                          # base_link -> box centre, half length, half width (body_m [-0.8, 3.9])
PLAN_AGE_S, NO_PLAN_S, PLAN_D0_M, PLAN_HE_RAD, LANE_D_M = 0.5, 2.0, 5.0, 0.5, 4.5
# a plan that keeps the car off the route lane (both > 2 m from it, 30 m ahead too) is the planner following a wrong lane,
# unless a standing object sits on the route lane ahead: then it is the obstacle bypass (10/28 shadow run: 3-4 m, plan 3 m)
PLAN_LANE_D_M, BYPASS_D_M = 2.0, 3.0
# velocity_planner (collection fork) when it injects one target speed into every point (curvature_velocity60/100.yaml,
# stop_line_state.yaml): curvature preview, time filter and rate limits, the stop-line cutoff on the ~50 m path. Only
# t0 information (user 10/28: no future light or object state in a label): the light holds its t0 state.
VP = dict(preview=45.0, spacing=2.0, a_lat=2.6, v_min=4.166666666666667, approach=1.70, eps=0.001, tau=0.35, acc=2.0,
          acc_curve=1.5, dec=5.0, cap60=16.666666666666668, cap100=27.77777777777778, stop_x=5.0, horizon=50.0, yellow_z=3.0)
SIGNAL_CODE = dict(unknown=0, red=1, yellow=2, green=3)     # the runner's code (as _ego): 0 none/unknown, 1 red, 2 yellow, 3 green
REASONS = ["ok", "plan_stale", "plan_far", "plan_heading", "off_lane", "no_plan", "wrong_way", "off_route", "no_history", "no_gt"]
OBJ_R_M = 120.0


# ------------------------------------------------------------------ records
def load_rec(rec_dir, t0, t1):
    """shadow_rec rows with t in [t0, t1], by kind, sorted by t."""
    out = {}
    fs = sorted(Path(rec_dir).glob("rec_*.jsonl")) or sorted(Path(rec_dir).glob("rec_*.jsonl.gz"))   # B2 keeps the .gz
    for f in fs:
        with (gzip.open(f, "rt") if f.suffix == ".gz" else open(f)) as fh:
            for line in fh:
                i = line.find(",", 2)
                try:
                    t = float(line[i + 1:line.find(",", i + 1)])
                except ValueError:
                    continue
                if t0 <= t <= t1:
                    try:
                        row = json.loads(line)
                    except ValueError:
                        continue                      # the line being written
                    out.setdefault(row[0], []).append(row)
    for v in out.values():
        v.sort(key=lambda r: r[1])
    return out


class Rec:
    def __init__(self, rows):
        self.rows = rows
        e = np.array([r[1:6] for r in rows.get("ego", [])], float).reshape(-1, 5)
        self.ego_t, self.ego = e[:, 0], e[:, 1:]
        self.ego[:, 2] = np.unwrap(self.ego[:, 2]) if len(e) else self.ego[:, 2]
        self.t = {k: np.array([r[1] for r in v]) for k, v in rows.items()}

    def last(self, kind, t):
        """the newest row of kind received at or before t, or None."""
        ts = self.t.get(kind)
        if ts is None or not len(ts):
            return None
        i = int(np.searchsorted(ts, t, side="right")) - 1
        return self.rows[kind][i] if i >= 0 else None

    def ego_at(self, t):
        if len(self.ego_t) < 2 or not self.ego_t[0] <= t <= self.ego_t[-1] + 0.2:
            return None
        return np.array([np.interp(t, self.ego_t, self.ego[:, k]) for k in range(4)])

    def objects(self, t):
        """GT objects at t: [(id, label, x, y, yaw, vx, vy, sx, sy)], velocity = GT when given (> 0.05) else the
        finite difference over >= 0.2 s, as the yield node estimates it. Object Info's velocity is in the object's own
        frame (10/28 shadow run: vs the finite difference 0.4 m/s off as body frame, 20 m/s as map frame), while the GT
        yield/AEB nodes read it as map axes and take a lead car on a north-south road for a crossing one: turned here."""
        cur = self.last("obj", t)
        if cur is None:
            return []
        prev = self.last("obj", cur[1] - 0.2)
        old = {o[0]: o for o in prev[2]} if prev else {}
        dt = cur[1] - prev[1] if prev else 0.0
        out = []
        for o in cur[2]:
            oid, lab, x, y, yaw, vx, vy, va, sx, sy = o
            if not (va and math.hypot(vx, vy) > 0.05):
                p = old.get(oid)
                vx, vy = ((x - p[2]) / dt, (y - p[3]) / dt) if p and dt > 0.05 else (0.0, 0.0)
            else:
                vx, vy = vx * math.cos(yaw) - vy * math.sin(yaw), vx * math.sin(yaw) + vy * math.cos(yaw)
            out.append((oid, lab, x, y, yaw, vx, vy, sx, sy))
        return out


# ------------------------------------------------------------------ route lane
class Lane:
    def __init__(self, route_json):
        r = json.load(open(route_json))
        self.P, self.S, self.lap = np.asarray(r["path"], float), np.asarray(r["s"], float), float(r["lap_m"])
        self.highway = tuple(r["exempt_s"])                 # 고주로 (60 km/h exemption): frenet_path picks the lane there
        keep = np.r_[True, np.diff(self.S) >= 0.2]          # route.json has a 0.1 m kink segment at s 160 (heading 10 deg)
        self.P, self.S = self.P[keep], self.S[keep]
        d = np.diff(self.P, axis=0)
        self.H = np.arctan2(d[:, 1], d[:, 0])

    def project(self, x, y, s_hint=None, win=30.0):
        """(s, d left +, heading) of (x, y), on the segments within s_hint +- win (all without a hint)."""
        P, S = self.P, self.S
        lo, hi = (0, len(P) - 1) if s_hint is None else (max(np.searchsorted(S, s_hint - win) - 1, 0), min(np.searchsorted(S, s_hint + win) + 1, len(P) - 1))
        a, b = P[lo:hi], P[lo + 1:hi + 1]; ab = b - a; L2 = np.maximum((ab ** 2).sum(1), 1e-12)
        u = np.clip(((np.array([x, y]) - a) * ab).sum(1) / L2, 0, 1); f = a + u[:, None] * ab
        j = int(np.argmin(np.hypot(x - f[:, 0], y - f[:, 1]))); h = math.atan2(ab[j, 1], ab[j, 0])
        d = -math.sin(h) * (x - f[j, 0]) + math.cos(h) * (y - f[j, 1])
        return S[lo + j] + u[j] * math.sqrt(L2[j]), d, h

    def ahead(self, s0, length, step=0.5):
        """lane centreline points from route s0 for length m (over the lap end)."""
        s = (s0 + np.arange(0.0, length, step)) % self.lap
        return np.stack([np.interp(s, self.S, self.P[:, 0]), np.interp(s, self.S, self.P[:, 1])], -1)


def extend(path, vp, lane, s0, v, sec):
    """a plan that ends before the drive does (it ends ~50 m ahead) goes on along the lane at the curvature speed."""
    L = float(np.sum(np.hypot(*np.diff(path, axis=0).T))) if len(path) > 1 else 0.0
    if L >= 6.4 * max(v, 1.0) + 20.0:
        return path, vp
    se = lane.project(path[-1, 0], path[-1, 1], s0 + L)[0]
    ext = lane.ahead(se + 0.5, 6.4 * max(v, 1.0) + 60.0)
    path, vp = np.vstack([path, ext]), np.r_[vp, curve_speed(ext, vp[-1], sec)]
    return path, approach(path, vp, len(vp) - len(ext))


def curve_speed(P, v_last, section):
    """velocity_planner's curvature speed on lane points P (0.5 m): min(cap, max(v_min, sqrt(a_lat / kappa))), kappa over
    2 m; 0 after a plan that ends stopped (a stop line or a yield it planned)."""
    if v_last < 0.1:
        return np.zeros(len(P))
    h = np.unwrap(np.arctan2(*np.gradient(P, axis=0).T[::-1]))
    k = np.abs(np.gradient(h, 0.5))
    k = np.convolve(k, np.ones(5) / 5, mode="same")
    cap = CURVE["cap100"] if section in NO_LIMIT_SECTIONS else CURVE["cap60"]
    return np.minimum(cap, np.maximum(CURVE["v_min"], np.sqrt(CURVE["a_lat"] / np.maximum(k, 1e-3))))


def approach(P, v, i0):
    """no point from i0 - 1 back faster than reaching the slower ones ahead at the planner's curve-approach deceleration."""
    d = np.r_[0, np.cumsum(np.hypot(*np.diff(P, axis=0).T))]
    out = v.copy()
    for i in range(len(v) - 2, max(i0 - 200, 0) - 1, -1):
        out[i] = min(out[i], math.sqrt(out[i + 1] ** 2 + 2 * CURVE["approach"] * (d[i + 1] - d[i])))
    return out


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


# ------------------------------------------------------------------ rollout
def load_gt(gt_dir):
    mods = {}
    for name, f in (("yield", "gt-yield-node.py"), ("aeb", "gt-aeb-node.py")):
        spec = importlib.util.spec_from_file_location(f"gt_{name}", Path(gt_dir) / f)
        m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m); mods[name] = m

    def project(path, cum, x, y):                     # the yield node's project(), vectorised (same result)
        P = np.asarray(path, float); a, b = P[:-1], P[1:]; ab = b - a; L2 = (ab ** 2).sum(1)
        ok = L2 >= 1e-9
        if not ok.any():
            return (None, 1e9)
        u = np.clip(((np.array([x, y]) - a) * ab).sum(1) / np.where(ok, L2, 1.0), 0, 1)
        d = np.where(ok, np.hypot(x - a[:, 0] - u * ab[:, 0], y - a[:, 1] - u * ab[:, 1]), np.inf)
        i = int(np.argmin(d))
        return (cum[i] + u[i] * math.sqrt(L2[i]), float(d[i]))
    mods["yield"].project = project
    return mods


def pp_step(st, path, v_path, speed=None):
    """control/cores/pp.hpp process() on the path in map frame; st: x y yaw v steer prev_v has_prev integ -> (accel, brake, steer rad, nearest)."""
    c, s = math.cos(st["yaw"]), math.sin(st["yaw"])
    dx, dy = path[:, 0] - st["x"], path[:, 1] - st["y"]
    lx, ly = c * dx + s * dy, -s * dx + c * dy
    nearest = int(np.argmin(np.hypot(lx, ly)))
    v = st["v"]
    L = min(max(max(0.0, v) * PP["lookahead_time_sec"], PP["lookahead_min_m"]), PP["lookahead_max_m"])
    tx, ty = lx[-1], ly[-1]
    i0 = max(1, nearest)
    if i0 < len(path):
        ax, ay, bx, by = lx[i0 - 1:-1], ly[i0 - 1:-1], lx[i0:], ly[i0:]
        ddx, ddy = bx - ax, by - ay
        sq = ddx * ddx + ddy * ddy
        lin = 2.0 * (ax * ddx + ay * ddy); con = ax * ax + ay * ay - L * L
        disc = lin * lin - 4.0 * sq * con
        with np.errstate(invalid="ignore", divide="ignore"):
            ratio = (-lin + np.sqrt(np.maximum(disc, 0))) / (2.0 * sq)
        ok = (sq > 0) & (disc >= 0) & (ratio >= 0) & (ratio <= 1) & (ax + ratio * ddx > 0)
        k = np.flatnonzero(ok)
        if len(k):
            k = k[0]; tx, ty = ax[k] + ratio[k] * ddx[k], ay[k] + ratio[k] * ddy[k]
    desired = math.atan2(2.0 * PP["wheelbase_m"] * ty, tx * tx + ty * ty) if tx > 0 else 0.0
    step = PP["steering_rate_limit_radps"] * PP["control_period_sec"]
    st["steer"] = min(max(min(max(desired, -STEER_LIMIT), STEER_LIMIT), st["steer"] - step), st["steer"] + step)
    target = float(v_path[nearest]) if speed is None else speed(nearest)
    deriv = (v - st["prev_v"]) / PP["control_period_sec"] if st["has_prev"] else 0.0
    st["prev_v"], st["has_prev"] = v, True
    if target == 0.0 and v <= PP["stop_speed_mps"]:
        st["integ"] = 0.0
        return 0.0, PP["hold_brake"], nearest
    err = target - v
    nxt = min(max(st["integ"] + err * PP["control_period_sec"], -PP["integral_limit_mps_sec"]), PP["integral_limit_mps_sec"])
    pdv = PP["speed_kp"] * err - PP["speed_kd"] * deriv
    req = pdv + PP["speed_ki"] * nxt
    up, lo = PP["maximum_accel"] * PP["acceleration_gain_mps2"], -PP["maximum_brake"] * PP["braking_gain_mps2"]
    if lo <= req <= up or (req > up and err < 0) or (req < lo and err > 0):
        st["integ"] = nxt
    acc = min(max(pdv + PP["speed_ki"] * st["integ"], lo), up)
    return (acc / PP["acceleration_gain_mps2"], 0.0, nearest) if acc >= 0 else (0.0, -acc / PP["braking_gain_mps2"], nearest)


class Speeds:
    """velocity_planner's target per 0.05 s step on this path (VP): min over the 2 m samples within 45 m ahead of
    sqrt(curve_target^2 + 2 * 1.7 * d), filtered (0.35 s) and rate limited from its t0 target, capped; 0 while a stop
    line it stops for lies within its 50 m path (stop_station: path distance of its stop point, 5 m before the line)."""

    def __init__(self, path, v_t0, cap, stop_station=None):
        cum = np.r_[0, np.cumsum(np.hypot(*np.diff(path, axis=0).T))]
        self.cum, self.cap, self.stop = cum, cap, stop_station
        g = np.arange(0.0, cum[-1], VP["spacing"])
        P = np.stack([np.interp(g, cum, path[:, 0]), np.interp(g, cum, path[:, 1])], -1)
        a, b, c = P[:-2], P[1:-1], P[2:]
        den = np.hypot(*(b - a).T) * np.hypot(*(c - b).T) * np.hypot(*(c - a).T)
        cross = np.abs((b[:, 0] - a[:, 0]) * (c[:, 1] - a[:, 1]) - (b[:, 1] - a[:, 1]) * (c[:, 0] - a[:, 0]))
        k = np.where(den > 1e-12, 2.0 * cross / np.maximum(den, 1e-12), 0.0)
        ct = np.clip(np.sqrt(VP["a_lat"] / np.maximum(k, 1e-9)), VP["v_min"], cap)
        self.g, self.ct = g[1:-1], np.where(k > VP["eps"], ct, np.inf)
        self.last = self.filt = min(v_t0, cap)

    def __call__(self, nearest):
        cur = self.cum[nearest]
        m = (self.g > cur) & (self.g <= cur + VP["preview"])
        raw = self.cap
        if m.any():
            raw = min(raw, float(np.min(np.sqrt(self.ct[m] ** 2 + 2.0 * VP["approach"] * (self.g[m] - cur)))))
        dt = PP["control_period_sec"]
        self.filt += (1.0 - math.exp(-dt / VP["tau"])) * (raw - self.filt)
        acc = VP["acc_curve"] if raw < self.cap - 1e-9 else VP["acc"]
        self.last = min(min(max(self.filt, self.last - VP["dec"] * dt), self.last + acc * dt), self.cap)
        self.filt = min(self.filt, self.cap)
        if self.stop is not None and self.stop - cur <= VP["horizon"]:
            return 0.0
        return self.last


class Schedule:
    """roundabout_merge's searched schedule (shadow_rec "rbs": t s v a from its plan time, cubic between states as its
    Edge) driven by time, one call per 0.05 s step: v(t_off + k dt), t_off = t0 - plan time + RB_LEAD_S. Its point speeds
    are by station: they lose a wait (a car stopped at the entry gets 0.08 m/s, its first jerk-limited step, and never
    starts) and end in the braking tail before the 40 m path's end, which replanning never drives. Past its horizon (t0
    predictions end there): its end speed, slowed for curvature as velocity_planner (Speeds), once it has merged (no
    speeding up behind ring cars no node sees there: 10/28 run b labels ran into them at 5-6 s), else it stays stopped."""

    def __init__(self, states, t_off, path, cap):
        self.S, self.t, self.k = np.asarray(states, float), t_off, 0
        self.path, self.cap, self.after = path, cap, None

    def v(self, t):
        S = self.S
        i = min(max(int(np.searchsorted(S[:, 0], t, side="right")) - 1, 0), len(S) - 2)
        (t0, _, v0, a0), (t1, _, v1, a1) = S[i], S[i + 1]
        h, u = t1 - t0, min(max(t - t0, 0.0), t1 - t0)
        c2, c3 = 3 * (v1 - v0) / h ** 2 - (2 * a0 + a1) / h, 2 * (v0 - v1) / h ** 3 + (a0 + a1) / h ** 2
        return max(0.0, v0 + u * (a0 + u * (c2 + u * c3)))

    def __call__(self, nearest):
        t = self.t + self.k * PP["control_period_sec"]
        self.k += 1
        if t <= self.S[-1, 0]:
            return self.v(t)
        if self.after is None:
            vt, at = self.S[-1, 2], self.S[-1, 3]
            sp = Speeds(self.path, vt, self.cap)
            self.after = (lambda n: min(vt, sp(n))) if vt >= RB_GO_MPS or (vt > 0.05 and at > 0) else (lambda n: 0.0)
        return self.after(nearest)


def _corners(x, y, h, hl, hw):
    c, s = math.cos(h), math.sin(h)
    return np.array([(x + a * c - b * s, y + a * s + b * c) for a, b in ((hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw))])


def _overlap(A, B):
    """two rectangles (4, 2) overlap: separating axis test."""
    for P in (A, B):
        for i in range(2):
            n = np.array([-(P[i + 1, 1] - P[i, 1]), P[i + 1, 0] - P[i, 0]])
            pa, pb = A @ n, B @ n
            if pa.max() < pb.min() or pb.max() < pa.min():
                return False
    return True


def ring_meet(traj, objs):
    """first 0.1 s step (or None) where traj (64, 4: x y yaw v) meets a ring car moved along the circle at its t0 speed."""
    cx, cy = RING_C
    ring = []
    for o in objs:
        r = math.hypot(o[2] - cx, o[3] - cy)
        if o[1] == "vehicle" and abs(r - RING_R) < RING_BAND_M and math.hypot(o[5], o[6]) > 0.3:
            w = ((o[2] - cx) * o[6] - (o[3] - cy) * o[5]) / (r * r)      # angular speed, + counter-clockwise
            ring.append((math.atan2(o[3] - cy, o[2] - cx), w, r, o[7] / 2 + RING_PAD_M, o[8] / 2 + RING_PAD_M))
    if not ring:
        return None
    for k, (x, y, h, _) in enumerate(traj):
        bx, by = x + EGO_BOX[0] * math.cos(h), y + EGO_BOX[0] * math.sin(h)
        A = None
        for a0, w, r, hl, hw in ring:
            a = a0 + w * 0.1 * (k + 1)
            ox, oy = cx + r * math.cos(a), cy + r * math.sin(a)
            if math.hypot(ox - bx, oy - by) > 8.0:
                continue
            A = _corners(bx, by, h, EGO_BOX[1] + RING_PAD_M, EGO_BOX[2] + RING_PAD_M) if A is None else A
            if _overlap(A, _corners(ox, oy, a + math.copysign(math.pi / 2, w), hl, hw)):
                return k
    return None


def outside_band(traj):
    """the car's front corners stay outside the ring's traffic band all along traj."""
    cx, cy = RING_C
    f = traj[:, :2] + (EGO_BOX[0] + EGO_BOX[1]) * np.stack([np.cos(traj[:, 2]), np.sin(traj[:, 2])], -1)
    return bool(np.all(np.hypot(f[:, 0] - cx, f[:, 1] - cy) > RING_R + RING_BAND_M))


def yield_at_band(path, speed):
    """speed (a Schedule) capped to stop RING_STOP_M before the path point where the car's front reaches the band."""
    cum = np.r_[0, np.cumsum(np.hypot(*np.diff(path, axis=0).T))]
    h = np.arctan2(*np.gradient(path, axis=0).T[::-1])
    f = path + (EGO_BOX[0] + EGO_BOX[1]) * np.stack([np.cos(h), np.sin(h)], -1)
    inside = np.flatnonzero(np.hypot(f[:, 0] - RING_C[0], f[:, 1] - RING_C[1]) <= RING_R + RING_BAND_M)
    end = (cum[inside[0]] if len(inside) else cum[-1]) - RING_STOP_M
    return lambda n: min(speed(n), math.sqrt(2.0 * RING_DEC_MPS2 * max(end - cum[n], 0.0)))


def rollout(path, v_path, x, y, yaw, v, steer, objs, section, gt, speed=None):
    """-> (x, y, yaw, v at 0.1 ... 6.4 s, flags dict): the collection driver from this state along path (M, 2) with
    point speeds v_path, or speed(nearest index) -> target (Speeds); objs from Rec.objects (moved at constant velocity)."""
    Y, A = gt["yield"], gt["aeb"]
    # the speed loop's integral as at steady speed (it holds the car against its drag; 0 at a stop, as pp resets it)
    i0 = (CAR["a0"] + CAR["drag"] * v) * PP["acceleration_gain_mps2"] / (CAR["accel"] * PP["speed_ki"]) if v > 0.5 else 0.0
    st = dict(x=x, y=y, yaw=yaw, v=v, steer=steer, prev_v=v, has_prev=False, integ=min(i0, PP["integral_limit_mps_sec"]))
    moving = [o for o in objs if math.hypot(o[5], o[6]) >= Y.MIN_SPEED and math.hypot(o[2] - x, o[3] - y) < OBJ_R_M
              and not (section in ROUNDABOUT and o[1] == "vehicle")]   # the ring traffic is roundabout_merge's
    aebo = [o for o in objs if o[1] != "vehicle" and math.hypot(o[2] - x, o[3] - y) < OBJ_R_M]
    dt = PP["control_period_sec"]
    out, yb, aeb_until, flags = [], None, -1.0, dict(yield_on=False, aeb_on=False, sf_on=False, yield0=False, aeb0=False)
    queue = [(0.0, 0.0)] * CAR["delay_steps"]         # commands reach the car one control step late
    for k in range(N_FUT * SUB):
        t = k * dt
        accel, brake, near = pp_step(st, path, v_path, speed)
        if (moving or aebo) and k % 2 == 0:
            ahead = path[near:]
            rel = np.vstack([[st["x"], st["y"]], ahead]) if len(ahead) else np.array([[st["x"], st["y"]]])
            pts = [tuple(p) for p in rel]
            if moving and k % 4 == 0:                 # yield: judge as the node does (20 Hz there, 5 Hz here)
                cum = [0.0]
                for (a0, b0), (c0, d0) in zip(pts, pts[1:]):
                    cum.append(cum[-1] + math.hypot(c0 - a0, d0 - b0))
                m = next((i for i, s in enumerate(cum) if s > Y.S_MAX + 5), len(cum))
                hit = None
                for o in moving:
                    c = Y.conflict(pts[:m], cum[:m], st["v"], (o[2] + o[5] * t, o[3] + o[6] * t, o[5], o[6], max(o[7], o[8]) / 2))
                    if c and (hit is None or c[0] < hit[0]):
                        hit = c
                yb = Y.brake_for(st["v"], hit[0], hit[2]) if hit else None
                flags["yield0"] |= k == 0 and yb is not None
            if aebo:                                  # AEB: every 0.1 s, held 0.5 s
                ob = [(o[0], o[2] + o[5] * t, o[3] + o[6] * t, o[5], o[6], (o[4], o[7] / 2, o[8] / 2)) for o in aebo]
                if len(pts) > 1 and A.danger(pts[:150], st["yaw"], abs(st["v"]), ob) is not None:
                    aeb_until = t + A.HOLD_S
                    flags["aeb0"] |= k == 0
        if t <= aeb_until:
            accel, brake = 0.0, 1.0; flags["aeb_on"] = True
        elif yb is not None:
            accel, brake = 0.0, yb; flags["yield_on"] = True
        if section not in NO_LIMIT_SECTIONS and abs(st["v"]) > SPEED_LIMIT_MPS:
            accel, brake = 0.0, 1.0; flags["sf_on"] = True
        queue.append((accel, brake)); accel, brake = queue.pop(0)
        v0 = st["v"]
        a = CAR["accel"] * accel - min(CAR["brake"] * brake, A_BRAKE_MAX) - ((CAR["a0"] + CAR["drag"] * v0) if v0 > 0.05 else 0.0)
        v1 = max(0.0, v0 + a * dt); vm = 0.5 * (v0 + v1)
        yr = vm * math.tan(st["steer"]) / PP["wheelbase_m"]
        hm = st["yaw"] + 0.5 * yr * dt
        st["x"] += vm * math.cos(hm) * dt; st["y"] += vm * math.sin(hm) * dt; st["yaw"] += yr * dt; st["v"] = v1
        if k % SUB == SUB - 1:
            out.append((st["x"], st["y"], st["yaw"], st["v"]))
    return np.array(out), flags


# ------------------------------------------------------------------ (a, kappa)
class Converter:
    """morai_vla.data.ego_trajectory + UnicycleAccelCurvatureActionSpace.traj_to_action, de-normalised (training's)."""

    def __init__(self, config, alpamayo=None):
        if alpamayo:
            sys.path.insert(0, str(alpamayo))
        sys.path.insert(0, str(HERE.parent / "server" / "vendor"))
        import torch
        from alpamayo_r1.action_space import UnicycleAccelCurvatureActionSpace
        from morai_vla.data import ego_trajectory
        self.torch, self.ego_trajectory = torch, ego_trajectory
        self.cfg = {k: v for k, v in config.items() if k != "_target_"}
        self.space = UnicycleAccelCurvatureActionSpace(**self.cfg)

    def __call__(self, V, YAW):
        """V, YAW (B, 80) 10 Hz speed and unwrapped heading, row 15 = t0 -> (ak (B, 64, 2) physical, fut xy (B, 64, 2) ego frame)."""
        t = self.torch
        parts = [self.ego_trajectory(v.astype(np.float64), yw.astype(np.float64), N_HIST - 1) for v, yw in zip(V, YAW)]
        hx, hr, fx, fr = (t.cat([p[i] for p in parts]).double() for i in range(4))
        with t.no_grad():
            act = self.space.traj_to_action(hx, hr, fx, fr).numpy()
        ak = np.stack([act[..., 0] * self.cfg["accel_std"] + self.cfg["accel_mean"],
                       act[..., 1] * self.cfg["curvature_std"] + self.cfg["curvature_mean"]], -1)
        return ak.astype(np.float32), fx[..., :2].numpy()


# ------------------------------------------------------------------ one episode
def rows_of(ep):
    """GT rows on simulator time: (tau, x, y, yaw unwrapped, v) arrays (rl_episode rows.json.gz ego_status)."""
    r = json.load(gzip.open(ep / "rows.json.gz", "rt"))["ego_status"]
    a = np.array([[ns / 1e9, *v] for ns, v in r], float)
    a[:, 3] = np.unwrap(a[:, 3])
    return a


def plan_of(rec, t):
    """-> (row or None, age)."""
    p = rec.last("vp", t)
    return (p, t - p[1]) if p else (None, math.inf)


_C = {}                                               # one episode's inputs, shared with the forked workers


def one(j):
    """decision j of _C's episode -> (reason, source, valid, rollout (64, 4) or None, dbg (9,), flags (5,))."""
    c = _C; steps, G, rec, lane, gt = c["steps"], c["G"], c["rec"], c["lane"], c["gt"]
    tw, ts = c["tw"][j], c["ts"][j]
    nan9, f5 = np.full(9, np.nan), np.zeros(5, bool)
    none = dict(verdict="none", valid=False, reasons="", tags="", signal=0, stop_d=math.inf, mode="")
    if not c["ok_hist"][j]:
        return REASONS.index("no_history"), 0, False, None, nan9, f5, none
    e = rec.ego_at(tw)
    if e is None:
        return REASONS.index("no_gt"), 0, False, None, nan9, f5, none
    x, y, yaw, _ = e
    v = float(np.interp(ts, G[:, 0], G[:, 4]))
    s0, d, h0 = lane.project(x, y, steps[j].get("s"))
    if abs(d) > 15.0:
        s0, d, h0 = lane.project(x, y)
    ey = wrap(yaw - h0)
    p, age = plan_of(rec, tw)
    P = Pv = None; d0 = he = dend = math.nan
    if p is not None and age <= NO_PLAN_S and len(p[3]) >= 2:
        P, Pv = np.stack([p[3], p[4]], -1), np.asarray(p[5], float)
        dd = np.hypot(P[:, 0] - x, P[:, 1] - y); i0 = int(np.argmin(dd)); d0 = float(dd[i0])
        cs = np.r_[0, np.cumsum(np.hypot(*np.diff(P[i0:], axis=0).T))]
        i1 = i0 + min(int(np.searchsorted(cs, 2.0)), len(cs) - 1); i_0 = i0 if i1 > i0 else max(i0 - 1, 0)
        he = wrap(math.atan2(P[i1, 1] - P[i_0, 1], P[i1, 0] - P[i_0, 0]) - yaw) if i1 > i_0 else 0.0   # over ~2 m
        k30 = min(i0 + int(np.searchsorted(cs, 30.0)), len(P) - 1)
        dend = lane.project(P[k30, 0], P[k30, 1], s0 + 30.0)[1]
    objs = rec.objects(tw)
    bypass = any(math.hypot(o[5], o[6]) < 0.3 and abs(o[2] - x) + abs(o[3] - y) < 120.0 and
                 -20.0 < (q := lane.project(o[2], o[3], s0 + 20.0, 60.0))[0] - s0 < 80.0 and abs(q[1]) < BYPASS_D_M for o in objs)
    sec = rec.last("sec", tw); sec = sec[2] if sec else 0
    # the highway (its route s span; the section id turns 13 before its end): frenet_path picks the lane there and the
    # expert drove up to 6 m off route.json's lane
    lanes_free = sec in NO_LIMIT_SECTIONS or lane.highway[0] <= s0 <= lane.highway[1]
    why = ("plan_stale" if P is None or age > PLAN_AGE_S else "plan_far" if d0 > PLAN_D0_M
           else "plan_heading" if abs(he) > PLAN_HE_RAD
           else "off_lane" if not (bypass or lanes_free) and (abs(d) > LANE_D_M or (abs(d) > PLAN_LANE_D_M and abs(dend) > PLAN_LANE_D_M))
           else "ok")
    dbg = np.array([age, d0, he, d, ey, dend, s0, v, steps[j].get("s", math.nan)], float)
    comp = rec.last("comp", tw); steer = float(comp[3]) if comp else 0.0
    if why == "ok":
        src = 0
        i0 = int(np.argmin(np.hypot(P[:, 0] - x, P[:, 1] - y)))
        path, vp = (P[i0:], Pv[i0:]) if len(P) - i0 >= 2 else (P, Pv)
    else:
        src = 1
        for bad, r in ((abs(ey) > math.pi / 2, "wrong_way"), (abs(d) > 15.0, "off_route"), (P is None, "no_plan")):
            if bad:
                return REASONS.index(r), 1, False, None, dbg, f5, none
        path = lane.ahead(s0, 160.0)
        i0 = int(np.argmin(np.hypot(P[:, 0] - x, P[:, 1] - y)))
        cs = np.r_[0, np.cumsum(np.hypot(*np.diff(P[i0:], axis=0).T))]
        vp = np.interp(np.arange(len(path)) * 0.5, cs, Pv[i0:])
    path, vp = extend(path, vp, lane, s0, v, sec)
    # the t0 light and the judged stop line ahead (scorer stop lines, front axle; LabelChecker's own reading)
    chk = c["chk"]
    ego = dict(x=x, y=y, yaw=yaw, v=v, s=steps[j].get("s"))
    sig = chk.signal_at(rec.rows.get("tlgt", []), tw, ego)
    sh = chk._s_hint(ego)
    front = float(chk._frenet(np.array([[x, y]]), sh, np.zeros(1))[0][0]) + chk.front
    ahead = {sid: chk._ahead(front, st_) for sid, st_ in chk.stops.items()}
    stop_d = min(ahead.values()) if ahead else math.inf
    tgt = min(chk.stops, key=lambda k: chk._ahead(front, chk.stops[k] + 5.0), default=None)   # zone: 5 m past the line
    past = tgt is not None and chk._ahead(front, chk.stops[tgt] + 5.0) < 5.0
    d_line = ahead[tgt] if tgt is not None and not past else math.inf
    uniform = bool(np.ptp(Pv) < 0.05)                 # velocity_planner injected one target (else the path planner's own speeds)
    plan_zero = uniform and Pv[0] < 0.05              # the planner is stopping at t0 (a light it saw, before its n_frames greens)
    mode = "plan"
    if d_line < VP["horizon"] + 30.0 or past:
        brake_m = v * c["max_brake_delay_s"] + v * v / (2 * c["max_brake_mps2"])
        if past and (sig in ("red", "yellow") or plan_zero):
            mode = "go_past"                          # front axle over the line at t0: go once the way is clear (yield/AEB)
        elif sig == "red" or (plan_zero and sig != "green") or (sig == "green" and plan_zero):
            mode = "stop" if d_line >= brake_m else "go_unstoppable"
        elif sig == "yellow":
            mode = "stop" if v * v / (2 * VP["yellow_z"]) <= d_line + chk.front - VP["stop_x"] else "go_yellow"
    cap = VP["cap100"] if sec in NO_LIMIT_SECTIONS else VP["cap60"]
    speed = None
    if uniform or mode != "plan":
        v_t0 = v if mode.startswith("go") and plan_zero else float(Pv[0])
        stop_station = (d_line + chk.front - VP["stop_x"]) if mode == "stop" else None
        if mode == "plan" and plan_zero:
            stop_station = 0.0                        # stopping for a line this map reading does not see: keep it
        speed = Speeds(path, v_t0, cap, stop_station)
    elif sec in ROUNDABOUT and (q := rec.last("rbs", p[1])) is not None and abs(q[2] - p[2]) < RB_STAMP_S:   # the plan's own cycle
        mode = "rb_schedule"
        if not q[3] and q[4:] == ["kinematic"]:
            i = int(np.searchsorted(rec.t["rbs"], p[1], side="right"))
            q, mode = next((r for r in reversed(rec.rows["rbs"][max(i - 40, 0):i]) if len(r[3]) >= 2), None), "rb_reuse"
            q = q if q is not None and tw - q[2] < RB_REUSE_S else None
        if q is not None and len(q[3]) >= 2:
            speed = Schedule(q[3], tw - q[2] + RB_LEAD_S, path, cap)
        else:
            mode = "plan"                             # no merge (the ring's objects): the planner's stop request
    out, fl = rollout(path, vp, x, y, yaw, v, steer, objs, sec, gt, speed)
    if mode.startswith("rb_") and outside_band(out[:1]) and ring_meet(out, objs) is not None:
        speed.k = 0                                   # the same schedule from t0, capped to yield at the band
        stop, fs = rollout(path, vp, x, y, yaw, v, steer, objs, sec, gt, yield_at_band(path, speed))
        if outside_band(stop) and ring_meet(stop, objs) is None:
            out, fl, mode = stop, fs, "rb_yield"
    r = chk.check(ego, out, signal=sig, objects=objs, steer=steer)
    extra = dict(verdict=r["verdict"], valid=bool(r["valid"]), reasons=" ".join(r["reasons"]), tags=" ".join(r["tags"]),
                 signal=SIGNAL_CODE.get(sig, 0), stop_d=stop_d, mode=mode)
    return REASONS.index(why), src, True, out, dbg, np.array([fl["yield0"], fl["aeb0"], fl["yield_on"], fl["aeb_on"], fl["sf_on"]]), extra


def label_episode(ep, rec_dir, lane, gt, conv, chk, actual=False, jobs=1):
    steps = [json.loads(l) for l in open(ep / "steps.jsonl")]
    if not steps:
        return None
    G = rows_of(ep)
    tw = np.array([s["t_wall"] for s in steps]); ts = np.array([s["t_sim"] for s in steps])
    rec = Rec(load_rec(rec_dir, tw.min() - 5.0, tw.max() + 8.0))
    N = len(steps)
    V, YAW, Va, YAWa = (np.zeros((N, 80)) for _ in range(4))
    th = ts[:, None] + DT * np.arange(-(N_HIST - 1), 1)[None]
    tf = ts[:, None] + DT * np.arange(1, N_FUT + 1)[None]
    ok_hist = th[:, 0] >= G[0, 0] - 0.05
    ok_act = ok_hist & (tf[:, -1] <= G[-1, 0])
    back = np.array([st.get("since_return_s", np.inf) for st in steps], float)   # rl_episode --returns: a checkpoint jump
    jumps = np.unique(np.round((ts - back)[np.isfinite(back)], 2))
    for tj in jumps:                                  # history across a jump: no label; a driven future across it: not compared
        ok_hist &= ~((ts >= tj) & (ts < tj + DT * (N_HIST - 1) + 1e-6))
        ok_act &= ~((ts < tj) & (ts + DT * N_FUT >= tj))
    for j in np.flatnonzero(ok_hist):
        V[j, :N_HIST], YAW[j, :N_HIST] = np.interp(th[j], G[:, 0], G[:, 4]), np.interp(th[j], G[:, 0], G[:, 3])
        Va[j], YAWa[j] = V[j], YAW[j]
        Va[j, N_HIST:], YAWa[j, N_HIST:] = np.interp(tf[j], G[:, 0], G[:, 4]), np.interp(tf[j], G[:, 0], G[:, 3])
    iv, n_iv = np.zeros(N, bool), 0                   # takeover.py's start interventions (wall clock windows)
    for f in (Path(rec_dir) / "takeover.jsonl", Path(rec_dir) / "takeover.jsonl.gz"):
        if f.exists():
            for line in (gzip.open(f, "rt") if f.suffix == ".gz" else open(f)):
                w = json.loads(line)
                hit = (tw >= w["t_start"]) & (tw <= w["t_end"])
                iv |= hit; n_iv += bool(hit.any())
            break
    sg = chk.lc["signal"]
    _C.update(steps=steps, G=G, rec=rec, lane=lane, gt=gt, tw=tw, ts=ts, ok_hist=ok_hist, chk=chk,
              max_brake_mps2=sg["max_brake_mps2"], max_brake_delay_s=sg["max_brake_delay_s"])
    if jobs > 1:
        import multiprocessing as mp
        with mp.get_context("fork").Pool(jobs) as pool:
            res = pool.map(one, range(N), chunksize=16)
    else:
        res = [one(j) for j in range(N)]
    reason = np.array([r[0] for r in res], np.int8); source = np.array([r[1] for r in res], np.int8)
    valid = np.array([r[2] for r in res], bool); dbg = np.stack([r[4] for r in res]); flags = np.stack([r[5] for r in res])
    ex = [r[6] for r in res]
    rolled = valid.copy()                             # a rollout exists (the label checker may still fail it)
    valid &= np.array([e["valid"] for e in ex], bool)   # LabelChecker: fail and unrecoverable -> no label
    roll = np.full((N, N_FUT, 4), np.nan)
    for j, r in enumerate(res):
        if r[3] is not None and rolled[j]:
            roll[j] = r[3]
            x0yaw = rec.ego_at(tw[j])[2]
            V[j, N_HIST:] = r[3][:, 3]
            YAW[j, N_HIST:] = YAW[j, N_HIST - 1] + np.unwrap(np.r_[x0yaw, r[3][:, 2]])[1:] - x0yaw
    ak = np.zeros((N, N_FUT, 2), np.float32); fxy = np.full((N, N_FUT, 2), np.nan)
    if rolled.any():
        ak[rolled], fxy[rolled] = conv(V[rolled], YAW[rolled])
    tmp = ep / "dagger_labels.tmp.npz"                # the runner reads it once it appears: write, then rename
    np.savez(tmp, step=np.arange(N, dtype=np.int32), ak=ak, valid=np.repeat(valid[:, None], N_FUT, 1), source=source,
             signal=np.array([e["signal"] for e in ex], np.int8), stop_d=np.array([e["stop_d"] for e in ex], np.float32),
             intervened=iv)
    tmp.replace(ep / "dagger_labels.npz")
    extra = {}
    if actual and ok_act.any():                       # the drive that really happened (expert-drive accuracy runs)
        extra["ak_actual"], extra["fxy_actual"] = np.zeros((N, N_FUT, 2), np.float32), np.full((N, N_FUT, 2), np.nan)
        extra["ak_actual"][ok_act], extra["fxy_actual"][ok_act] = conv(Va[ok_act], YAWa[ok_act])
    np.savez(ep / "dagger_debug.npz", t_wall=tw, t_sim=ts, rollout=roll, fxy=fxy, dbg=dbg, reason=reason, flags=flags, rolled=rolled,
             verdict=np.array([e["verdict"] for e in ex]), check_reasons=np.array([e["reasons"] for e in ex]),
             check_tags=np.array([e["tags"] for e in ex]), mode=np.array([e["mode"] for e in ex]),
             ok_actual=ok_act, reasons=np.array(REASONS), dbg_cols=np.array(["plan_age", "plan_d0", "plan_he", "d_ego", "yaw_err",
             "d_plan_30m", "route_s", "v", "scorer_s"]), flag_cols=np.array(["yield0", "aeb0", "yield", "aeb", "speed_filter"]), **extra)
    import collections
    return dict(episode=str(ep), n=N, valid=int(valid.sum()), source1=int((source == 1).sum()),
                reasons={r: int((reason == i).sum()) for i, r in enumerate(REASONS) if (reason == i).any()},
                verdicts=dict(collections.Counter(e["verdict"] for e in ex)), interventions=n_iv, intervened_rows=int(iv.sum()),
                modes=dict(collections.Counter(e["mode"] for e in ex if e["mode"])))


def selftest():
    """rollout + pp on synthetic paths (no ROS, no torch)."""
    class Y:  # stubs: no objects
        MIN_SPEED, S_MAX = 0.3, 70.0
    class A:
        HOLD_S = 0.5
    gt = {"yield": Y, "aeb": A}
    s = np.arange(0, 300, 0.5)
    straight = np.stack([s, np.zeros_like(s)], -1)
    out, _ = rollout(straight, np.full(len(s), 10.0), 0.0, 0.0, 0.0, 10.0, 0.0, [], 1, gt)
    assert abs(out[-1, 1]) < 0.05 and abs(out[-1, 3] - 10.0) < 0.2 and abs(out[-1, 0] - 64.0) < 1.0, out[-1]
    out, _ = rollout(straight, np.full(len(s), 10.0), 0.0, 1.5, 0.0, 10.0, 0.0, [], 1, gt)    # 1.5 m left: back to the line
    assert abs(out[-1, 1]) < 0.3, out[-1]
    R = 30.0; th = s / R; circ = np.stack([R * np.sin(th), R - R * np.cos(th)], -1)
    out, _ = rollout(circ, np.full(len(s), 8.0), 0.0, 0.0, 0.0, 8.0, 0.0, [], 1, gt)
    r = np.hypot(out[-20:, 0], out[-20:, 1] - R)
    assert np.all(np.abs(r - R) < 1.0), r
    vs = np.where(s < 40.0, 10.0 * np.clip((40.0 - s) / 40.0, 0, 1) ** 0.5, 0.0)            # stop at 40 m
    out, _ = rollout(straight, vs, 0.0, 0.0, 0.0, 10.0, 0.0, [], 1, gt)
    assert out[-1, 3] < 2.0 and out[:, 0].max() < 42.0, out[-1]   # pp's lag: it creeps up to the line
    out, fl = rollout(straight, np.full(len(s), 25.0), 0.0, 0.0, 0.0, 17.0, 0.0, [], 1, gt)   # speed filter at 60 km/h
    assert fl["sf_on"] and out[:, 3].max() < 17.5, out[:, 3].max()
    sched = [[0.0, 0.0, 0.0, 0.0], [2.0, 0.0, 0.0, 0.0], [2.5, 0.083, 1.0, 2.0], [3.0, 0.583, 2.0, 2.0], [4.5, 3.583, 4.0, 0.0]]
    out, _ = rollout(straight, np.zeros(len(s)), 0.0, 0.0, 0.0, 0.0, 0.0, [], 9, gt, Schedule(sched, RB_LEAD_S, straight, 16.7))
    assert out[10, 3] < 0.05 and 3.5 < out[-1, 3] < 4.5 and out[-1, 0] > 10.0, out[[10, -1]]   # waits ~2 s (less its lead), goes on at 4 m/s
    out, _ = rollout(straight, np.zeros(len(s)), 0.0, 0.0, 0.0, 0.0, 0.0, [], 9, gt, Schedule(sched[:2], RB_LEAD_S, straight, 16.7))
    assert out[:, 3].max() < 0.05, out[:, 3].max()                                      # still waiting at its horizon: stays
    cx, cy = RING_C                                    # a straight path into the ring from 30 m out, a ring car 90 deg away
    a = np.arange(0, 60, 0.5); line = np.stack([np.full_like(a, cx), cy - RING_R - 12.0 + a], -1)
    a0 = -math.pi / 2 - 0.5                            # 9 m before the entry point, counter-clockwise at 4 m/s
    car = [(1, "vehicle", cx + RING_R * math.cos(a0), cy + RING_R * math.sin(a0), a0 + math.pi / 2,
            -4.0 * math.sin(a0), 4.0 * math.cos(a0), 4.6, 1.9)]
    go, _ = rollout(line, np.full(len(a), 5.0), cx, cy - RING_R - 12.0, math.pi / 2, 5.0, 0.0, [], 9, gt)
    assert ring_meet(go, car) is not None and ring_meet(go, []) is None and outside_band(go[:1]) and not outside_band(go)
    sched = Schedule([[0.0, 0.0, 5.0, 0.0], [5.0, 25.0, 5.0, 0.0]], RB_LEAD_S, line, 16.7)
    yl, _ = rollout(line, np.full(len(a), 5.0), cx, cy - RING_R - 12.0, math.pi / 2, 5.0, 0.0, [], 9, gt, yield_at_band(line, sched))
    gap = (cy - yl[-1, 1] - EGO_BOX[0] - EGO_BOX[1]) - RING_R - RING_BAND_M       # its front stops this short of the band
    assert outside_band(yl) and yl[-1, 3] < 0.1 and 0 < gap < RING_STOP_M, (gap, yl[-1])
    print("selftest ok")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("episodes", nargs="*")
    ap.add_argument("--rec", help="shadow_rec dir (rec_w<n>)")
    ap.add_argument("--gt", help="the collection GT node scripts (morai-parallel-collect scripts/wsl)")
    ap.add_argument("--config", help="delivery config.json (action_space_cfg); default: the release stats")
    ap.add_argument("--alpamayo", help="alpamayo src dir when alpamayo_r1 is not installed")
    ap.add_argument("--route", default=str(HERE.parent / "assets" / "route.json"))
    ap.add_argument("--actual", action="store_true", help="also convert the driven future (expert-drive accuracy)")
    ap.add_argument("--jobs", type=int, default=1, help="worker processes for the rollouts (fork)")
    ap.add_argument("--scorer", default=str(HERE.parents[3] / "Alpamayo-Cosmos" / "tools" / "scorer"),
                    help="tools/scorer (LabelChecker: verdicts, judged stop lines)")
    ap.add_argument("--lookahead-s", type=float, help="pp lookahead time (pp.yaml 0.6 s), for the curve test")
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    if a.selftest:
        return selftest()
    if a.lookahead_s:
        PP["lookahead_time_sec"] = a.lookahead_s
    cfg = json.load(open(a.config))["action_space_cfg"] if a.config else dict(
        accel_mean=0.02902694707164455, accel_std=0.6810426736454882, curvature_mean=0.0002692167976330542,
        curvature_std=0.026148280660833106, a_lambda=1e-4, a_ridge=1e-4, kappa_lambda=1e-4, kappa_ridge=1e-4,
        theta_lambda=1e-6, theta_ridge=1e-8, v_lambda=1e-6, v_ridge=1e-4, dt=0.1, n_waypoints=64)   # train/tests/test_data_math.py
    conv, lane, gt = Converter(cfg, a.alpamayo), Lane(a.route), load_gt(a.gt)
    sys.path.insert(0, a.scorer)
    from label_check import LabelChecker
    chk = LabelChecker()
    for ep in a.episodes:
        print(json.dumps(label_episode(Path(ep), a.rec, lane, gt, conv, chk, a.actual, a.jobs)), flush=True)


if __name__ == "__main__":
    main()
