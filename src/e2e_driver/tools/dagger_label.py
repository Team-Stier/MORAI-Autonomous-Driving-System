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
  - (a, kappa): the 16-point GT history (rows, simulator time) + the rollout's speed and heading at 0.1 s.
Out per episode (the runner's contract, do not change): EP_DIR/dagger_labels.npz step int32 (N,) = steps.jsonl line,
ak float32 (N, 64, 2) a m/s^2 and kappa 1/m at 0.1 s, valid bool (N, 64), source int8 (N,) 0 planner / 1 official lane.
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
PLAN_AGE_S, NO_PLAN_S, PLAN_D0_M, PLAN_HE_RAD, LANE_D_M = 0.5, 2.0, 5.0, 0.5, 4.5
# a plan that keeps the car off the route lane (both > 2 m from it, 30 m ahead too) is the planner following a wrong lane,
# unless a standing object sits on the route lane ahead: then it is the obstacle bypass (10/28 shadow run: 3-4 m, plan 3 m)
PLAN_LANE_D_M, BYPASS_D_M = 2.0, 3.0
REPLAN_D_M = 3.0                                      # a later plan applies to the rolled-out car when it starts this close
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


def pp_step(st, path, v_path):
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
    target = float(v_path[nearest])
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


def rollout(path, v_path, x, y, yaw, v, steer, objs, section, gt, replan=None):
    """-> (x, y, yaw, v at 0.1 ... 6.4 s, flags dict): the collection driver from this state along path (M, 2) with
    point speeds v_path; objs from Rec.objects (moved at constant velocity). replan(t, x, y, yaw) -> (path, v_path) or
    None: the planner's later output where it applies to this car (every 0.1 s)."""
    Y, A = gt["yield"], gt["aeb"]
    # the speed loop's integral as at steady speed (it holds the car against its drag; 0 at a stop, as pp resets it)
    i0 = (CAR["a0"] + CAR["drag"] * v) * PP["acceleration_gain_mps2"] / (CAR["accel"] * PP["speed_ki"]) if v > 0.5 else 0.0
    st = dict(x=x, y=y, yaw=yaw, v=v, steer=steer, prev_v=v, has_prev=False, integ=min(i0, PP["integral_limit_mps_sec"]))
    moving = [o for o in objs if math.hypot(o[5], o[6]) >= Y.MIN_SPEED and math.hypot(o[2] - x, o[3] - y) < OBJ_R_M]
    aebo = [o for o in objs if o[1] != "vehicle" and math.hypot(o[2] - x, o[3] - y) < OBJ_R_M]
    dt = PP["control_period_sec"]
    out, yb, aeb_until, flags = [], None, -1.0, dict(yield_on=False, aeb_on=False, sf_on=False, yield0=False, aeb0=False)
    queue = [(0.0, 0.0)] * CAR["delay_steps"]         # commands reach the car one control step late
    for k in range(N_FUT * SUB):
        t = k * dt
        if replan is not None and k % 2 == 0 and k:
            new = replan(t, st["x"], st["y"], st["yaw"])
            if new is not None:
                path, v_path = new; flags["replans"] = flags.get("replans", 0) + 1
        accel, brake, near = pp_step(st, path, v_path)
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
    if not c["ok_hist"][j]:
        return REASONS.index("no_history"), 0, False, None, nan9, f5
    e = rec.ego_at(tw)
    if e is None:
        return REASONS.index("no_gt"), 0, False, None, nan9, f5
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
    lanes_free = sec in NO_LIMIT_SECTIONS             # the highway: frenet_path picks its lane, the expert drove up to 6 m off route.json's
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
                return REASONS.index(r), 1, False, None, dbg, f5
        path = lane.ahead(s0, 160.0)
        i0 = int(np.argmin(np.hypot(P[:, 0] - x, P[:, 1] - y)))
        cs = np.r_[0, np.cumsum(np.hypot(*np.diff(P[i0:], axis=0).T))]
        vp = np.interp(np.arange(len(path)) * 0.5, cs, Pv[i0:])
    path, vp = extend(path, vp, lane, s0, v, sec)
    rp = None
    if src == 0 and c["replan"]:
        last = [p[1]]

        def rp(t, cx, cy, cyaw):
            """velocity_planner injects one time-filtered target speed into every point (10/28: 14.7 m/s on the whole
            path, 0 two seconds later at a stop line), so the plan at t0 cannot see the curve or the stop ahead: the
            planner's own later output takes over where it starts at this car (REPLAN_D_M, PLAN_HE_RAD)."""
            q = rec.last("vp", float(np.interp(ts + t, c["ts"], c["tw"], right=c["tw"][-1] + (ts + t - c["ts"][-1]) * c["rho"])))
            if q is None or q[1] <= last[0] or len(q[3]) < 2:
                return None
            Q, Qv = np.stack([q[3], q[4]], -1), np.asarray(q[5], float)
            dd = np.hypot(Q[:, 0] - cx, Q[:, 1] - cy); i = int(np.argmin(dd))
            j = min(i + 4, len(Q) - 1)
            if dd[i] > REPLAN_D_M or j == i or abs(wrap(math.atan2(Q[j, 1] - Q[i, 1], Q[j, 0] - Q[i, 0]) - cyaw)) > PLAN_HE_RAD:
                return None
            last[0] = q[1]
            return extend(Q[i:], Qv[i:], lane, lane.project(cx, cy, s0 + 30.0, 60.0)[0], v, sec)
    out, fl = rollout(path, vp, x, y, yaw, v, steer, objs, sec, gt, rp)
    return REASONS.index(why), src, True, out, dbg, np.array([fl["yield0"], fl["aeb0"], fl["yield_on"], fl["aeb_on"], fl["sf_on"]])


def label_episode(ep, rec_dir, lane, gt, conv, actual=False, jobs=1, replan=True):
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
    for j in np.flatnonzero(ok_hist):
        V[j, :N_HIST], YAW[j, :N_HIST] = np.interp(th[j], G[:, 0], G[:, 4]), np.interp(th[j], G[:, 0], G[:, 3])
        Va[j], YAWa[j] = V[j], YAW[j]
        Va[j, N_HIST:], YAWa[j, N_HIST:] = np.interp(tf[j], G[:, 0], G[:, 4]), np.interp(tf[j], G[:, 0], G[:, 3])
    rho = float(np.polyfit(ts, tw, 1)[0]) if len(ts) > 10 else 1.0   # wall s per simulator s
    _C.update(steps=steps, G=G, rec=rec, lane=lane, gt=gt, tw=tw, ts=ts, ok_hist=ok_hist, rho=rho, replan=replan)
    if jobs > 1:
        import multiprocessing as mp
        with mp.get_context("fork").Pool(jobs) as pool:
            res = pool.map(one, range(N), chunksize=16)
    else:
        res = [one(j) for j in range(N)]
    reason = np.array([r[0] for r in res], np.int8); source = np.array([r[1] for r in res], np.int8)
    valid = np.array([r[2] for r in res], bool); dbg = np.stack([r[4] for r in res]); flags = np.stack([r[5] for r in res])
    roll = np.full((N, N_FUT, 4), np.nan)
    for j, r in enumerate(res):
        if r[3] is not None:
            roll[j] = r[3]
            x0yaw = rec.ego_at(tw[j])[2]
            V[j, N_HIST:] = r[3][:, 3]
            YAW[j, N_HIST:] = YAW[j, N_HIST - 1] + np.unwrap(np.r_[x0yaw, r[3][:, 2]])[1:] - x0yaw
    ak = np.zeros((N, N_FUT, 2), np.float32); fxy = np.full((N, N_FUT, 2), np.nan)
    if valid.any():
        ak[valid], fxy[valid] = conv(V[valid], YAW[valid])
    np.savez(ep / "dagger_labels.npz", step=np.arange(N, dtype=np.int32), ak=ak,
             valid=np.repeat(valid[:, None], N_FUT, 1), source=source)
    extra = {}
    if actual and ok_act.any():                       # the drive that really happened (expert-drive accuracy runs)
        extra["ak_actual"], extra["fxy_actual"] = np.zeros((N, N_FUT, 2), np.float32), np.full((N, N_FUT, 2), np.nan)
        extra["ak_actual"][ok_act], extra["fxy_actual"][ok_act] = conv(Va[ok_act], YAWa[ok_act])
    np.savez(ep / "dagger_debug.npz", t_wall=tw, t_sim=ts, rollout=roll, fxy=fxy, dbg=dbg, reason=reason, flags=flags,
             ok_actual=ok_act, reasons=np.array(REASONS), dbg_cols=np.array(["plan_age", "plan_d0", "plan_he", "d_ego", "yaw_err",
             "d_plan_30m", "route_s", "v", "scorer_s"]), flag_cols=np.array(["yield0", "aeb0", "yield", "aeb", "speed_filter"]), **extra)
    return dict(episode=str(ep), n=N, valid=int(valid.sum()), source1=int((source == 1).sum()),
                reasons={r: int((reason == i).sum()) for i, r in enumerate(REASONS) if (reason == i).any()})


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
    ap.add_argument("--no-replan", action="store_true", help="the plan at t0 only (no later planner output)")
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    if a.selftest:
        return selftest()
    cfg = json.load(open(a.config))["action_space_cfg"] if a.config else dict(
        accel_mean=0.02902694707164455, accel_std=0.6810426736454882, curvature_mean=0.0002692167976330542,
        curvature_std=0.026148280660833106, a_lambda=1e-4, a_ridge=1e-4, kappa_lambda=1e-4, kappa_ridge=1e-4,
        theta_lambda=1e-6, theta_ridge=1e-8, v_lambda=1e-6, v_ridge=1e-4, dt=0.1, n_waypoints=64)   # train/tests/test_data_math.py
    conv, lane, gt = Converter(cfg, a.alpamayo), Lane(a.route), load_gt(a.gt)
    for ep in a.episodes:
        print(json.dumps(label_episode(Path(ep), a.rec, lane, gt, conv, a.actual, a.jobs, not a.no_replan)), flush=True)


if __name__ == "__main__":
    main()
