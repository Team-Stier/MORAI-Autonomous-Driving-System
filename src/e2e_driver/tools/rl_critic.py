"""Privileged critic input for asymmetric PPO (user 10/27): simulator ground truth, for training only, never deployed.
One vector per decision in the FIELDS order (fixed; SI units: m, s, m/s, m/s^2, rad); never NaN: a missing car gives
gap GAP0, rel_v 0, ttc TTC0, a missing lane 0 / LANE_W. New fields (full course: next stop line, light state and time
left, pedestrians) go at the END so the existing indices keep their meaning.
Sources (rl_episode.py, 20 Hz on simulator time): Ego GT from gRPC GetActorState (base_link pose, speed), route s on the
scorer's path, MORAI Object Info (UDP 7505 + world offset: id, type, bbox centre x y, length; NPC vehicles only, so the
unscaled-size issue of scaled props does not apply) and the scorer's map (lanelets, judged lane lines).
- a, yaw_rate and every car's speed: differences over DIFF_S of simulator time.
- lanes: the lanelet under the Ego base_link whose driving direction is within 45 deg of the heading; its centreline
  (mean of the two bounds) gives lane_offset (left +), heading_err and lane_width; has_left/right_lane: a lanelet of the
  same direction one lane width to that side.
- cars: own / left / right lane by their route offset relative to the Ego lane centre (the route path runs parallel to
  the lanes on the 고주로; the full course will want lanelet adjacency instead), nearest ahead and behind by route s;
  gap = bumper to bumper, rel_v = car speed along the route - Ego speed, ttc = gap / closing speed (TTC0 if opening).
- penalty state (the scorer's rules, live): lane_touch_s = continuous contact of a wheel with judged paint outside the
  GPS shadow (+5 s per 3 s), over_limit_s = continuous time above the limit outside the exemption (+15 s, again per 3 s).
- prev_a, prev_kappa: the previous decision's first plan step (engine response accel[0], kappa[0]; 0 for the first).
"""
import math
import xml.etree.ElementTree as ET
from collections import deque
from pathlib import Path

import numpy as np

GAP0, TTC0, LANE_W, DIFF_S = 200.0, 100.0, 3.5, 0.3
EGO_LEN, EGO_CENTRE = 4.64, 1.5     # Ioniq5 length; bbox centre ahead of base_link (eval_manager Sim.ego)
FIELDS = (["v", "a", "yaw_rate", "s", "d_exit", "d_goal", "in_exempt", "lane_offset", "heading_err", "lane_width",
           "has_left_lane", "has_right_lane"]
          + [f"{lane}_{side}_{q}" for lane in ("own", "left", "right") for side in ("front", "rear") for q in ("gap", "rel_v", "ttc")]
          + ["lane_touch_s", "over_limit_s", "prev_a", "prev_kappa"])


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def _resample(line, n):
    s = np.r_[0.0, np.cumsum(np.hypot(*np.diff(line, axis=0).T))]
    u = np.linspace(0.0, s[-1], n)
    return np.c_[np.interp(u, s, line[:, 0]), np.interp(u, s, line[:, 1])]


class Lanes:
    """lanelet centrelines (driving direction) of the scorer's osm, as segments with their lane width."""

    def __init__(self, cfg):
        m = cfg["map"]
        root = ET.parse((Path(cfg["_path"]).parent / m["dir"]).resolve() / m["osm"]).getroot()
        tag = lambda e: {t.get("k"): t.get("v") for t in e.findall("tag")}
        nodes = {n.get("id"): (float(tag(n)["local_x"]), float(tag(n)["local_y"])) for n in root.iter("node")}
        ways = {w.get("id"): np.array([nodes[r.get("ref")] for r in w.findall("nd")]) for w in root.iter("way")}
        a, b, w = [], [], []
        for rel in root.iter("relation"):
            if tag(rel).get("type") != "lanelet":
                continue
            bd = {mb.get("role"): ways[mb.get("ref")] for mb in rel.findall("member") if mb.get("role") in ("left", "right")}
            left, right = bd["left"], bd["right"]
            if np.hypot(*(left[0] - right[0])) > np.hypot(*(left[0] - right[-1])):
                right = right[::-1]
            n = 2 + int(max(np.hypot(*np.diff(left, axis=0).T).sum(), np.hypot(*np.diff(right, axis=0).T).sum()) / 2.0)
            L, R = _resample(left, n), _resample(right, n)
            c, t = (L + R) / 2, (L + R)[-1] / 2 - (L + R)[0] / 2
            if t[0] * (L - R).mean(0)[1] - t[1] * (L - R).mean(0)[0] < 0:     # the left bound lies left of the direction
                c, L, R = c[::-1], L[::-1], R[::-1]
            a.append(c[:-1]); b.append(c[1:]); w.append(np.hypot(*(L - R).T)[:-1])
        self.a, self.b, self.w = np.concatenate(a), np.concatenate(b), np.concatenate(w)
        self.h = np.arctan2(*(self.b - self.a)[:, ::-1].T)
        self.cell = {}
        for i, (p, q) in enumerate(zip(self.a, self.b)):
            for cx in range(int(min(p[0], q[0]) // 5), int(max(p[0], q[0]) // 5) + 1):
                for cy in range(int(min(p[1], q[1]) // 5), int(max(p[1], q[1]) // 5) + 1):
                    self.cell.setdefault((cx, cy), []).append(i)

    def at(self, x, y, yaw):
        """-> (offset left +, heading error, width) on the aligned lane under (x, y), or None."""
        idx = np.array(sorted({i for dx in (-1, 0, 1) for dy in (-1, 0, 1) for i in self.cell.get((int(x // 5) + dx, int(y // 5) + dy), ())}), int)
        if not len(idx):
            return None
        idx = idx[np.cos(self.h[idx] - yaw) > math.cos(math.radians(45))]
        if not len(idx):
            return None
        a, ab = self.a[idx], self.b[idx] - self.a[idx]
        u = np.clip(((np.array([x, y]) - a) * ab).sum(1) / np.maximum((ab * ab).sum(1), 1e-9), 0, 1)
        r = np.array([x, y]) - (a + u[:, None] * ab)
        d = np.hypot(*r.T)
        j = int(d.argmin())
        if d[j] > self.w[idx[j]] / 2 + 0.3:
            return None
        side = ab[j, 0] * r[j, 1] - ab[j, 1] * r[j, 0]
        return math.copysign(d[j], side), _wrap(yaw - self.h[idx[j]]), float(self.w[idx[j]])


class Critic:
    """samples the privileged state at 20 Hz (step) and gives the vectors at decision times (at)."""

    def __init__(self, kmap, cfg, lanes, s_exit, s_goal):
        self.km, self.cfg, self.lanes, self.s_exit, self.s_goal = kmap, cfg, lanes, s_exit, s_goal
        self.wheels = np.array(list(cfg["vehicle"]["wheels_base_link_m"].values()))
        self.exempt = kmap.speed_exempt_s
        self.rows, self.ego_hist, self.car_hist = [], deque(), {}
        self.touch0 = self.over0 = None

    def route(self, pts, i):
        """route s and signed offset (left +) of points, searched in the path window around vertex i."""
        P = self.km.path
        lo, hi = max(i - 600, 0), min(i + 600, len(P) - 1)
        a, ab = P[lo:hi], P[lo + 1:hi + 1] - P[lo:hi]
        out = []
        for p in pts:
            u = np.clip(((p - a) * ab).sum(1) / np.maximum((ab * ab).sum(1), 1e-9), 0, 1)
            r = p - (a + u[:, None] * ab)
            j = int(np.hypot(*r.T).argmin())
            out.append((self.km.s[lo + j] + u[j] * np.hypot(*ab[j]), math.copysign(np.hypot(*r[j]), ab[j, 0] * r[j, 1] - ab[j, 1] * r[j, 0])))
        return out

    def step(self, tau, x, y, yaw, v, s, i, objects, shade):
        """objects: Object Info records (id, type, x, y, z, heading, length, width, height)."""
        eh = self.ego_hist
        eh.append((tau, v, yaw))
        while eh[0][0] < tau - DIFF_S:
            eh.popleft()
        dt = tau - eh[0][0]
        acc, yr = ((v - eh[0][1]) / dt, _wrap(yaw - eh[0][2]) / dt) if dt > 0.05 else (0.0, 0.0)
        lane = self.lanes.at(x, y, yaw)
        off, herr, width = lane if lane else (0.0, 0.0, LANE_W)
        c, sn = math.cos(yaw), math.sin(yaw)
        side = [self.lanes.at(x - sn * k * width, y + c * k * width, yaw) is not None for k in (1, -1)]
        vec = [v, acc, yr, s, self.s_exit - s, self.s_goal - s, float(self.exempt[0] <= s <= self.exempt[1]), off, herr, width] + side
        cars = [o for o in objects if o[1] not in (-1, 0, 2)]           # Ego record, pedestrians, obstacles out
        centre = np.array([x + c * EGO_CENTRE, y + sn * EGO_CENTRE])
        fr = self.route([centre] + [np.array(o[2:4]) for o in cars], i)
        (se, de), near = fr[0], {}
        for o, (so, do) in zip(cars, fr[1:]):
            hist = self.car_hist.setdefault(o[0], deque())
            hist.append((tau, so))
            while hist[0][0] < tau - DIFF_S:
                hist.popleft()
            vo = (so - hist[0][1]) / (tau - hist[0][0]) if tau - hist[0][0] > 0.05 else v
            k = round((do - (de - off)) / width)
            if k not in (0, 1, -1) or abs(so - se) > GAP0:
                continue
            ahead = so >= se
            gap = max(abs(so - se) - (EGO_LEN + o[6]) / 2, 0.0)
            key = ({0: "own", 1: "left", -1: "right"}[k], "front" if ahead else "rear")
            if key not in near or gap < near[key][0]:
                closing = (v - vo) if ahead else (vo - v)
                near[key] = (gap, vo - v, gap / closing if closing > 0.1 else TTC0)
        for lname in ("own", "left", "right"):
            for sname in ("front", "rear"):
                vec += list(near.get((lname, sname), (GAP0, 0.0, TTC0)))
        w = np.c_[x + c * self.wheels[:, 0] - sn * self.wheels[:, 1], y + sn * self.wheels[:, 0] + c * self.wheels[:, 1]]
        touch = not shade and bool(self.km.near_line(w, self.cfg["vehicle"]["tire_half_width_m"]).any())
        over = abs(v) * 3.6 > self.cfg["speed"]["limit_kph"] and not self.exempt[0] <= s <= self.exempt[1]
        self.touch0 = (self.touch0 if self.touch0 is not None else tau) if touch else None
        self.over0 = (self.over0 if self.over0 is not None else tau) if over else None
        vec += [tau - self.touch0 if touch else 0.0, tau - self.over0 if over else 0.0]
        self.rows.append((tau, vec))

    def at(self, t_dec, prev):
        """vectors (lists, FIELDS order) at decision times (simulator s): the newest 20 Hz sample at or before each, +
        prev [(a, kappa)] per decision."""
        ts = np.array([t for t, _ in self.rows])
        out = []
        for t, (pa, pk) in zip(t_dec, prev):
            j = max(int(np.searchsorted(ts, t, side="right")) - 1, 0)
            out.append([round(float(x), 4) for x in np.nan_to_num(self.rows[j][1] + [pa, pk], nan=0.0, posinf=TTC0, neginf=-TTC0)])
        return out


assert len(FIELDS) == 34, len(FIELDS)
