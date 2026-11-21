#!/usr/bin/env python3
"""Random lap scenes for closed-loop evaluation, from the collection scene (Alpamayo-Cosmos DATA_PLAN.md §6, DATA_PLAN_R2.md §4).
Copy of the collection skill's scripts/wsl/scenario-variants.py (sha256 50c7e3e1, 2026-10-26); eval_manager.py makes
one lap scene per lap with it (no checkpoints csv: the Ego keeps the base scene's START spawn). Eval changes (10/27): a
pedestrian's standardPos follows its new position (in the sample standardPos == initPos), a moved pedestrian gets its own
copy of the sample's waypoint path (without it the 10/27 laps' pedestrian never walked), and spawn points keep the base
scene's vehicle type.

Usage: scenario-variants.py <scenario dir> <base> <count> <seed> <global path txt> [prefix] [checkpoints csv] [starts]
       scenario-variants.py pool <scenario dir> <base> <seed> <global path txt> <name>
pool (continuous laps, user 10/29): the run's one scene, never reloaded: <name>.json, EgoNetwork/<name>_MN.json and
<name>-pool.json (pool_scene). Each lap the director (world_director.py) puts the Ego back at START and lays the pool's
objects out over gRPC SetTransform from layout(), which draws make()'s obstacle distributions.
With a checkpoints csv the Ego starts at a checkpoint just before a mission (section episodes, E1), 30% of them
offset laterally +-1 m and in heading +-8 deg (spawn-offset recovery, R2).
starts ("row:weight,...", e.g. "2:1,14:1") replaces the STARTS mix below.
Writes <prefix><seed>_<k>.json and EgoNetwork/<prefix><seed>_<k>_MN.json (a copy of the base's network file, so
each simulator copy keeps its own ports) plus <prefix><seed>-manifest.json with every sampled parameter.
The same seed gives the same scenarios on both simulator copies.

Randomized (v2, 10/29: one distribution for lap videos, sync eval and the real-time actors; every case the competition
sources allow is drawn uniformly, worst cases included; numbers they do not give are wide assumptions):
- signals: each light's elapsed time in its current state (arrival phase; the venue's lights react to the Ego, 10/27)
- NPC spawn points (user 10/29): each car's type at random (-1 over every length class, MORAI's per-car draw), each
  interval uniform in 3-10 s (roundabout) / 5-10 s (highway; env NPC_PERIOD: eval_manager --density), the count following
  from them; desired speed 10-35 kph per ring point, 50-100 kph per highway point (MORAI Q&A: fixed within a lane);
  lateral bias, safety distance, TTC per point; no lane change (10/19). pool_scene() has none (world_director spawns)
- static obstacles: 1-2 in the rules' static box (s 280-390) and, in a tunnel with both kinds, 1-2 there; every object
  asset uniform, position uniform over the box's whole lane area (both directions, the junction included; the tunnel's
  two lanes) and yaw uniform, footprint on the road, apart from each other, the road never shut (user 10/29: MORAI's 7/23
  plan to keep them passable without crossing a line is not applied); height 1-2.5x for small props, sometimes floating
- dynamic obstacles: 1-2 in the dynamic box (s 421-503) and 1-2 in the tunnel, every movable asset uniform (pedestrian-type
  and props on a waypoint path, 10/6; no car prop in the one-lane tunnel), crossing left-right / right-left / from just past
  the centre line / from behind a parked car uniform where possible; timing must stop / pass in front 85/15 % (the
  real-time actors' 10/28 preset) with the trigger distance from it, never so short that the object enters the lane after
  the Ego has passed; 3-12 kph (car props 10-30)
- tunnel: dynamic / static + dynamic 60/40 % (the real-time actors' preset) over the whole tunnel (s 1880-1996, both lanes)
- GPS shadow area: shifted +-15 m along the road
The sample's vehicleList (9 NPCs its spawn points had made when it was saved) is dropped. Not randomized here: weather
and time (eval_manager / rl_episode, rules §2-7-2). SCENE_PRESET is ignored (v2 has one distribution).
Map coordinates of the scenario equal the driver's map frame (sample Ego start = route start).
"""
import copy
import json
import math
import os
import random
import sys

VEHICLES = [30100004, 30100005, 30100010, 30100014, 30200003, 30300002, 30400004, 30700003]  # IDs seen in saved scenes
# Obstacle DataIDs that loaded in the 2026-10-20 probe (probe-assets.py), by unit-scale bbox (m). Vehicle-sized ones
# keep scale 1. shortcut: only the 20 Object Info records nearest the start were read; probe the rest in batches.
SMALL = [40100001, 40100002, 40100003, 40100004, 40100005, 40100026, 40100027, 40100028, 40100029, 40100033,
         40100058, 40100059]   # boxes 0.75/1.2, drums, barrels, bollards, barricades 0.6x1.35, concrete barrier 1x3
VEHICLE = [40100008, 40100009, 40100010, 40100011]   # parked-car/truck obstacles 4.6-6.6 m


class Route:
    def __init__(self, path_txt):
        self.p = [tuple(map(float, l.split()[:3])) for l in open(path_txt) if l.strip()]
        self.s = [0.0]
        for a, b in zip(self.p, self.p[1:]):
            self.s.append(self.s[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))

    def project(self, x, y):
        i = min(range(len(self.p) - 1), key=lambda k: (self.p[k][0] - x) ** 2 + (self.p[k][1] - y) ** 2)
        (x0, y0, _), (x1, y1, _) = self.p[i], self.p[i + 1]
        h = math.atan2(y1 - y0, x1 - x0)
        dx, dy = x - x0, y - y0
        return self.s[i] + dx * math.cos(h) + dy * math.sin(h), -dx * math.sin(h) + dy * math.cos(h)

    def at(self, s, lat):
        i = max(0, min(len(self.s) - 2, next((k for k in range(len(self.s) - 1) if self.s[k + 1] >= s), len(self.s) - 2)))
        (x0, y0, z0), (x1, y1, z1) = self.p[i], self.p[i + 1]
        h = math.atan2(y1 - y0, x1 - x0)
        u = (s - self.s[i]) / max(self.s[i + 1] - self.s[i], 1e-6)
        x, y, z = x0 + u * (x1 - x0), y0 + u * (y1 - y0), z0 + u * (z1 - z0)
        return x - lat * math.sin(h), y + lat * math.cos(h), z, math.degrees(h)


def vec(x, y, z):
    return {"x": x, "y": y, "z": z, "_x": f"{x:.3f}", "_y": f"{y:.3f}", "_z": f"{z:.3f}"}


def rot(yaw):
    return {"roll": "0.000", "pitch": "0.000", "yaw": f"{(yaw + 180) % 360 - 180:.3f}"}


# checkpoints.csv row -> weight: rows just before missions (signals, static/dynamic obstacles, left turn,
# roundabout, highway merge, tunnel). Row 14 is left out: its spawn gets GPS (0,0) (MORAI reply 2026-10-15). rs2041 (10/23,
# rows 2 and 14 only) found no blackout while parked at row 14; the shade starts 0.7-5 s after departure.
# Row 9 (roundabout exit) is left out: both runs from it drove the route backwards (run1020c, 2026-10-20).
# Weights follow the ledger's shortfall: traj_ok seconds per 180 s episode by start row (run1020o-run1021i, the stable
# driver) fed a max-min LP over the cell deficits. 10/21 16:15 (C1 3.4 h, C2 3.3, C4 2.0, C5 2.0, C6 0.9 short; C0, C3
# done): row 3 (C2), row 1 (C1), row 8 (C4/C5/C6, the cheapest C4), with rv laps at 25% of episodes (SCENES: rv x1 + rs x4).
# 10/21 23:40: row k starts section k+1, so row 7 (s 635.1) starts C0 section 8, not signal section 7: measured C0 1.0,
# C4 0.33 min/ep. Row 8 (ring entry) measured C4 0.45, C5 1.18, C6 0.66 on 2 sims, the best C4 per sim-second; the LP on
# measured yields gives rv 24 / row 8 53 / row 1 23. Row 6 (s 580.9, signal section 7 then the ring) at 10% to measure it.
# 10/22 03:20 (C1 1.8 h, C2 1.45, C4 1.3 short; C5/C6 nearly done): LP on rs2039 yields (2 sims) gives row 8 63 / row 1 26 /
# row 3 11 and no rv laps (8.7 h vs 12.1 h for the 03:40 mix). SCENES = rs only from run1021v.
STARTS = {8: 60, 1: 25, 3: 15}


_DECK = []


def start_at(d, rng, cps, meta):
    # Stratified draw: shuffled decks of the weights (reduced by their gcd), so 60 scenes hit the mix exactly instead
    # of rng.choices drifting (rs2029/rs2032 drew 18-22% row 3 for a 40% weight).
    if not _DECK:
        g = math.gcd(*STARTS.values())
        _DECK.extend(row for row, w in STARTS.items() for _ in range(w // g))
        rng.shuffle(_DECK)
    row = _DECK.pop()
    x, y, z, yaw = cps[row]
    # +-0.6 m: at +-1 m a wheel sat on the lane line through the 8 s standstill before driving (+10-15 s lane penalty)
    lat, dyaw = (rng.uniform(-0.6, 0.6), rng.uniform(-8, 8)) if rng.random() < 0.3 else (0.0, 0.0)
    h = math.radians(yaw)
    ego = d["egoVehicle"]["initPosition"]
    ego["pos"] = vec(x - lat * math.sin(h), y + lat * math.cos(h), z + 0.35)   # sample start sits 0.35 m above the route z
    ego["rot"]["yaw"] = f"{yaw + dyaw:.3f}"
    meta["start"] = dict(checkpoint=row, lat=round(lat, 2), dyaw=round(dyaw, 1))


# ----- dynamic objects: the R2 collection generator's (DATA_PLAN_R2.md §4 A/B; skill scenario-variants.py sha256
# d4a7eda2, 10/27), as a lap needs them; v2 (10/29) draws the assets, shapes and timing below (r2_dynamic)
PERSONS = [*range(50100001, 50100019), 50100055, 50100058, 50100074, 50100077, *range(50200001, 50200006)]
# deer, dogs, person with pet, cyclist. The cyclist 50100084 reports a 0 x 0 x 0 box; the driver's Object Info relay
# dropped whole packets for it (r2c3-w1-015), so R2 stacks receive Object Info in gt-objects-filter-node.py (default box).
OTHER_PEDS = [50300001, 50300002, 50300003, 50300018, 50100084]
OTHER_PROPS = [40100059, 40100060, 40100062, 40100063, 40200011, 40200012, 40200013, 40200014, 40200015, 40200016,
               40200017, 40200018, 40200019, 40200020, 40200021]   # bicycles, scooters, strollers on a waypoint path
CAR_PROPS = [40100008, 40100010, 40100011, 40100012, 40100013, 40100018, 40100019, 40100020, 40100021, 40100022,
             40100025]   # car-shaped props: moving (cut-in/crossing NPC) or parked; vehicleList NPCs have no trigger
PROP_HALF_L = {40100008: 2.51, 40100010: 2.51, 40100011: 2.32, 40100012: 2.35, 40100013: 2.43, 40100018: 2.42,
               40100019: 2.48, 40100020: 2.17, 40100021: 2.4, 40100022: 2.13, 40100025: 2.19}
# Road cross-sections from kcity_lanelet2.osm (lat + = left of the route, 2026-10-27):
#   ra s 470-500: right curb -5.6, right lane [-5.6, -1.93], route lane [-1.93, +1.93] (yellow), opposite to +9.4
#   rb tunnel s 1880-1996: route lane [-1.74, +1.74] (yellow), opposite lane to +5.3 (wall line)
#   rd starts: right edge, near/far yellow (None: the route lane has same-direction lanes on its left)
A_CURB, B_CURB = (-5.6, 9.4), (-1.75, 5.3)
# Episodes end ~60 m (6.4 s of targets) past the last mission frame: curve end + 60, tunnel exit 1996 + 74, A s 500 + 60.
D_STARTS = {0: dict(edge=-2.03, yellow=(2.0, 3.01), end=180.0), 1: dict(edge=-3.1, yellow=(2.06, 3.07), end=255.0),
            4: dict(edge=-1.9, yellow=None, end=500.0), 5: dict(edge=-6.07, yellow=(1.93, 1.93), end=655.0),
            7: dict(edge=-5.67, yellow=(1.8, 2.75), end=745.0)}
# Left limits from the pilot sweep (r2p2, standing starts): the driver never moved from CP4 +4.5 or CP7 +2.74/+4.0/+5.0
# (its planner keeps the Ego inside the route corridor); CP0/1/5 recovered from +4.5-5.0 and every right-side start.
D_LEFT_MAX = {0: 5.0, 1: 5.0, 4: 3.5, 5: 5.0, 7: 1.2}   # CP7 +1.7 m (24.9 deg) on the yellow line never started either
R2_END = {"ra": 560.0, "rb": 2070.0}
# Ego speed assumed when sizing the trigger distance: the policy's there (10/28 shadow runs c-f, median 7.2-8.5 m/s at
# s 430-500, 13-15.6 in the tunnel; the real-time actors' preset)
V_NOMINAL = {"ra": 8.5, "rb": 14.0}
# Trigger distance cap. The plan's 5-25 m (official sample 15) cannot time a far-side (+9.4 m curb) crossing into the
# route lane at 3-12 kph, so the cap is 45 m (logged decision, 2026-10-27).
ACTIVE_MAX = 45.0
ACTIVE_MIN = 8.0   # a 5 m trigger beside the lane never fired as the Ego passed (r2p2-w1-002)


def pick(rng, table):
    r, acc = rng.random(), 0.0
    for k, w in table:
        acc += w
        if r < acc:
            return k
    return table[-1][0]


# The Ego's time to the crossing (s): must stop, when the object enters its corridor; pass in front, when it has left it
# 10/29 user: from the 10:25 videos' leniency (must_stop 0.5-1.25, pass_front 0.15-0.6) to the 10:43 ones' abruptness
# (0.3-0.9, 0.1-0.4), random in between
LEAD = {"must_stop": (0.3, 1.25), "pass_front": (0.1, 0.6)}
RT_KEYS = ("z", "yaw_w", "path", "corr_in_m", "corr_out_m", "s_cross")   # layout()'s fields, not in make()'s manifest


def r2_actor(d, rng, route, uid, kind, s0, side, ptype, timing, curb, v_e, sample_ped, sample_obj, cls, data_id, as_ped):
    """One moving object, asset data_id (a pedestrian-type one if as_ped, else a prop on an ObstaclePath) of class cls
    (person/other/car: car-shaped props). side: +1 starts left of the route, -1 right. ptype from_centre: from just past
    the centre line in the opposite lane (its near edge 0.1-1 m beyond it, user 10/29) across the route lane to the right."""
    is_car = cls == "car"
    # a dash, not a stroll: the official sample's pedestrian runs 10 km/h from 3.5 m off the lane once the Ego is 15 m
    # away (0.8 s to the lane); user 10/29: at least that abrupt
    kph = rng.uniform(10, 30) if is_car else rng.uniform(10, 25)   # 10:25 videos 10-20, 10:43 ones 12-25
    v_o = kph / 3.6
    half_w = 1.05 if is_car else 0.35
    # from the curb: 0-0.8 m (10:43 videos, little time in view) to 0-2 m of sidewalk (10:25 ones; the tunnel wall 0.8)
    off = rng.uniform(0, 2 if kind == "ra" else 0.8)
    lat0 = (curb[1] + off if side > 0 else curb[0] - off) if not is_car else (curb[1] - 1.5 if side > 0 else curb[0] + 1.6)
    s_start, extra = s0, []
    if ptype == "from_centre":
        lat0, ptype = CENTRE[kind] + half_w + rng.uniform(0.1, 1.0), "crossing"
    if ptype == "along":                            # along the roadside (cars: along the next lane), no conflict
        lat0 = curb[1] + off + 0.5 if side > 0 else curb[0] - off - 0.5
        length = rng.uniform(4, 12)
        s_end, lat_end = s0 + rng.choice((-1, 1)) * length, lat0
        if is_car:   # a car that stopped after 10 m at the intersection edge was hit (r2c3-w1-096): keep driving
            if curb[0] < -3:   # next lane, same way, from past the intersection area to beyond the episode end
                lat0 = lat_end = curb[0] + 1.85
                s_start = s0 = max(s0, 472.0)
                s_end = R2_END[kind] + rng.uniform(15, 30)
            else:              # tunnel: oncoming traffic in the oncoming lane
                lat0 = lat_end = 3.5
                s_end = s0 - rng.uniform(40, 60)
    elif ptype == "curb_stop":                      # walks up to the curb along the sidewalk and stays there
        out = (1.0, 2.0) if kind == "ra" else (0.4, 0.8)   # tunnel: the walkway along the wall
        lat0 = curb[1] + rng.uniform(*out) if side > 0 else curb[0] - rng.uniform(*out)
        lat_end = curb[1] + rng.uniform(0.0, 0.3) if side > 0 else curb[0] - rng.uniform(0.0, 0.3)
        s_start, s_end = s0 - rng.choice((-1, 1)) * rng.uniform(3.0, 6.0), s0
    elif ptype == "short_cross":                    # into the road, stopping 1.5 m short of the route lane
        lat_end = rng.uniform(3.45, min(6.0, curb[1] - 0.3)) if side > 0 else rng.uniform(max(-4.6, curb[0] + 0.3), -3.45)
        s_end = s0 + rng.uniform(-1.5, 1.5)
    elif ptype == "enter_stop":                     # walks into the route lane and stays
        lat_end = rng.uniform(-1.2, 1.2)
        s_end = s0
    elif ptype == "cutin" :                         # car: from the next lane into the route lane ahead of the Ego,
        lat0 = -3.75 if curb[0] < -3 else 3.7       # along it, then back out (one that stopped in the lane blocked the
        s_start = s0                                # tunnel for 53-60 s: r2p2-w1-011/015)
        s_in, lat_in = s0 + rng.uniform(8, 14), rng.uniform(-0.6, 0.6)
        # It stays in the route lane past the episode end: one that pulled back out stopped askew beside the waiting
        # Ego and touched it 10 times (r2c3-w1-020). The Ego follows it (GT yield node, follow mode).
        extra = [(s_in, lat_in)]
        s_end, lat_end = R2_END[kind] + rng.uniform(15, 30), lat_in
    else:                                           # crossing (also from behind a parked car)
        # Cross to the far curb (tunnel: wall). One left standing in a lane blocked the driver's bypass and the GT AEB held
        # the Ego for 58-78 s (r2p1-w2-002, r2p2-w1-012); crossing the whole road is 9-17 m, beyond the plan's 4-12 m.
        far = 1.0 if kind == "ra" else 0.8
        lat_end = curb[0] - rng.uniform(0.3, far) if side > 0 else curb[1] + rng.uniform(0.3, far)
        s_end = s0 + rng.uniform(-1.5, 1.5)
    pts_sl = [(s_start, lat0)] + extra + [(s_end, lat_end)]
    length = sum(math.hypot(b_[0] - a_[0], b_[1] - a_[1]) for a_, b_ in zip(pts_sl, pts_sl[1:]))
    if ptype == "crossing" and length > 12 and side > 0:
        # shortcut: the far-side curb (+9.4 m) is 11-14 m from the route lane, more than the plan's 12 m; keep the
        # length the crossing needs rather than stop short of the lane
        pass
    # Trigger distance from the timing class (Ego assumed at v_e): t_in/t_out = when the object enters/leaves the Ego's
    # corridor (route lane +-1.2 m + object half width).
    corr = 1.2 + half_w
    first = pts_sl[1]   # the leg that enters the lane
    leg = math.hypot(first[0] - s_start, first[1] - lat0)
    d_lat = abs(first[1] - lat0)
    d_in = max(abs(lat0) - corr, 0.0) * (leg / d_lat if d_lat > 1e-3 else 1.0)
    if timing in ("must_stop", "pass_front") and not is_car:
        # The Ego covers at most ACTIVE_MAX at v_e before it reaches the object, so a far-side start must walk fast
        # enough to be in (must_stop) or through (pass_front) the lane by then; raise the speed within 3-12 kph.
        need = (d_in + (2 * corr if timing == "pass_front" else 0.0)) / (0.75 * ACTIVE_MAX / v_e)
        kph = min(max(kph, need * 3.6), 12.0)
        v_o = kph / 3.6
    t_in = d_in / v_o
    t_out = t_in + (2 * corr / v_o if ptype != "enter_stop" else 99.0)
    if timing == "no_conflict" and t_in < 2.5 and ptype not in ("curb_stop", "short_cross", "along"):
        timing = "passed"   # it reaches the lane too soon for the Ego to be through first
    if timing == "must_stop":
        t_e = t_in / rng.uniform(0.25, 0.75) if t_in > 0.3 else rng.uniform(*LEAD["must_stop"])
    elif timing == "pass_front":
        t_e = t_out + rng.uniform(*LEAD["pass_front"])
    elif timing == "passed":
        t_e = t_out + rng.uniform(2.5, 5.0)
    else:   # no_conflict: the Ego is past before the object reaches its lane
        t_e = max(t_in - rng.uniform(1.0, 3.0), 0.3)
    if ptype in ("along", "curb_stop", "short_cross"):
        active = rng.uniform(ACTIVE_MIN, 25)
    else:
        lon = v_e * t_e
        # Room to stop at ~3 m/s^2 plus the Ego's nose and a gap: a car triggered 8 m ahead of an Ego at 8 m/s was hit
        # (r2c3-b1-003). The "no conflict" class keeps its short trigger (the object reaches the lane after the Ego).
        safe = v_e * v_e / 6.0 + 8.0 if timing != "no_conflict" else ACTIVE_MIN
        active = min(max(math.hypot(lon, abs(lat0)), ACTIVE_MIN, safe), ACTIVE_MAX)
    if timing in ("must_stop", "pass_front"):
        # The Ego (on the route at v_e) is set off far enough back that the object is in the lane (must stop) or through
        # it (pass in front) before the Ego gets there, over the caps above: a far-side start triggered late entered the
        # lane after the Ego had passed (user 10/29), and one triggered nearer than its own offset never moved
        active = max(active, math.hypot(1.1 * v_e * (t_in if timing == "must_stop" else t_out) + 1.0, lat0) + 0.5)
    x, y, z, h = route.at(s_start, lat0)
    pts = [(x, y)]
    for s_, l_ in extra + [(s_end, lat_end)]:
        xx, yy, _, _ = route.at(s_, l_)
        pts.append((xx, yy))
    heading = math.degrees(math.atan2(pts[-1][1] - pts[0][1], pts[-1][0] - pts[0][0]))
    wps = [{"pos": vec(-px, z, -py), "rot": {"roll": "0.000", "pitch": "0.000", "yaw": "0.000"}} for px, py in pts]
    if as_ped:
        o = copy.deepcopy(sample_ped)
        o.update(DataID=data_id, UNIQUEID=uid, pos=vec(x, y, z), rot=rot(heading), initPos=vec(-x, z, -y),
                 standardPos=vec(-x, z, -y), activeDistance=active, speed=kph, movingDistance=length, loop=False,
                 isWaypointMode=True)
        d["pedestrianList"].append(o)
        d["waypointDataList"].append({"pathName": f"PedPath - {uid}", "waypointData": wps})
    else:
        o = copy.deepcopy(sample_obj)
        o.update(DataID=data_id, UNIQUEID=uid, pos=vec(x, y, z), rot=rot(heading), scale=vec(1.0, 1.0, 1.0),
                 isWaypointMode=True, moveSpeed=kph, activeDistance=active)
        d["objectList"].append(o)
        d["waypointDataList"].append({"pathName": f"ObstaclePath - {uid}", "waypointData": wps})
    # For the director (layout): where the path enters and leaves the corridor, as distances along the xy path it walks,
    # and the route s where it crosses the route line
    walked, inside, s_cross = 0.0, [], None
    for (sa, la), (sb, lb), pa, pb in zip(pts_sl, pts_sl[1:], pts, pts[1:]):
        seg = math.dist(pa, pb)
        if lb != la:
            u0, u1 = sorted(((-corr - la) / (lb - la), (corr - la) / (lb - la)))
            if s_cross is None and la * lb <= 0:
                s_cross = sa + la / (la - lb) * (sb - sa)
        else:
            u0, u1 = (0.0, 1.0) if abs(la) <= corr else (1.0, 0.0)
        if max(u0, 0.0) <= min(u1, 1.0):
            inside.append((walked + max(u0, 0.0) * seg, walked + min(u1, 1.0) * seg))
        walked += seg
    c_in, c_out = (inside[0][0], inside[-1][1]) if inside else (walked, walked)
    return dict(zone=f"{kind}_dynamic", cls=cls, id=data_id, ped=as_ped, ptype=ptype, timing=timing, s=round(s_start, 1),
                xy=[round(x, 2), round(y, 2)], xy_end=[round(pts[-1][0], 2), round(pts[-1][1], 2)],
                lat=round(lat0, 2), s_end=round(s_end, 1), lat_end=round(lat_end, 2), kph=round(kph, 1),
                active=round(active, 1), move=round(length, 1), t_in=round(t_in, 2), t_e=round(t_e, 2),
                z=round(z, 3), yaw_w=round(heading, 2), path=[[round(px, 2), round(py, 2)] for px, py in pts],
                corr_in_m=round(c_in, 2), corr_out_m=round(c_out, 2),
                s_cross=None if s_cross is None else round(s_cross, 1))


def r2_static(d, rng, route, uid, s, lat, sample_obj, data_id, tag, yaw_jitter=15.0):
    o = copy.deepcopy(sample_obj)
    x, y, z, h = route.at(s, lat)
    sz = 1.0 if data_id in VEHICLE or data_id in CAR_PROPS else rng.uniform(1, 2.5)
    yaw = h + rng.choice((0.0, 180.0)) + rng.uniform(-yaw_jitter, yaw_jitter)
    o.update(DataID=data_id, UNIQUEID=uid, pos=vec(x, y, z), rot=rot(yaw), scale=vec(1.0, 1.0, sz), isWaypointMode=True,
             moveSpeed=0.0, activeDistance=0.0)
    d["objectList"].append(o)
    d["waypointDataList"].append({"pathName": f"ObstaclePath - {uid}", "waypointData": []})
    return dict(zone=tag, id=data_id, s=round(s, 1), lat=round(lat, 2), scale_z=round(sz, 2), xy=[round(x, 2), round(y, 2)],
                z=round(z, 3), yaw_w=round((yaw + 180) % 360 - 180, 2))


# ----- v2 (user 10/29): one distribution, every case the competition sources allow drawn uniformly, worst cases included.
# Sources (Team-Stier morai-stier-codex-plugin 5c0b166): rules v1.2 §3-1 figure, §3-2 "장애물 회피 구간에서 임의 장애물";
# MORAI replies 10/6 (any F2 object asset
# in the boxes and the tunnel, static ones stay put, static assets may get a waypoint path), 10/15 (objects, positions and
# timing vary), 10/22 (only inside the figure's boxes), 10/27 (tunnel mix undisclosed). Counts are undisclosed.
GEN = "v2-20261029"
N_OBJ = (1, 2)   # obstacles per box and kind, uniform (user 10/29; the sources give no count)
# The §3-1 boxes on the route (the figure registered onto the map with START and the shadow box, +-5 m; the static box's
# T junction falls at s 326 and the dynamic box's crossroads at s 456, as on the map)
STATIC_BOX, DYN_BOX, TUNNEL_S = (280.0, 390.0), (421.0, 503.0), (1880.0, 1996.0)   # tunnel: its whole interior
# The lane area across the static box (kcity_lanelet2.osm, 10/29), from s on: right road edge, right edge of the lane area
# (s 276-318: the solid line left of a 1.5 m shoulder, then the road edge); its left edge is FAR_L (the opposite lanes'
# edge). s 316-346 is the junction. Static objects stand uniformly anywhere on it (user 10/29: the box's whole lane area,
# both directions; MORAI's 7/23 plan to keep them passable without crossing a line is not applied).
STATIC_SEC = [(276, -3.0, -1.5), (311, -2.9, -1.6), (316, -2.8, -1.7), (317, -2.9, -1.9), (318, -2.9, -2.4),
              (319, -2.9, -2.9), (320, -3.6, -3.6), (321, -4.5, -4.5), (322, -5.2, -5.2), (341, -5.5, -5.5),
              (342, -5.2, -4.4), (343, -3.6, -3.6), (344, -3.0, -3.0), (345, -2.5, -2.5), (346, -2.1, -2.1),
              (347, -1.8, -1.8), (348, -1.7, -1.7), (352, -1.6, -1.6), (357, -1.7, -1.7), (362, -1.8, -1.8),
              (366, -1.9, -1.9), (368, -2.0, -2.0), (369, -2.1, -2.1), (372, -2.5, -2.5), (375, -2.9, -2.9),
              (378, -3.2, -3.2), (381, -3.6, -3.6), (384, -4.0, -4.0), (387, -4.3, -4.3), (390, -4.7, -4.7)]
FAR_L = [(276, 4.9), (300, 7.1), (320, 10.2), (346, 13.0), (377, 16.5), (394, 16.5)]
STATIC_SPAN = (-5.5, 16.5)   # lateral bounds of the lane area: positions are drawn in this rectangle and kept on the area
# The Ego's own lane across the static box: the route's lanelet in kcity_lanelet2.osm (yellow centre line on its left),
# from s on: right bound, left bound (route frame, + left; 10/29). Static objects stand anywhere it holds them (user 10/29:
# the box's Ego lane, "enough overlap counts": a big box whose centre is outside still blocks it) -> in_ego_lane()
EGO_LANE = [(276, -1.49, 1.48), (311, -1.56, 1.55), (313, -1.6, 1.6), (315, -1.64, 1.66), (316, -1.71, 1.67),
            (318, -1.75, 1.66), (325, -1.75, 1.7), (333, -1.75, 1.74), (347, -1.82, 1.78), (348, -1.72, 1.73),
            (349, -1.66, 1.69), (351, -1.63, 1.65), (359, -1.69, 1.69), (361, -1.74, 1.73), (363, -1.78, 1.78),
            (365, -1.83, 1.83), (367, -1.85, 1.9), (369, -1.85, 1.98), (372, -1.93, 2.01), (373, -1.97, 2.01),
            (394, -1.97, 2.01)]
EGO_SPAN = (-4.5, 4.5)   # every centre in_ego_lane can accept: band edge 0.95 + a 6.55 m truck's half length 3.28
TUNNEL_LANE = (B_CURB[0], 1.74)   # the tunnel's Ego lane: right wall line to the yellow line (A_CURB / B_CURB comment)
TUNNEL_SEC, TUNNEL_SPAN = (B_CURB[0], B_CURB[0], B_CURB[1]), B_CURB   # both lanes, wall line to wall line
EGO_PASS = 1.892 + 2 * 0.05   # the Ego's body (rules §2-7) and 5 cm a side
CAR_SHAPED = sorted(set(VEHICLE + CAR_PROPS))
STATIC_POOL = sorted(set(SMALL + CAR_SHAPED + OTHER_PROPS))   # every object asset that loaded (38)
# Movers: the pedestrian-type assets only (people, animals, person with pet, cyclist; 32). MORAI (Discord Q&A, 10/06): no
# exceptions such as static objects moving in the dynamic zone -> no box, barricade or car prop crosses (user 10/29)
MOVERS = [(i, True) for i in sorted(set(PERSONS + OTHER_PEDS))]
PARK_S = ((421.0, 432.0), (472.0, 503.0))   # right lane free for a parked car (s 433-471: stop line and junction)
# (length, width) m from Object Info of the 10/28 eval laps; shortcut: class defaults (size_of) for assets not seen there
SIZE = {40100001: (0.75, 0.75), 40100002: (1.2, 1.2), 40100004: (0.56, 0.56), 40100005: (0.81, 0.81),
        40100008: (5.02, 2.07), 40100009: (6.55, 2.51), 40100010: (5.02, 1.91), 40100011: (4.64, 2.11),
        40100012: (4.69, 2.2), 40100013: (4.86, 2.12), 40100022: (4.25, 2.0), 40100026: (0.2, 0.2),
        40100027: (0.49, 0.49), 40100028: (0.6, 1.35), 40100029: (0.6, 1.35), 40100033: (1.08, 3.01),
        40200011: (1.22, 0.6), 40200017: (1.63, 0.6), 40200018: (1.21, 0.56)}
TIMINGS = [("must_stop", 0.85), ("pass_front", 0.15)]   # the real-time actors' (10/28 brake preset), user 10/29
TUNNEL = [("dynamic", 0.6), ("both", 0.4)]               # the same
CENTRE = {"ra": 1.93, "rb": 1.74}   # centre line (A_CURB / B_CURB comment): from_centre movers start just past it


def size_of(i):
    if i in SIZE:
        return SIZE[i]
    if i in CAR_SHAPED:
        return 2 * PROP_HALF_L.get(i, 2.5), 2.1
    return (0.4, 0.6) if i in PERSONS else (1.2, 1.2) if i in SMALL else (1.7, 0.6)


def foot(i, dyaw):
    """-> (half extent along the lane, half across) of asset i turned dyaw deg from the lane"""
    (ln, wd), c, s = size_of(i), abs(math.cos(math.radians(dyaw))), abs(math.sin(math.radians(dyaw)))
    return (ln * c + wd * s) / 2, (ln * s + wd * c) / 2


def static_sec(s):
    """-> (right road edge, right edge of the lane area, left road edge) at s"""
    row = next(r for r in reversed(STATIC_SEC) if r[0] <= s or r is STATIC_SEC[0])
    (s0, l0), (s1, l1) = next((a, b) for a, b in zip(FAR_L, FAR_L[1:]) if s <= b[0] or b is FAR_L[-1])
    return row[1], row[2], l0 + (l1 - l0) * min(max((s - s0) / (s1 - s0), 0.0), 1.0)


def ego_lane(s):
    """-> (right, left) bound of the Ego's lane at s (EGO_LANE, linear between rows)"""
    (s0, r0, l0), (s1, r1, l1) = next((a, b) for a, b in zip(EGO_LANE, EGO_LANE[1:]) if s <= b[0] or b is EGO_LANE[-1])
    f = min(max((s - s0) / (s1 - s0), 0.0), 1.0)
    return r0 + (r1 - r0) * f, l0 + (l1 - l0) * f


def in_ego_lane(s, box, lane=ego_lane):
    """a static footprint (s_lo, s_hi, lat_lo, lat_hi) counts as in the Ego's lane (lane(s) -> its bounds) at s: wholly
    inside it, or reaching the band the Ego's body sweeps driving down the lane's middle (it has to swerve or stop for it)"""
    r, l = lane(s)
    c = (r + l) / 2
    return r <= box[2] and box[3] <= l or (box[2] < c + EGO_PASS / 2 and box[3] > c - EGO_PASS / 2)


def overlap(a, b):
    return a[0] < b[1] and b[0] < a[1] and a[2] < b[3] and b[2] < a[3]


def passable(boxes, sec):
    """boxes [(s_lo, s_hi, lat_lo, lat_hi)] do not shut the road: at every s from 5 m before to 5 m after each (room to
    swerve) some gap across the whole road (lines ignored) fits the Ego. Checked at every whole s (sec changes there) and
    every window end (the boxes in play change there)."""
    ends = [e for c in boxes for e in (c[0] - 5.0, c[1] + 5.0)]
    for b in boxes:
        a_, b_ = b[0] - 5.0, b[1] + 5.0
        for q in [e for e in ends if a_ <= e <= b_] + list(range(math.ceil(a_), math.floor(b_) + 1)):
            edge, _, far = sec(q)
            gaps, free = [], edge
            for lo, hi in sorted((c[2], c[3]) for c in boxes if c[0] - 5.0 <= q <= c[1] + 5.0):
                gaps.append(min(lo, far) - free)
                free = max(free, hi)
            if max(gaps + [far - free]) < EGO_PASS:
                return False
    return True


def choose(rng, cat, xs):
    """make()'s asset draw: uniform with replacement (layout() passes one without replacement over its pool)"""
    return rng.choice(xs)


def place_static(d, rng, route, uid, tag, s_lo, s_hi, sec, span, sample_obj, info, boxes, draw=choose, keep=None):
    """One object asset (uniform) at a position uniform over the lane area (sec, inside the span rectangle; keep(s, box)
    instead when given) with a uniform yaw, its footprint on the road along its whole length, clear of the others, the
    road not shut (passable); the footprint goes to boxes. Left out when 400 tries found no such place."""
    data_id = draw(rng, "static", STATIC_POOL)
    for _ in range(400):
        s, lat, dyaw = rng.uniform(s_lo, s_hi), rng.uniform(*span), rng.uniform(-180.0, 180.0)
        edge, lane_r, far = sec(s)
        hl, hw = foot(data_id, dyaw)
        box = (s - hl, s + hl, lat - hw, lat + hw)
        road = [sec(q) for q in [s - hl, s + hl] + list(range(math.ceil(s - hl), math.floor(s + hl) + 1))]
        if (keep(s, box) if keep else lane_r <= lat <= far) and max(r[0] for r in road) <= box[2] and box[3] <= min(r[2] for r in road) \
                and not any(overlap(box, b) for b in boxes) and passable(boxes + [box], sec):
            break
    else:
        return
    boxes.append(box)
    # Height only: Object Info reports props at their unscaled size, so a scaled footprint misleads the GT planner and AEB
    # (rs2028_004, 2026-10-21); sometimes floating as some F2 assets do (10/6)
    sz = rng.uniform(1, 2.5) if data_id in SMALL else 1.0
    lift = rng.uniform(0, 0.4) if rng.random() < 0.3 else 0.0
    x, y, z, h = route.at(s, lat)
    o = copy.deepcopy(sample_obj)
    o.update(DataID=data_id, UNIQUEID=uid[0], pos=vec(x, y, z + lift), rot=rot(h + dyaw), scale=vec(1.0, 1.0, sz))
    uid[0] += 1
    d["objectList"].append(o)
    info.append(dict(zone=tag, id=data_id, s=round(s, 1), lat=round(lat, 2), yaw=round(dyaw, 1), size=list(size_of(data_id)),
                     scale=[1.0, 1.0, round(sz, 2)], lift=round(lift, 2), xy=[round(x, 2), round(y, 2)],
                     z=round(z + lift, 3), yaw_w=round((h + dyaw + 180) % 360 - 180, 2)))


def mover_cls(i):
    return "person" if i in PERSONS else "car" if i in CAR_SHAPED else "other"


def r2_dynamic(d, rng, route, uid, kind, s_lo, s_hi, curb, sample_ped, sample_obj, info, avoid=(), draw=choose):
    """1-2 movers 12 m apart and clear of the static footprints in avoid [(s, half length)] by 3 m (a deer crossing where
    the Ego bypassed a box walked into it, r2c3-b1-022), each a uniform movable asset (no car prop in the one-lane tunnel: one starts in the Ego's
    lane or ends with its body in it at the right wall, shadow-dyn-20261028b), its shape uniform over those possible there."""
    placed = [(s, hl + 3.0) for s, hl in avoid]
    pool = MOVERS if kind == "ra" else [m for m in MOVERS if m[0] not in CAR_SHAPED]
    for _ in range(rng.choice(N_OBJ)):
        for _try in range(50):
            s0 = rng.uniform(s_lo, s_hi)
            if all(abs(s0 - q) >= r for q, r in placed):
                break
        else:
            continue
        placed.append((s0, 12.0))
        data_id, as_ped = draw(rng, "movers", pool)
        cls = mover_cls(data_id)
        # behind a parked car: room in the right lane, and not a car prop (it overlaps the parked car and never moves, R2 QA)
        park = kind == "ra" and cls != "car" and any(a <= s0 <= b for a, b in PARK_S)
        # the near (right) roadside 2/3, the far side (its curb or just past the centre line) 1/3 (user 10/29)
        near = (("cross_rl",) + (("behind_parked",) if park else ())) if rng.random() < 2 / 3 else ()
        ptype = rng.choice(near or ("cross_lr", "from_centre"))
        timing, side = pick(rng, TIMINGS), -1 if ptype in ("cross_rl", "behind_parked") else 1
        if ptype == "behind_parked":   # the parked car in the right lane hides the object, which starts just past its front
            car = draw(rng, "parked_cars", CAR_SHAPED)
            s_car = s0 - size_of(car)[0] / 2 - rng.uniform(0.5, 1.5)
            info.append(r2_static(d, rng, route, uid[0], s_car, curb[0] + 1.6, sample_obj, car, f"{kind}_parked", 3.0))
            uid[0] += 1
        a = r2_actor(d, rng, route, uid[0], kind, s0, side, "from_centre" if ptype == "from_centre" else "crossing", timing,
                     curb, V_NOMINAL[kind], sample_ped, sample_obj, cls, data_id, as_ped)
        a["ptype"] = ptype
        info.append(a)
        uid[0] += 1


LENGTHS_ALL = [0, 1, 2, 3, 4]   # every vehicle length class (ring_probe.py)
RING_XY, RING_M = (-111.34, 358.26), 30.0   # clcol's roundabout spawn point (UNIQUEID 4); the highway points are 180 m+ away


def ring_points(d):
    """The roundabout's spawn points, found by position. ring_probe.py on a 5090, 10/28: each of the 8 VEHICLES spawns
    there when set (4-7 in 40 s at 3 s), one point takes one set type, and copies of the point with other types spawned
    nothing; type -1 made MORAI draw each spawn's type (length class 2, no bus or van)."""
    return [sp for sp in d.get("spawnPointList", [])
            if math.hypot(sp["pos"]["x"] - RING_XY[0], sp["pos"]["y"] - RING_XY[1]) < RING_M]


def traffic(d, rng, meta):
    """Signal phases and spawn points, and no vehicleList (make() and pool_scene())"""
    for light in d.get("tlList", []):
        light["deltaTime"] = rng.uniform(0, 60)
    meta["signal_delta"] = [round(t["deltaTime"], 1) for t in d.get("tlList", [])]

    # User 10/29: every spawn point draws each car's type at random (-1 over every length class: MORAI draws it per car,
    # with replacement; shortcut: a draw without replacement needs the manager to spawn each car over gRPC
    # Actor/SpawnVehicle, not called yet) and each interval uniformly between min and max; the count follows from those
    sps = []
    ring = ring_points(d)
    # eval_manager --density with --reload-each-lap: "ring lo:hi,highway lo:hi" s (world_director RING_PERIOD, HW_PERIOD)
    per = [tuple(map(float, r.split(":"))) for r in os.environ.get("NPC_PERIOD", "3:10,5:10").split(",")]
    for sp in d.get("spawnPointList", []):
        roundabout = any(sp is q for q in ring)
        # highway (user 10/29; MORAI Q&A: the speed is fixed within a lane and differs between lanes): each point spawns
        # a random car every 5-10 s (MORAI draws each interval between min and max) at the point's own speed
        lo, hi = per[0] if roundabout else per[1]
        v_lo = v_hi = rng.uniform(10, 35) if roundabout else rng.uniform(50, 100)
        bias = rng.uniform(0, 0.4)
        # Safety distance and TTC never below the sample's 5 m / 1 s: lower values made roundabout NPCs drive into the
        # waiting Ego (64 collisions in 35 episodes, none with the sample scene).
        sp.update(spawnVehicleTypeID=-1, listVehicleLengthType=LENGTHS_ALL,
                  MinDesiredVelocity_Custom=v_lo, MaxDesiredVelocity_Custom=v_hi, minSpawnPeriod=lo, maxSpawnPeriod=hi,
                  MinLatBias=-bias, MaxLatBias=bias, SafetyDist=rng.uniform(5, 10), TTC=rng.uniform(1.0, 2.0),
                  isLaneChange=False,
                  # "Maximum Spawn" counts every vehicle a point ever spawns: the sample's 50 ran out within minutes at
                  # 3-5 s, and the 10/27 W1 lap met an empty roundabout and highway. 1000 outlasts a 15 min lap at 3 s.
                  maximumSpawnVehicle=max(sp.get("maximumSpawnVehicle", 0), 1000))
        sps.append(dict(uid=sp["UNIQUEID"], roundabout=roundabout, kph=[round(v_lo, 1), round(v_hi, 1)], period=[lo, hi],
                        vehicle=-1, lengths=LENGTHS_ALL, bias=round(bias, 2)))
    meta["spawn"] = sps
    # The sample's vehicleList is a snapshot of NPCs its spawn points had made when it was saved (t 31.4 s, each with its
    # spawn point and destination): fixed cars at fixed places in every scene. The spawn points make all traffic.
    d["vehicleList"] = []


def shade(d, route, rng, meta):
    area = d["shadedAreaList"][0]
    s_area, lat_area = route.project(area["pos"]["x"], area["pos"]["y"])
    shift = rng.uniform(-15, 15)   # along the route, so the box's own yaw convention does not matter
    x, y, _, _ = route.at(s_area + shift, lat_area)
    area["pos"] = vec(x, y, area["pos"]["z"])
    meta["shade_shift_m"] = round(shift, 1)


def lay_boxes(d, rng, route, uid, sample_obj, sample_ped, info, draw=choose):
    boxes = []
    for _ in range(rng.choice(N_OBJ)):
        place_static(d, rng, route, uid, "static", *STATIC_BOX, static_sec, EGO_SPAN, sample_obj, info, boxes, draw,
                     in_ego_lane)
    r2_dynamic(d, rng, route, uid, "ra", *DYN_BOX, A_CURB, sample_ped, sample_obj, info, draw=draw)


def lay_tunnel(d, rng, route, uid, sample_obj, sample_ped, info, draw=choose):
    """-> the tunnel's composition; its objects go to info"""
    comp = pick(rng, TUNNEL)
    # the random mission anywhere in the tunnel (rules §3-2 shadow area; the §3-1 figure's box covers the tunnel, s
    # 1876-2000 registered) up to its exit (a box at 1982.7 drove the 10/21 driver into the field past CP15: the worst
    # case); static ones in the Ego's lane by the static box's rule (user 10/29), their footprint on either lane
    (lo, hi), t_boxes = TUNNEL_S, []
    if comp == "both":
        for _ in range(rng.choice(N_OBJ)):
            place_static(d, rng, route, uid, "tunnel_static", lo, hi, lambda q: TUNNEL_SEC, EGO_SPAN, sample_obj,
                         info, t_boxes, draw, lambda q, b: in_ego_lane(q, b, lambda _: TUNNEL_LANE))
    r2_dynamic(d, rng, route, uid, "rb", lo, hi, B_CURB, sample_ped, sample_obj, info,
               [((b[0] + b[1]) / 2, (b[1] - b[0]) / 2) for b in t_boxes], draw)
    return comp


def make(base, route, rng, uid0, cps=None):
    d = copy.deepcopy(base)
    meta = dict(gen=GEN)
    if cps:
        start_at(d, rng, cps, meta)
    traffic(d, rng, meta)
    uid, info = [uid0], []
    sample_obj, sample_ped = base["objectList"][0], base["pedestrianList"][0]
    d["objectList"], d["pedestrianList"] = [], []
    lay_boxes(d, rng, route, uid, sample_obj, sample_ped, info)
    if d.get("shadedAreaList"):
        shade(d, route, rng, meta)
        meta["tunnel"] = lay_tunnel(d, rng, route, uid, sample_obj, sample_ped, info)
    meta["actors"] = [{k: v for k, v in a.items() if k not in RT_KEYS} for a in info]
    return d, meta


# ----- continuous laps (user 10/29): pool_scene() is the run's one scene, layout() each lap's obstacles in it.
# Parking for the pool: lane centre lines (mean of the bounds' local_x/y/z) of kcity_lanelet2.osm lanelets 96262 (link
# A2256W000458), 97328-97371 (A2256W000400) and 110743/110748/107555 (A2256W001211, 000965), cut to 6 cm and 2 cm of z;
# every point 202-330 m from the route (Object Info lists objects near the Ego only), and no traffic drives there: the
# highway spawn points' paths stay within 23 m of the route and the roundabout's point is dropped.
PARK_LANES = [
    [(-301.65, 254.23, 28.65), (-301.44, 171.30, 28.69), (-300.88, 64.38, 28.71), (-300.46, 15.42, 28.73),
     (-300.35, -31.03, 28.73), (-300.09, -70.55, 28.72), (-300.03, -98.03, 28.72), (-299.69, -144.46, 28.73),
     (-299.71, -162.50, 28.73), (-299.47, -185.95, 28.73)],
    [(-396.39, 442.69, 28.47), (-395.51, 354.50, 28.48), (-394.85, 291.55, 28.59), (-394.65, 273.42, 28.56),
     (-394.31, 241.46, 28.63), (-393.82, 193.10, 28.65), (-393.68, 178.52, 28.70), (-393.53, 162.44, 28.66),
     (-392.88, 96.52, 28.73)],
    [(-342.04, 387.34, 28.54), (-341.61, 297.15, 28.54), (-341.39, 253.77, 28.57), (-340.91, 158.77, 28.54),
     (-340.47, 85.91, 28.58)]]
PARK_GAP = max(max(size_of(i)) for i in STATIC_POOL + [i for i, _ in MOVERS]) + 1.0   # no two touch at any yaw


def parking(n):
    """-> n poses (x, y, z, yaw) along PARK_LANES, every two PARK_GAP apart"""
    out = []
    for lane in PARK_LANES:
        for a, b in zip(lane, lane[1:]):
            seg, yaw = math.dist(a[:2], b[:2]), math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))
            for k in range(int(seg / 0.5)):
                p = [a[q] + k * 0.5 / seg * (b[q] - a[q]) for q in range(3)]
                if all(math.dist(p[:2], o[:2]) >= PARK_GAP for o in out):
                    out.append((*p, yaw))
                    if len(out) == n:
                        return out
    raise ValueError(f"PARK_LANES hold {len(out)} of {n} objects")


def pool_scene(base, route, rng):
    """-> (the run's one scene, its pool). Signals and the GPS shade as make() draws them; no spawn points (the director
    spawns the roundabout's and the highway's cars itself, world_director.HIGHWAY); one copy of every static asset, every
    mover and every car-shaped asset (behind_parked's parked car) parked at its own pose with no trigger and no waypoint
    path, so MORAI never moves them: the director does, by SetTransform on str(UNIQUEID).
    pool = {parking: [[uid, x, y, z, yaw]], static: [{uid, id, size}], movers: [{uid, id, ped, cls, size}],
    parked_cars: [{uid, id, size}], scene: the signal / shade draws as make()'s manifest has them (spawn [])}"""
    d, meta = copy.deepcopy(base), {}
    d["spawnPointList"] = []
    traffic(d, rng, meta)
    if d.get("shadedAreaList"):
        shade(d, route, rng, meta)
    sample_obj, sample_ped = base["objectList"][0], base["pedestrianList"][0]
    d["objectList"], d["pedestrianList"] = [], []
    objs = [("static", i, False) for i in STATIC_POOL] + [("movers", i, p) for i, p in MOVERS] + \
        [("parked_cars", i, False) for i in CAR_SHAPED]
    pool = dict(parking=[], static=[], movers=[], parked_cars=[], scene=meta)
    for uid, (cat, i, ped), pose in zip(range(1000, 1000 + len(objs)), objs, parking(len(objs))):
        x, y, z, yaw = (round(v, 3) for v in pose)
        o = copy.deepcopy(sample_ped if ped else sample_obj)
        o.update(DataID=i, UNIQUEID=uid, pos=vec(x, y, z), rot=rot(yaw), activeDistance=0.0, isWaypointMode=False)
        if ped:   # nowhere to walk should MORAI trigger it anyway
            o.update(initPos=vec(-x, z, -y), standardPos=vec(-x, z, -y), speed=0.0, movingDistance=0.0, loop=False)
        else:
            o.update(scale=vec(1.0, 1.0, 1.0), moveSpeed=0.0)
        d["pedestrianList" if ped else "objectList"].append(o)
        pool["parking"].append([uid, x, y, z, yaw])
        e = dict(uid=uid, id=i, size=list(size_of(i)))
        if cat == "movers":
            e.update(ped=ped, cls=mover_cls(i))
        pool[cat].append(e)
    return d, pool


def layout(route, rng, pool):
    """-> one lap's obstacles over pool (pool_scene) with make()'s distributions, each pool object used once at most:
    {gen, tunnel, actors}, each actor as make()'s manifest has it plus its pool uid, z and world yaw (statics: dyaw is
    make()'s yaw from the lane); a mover also its path ([x, y] from xy to xy_end), corr_in_m / corr_out_m (along the
    path to where it enters / leaves the Ego's corridor), s_cross (route s where it crosses the route line) and lead (s):
    the Ego's time to the crossing when the mover enters the corridor (must_stop) or has left it (pass_front)."""
    left = {(cat, (m["id"], m["ped"]) if cat == "movers" else m["id"]): m["uid"]
            for cat in ("static", "movers", "parked_cars") for m in pool[cat]}
    uids = dict(left)

    def draw(rng_, cat, xs):
        x = rng_.choice([x for x in xs if (cat, x) in left])
        del left[cat, x]
        return x

    d, uid, info = dict(objectList=[], pedestrianList=[], waypointDataList=[]), [0], []   # scratch: pool_scene's is it
    lay_boxes(d, rng, route, uid, {}, {}, info, draw)
    comp = lay_tunnel(d, rng, route, uid, {}, {}, info, draw)
    for a in info:
        cat = "movers" if a["zone"].endswith("_dynamic") else "parked_cars" if a["zone"].endswith("_parked") else "static"
        a["uid"] = uids[cat, (a["id"], a["ped"]) if cat == "movers" else a["id"]]
        if "yaw" in a:
            a["dyaw"] = a["yaw"]
        a["yaw"] = a.pop("yaw_w")
        if cat == "movers":
            a["lead"] = round(rng.uniform(*LEAD[a["timing"]]), 2)
    return dict(gen=GEN, tunnel=comp, actors=info)


def main():
    pool = sys.argv[1] == "pool"
    if pool:
        scen_dir, base_name, seed, path_txt, name = sys.argv[2:7]
    else:
        scen_dir, base_name, count, seed, path_txt = sys.argv[1:6]
    base = json.load(open(f"{scen_dir}/{base_name}.json", encoding="utf-8-sig"))
    mn = open(f"{scen_dir}/EgoNetwork/{base_name}_MN.json", encoding="utf-8-sig").read()
    route, rng = Route(path_txt), random.Random(int(seed))
    if pool:
        d, p = pool_scene(base, route, rng)
        open(f"{scen_dir}/{name}.json", "w").write(json.dumps(d, ensure_ascii=False))
        open(f"{scen_dir}/EgoNetwork/{name}_MN.json", "w").write(mn)
        json.dump(p, open(f"{scen_dir}/{name}-pool.json", "w"), indent=1)
        print(f"{name}/62")
        return
    prefix = sys.argv[6] if len(sys.argv) > 6 else "rv"
    cps = None
    if len(sys.argv) > 8:   # "row:weight,..." overrides STARTS (e.g. post-retreat restarts at rows 2 and 14)
        STARTS.clear()
        STARTS.update((int(r), int(w)) for r, w in (kv.split(":") for kv in sys.argv[8].split(",")))
    if len(sys.argv) > 7:
        import csv
        cps = [(float(r["x"]), float(r["y"]), float(r["z"]), float(r["yaw"])) for r in csv.DictReader(open(sys.argv[7]))]
    manifest = {}
    for k in range(int(count)):
        name = f"{prefix}{seed}_{k:03d}"
        d, meta = make(base, route, rng, 1000, cps)
        open(f"{scen_dir}/{name}.json", "w").write(json.dumps(d, ensure_ascii=False))
        open(f"{scen_dir}/EgoNetwork/{name}_MN.json", "w").write(mn)
        manifest[name] = meta
    json.dump(manifest, open(f"{scen_dir}/{prefix}{seed}-manifest.json", "w"), indent=1)
    print(" ".join(f"{n}/62" for n in manifest))


if __name__ == "__main__":
    main()
