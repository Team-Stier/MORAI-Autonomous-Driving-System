#!/usr/bin/env python3
"""Random lap scenes for closed-loop evaluation, from the collection scene (Alpamayo-Cosmos DATA_PLAN.md §6, DATA_PLAN_R2.md §4).
Copy of the collection skill's scripts/wsl/scenario-variants.py (sha256 50c7e3e1, 2026-10-26); eval_manager.py makes
one lap scene per lap with it (no checkpoints csv: the Ego keeps the base scene's START spawn). Eval changes (10/27): a
pedestrian's standardPos follows its new position (in the sample standardPos == initPos), a moved pedestrian gets its own
copy of the sample's waypoint path (without it the 10/27 laps' pedestrian never walked), and spawn points keep the base
scene's vehicle type.

Usage: scenario-variants.py <scenario dir> <base> <count> <seed> <global path txt> [prefix] [checkpoints csv] [starts]
With a checkpoints csv the Ego starts at a checkpoint just before a mission (section episodes, E1), 30% of them
offset laterally +-1 m and in heading +-8 deg (spawn-offset recovery, R2).
starts ("row:weight,...", e.g. "2:1,14:1") replaces the STARTS mix below.
Writes <prefix><seed>_<k>.json and EgoNetwork/<prefix><seed>_<k>_MN.json (a copy of the base's network file, so
each simulator copy keeps its own ports) plus <prefix><seed>-manifest.json with every sampled parameter.
The same seed gives the same scenarios on both simulator copies.

Randomized (official numbers are undisclosed, so ranges are wide):
- signals: each light's elapsed time in its current state (arrival phase)
- NPC spawn points: vehicle model, desired speed, period, lateral bias, safety distance and TTC
- static obstacles (zone of the official sample object): 1-2 at least 12 m apart, position along the route, lateral offset,
  height scale 1-2.5x (footprint 1x: Object Info sizes are unscaled), yaw along the lane +-15 deg, sometimes floating 0-0.4 m
- dynamic zone (the official sample pedestrian's, s 445-500; DATA_PLAN_R2 §4 A, the R2 generator's code): 1-2 objects,
  person 70 % (every pedestrian-type asset) / other mover 15 % (animals, cyclist, bicycles, scooters, strollers) / car
  prop 15 %; crossing left-right or right-left, from behind a parked car, along the roadside; timing must stop / pass
  in front / already passed / no conflict 44/22/22/11 % with the trigger distance from it (8-45 m); 3-12 kph (cars
  10-30). Eval: no object ends in the route lane (no enter-and-stop, no car cut-in or car along the lane)
- tunnel (DATA_PLAN_R2 §4 B): dynamic / static + dynamic / static / none 40/30/20/10 within s 1894-1965
- GPS shadow area: shifted +-15 m along the road
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
# d4a7eda2, 10/27), as a lap needs them: see the PTYPES / cls / cut-in changes marked eval
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
V_NOMINAL = {"ra": 11.0, "rb": 11.0}         # ego speed assumed when sizing the trigger distance (pilot: 10-13 m/s)
# Trigger distance cap. The plan's 5-25 m (official sample 15) cannot time a far-side (+9.4 m curb) crossing into the
# route lane at 3-12 kph, so the cap is 45 m (logged decision, 2026-10-27).
ACTIVE_MAX = 45.0
R2_CAR_MODE = "prop"   # car-shaped props on an ObstaclePath (moved 6-10 m in the R2 pilot)
ACTIVE_MIN = 8.0   # a 5 m trigger beside the lane never fired as the Ego passed (r2p2-w1-002)


def pick(rng, table):
    r, acc = rng.random(), 0.0
    for k, w in table:
        acc += w
        if r < acc:
            return k
    return table[-1][0]


def r2_actor(d, rng, route, uid, kind, s0, side, ptype, timing, curb, v_e, sample_ped, sample_obj, cls):
    """One moving object of class cls (person/other/car). side: +1 starts left of the route, -1 right."""
    is_car = cls == "car" and R2_CAR_MODE == "prop"
    if cls == "person":
        data_id, as_ped = rng.choice(PERSONS), True
    elif cls == "other":   # half pedestrian-type movers, half waypoint props (both moved in the pilot)
        as_ped = rng.random() < 0.5
        data_id = rng.choice(OTHER_PEDS if as_ped else OTHER_PROPS)
    elif is_car:
        data_id, as_ped = rng.choice(CAR_PROPS), False
    else:   # R2_CAR_MODE=cyclist: the cyclist (50100084) at 10-20 kph stands in for a moving car
        data_id, as_ped = 50100084, True
    kph = rng.uniform(10, 30) if is_car else rng.uniform(10, 20) if cls == "car" else rng.uniform(3, 12)
    v_o = kph / 3.6
    half_w = 1.05 if is_car else 0.35
    off = rng.uniform(0, 2 if kind == "ra" else 0.8)   # from the curb: 0-2 m of sidewalk, 0-0.8 m to the tunnel wall
    lat0 = (curb[1] + off if side > 0 else curb[0] - off) if not is_car else (curb[1] - 1.5 if side > 0 else curb[0] + 1.6)
    s_start, extra = s0, []
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
        t_e = t_in / rng.uniform(0.25, 0.75) if t_in > 0.3 else rng.uniform(1.0, 2.5)
    elif timing == "pass_front":
        t_e = t_out + rng.uniform(0.3, 1.2)
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
    return dict(zone=f"{kind}_dynamic", cls=cls, id=data_id, ped=as_ped, ptype=ptype, timing=timing, s=round(s_start, 1),
                xy=[round(x, 2), round(y, 2)], xy_end=[round(pts[-1][0], 2), round(pts[-1][1], 2)],
                lat=round(lat0, 2), s_end=round(s_end, 1), lat_end=round(lat_end, 2), kph=round(kph, 1),
                active=round(active, 1), move=round(length, 1), t_in=round(t_in, 2), t_e=round(t_e, 2))


def r2_static(d, rng, route, uid, s, lat, sample_obj, data_id, tag, yaw_jitter=15.0):
    o = copy.deepcopy(sample_obj)
    x, y, z, h = route.at(s, lat)
    sz = 1.0 if data_id in VEHICLE or data_id in CAR_PROPS else rng.uniform(1, 2.5)
    yaw = h + rng.choice((0.0, 180.0)) + rng.uniform(-yaw_jitter, yaw_jitter)
    o.update(DataID=data_id, UNIQUEID=uid, pos=vec(x, y, z), rot=rot(yaw), scale=vec(1.0, 1.0, sz), isWaypointMode=True,
             moveSpeed=0.0, activeDistance=0.0)
    d["objectList"].append(o)
    d["waypointDataList"].append({"pathName": f"ObstaclePath - {uid}", "waypointData": []})
    return dict(zone=tag, id=data_id, s=round(s, 1), lat=round(lat, 2), scale_z=round(sz, 2), xy=[round(x, 2), round(y, 2)])


PTYPES = [("cross_lr", 0.35), ("cross_rl", 0.35), ("behind_parked", 0.18), ("along", 0.12)]   # eval: no enter_stop
# Timing of the 90 % non-"along" objects; "along" (10 %) is no-conflict too, so 0.444/0.222/0.222/0.111 gives the plan's
# 40/20/20/20 overall. R2_TIMING="must_stop:w,pass_front:w,passed:w,no_conflict:w" overrides (catch-up batches).
TIMINGS = [("must_stop", 0.444), ("pass_front", 0.222), ("passed", 0.222), ("no_conflict", 0.112)]


def r2_dynamic(d, rng, route, uid, kind, s_lo, s_hi, curb, sample_ped, sample_obj, info, avoid=None):
    """avoid: s of a static obstacle; movers keep 25 m from it (a deer crossing where the Ego bypassed a box in the
    oncoming lane walked into it, r2c3-b1-022)."""
    n = 1 if rng.random() < 0.7 else 2
    placed = [] if avoid is None else [avoid]
    for _ in range(n):
        for _try in range(20):
            s0 = rng.uniform(s_lo, s_hi)
            if all(abs(s0 - q) >= 12 for q in placed) and (avoid is None or abs(s0 - avoid) >= 25):
                break
        else:
            continue
        placed.append(s0)
        ptype, timing = pick(rng, PTYPES), pick(rng, TIMINGS)
        side = {"cross_lr": 1, "cross_rl": -1, "behind_parked": -1}.get(ptype, rng.choice((-1, 1)))
        if kind == "rb" and ptype in ("along", "enter_stop"):
            side = 1 if ptype == "along" else side   # tunnel walkers keep to the left wall
        if ptype == "behind_parked" and kind == "ra":
            s0 = max(s0, rng.uniform(472.0, 500.0))   # s 445-470 is the intersection area: the driver bypassed the car
        if ptype == "behind_parked" and kind == "rb":
            ptype = "cross_rl"   # no parking room in the tunnel: a car there would stand in the route lane
        if ptype == "behind_parked":
            # a parked car in the right lane hides the object, which starts just past its front
            car = rng.choice(CAR_PROPS[:5])
            s_car = s0 - PROP_HALF_L[car] - rng.uniform(0.5, 1.5)
            info.append(r2_static(d, rng, route, uid[0], s_car, curb[0] + 1.6, sample_obj, car, f"{kind}_parked", 3.0))
            uid[0] += 1
        cls = pick(rng, [("person", 0.70), ("other", 0.15), ("car", 0.15)])
        if cls == "car" and ptype in ("behind_parked", "along"):   # a car prop behind the parked car overlaps it and
            cls = "person"                                        # never moves (R2 QA); one along the lane stays in it
        if ptype == "along":
            timing = "no_conflict"
        elif timing == "no_conflict":
            # Shapes that never reach the route lane: it stops at the curb, crosses only part of the road (stops 1.5 m
            # short of the route lane), or crosses after the Ego is through (needs >= 2.5 s to the lane, else it was
            # turned into "passed": no-conflict fell to 8 %, r2c3). Cars drive along the next lane.
            ptype = "late_cross" if cls == "car" else pick(rng, [("curb_stop", 0.35), ("short_cross", 0.35), ("late_cross", 0.30)])
            if ptype == "late_cross":
                ptype = rng.choice(("cross_lr", "cross_rl"))
                side = 1 if ptype == "cross_lr" else -1
            elif kind == "rb":
                side = 1   # tunnel: the left wall only; at the right wall line (-1.75 m) a stopped pedestrian is inside
                           # the Ego's corridor and the GT AEB held it (r2c3-w2-035)
        geo = "crossing" if ptype in ("cross_lr", "cross_rl", "behind_parked") else ptype
        # eval: no cut-in (the prop stays in the route lane at its path end and blocks the rest of the lap)
        a = r2_actor(d, rng, route, uid[0], kind, s0, side, geo, timing, curb, V_NOMINAL[kind], sample_ped, sample_obj,
                     cls)
        if timing == "no_conflict" and a["timing"] != "no_conflict":   # too close/fast to cross late: stop at the curb
            (d["pedestrianList"] if a["ped"] else d["objectList"]).pop()
            d["waypointDataList"].pop()
            ptype = geo = "curb_stop"
            a = r2_actor(d, rng, route, uid[0], kind, s0, side, geo, timing, curb, V_NOMINAL[kind], sample_ped,
                         sample_obj, cls)
        a["ptype"] = "cutin" if geo == "cutin" else ptype
        info.append(a)
        uid[0] += 1


def make(base, route, rng, uid0, cps=None):
    d = copy.deepcopy(base)
    meta = {}
    if cps:
        start_at(d, rng, cps, meta)
    for light in d.get("tlList", []):
        light["deltaTime"] = rng.uniform(0, 60)
    meta["signal_delta"] = [round(t["deltaTime"], 1) for t in d.get("tlList", [])]

    sps = []
    for sp in d.get("spawnPointList", []):
        roundabout = sp.get("spawnVehicleTypeID", -1) != -1 and sp.get("MinDesiredVelocity_Custom", 0) < 30
        v = rng.uniform(10, 30) if roundabout else rng.uniform(50, 100)
        period = rng.uniform(3, 10) if roundabout else 5.0   # highway: the densest the rules allow (eval 10/27; collection 5-15 s)
        bias = rng.uniform(0, 0.4)
        # Safety distance and TTC never below the sample's 5 m / 1 s: lower values made roundabout NPCs drive into the
        # waiting Ego (64 collisions in 35 episodes, none with the sample scene).
        sp.update(MinDesiredVelocity_Custom=v, MaxDesiredVelocity_Custom=v, minSpawnPeriod=period, maxSpawnPeriod=period,
                  MinLatBias=-bias, MaxLatBias=bias, SafetyDist=rng.uniform(5, 10), TTC=rng.uniform(1.0, 2.0),
                  isLaneChange=False,   # vehicle type kept from the base scene: the 10/27 eval lap on bc-250 had no
                                        # roundabout NPC with the randomized 30200003 (the sample's 30100004 is kept)
                  # "Maximum Spawn" counts every vehicle a point ever spawns: the sample's 50 ran out within minutes at
                  # 3-5 s, and the 10/27 W1 lap met an empty roundabout and highway. 1000 outlasts a 15 min lap at 3 s.
                  maximumSpawnVehicle=max(sp.get("maximumSpawnVehicle", 0), 1000))
        sps.append(dict(uid=sp["UNIQUEID"], roundabout=roundabout, kph=round(v, 1), period=round(period, 1),
                        vehicle=sp["spawnVehicleTypeID"], bias=round(bias, 2)))
    meta["spawn"] = sps

    uid = [uid0]

    def obstacle(template, s, lat, tag):
        o = copy.deepcopy(template)
        x, y, z, h = route.at(s, lat)
        lift = rng.uniform(0, 0.4) if rng.random() < 0.3 else 0.0
        data_id = rng.choice(VEHICLE) if rng.random() < 0.2 else rng.choice(SMALL)
        # Footprint stays 1x: Object Info reports props at their unscaled size, so the GT planner and AEB saw a scaled
        # box up to 3x too small and squeezed into it (rs2028_004, 2026-10-21). Height does not change the footprint.
        sx, sy, sz = (1.0, 1.0, 1.0) if data_id in VEHICLE else (1.0, 1.0, rng.uniform(1, 2.5))
        # Along the lane +-15 deg (either way round): car-sized props (40100008/10/11, 4.6-5 m) at a random yaw stood
        # across the lane and blocked both lanes in the tunnel (rv1028_050, 2026-10-21).
        yaw = h + rng.choice((0.0, 180.0)) + rng.uniform(-15, 15)
        o.update(DataID=data_id, UNIQUEID=uid[0], pos=vec(x, y, z + lift), rot=rot(yaw), scale=vec(sx, sy, sz))
        uid[0] += 1
        return o, dict(zone=tag, id=data_id, s=round(s, 1), lat=round(lat, 2), scale=[round(sx, 2), round(sy, 2), round(sz, 2)],
                       lift=round(lift, 2), xy=[round(x, 2), round(y, 2)])

    objs, peds, info = [], [], []
    sample_obj = base["objectList"][0]
    s0, lat0 = route.project(sample_obj["pos"]["x"], sample_obj["pos"]["y"])
    placed = []
    for _ in range(rng.choice([1, 1, 2, 2, 3])):
        for _try in range(20):
            # The sample sits 13 m past stop line s=315.9; stay out of the junction and leave ~12 m to rejoin the
            # route lane before checkpoint 4 (s=346.7, 3 m radius), or the judge counts a skipped checkpoint.
            # s0-7 at the earliest (about 6 m past the stop line): a 4.6 m prop 3.5 m past it made the red wait start
            # the bypass before the line (rv1030_065, 2026-10-21).
            s = s0 + rng.uniform(-7, 5)
            if all(abs(s - q) >= 12 for q in placed):
                break
        else:
            # No 12 m gap left in the 15 m zone: drop it. Keeping the last try piled 2-3 obstacles within 4 m on
            # both sides and the driver stalled behind them for the rest of the episode (rv1024_039, rs2025_048).
            continue
        placed.append(s)
        # The route is the rightmost lane here (only the left lane is road in the lanelet map): an obstacle right of
        # centre makes the driver pass in the left lane; left of centre it swerved off the road (2026-10-20).
        lat = -rng.uniform(0.2, 1.0) if rng.random() < 0.8 else rng.uniform(0.2, 0.6)
        o, i = obstacle(sample_obj, s, lat, "static")
        objs.append(o)
        info.append(i)

    sample_ped = base["pedestrianList"][0]
    d["objectList"], d["pedestrianList"] = objs, peds
    # dynamic zone (the official sample pedestrian's, past stop line s 436.8): DATA_PLAN_R2 §4 A objects, s 445-500
    r2_dynamic(d, rng, route, uid, "ra", 445.0, 500.0, A_CURB, sample_ped, sample_obj, info)

    if d.get("shadedAreaList"):
        area = d["shadedAreaList"][0]
        s_area, lat_area = route.project(area["pos"]["x"], area["pos"]["y"])
        shift = rng.uniform(-15, 15)   # along the route, so the box's own yaw convention does not matter
        x, y, _, _ = route.at(s_area + shift, lat_area)
        area["pos"] = vec(x, y, area["pos"]["z"])
        st = s_area + shift
        half = float(area["size"]["x"]) * 0.4
        meta["shade_shift_m"] = round(shift, 1)
        comp = pick(rng, [("dynamic", 0.4), ("both", 0.3), ("static", 0.2), ("none", 0.1)])   # DATA_PLAN_R2 §4 B
        meta["tunnel"] = comp
        if comp in ("both", "static"):
            # Only past s 1894 (front face >= 1890 even for a 6.6 m vehicle model): before that the solid left line of
            # lanelet 96340 leaves no legal bypass and the driver can only stand (rs2028_004, 2026-10-21).
            # and before s 1965: a box at 1982.7 left no room to rejoin before the exit curve, and the Ego ran into the
            # field past CP15 (run1021e-w1-005).
            s_box = min(max(st + rng.uniform(-half, half), 1894.0 + rng.uniform(0, 8)), 1965.0 - rng.uniform(0, 8))
            # lat <= +0.6: a 5 m prop at 15 deg skew reaches +1.65 m beyond its centre, and the left bypass in 102071
            # needs the box to end by about +2.37 m (driver session, 2026-10-21)
            o, i = obstacle(sample_obj, s_box, rng.uniform(-1.0, 0.6), "tunnel_static")
            objs.append(o)
            info.append(i)
        if comp in ("both", "dynamic"):
            r2_dynamic(d, rng, route, uid, "rb", 1894.0, 1965.0, B_CURB, sample_ped, sample_obj, info,
                       info[-1]["s"] if comp == "both" else None)
    meta["actors"] = info
    return d, meta


def main():
    scen_dir, base_name, count, seed, path_txt = sys.argv[1:6]
    prefix = sys.argv[6] if len(sys.argv) > 6 else "rv"
    cps = None
    if len(sys.argv) > 8:   # "row:weight,..." overrides STARTS (e.g. post-retreat restarts at rows 2 and 14)
        STARTS.clear()
        STARTS.update((int(r), int(w)) for r, w in (kv.split(":") for kv in sys.argv[8].split(",")))
    if len(sys.argv) > 7:
        import csv
        cps = [(float(r["x"]), float(r["y"]), float(r["z"]), float(r["yaw"])) for r in csv.DictReader(open(sys.argv[7]))]
    base = json.load(open(f"{scen_dir}/{base_name}.json", encoding="utf-8-sig"))
    mn = open(f"{scen_dir}/EgoNetwork/{base_name}_MN.json", encoding="utf-8-sig").read()
    route, rng = Route(path_txt), random.Random(int(seed))
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
