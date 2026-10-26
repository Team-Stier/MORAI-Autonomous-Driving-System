#!/usr/bin/env python3
"""Random variants of the collection scene for data balance (Alpamayo-Cosmos DATA_PLAN.md §6).
Copy of the collection skill's scripts/wsl/scenario-variants.py (sha256 50c7e3e1, 2026-10-26); eval_manager.py makes
one lap scene per lap with it (no checkpoints csv: the Ego keeps the base scene's START spawn). Eval changes (10/27): a
pedestrian's standardPos follows its new position (it stayed at the sample's, and the 10/27 lap's pedestrian never walked;
in the sample standardPos == initPos), and spawn points keep the base scene's vehicle type.

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
- pedestrian (zone of the official sample): position along the road, trigger distance, speed, walking distance
- GPS shadow area: shifted +-15 m along the road; tunnel obstacle: none / static / pedestrian 40/40/20
Map coordinates of the scenario equal the driver's map frame (sample Ego start = route start).
"""
import copy
import json
import math
import random
import sys

VEHICLES = [30100004, 30100005, 30100010, 30100014, 30200003, 30300002, 30400004, 30700003]  # IDs seen in saved scenes
# Obstacle DataIDs that loaded in the 2026-10-20 probe (probe-assets.py), by unit-scale bbox (m). Vehicle-sized ones
# keep scale 1. shortcut: only the 20 Object Info records nearest the start were read; probe the rest in batches.
SMALL = [40100001, 40100002, 40100003, 40100004, 40100005, 40100026, 40100027, 40100028, 40100029, 40100033,
         40100058, 40100059]   # boxes 0.75/1.2, drums, barrels, bollards, barricades 0.6x1.35, concrete barrier 1x3
VEHICLE = [40100008, 40100009, 40100010, 40100011]   # parked-car/truck obstacles 4.6-6.6 m
PEDESTRIANS = [50100001, 50100017, 50100018]


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
                  isLaneChange=False)   # vehicle type kept from the base scene: the 10/27 eval lap on bc-250 had no
                                        # roundabout NPC with the randomized 30200003 (the sample's 30100004 is kept)
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
                       lift=round(lift, 2))

    def pedestrian(template, s, lat, tag):
        # keeps the sample's side and heading: the curb on the other side is not at -lat (the route is a lane)
        p = copy.deepcopy(template)
        x, y, z, h = route.at(s, lat)
        p.update(DataID=rng.choice(PEDESTRIANS), UNIQUEID=uid[0], pos=vec(x, y, z), initPos=vec(-x, z, -y), standardPos=vec(-x, z, -y),
                 activeDistance=rng.uniform(8, 25), speed=rng.uniform(4, 12), movingDistance=rng.uniform(4, 10))
        uid[0] += 1
        return p, dict(zone=tag, id=p["DataID"], s=round(s, 1), active=round(p["activeDistance"], 1),
                       speed=round(p["speed"], 1), move=round(p["movingDistance"], 1))

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
    sp0, latp = route.project(sample_ped["pos"]["x"], sample_ped["pos"]["y"])
    p, i = pedestrian(sample_ped, sp0 + rng.uniform(-15, 20), latp, "dynamic")   # stop line s=436.8 is 23 m back
    peds.append(p)
    info.append(i)

    if d.get("shadedAreaList"):
        area = d["shadedAreaList"][0]
        s_area, lat_area = route.project(area["pos"]["x"], area["pos"]["y"])
        shift = rng.uniform(-15, 15)   # along the route, so the box's own yaw convention does not matter
        x, y, _, _ = route.at(s_area + shift, lat_area)
        area["pos"] = vec(x, y, area["pos"]["z"])
        st = s_area + shift
        half = float(area["size"]["x"]) * 0.4
        meta["shade_shift_m"] = round(shift, 1)
        r = rng.random()
        if r < 0.4:
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
        elif r < 0.6:
            p, i = pedestrian(sample_ped, st + rng.uniform(-half, half), latp, "tunnel_dynamic")
            peds.append(p)
            info.append(i)
    d["objectList"], d["pedestrianList"] = objs, peds
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
