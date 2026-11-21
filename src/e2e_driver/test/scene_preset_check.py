"""scenario_variants.py v2 (one distribution for lap videos, sync eval and the real-time actors, 10/29): obstacles only in
the rules' boxes and the tunnel, 1-2 per box and kind, every asset drawn, static ones uniform over the Ego lane (on the
road, apart, the road never shut), movers cross (also from just past the centre line) at a must-stop / pass-in-front timing set off early enough, no car prop in
the tunnel, behind-parked only where the right lane is free, spawn points
drawing each car's type and interval, no NPC from the sample's saved snapshot, tunnel objects over the whole tunnel.
Builds 600 scenes from a stand-in base (the sample object, the shade in the tunnel, a ring and a highway point).
Continuous laps (10/29): pool_scene() has every asset once, parked on the road 150 m+ from the route and apart, never
moved by MORAI, no spawn points (world_director spawns the traffic); 600 layout()s over it pass the same checks, each pool object at most once
a lap, with the director's fields (uid, path, corridor entry / exit, s_cross, lead).
Mac: python3 scene_preset_check.py
"""
import bisect
import collections
import json
import math
import random
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "tools"))
import scenario_variants as sv  # noqa: E402

route = sv.Route(HERE.parents[1] / "map" / "2026_molit_comp_global_path.txt")
x, y, z, _ = route.at(329.0, -0.5)
sx, sy, _, _ = route.at(1930.0, 0.0)
spawn = lambda uid, px, py: dict(UNIQUEID=uid, pos={"x": px, "y": py}, spawnVehicleTypeID=-1)  # noqa: E731
base = dict(egoVehicle={"initPosition": {"pos": {}, "rot": {}}}, tlList=[], waypointDataList=[],
            spawnPointList=[spawn(4, -111.3, 358.3), spawn(10, 73.0, 265.6)],
            objectList=[{"pos": {"x": x, "y": y, "z": z}}], pedestrianList=[{}],
            shadedAreaList=[{"pos": {"x": sx, "y": sy, "z": 0.0}, "size": {"x": 117.959}}])


def run(laps):
    """Every lap's actors (make()'s manifest form) pass the v2 checks; over all laps every asset and shape shows up and
    static objects spread uniformly over the lane area."""
    n, static_ids, mover_ids, centres, shapes = collections.Counter(), set(), set(), [], collections.Counter()
    for acts in laps:
        for zone, lo, hi in (("static", *sv.STATIC_BOX), ("ra_dynamic", 421.0 - 6, 503.0 + 6)):
            k = sum(a["zone"] == zone for a in acts)
            assert 1 <= k <= 2 and all(lo <= a["s"] <= hi for a in acts if a["zone"] == zone), (zone, k)
            n[(zone, k)] += 1
        assert any(a["zone"] == "rb_dynamic" for a in acts)
        assert all(sv.TUNNEL_S[0] - 6 <= a["s"] <= sv.TUNNEL_S[1] + 6 for a in acts if a["zone"][:2] in ("tu", "rb")), acts
        for tag, sec in (("static", sv.static_sec), ("tunnel_static", lambda q: sv.TUNNEL_SEC)):
            boxes = []
            for a in acts:
                if a["zone"] == tag:
                    hl, hw = sv.foot(a["id"], a["yaw"])
                    hl, hw = hl - 0.06, hw - 0.02   # inside the generator's box despite the manifest's rounding
                    boxes.append((a["s"] - hl, a["s"] + hl, a["lat"] - hw, a["lat"] + hw))
                    road = [sec(q) for q in (boxes[-1][0], a["s"], boxes[-1][1])]   # the manifest's s is rounded to 0.1 m
                    grown = (boxes[-1][0], boxes[-1][1], boxes[-1][2] - 0.06, boxes[-1][3] + 0.06)
                    lane = sv.ego_lane if tag == "static" else (lambda _: sv.TUNNEL_LANE)   # the overlap rule, both
                    held = sv.in_ego_lane(a["s"], boxes[-1], lane) or sv.in_ego_lane(a["s"], grown, lane)
                    assert held and min(r[0] for r in road) - 0.05 <= boxes[-1][2] and \
                        boxes[-1][3] <= max(r[2] for r in road) + 0.05, a
                    static_ids.add(a["id"])
                    if tag == "static":
                        centres.append(a["s"])
            assert sv.passable(boxes, sec) and not any(sv.overlap(p, q) for i, p in enumerate(boxes) for q in boxes[i + 1:])
        for a in acts:
            if not a["zone"].endswith("_dynamic"):
                continue
            mover_ids.add(a["id"])
            assert a["timing"] in ("must_stop", "pass_front") and a["ptype"] in ("cross_lr", "cross_rl", "from_centre",
                                                                              "behind_parked"), a
            assert a["zone"] != "rb_dynamic" or (a["cls"] != "car" and a["ptype"] != "behind_parked"), a
            assert a["ptype"] != "behind_parked" or any(p - 0.1 <= a["s"] <= q + 0.1 for p, q in sv.PARK_S), a
            kind = a["zone"][:2]
            # set off early enough: the Ego (on the route at V_NOMINAL) gets to the crossing only after the object is in
            # the lane (must stop) / through it (pass in front)
            corr = 1.2 + (1.05 if a["cls"] == "car" else 0.35)
            t_out = a["t_in"] + 2 * corr / (a["kph"] / 3.6)
            assert a["active"] > abs(a["lat"]), a
            t_ego = math.sqrt(a["active"] ** 2 - a["lat"] ** 2) / sv.V_NOMINAL[kind]
            assert t_ego >= (a["t_in"] if a["timing"] == "must_stop" else t_out) - 0.05, (t_ego, a)
            if a["ptype"] == "from_centre":
                assert sv.CENTRE[kind] < a["lat"] <= sv.CENTRE[kind] + 1.05 + 1.0 + 0.01, a
            shapes[a["ptype"]] += 1
    # uniform over the valid places: the share of static objects before s 335 is that part's share of the uniform
    # proposals place_static accepts on their own (asset, s, lat, yaw; on the road and in the Ego lane)
    mc, acc = random.Random(9), []
    for _ in range(40000):
        i, q, lat, dy = mc.choice(sv.STATIC_POOL), mc.uniform(*sv.STATIC_BOX), mc.uniform(*sv.EGO_SPAN), mc.uniform(-180, 180)
        hl, hw = sv.foot(i, dy)
        b = (q - hl, q + hl, lat - hw, lat + hw)
        road = [sv.static_sec(t) for t in (q - hl, q, q + hl)]
        if sv.in_ego_lane(q, b) and max(r[0] for r in road) <= b[2] and b[3] <= min(r[2] for r in road):
            acc.append(q)
    want, got = sum(c < 335 for c in acc) / len(acc), sum(c < 335 for c in centres) / len(centres)
    assert abs(got - want) < 0.06, (got, want)
    assert static_ids == set(sv.STATIC_POOL), set(sv.STATIC_POOL) - static_ids
    assert mover_ids == {i for i, _ in sv.MOVERS}, {i for i, _ in sv.MOVERS} - mover_ids
    assert set(shapes) == {"cross_lr", "cross_rl", "from_centre", "behind_parked"}, shapes
    near = (shapes["cross_rl"] + shapes["behind_parked"]) / sum(shapes.values())
    assert abs(near - 2 / 3) < 0.04, ("near roadside share", near)   # user 10/29: 2/3 from the near side
    return dict(sorted(n.items())), dict(shapes)


def scenes(rng):
    for _ in range(600):
        d, meta = sv.make(base, route, rng, 1000)
        assert d["vehicleList"] == []   # the sample's saved NPC snapshot is dropped
        assert meta["gen"] == sv.GEN and meta["tunnel"] in ("dynamic", "both"), meta["tunnel"]
        assert [sp["roundabout"] for sp in meta["spawn"]] == [True, False] and all(
            sp["vehicle"] == -1 and sp["lengths"] == sv.LENGTHS_ALL and sp["period"][0] < sp["period"][1]
            for sp in meta["spawn"])
        assert not any(k in a for a in meta["actors"] for k in sv.RT_KEYS)   # make()'s manifest as before
        yield meta["actors"]


print("make ok:", *run(scenes(random.Random(1))))


# ----- pool_scene + layout
def seg_dist(p, a, b):
    ax, ay = b[0] - a[0], b[1] - a[1]
    u = max(0.0, min(1.0, ((p[0] - a[0]) * ax + (p[1] - a[1]) * ay) / (ax * ax + ay * ay or 1e-9)))
    return math.hypot(a[0] + u * ax - p[0], a[1] + u * ay - p[1])


def lanelets():
    """-> [(left, right)] bound polylines [(x, y, z)] of every lanelet in kcity_lanelet2.osm"""
    nodes, ways, out = {}, {}, []
    for _, el in ET.iterparse(HERE.parents[1] / "map" / "kcity_lanelet2.osm"):
        if el.tag == "node":
            t = {c.get("k"): c.get("v") for c in el.findall("tag")}
            nodes[el.get("id")] = (float(t["local_x"]), float(t["local_y"]), float(t["local_z"]))
        elif el.tag == "way":
            ways[el.get("id")] = [nodes[nd.get("ref")] for nd in el.findall("nd")]
        elif el.tag == "relation":
            m = {mb.get("role"): ways[mb.get("ref")] for mb in el.findall("member")}
            out.append((m["left"], m["right"]))
        if el.tag in ("node", "way", "relation"):
            el.clear()
    return out


def on_lanelet(p, lls):
    """p (x, y, z) lies between a lanelet's bounds (its distances to them add up to the width there) at its z (5 cm)"""
    for left, right in lls:
        nl = min(left, key=lambda q: math.dist(q[:2], p[:2]))
        if math.dist(nl[:2], p[:2]) > 30:
            continue
        nr = min(right, key=lambda q: math.dist(q[:2], p[:2]))
        dl = min(seg_dist(p, a, b) for a, b in zip(left, left[1:]))
        dr = min(seg_dist(p, a, b) for a, b in zip(right, right[1:]))
        width = min(seg_dist(nl, a, b) for a, b in zip(right, right[1:]))
        if dl + dr <= width + 0.1 and abs(p[2] - (nl[2] + nr[2]) / 2) < 0.05:
            return True
    return False


def lat_near(p, s0):
    """lateral offset of p from the route near s0 (Route.project snaps to the route's other pass through a crossroads)"""
    ks = range(bisect.bisect_left(route.s, s0 - 40), min(bisect.bisect_right(route.s, s0 + 40), len(route.p) - 1))
    i = min((k for k in ks if route.s[k + 1] > route.s[k]), key=lambda k: math.dist(route.p[k][:2], p))   # path repeats points
    (x0, y0, _), (x1, y1, _) = route.p[i], route.p[i + 1]
    h = math.atan2(y1 - y0, x1 - x0)
    return -(p[0] - x0) * math.sin(h) + (p[1] - y0) * math.cos(h)


def at_dist(path, m):
    for a, b in zip(path, path[1:]):
        seg = math.dist(a, b)
        if m <= seg:
            return a[0] + m / seg * (b[0] - a[0]), a[1] + m / seg * (b[1] - a[1])
        m -= seg
    return path[-1]


rng = random.Random(2)
d, pool = sv.pool_scene(base, route, rng)
pool = json.loads(json.dumps(pool))   # as eval_manager reads it
assert sorted(m["id"] for m in pool["static"]) == sv.STATIC_POOL
assert sorted((m["id"], m["ped"]) for m in pool["movers"]) == sorted(sv.MOVERS)
assert sorted(m["id"] for m in pool["parked_cars"]) == sv.CAR_SHAPED
by_uid = {m["uid"]: (cat, m) for cat in ("static", "movers", "parked_cars") for m in pool[cat]}
assert len(by_uid) == len(pool["static"]) + len(pool["movers"]) + len(pool["parked_cars"]) and min(by_uid) >= 1000
assert sorted(p[0] for p in pool["parking"]) == sorted(by_uid)
gap = max(max(m["size"]) for _, m in by_uid.values()) + 1.0
park = pool["parking"]
near = min(math.dist(p[1:3], q[:2]) for p in park for q in route.p)
spacing = min(math.dist(p[1:3], q[1:3]) for i, p in enumerate(park) for q in park[i + 1:])
assert near >= 150 and spacing >= gap, (near, spacing, gap)
lls = lanelets()
assert all(on_lanelet(p[1:4], lls) for p in park), [p for p in park if not on_lanelet(p[1:4], lls)]
objs = {o["UNIQUEID"]: o for o in d["objectList"] + d["pedestrianList"]}
assert sorted(objs) == sorted(by_uid) and {o["UNIQUEID"] for o in d["pedestrianList"]} == \
    {u for u, (cat, m) in by_uid.items() if cat == "movers" and m["ped"]}
for uid, px, py, pz, yaw in park:
    o = objs[uid]
    assert (o["DataID"], o["pos"]["x"], o["pos"]["y"], o["pos"]["z"]) == (by_uid[uid][1]["id"], px, py, pz), o
    assert not o["isWaypointMode"] and o["activeDistance"] == 0.0 and o.get("moveSpeed", 0.0) == 0.0 and \
        o.get("speed", 0.0) == 0.0, o
assert not any(w["pathName"].endswith(f" - {u}") for w in d["waypointDataList"] for u in by_uid)
assert d["spawnPointList"] == [] and d["vehicleList"] == [] and pool["scene"]["spawn"] == []   # world_director spawns
cat_of = lambda a: "movers" if a["zone"].endswith("_dynamic") else "parked_cars" if a["zone"].endswith("_parked") \
    else "static"  # noqa: E731
used = collections.Counter()


def layouts(rng):
    for _ in range(600):
        meta = sv.layout(route, rng, pool)
        assert meta["gen"] == sv.GEN and meta["tunnel"] in ("dynamic", "both"), meta["tunnel"]
        acts = meta["actors"]
        uids = [a["uid"] for a in acts]
        assert len(set(uids)) == len(uids), uids
        for a in acts:
            cat, m = by_uid[a["uid"]]
            assert cat == cat_of(a) and m["id"] == a["id"] and (cat != "movers" or m["ped"] == a["ped"]), (a, m)
            assert isinstance(a["z"], float) and -180 <= a["yaw"] <= 180, a
            used[cat] += 1
            if cat == "static":
                assert min(abs((a["yaw"] - route.at(a["s"] + ds, a["lat"])[3] - a["dyaw"] + 180) % 360 - 180)
                           for ds in (-0.05, 0.0, 0.05)) < 0.1, a   # s rounded to 0.1 m, the path's kinks up to 2 deg
            if cat != "movers":
                continue
            path = a["path"]
            plen = sum(math.dist(p, q) for p, q in zip(path, path[1:]))
            assert path[0] == a["xy"] and path[-1] == a["xy_end"] and len(path) >= 2, a
            assert 0 <= a["corr_in_m"] <= a["corr_out_m"] <= plen + 0.02, (a, plen)
            lo, hi = sv.LEAD[a["timing"]]
            assert lo <= a["lead"] <= hi, a
            assert min(a["s"], a["s_end"]) - 0.1 <= a["s_cross"] <= max(a["s"], a["s_end"]) + 0.1, a
            assert abs(a["corr_in_m"] - a["t_in"] * a["kph"] / 3.6) < 0.3, a   # the timing's own entry distance
            corr = 1.2 + (1.05 if a["cls"] == "car" else 0.35)
            for m_ in (a["corr_in_m"], a["corr_out_m"]):   # on the corridor's edge where it says it enters / leaves
                assert abs(abs(lat_near(at_dist(path, m_), a["s"])) - corr) < 0.15, (a, m_)
            hd = math.degrees(math.atan2(path[-1][1] - path[0][1], path[-1][0] - path[0][0]))
            assert abs((a["yaw"] - hd + 180) % 360 - 180) < 0.5, a
        yield [dict(a, yaw=a["dyaw"]) if "dyaw" in a else a for a in acts]   # make()'s yaw for run()'s footprints


print("layout ok:", *run(layouts(random.Random(3))), dict(used),
      f"pool {len(by_uid)} objects, parked {near:.0f} m+ from the route, {spacing:.2f} m apart (>= {gap:.2f})")
