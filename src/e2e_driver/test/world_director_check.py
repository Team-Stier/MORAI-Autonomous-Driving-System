"""world_director offline: a fake MORAI that applies SetTransform, spawns / drives / despawns ring vehicles the way the
5090 box did on 10/29, drives highway vehicles along their lane's links (current_link_info) and on past their route end,
and raises on chosen calls. Checks the lap layout, the movers' timing against Ego speed profiles (measured from the
poses the fake received, not from the director's own events), the roundabout sets and the highway lanes per density.

    python3 test/world_director_check.py
"""
import json
import math
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "tools"))
import scenario_variants as sv  # noqa: E402
import world_director as wd  # noqa: E402

DT = 0.05
SPAWN = (wd.RING_SPAWN["location"]["x"], wd.RING_SPAWN["location"]["y"])
SPAWN_A = math.atan2(SPAWN[1] - wd.RING_C[1], SPAWN[0] - wd.RING_C[0])
LANE_R, REUSE_XY, CLEAR_M = 19.9, (69.3, 309.7), 15.0   # a spawn is refused while a car is within CLEAR_M
# the fake's highway lanes: straight on from the point along its yaw, every link HW_LINK_WP waypoints HW_WP_M apart
HW_WP_M, HW_LINK_WP, HW_REUSE = 0.5, 300, (-300.0, -300.0)   # HW_REUSE: where a vanished highway car's id shows again


def hw_end(h):
    """travel (m from the lane's first waypoint) of highway point h's endLinkInfo"""
    return ((len(h["route"]) - 1) * HW_LINK_WP + h["end"][1]) * HW_WP_M


class FakeSim:
    def __init__(self):
        self.tau, self.calls, self.count, self.fail = 0.0, [], {}, {}
        self.pose, self.track = {}, {}             # scene uid -> pose; uid -> [(tau, x, y)]
        self.cars, self.reused, self.gone = {}, {}, set()
        self.block_until, self.no_vanish, self.vanish, self.bad = 0.0, set(), set(), []
        self.spawn_raise, self.stray, self.over = set(), set(), {}   # car ids: created but the call raises; highway
        # cars that turn off their route halfway; highway id -> m driven past its route end

    def call(self, method, req, svc=None):
        n = self.count[method] = self.count.get(method, 0) + 1
        aid = (req.get("actor_info") or req.get("spawn_info", {}).get("actor_info") or req).get("id", {}).get("value")
        self.calls.append((self.tau, method, aid, req))
        if aid in self.gone and method != "GetAllActorsState":
            self.bad.append((self.tau, method, aid))   # touching an id MORAI may have reused
        if n in self.fail.get(method, ()):
            raise RuntimeError(f"fake: Exception was thrown by handler ({method} #{n})")
        if method == "SetTransform":
            loc, rot = req["transform"]["location"], req["transform"]["rotation"]
            self.pose[aid] = (loc["x"], loc["y"], loc["z"], rot["z"])
            self.track.setdefault(aid, []).append((self.tau, loc["x"], loc["y"]))
            return {}
        if method == "SpawnVehicle":
            loc = req["spawn_info"]["transform"]["location"]
            at = (loc["x"], loc["y"])
            hw = next((h for h in wd.HIGHWAY if math.dist(h["pose"][:2], at) < 0.01), None)
            if hw is None and math.dist(at, SPAWN) > 0.01:
                raise RuntimeError("fake: Exception was thrown by handler (no spawn point there)")   # as the ring did
            taken = self.tau < self.block_until or any(math.dist(self.xy(c), at) < CLEAR_M for c in self.cars.values())
            if taken:
                return {"status": 7, "description": "NOT_CREATE_OBJECT"}
            self.cars[aid] = dict(model=req["spawn_info"]["model_name"], hw=hw, a=SPAWN_A, r=math.dist(SPAWN, wd.RING_C),
                                  travel=hw["route"][0][1] * HW_WP_M if hw else 0.0, v=None, phys=False, paused=True,
                                  route=None, hist=[("spawn", self.tau)])
            if aid in self.spawn_raise:
                raise RuntimeError("fake: deadline exceeded (SpawnVehicle went through)")
            return {"status": 1, "description": "SUCCESS"}
        c = self.cars.get(aid)
        if method == "GetAllActorsState":
            st = [(i, *self.xy(c), *(self.link(i, c) if c["hw"] else (None, 0))) for i, c in self.cars.items()]
            st += [(i, *p) for i, p in self.reused.items()]
            st += [(u, *p[:2], None, 0) for u, p in list(self.pose.items())[:5]]   # a few scene objects: skipped
            return {"states": [dict(actor_info={"id": {"value": i}, "object_type": 1},
                                    transform={"location": {"x": x, "y": y, "z": 28.6}},
                                    # MessageToDict leaves out a 0 waypoint_idx
                                    **({"vehicle_state": {"current_link_info": {"id": {"value": li}, **(
                                        {"waypoint_idx": w} if w else {})}}} if li else {}))
                               for i, x, y, li, w in st]}
        if method == "DestroyActor":
            if aid in self.reused:
                self.bad.append((self.tau, "destroyed a reused id", aid))
            if c is None:
                raise RuntimeError("fake: no such actor")
            del self.cars[aid]
            return {}
        if c is None:
            raise RuntimeError(f"fake: {method} on unknown actor {aid}")
        c["hist"].append((method, self.tau))
        if method == "SetPhysics":
            c["phys"] = req["enable"]
        elif method == "SetPause":
            c["paused"] = req["enable"]
        elif method == "SetVehicleDynamicsSpeedLimit":
            c["v"] = req["speed_limit"]
        elif method in ("SetVehicleRoute", "SetVehicleDestination"):
            if c["route"] is not None or method == "SetVehicleDestination":
                self.bad.append((self.tau, f"{method} again: MORAI despawns it", aid))
            c["route"] = req.get("links")
        return {"status": 1}

    @staticmethod
    def xy(c):
        if c["hw"]:
            x, y, _, yaw = c["hw"]["pose"]
            d = c["travel"] - c["hw"]["route"][0][1] * HW_WP_M
            return x + d * math.cos(math.radians(yaw)), y + d * math.sin(math.radians(yaw))
        return wd.RING_C[0] + c["r"] * math.cos(c["a"]), wd.RING_C[1] + c["r"] * math.sin(c["a"])

    def link(self, i, c):
        """-> (link id, waypoint idx) of highway car c: its lane's links by travel, on past them off the route"""
        if i in self.stray and c["travel"] > hw_end(c["hw"]) / 2:
            return "A2256W_STRAY", 0
        k, wp = divmod(int(c["travel"] / HW_WP_M), HW_LINK_WP)
        return (c["hw"]["route"][k][0] if k < len(c["hw"]["route"]) else "A2256W_AFTER"), wp

    def advance(self, dt):
        self.tau += dt
        for i, c in list(self.cars.items()):
            if c["phys"] and not c["paused"] and c["v"] and c["route"]:
                c["travel"] += c["v"] * dt
                if c["hw"]:
                    past = c["travel"] - hw_end(c["hw"])
                    if past > 0 and i in self.vanish:                      # removed at its route end, its id
                        del self.cars[i]                                   # listed again far off on the lane's first link
                        self.gone.add(i)
                        self.reused[i] = (*HW_REUSE, c["hw"]["route"][0][0], 50)
                    elif past > 0:                                         # drives on (a gRPC route car, 10/29)
                        self.over[i] = max(self.over.get(i, 0.0), past)
                elif c["travel"] < 0.75 * 2 * math.pi * LANE_R or i in self.no_vanish:   # RING_ROUTE: 3/4 ring
                    c["a"] += c["v"] / LANE_R * dt
                    c["r"] = LANE_R
                elif i in self.vanish:                                    # removed at its route end
                    del self.cars[i]
                    self.gone.add(i)
                    self.reused[i] = (*REUSE_XY, None, 0)                 # MORAI hands the id to highway traffic
                else:
                    c["r"] += c["v"] * dt                                 # drives on off the ring (the 5090 box, 10/29)


# ----- lap layout
def check_new_lap():
    sim = FakeSim()
    pool = {"parking": [[1000 + i, -300.0 - 3 * i, 500.0, 30.0, 90.0] for i in range(8)], "static": [], "movers": [],
            "parked_cars": []}
    d = wd.Director(sim, None, pool, random.Random(1))
    st = lambda uid, x, y, zone="static": dict(uid=uid, zone=zone, xy=[x, y], z=31.0, yaw=12.0)
    mv = lambda uid, x, y: dict(uid=uid, zone="ra_dynamic", xy=[x, y], z=31.5, yaw=0.0, ped=True, cls="person",
                                path=[[x, y], [x, y + 10.0]], kph=5.0, timing="must_stop", lead=1.5, corr_in_m=4.0,
                                corr_out_m=7.0, s_cross=480.0)
    d.new_lap(dict(actors=[st(1000, 10, 20), st(1001, 11, 21, "ra_parked"), mv(1002, 12, 22), mv(1003, 13, 23)]))
    lap1 = sorted(c[2] for c in sim.calls if c[1] == "SetTransform")
    assert lap1 == ["1000", "1001", "1002", "1003"], lap1
    assert sim.pose["1000"] == (10, 20, 31.0, 12.0) and sim.pose["1001"] == (11, 21, 31.0, 12.0)
    assert sim.pose["1002"] == (12, 22, 31.5, 90.0), sim.pose["1002"]          # path[0], facing along the path
    assert sim.calls[2][3]["actor_info"]["object_type"] == wd.PED and sim.calls[0][3]["actor_info"]["object_type"] == wd.OBS
    for k in range(40):                            # movers walk off (the Ego just before their crossing)
        sim.tau = k * DT
        d.step(sim.tau, dict(x=0, y=0, yaw=0, v=8.0, s=480.0 - wd.EGO_FRONT_M - 8.0 * 1.6))
    assert sim.pose["1002"][1] > 22.5, sim.pose["1002"]
    n0 = len(sim.calls)
    sim.fail["SetTransform"] = {sim.count["SetTransform"] + 1}   # lap 2's first one (1004) fails: retried next step
    ev = d.new_lap(dict(actors=[st(1000, 10, 20), st(1004, 14, 24)]))       # 1000 unchanged; 1001-1003 back to parking
    moved = [c[2] for c in sim.calls[n0:] if c[1] == "SetTransform"]
    assert sorted(moved) == ["1001", "1002", "1003", "1004"], moved
    assert any(e["event"] == "error" and e["call"] == "SetTransform" for e in ev)
    assert "1004" not in sim.pose
    sim.tau += DT
    d.step(sim.tau, dict(x=0, y=0, yaw=0, v=8.0, s=0.0))
    assert sim.pose["1004"] == (14, 24, 31.0, 12.0), "failed placement not retried"
    park = {str(u): (x, y, z, yaw) for u, x, y, z, yaw in pool["parking"]}
    for u in ("1001", "1002", "1003"):
        assert sim.pose[u] == park[u], (u, sim.pose[u])
    n1 = len(sim.calls)
    d.new_lap(dict(actors=[st(1000, 10, 20), st(1004, 14, 24)]))
    assert not [c for c in sim.calls[n1:] if c[1] == "SetTransform"], "an unchanged lap moved something"
    print(f"new_lap: lap 1 placed {lap1}; lap 2 moved {sorted(moved)} (1000 unchanged, 1004 retried after an error); "
          "lap 3 identical: 0 calls  OK")


# ----- movers
# (name, s0, lat0, lat_end, kph, ped, cls): v2 layout shapes (scenario_variants r2_actor): ra s 470-500, rb tunnel
SHAPES = [("near_ped", 480.0, -6.6, 10.0, 5.0, True, "person"), ("far_ped", 485.0, 11.0, -6.0, 4.0, True, "person"),
          ("slow_ped", 478.0, 10.5, -6.2, 3.0, True, "person"), ("centre_prop", 475.0, 2.78, -6.0, 7.0, False, "other"),
          ("car_prop", 490.0, -4.0, 10.0, 20.0, False, "car"), ("tunnel_ped", 1950.0, 5.8, -2.25, 4.0, True, "person")]
LEADS = {k: (lo, (lo + hi) / 2, hi) for k, (lo, hi) in sv.LEAD.items()}   # each end and the middle


def profiles():
    """Ego speed rules v(t, v, rem front-to-crossing m) -> new v"""
    def const(vc):
        return lambda st, v, rem: vc

    # 8.5 m/s, brakes to 0 60 m out (movers on their way, none in the lane yet), stands 4 s, back to 8.5
    def brake(st, v, rem):
        if st.setdefault("ph", "cruise") == "cruise" and rem <= 60.0:
            st["ph"] = "brake"
        if st["ph"] == "brake":
            v = max(v - 2.5 * DT, 0.0)
            if v == 0.0:
                st["ph"], st["t"] = "stand", 0.0
        elif st["ph"] == "stand":
            st["t"] += DT
            if st["t"] >= 4.0:
                st["ph"] = "go"
        elif st["ph"] == "go":
            v = min(v + 1.5 * DT, 8.5)
        return v if st["ph"] != "cruise" else 8.5

    def accel(st, v, rem):                         # 5 m/s, then 1 m/s^2 up to 16 from 120 m out
        return min(v + 1.0 * DT, 16.0) if rem <= 120.0 else 5.0
    return [("3.0", 3.0, const(3.0)), ("8.5", 8.5, const(8.5)), ("12.0", 12.0, const(12.0)), ("16.7", 16.7, const(16.7)),
            ("brake 8.5->0->8.5", 8.5, brake), ("accel 5->16", 5.0, accel)]


def mover_case(route, uid, shape, timing, lead):
    name, s0, lat0, lat_end, kph, ped, cls = shape
    ds = 1.0
    (x0, y0, z, h), (x1, y1, _, _) = route.at(s0, lat0), route.at(s0 + ds, lat_end)
    corr = 1.2 + (1.05 if cls == "car" else 0.35)
    L, span = math.hypot(x1 - x0, y1 - y0), abs(lat0 - lat_end)
    a = dict(uid=uid, zone="rb_dynamic" if s0 > 1800 else "ra_dynamic", xy=[x0, y0], z=z, yaw=h, ped=ped, cls=cls,
             path=[[x0, y0], [x1, y1]], kph=kph, timing=timing, lead=lead, corr_in_m=L * (abs(lat0) - corr) / span,
             corr_out_m=L * (abs(lat0) + corr) / span, s_cross=s0 + ds * abs(lat0) / span)
    cx, cy, _, ch = route.at(a["s_cross"], 0.0)
    hr = math.radians(ch)
    lat = lambda x, y: -(x - cx) * math.sin(hr) + (y - cy) * math.cos(hr)   # the checker's own corridor test
    return a, corr, lat


def run_mover(route, d, sim, shape, timing, lead, prof, uid):
    a, corr, lat = mover_case(route, uid, shape, timing, lead)
    d.new_lap(dict(actors=[a]))
    _, v0, rule = prof
    s, v, st, log = a["s_cross"] - wd.EGO_FRONT_M - 200.0, v0, {}, []
    while s < a["s_cross"] + 30.0 and sim.tau < 1e6:
        rem = a["s_cross"] - s - wd.EGO_FRONT_M
        x, y, _, h = route.at(s, 0.0)
        d.step(sim.tau, dict(x=x, y=y, yaw=math.radians(h), v=v, s=s))
        log.append((sim.tau, rem / max(v, 0.5), rem, v))
        v = rule(st, v, rem)
        s += v * DT
        sim.advance(DT)
    # the mover's pose at each step from the SetTransforms the fake got
    tr, k, pos = sim.track.get(str(uid), []), 0, []
    last = tuple(a["path"][0])
    for tau, *_ in log:
        while k < len(tr) and tr[k][0] <= tau + 1e-9:
            last = tr[k][1:]
            k += 1
        pos.append(last)
    lats = [lat(*p) for p in pos]
    i_in = next((i for i, q in enumerate(lats) if abs(q) <= corr), None)
    i_out = next((i for i in range(i_in or 0, len(lats)) if abs(lats[i]) > corr), None) if i_in is not None else None
    end, vmax = tuple(a["path"][-1]), max(a["kph"] / 3.6, wd.V_SPRINT.get(a["cls"], wd.SPRINT_M_S))
    i_end = next((i for i, p in enumerate(pos) if math.dist(p, end) < 1e-6), len(pos))
    sp = [math.dist(pos[i], pos[i - 1]) / DT for i in range(1, i_end)]
    acc = [abs(b - c) / DT for b, c in zip(sp[1:], sp)]
    i_ev = i_in if timing == "must_stop" else i_out
    t_ev = log[i_ev][1] if i_ev is not None else None
    rem_ev = log[i_ev][2] if i_ev is not None else None
    ok = t_ev is not None and abs(t_ev - lead) <= 0.3 and rem_ev > 0 and max(sp, default=0) <= vmax + 1e-6 \
        and max(acc, default=0) <= wd.A_MAX + 1e-6
    return dict(shape=shape[0], timing=timing, lead=lead, t=t_ev, rem=rem_ev, vmax=max(sp, default=0) / vmax,
                amax=max(acc, default=0), ok=ok, done=i_end < len(pos))


def check_movers():
    route = sv.Route(os.path.join(HERE, "..", "..", "map", "2026_molit_comp_global_path.txt"))
    fails, uid = [], 2000
    print("movers: entry (must_stop) / exit (pass_front) T = Ego front time to the crossing at its speed, from the fake's"
          " poses; tolerance lead +-0.3 s")
    print(f"  {'Ego profile':18s} {'runs':>4s} {'must_stop T-lead':>22s} {'pass_front T-lead':>22s} {'v/vmax':>7s}"
          f" {'|a| max':>8s} {'path end':>8s}")
    for prof in profiles():
        sim = FakeSim()
        d = wd.Director(sim, route, {"parking": []}, random.Random(3))
        res = []
        for shape in SHAPES:
            for timing, leads in LEADS.items():
                for lead in leads:
                    uid += 1
                    res.append(run_mover(route, d, sim, shape, timing, lead, prof, uid))
        fails += [(prof[0], r) for r in res if not r["ok"]]
        rng = lambda tm: [r["t"] - r["lead"] for r in res if r["timing"] == tm and r["t"] is not None]
        fmt = lambda xs: f"{min(xs):+.2f} .. {max(xs):+.2f} s" if xs else "none"
        print(f"  {prof[0]:18s} {len(res):4d} {fmt(rng('must_stop')):>22s} {fmt(rng('pass_front')):>22s}"
              f" {max(r['vmax'] for r in res):7.2f} {max(r['amax'] for r in res):8.2f}"
              f" {sum(r['done'] for r in res):4d}/{len(res)}")
        ev = [e for e in d.flush() if e.get("event") in ("missed", "error")]
        assert not ev, ev
    for p, r in fails:
        print("  FAIL", p, r)
    assert not fails, f"{len(fails)} mover runs out of tolerance"
    print("  OK: every mover met its timing, before the Ego, within its speed and acceleration limits")


# ----- roundabout
def check_ring():
    sim = FakeSim()
    models = [f"model_{i}" for i in range(6)]
    d = wd.Director(sim, None, {"parking": []}, random.Random(7), models)
    sim.block_until = 1.0                          # the spawn point taken at first: retried each step
    sim.fail = {"SetPhysics": {1}, "GetAllActorsState": {40}}
    sim.spawn_raise = {"wd4"}                      # one spawn goes through but its call raises
    sim.no_vanish = {"wd2"}                        # one car that stays on the ring past its route end
    sim.vanish = {"wd1", "wd3"}                    # two MORAI removes at their route end and reuses the id of
    events, route = [], [{"id": {"value": link}, "waypoint_idx": w} for link, w in wd.RING_ROUTE]
    print("ring: per lap one model / period / speed; cars every period from the last spawn, refused spawns retried")
    for lap in range(3):
        mine = set(sim.cars)
        ev = d.new_lap(dict(actors=[]))
        events += ev
        assert not set(sim.cars) & mine, f"lap {lap}: last lap's cars still on the ring"
        cfg = next(e for e in ev if e["event"] == "ring_lap")
        lo, hi = wd.RING_PERIOD[cfg["density"]]
        assert cfg["model"] in models and lo <= cfg["period_s"] <= hi and wd.RING_KPH[0] <= cfg["kph"] <= wd.RING_KPH[1], cfg
        lap_ev, t_end = [], sim.tau + 150.0
        while sim.tau < t_end:
            lap_ev += d.step(sim.tau, dict(x=0, y=0, yaw=0, v=0, s=0))
            sim.advance(DT)
        events += lap_ev
        sp = [e for e in lap_ev if e["event"] == "ring_spawn"]
        gaps = [b["tau"] - a["tau"] for a, b in zip(sp, sp[1:])]
        for g, b in zip(gaps, sp[1:]):             # every period, later only while the point was taken
            assert g >= cfg["period_s"] - 1e-9 and (b["refused"] > 0 or g <= cfg["period_s"] + DT + 1e-9), (g, b, cfg)
        for e in sp:
            c = {q[1]: q[3] for q in sim.calls if q[2] == e["car"]}   # the last call of each kind
            assert c["SpawnVehicle"]["spawn_info"]["model_name"] == cfg["model"], (e, c)
            assert abs(c.get("SetVehicleDynamicsSpeedLimit", {}).get("speed_limit", cfg["kph"] / 3.6) - cfg["kph"] / 3.6) < 1e-9
        print(f"  lap {lap}: {cfg['model']} every {cfg['period_s']} s at {cfg['kph']} kph -> {len(sp)} cars, gaps "
              f"{min(gaps):.2f}-{max(gaps):.2f} s, refused spawns {sum(e['refused'] for e in sp)}, last lap's cars "
              f"destroyed {[e['car'] for e in ev if e['event'] == 'ring_destroy']}")
    spawns = [e["car"] for e in events if e["event"] == "ring_spawn"]
    assert events[[e["event"] for e in events].index("ring_spawn")]["refused"] > 0, "no retry while the point was taken"
    for cid in spawns:                             # spawn, physics, unpause, cruise, one route; never a destination
        names = [c[1] for c in sim.calls if c[2] == cid and c[1] not in ("GetAllActorsState", "DestroyActor")]
        dedup = [n for k, n in enumerate(names) if k == 0 or n != names[k - 1]]   # retries of a failed call
        assert dedup == ["SpawnVehicle", "SetPhysics", "SetPause", "SetVehicleDynamicsSpeedLimit", "SetVehicleRoute"], (cid, names)
        routes = [c[3]["links"] for c in sim.calls if c[2] == cid and c[1] == "SetVehicleRoute"]
        assert routes == [route], (cid, routes)
    done = [e["car"] for e in events if e["event"] == "ring_done"]
    exits = [e["car"] for e in events if e["event"] == "ring_exit"]
    assert "wd2" not in done + exits and any(e["event"] == "ring_destroy" and e["car"] == "wd2" for e in events)
    assert {"wd1", "wd3"} <= set(done) and not {"wd1", "wd3"} & set(exits), (done, exits)
    far = [(i, round(c["r"], 1)) for i, c in sim.cars.items() if not c["hw"] and c["r"] > wd.RING_EXIT + 1.0]
    assert exits and not far, ("cars driving on off the ring", far)   # removed within a step of leaving it
    assert not sim.bad, sim.bad[:5]
    assert any(e["event"] == "error" and e["call"] == "SetPhysics" for e in events)
    assert sim.cars.get("wd4", {}).get("route") or "wd4" in done + exits, "the car whose spawn call raised never drove"
    print(f"  {len(spawns)} cars: {len(exits)} destroyed on leaving the ring, {len(done)} gone at their route end and never "
          f"touched again ({len(sim.reused)} ids reused far off by the fake), wd2 (stayed on) destroyed at the next lap; injected SetPhysics / "
          f"GetAllActorsState / SpawnVehicle errors survived  OK")
    alive = sorted(sim.cars)
    left = d.close()
    assert alive and not sim.cars and left == [] and not sim.bad, (alive, sim.cars, left, sim.bad)
    d.step(sim.tau + 20, dict(x=0, y=0, yaw=0, v=0, s=0))
    assert not sim.cars, "spawned after close()"
    print(f"  close(): destroyed {alive}, left {left}, no spawn after it; reused ids not destroyed  OK")
    d2 = wd.Director(FakeSim(), None, {"parking": []}, random.Random(1))
    cfg = next(e for e in d2.new_lap(dict(actors=[])) if e["event"] == "ring_lap")
    assert cfg["model"] in wd.RING_MODELS, cfg
    print(f"  default models: RING_MODELS ({len(wd.RING_MODELS)}), drew {cfg['model']}  OK")


# ----- highway and density
def check_highway():
    sim = FakeSim()
    models = [f"model_{i}" for i in range(6)]
    d = wd.Director(sim, None, {"parking": []}, random.Random(11), models)
    sim.vanish, sim.stray = {"wh2"}, {"wh3"}       # one removed at its route end (id reused far off), one turns off
    sim.fail = {"SetVehicleRoute": {3}, "DestroyActor": {2}}
    modes = [wd.lap_density("alternate", k) for k in range(4)]
    assert modes == ["low", "high", "low", "high"] and wd.lap_density("high", 0) == "high" and \
        wd.lap_density("low", 1) == "low", modes
    pts = {h["sp"]: h for h in wd.HIGHWAY}
    print("highway + density: --density alternate laps", modes, "; per lane one speed per lap, a period per car")
    events, n_by = [], {"low": [], "high": []}
    for lap, dens in enumerate(modes):
        mine = set(sim.cars)
        ev = d.new_lap(dict(actors=[]), dens)
        assert not set(sim.cars) & mine, f"lap {lap}: last lap's cars still there"
        ring = next(e for e in ev if e["event"] == "ring_lap")
        hwl = next(e for e in ev if e["event"] == "highway_lap")
        lo, hi = wd.RING_PERIOD[dens]
        assert ring["density"] == hwl["density"] == dens and lo <= ring["period_s"] <= hi and ring["model"] in models \
            and wd.RING_KPH[0] <= ring["kph"] <= wd.RING_KPH[1], ring
        kph = hwl["kph"]
        assert hwl["period_s"] == list(wd.HW_PERIOD[dens]) and sorted(kph) == sorted(pts) \
            and all(wd.HW_KPH[0] <= v <= wd.HW_KPH[1] for v in kph.values()), hwl
        lap_ev, t_end = [], sim.tau + 150.0
        while sim.tau < t_end:
            lap_ev += d.step(sim.tau, dict(x=0, y=0, yaw=0, v=0, s=0))
            sim.advance(DT)
        events += ev + lap_ev
        rs = [e["tau"] for e in lap_ev if e["event"] == "ring_spawn"]
        assert all(b - a >= ring["period_s"] - 1e-9 for a, b in zip(rs, rs[1:])), (rs, ring)
        row = []
        for sp, h in pts.items():
            sps = [e for e in lap_ev if e["event"] == "highway_spawn" and e["sp"] == sp]
            assert len(sps) >= 5, (lap, sp, len(sps))
            plo, phi = wd.HW_PERIOD[dens]
            for a, b in zip(sps, sps[1:]):         # each a fresh period from the last spawn, later only while taken
                g = b["tau"] - a["tau"]
                assert plo <= a["next_s"] <= phi and g >= a["next_s"] - 1e-9 and \
                    (b["refused"] > 0 or g <= a["next_s"] + DT + 1e-9), (lap, sp, a, b, g)
            for e in sps:
                c = {q[1]: q[3] for q in sim.calls if q[2] == e["car"]}   # the last call of each kind
                assert e["model"] in models and c["SpawnVehicle"]["spawn_info"]["model_name"] == e["model"], (e, c)
                assert c["SpawnVehicle"]["spawn_info"]["transform"] == wd.transform(*h["pose"]), c["SpawnVehicle"]
                assert e["kph"] == kph[sp] and abs(c["SetVehicleDynamicsSpeedLimit"]["speed_limit"] - kph[sp] / 3.6) < 1e-9
                names = [q[1] for q in sim.calls if q[2] == e["car"] and q[1] not in ("GetAllActorsState", "DestroyActor")]
                dedup = [x for k, x in enumerate(names) if k == 0 or x != names[k - 1]]   # retries of a failed call
                assert dedup == ["SpawnVehicle", "SetPhysics", "SetPause", "SetVehicleDynamicsSpeedLimit", "SetVehicleRoute"], names
                routes = [q[3]["links"] for q in sim.calls if q[2] == e["car"] and q[1] == "SetVehicleRoute"]
                want = [{"id": {"value": li}, "waypoint_idx": w} for li, w in h["route"]]
                assert routes and all(r == want for r in routes), (e, routes)   # >1: a failed call retried
            gaps = [b["tau"] - a["tau"] for a, b in zip(sps, sps[1:])]
            n_by[dens].append(len(sps))
            row.append(f"{sp}: {kph[sp]} kph {len(sps)} cars gaps {min(gaps):.1f}-{max(gaps):.1f} s"
                       f" refused {sum(e['refused'] for e in sps)}")
        print(f"  lap {lap} {dens:4s}: ring {ring['model']} every {ring['period_s']} s at {ring['kph']} kph, {len(rs)} cars;"
              f" highway {'; '.join(row)}")
    assert min(n_by["high"]) > max(n_by["low"]), n_by
    ex = {e["car"] for e in events if e["event"] == "highway_exit"}
    done = {e["car"] for e in events if e["event"] == "highway_done"}
    spawned = [e["car"] for e in events if e["event"] == "highway_spawn"]
    assert "wh2" in done and "wh2" not in ex and "wh3" in ex and "wh3" not in sim.over, (sorted(done), sorted(ex))
    over = max(sim.over.values())
    assert len(ex) > len(spawned) / 2 and over < 3.5, (len(ex), len(spawned), over)   # within a step or two of the end
    assert not sim.bad, sim.bad[:5]
    assert any(e["event"] == "error" and e["call"] == "SetVehicleRoute" for e in events)
    print(f"  {len(spawned)} highway cars: {len(ex)} destroyed at their route end or off it (wh3 turned off halfway), at most"
          f" {over:.2f} m past the end; wh2 removed by MORAI, its id reused far off and never touched; high laps"
          f" {n_by['high']} vs low {n_by['low']} cars per lane; injected SetVehicleRoute / DestroyActor errors retried  OK")
    alive = sorted(sim.cars)
    left = d.close()
    assert any(i.startswith("wh") for i in alive) and not sim.cars and left == [] and not sim.bad, (alive, sim.cars, left)
    d.step(sim.tau + 20, dict(x=0, y=0, yaw=0, v=0, s=0))
    assert not sim.cars, "spawned after close()"
    print(f"  close(): destroyed {len(alive)} cars ({sum(i.startswith('wh') for i in alive)} highway), left {left}, no spawn"
          " after it  OK")


# ----- end to end: scenario_variants.pool_scene + layout (the stand-in base of scene_preset_check.py)
def check_layouts(laps=30):
    route = sv.Route(os.path.join(HERE, "..", "..", "map", "2026_molit_comp_global_path.txt"))
    x, y, z, _ = route.at(329.0, -0.5)
    sx, sy, _, _ = route.at(1930.0, 0.0)
    spawn = lambda uid, px, py: dict(UNIQUEID=uid, pos={"x": px, "y": py}, spawnVehicleTypeID=-1)  # noqa: E731
    base = dict(egoVehicle={"initPosition": {"pos": {}, "rot": {}}}, tlList=[], waypointDataList=[],
                spawnPointList=[spawn(4, -111.3, 358.3), spawn(10, 73.0, 265.6)],
                objectList=[{"pos": {"x": x, "y": y, "z": z}}], pedestrianList=[{}],
                shadedAreaList=[{"pos": {"x": sx, "y": sy, "z": 0.0}, "size": {"x": 117.959}}])
    rng = random.Random(5)
    _, pool = sv.pool_scene(base, route, rng)
    sim = FakeSim()
    d = wd.Director(sim, route, json.loads(json.dumps(pool)), random.Random(6))
    park = {str(u): tuple(p) for u, *p in pool["parking"]}
    errs, early, n_mov, n_stat, stops = [], [], 0, 0, 0
    for lap in range(laps):
        meta = sv.layout(route, rng, pool)
        d.new_lap(meta)
        lay = {str(a["uid"]): a for a in meta["actors"]}
        for u, p in park.items():                  # this lap's statics in place, movers at path[0], the rest parked
            a = lay.get(u)
            want = p if a is None else tuple(a["path"][0]) if "path" in a else (*a["xy"], a["z"], a["yaw"])
            assert sim.pose.get(u, p)[:len(want)] == want, (lap, u, sim.pose.get(u), want)
        vc, s, v, stop = rng.uniform(3.0, 16.7), 250.0, 0.0, rng.random() < 0.5   # half the laps: a red light at s 433
        stops += stop
        ev, ph, t0 = [], "go", 0.0
        while s < 2100.0:
            ev += d.step(sim.tau, dict(x=0.0, y=0.0, yaw=0.0, v=v, s=s))
            if stop and ph == "go" and s < 433.0 <= s + v * v / 5.0:   # 2.5 m/s^2 to stand at the stop line
                ph = "brake"
            if ph == "brake":
                v = max(v - 2.5 * DT, 0.0)
                ph, t0 = ("stand", sim.tau) if v == 0.0 else (ph, t0)
            elif ph != "stand" or sim.tau - t0 > 5.0:
                ph, v = "went" if ph == "stand" else ph, min(v + 1.5 * DT, vc)
            s += v * DT
            sim.advance(DT)
        for a in meta["actors"]:
            if "path" not in a:
                n_stat += 1
                continue
            n_mov += 1
            key = "enter" if a["timing"] == "must_stop" else "exit"
            e = [q for q in ev if q.get("uid") == str(a["uid"]) and q["event"] in (key, "missed")]
            row = (lap, a["zone"], a["ptype"], a["timing"], a["lead"], round(vc, 1), stop, e[:1])
            if not e or e[0]["event"] == "missed" or e[0]["ego_t"] < a["lead"] - 0.3:
                errs.append(row)                   # late, or never: the Ego got there first
            elif e[0]["ego_t"] > a["lead"] + 0.3:
                early.append(row)                  # the Ego slowed while it was already crossing: it went on through
    for e in errs:
        print("  FAIL", e)
    for e in early:
        print("  early", e)
    print(f"end to end: {laps} laps of layout() over pool_scene ({len(pool['parking'])} pool objects), Ego 3-16.7 m/s, "
          f"{stops} laps with a 5 s red-light stop at s 433: {n_stat} statics placed / the rest parked, {n_mov} movers: "
          f"{n_mov - len(errs) - len(early)} on time (+-0.3 s), {len(early)} early (the Ego braked for the light while "
          f"they crossed), {len(errs)} late or missed")
    assert not errs and all(e[6] for e in early)


if __name__ == "__main__":
    check_new_lap()
    check_movers()
    check_ring()
    check_highway()
    check_layouts()
    print("all checks passed")
