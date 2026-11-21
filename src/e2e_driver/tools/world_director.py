#!/usr/bin/env python3
"""World director for closed-loop laps on one pool scene (user 10/29): the run loads a scene with every obstacle asset
parked off the route (scenario_variants.pool_scene) and never reloads it; per lap this moves the lap's obstacles in and
the rest out, times the dynamic obstacles on the Ego and runs the roundabout's spawn point, all over gRPC.

    d = Director(sim, route, pool, random.Random(seed), models)    sim: eval_manager.Sim (Actor service)
    d.new_lap(meta, density)   meta = scenario_variants.layout(...): statics / parked cars to their poses, movers to path[0],
                               last lap's cars destroyed, this lap's ring model / period / speed and highway lane speeds
                               drawn for density low / high (lap_density(): --density alternate = lap 0 low, 1 high, ...)
    events = d.step(tau, ego)  every step; ego = dict(x, y, yaw rad, v m/s, s route s of base_link)
    d.close()                  before any scene reload (a reload with spawned vehicles left crashed the simulator, 10/29)

gRPC as measured on a 5090 box (MORAI 25.S4, 10/29). Scene objects are actors str(UNIQUEID); only SetTransform moves them
(ControlPedestrian / ControlObstacle / SetPedestrianWaypoint answer but move nothing, SetPause freezes a pedestrian for
good), so a mover is moved by one SetTransform per step along its path and stays where it was last put.
Movers ("a skilled insurance fraudster"): T = time for the Ego's front to reach the crossing at its measured speed. A
must_stop mover enters the Ego's corridor when T = lead (1.0-2.5 s), a pass_front one leaves it when T = lead (0.3-1.2 s);
each step the speed that meets this from where it is now, counting the time to speed up, is set (start only once it needs
START x its nominal speed, stop at the curb while the Ego is slow or stopped, never stand inside the lane, sprint up to
6 m/s, 12 for car props, at most A_MAX m/s^2 either way). Once inside (must_stop) or through (pass_front) it walks on at
its nominal speed to the end of its path and stays there.
Roundabout (user 10/29, the competition's way): one spawn point with one vehicle model, one spawn period (3-10 s) and one
speed (10-35 kph), drawn per lap and fixed for it; the count follows. A car is spawned at the official ring spawn point
(any other pose threw, and a later reload crashed the simulator) every period from the last spawn, a refused spawn
(NOT_CREATE_OBJECT while the last car is within ~15 m) retried each step, and given one SetVehicleRoute (RING_ROUTE):
MORAI despawns it at the route end. A second route or any SetVehicleDestination despawns a car within ~2 s and MORAI
reuses its id for spawn-point traffic elsewhere, so a car gone from GetAllActorsState (or listed far from the ring) is
done and never destroyed or tracked again.
Highway (user 10/29; MORAI Q&A: speed fixed within a lane and different between lanes, no lane change, type random per
car): the pool scene has no spawn points; each of the base scene's three highway points (HIGHWAY) spawns, the same way
as the ring, a car of a random model every period drawn anew after each car (HW_PERIOD), all at that lane's speed for
the lap (HW_KPH, one draw per lane per lap), each with one SetVehicleRoute along the point's own links. A gRPC route car
drives on past its route end, so one listed past the point's endLinkInfo or on a link off its route (current_link_info)
is destroyed; one listed farther from where it was last seen than it can drive is MORAI's reuse of the id, never touched.
Every call catches its own error: a failed one is logged as an event and retried where that matters, never ending the lap.
"""
import bisect
import math

VEH, PED, OBS = "OBJECT_TYPE_VEHICLE", "OBJECT_TYPE_PEDESTRIAN", "OBJECT_TYPE_OBSTACLE"
# ego.s is base_link progress (eval_manager Track: rear axle); the Ioniq 5's front bumper is ~3.8 m ahead (wheelbase 3.0 +
# ~0.85 overhang). Timing on the rear axle would let a pass_front mover leave the lane after the bumper got there.
EGO_FRONT_M = 3.8
# x nominal speed: start when the timing needs this much (0.6 walked slow movers through the lane for 6 s, still in it
# when the Ego braked); below HOLD wait at the curb
START, HOLD = 1.0, 0.3
# the rate T falls at (1 at a steady Ego speed; ~2 for an Ego pulling away from a stop), clamped, and its smoothing (s).
# 3 overshot: a car prop entered for an Ego still speeding up, which then held its speed, and left 0.6 s early (check)
R_MAX, R_TC = 2.0, 0.3
A_MAX = 5.0                  # m/s^2 either way: a mover never teleports (10/29: 3 -> 5, a dash)
V_SPRINT = {"car": 12.0}     # m/s, top speed by class (others SPRINT_M_S): a fraudster may sprint to keep the timing
SPRINT_M_S = 9.0             # 10/29: movers run 12-25 km/h (the sample 10), so room above it
DT_MAX = 0.3                 # a step longer than this (a hiccup) moves the movers by this much only
RING_C, RING_NEAR = (-99.5, 342.0), 60.0   # a car of ours listed farther from the ring centre is MORAI's reuse of its id
RING_EXIT, JUMP_M = 30.0, 15.0   # m from the centre: off the ring (lane r ~20); a step's move larger than JUMP_M
                                 # is not our car driving but MORAI's reuse of its id
RING_SPAWN = {"location": {"x": -111.34, "y": 358.26, "z": 28.6}, "rotation": {"x": 0.0, "y": 0.0, "z": -136.2}}
# the sample spawn point's own route, as MORAI drove its car (10/29): entry, 3/4 of the ring (~24 s), exit, then removed
RING_ROUTE = [("A2256W000144", 26), ("A2256W000133", 0), ("A2256W000134", 0), ("A2256W000135", 0), ("A2256W000102", 0)]
# GetAvailableObject surround_vehicle on the 5090 box (10/29), 'RandomVehicle' left out
RING_MODELS = ['2017_Hyundai_Grandeur', '2016_Hyundai_Sonata', '2015_Kia_K5', '2014_Kia_K7', '2016_Hyundai_Ioniq',
               '2020_hyundai_sonata', '2020_Kia_Stinger', '2021_Hyundai_G80', '2019_Toyota_Prius', '2019_hyundai_genesis_g90',
               '2016_Hyundai_Genesis_DH', '2020_Kia_K5', 'DEFAULT_SEDAN', '2016_Hyundai_Santafe', '2017_Kia_Sorento',
               '2017_Kia_Niro(HEV)', '2019_Hyundai_Nexo', '2020_volvo_xc90', '2021_Hyundai_Genesis_GV80',
               '2021_Volkswagen_Golf_GTI', 'DEFAULT_SUV', '2021_Hyundai_GV70', '2023_Kia_EV6', '2023_Hyundai_Ioniq5',
               '2016_Hyundai_Universe', '2017_hyundai_solati_h350', 'DEFAULT_BUS', '2013_Hyundai_Porter2', '2012_Hyundai_HD65',
               '2013_Hyundai_Xcient(P520)', '2013_Hyundai_Xcient(P540)', '2014_Hyundai_Xcient(Mixer)', 'DEFAULT_TRUCK',
               '2015_Toyota_Prius(HEV)', 'DEFAULT_WAGON', '2019_Hyundai_Kona(EV)', '2020_Kia_Niro(EV)', '2016_Kia_Soul(EV)',
               '2020_Kia_Soul(EV)', '2019_kia_sedona', '2021_Kia_Carnival', '2020_chrysler_pacifica_hybrid', 'DEFAULT_MPV']
# traffic density per lap (user 10/29), s: ring one period per lap; highway per spawn point, drawn anew after each car
RING_PERIOD = {"high": (3.0, 5.0), "low": (8.0, 12.0)}
HW_PERIOD = {"high": (3.0, 5.0), "low": (10.0, 15.0)}
# ring: one per lap; highway: one per lane (spawn point) per lap (MORAI Q&A: a lane's speed is fixed, lanes differ, the
# bounds undisclosed). The official sample (10/03): ring 20 km/h, highway SP 6/8/10 at 40/50/60, so widened below
# (slow lanes to merge into, user 10/29: hard cases too); a gRPC route car never passes the link limit (60 on the
# highway) and ring cars hold ~22 in the curve (probe8/9 10/29), so no draw above them
RING_KPH, HW_KPH = (8.0, 22.0), (20.0, 60.0)
HW_NEAR = 300.0   # m from where a highway car was last seen: still ours when the lap ends (~11 s at 100 kph)
# The base scene's highway spawn points (UNIQUEID, x, y, z, yaw deg) from the official sample scenario.json (2026-10-03,
# sha256 1194ce3f; ~/morai-comp/analysis/npc-spawn.md: the same points in the eval scenes; clcol itself not read), z +0.27
# as RING_SPAWN over the ring point's 28.331. route: the shortest MGEO link path (map-link_set.json) from the point's
# startLinkInfo (link, point) to its endLinkInfo, which also gives RING_ROUTE; end: endLinkInfo, where MORAI removed the
# point's own cars. None of them is on the Ego's entry lane (A2256W000411, merging into 6's A2256W000420).
HIGHWAY = [
    dict(sp=6, pose=(65.769, 309.277, 28.56, -83.065), end=("A2256W001105", 83), route=[
        ("A2256W000406", 78), ("A2256W000409", 0), ("A2256W000420", 0), ("A2256W000179", 0), ("A2256W001105", 0)]),
    dict(sp=8, pose=(69.323, 309.680, 28.59, -82.905), end=("A2256W000422", 59), route=[
        ("A2256W000175", 78), ("A2256W000410", 0), ("A2256W000430", 0), ("A2256W000434", 0), ("A2256W000422", 0)]),
    dict(sp=10, pose=(73.008, 265.605, 28.62, -82.461), end=("A2256W000432", 246), route=[
        ("A2256W000029", 167), ("A2256W000418", 0), ("A2256W000431", 0), ("A2256W000435", 0), ("A2256W000423", 0),
        ("A2256W000432", 0)])]
STATES ={"client_key": "", "vehicle": True, "pedestrian": True, "obstacle": True}


def lap_density(mode, lap):
    """eval_manager --density: low / high, or alternate (lap 0 low, lap 1 high, ...)"""
    return ("low", "high")[lap % 2] if mode == "alternate" else mode


def actor(aid, kind):
    return {"id": {"value": str(aid)}, "object_type": kind, "client_key": ""}


def transform(x, y, z, yaw):
    return {"location": {"x": x, "y": y, "z": z}, "rotation": {"x": 0.0, "y": 0.0, "z": yaw}}


def brake(e, dt):
    """highest speed from which A_MAX per step (d += v dt after v) still stops within e m"""
    h = A_MAX * dt / 2
    return math.sqrt(h * h + 2 * A_MAX * max(e, 0.0)) - h


def cruise(need, slack, v):
    """speed to cover need m in slack s when reached at A_MAX from v and then held (None: full A_MAX is too late)"""
    b = v + A_MAX * slack
    disc = b * b - v * v - 2 * A_MAX * need
    return b - math.sqrt(disc) if disc >= 0 else None


def along(m, d):
    """-> (x, y, heading deg) at distance d along the mover's path"""
    cum, P = m["cum"], m["path"]
    i = max(0, min(len(cum) - 2, bisect.bisect_right(cum, d) - 1))
    (x0, y0), (x1, y1) = P[i], P[i + 1]
    u = (d - cum[i]) / max(cum[i + 1] - cum[i], 1e-9)
    return x0 + u * (x1 - x0), y0 + u * (y1 - y0), math.degrees(math.atan2(y1 - y0, x1 - x0))


class Director:
    def __init__(self, sim, route, pool, rng, models=None):
        self.sim, self.route, self.rng = sim, route, rng
        self.models = list(models or RING_MODELS)
        self.park = {str(u): (x, y, z, yaw) for u, x, y, z, yaw in pool["parking"]}
        self.at = dict(self.park)        # last pose set per uid: the pool scene loads every one parked
        self.kind, self.todo = {}, {}    # uid -> object type (from the lap that placed it); poses still to set
        self.movers, self.events, self.errors = [], [], {}
        self.tau = self.st_tau = None
        self.st = None
        self.cars, self.seq, self.ring_cfg, self.next_spawn, self.refused = {}, 0, None, None, 0
        self.hw, self.hw_seq, self.density = [], 0, None   # this lap's highway points: HIGHWAY rows + kph, next, refused

    def call(self, method, req, who=None):
        try:
            return self.sim.call(method, req)
        except Exception as e:   # noqa: BLE001  grpc errors and the handler's own exceptions alike
            n = self.errors[method] = self.errors.get(method, 0) + 1
            if n & (n - 1) == 0:   # 1st, 2nd, 4th, ... per method: a dead call must not flood the lap record
                self.events.append(dict(event="error", call=method, id=who, n=n, error=str(e)[:160], tau=self.tau))
            return None

    def flush(self):
        out, self.events = self.events, []
        return out

    # ----- lap layout
    def new_lap(self, meta, density="low"):
        self.movers, want = [], {}
        for a in meta["actors"]:
            u = str(a["uid"])
            self.kind[u] = PED if a.get("ped") else OBS
            if a["zone"].endswith("_dynamic"):
                m = self.mover(u, a)
                self.movers.append(m)
                want[u] = (*m["path"][0], m["z"], along(m, 0.0)[2])
            else:
                want[u] = (a["xy"][0], a["xy"][1], a["z"], a["yaw"])
        for u, pose in self.at.items():   # last lap's ones back to parking
            if u not in want and u in self.park and pose != self.park[u]:
                want[u] = self.park[u]
        self.todo = {u: p for u, p in want.items()
                     if u not in self.at or any(abs(a - b) > 1e-3 for a, b in zip(p, self.at[u]))}
        self.place()
        self.new_traffic(density)
        return self.flush()

    def place(self):
        for u, pose in list(self.todo.items()):
            if self.call("SetTransform", {"actor_info": actor(u, self.kind.get(u, OBS)), "transform": transform(*pose)},
                         u) is not None:
                self.at[u] = pose
                del self.todo[u]

    def mover(self, u, a):
        P = [tuple(p[:2]) for p in a["path"]]
        cum = [0.0]
        for p, q in zip(P, P[1:]):
            cum.append(cum[-1] + math.hypot(q[0] - p[0], q[1] - p[1]))
        vn = a["kph"] / 3.6
        m = dict(uid=u, path=P, cum=cum, z=a["z"], vn=vn, vmax=max(vn, V_SPRINT.get(a.get("cls"), SPRINT_M_S)),
                 timing=a["timing"], lead=a["lead"], cin=a["corr_in_m"], cout=a["corr_out_m"], s_cross=a["s_cross"],
                 goal=a["corr_in_m"] if a["timing"] == "must_stop" else a["corr_out_m"],
                 phase="wait", d=0.0, v=0.0, entered=False, exited=False, T=None, r=1.0)
        if m["s_cross"] is None:         # layout gives None for a path that never crosses the route line: its entry point
            m["s_cross"] = self.route.project(*along(m, m["cin"])[:2])[0]
        return m

    # ----- per step
    def step(self, tau, ego):
        dt = 0.0 if self.tau is None else min(max(tau - self.tau, 0.0), DT_MAX)
        self.tau = tau
        if self.todo:                    # a static that failed to go in: its lap's mission depends on it
            self.place()
        for m in self.movers:
            self.move(m, dt, ego)
        self.traffic()
        return self.flush()

    def move(self, m, dt, ego):
        if m["phase"] == "done":
            return
        rem = m["s_cross"] - ego["s"] - EGO_FRONT_M
        T = rem / max(ego["v"], 0.5)

        def ev(name):
            self.events.append(dict(uid=m["uid"], event=name, tau=self.tau, ego_t=round(T, 2), ego_s=round(ego["s"], 1),
                                    ego_v=round(ego["v"], 2)))
        d, v, cmd = m["d"], m["v"], m["vn"]
        if m["T"] is not None and dt > 0:
            # an Ego speeding up (from a stop: twice as fast) shortens T faster than the clock; plan on that rate
            m["r"] += (min(max((m["T"] - T) / dt, 1.0), R_MAX) - m["r"]) * min(dt / R_TC, 1.0)
        m["T"] = T
        if m["phase"] != "free":
            if rem < 0:                  # the Ego's front passed before the timing was met: should not happen
                ev("missed")
                m["phase"] = "free"
            else:
                slack, need = (T - m["lead"]) / m["r"], m["goal"] - d
                if T - m["lead"] <= 0.1:
                    v_req = m["vmax"]
                elif need < v * slack:   # early at this speed: the speed that arrives on time
                    v_req = need / slack
                else:                    # late or starting: counting the time to speed up (from rest one started 1 s late)
                    u = cruise(need, slack, v)
                    v_req = m["vmax"] if u is None else u
                if m["phase"] == "wait":
                    if v_req < START * m["vn"]:
                        return
                    m["phase"] = "timed"
                    ev("start")
                cmd = min(v_req, m["vmax"]) if v_req >= HOLD * m["vn"] else 0.0
                if d < m["cin"] and need < v * slack:   # early at this speed: stay able to stop at the curb
                    cmd = min(cmd, brake(m["cin"] - d, dt))
                elif d >= m["cin"]:      # pass_front inside the lane: never stand in it (that would be must_stop)
                    cmd = max(min(v_req, m["vmax"]), HOLD * m["vn"])
        v = max(0.0, min(max(cmd, v - A_MAX * dt), v + A_MAX * dt))
        d = min(d + v * dt, m["cum"][-1])
        if d != m["d"]:
            x, y, yaw = along(m, d)
            if self.call("SetTransform", {"actor_info": actor(m["uid"], self.kind[m["uid"]]),
                                          "transform": transform(x, y, m["z"], yaw)}, m["uid"]) is not None:
                self.at[m["uid"]] = (x, y, m["z"], yaw)
        m["d"], m["v"] = d, v
        for key, edge, name in (("entered", "cin", "enter"), ("exited", "cout", "exit")):
            if not m[key] and d >= m[edge]:
                m[key] = True
                ev(name)
        if m["phase"] == "timed" and d >= m["goal"]:
            m["phase"] = "free"
        if d >= m["cum"][-1]:
            m["phase"] = "done"
            ev("done")

    # ----- roundabout and highway
    def states(self):
        """{id: (x, y, current link id, its waypoint idx)} of every actor, one GetAllActorsState per step (None when it
        failed)"""
        if self.st_tau != self.tau:
            r = self.call("GetAllActorsState", STATES)
            self.st_tau, self.st = self.tau, None if r is None else {}
            for s in [] if r is None else r.get("states", []):
                loc = s.get("transform", {}).get("location", {})
                li = s.get("vehicle_state", {}).get("current_link_info", {})
                self.st[s["actor_info"]["id"]["value"]] = (loc.get("x", 0.0), loc.get("y", 0.0),
                                                           li.get("id", {}).get("value"), li.get("waypoint_idx", 0))
        return self.st

    def on_ring(self, c, p):
        """-> (on the ring, ours past its route end: on its last link 102, where the sample's cars are removed, or farther
        than RING_EXIT). A slow car stops at its route end on 102 inside RING_EXIT and the whole ring queued behind it
        (v3k lap 3, 10/29)."""
        d = None if p is None else math.hypot(p[0] - RING_C[0], p[1] - RING_C[1])
        out = d is not None and (d > RING_EXIT or p[2] == RING_ROUTE[-1][0])
        return d is not None and not out, out and c["seen"] and d <= RING_NEAR and math.dist(p[:2], c["p"]) < JUMP_M

    def on_route(self, c, p):
        """-> (on its links before endLinkInfo, ours past it or off its links; never seen on them 5 s after the spawn
        counts as off). Listed farther from where it was last seen than it can drive: MORAI's reuse of the id."""
        if p is None or math.dist(p[:2], c["p"]) > JUMP_M + HW_KPH[1] / 3.6 * (self.tau - c["t"]):
            return False, False
        on = p[2] in c["links"] and not (p[2] == c["end"][0] and p[3] >= c["end"][1])
        return on, not on and (c["seen"] or self.tau - c["t0"] > 5.0)

    def traffic(self):
        st = self.states() if self.cars else None
        for cid, c in list(self.cars.items()):
            while c["setup"]:            # physics, unpause, cruise, route: in order, each retried until it answers
                if self.call(c["setup"][0][0], c["setup"][0][1], cid) is None:
                    break
                c["setup"].pop(0)
            if st is None:
                continue
            p = st.get(cid)
            on, off = (self.on_ring if c["kind"] == "ring" else self.on_route)(c, p)
            if on:
                c["seen"], c["p"], c["t"] = True, p[:2], self.tau
            elif off:
                # past its route end: the sample's cars are removed there, a gRPC route car drives on (10/29)
                c["p"], c["t"] = p[:2], self.tau
                if self.call("DestroyActor", actor(cid, VEH), cid) is not None:
                    del self.cars[cid]
                    self.events.append(dict(event=c["kind"] + "_exit", car=cid, age_s=round(self.tau - c["t0"], 1),
                                            tau=self.tau))
            elif c["seen"] or self.tau - c["t0"] > 5.0:
                # gone at its route end (or never there): MORAI may reuse the id elsewhere, so it is never touched again
                del self.cars[cid]
                self.events.append(dict(event=c["kind"] + "_done", car=cid, age_s=round(self.tau - c["t0"], 1),
                                        tau=self.tau))
        if self.ring_cfg and (self.next_spawn is None or self.tau >= self.next_spawn):
            self.spawn_ring()
        for h in self.hw:
            if h["next"] is None or self.tau >= h["next"]:
                self.spawn_hw(h)

    def spawn(self, cid, pose, model, kph, links):
        """-> the car's record with its setup calls still to make, or None: the point is still taken (NOT_CREATE_OBJECT)"""
        r = self.call("SpawnVehicle", {"spawn_info": {"actor_info": actor(cid, VEH), "transform": pose,
                                                      "model_name": model, "label": cid}, "velocity": 0.0}, cid)
        if r is not None and r.get("status") != 1 and r.get("description") != "SUCCESS":
            return None
        # an exception may still have spawned it: tracked like a spawned car, dropped if it never shows up
        a = actor(cid, VEH)
        return dict(t0=self.tau, t=self.tau, seen=False, p=(pose["location"]["x"], pose["location"]["y"]), setup=[
            ("SetPhysics", {"actor_info": a, "enable": True}), ("SetPause", {"actor_info": a, "enable": False}),
            # the speed: a dynamics cap (m/s) holds it; cruise CONSTANT / LINK ratio were ignored, the cars ran the link's
            # limit (probe8/9 on the 5090 box, 10/29: cap 11.1 -> 39.3-40 km/h steady)
            ("SetVehicleDynamicsSpeedLimit", {"actor_info": a, "speed_limiter": True, "speed_limit": kph / 3.6}),
            ("SetVehicleRoute", {"actor_info": a, "decision_range": 3.0, "links": [     # once: a 2nd one despawns it
                {"id": {"value": link}, "waypoint_idx": w} for link, w in links]})])

    def spawn_ring(self):
        cid, cfg = f"wd{self.seq + 1}", self.ring_cfg
        c = self.spawn(cid, RING_SPAWN, cfg["model"], cfg["kph"], RING_ROUTE)
        if c is None:
            self.refused += 1            # next step
            return
        self.seq += 1
        self.next_spawn = self.tau + cfg["period_s"]
        self.cars[cid] = dict(c, kind="ring")
        self.events.append(dict(event="ring_spawn", car=cid, refused=self.refused, tau=self.tau))
        self.refused = 0

    def spawn_hw(self, h):
        """one car at highway point h: a random model at the lane's speed; the next one a fresh period later"""
        cid, model = f"wh{self.hw_seq + 1}", self.rng.choice(self.models)
        c = self.spawn(cid, h["tf"], model, h["kph"], h["route"])
        if c is None:
            h["refused"] += 1            # next step
            return
        self.hw_seq += 1
        period = round(self.rng.uniform(*HW_PERIOD[self.density]), 2)
        h["next"] = self.tau + period
        self.cars[cid] = dict(c, kind="highway", links={link for link, _ in h["route"]}, end=h["end"])
        self.events.append(dict(event="highway_spawn", car=cid, sp=h["sp"], model=model, kph=h["kph"], next_s=period,
                                refused=h["refused"], tau=self.tau))
        h["refused"] = 0

    def new_traffic(self, density):
        """this lap's traffic for density low / high: the ring's one model / period / speed (rules: one spawn point, its
        vehicle, period and speed fixed for the lap) and each highway lane's speed"""
        left = self.clear()
        self.density = density
        self.ring_cfg = dict(density=density, model=self.rng.choice(self.models),
                             period_s=round(self.rng.uniform(*RING_PERIOD[density]), 2),
                             kph=round(self.rng.uniform(*RING_KPH), 1))
        self.next_spawn, self.refused = None, 0
        self.hw = [dict(h, tf=transform(*h["pose"]), kph=round(self.rng.uniform(*HW_KPH), 1), next=None, refused=0)
                   for h in HIGHWAY]
        self.events.append(dict(event="ring_lap", **self.ring_cfg, left=[i for i in left if i.startswith("wd")],
                                tau=self.tau))
        self.events.append(dict(event="highway_lap", density=density, period_s=list(HW_PERIOD[density]),
                                kph={h["sp"]: h["kph"] for h in self.hw}, left=[i for i in left if i.startswith("wh")],
                                tau=self.tau))

    def mine(self, c, p):
        """a car of ours still listed where it can be (else gone, or its id reused elsewhere)"""
        if p is None:
            return False
        if c["kind"] == "ring":
            return math.hypot(p[0] - RING_C[0], p[1] - RING_C[1]) <= RING_NEAR
        # shortcut: a highway car more than HW_NEAR past where it was last seen (real time: > ~11 s between the last
        # step and new_lap / close) is left driving; time that gap if lap ends ever take that long
        return math.dist(p[:2], c["p"]) <= HW_NEAR

    def clear(self):
        """destroy every car of ours still listed -> ids that may still be there"""
        for k in range(4):               # a car counts as gone only once GetAllActorsState stops listing it
            self.st_tau = object()       # fresh states, not this step's cache
            st = self.states() if self.cars else {}
            if st is None:               # unknown: a blind DestroyActor could hit a reused id
                continue
            for cid, c in list(self.cars.items()):
                if not self.mine(c, st.get(cid)):
                    del self.cars[cid]   # gone, or its id reused elsewhere
            if not self.cars or k == 3:
                break
            for cid, c in list(self.cars.items()):
                if self.call("DestroyActor", actor(cid, VEH), cid) is not None:
                    self.events.append(dict(event=c["kind"] + "_destroy", car=cid, tau=self.tau))
        self.st_tau = None
        return sorted(self.cars)

    def close(self):
        """before a scene reload: no more spawns, our cars destroyed -> ids that may still be there (reload only if [])"""
        self.ring_cfg, self.hw = None, []
        left = self.clear()
        self.events.append(dict(event="close", left=left, tau=self.tau))
        return left
