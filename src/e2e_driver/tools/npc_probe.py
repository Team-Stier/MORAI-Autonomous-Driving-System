#!/usr/bin/env python3
"""npc_probe.py [SEED]: what MORAI's gRPC Actor API does to scene objects and spawned vehicles (10/29 dev box, before the
ring-set and ego-timed mover directors). On the simulator host of world 1 (gRPC 127.0.0.1:7789), after worlds.sh up 1.
Loads a v2 scene (scenario_variants.py), plays it and prints, step by step: the available models, every actor's id/type/
pose, a scene pedestrian moved with ControlPedestrian, a scene prop with ControlObstacle (kinematics), a vehicle spawned
at the roundabout with SetAI and SetVehicleRoute, and DestroyActor. Each step catches its own error.
"""
import json
import math
import os
import shutil
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import eval_manager as em  # noqa: E402

em.BC = "local"
em.LAUNCHER = os.path.expanduser("~/Workspaces/scratches/morai/MoraiLauncher_Lin_Data")
em.SCEN = em.LAUNCHER + "/SaveFile/Scenario/R_KR_PR_K-city_2025"
for src, dst in ((HERE / "scenario_variants.py", "morai-eval-scenario-variants.py"), (HERE / "msc.py", "morai-eval-msc.py"),
                 (em.MAP / "2026_molit_comp_global_path.txt", "morai-eval-global-path.txt")):
    shutil.copy(src, Path.home() / dst)
SIMSVC = "morai_sim_api.simualtor.Simulator"
sim = em.Sim("127.0.0.1:7789")


def step(name, fn):
    print(f"\n### {name}", flush=True)
    try:
        r = fn()
        if r is not None:
            print(json.dumps(r, ensure_ascii=False)[:3000], flush=True)
        return r
    except Exception as e:   # noqa: BLE001 - a probe: report and go on
        print(f"ERROR {type(e).__name__}: {str(e)[:800]}", flush=True)


def actor(aid, typ):
    return {"id": {"value": str(aid)}, "object_type": typ, "client_key": ""}


def state(aid, typ):
    m = sim.call("GetActorState", actor(aid, typ))
    loc = m.get("transform", {}).get("location", {})
    return round(loc.get("x", 0.0), 2), round(loc.get("y", 0.0), 2)


def track(aid, typ, secs, dt=1.0):
    out = []
    for _ in range(int(secs / dt)):
        out.append(state(aid, typ))
        time.sleep(dt)
    return out


seed = int(sys.argv[1]) if len(sys.argv) > 1 else 7
name, _, meta = em.make_scene(seed)
print("scene", name, json.dumps([{k: a.get(k) for k in ("zone", "id", "cls", "ptype", "xy", "ped")} for a in meta["actors"]]))
step("GetAvailableObject", lambda: sim.call("GetAvailableObject", {}, svc=SIMSVC))
print(em.msc("pause"), em.msc("load-scene", name + "/62", 90), em.msc("play"), flush=True)
time.sleep(8)
allst = step("GetAllActorsState", lambda: sim.call("GetAllActorsState", {"client_key": "", "vehicle": True, "pedestrian": True,
                                                                         "obstacle": True}))
movers = [a for a in meta["actors"] if a["zone"].endswith("_dynamic")]
peds = [a for a in movers if a.get("ped")]
props = [a for a in movers if not a.get("ped")] + [a for a in meta["actors"] if not a["zone"].endswith("_dynamic")]


def nearest(xy, typ):   # the actor id the API gives the scene object placed at xy
    best = None
    for st in (allst or {}).get("actor_states", (allst or {}).get("states", [])):
        info = st.get("actor_info", {})
        loc = st.get("transform", {}).get("location", {})
        d = math.hypot(loc.get("x", 1e9) - xy[0], loc.get("y", 1e9) - xy[1])
        if info.get("object_type") in (typ, em_type(typ)) and (best is None or d < best[0]):
            best = (d, info.get("id", {}).get("value"))
    return best


def em_type(t):
    return {"OBJECT_TYPE_PEDESTRIAN": 2, "OBJECT_TYPE_OBSTACLE": 3, "OBJECT_TYPE_VEHICLE": 1}.get(t, t)


if peds:
    p = peds[0]
    hit = step(f"scene pedestrian {p['id']} at {p['xy']}: its API id", lambda: nearest(p["xy"], "OBJECT_TYPE_PEDESTRIAN"))
    if hit:
        pid = hit[1]
        step("pedestrian state", lambda: sim.call("GetActorState", actor(pid, "OBJECT_TYPE_PEDESTRIAN")))
        dx, dy = p["xy_end"][0] - p["xy"][0], p["xy_end"][1] - p["xy"][1]
        n = math.hypot(dx, dy)
        step("ControlPedestrian 1.5 m/s toward its path end", lambda: sim.call("ControlPedestrian", {
            "actor_info": actor(pid, "OBJECT_TYPE_PEDESTRIAN"), "direction": {"x": dx / n, "y": dy / n, "z": 0.0}, "speed": 1.5}))
        step("pedestrian track 6 s", lambda: track(pid, "OBJECT_TYPE_PEDESTRIAN", 6))
        step("ControlPedestrian speed 0", lambda: sim.call("ControlPedestrian", {
            "actor_info": actor(pid, "OBJECT_TYPE_PEDESTRIAN"), "direction": {"x": dx / n, "y": dy / n, "z": 0.0}, "speed": 0.0}))
        step("pedestrian track 3 s (should stand)", lambda: track(pid, "OBJECT_TYPE_PEDESTRIAN", 3))
if props:
    o = props[0]
    hit = step(f"scene prop {o['id']} at {o['xy']}: its API id", lambda: nearest(o["xy"], "OBJECT_TYPE_OBSTACLE"))
    if hit:
        oid = hit[1]
        step("ControlObstacle kinematics vx 1.0", lambda: sim.call("ControlObstacle", {
            "actor_info": actor(oid, "OBJECT_TYPE_OBSTACLE"), "mode": "OBJECT_CONTROL_MODE_KINEMATICS", "velocity_x": 1.0,
            "velocity_y": 0.0, "yaw_rate": 0.0}))
        step("prop track 5 s", lambda: track(oid, "OBJECT_TYPE_OBSTACLE", 5))
        step("ControlObstacle kinematics 0", lambda: sim.call("ControlObstacle", {
            "actor_info": actor(oid, "OBJECT_TYPE_OBSTACLE"), "mode": "OBJECT_CONTROL_MODE_KINEMATICS", "velocity_x": 0.0,
            "velocity_y": 0.0, "yaw_rate": 0.0}))
models = os.environ.get("MODELS", "2015_Kia_K5").split(",")
vid = "probe_v1"
step(f"SpawnVehicle {models[0]} at the ring point", lambda: sim.call("SpawnVehicle", {
    "spawn_info": {"actor_info": actor(vid, "OBJECT_TYPE_VEHICLE"), "transform": {
        "location": {"x": -111.34, "y": 358.26, "z": 28.6}, "rotation": {"x": 0.0, "y": 0.0, "z": -136.2}},
        "model_name": models[0], "label": vid}, "velocity": 3.0}))
step("all vehicles after the spawn", lambda: [(s.get("actor_info", {}).get("id"), s.get("transform", {}).get("location"))
                                               for s in sim.call("GetAllActorsState", {"client_key": "", "vehicle": True}).get(
                                                   "actor_states", [])][:20])
step("SetAI on", lambda: sim.call("SetAI", {"actor_info": actor(vid, "OBJECT_TYPE_VEHICLE"), "enable": True}))
step("vehicle state", lambda: sim.call("GetActorState", actor(vid, "OBJECT_TYPE_VEHICLE")))
step("vehicle track 10 s", lambda: track(vid, "OBJECT_TYPE_VEHICLE", 10))
RC, RR = (-86.15, 330.3), 19.7   # ring centre and lane radius (kcity_lanelet2 fit, memory: real ring R ~ 19.7)


def ring_dest(lead_deg):
    x, y = state(vid, "OBJECT_TYPE_VEHICLE")
    a = math.atan2(y - RC[1], x - RC[0]) + math.radians(lead_deg)
    return {"x": RC[0] + RR * math.cos(a), "y": RC[1] + RR * math.sin(a), "z": 28.6}


for lead in (150, -150):   # counter-clockwise first, then the other way
    step(f"SetVehicleDestination {lead} deg ahead on the ring", lambda: sim.call("SetVehicleDestination", {
        "actor_info": actor(vid, "OBJECT_TYPE_VEHICLE"), "decision_range": 30.0, "position": ring_dest(lead)}))
    tr = step("vehicle track 20 s (angle about the ring centre, radius)", lambda: [
        (p, round(math.degrees(math.atan2(p[1] - RC[1], p[0] - RC[0]))), round(math.hypot(p[0] - RC[0], p[1] - RC[1]), 1))
        for p in track(vid, "OBJECT_TYPE_VEHICLE", 20, 2.0)])
links = os.environ.get("RING_LINKS", "")
if links:
    step("SetVehicleRoute ring links x3", lambda: sim.call("SetVehicleRoute", {
        "actor_info": actor(vid, "OBJECT_TYPE_VEHICLE"), "decision_range": 30.0,
        "links": [{"id": {"value": lk}, "waypoint_idx": 0} for lk in links.split(",") * 3]}))
    step("vehicle track 40 s", lambda: track(vid, "OBJECT_TYPE_VEHICLE", 40, 2.0))
step("DestroyActor", lambda: sim.call("DestroyActor", actor(vid, "OBJECT_TYPE_VEHICLE")))
step("vehicle state after destroy", lambda: sim.call("GetActorState", actor(vid, "OBJECT_TYPE_VEHICLE")))
print(em.msc("pause"))
