#!/usr/bin/env python3
"""npc_probe2.py: npc_probe.py's scene (ev7_000, loaded) with the simulation running: is simulator time moving, then
ControlPedestrian / ControlObstacle / a spawned vehicle's motion again, and the vehicles the spawn points make."""
import json
import math
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import eval_manager as em  # noqa: E402

em.BC = "local"
em.LAUNCHER = os.path.expanduser("~/Workspaces/scratches/morai/MoraiLauncher_Lin_Data")
em.SCEN = em.LAUNCHER + "/SaveFile/Scenario/R_KR_PR_K-city_2025"
sim = em.Sim("127.0.0.1:7789")
SIMSVC, SIMU = "morai_sim_api.simualtor.Simulator", em.SIMU


def step(name, fn):
    print(f"\n### {name}", flush=True)
    try:
        r = fn()
        if r is not None:
            print(json.dumps(r, ensure_ascii=False)[:2500], flush=True)
        return r
    except Exception as e:   # noqa: BLE001
        print(f"ERROR {type(e).__name__}: {str(e)[:600]}", flush=True)


def actor(aid, typ):
    return {"id": {"value": str(aid)}, "object_type": typ, "client_key": ""}


def pos(aid, typ):
    loc = sim.call("GetActorState", actor(aid, typ)).get("transform", {}).get("location", {})
    return round(loc.get("x", 0.0), 2), round(loc.get("y", 0.0), 2)


def track(aid, typ, secs, dt=1.0):
    out = []
    for _ in range(int(secs / dt)):
        out.append(pos(aid, typ))
        time.sleep(dt)
    return out


def vehicles():
    st = sim.call("GetAllActorsState", {"client_key": "", "vehicle": True, "pedestrian": False, "obstacle": False})
    return [(s["actor_info"]["id"]["value"], round(s["transform"]["location"].get("x", 0), 1),
             round(s["transform"]["location"].get("y", 0), 1)) for s in st.get("states", [])]


ts = lambda: sim.call("GetTimestamp", {}, svc=SIMSVC)   # noqa: E731
step("timestamp", ts)
time.sleep(2)
step("timestamp 2 s later", ts)
step("Simulation.Resume", lambda: sim.call("Resume", {}, svc=SIMU))
print(em.msc("play"), flush=True)
time.sleep(5)
step("timestamp after resume/play", ts)
time.sleep(2)
step("timestamp 2 s later", ts)
step("Ego pose (should stay: no stack)", lambda: pos("Ego", "OBJECT_TYPE_VEHICLE"))
step("vehicles now", vehicles)
step("ControlPedestrian 1003 1.5 m/s", lambda: sim.call("ControlPedestrian", {
    "actor_info": actor("1003", "OBJECT_TYPE_PEDESTRIAN"), "direction": {"x": 0.6, "y": -0.8, "z": 0.0}, "speed": 1.5}))
step("pedestrian 1003 track 5 s", lambda: track("1003", "OBJECT_TYPE_PEDESTRIAN", 5))
for mode in ("OBJECT_CONTROL_MODE_KINEMATICS", "OBJECT_CONTROL_MODE_WAYPOINT", "OBJECT_CONTROL_MODE_NORMAL"):
    step(f"ControlObstacle 1001 {mode} vx 1", lambda: sim.call("ControlObstacle", {
        "actor_info": actor("1001", "OBJECT_TYPE_OBSTACLE"), "mode": mode, "velocity_x": 1.0, "velocity_y": 0.0,
        "yaw_rate": 0.0}))
    step("prop 1001 track 4 s", lambda: track("1001", "OBJECT_TYPE_OBSTACLE", 4))
step("SpawnVehicle K5 v 5", lambda: sim.call("SpawnVehicle", {"spawn_info": {
    "actor_info": actor("pv2", "OBJECT_TYPE_VEHICLE"), "transform": {"location": {"x": -111.34, "y": 358.26, "z": 28.6},
                                                                       "rotation": {"x": 0, "y": 0, "z": -136.2}},
    "model_name": os.environ.get("MODEL", "2015_Kia_K5"), "label": "pv2"}, "velocity": 5.0}))
step("pv2 track 8 s", lambda: track("pv2", "OBJECT_TYPE_VEHICLE", 8))
step("SetAI pv2", lambda: sim.call("SetAI", {"actor_info": actor("pv2", "OBJECT_TYPE_VEHICLE"), "enable": True}))
step("SetVehicleDestination pv2 -150 deg", lambda: sim.call("SetVehicleDestination", {
    "actor_info": actor("pv2", "OBJECT_TYPE_VEHICLE"), "decision_range": 30.0,
    "position": {"x": -86.15 + 19.7 * math.cos(math.radians(-18)), "y": 330.3 + 19.7 * math.sin(math.radians(-18)), "z": 28.6}}))
step("pv2 track 20 s", lambda: track("pv2", "OBJECT_TYPE_VEHICLE", 20, 2.0))
step("pv2 state", lambda: sim.call("GetActorState", actor("pv2", "OBJECT_TYPE_VEHICLE")).get("vehicle_state"))
step("vehicles after 30 s", vehicles)
step("DestroyActor pv2", lambda: sim.call("DestroyActor", actor("pv2", "OBJECT_TYPE_VEHICLE")))
print(em.msc("pause"))
