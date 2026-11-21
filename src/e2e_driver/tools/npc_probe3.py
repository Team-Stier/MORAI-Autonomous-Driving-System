#!/usr/bin/env python3
"""npc_probe3.py: what makes a gRPC-spawned vehicle drive and a scene pedestrian / prop move (after npc_probe2.py:
both accepted their commands and stayed put). ev7_000 is loaded and playing."""
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
sim = em.Sim("127.0.0.1:7789")
V, P, O = "OBJECT_TYPE_VEHICLE", "OBJECT_TYPE_PEDESTRIAN", "OBJECT_TYPE_OBSTACLE"


def step(name, fn):
    print(f"\n### {name}", flush=True)
    try:
        r = fn()
        if r is not None:
            print(json.dumps(r, ensure_ascii=False)[:1500], flush=True)
        return r
    except Exception as e:   # noqa: BLE001
        print(f"ERROR {type(e).__name__}: {str(e)[:400]}", flush=True)


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


print(em.msc("play"), flush=True)
time.sleep(3)
# vehicle
step("SpawnVehicle pv3", lambda: sim.call("SpawnVehicle", {"spawn_info": {
    "actor_info": actor("pv3", V), "transform": {"location": {"x": -111.34, "y": 358.26, "z": 28.6}, "rotation": {"z": -136.2}},
    "model_name": "2015_Kia_K5", "label": "pv3"}, "velocity": 0.0}))
step("GetVehicleControlMode", lambda: sim.call("GetVehicleControlMode", actor("pv3", V)))
step("SetPhysics on", lambda: sim.call("SetPhysics", {"actor_info": actor("pv3", V), "enable": True}))
step("SetPause off", lambda: sim.call("SetPause", {"actor_info": actor("pv3", V), "enable": False}))
step("SetVehicleControlMode AUTO", lambda: sim.call("SetVehicleControlMode", {"actor_info": actor("pv3", V),
                                                                              "mode": "VEHICLE_CONTROL_AUTO_MODE"}))
step("SetVehicleEgoCruise constant 5", lambda: sim.call("SetVehicleEgoCruise", {
    "actor_info": actor("pv3", V), "cruise_on": True, "cruise_type": "EGO_CRUISE_TYPE_CONSTANT", "constant_velocity": 5.0}))
step("SetVehicleDestination", lambda: sim.call("SetVehicleDestination", {"actor_info": actor("pv3", V), "decision_range": 30.0,
    "position": {"x": -86.15 + 19.7 * math.cos(math.radians(-18)), "y": 330.3 + 19.7 * math.sin(math.radians(-18)), "z": 28.6}}))
step("pv3 track 12 s", lambda: track("pv3", V, 12, 2.0))
step("ControlVehicle SPEED 4", lambda: sim.call("ControlVehicle", {"actor_info": actor("pv3", V), "long_cmd_type":
                                                                   "LONG_CMD_TYPE_SPEED", "velocity": 4.0, "steer": 0.0}))
step("pv3 track 6 s", lambda: track("pv3", V, 6, 2.0))
step("pv3 state", lambda: sim.call("GetActorState", actor("pv3", V)).get("vehicle_state"))
# pedestrian 1003 (ev7 tunnel cyclist-type) and prop 1001
dst = {"x": -5.63 + 6.0, "y": -510.15 - 4.0, "z": 28.42}
step("SetPause 1003 off", lambda: sim.call("SetPause", {"actor_info": actor("1003", P), "enable": False}))
step("SetPedestrianWaypoint 1003", lambda: sim.call("SetPedestrianWaypoint", {"actor_info": actor("1003", P), "is_create": True,
    "waypoints": {"waypoints": [{"waypoint": dst, "pause_time": 0.0, "speed": 1.5}]}}))
step("1003 track 6 s", lambda: track("1003", P, 6))
step("ControlPedestrian 1003", lambda: sim.call("ControlPedestrian", {"actor_info": actor("1003", P),
                                                                      "direction": {"x": 0.83, "y": -0.55, "z": 0.0}, "speed": 1.5}))
step("1003 track 5 s", lambda: track("1003", P, 5))
step("SetPause 1001 off", lambda: sim.call("SetPause", {"actor_info": actor("1001", O), "enable": False}))
step("SetPhysics 1001 on", lambda: sim.call("SetPhysics", {"actor_info": actor("1001", O), "enable": True}))
for fr in (1, 10):
    step(f"ControlObstacle 1001 kinematics frame {fr}", lambda: sim.call("ControlObstacle", {"actor_info": actor("1001", O),
        "mode": "OBJECT_CONTROL_MODE_KINEMATICS", "frame": fr, "velocity_x": 1.0, "velocity_y": 0.5, "yaw_rate": 0.0}))
    step("1001 track 4 s", lambda: track("1001", O, 4))
step("SpawnPedestrian pp1 walking at once", lambda: sim.call("SpawnPedestrian", {"spawn_info": {
    "actor_info": actor("pp1", P), "transform": {"location": {"x": -60.0, "y": -5.0, "z": 28.6}, "rotation": {"z": 0.0}},
    "model_name": os.environ.get("PED", "Pedestrian_1"), "label": "pp1"}, "velocity": 1.5, "active_dist": 1000.0, "move_dist": 10.0,
    "start_action": True, "waypoints": {"waypoints": [{"waypoint": {"x": -50.0, "y": -5.0, "z": 28.6}, "pause_time": 0, "speed": 1.5}]}}))
step("pp1 track 6 s", lambda: track("pp1", P, 6))
for a, t in (("pv3", V), ("pp1", P)):
    step(f"DestroyActor {a}", lambda: sim.call("DestroyActor", actor(a, t)))
print(em.msc("pause"))
