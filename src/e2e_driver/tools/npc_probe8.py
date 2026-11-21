#!/usr/bin/env python3
"""npc_probe8.py: which call sets a gRPC route car's speed (10/29 v3d: the cruise speed was ignored, highway cars ran 60
km/h and ring cars ~22 whatever was asked). One car per highway spawn point, all asked for 50.4 km/h (14 m/s):
  hw6  spawn at 0, SetPhysics, SetPause off, SetVehicleEgoCruise CONSTANT 14, route   (the director's way)
  hw8  spawn AT 14 m/s, SetPhysics, SetPause off, SetVehicleEgoCruise CONSTANT 14, route
  hw10 spawn AT 14 m/s, SetPhysics, SetPause off, route (no cruise)
  rg   the ring point: spawn at 0, the director's way, then ControlVehicle SPEED 14 after 3 s
Speeds (GetAllActorsState velocity, km/h) for 30 s; every car destroyed at the end. Run on a playing world."""
import json
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import eval_manager as em  # noqa: E402
import world_director as wd  # noqa: E402

em.BC = "local"
sim = em.Sim("127.0.0.1:7789")
V, MPS = "OBJECT_TYPE_VEHICLE", 14.0
live = []


def actor(aid):
    return {"id": {"value": aid}, "object_type": V, "client_key": ""}


def call(m, req):
    try:
        r = sim.call(m, req)
    except Exception as e:   # noqa: BLE001
        r = {"error": str(e)[:80]}
    print(f"  {m} -> {json.dumps(r)[:100]}", flush=True)
    return r


def car(aid, tf, links, spawn_v, cruise):
    r = call("SpawnVehicle", {"spawn_info": {"actor_info": actor(aid), "transform": tf, "model_name": "2015_Kia_K5",
                                             "label": aid}, "velocity": spawn_v})
    live.append(aid)
    call("SetPhysics", {"actor_info": actor(aid), "enable": True})
    call("SetPause", {"actor_info": actor(aid), "enable": False})
    if cruise:
        call("SetVehicleEgoCruise", {"actor_info": actor(aid), "cruise_on": True, "cruise_type": "EGO_CRUISE_TYPE_CONSTANT",
                                     "constant_velocity": MPS})
    call("SetVehicleRoute", {"actor_info": actor(aid), "decision_range": 3.0,
                             "links": [{"id": {"value": l}, "waypoint_idx": w} for l, w in links]})
    return r


try:
    hw = {h["sp"]: h for h in wd.HIGHWAY}
    car("hw6", wd.transform(*hw[6]["pose"]), hw[6]["route"], 0.0, True)
    car("hw8", wd.transform(*hw[8]["pose"]), hw[8]["route"], MPS, True)
    car("hw10", wd.transform(*hw[10]["pose"]), hw[10]["route"], MPS, False)
    car("rg", wd.RING_SPAWN, wd.RING_ROUTE, 0.0, True)
    for i in range(30):
        if i == 3:
            call("ControlVehicle", {"actor_info": actor("rg"), "long_cmd_type": "LONG_CMD_TYPE_SPEED", "velocity": MPS,
                                    "steer": 0.0})
        st = sim.call("GetAllActorsState", {"client_key": "", "vehicle": True})
        row = []
        for s in st.get("states", []):
            i_ = s["actor_info"]["id"]["value"]
            if i_ in live:
                v = s.get("velocity", {})
                row.append(f"{i_} {math.hypot(v.get('x', 0.0), v.get('y', 0.0)):5.1f}")
        print(f"t{i:2d} " + "  ".join(sorted(row)), flush=True)
        time.sleep(1.0)
finally:
    for a in live:
        call("DestroyActor", actor(a))
