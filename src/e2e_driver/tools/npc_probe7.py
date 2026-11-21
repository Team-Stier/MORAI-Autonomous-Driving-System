#!/usr/bin/env python3
"""npc_probe7.py: ring vehicles the way pv3 drove (npc_probe3.py): spawn at 0, SetPhysics on, SetPause off, cruise,
destination; positions only from GetAllActorsState (GetActorState threw on fresh actors). rv1 slow, rv2 fast once rv1 has
left the spawn point: circling with destinations kept 120 deg ahead, the gap (does rv2 slow behind rv1), then the Ego set on
the ring in front of rv2 (does it stop)."""
import json
import math
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import eval_manager as em  # noqa: E402

em.BC = "local"
sim = em.Sim("127.0.0.1:7789")
V = "OBJECT_TYPE_VEHICLE"
RC, RR = (-99.5, 342.0), 19.7
SPAWN = (-111.34, 358.26, -136.2)
live = []


def actor(aid):
    return {"id": {"value": aid}, "object_type": V, "client_key": ""}


def call(m, req, quiet=False):
    try:
        r = sim.call(m, req)
    except Exception as e:   # noqa: BLE001
        r = {"error": str(e)[:80]}
    if not quiet:
        print(f"  {m} -> {json.dumps(r)[:120]}", flush=True)
    return r


def poses():
    st = call("GetAllActorsState", {"client_key": "", "vehicle": True}, quiet=True)
    return {s["actor_info"]["id"]["value"]: (s["transform"]["location"].get("x", 0.0), s["transform"]["location"].get("y", 0.0))
            for s in st.get("states", [])}


def dest(aid, p):
    a = math.atan2(p[1] - RC[1], p[0] - RC[0]) + math.radians(120)
    return call("SetVehicleDestination", {"actor_info": actor(aid), "decision_range": 30.0, "position": {
        "x": RC[0] + RR * math.cos(a), "y": RC[1] + RR * math.sin(a), "z": 28.6}}, quiet=True)


def spawn(aid, kph):
    r = call("SpawnVehicle", {"spawn_info": {"actor_info": actor(aid), "transform": {
        "location": {"x": SPAWN[0], "y": SPAWN[1], "z": 28.6}, "rotation": {"x": 0.0, "y": 0.0, "z": SPAWN[2]}},
        "model_name": "2015_Kia_K5", "label": aid}, "velocity": 0.0})
    if r.get("status") != 1:
        return False
    live.append(aid)
    call("SetPhysics", {"actor_info": actor(aid), "enable": True})
    call("SetPause", {"actor_info": actor(aid), "enable": False})
    call("SetVehicleEgoCruise", {"actor_info": actor(aid), "cruise_on": True, "cruise_type": "EGO_CRUISE_TYPE_CONSTANT",
                                 "constant_velocity": kph / 3.6})
    p = poses().get(aid)
    print("  dest", dest(aid, p) if p else "no pose", flush=True)
    return True


def tick(i, show):
    ps = poses()
    for aid in live:
        if aid in ps and i % 2 == 0:
            dest(aid, ps[aid])
    if show:
        row = [f"{a}: {math.degrees(math.atan2(ps[a][1] - RC[1], ps[a][0] - RC[0])):5.0f}deg r{math.hypot(ps[a][0] - RC[0], ps[a][1] - RC[1]):5.1f}"
               for a in live if a in ps]
        if "rv1" in ps and "rv2" in ps:
            row.append(f"gap {math.hypot(ps['rv1'][0] - ps['rv2'][0], ps['rv1'][1] - ps['rv2'][1]):5.1f}")
        print(f"  t{i:3d} " + "  ".join(row), flush=True)
    return ps


try:
    print(em.msc("play")[:40], flush=True)
    time.sleep(2)
    spawn("rv1", 11.0)
    for i in range(30):   # until rv1 is 15 m away from the spawn point
        ps = tick(i, i % 3 == 0)
        if "rv1" in ps and math.hypot(ps["rv1"][0] - SPAWN[0], ps["rv1"][1] - SPAWN[1]) > 15:
            break
        time.sleep(1.0)
    print("  rv2:", spawn("rv2", 22.0), flush=True)
    for i in range(40):
        tick(i, True)
        time.sleep(1.0)
    ps = poses()
    if "rv2" in ps:
        a = math.atan2(ps["rv2"][1] - RC[1], ps["rv2"][0] - RC[0]) + math.radians(25)
        x, y = RC[0] + RR * math.cos(a), RC[1] + RR * math.sin(a)
        print("  ego on the ring", em.Sim.put(sim, x, y, 28.9, math.degrees(a) + 90.0), flush=True)
        for i in range(15):
            ps = tick(i, False)
            e, v = ps.get("Ego"), ps.get("rv2")
            print(f"  t{i:3d} ego-rv2 {math.hypot(e[0] - v[0], e[1] - v[1]):5.1f} m" if e and v else "  -", flush=True)
            time.sleep(1.0)
finally:
    for aid in live:
        call("DestroyActor", actor(aid))
print(em.msc("pause")[:40])
