#!/usr/bin/env python3
"""npc_probe5.py: after npc_probe4.py crashed the world (a scene reload with gRPC vehicles left over): only the official
ring spawn point as spawn pose, every spawned vehicle destroyed before any reload.
(B) ring: rv1 (slow) then rv2 (fast) from the ring spawn point, cruise speeds, destinations kept 120 deg ahead
counter-clockwise: do they circle, does rv2 slow behind rv1; then the Ego parked on the ring in front of rv2.
(C) scene movers with activeDistance 500, held by SetPause right after the load, then released."""
import json
import math
import os
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import eval_manager as em  # noqa: E402

em.BC = "local"
em.LAUNCHER = os.path.expanduser("~/Workspaces/scratches/morai/MoraiLauncher_Lin_Data")
em.SCEN = em.LAUNCHER + "/SaveFile/Scenario/R_KR_PR_K-city_2025"
sim = em.Sim("127.0.0.1:7789")
V, P, O = "OBJECT_TYPE_VEHICLE", "OBJECT_TYPE_PEDESTRIAN", "OBJECT_TYPE_OBSTACLE"
RC, RR = (-99.5, 342.0), 19.7
SPAWN = {"location": {"x": -111.34, "y": 358.26, "z": 28.6}, "rotation": {"x": 0.0, "y": 0.0, "z": -136.2}}
spawned = []


def actor(aid, typ):
    return {"id": {"value": str(aid)}, "object_type": typ, "client_key": ""}


def call(m, req):
    r = sim.call(m, req)
    print(f"  {m} -> {json.dumps(r)[:160]}", flush=True)
    return r


def pos(aid, typ=V):
    loc = sim.call("GetActorState", actor(aid, typ)).get("transform", {}).get("location", {})
    return loc.get("x", 0.0), loc.get("y", 0.0)


def ring(aid):
    x, y = pos(aid)
    return math.atan2(y - RC[1], x - RC[0]), math.hypot(x - RC[0], y - RC[1]), (x, y)


def ahead(aid):
    a, _, _ = ring(aid)
    b = a + math.radians(120)
    return sim.call("SetVehicleDestination", {"actor_info": actor(aid, V), "decision_range": 30.0,
                                              "position": {"x": RC[0] + RR * math.cos(b), "y": RC[1] + RR * math.sin(b), "z": 28.6}})


def spawn(vid, kph, model="2015_Kia_K5"):
    r = call("SpawnVehicle", {"spawn_info": {"actor_info": actor(vid, V), "transform": SPAWN, "model_name": model, "label": vid},
                              "velocity": kph / 3.6})
    if r.get("status") == 1:
        spawned.append(vid)
        call("SetPhysics", {"actor_info": actor(vid, V), "enable": True})   # without these two it stood (probe5 run 2)
        call("SetPause", {"actor_info": actor(vid, V), "enable": False})
        call("SetVehicleEgoCruise", {"actor_info": actor(vid, V), "cruise_on": True, "cruise_type": "EGO_CRUISE_TYPE_CONSTANT",
                                     "constant_velocity": kph / 3.6})
        print("  dest", ahead(vid), flush=True)


def watch(secs, ids):
    for i in range(int(secs)):
        row = []
        for vid in ids:
            try:
                a, r, (x, y) = ring(vid)
                if i % 2 == 0:
                    ahead(vid)
                row.append(f"{vid}:{math.degrees(a):6.0f}deg r{r:5.1f}")
            except Exception as e:   # noqa: BLE001
                row.append(f"{vid}:ERR {str(e)[:40]}")
        if len(ids) == 2 and all(v in spawned for v in ids):
            p, q = pos(ids[0]), pos(ids[1])
            row.append(f"gap {math.hypot(p[0] - q[0], p[1] - q[1]):5.1f}")
        print(f"  t{i:3d} " + "  ".join(row), flush=True)
        time.sleep(1.0)


try:
    print(em.msc("play"), flush=True)
    time.sleep(3)
    print("\n### B ring: rv1 11 kph, rv2 22 kph 6 s later", flush=True)
    spawn("rv1", 11.0)
    for _ in range(15):   # the spawn point takes the next car only once the last one has left it (NOT_CREATE_OBJECT at 6 s)
        time.sleep(1.0)
        ahead("rv1")
    spawn("rv2", 22.0)
    watch(40, [v for v in ("rv1", "rv2") if v in spawned])
    print("\n### B2 Ego parked on the ring 25 deg ahead of rv2", flush=True)
    a, _, _ = ring(spawned[-1])
    b = a + math.radians(25)
    x, y = RC[0] + RR * math.cos(b), RC[1] + RR * math.sin(b)
    print("  put ego", em.Sim.put(sim, x, y, 28.9, math.degrees(b) + 90.0), flush=True)
    watch(15, [spawned[-1]])
    print("  ego", pos("Ego"), spawned[-1], pos(spawned[-1]), flush=True)
finally:
    for vid in spawned:
        call("DestroyActor", actor(vid, V))
time.sleep(2)
sys.exit(0)   # C: npc_probe6.py (SetPause freezes pedestrians for good, does not hold props)
print("\n### C movers held by SetPause", flush=True)
name, _, meta = em.make_scene(7)
p = Path(em.SCEN) / f"{name}.json"
d = json.loads(p.read_text())
ids = []
for o in d["pedestrianList"] + d["objectList"]:
    if o.get("activeDistance", 0) > 0:
        o["activeDistance"] = 500.0
        ids.append((str(o["UNIQUEID"]), P if o in d["pedestrianList"] else O))
p.write_text(json.dumps(d, ensure_ascii=False))
print(em.msc("load-scene", name + "/62", 90), flush=True)   # no MSC pause first: pause + load crashed the world twice
for aid, typ in ids:
    call("SetPause", {"actor_info": actor(aid, typ), "enable": True})
print(em.msc("play"), flush=True)
for i in range(4):
    print("  held", i, [(aid, tuple(round(v, 2) for v in pos(aid, typ))) for aid, typ in ids], flush=True)
    time.sleep(1.0)
for aid, typ in ids:
    call("SetPause", {"actor_info": actor(aid, typ), "enable": False})
for i in range(6):
    print("  released", i, [(aid, tuple(round(v, 2) for v in pos(aid, typ))) for aid, typ in ids], flush=True)
    time.sleep(1.0)
print(em.msc("pause"))
