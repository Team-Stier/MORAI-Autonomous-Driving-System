#!/usr/bin/env python3
"""npc_probe6.py: can the manager hold a scene mover and release it on its own clock? ev7 with every mover's activeDistance
500 (MORAI starts them at once), SetPause on each right after the load (no MSC pause first: pause + load crashed the world),
positions for 4 s held, then released, positions for 8 s; then held again mid-walk and released."""
import json
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
P, O = "OBJECT_TYPE_PEDESTRIAN", "OBJECT_TYPE_OBSTACLE"


def actor(aid, typ):
    return {"id": {"value": str(aid)}, "object_type": typ, "client_key": ""}


def pos(aid, typ):
    try:
        loc = sim.call("GetActorState", actor(aid, typ)).get("transform", {}).get("location", {})
        return round(loc.get("x", 0.0), 2), round(loc.get("y", 0.0), 2)
    except Exception as e:   # noqa: BLE001
        return f"ERR {str(e)[:30]}"


def pause(ids, on):
    return [sim.call("SetPause", {"actor_info": actor(a, t), "enable": on}).get("description") for a, t in ids]


name, _, meta = em.make_scene(7)
p = Path(em.SCEN) / f"{name}.json"
d = json.loads(p.read_text())
ids = []
for o in d["pedestrianList"] + d["objectList"]:
    if o.get("activeDistance", 0) > 0:
        o["activeDistance"] = 500.0
        ids.append((str(o["UNIQUEID"]), P if o in d["pedestrianList"] else O))
p.write_text(json.dumps(d, ensure_ascii=False))
print("movers", ids, flush=True)
print(em.msc("load-scene", name + "/62", 90)[:60], flush=True)
print("hold", pause(ids, True), flush=True)
print(em.msc("play")[:40], flush=True)
print("hold again", pause(ids, True), flush=True)
for i in range(4):
    print(" held", i, [pos(a, t) for a, t in ids], flush=True)
    time.sleep(1.0)
print("release", pause(ids, False), flush=True)
for i in range(8):
    print(" walking", i, [pos(a, t) for a, t in ids], flush=True)
    time.sleep(1.0)
print("hold mid-walk", pause(ids, True), flush=True)
for i in range(3):
    print(" held", i, [pos(a, t) for a, t in ids], flush=True)
    time.sleep(1.0)
print("release", pause(ids, False), flush=True)
for i in range(4):
    print(" walking", i, [pos(a, t) for a, t in ids], flush=True)
    time.sleep(1.0)
