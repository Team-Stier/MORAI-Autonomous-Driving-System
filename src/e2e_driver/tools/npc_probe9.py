#!/usr/bin/env python3
"""npc_probe9.py: speed control for gRPC route cars (probe8: SetVehicleEgoCruise CONSTANT is ignored, cars run the link
limit). One car per highway spawn point, the director's call sequence, then:
  pz6   SetVehicleDynamicsSpeedLimit on, 40       (km/h -> ~40; m/s -> no cap below the 60 limit)
  pz8   SetVehicleDynamicsSpeedLimit on, 11.1     (km/h -> ~11; m/s -> ~40)
  pz10  SetVehicleEgoCruise LINK, link_speed_ratio 70   (percent of the link limit -> ~42)
Speeds (km/h) for 35 s, then every probe car destroyed. Runs beside a lap on a playing world (spawns may be refused while
the director's cars hold a point: retried)."""
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
V = "OBJECT_TYPE_VEHICLE"
live, todo = [], {}


def actor(aid):
    return {"id": {"value": aid}, "object_type": V, "client_key": ""}


def call(m, req, quiet=False):
    try:
        r = sim.call(m, req)
    except Exception as e:   # noqa: BLE001
        r = {"error": str(e)[:80]}
    if not quiet:
        print(f"  {m} -> {json.dumps(r)[:100]}", flush=True)
    return r


def spawn(aid, h, extra):
    r = call("SpawnVehicle", {"spawn_info": {"actor_info": actor(aid), "transform": wd.transform(*h["pose"]),
                                             "model_name": "2015_Kia_K5", "label": aid}, "velocity": 0.0}, quiet=True)
    if r.get("status") != 1 and r.get("description") != "SUCCESS":
        return False
    live.append(aid)
    a = actor(aid)
    call("SetPhysics", {"actor_info": a, "enable": True}, True)
    call("SetPause", {"actor_info": a, "enable": False}, True)
    for m, req in extra:
        call(m, dict(req, actor_info=a))
    call("SetVehicleRoute", {"actor_info": a, "decision_range": 3.0,
                             "links": [{"id": {"value": l}, "waypoint_idx": w} for l, w in h["route"]]}, True)
    print(f"  {aid} spawned", flush=True)
    return True


hw = {h["sp"]: h for h in wd.HIGHWAY}
todo = {"pz6": (hw[6], [("SetVehicleDynamicsSpeedLimit", {"speed_limiter": True, "speed_limit": 40.0})]),
        "pz8": (hw[8], [("SetVehicleDynamicsSpeedLimit", {"speed_limiter": True, "speed_limit": 11.1})]),
        "pz10": (hw[10], [("SetVehicleEgoCruise", {"cruise_on": True, "cruise_type": "EGO_CRUISE_TYPE_LINK",
                                                   "link_speed_ratio": 70})])}
try:
    t0 = time.time()
    while time.time() - t0 < 50:
        for aid in [a for a in todo if a not in live]:
            spawn(aid, *todo[aid])
        st = sim.call("GetAllActorsState", {"client_key": "", "vehicle": True})
        row = []
        for s in st.get("states", []):
            i = s["actor_info"]["id"]["value"]
            if i in live:
                v = s.get("velocity", {})
                row.append(f"{i} {math.hypot(v.get('x', 0.0), v.get('y', 0.0)):5.1f}")
        print(f"t{time.time() - t0:5.1f} " + "  ".join(sorted(row)), flush=True)
        time.sleep(1.0)
finally:
    for a in live:
        call("DestroyActor", actor(a))
