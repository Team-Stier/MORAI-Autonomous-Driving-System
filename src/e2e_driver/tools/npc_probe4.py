#!/usr/bin/env python3
"""npc_probe4.py: (A) highway spawn points: each car's speed and the spawn intervals per lane, 60 s; (B) two vehicles
spawned on the ring at different cruise speeds, destinations kept ahead: do they circle and does the faster one keep its
distance; (C) a scene with the movers' activeDistance 500: SetPause holds them, unpausing starts their walk."""
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
sim = em.Sim("127.0.0.1:7789")
V, P, O = "OBJECT_TYPE_VEHICLE", "OBJECT_TYPE_PEDESTRIAN", "OBJECT_TYPE_OBSTACLE"
RC, RR = (-99.5, 342.0), 19.7


def actor(aid, typ):
    return {"id": {"value": str(aid)}, "object_type": typ, "client_key": ""}


def pos(aid, typ):
    loc = sim.call("GetActorState", actor(aid, typ)).get("transform", {}).get("location", {})
    return loc.get("x", 0.0), loc.get("y", 0.0)


def allv():
    st = sim.call("GetAllActorsState", {"client_key": "", "vehicle": True})
    return {s["actor_info"]["id"]["value"]: (s["transform"]["location"].get("x", 0.0), s["transform"]["location"].get("y", 0.0))
            for s in st.get("states", [])}


def run(name, fn):
    print(f"\n### {name}", flush=True)
    try:
        r = fn()
        if r is not None:
            print(json.dumps(r, ensure_ascii=False, default=str)[:2500], flush=True)
        return r
    except Exception as e:   # noqa: BLE001
        print(f"ERROR {type(e).__name__}: {str(e)[:300]}", flush=True)


print(em.msc("play"), flush=True)
time.sleep(3)


def highway():   # vehicles on the highway (x 60-80) heading south: speed per car, first-seen time per lane
    seen, last = {}, {}
    t0 = time.time()
    speeds = {}
    while time.time() - t0 < 60:
        now = time.time() - t0
        for k, (x, y) in allv().items():
            if k == "Ego" or not (60 < x < 80 and -250 < y < 320):
                continue
            lane = round((x - 65.8) / 3.6)
            seen.setdefault(k, (round(now, 1), lane, round(y)))
            if k in last:
                (t1, x1, y1) = last[k]
                if now - t1 > 0.8:
                    speeds.setdefault(k, []).append(round(math.hypot(x - x1, y - y1) / (now - t1) * 3.6))
            last[k] = (now, x, y)
        time.sleep(1.0)
    return dict(first_seen=seen, kph_median={k: sorted(v)[len(v) // 2] for k, v in speeds.items() if v})


run("A highway 60 s", highway)


def ring_two():
    out = {}
    for vid, ang, kph in (("rv1", 200.0, 22.0), ("rv2", 140.0, 11.0)):   # rv1 behind rv2 going counter-clockwise?
        a = math.radians(ang)
        x, y = RC[0] + RR * math.cos(a), RC[1] + RR * math.sin(a)
        yaw = math.degrees(a) + 90.0   # tangent, counter-clockwise
        out[vid] = sim.call("SpawnVehicle", {"spawn_info": {"actor_info": actor(vid, V), "transform": {
            "location": {"x": x, "y": y, "z": 28.6}, "rotation": {"z": yaw}}, "model_name": "2015_Kia_K5", "label": vid},
            "velocity": kph / 3.6})
        sim.call("SetVehicleEgoCruise", {"actor_info": actor(vid, V), "cruise_on": True, "cruise_type": "EGO_CRUISE_TYPE_CONSTANT",
                                         "constant_velocity": kph / 3.6})
    tr = {"rv1": [], "rv2": []}
    for i in range(30):
        for vid in tr:
            x, y = pos(vid, V)
            a = math.atan2(y - RC[1], x - RC[0])
            tr[vid].append((round(math.degrees(a)), round(math.hypot(x - RC[0], y - RC[1]), 1)))
            if i % 2 == 0:
                for lead in (120, -120):
                    d = {"x": RC[0] + RR * math.cos(a + math.radians(lead)), "y": RC[1] + RR * math.sin(a + math.radians(lead)), "z": 28.6}
                    r = sim.call("SetVehicleDestination", {"actor_info": actor(vid, V), "decision_range": 30.0, "position": d})
                    if r.get("status") == 1:
                        tr[vid][-1] += (lead,)
                        break
        time.sleep(1.0)
    gaps = [round(math.hypot(*(lambda p, q: (p[0] - q[0], p[1] - q[1]))(pos("rv1", V), pos("rv2", V))), 1)]
    for vid in tr:
        sim.call("DestroyActor", actor(vid, V))
    return dict(spawn=out, track=tr, final_gap=gaps)


run("B two ring vehicles", ring_two)


def movers():
    name, _, meta = em.make_scene(7)
    p = Path(em.SCEN) / f"{name}.json"
    d = json.loads(p.read_text())
    ids = []
    for o in d["pedestrianList"] + d["objectList"]:
        if o.get("activeDistance", 0) > 0:
            o["activeDistance"] = 500.0
            ids.append((str(o["UNIQUEID"]), P if o in d["pedestrianList"] else O))
    p.write_text(json.dumps(d, ensure_ascii=False))
    print(em.msc("pause"), em.msc("load-scene", name + "/62", 90), flush=True)
    for aid, typ in ids:   # hold them before they can walk
        sim.call("SetPause", {"actor_info": actor(aid, typ), "enable": True})
    print(em.msc("play"), flush=True)
    time.sleep(3)
    for aid, typ in ids:
        sim.call("SetPause", {"actor_info": actor(aid, typ), "enable": True})
    hold = {aid: [tuple(round(v, 2) for v in pos(aid, typ)) for _ in range(3) if not time.sleep(1.0)] for aid, typ in ids}
    for aid, typ in ids:
        sim.call("SetPause", {"actor_info": actor(aid, typ), "enable": False})
    walk = {aid: [tuple(round(v, 2) for v in pos(aid, typ)) for _ in range(5) if not time.sleep(1.0)] for aid, typ in ids}
    return dict(ids=ids, held=hold, released=walk)


run("C movers held by SetPause, then released", movers)
print(em.msc("pause"))
