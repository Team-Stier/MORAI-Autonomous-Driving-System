#!/usr/bin/env python3
"""ring_probe.py LAUNCHER_DATA [OBJINFO_PORT] [OBS_S] [multi]: which vehicle types the roundabout spawn point (clcol
UNIQUEID 4) really spawns (10/28: every roundabout NPC of the eval laps was the base's 30100004). One scene per case from
clcol: the point's spawnVehicleTypeID and listVehicleLengthType set, every 3 s, the other spawn points and the initial
NPCs near the roundabout removed, the Ego 15 m before CP8. Played OBS_S (40) s; Object Info vehicles within R_M of the
point count as spawned, with their boxes (a type's box = its own case's). Types without a spawn are tried again with
every length class. multi: type -1 plus two copies of the point with set types (they spawned nothing).
Runs on the simulator host (eval3.sh world 1) -> stdout + ring_probe.json
"""
import json
import math
import sys
import time
from collections import Counter
from pathlib import Path

import eval_manager as em
import scenario_variants as sv

R_M, POINT = 70.0, 4
LENGTHS_ALL = [0, 1, 2, 3, 4]


def main():
    em.BC, em.LAUNCHER = "local", sys.argv[1]
    em.SCEN = em.LAUNCHER + "/SaveFile/Scenario/R_KR_PR_K-city_2025"
    port = int(sys.argv[2]) if len(sys.argv) > 2 else 7505
    obs_s = float(sys.argv[3]) if len(sys.argv) > 3 else 40.0
    scen = Path(em.SCEN)
    base = json.load(open(scen / "clcol.json", encoding="utf-8-sig"))
    mn = open(scen / "EgoNetwork/clcol_MN.json", encoding="utf-8-sig").read()
    sp = next(p for p in base["spawnPointList"] if p["UNIQUEID"] == POINT)
    px, py = sp["pos"]["x"], sp["pos"]["y"]
    x, y, z, yaw = [float(v) for v in open(em.HERE / "checkpoints_20261007.csv").read().splitlines()[1 + 8].split(",")]
    ex, ey = x - 15 * math.cos(math.radians(yaw)), y - 15 * math.sin(math.radians(yaw))
    drain, out = em.objinfo(port), []
    near = lambda q: math.hypot(q["x"] - px, q["y"] - py) < R_M

    def run(k, vtype, lengths, extra=()):
        d = json.loads(json.dumps(base))
        p = next(q for q in d["spawnPointList"] if q["UNIQUEID"] == POINT)
        p.update(spawnVehicleTypeID=vtype, listVehicleLengthType=lengths, minSpawnPeriod=3.0, maxSpawnPeriod=3.0,
                 maximumSpawnVehicle=1000)
        d["spawnPointList"] = [p] + [dict(json.loads(json.dumps(p)), **e) for e in extra]
        d["vehicleList"] = [v for v in d["vehicleList"] if not near(v["initPosition"]["pos"])]
        d["egoVehicle"]["initPosition"]["pos"] = sv.vec(ex, ey, z + 0.35)
        d["egoVehicle"]["initPosition"]["rot"]["yaw"] = f"{yaw:.3f}"
        name = f"rp{k:02d}"
        json.dump(d, open(scen / f"{name}.json", "w"))
        open(scen / f"EgoNetwork/{name}_MN.json", "w").write(mn)
        em.msc("pause")
        r = em.msc("load-scene", name + "/62", 90)
        if not r.startswith("ok"):
            return dict(type=vtype, lengths=lengths, load=r[:120])
        drain()
        em.msc("play")
        t0, seen = time.time(), {}
        while time.time() - t0 < obs_s:
            time.sleep(0.5)
            for o in drain():                                             # vehicles only, not the Ego
                if o[1] == 1 and math.hypot(o[2] - ex, o[3] - ey) >= 3 and math.hypot(o[2] - px, o[3] - py) < R_M:
                    seen.setdefault(o[0], tuple(round(v, 2) for v in o[6:9]))
        em.msc("pause")
        return dict(type=vtype, lengths=lengths, extra=list(extra), load="ok", spawned=len(seen),
                    boxes=dict(Counter(map(str, seen.values()))))

    if sys.argv[4:5] == ["multi"]:                         # copies of the point with set types: the bus and the van
        out.append(run(0, -1, [2], [dict(UNIQUEID=900 + 2 * i, destinationObjUniqueID=901 + 2 * i, spawnVehicleTypeID=t,
                                         minSpawnPeriod=6.0, maxSpawnPeriod=6.0) for i, t in enumerate((30400004, 30300002))]))
        print(json.dumps(out[-1]), flush=True)
    else:
        cases = [(-1, [2]), (-1, LENGTHS_ALL)] + [(t, [2]) for t in sv.VEHICLES]
        for k, (t, ls) in enumerate(cases):
            out.append(run(k, t, ls)); print(json.dumps(out[-1]), flush=True)
        for t in [r["type"] for r in out if r["type"] != -1 and not r.get("spawned")]:
            out.append(run(len(out), t, LENGTHS_ALL)); print(json.dumps(out[-1]), flush=True)
    box_type = {b: r["type"] for r in out if r["type"] != -1 and not r["extra"] for b in r.get("boxes", {})}
    for r in out:
        r["types"] = {box_type.get(b, "?" + b): n for b, n in r.get("boxes", {}).items()}
        print(r["type"], r["lengths"], r["extra"], r["load"], r.get("spawned"), r["types"])
    json.dump(out, open("ring_probe.json", "w"), indent=1)


if __name__ == "__main__":
    main()
