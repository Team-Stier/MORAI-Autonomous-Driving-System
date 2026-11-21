"""scenario_variants.py SCENE_PRESET=popout (near pop-out collection, user 10/29): one mover per zone (dynamic s 445-500,
tunnel 1885-1970) that crosses the whole road at 10-15 kph, start near/far/hidden 25/25/50 %, trigger 12-20 m ahead
(far shoulders: timed, 12-35 m), and a hidden start that the checker's visible() cannot see from the route
lane before the trigger while the near/far starts are in view from 30 m. Builds 400 scenes from a stand-in base.
Mac: SCENE_PRESET=popout python3 popout_check.py [Alpamayo-Cosmos tools/scorer]
"""
import collections
import math
import os
import random
import sys
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "tools"))
assert os.environ.get("SCENE_PRESET") == "popout", "run with SCENE_PRESET=popout"
import scenario_variants as sv  # noqa: E402

scorer = Path(sys.argv[1] if len(sys.argv) > 1 else HERE.parents[3] / "Alpamayo-Cosmos/tools/scorer")
sys.path.insert(0, str(scorer))
from label_check import visible  # noqa: E402
vis_cfg = yaml.safe_load(open(scorer / "label_check.yaml"))["visibility"]

route = sv.Route(HERE.parents[1] / "map" / "2026_molit_comp_global_path.txt")
x, y, z, _ = route.at(329.0, -0.5)
sx, sy, _, _ = route.at(1930.0, 0.0)
base = dict(egoVehicle={"initPosition": {"pos": {}, "rot": {}}}, tlList=[], spawnPointList=[], waypointDataList=[],
            objectList=[{"pos": {"x": x, "y": y, "z": z}}], pedestrianList=[{}],
            shadedAreaList=[{"pos": {"x": sx, "y": sy, "z": 0.0}, "size": {"x": 60.0}}])


def seen_from(ego_s, a, car):
    """visible() of mover a from the route lane centre ego_s, the parked car (if any) as the only other object."""
    ex, ey, _, eh = route.at(ego_s, 0.0)
    _, _, _, h = route.at(a["s"], 0.0)
    objs = [(1, "pedestrian", *a["xy"], math.radians(h), 0, 0, 0.7, 0.7)]
    if car:
        objs.append((2, "obstacle", *car["xy"], math.radians(h), 0, 0, 2 * sv.PROP_HALF_L[car["id"]], 1.9))
    return bool(visible(ex, ey, math.radians(eh), objs, vis_cfg)[0])


rng, starts, n_vis = random.Random(1), collections.Counter(), collections.Counter()
for _ in range(400):
    _, meta = sv.make(base, route, rng, 1000)
    assert meta["tunnel"] == "popout"
    movers = [a for a in meta["actors"] if a["zone"].endswith("_dynamic")]
    assert sorted(a["zone"] for a in movers) == ["ra_dynamic", "rb_dynamic"], movers
    for a in movers:
        ra = a["zone"] == "ra_dynamic"
        lo, hi = (445.0, 500.0) if ra else (1885.0, 1970.0)
        assert lo <= a["s"] <= hi and 10.0 <= a["kph"] <= 15.0 and a["cls"] in ("person", "other"), a
        assert a["id"] != 50100084, a
        assert a["lat"] * a["lat_end"] < 0 and min(abs(a["lat"]), abs(a["lat_end"])) > 1.93, a   # the whole road
        assert (12.0 <= a["lon"] <= 35.0 and a["timing"] == "must_stop") if a["start"] == "far" else (12.0 <= a["lon"] <= 20.0), a
        assert abs(a["active"] - math.hypot(a["lon"], a["lat"])) < 0.11, a
        starts[(a["zone"][:2], a["start"])] += 1
        car = next((o for o in meta["actors"] if o["zone"] == a["zone"][:2] + "_parked"), None)
        assert (car is not None) == (a["start"] == "hidden"), (a, car)
        if a["start"] == "hidden":                    # hidden from the route lane from 25 m out and farther
            for back in (40.0, 25.0):                  # 1.2-1.6 m clear of the nose (po29b): may show at the trigger or round the tunnel curve
                assert not seen_from(a["s"] - back, a, car), (back, a, car)
        else:                                         # in view from 30 m out
            n_vis[seen_from(a["s"] - 30.0, a, None)] += 1
assert n_vis[False] == 0, n_vis
share = {k: round(v / 400, 2) for k, v in sorted(starts.items())}
for z in ("ra", "rb"):
    assert abs(share[(z, "hidden")] - 2 / 3) < 0.08 and abs(share[(z, "near")] - 1 / 6) < 0.07, share
print("ok:", share, dict(n_vis))
