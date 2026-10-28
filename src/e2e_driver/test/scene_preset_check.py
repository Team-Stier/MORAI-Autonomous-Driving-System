"""scenario_variants.py SCENE_PRESET=brake (DAgger dynamic/tunnel segments): every mover in the dynamic zone and the
tunnel crosses the route lane at a must-stop (or pass-in-front) timing, no roadside walkers, every tunnel has a mover,
the dynamic zone's movers are not set off from its stop line (s 436.8), no car props in the tunnel.
Builds 300 scenes from a stand-in base (the sample object at the static zone, the shade in the tunnel) on the route.
Mac: SCENE_PRESET=brake python3 scene_preset_check.py
"""
import collections
import math
import os
import random
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "tools"))
assert os.environ.get("SCENE_PRESET") == "brake", "run with SCENE_PRESET=brake"
import scenario_variants as sv  # noqa: E402

route = sv.Route(HERE.parents[1] / "map" / "2026_molit_comp_global_path.txt")
x, y, z, _ = route.at(329.0, -0.5)
sx, sy, _, _ = route.at(1930.0, 0.0)
base = dict(egoVehicle={"initPosition": {"pos": {}, "rot": {}}}, tlList=[], spawnPointList=[], waypointDataList=[],
            objectList=[{"pos": {"x": x, "y": y, "z": z}}], pedestrianList=[{}],
            shadedAreaList=[{"pos": {"x": sx, "y": sy, "z": 0.0}, "size": {"x": 60.0}}])
rng, tun, kinds = random.Random(1), collections.Counter(), collections.Counter()
for _ in range(300):
    _, meta = sv.make(base, route, rng, 1000)
    tun[meta["tunnel"]] += 1
    movers = [a for a in meta["actors"] if a["zone"].endswith("_dynamic")]
    assert any(a["zone"] == "ra_dynamic" for a in movers)
    assert meta["tunnel"] in ("dynamic", "both") and any(a["zone"] == "rb_dynamic" for a in movers), meta["tunnel"]
    for a in movers:
        assert a["timing"] in ("must_stop", "pass_front") and a["ptype"] in ("cross_lr", "cross_rl", "behind_parked"), a
        assert a["zone"] != "rb_dynamic" or a["cls"] != "car", a   # no car props in the one-lane tunnel
        if a["zone"] == "ra_dynamic":                 # not set off while the Ego waits at the s 436.8 stop line
            assert a["s"] >= 465.0 and a["active"] < math.hypot(a["s"] - 436.8, a["lat"]), a
        kinds[(a["zone"], a["timing"])] += 1
print("ok:", dict(tun), dict(kinds))
