"""Mac side: everything the soft-target rule check (scripts/rule_check.py) needs on the training box besides the step-0
_ego tables (per-row pose, route progress, GT signal code, GPS blackout), in one small npz:
- judged solid lane-line segments with their paint half widths, judged stop lines, the global path and the speed-exempt
  route span: tools/scorer KMap + thresholds.yaml (the competition rules the labels were judged with);
- vehicle geometry: wheel contact points, tyre half width, front axle (thresholds.yaml), body box;
- per episode its static scenario obstacles (manifest actors, zone static / tunnel_static) as boxes in map coordinates.
Usage (cwd train/): uv run --with pandas,pyarrow,pyyaml,numpy,scipy python -m morai_vla.rule_context assets/rule_context.npz
"""
import sys
from pathlib import Path

import numpy as np

TOOLS = Path(__file__).resolve().parents[2] / "tools"
sys.path.insert(0, str(TOOLS / "coc_labeler"))
from coc_labeler import data   # noqa: E402  (KMap, thresholds, route geometry, scene log, manifests)

BODY = (-1.0, data.BUMPER, 0.95)        # base_link (rear axle) box: rear, front, half width (Ioniq5 1.89 m wide)
# shortcut: Object Info reports props unscaled and there is no per-asset size table (DATA_PLAN 10/21); props are taken as
# 1.0 m x scale, parked-vehicle props as 4.6 x 1.9 m. Upgrade with measured asset footprints if rule_ok rates look off.
PROP_M, VEHICLE_LW = 1.0, (4.6, 1.9)


def obstacles(ep):
    """static scenario obstacles of an episode: rows (cx, cy, heading, half length, half width) in map coordinates."""
    _, sc = data.scene_of(ep)
    out = []
    for a in (sc or {}).get("actors", []):
        if not a["zone"].endswith("static"): continue
        x, y, h = data.route_point(a["s"], a["lat"])
        L, W = VEHICLE_LW if a.get("id") in data.VEHICLE_PROPS else (PROP_M * a["scale"][0], PROP_M * a["scale"][1])
        out.append((x, y, h, L / 2, W / 2))
    return out


def main(out):
    km, cfg = data.kmap(), data.load_config()
    sig = cfg["signal"]
    judged = [sid for sid in sig["stop_lines"] if sid in km.route_stops]
    stops = [km.stop_lines[sid] for sid in judged]
    eps = sorted(data.scene_index())
    obs = [(k, *o) for k, ep in enumerate(eps) for o in obstacles(ep)]
    veh = cfg["vehicle"]
    np.savez_compressed(
        out, seg_a=km.seg_a, seg_b=km.seg_b, seg_hw=km.seg_hw, path=km.path, path_s=km.s, exempt_s=np.array(km.speed_exempt_s),
        stop_ids=np.array(judged), stop_pts=np.concatenate(stops), stop_off=np.cumsum([0] + [len(p) for p in stops]),
        wheels=np.array(list(veh["wheels_base_link_m"].values()), float), tire_hw=veh["tire_half_width_m"], front_axle=veh["front_axle_m"],
        body=np.array(BODY), speed_limit_kph=cfg["speed"]["limit_kph"], lane_period_s=cfg["lane"]["period_s"],
        violating=np.array(sig["violating_states"]), episodes=np.array(eps), obs=np.array(obs, float).reshape(-1, 6),
        thresholds_hash=cfg["_hash"])
    print(f"{out}: {len(km.seg_a)} judged lane segments, {len(judged)} stop lines, {len(eps)} episodes, {len(obs)} static obstacles")


if __name__ == "__main__":
    main(sys.argv[1])
