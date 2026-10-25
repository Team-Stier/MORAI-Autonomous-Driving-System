"""Mac side: write assets/route.json from the training code, so the live node builds nav text exactly as
train/morai_vla/build_index.py did and tracks route progress exactly as the scorer's KMap.progress did (path_s of the
training rows came from it, on the GT base_link pose).

Usage (cwd: Alpamayo-Cosmos/train, the build_index environment):
  uv run --with pandas,pyarrow,pyyaml,numpy,scipy python <this file> <out.json>
"""
import hashlib, json, sys
from pathlib import Path

sys.path.insert(0, ".")
from morai_vla import build_index as B          # noqa: E402  (imports coc_labeler.data -> scorer KMap)

data = B.data


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()[:16]


def main(out):
    km = data.kmap()
    M = B.maneuvers()
    lap = float(data.path_xy()[1][-1])
    asset = {
        "about": "route geometry and nav-text tables for e2e_driver (tools/make_route_asset.py)",
        "nav_range_m": B.NAV_RANGE,
        "lap_m": lap,
        "exempt_s": [float(km.speed_exempt_s[0]), float(km.speed_exempt_s[1])],
        "maneuvers": [[float(p), float(e), t] for p, e, t in M],
        "path": [[float(x), float(y)] for x, y in km.path],
        "s": [float(v) for v in km.s],
        "heading": [float(v) for v in km.heading],
        "checkpoint_s": [float(v) for v in km.cp_s],
        "sources": {
            "build_index.py": sha(B.__file__), "route.yaml": sha(data.PKG / "route.yaml"),
            "scorer.py": sha(data.TOOLS / "scorer" / "scorer.py"),
        },
    }
    Path(out).write_text(json.dumps(asset, separators=(",", ":")))
    print(f"{out}: {len(M)} maneuvers, path {len(km.path)} pts, lap {lap:.1f} m, exempt {asset['exempt_s']}")


if __name__ == "__main__":
    main(sys.argv[1])
