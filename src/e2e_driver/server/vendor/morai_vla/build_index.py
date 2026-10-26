"""Mac side: one sample index for teacher/student training (MODEL_PLAN.md step 0 outputs -> index.parquet + splits.json).

Per 10 Hz tau-grid row of every episode: traj_ok on the sim clock, cell, nav_text, split. CoC is not baked in: the
dataset joins a CoC parquet given in its config (data.coc_parquet), so a relabel needs no rebuild.
Only rows that can be trained on are written: 1.5 s of history, 6.4 s of future, traj_ok, no speed glitch.
Usage: uv run --with pandas,pyarrow,pyyaml,numpy,scipy python -m morai_vla.build_index OUT_DIR   (cwd: train/)
"""
import argparse, json, sys, zlib
from pathlib import Path

import numpy as np, pandas as pd

TOOLS = Path(__file__).resolve().parents[2] / "tools"
sys.path.insert(0, str(TOOLS / "coc_labeler"))
from coc_labeler import data   # noqa: E402  (route.yaml, labels cache, manifests, scene log, map)

EGO = Path.home() / ".cache/step0_ego"
HIST, FUT, HORIZON_S = 16, 64, 6.4
NAV_RANGE = 150.0                     # m: farther maneuvers -> "Follow the road"
ASSET_WINDOW = (-120.0, 30.0)         # m of route s around a held-out actor whose rows leave train
MAX_DV = 1.5                          # m/s per 0.1 s (15 m/s^2): beyond any braking; GT speed glitches / impacts


def traj_ok_on_grid(G, L):
    """label traj_ok is judged over [t, t + 6.4 s] of WALL time; the sim ran at 0.7-0.95x, so re-judge on tau:
    row i is ok iff every label row from tau_i until one whose window reaches tau_i + 6.4 is ok."""
    tau_of = lambda ns: np.interp(ns, G["bag_ns"], G["tau"], right=np.inf)
    lt = tau_of(L.bag_time_ns.to_numpy(np.float64))
    end = tau_of(L.bag_time_ns.to_numpy(np.float64) + HORIZON_S * 1e9)          # window end of each label row (inf: past the bag)
    end = np.maximum.accumulate(np.where(np.isfinite(end), end, np.inf))
    bad = np.r_[0, np.cumsum(~L.traj_ok.to_numpy(bool))]
    j0 = np.searchsorted(lt, G["tau"]); j1 = np.searchsorted(end, G["tau"] + HORIZON_S)
    return (j1 < len(L)) & (bad[np.minimum(j1 + 1, len(L))] - bad[j0] == 0)


def maneuvers():
    """(route s of the maneuver point, text with {d}) from route.yaml."""
    out = []
    for f in data.ROUTE["features"]:
        if f["type"] == "intersection":
            out.append((f["stop_s"], f["s"][1], "Go straight through the intersection in {d}m" if f["movement"] == "straight"
                        else f"Turn {f['movement']} in {{d}}m"))
        elif f["type"] == "roundabout":
            out += [(f["entry_s"], f["entry_s"], "Enter the roundabout in {d}m"), (f["exit_s"], f["exit_s"], "Exit the roundabout in {d}m")]
        elif f.get("ramp"):
            out.append((f["s"][0], f["s"][0], "Take the ramp on the right in {d}m"))
        elif f["type"] == "merge":
            out.append((f["cross_s"], f["cross_s"], "Merge left in {d}m"))
        elif f["type"] == "toll_plaza":
            out.append((f["gate_s"], f["gate_s"], "Go through the toll gate in {d}m"))
    return sorted(out)


def nav_text(s, M, lap, exempt):
    """next maneuver within NAV_RANGE (distance clamped at 0 inside its junction), plus the speed rule."""
    text = "Follow the road"
    for k in (0, 1):                                           # this lap, then the next lap's first maneuvers
        hit = [(p + k * lap, t) for p, end, t in M if s <= end + k * lap and p + k * lap - s <= NAV_RANGE]
        if hit:
            p, t = hit[0]; text = t.format(d=int(round(max(p - s, 0.0)))); break
    return text + (". No speed limit" if exempt[0] <= s <= exempt[1] else ". Speed limit 60 km/h")


def held_out_assets(frac, seed):
    ids = set()
    for f in data.MANIFESTS.glob("*-manifest.json"):
        for v in json.load(open(f)).values():
            ids |= {a["id"] for a in v.get("actors", []) if a.get("id")}
    rng = np.random.default_rng(seed); out = []
    for group in (401, 501):                                   # obstacles 40100xxx, pedestrians 50100xxx
        g = sorted(i for i in ids if i // 100000 == group)
        out += sorted(int(x) for x in rng.choice(g, max(1, round(frac * len(g))), replace=False))
    return out


def unit_hash(*key):
    return zlib.crc32("/".join(map(str, key)).encode()) / 2**32


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("out"); ap.add_argument("--val-frac", type=float, default=0.05)
    ap.add_argument("--asset-frac", type=float, default=0.15); ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args(); out = Path(a.out); out.mkdir(parents=True, exist_ok=True)
    M, lap, exempt = maneuvers(), float(data.path_xy()[1][-1]), data.kmap().speed_exempt_s
    held = held_out_assets(a.asset_frac, a.seed)
    rows, stats = [], {"episodes": 0, "no_labels": [], "glitch_rows": 0}
    for f in sorted(EGO.glob("*.npz")):
        ep = f.stem
        try:
            L = pd.read_parquet(data.labels_file(ep), columns=["bag_time_ns", "traj_ok"])
        except FileNotFoundError:
            stats["no_labels"].append(ep); continue
        z = np.load(f); G = {k: z[k] for k in ("tau", "bag_ns", "path_s", "section", "fog", "hour", "v")}
        n = len(G["tau"]); i = np.arange(n)
        ok = traj_ok_on_grid(G, L) & (i >= HIST - 1) & (i + FUT < n)
        jump = np.r_[0, np.cumsum(np.abs(np.diff(G["v"].astype(float))) > MAX_DV)]          # trajectories integrate GT speed
        glitch = (jump[np.minimum(i + FUT, n - 1)] - jump[np.maximum(i - HIST + 1, 0)]) > 0
        stats["glitch_rows"] += int((ok & glitch).sum()); ok &= ~glitch
        sc_id, sc = data.scene_of(ep)
        unit = sc_id or ep                                     # scenario seed = scene variant; old runs without one: the episode
        split = np.full(n, "val" if unit_hash(a.seed, unit) < a.val_frac else "train", object)
        for act in (sc or {}).get("actors", []):
            if act.get("id") in held:
                near = (G["path_s"] >= act["s"] + ASSET_WINDOW[0]) & (G["path_s"] <= act["s"] + ASSET_WINDOW[1])
                split[near & (split == "train")] = "val_asset"
        cells = data.ROUTE["cells"]
        for k in np.flatnonzero(ok):
            rows.append((ep, int(k), float(G["tau"][k]), cells.get(int(G["section"][k]), "C?"), split[k],
                         nav_text(float(G["path_s"][k]), M, lap, exempt), bool(G["fog"][k]), int(G["hour"][k]), sc_id or ""))
        stats["episodes"] += 1
    df = pd.DataFrame(rows, columns=["episode", "i", "tau", "cell", "split", "nav_text", "fog", "hour", "scene"])
    for c in ("episode", "cell", "split", "nav_text", "scene"): df[c] = df[c].astype("category")
    df.to_parquet(out / "index.parquet", index=False)
    summary = dict(stats, rows=len(df), held_out_assets=held, val_frac=a.val_frac, seed=a.seed,
                   by_split=df.split.value_counts().to_dict(),
                   hours_by_cell={c: round(n / 36000, 2) for c, n in df[df.split == "train"].cell.value_counts().items()})
    json.dump(summary, open(out / "splits.json", "w"), indent=1, default=str)
    print(json.dumps({k: v for k, v in summary.items() if k != "no_labels"}, indent=1, default=str))


if __name__ == "__main__":
    main()
