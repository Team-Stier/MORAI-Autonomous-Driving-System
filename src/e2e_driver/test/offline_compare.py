"""Offline parity test, step 2 (training venv): the requests offline_replay.py dumped, run through the model server's
assembly (server/assemble.py), against morai_vla.data.MoraiDataset.sample of the same rows (nominal frame spacing, as at
inference) and the processor batch both produce.

Usage: MORAI_VLA_TRAIN=~/morai-student/train python offline_compare.py --root <step0 share> --episode EP \
           --config <dir with config.json> --vlm Qwen/Qwen3-VL-4B-Instruct --out report.json DUMP_DIR [DUMP_DIR ...]
"""
import argparse, json, os, sys, tempfile
from pathlib import Path

import numpy as np
import pandas as pd
import torch

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE.parent / "server"), str(HERE.parent / "src"), os.path.expanduser(os.environ.get("MORAI_VLA_TRAIN", "~/morai-student/train"))]
os.environ.setdefault("HF_HUB_OFFLINE", "1")

from assemble import build_sample, tokenize      # noqa: E402
from e2e_driver import wire                      # noqa: E402
from morai_vla.data import MoraiDataset          # noqa: E402
from morai_vla.model import load_config          # noqa: E402


def diff_u8(a, b):
    a, b = a.to(torch.int16), b.to(torch.int16)
    d = (a - b).abs()
    return dict(equal=bool(torch.equal(a, b)), frac_diff=round(float((d > 0).float().mean()), 6), max_abs=int(d.max()))


def compare(ds, idx, req, cfg):
    tr, nd = ds.sample(idx, rng=None), build_sample(req)
    r = {}
    cams = [diff_u8(a, b) for a, b in zip(tr["camera_frames"], nd["camera_frames"])]
    r["camera_equal"] = all(c["equal"] for c in cams)
    r["camera_frac_diff"] = max(c["frac_diff"] for c in cams)
    r["camera_times_maxdiff"] = float(np.abs(tr["camera_times"] - nd["camera_times"]).max())
    if "bev_frames" in tr and "bev_frames" in nd:
        per = [diff_u8(tr["bev_frames"][:, c], nd["bev_frames"][:, c]) for c in range(3)]
        r["bev_equal"] = all(p["equal"] for p in per)
        r["bev_frac_diff_channels"] = [p["frac_diff"] for p in per]
        occ = (tr["bev_frames"][:, 0] > 0) | (nd["bev_frames"][:, 0] > 0)
        r["bev_occupied_iou"] = round(float(((tr["bev_frames"][:, 0] > 0) & (nd["bev_frames"][:, 0] > 0)).sum() / max(int(occ.sum()), 1)), 4)
        r["bev_times_maxdiff"] = float(np.abs(tr["bev_times"] - nd["bev_times"]).max())
    r["hist_xyz_maxdiff_m"] = float((tr["ego_history_xyz"] - nd["ego_history_xyz"]).abs().max())
    r["hist_rot_maxdiff"] = float((tr["ego_history_rot"] - nd["ego_history_rot"]).abs().max())
    r["nav_equal"] = tr["nav_text"] == nd["nav_text"]
    if not r["nav_equal"]: r["nav"] = [tr["nav_text"], nd["nav_text"]]
    bt, bn = tokenize(tr, cfg), tokenize(nd, cfg)
    ti, ni = bt["tokenized_data"]["input_ids"], bn["tokenized_data"]["input_ids"]
    r["input_ids_equal"] = ti.shape == ni.shape and bool(torch.equal(ti, ni))
    pt, pn = bt["tokenized_data"]["pixel_values"], bn["tokenized_data"]["pixel_values"]
    r["pixel_values_maxdiff"] = float((pt - pn).abs().max()) if pt.shape == pn.shape else float("nan")
    r["hist_tensor_maxdiff_m"] = float((bt["ego_history_xyz"] - bn["ego_history_xyz"]).abs().max())
    return r


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", required=True); ap.add_argument("--episode", required=True)
    ap.add_argument("--config", required=True); ap.add_argument("--vlm", default="Qwen/Qwen3-VL-4B-Instruct")
    ap.add_argument("--out", required=True); ap.add_argument("dumps", nargs="+")
    a = ap.parse_args()
    cfg = load_config(a.config, a.vlm)
    idx = pd.read_parquet(Path(a.root) / "_index/index.parquet")
    sub = idx[idx.episode == a.episode]
    tmp = Path(tempfile.mkdtemp()) / "index.parquet"; sub.to_parquet(tmp, index=False)
    ds = MoraiDataset(a.root, split=["train", "val", "val_asset"], frame_stride=1, augment=False, index=str(tmp))
    pos = {int(i): k for k, i in enumerate(ds.rows.i)}
    report = {}
    for dump in a.dumps:
        meta = json.load(open(Path(dump) / "meta.json")); rows = []
        for m in meta["rows"]:
            if not m["ok"] or m["i"] not in pos:
                rows.append(dict(i=m["i"], skipped=m.get("problems") or "not a usable training row")); continue
            req = wire.unpack((Path(dump) / ("req_%06d.bin" % m["i"])).read_bytes())
            rows.append(dict(i=m["i"], problems=m["problems"], **compare(ds, pos[m["i"]], req, cfg)))
        done = [r for r in rows if "skipped" not in r]
        keys = [k for k in done[0] if k not in ("i", "problems", "nav", "bev_frac_diff_channels")] if done else []
        summ = {}
        for k in keys:
            vals = [r[k] for r in done if k in r]
            summ[k] = (round(sum(vals) / len(vals), 4) if isinstance(vals[0], bool) else
                       dict(max=float(np.nanmax(vals)), median=float(np.nanmedian(vals))))
        report[meta["mode"]] = dict(rows=len(done), skipped=len(rows) - len(done), summary=summ,
                                    nav_mismatch=[r["nav"] + [r["i"]] for r in done if not r["nav_equal"]][:10], per_row=rows)
        print(meta["mode"], json.dumps(report[meta["mode"]]["summary"], indent=1))
    json.dump(report, open(a.out, "w"), indent=1)


if __name__ == "__main__":
    main()
