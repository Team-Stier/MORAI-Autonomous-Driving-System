#!/usr/bin/env python3
"""DAgger shadow debug report from a dagger_pod.sh run (Mac): per-scene BEV overlays of the policy laps (driven path,
label rollout, plan, official lane, GT objects, light), the label error of the expert's own drive (label from plan +
rollout vs the 6.4 s it really drove), label coverage (empty share and reasons, official-lane share) and the policy
drive's timing (late, model_ms, RTF) with the shadow attached.

    dagger_eval.py RUN_DIR [--route assets/route.json]     (RUN_DIR = ~/morai-eval/dagger/NAME: spool/, rec_w1/, results/)
Out: RUN_DIR/report/ (scene_*.png, errors.csv, summary.json)
"""
import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import dagger_label as dl  # noqa: E402

TL_NAMES = {0: "unknown", 1: "off", 2: "RED", 3: "YELLOW", 4: "GREEN", 5: "LEFT", 11: "GREEN+LEFT", 7: "RED+GREEN", 8: "RED+LEFT"}


def episodes(run, kind):
    """kind "" = the policy laps (NAME_w1), "drive_" = the expert's own laps (NAME_drive_w1)."""
    return sorted(p for p in (run / "spool").glob(f"*_{kind}w1/ep*") if (p / "dagger_debug.npz").exists()
                  and (kind or not p.parent.name.endswith("_drive_w1")))


def load(ep):
    d = dict(np.load(ep / "dagger_debug.npz"))
    z = np.load(ep / "dagger_labels.npz")
    d.update(ak=z["ak"], valid=z["valid"][:, 0], source=z["source"])
    for k in ("signal", "stop_d", "intervened"):
        if k in z.files:
            d[k] = z[k]
    if "rolled" not in d:
        d["rolled"] = d["valid"]
    return d


def checks(D):
    """label checker verdicts and reasons, signal and lane summaries over the decisions of D."""
    import collections
    V = np.concatenate([d["verdict"] for d in D]) if "verdict" in D[0] else np.array([])
    R = [r for d in D for r in d.get("check_reasons", [])]
    reasons = collections.Counter(x for r in R for x in str(r).split())
    sig = np.concatenate([d["signal"] for d in D]) if "signal" in D[0] else np.zeros(0)
    mode = collections.Counter(m for d in D for m in d.get("mode", []) if m)
    red = sig == 1
    red_cross = sum(("signal:" in str(r)) and not str(r).startswith("start") for d in D for r, g in zip(d.get("check_reasons", []), d.get("signal", [])) if g == 1)
    lane = np.array(["lane:" in str(r) for r in R], bool)
    return dict(verdicts=dict(collections.Counter(V.tolist())), reasons=dict(reasons.most_common(20)),
                signal_rows={k: int((sig == c).sum()) for k, c in (("unknown", 0), ("red", 1), ("yellow", 2), ("green", 3))},
                modes=dict(mode), red_rows_with_signal_reason=int(red_cross),
                lane_reason_pct=round(100 * float(lane.mean()), 2) if len(lane) else None,
                interventions_rows=int(sum(int(d.get("intervened", np.zeros(0, bool)).sum()) for d in D)))


def errors(d):
    """per decision: ADE, FDE, lateral/longitudinal mean abs (m) of label vs driven future (ego frame at t0), and the
    same at 1 / 3 s; lat/lon split along the driven path's local heading."""
    m = d["valid"] & d["ok_actual"]
    L, A = d["fxy"][m], d["fxy_actual"][m]
    e = L - A
    h = np.arctan2(*np.gradient(A, axis=1).transpose(2, 0, 1)[::-1])
    lon = e[..., 0] * np.cos(h) + e[..., 1] * np.sin(h); lat = -e[..., 0] * np.sin(h) + e[..., 1] * np.cos(h)
    n = np.hypot(e[..., 0], e[..., 1])
    return m, dict(ade=n.mean(1), fde=n[:, -1], lat=np.abs(lat).mean(1), lon=np.abs(lon).mean(1), lat_end=np.abs(lat[:, -1]),
                   lon_end=np.abs(lon[:, -1]), ade_1s=n[:, :10].mean(1), ade_3s=n[:, :30].mean(1),
                   a_rmse=np.sqrt(((d["ak"][m, :, 0] - d["ak_actual"][m, :, 0]) ** 2).mean(1)),
                   k_rmse=np.sqrt(((d["ak"][m, :, 1] - d["ak_actual"][m, :, 1]) ** 2).mean(1)))


def stat(v):
    v = np.asarray(v, float)
    return dict(n=int(len(v)), mean=round(float(v.mean()), 3), p50=round(float(np.median(v)), 3), p90=round(float(np.percentile(v, 90)), 3)) if len(v) else dict(n=0)


def classify(d, rec, tw):
    """scene tags of each decision from the shadow records: speed, curvature, light, section, objects."""
    tags = []
    for j, t in enumerate(tw):
        e = rec.ego_at(t)
        tg = set()
        if e is None:
            tags.append(tg); continue
        x, y, yaw, v = e
        k = np.abs(d["ak"][j, :30, 1]).max() if d["valid"][j] else 0.0
        sec = rec.last("sec", t); sec = sec[2] if sec else 0
        tl = rec.last("tl", t); tl = tl[2] if tl else 0
        objs = rec.objects(t)
        c, s = math.cos(yaw), math.sin(yaw)
        ahead = [(o, c * (o[2] - x) + s * (o[3] - y), -s * (o[2] - x) + c * (o[3] - y)) for o in objs]
        ahead = [(o, lx, ly) for o, lx, ly in ahead if 0 < lx < 50 and abs(ly) < 6]
        veh = [a for a in ahead if a[0][1] == "vehicle"]
        if d["source"][j] == 1:
            tg.add("deviation")
        if d["valid"][j] and v > 6 and k < 0.004 and not ahead:
            tg.add("straight")
        if d["valid"][j] and v > 3 and k > 0.04:
            tg.add("curve")
        if tl in (2, 6, 7, 8) and d["valid"][j] and d["rollout"][j, -1, 3] < 0.5 and v > 2 and not d["flags"][j, :4].any():
            tg.add("red_stop")
        if d["flags"][j, 0] and any(a[0][1] == "pedestrian" for a in ahead + [(o, 0, 0) for o in objs if o[1] == "pedestrian"]):
            tg.add("ped_yield")
        if sec == 9 and veh:
            tg.add("roundabout_follow")
        if sec in (11, 12) and veh:
            tg.add("highway_npc")
        if any(a[0][1] not in ("vehicle", "pedestrian") and math.hypot(a[0][5], a[0][6]) < 0.3 for a in ahead) and sec in (4, 5, 6):
            tg.add("static_obstacle")
        tags.append(tg)
    return tags


def figure(path, d, j, rec, lane, title):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon
    t = d["t_wall"][j]
    x, y, yaw, v = rec.ego_at(t)
    fig, (ax, bx) = plt.subplots(1, 2, figsize=(14, 6.6), gridspec_kw=dict(width_ratios=[1.25, 1]))
    W = 45.0
    s0 = d["dbg"][j, 6]
    if np.isfinite(s0):
        LP = lane.ahead(s0 - 40.0, 140.0)
        ax.plot(LP[:, 0], LP[:, 1], color="0.6", lw=6, alpha=0.35, label="official lane (route.json)")
    p = rec.last("vp", t)
    if p is not None:
        ax.plot(p[3], p[4], color="tab:green", lw=1.6, label=f"plan (velocity_injected, {t - p[1]:.2f} s old)")
    m = (rec.ego_t > t - 6) & (rec.ego_t < t + 8)
    ax.plot(rec.ego[m, 0], rec.ego[m, 1], color="tab:blue", lw=1.4, label="driven (GT, -6..+8 s)")
    if d["valid"][j]:
        r = d["rollout"][j]
        ax.plot(r[:, 0], r[:, 1], color="tab:red", lw=2.2, label=f"label rollout 6.4 s (source {d['source'][j]})")
        ax.plot(r[9::10, 0], r[9::10, 1], "o", color="tab:red", ms=3)
    for o in rec.objects(t):
        oid, lab, ox, oy, oyaw, vx, vy, sx, sy = o
        if math.hypot(ox - x, oy - y) > W * 1.5:
            continue
        cc, ss = math.cos(oyaw), math.sin(oyaw)
        corners = [(ox + cc * a - ss * b, oy + ss * a + cc * b) for a, b in ((sx / 2, sy / 2), (-sx / 2, sy / 2), (-sx / 2, -sy / 2), (sx / 2, -sy / 2))]
        col = {"vehicle": "tab:purple", "pedestrian": "tab:orange"}.get(lab, "tab:brown")
        ax.add_patch(Polygon(corners, closed=True, fc=col, ec="k", alpha=0.6))
        if math.hypot(vx, vy) > 0.3:
            ax.arrow(ox, oy, vx, vy, color=col, width=0.08, head_width=0.6)
        ax.text(ox, oy, f"{lab[:3]} {math.hypot(vx, vy):.1f}", fontsize=6)
    ax.add_patch(Polygon([(x + math.cos(yaw) * a - math.sin(yaw) * b, y + math.sin(yaw) * a + math.cos(yaw) * b)
                          for a, b in ((3.85, 0.95), (-1.0, 0.95), (-1.0, -0.95), (3.85, -0.95))], closed=True, fc="tab:blue", ec="k"))
    tl = rec.last("tl", t)
    sec = rec.last("sec", t)
    ax.set_xlim(x - W, x + W); ax.set_ylim(y - W, y + W); ax.set_aspect("equal"); ax.grid(alpha=0.3)
    ax.legend(fontsize=7, loc="lower left")
    ax.set_title(f"light {TL_NAMES.get(tl[2], tl[2]) if tl else '-'}, section {sec[2] if sec else '-'}, v {v * 3.6:.1f} km/h", fontsize=9)
    tt = 0.1 * np.arange(1, 65)
    if d["valid"][j]:
        bx.plot(tt, d["rollout"][j, :, 3] * 3.6, color="tab:red", label="label speed km/h")
    tf = (rec.ego_t > t) & (rec.ego_t < t + 6.4 / max(d.get("rho", 0.84), 0.3))
    bx.plot(np.interp(rec.ego_t[tf] - t, [0, 1], [0, 1]) * 0 + (rec.ego_t[tf] - t), rec.ego[tf, 3] * 3.6, color="tab:blue", lw=1, label="driven speed km/h (wall s)")
    b2 = bx.twinx()
    if d["valid"][j]:
        b2.plot(tt, d["ak"][j, :, 1], color="tab:green", lw=1, label="label kappa 1/m")
        b2.plot(tt, d["ak"][j, :, 0] / 10, color="tab:gray", lw=1, ls="--", label="label a / 10 (m/s^2)")
    if "ak_actual" in d and d["ok_actual"][j]:
        b2.plot(tt, d["ak_actual"][j, :, 1], color="tab:olive", lw=1, ls=":", label="driven kappa")
    bx.set_xlabel("s"); bx.grid(alpha=0.3); bx.legend(fontsize=7, loc="upper left"); b2.legend(fontsize=7, loc="upper right")
    fl = [str(n) for n, f in zip(d["flag_cols"], d["flags"][j]) if f]
    ck = f", check {d['verdict'][j]} {d['check_reasons'][j]}" if "verdict" in d else ""
    md = f", mode {d['mode'][j]}" if "mode" in d else ""
    fig.suptitle(f"{title}  |  {d['reasons'][d['reason'][j]]}{md}, flags {fl}, d_lane {d['dbg'][j, 3]:.2f} m{ck}", fontsize=9)
    fig.tight_layout(); fig.savefig(path, dpi=110); plt.close(fig)


def timing(run):
    """late share and model_ms of the policy laps (stack /e2e_driver/debug), RTF per episode."""
    out = {}
    f = run / "results" / "dbg_w1.csv"
    if f.exists():
        dbg = [json.loads(l.split(",", 1)[1]) for l in f.read_text().splitlines()[1:] if "," in l]
        if dbg:
            late = np.array([bool(x.get("late")) for x in dbg]); ms = np.array([x.get("model_ms", np.nan) for x in dbg], float)
            out.update(steps=len(dbg), late_pct=round(100 * late.mean(), 1), model_ms=stat(ms[np.isfinite(ms)]))
    for kind in ("", "drive_"):
        for f in (run / "spool").glob(f"*_{kind}w1/episodes.jsonl"):
            out[f"episodes_{kind or 'policy'}"] = [{k: e.get(k) for k in ("episode", "scene", "end", "t_sim", "wall_s", "rho_end", "decisions")}
                                                   | dict(penalty_s=e.get("scorer", {}).get("penalty_s")) for e in map(json.loads, f.read_text().splitlines())]
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run")
    ap.add_argument("--route", default=str(HERE.parent / "assets" / "route.json"))
    a = ap.parse_args()
    run = Path(a.run).expanduser(); rep = run / "report"; rep.mkdir(exist_ok=True)
    lane = dl.Lane(a.route)
    summ = dict(timing=timing(run), coverage={}, errors={})
    for kind, name in (("", "policy"), ("drive_", "expert")):
        eps = episodes(run, kind)
        if not eps:
            continue
        D = [load(e) for e in eps]
        R = np.concatenate([d["reason"] for d in D]); S = np.concatenate([d["source"] for d in D]); Vd = np.concatenate([d["valid"] for d in D])
        reasons = D[0]["reasons"]
        summ.setdefault("checks", {})[name] = checks(D)
        summ["coverage"][name] = dict(decisions=int(len(R)), empty_pct=round(100 * (~Vd).mean(), 2),
                                      empty_reasons={str(reasons[i]): int(((R == i) & ~Vd).sum()) for i in np.unique(R[~Vd])},
                                      source1_pct=round(100 * (S == 1).mean(), 2),
                                      source1_reasons={str(reasons[i]): int(((R == i) & (S == 1)).sum()) for i in np.unique(R[S == 1])})
        if name == "expert":
            rows = []
            for e, d in zip(eps, D):
                if "fxy_actual" not in d:
                    continue
                m, err = errors(d)
                v0 = d["dbg"][m, 7]; fl = d["flags"][m]
                for i in range(int(m.sum())):
                    rows.append(dict(ep=e.name, v0=v0[i], source=int(d["source"][m][i]), yield_=bool(fl[i, 2]), aeb=bool(fl[i, 3]),
                                     **{k: float(v[i]) for k, v in err.items()}))
            if rows:
                import pandas as pd
                df = pd.DataFrame(rows); df.to_csv(rep / "errors.csv", index=False)
                groups = dict(all=df, moving=df[df.v0 > 0.5], stopped=df[df.v0 <= 0.5], yield_active=df[df.yield_], source1=df[df.source == 1])
                summ["errors"] = {g: {k: stat(x[k]) for k in ("ade", "fde", "lat", "lon", "lat_end", "lon_end", "ade_1s", "ade_3s", "a_rmse", "k_rmse")}
                                  for g, x in groups.items() if len(x)}
        if True:                                      # scene figures: the policy laps, then the expert's own laps
            rec_dir = run / "rec_w1"
            picked, pre = {}, "" if name == "policy" else "drive_"
            for e, d in zip(eps, D):
                rec = dl.Rec(dl.load_rec(rec_dir, d["t_wall"].min() - 8, d["t_wall"].max() + 10))
                tags = classify(d, rec, d["t_wall"])
                md = d.get("mode", np.array([""] * len(d["t_wall"])))
                sg = d.get("signal", np.zeros(len(d["t_wall"]), np.int8))
                nxt = np.r_[sg[1:], sg[-1:]]
                hold = [(sg[j] == 1 and md[j] == "stop" and (sg[j + 1:j + 30] == 3).any()) for j in range(len(sg))]   # picking only
                for j, tg in enumerate(tags):
                    tg |= {t for t, ok in (("red_hold_before_green", hold[j]), ("red_go_unstoppable", md[j] == "go_unstoppable"),
                                           ("red_go_past", md[j] == "go_past"),
                                           ("takeover", bool(d.get("intervened", np.zeros(len(tags), bool))[j]))) if ok}
                for tag in ("straight", "curve", "deviation", "red_stop", "ped_yield", "roundabout_follow", "highway_npc", "static_obstacle",
                            "red_hold_before_green", "red_go_unstoppable", "red_go_past", "takeover"):
                    js = [j for j, tg in enumerate(tags) if tag in tg]
                    cnt = summ.setdefault(f"scene_counts_{name}", {}); cnt[tag] = cnt.get(tag, 0) + len(js)
                    if js and tag not in picked:
                        j = js[len(js) // 2]
                        figure(rep / f"{pre}scene_{tag}.png", d, j, rec, lane, f"{name} {tag}: {e.parent.name}/{e.name} t_sim {d['t_sim'][j]:.1f}")
                        picked[tag] = f"{e.parent.name}/{e.name} j {j}"
            summ[f"figures_{name}"] = picked
    for f in ("check.txt", "cpu.txt"):
        p = run / "results" / f
        if p.exists():
            summ[f] = p.read_text().splitlines()
    json.dump(summ, open(rep / "summary.json", "w"), indent=1, default=str)
    print(json.dumps({k: v for k, v in summ.items() if k not in ("check.txt",)}, indent=1, default=str)[:6000])


if __name__ == "__main__":
    main()
