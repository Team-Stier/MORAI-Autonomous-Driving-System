"""Summary of one tools/two_pc.sh drive (files in ~/morai-eval/data): model step rate and latency (dbg_NAME.csv,
/e2e_driver/debug), simulator stalls, scene reloads and RTF (gtraw_NAME.txt, raw Ego status), localization error and
re-sync time after each reload (loc_NAME.csv against the raw Ego status), plus the probes' live_NAME.json rates.
Usage: python3 drive_summary.py NAME"""
import json, os, sys

import numpy as np

D = os.path.expanduser("~/morai-eval/data")


def q(v):
    v = np.asarray(v, float)
    return dict(median=round(float(np.median(v)), 1), p95=round(float(np.percentile(v, 95)), 1), max=round(float(v.max()), 1)) if len(v) else None


def main(name):
    out = {}
    d = [json.loads(l.split(",", 1)[1]) for l in open(f"{D}/dbg_{name}.csv").read().splitlines()[1:] if "," in l]
    t = np.array([x["t0"] for x in d]); dt = np.diff(t); gap = dt > 0.3
    out["steps"] = dict(n=len(t), mean_hz=round((len(t) - 1) / (t[-1] - t[0]), 2), share_0p1s=round(float(np.mean(dt <= 0.105)), 3),
                        gaps_over_0p3s=int(gap.sum()), gap_s=round(float(dt[gap].sum()), 1),
                        hz_without_gaps=round(float((~gap).sum() / dt[~gap].sum()), 2))
    out["latency_ms"] = q([x["latency_ms"] for x in d]); out["server_ms"] = q([x["server_ms"] for x in d])
    out["preprocess_ms"] = q([x["server"].get("preprocess_ms", 0) for x in d])
    m = [x["server"]["model_ms"] for x in d if "model_ms" in x["server"]]
    if m: out["model_ms"] = q(m)

    rows = [l.split() for l in open(f"{D}/gtraw_{name}.txt")]
    rows = [a for i, a in enumerate(rows) if i == 0 or a[1] != rows[i - 1][1]]   # bc-250 sends each Ego status twice
    g = np.array([[float(a[0]), float(a[2]) / 3.6, float(a[3]), float(a[4])] for a in rows])
    gt, step = np.diff(g[:, 0]), np.hypot(*np.diff(g[:, 2:], axis=0).T)
    jump = step > 5.0                                                            # a reload teleport (7 m/s x 0.1 s = 0.7 m)
    st = gt[(gt > 0.5) & ~jump]                                                  # the reload's own gap is not a stall
    out["sim_stalls"] = dict(over_0p5s=len(st), total_s=round(float(st.sum()), 1), max_s=round(float(st.max()), 2) if len(st) else 0.0,
                             per_min=round(60 * len(st) / (g[-1, 0] - g[0, 0]), 2))
    resets = g[1:][jump, 0]
    out["scene_reloads"] = len(resets)
    mv = (g[1:, 1] > 1.0) & ~jump
    out["rtf"] = round(float(step[mv].sum() / np.sum(g[1:, 1][mv] * gt[mv])), 3) if mv.any() else None
    out["distance_m"] = round(float(step[~jump].sum()), 1)

    if os.path.exists(f"{D}/loc_{name}.csv"):
        lc = [l.split(",") for l in open(f"{D}/loc_{name}.csv").read().splitlines()[1:]]
        lt = np.array([int(a[0]) * 1e-9 for a in lc]); lxy = np.array([[float(a[1]), float(a[2])] for a in lc])
        err = np.hypot(lxy[:, 0] - np.interp(lt, g[:, 0], g[:, 2]), lxy[:, 1] - np.interp(lt, g[:, 0], g[:, 3]))
        k = np.clip(np.searchsorted(g[:, 0], lt), 1, len(g) - 1)
        fresh = np.minimum(np.abs(g[k, 0] - lt), np.abs(g[k - 1, 0] - lt)) < 0.2     # GT within 0.2 s, not across a gap
        err, lt = err[fresh], lt[fresh]
        settled = np.ones(len(lt), bool)
        for r in resets:                                                             # 5 s after a reload: re-sync, reported apart
            settled &= ~((lt >= r) & (lt < r + 5.0))
        out["loc_err_m"] = q(err[settled & (lt > g[0, 0])])
        sync = []
        for r in resets:
            after = (lt >= r) & (err < 3.0)
            sync.append(round(float(lt[after][0] - r), 1) if after.any() else None)
        out["resync_s_after_reload"] = sync
    out["route_s_spawn_vs_3s_after_reload"] = [round(d[0]["s"], 1)] + [round(float(np.interp(r + 3.0, t, [x["s"] for x in d])), 1) for r in resets]
    if os.path.exists(f"{D}/sec_{name}.csv"):                                  # section flips: localization pose jumps
        sec = np.array([int(l.split(",")[1]) for l in open(f"{D}/sec_{name}.csv").read().splitlines()[1:] if "," in l])
        out["section_changes"] = int(np.count_nonzero(np.diff(sec))) if len(sec) else 0
    p = f"{D}/live_{name}.json"
    if os.path.exists(p):
        live = json.load(open(p)); out["rates_hz"] = live.get("rates_hz"); out["nav_equal"] = live.get("nav_equal")
    print(json.dumps(out, indent=1))
    json.dump(out, open(f"{D}/summary_{name}.json", "w"), indent=1)


if __name__ == "__main__":
    main(sys.argv[1])
