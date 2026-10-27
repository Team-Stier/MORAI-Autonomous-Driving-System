#!/usr/bin/env python3
"""eval3.sh results -> OUT/summary.json and a table: per drive (OUT/wN) the lap (eval_manager laps.jsonl: end, lap and
total time, returns, scorer penalties), RTF (simulator s per wall s over the lap, lap000.clock.json), the stack's model
steps on tau (/e2e_driver/debug: rate, share of 0.1 s intervals, late = reply after t0 + apply_delay_s, model_ms), and
the whole pod's simulator hours per wall hour (all laps' tau / the wall span from the first start to the last end).
Restart time: timeline.txt (eval_pod.sh's pod create stamp to "drives started").

    eval3_summary.py OUT
"""
import json
import sys
from pathlib import Path

import numpy as np


def q(v):
    v = np.asarray(v, float)
    return dict(p50=round(float(np.median(v)), 1), p95=round(float(np.percentile(v, 95)), 1), max=round(float(v.max()), 1)) if len(v) else None


def drive(d):
    n = d.name
    lap = json.loads((d / "eval_map/laps.jsonl").read_text().splitlines()[-1])
    c = np.array(json.load(open(d / "eval_map/lap000.clock.json")))
    sc = lap.get("scorer", {})
    r = dict(world=n, seed=lap["seed"], end=lap["end"], lap_s=lap["lap_s"], total_s=lap["total_s"],
             penalty_s=sc.get("penalty_s"), returns=len(lap["returns"]), return_reasons=[x["reason"] for x in lap["returns"]],
             checkpoints=len(lap["checkpoints"]), counts=sc.get("counts"), competition_failed=sc.get("competition_failed"),
             wall_s=lap["wall_s"], tau_s=round(float(c[-1, 1] - c[0, 1]), 1), wall_span_s=round(float(c[-1, 0] - c[0, 0]), 1))
    r["rtf"] = round(r["tau_s"] / r["wall_span_s"], 3)
    v = lap.get("validation")                         # eval_manager's GT replay check (None: not recorded)
    r["replay_ok"], r["replay_failures"] = (v["ok"], v["failures"]) if v else (None, [])
    r["scene"] = dict(tunnel=lap.get("scene_meta", {}).get("tunnel"), weather=lap.get("weather"),
                      actors=[{k: a.get(k) for k in ("zone", "id", "cls", "ptype", "timing")} for a in lap.get("scene_meta", {}).get("actors", [])],
                      npcs=dict(roundabout=v and v.get("roundabout_npcs"), highway=v and v.get("highway_npcs"), sizes=v and v.get("vehicle_sizes")))
    dbg = [json.loads(l.split(",", 1)[1]) for l in (d / f"dbg_{n}.csv").read_text().splitlines()[1:] if "," in l]
    if dbg:
        t = np.array([x["t0"] for x in dbg]); dt = np.diff(t)
        late = [bool(x.get("late")) for x in dbg]
        r["steps"] = dict(n=len(t), hz_tau=round((len(t) - 1) / (t[-1] - t[0]), 2), share_0p1s=round(float(np.mean(dt <= 0.105)), 3),
                          late_share=round(float(np.mean(late)), 3) if "late" in dbg[-1] else None,
                          skipped=dbg[-1].get("skipped"), model_ms=q([x["server"]["model_ms"] for x in dbg if "model_ms" in x["server"]]),
                          server_ms=q([x["server_ms"] for x in dbg]))
    return r, (c[0, 0], c[-1, 0], r["tau_s"])


def main(out):
    out = Path(out)
    drives, spans = [], []
    for d in sorted(out.glob("w[0-9]*")):
        try:
            r, s = drive(d)
            drives.append(r); spans.append(s)
        except (OSError, ValueError, IndexError, KeyError) as e:
            drives.append(dict(world=d.name, error=repr(e)))
    tl = [l.split(" ", 2) for l in (out / "timeline.txt").read_text().splitlines()] if (out / "timeline.txt").exists() else []
    first = lambda key: next((float(a[0]) for a in tl if a[2].startswith(key)), None)
    tot = dict(drives=len(spans))
    if spans:
        w = max(s[1] for s in spans) - min(s[0] for s in spans)
        tot.update(sim_s=round(sum(s[2] for s in spans), 1), wall_s=round(w, 1), sim_h_per_wall_h=round(sum(s[2] for s in spans) / w, 3))
    t0, t1 = first("pod create"), first("drives started")
    if t0 and t1:
        tot["create_to_drives_started_s"] = round(t1 - t0, 1)
    summ = dict(total=tot, drives=drives, timeline=[a[1] + " " + a[2] for a in tl])
    json.dump(summ, open(out / "summary.json", "w"), indent=1, ensure_ascii=False)
    for r in drives:
        if "error" in r:
            print(r["world"], "ERROR", r["error"]); continue
        st = r.get("steps", {})
        print(f"{r['world']} seed {r['seed']}: {r['end']} lap {r['lap_s']:.1f} s + pen {r['penalty_s']} + returns {r['returns']} "
              f"= {r['total_s']:.1f} s | RTF {r['rtf']:.2f} | tau {st.get('hz_tau')} Hz, 0.1 s {st.get('share_0p1s')}, "
              f"late {st.get('late_share')} | model_ms {st.get('model_ms')} | {r['return_reasons']} | replay "
              f"{'ok' if r['replay_ok'] else r['replay_failures'] if r['replay_ok'] is False else '-'}")
    print("total", json.dumps(tot))


def selfcheck():
    """two fake drives: RTF, tau rate, late share and the pod total from known numbers"""
    import tempfile
    with tempfile.TemporaryDirectory() as tmp:
        o = Path(tmp)
        (o / "timeline.txt").write_text("100.0 00:00:00 pod create\n400.0 00:05:00 drives started\n")
        for k, (w0, rho) in enumerate(((1000.0, 0.5), (1010.0, 0.25)), 1):
            d = o / f"w{k}"; (d / "eval_map").mkdir(parents=True)
            json.dump([[w0 + i, i * rho] for i in range(601)], open(d / "eval_map/lap000.clock.json", "w"))
            (d / "eval_map/laps.jsonl").write_text(json.dumps(dict(seed=k, end="finish", lap_s=1.0, total_s=2.0, returns=[], checkpoints=[], wall_s=600)) + "\n")
            rows = ["%time,field"] + [f"0,{json.dumps(dict(t0=500 + 0.1 * i, late=i % 4 == 0, server_ms=50, server=dict(model_ms=90)))}" for i in range(101)]
            (d / f"dbg_w{k}.csv").write_text("\n".join(rows) + "\n")
        main(o)
        s = json.load(open(o / "summary.json"))
        assert [r["rtf"] for r in s["drives"]] == [0.5, 0.25], s["drives"]
        assert s["drives"][0]["steps"]["hz_tau"] == 10.0 and s["drives"][0]["steps"]["late_share"] == round(26 / 101, 3)
        assert s["total"]["sim_h_per_wall_h"] == round((300 + 150) / 610, 3) and s["total"]["create_to_drives_started_s"] == 300.0
    print("selfcheck ok")


if __name__ == "__main__":
    selfcheck() if sys.argv[1] == "--selfcheck" else main(sys.argv[1])
