#!/usr/bin/env python3
"""lap_report.py DRIVE_DIR K: one lap of an eval3 drive (DRIVE_DIR = OUT/wN, K from 0) -> a one-line summary (stdout) and
DRIVE_DIR/lap_KK.json (KK = K + 1): end, lap and total time, scorer penalty, returns, late share and model_ms p50/p95
over the lap's model steps (/e2e_driver/debug rows whose t0 lies in the lap: the lap's wall span from lapK.clock.json,
mapped to the stack's tau through /sim_clock/state in clk_wN.csv), seed, weather, scene (tunnel, dynamic objects),
NPCs and the replay check.      lap_report.py DIR K | lap_report.py --selfcheck
"""
import csv
import json
import os
import sys
from collections import Counter
from pathlib import Path

import numpy as np


def report(d, k):
    d = Path(d)
    lap = next(x for x in map(json.loads, open(d / "eval_map/laps.jsonl")) if x["lap"] == k)
    c = np.array(json.load(open(d / f"eval_map/lap{k:03d}.clock.json")))
    rows = list(csv.reader(open(d / f"clk_{d.name}.csv")))
    i0, i1 = rows[0].index("field.data0"), rows[0].index("field.data1")    # [wall, tau, rho, n]
    clk = np.array([(float(r[i0]), float(r[i1])) for r in rows[1:] if len(r) > i1])
    t0, t1 = np.interp([c[0, 0], c[-1, 0]], clk[:, 0], clk[:, 1])
    dbg = [json.loads(l.split(",", 1)[1]) for l in open(d / f"dbg_{d.name}.csv").read().splitlines()[1:] if "," in l]
    dbg = [x for x in dbg if t0 <= x["t0"] <= t1]
    ms = [x["server"]["model_ms"] for x in dbg if "model_ms" in x.get("server", {})]
    sc, v, meta = lap.get("scorer", {}), lap.get("validation"), lap.get("scene_meta", {})
    dyn = Counter(a.get("cls") for a in meta.get("actors", []) if a.get("zone", "").endswith("_dynamic"))
    r = dict(lap=k + 1, seed=lap["seed"], scene=lap["scene"], end=lap["end"], lap_s=lap["lap_s"], penalty_s=sc.get("penalty_s"),
             returns=len(lap["returns"]), total_s=lap["total_s"], steps=len(dbg),
             late=round(float(np.mean([bool(x.get("late")) for x in dbg])), 3) if dbg else None,
             model_ms=[round(float(np.percentile(ms, q)), 1) for q in (50, 95)] if ms else None,
             weather=sorted({f"{w[1]}{w[2]}" for w in lap.get("weather", [])}), tunnel=meta.get("tunnel"), dynamic=dict(dyn),
             counts=sc.get("counts"), replay_ok=v["ok"] if v else None, replay_failures=v["failures"] if v else [],
             npcs=dict(roundabout=v.get("roundabout_npcs"), highway=v.get("highway_npcs"), sizes=len(v.get("vehicle_sizes", []))) if v else None)
    json.dump(r, open(d / f"lap_{k + 1:02d}.json", "w"), ensure_ascii=False, indent=1)
    f = lambda x, n=0: "-" if x is None else f"{x:.{n}f}"
    line = (f"{d.parent.name} 바퀴 {r['lap']} (seed {r['seed']}): {r['end']} {f(r['lap_s'])} s + 감점 {f(r['penalty_s'])} s + 복귀 "
            f"{r['returns']}×15 s = {f(r['total_s'])} s | late {f(100 * r['late'] if r['late'] is not None else None, 1)} % | "
            f"model {'/'.join(map(str, r['model_ms'])) if r['model_ms'] else '-'} ms (p50/p95) | 날씨 {','.join(r['weather'])} | "
            f"터널 {r['tunnel']} | 동적 {dict(dyn)} | NPC {r['npcs']} | 재생 "
            + ("확인됨" if r["replay_ok"] else f"실패 {r['replay_failures']}" if r["replay_ok"] is False else "기록 없음"))
    ev, tl = os.environ.get("EVENT_T0"), d.parent / "timeline.txt"
    first = [float(l.split()[0]) for l in (tl.read_text().splitlines() if tl.exists() else []) if l.split(" ", 2)[2:3] == ["first steps"]]
    if k == 0 and ev and first:                      # deliver_watch.sh: the delivery event's epoch
        s = first[0] - float(ev); r["event_to_first_step_s"] = round(s)
        line += f" | 납품 이벤트→첫 주행 step {int(s // 60)}분 {int(s % 60)}초"
    return line, r


def selfcheck():
    """a fake drive: lap 0 over wall 1000-1100 (stack tau 500-560 at rho 0.6), steps in and out of it."""
    import tempfile
    with tempfile.TemporaryDirectory() as tmp:
        d = Path(tmp) / "run" / "w1"; (d / "eval_map").mkdir(parents=True)
        lap = dict(lap=0, seed=28000, scene="ev28000_000", end="finish", lap_s=480.2, total_s=510.2, returns=[{}], weather=[[0, "SUNNY", 11, 0, "grpc"]],
                   scorer=dict(penalty_s=15.0, counts={}), scene_meta=dict(tunnel="both", actors=[dict(zone="ra_dynamic", cls="person")]),
                   validation=dict(ok=True, failures=[], roundabout_npcs=3, highway_npcs=5, vehicle_sizes=[[4.6, 1.9, 1.5]]))
        (d / "eval_map/laps.jsonl").write_text(json.dumps(lap) + "\n")
        json.dump([[1000 + i, 0.6 * i] for i in range(101)], open(d / "eval_map/lap000.clock.json", "w"))
        (d / "clk_w1.csv").write_text("%time,field.layout.dim,field.layout.data_offset,field.data0,field.data1,field.data2,field.data3\n"
                                      + "".join(f"0,[],0,{1000 + i},{500 + 0.6 * i},0.6,{i}\n" for i in range(101)))
        rows = [f"0,{json.dumps(dict(t0=490 + 0.1 * i, late=i % 5 == 0, server=dict(model_ms=90 + i % 10)))}" for i in range(800)]
        (d / "dbg_w1.csv").write_text("%time,field.data\n" + "\n".join(rows) + "\n")
        line, r = report(d, 0)
        assert r["steps"] == 601 and abs(r["late"] - 0.2) < 0.01 and r["model_ms"][0] == 94.0, r     # t0 500..560
        assert (d / "lap_01.json").exists() and "재생 확인됨" in line, line
    print("selfcheck ok")


if __name__ == "__main__":
    if sys.argv[1] == "--selfcheck":
        selfcheck()
    else:
        print(report(sys.argv[1], int(sys.argv[2]))[0])
