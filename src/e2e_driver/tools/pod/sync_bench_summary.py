#!/usr/bin/env python3
"""sync_bench.sh results -> OUT/bench.json and a table, one row per combination (nN_tT_M): simulator seconds per wall
second (per world and total, from steps.jsonl after the first WARM_S of stepping), USD per simulator hour at DPH_USD,
the wall time of one decision (0.1 s of simulation: its steps' tick = render + physics, the stack's part and its waits:
sensors after the tick, plans = the reply awaited in the step (overlap), reply = awaited before the tick (serial)),
model_ms (dbg), GPU util and memory (gpu.csv), CPU cores (cpu.txt cgroup total).

    sync_bench_summary.py OUT        (env DPH_USD 1.19, WARM_S 20)
    sync_bench_summary.py --selfcheck
"""
import json
import os
import re
import sys
from pathlib import Path

import numpy as np

DPH, WARM = float(os.environ.get("DPH_USD", 1.19)), float(os.environ.get("WARM_S", 20))


def med(v):
    return round(float(np.median(v)), 1) if len(v) else None


def combo(c):
    n, tick, mode = re.match(r"n(\d+)_t(\d+)_(\w)", c.name).groups()
    worlds, parts = [], {k: [] for k in ("tick", "stack", "ego", "sensors", "plans", "request", "reply", "tx", "decision")}
    timeouts = 0
    for f in sorted(c.glob("w*/steps.jsonl")):
        s = [json.loads(l) for l in f.read_text().splitlines() if l.strip()]
        if len(s) < 10:
            continue
        w0 = s[0]["wall"] + WARM
        s = [x for x in s if x["wall"] >= w0]
        if len(s) < 10:
            continue
        dt = int(tick) / 1000.0
        span = s[-1]["wall"] - s[0]["wall"]
        worlds.append(dict(world=f.parent.name, steps=len(s), rtf=round((len(s) - 1) * dt / span, 3)))
        m = round(0.1 / dt)
        for x in s:
            a = x.get("ack") or {}
            parts["tick"].append(x["tick_ms"]); parts["stack"].append(x["stack_ms"]); parts["ego"].append(x["ego_ms"])
            parts["tx"].append(x["tx_ms"])
            for k in ("sensors", "plans", "request", "reply"):
                parts[k].append(a.get(k + "_ms", 0.0))
            timeouts += bool(x.get("ack_timeout") or x.get("tx_timeout") or a.get("timeout"))
        # one decision = m consecutive steps
        walls = [x["wall"] for x in s]
        parts["decision"] += list(np.diff(walls[::m]) * 1e3)
    model = []
    for f in c.glob("dbg_w*.csv"):
        for l in f.read_text().splitlines()[1:]:
            if "," in l:
                try:
                    model.append(json.loads(l.split(",", 1)[1])["server"]["model_ms"])
                except (ValueError, KeyError):
                    pass
    gpu = [l.split(", ") for l in (c / "gpu.csv").read_text().splitlines() if l.count(",") >= 3] if (c / "gpu.csv").exists() else []
    util = [float(g[1]) for g in gpu if g[1].strip().replace(".", "").isdigit()]
    mem = [float(g[2]) for g in gpu if g[2].strip().replace(".", "").isdigit()]
    cpu = re.search(r"cgroup total\s+([\d.]+)", (c / "cpu.txt").read_text()) if (c / "cpu.txt").exists() else None
    tot = round(sum(w["rtf"] for w in worlds), 3)
    return dict(combo=c.name, n=int(n), step_ms=int(tick), mode={"o": "overlap", "s": "serial"}.get(mode, mode),
                worlds_ok=len(worlds), rtf_world=[w["rtf"] for w in worlds], sim_s_per_s=tot,
                usd_per_sim_h=round(DPH / tot, 2) if tot else None,
                decision_ms=med(parts["decision"]), **{k + "_ms": med(v) for k, v in parts.items() if k != "decision"},
                model_ms=med(model), gpu_util=round(float(np.mean(util)), 1) if util else None,
                vram_gb=round(max(mem) / 1024, 1) if mem else None, cpu_cores=float(cpu.group(1)) if cpu else None,
                timeouts=timeouts)


def main(out):
    out = Path(out)
    rows = [combo(c) for c in sorted(out.glob("n*_t*_*"), key=lambda c: (int(c.name.split("_")[0][1:]), c.name))]
    json.dump(rows, open(out / "bench.json", "w"), indent=1)
    print(f"{'combo':14} {'ok':>3} {'sim-s/s':>7} {'$/sim-h':>7} {'dec ms':>6} {'tick':>5} {'stack':>5} {'sens':>5} "
          f"{'plans':>5} {'reply':>5} {'model':>5} {'gpu%':>5} {'vram':>5} {'cpu':>5} {'tmo':>3}  per-world RTF")
    for r in rows:
        f = lambda k: "-" if r[k] is None else str(r[k])
        print(f"{r['combo']:14} {r['worlds_ok']:>3} {f('sim_s_per_s'):>7} {f('usd_per_sim_h'):>7} {f('decision_ms'):>6} "
              f"{f('tick_ms'):>5} {f('stack_ms'):>5} {f('sensors_ms'):>5} {f('plans_ms'):>5} {f('reply_ms'):>5} "
              f"{f('model_ms'):>5} {f('gpu_util'):>5} {f('vram_gb'):>5} {f('cpu_cores'):>5} {r['timeouts']:>3}  {r['rtf_world']}")


def selfcheck():
    """two worlds stepping 100 ms of simulation every 250 ms of wall: RTF 0.4 each, 0.8 total, $1.19 / 0.8 per sim-h."""
    import tempfile
    with tempfile.TemporaryDirectory() as tmp:
        c = Path(tmp) / "n2_t100_o"
        for w in (1, 2):
            (c / f"w{w}").mkdir(parents=True)
            (c / f"w{w}/steps.jsonl").write_text("".join(json.dumps(dict(
                k=i, wall=1000 + 0.25 * i, tick_ms=120, stack_ms=110, ego_ms=5, tx_ms=7,
                ack=dict(sensors_ms=10, plans_ms=80, request_ms=15, reply_ms=0))) + "\n" for i in range(400)))
        main(tmp)
        r = json.load(open(Path(tmp) / "bench.json"))[0]
        assert r["rtf_world"] == [0.4, 0.4] and r["sim_s_per_s"] == 0.8 and r["usd_per_sim_h"] == round(DPH / 0.8, 2), r
        assert r["decision_ms"] == 250.0 and r["plans_ms"] == 80.0, r
    print("selfcheck ok")


if __name__ == "__main__":
    selfcheck() if sys.argv[1] == "--selfcheck" else main(sys.argv[1])
