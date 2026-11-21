"""popout_step0.py stage SPOOL NAME S0: the popout_box.sh episodes of run NAME (SPOOL/NAME{ra,rb}BB/epK, bags in
SPOOL/../bags) staged for the R2 step-0 chain (Alpamayo-Cosmos tools/step0, coc_labeler.data) on this box:
~/.claude-morai-mac/labels-cache/mac-win-1/<EP>.labels.parquet (rl_episode's scorer labels on tau, bag_time_ns on the host
clock), env-w1.log (weather), episode-scenes.log + manifests/ (scene), and S0/_popout/<EP>/ (labels, Object Info frames,
episode record) with S0/_popout/ok_NAME.txt: episodes with labels and a closed bag. EP = <run>-w1-KKK as the bag."""
import glob
import gzip
import math
import json
import os
import shutil
import sys
from pathlib import Path

spool, name, s0 = sys.argv[2:5]
assert sys.argv[1] == "stage"
mac = Path.home() / ".claude-morai-mac"
lab = mac / "labels-cache" / "mac-win-1"
for p in (lab, mac / "manifests", Path(s0) / "_popout"):
    p.mkdir(parents=True, exist_ok=True)
bags = Path(spool).parent / "bags"
ok, env, skipped = [], [], []
for run in sorted(glob.glob(f"{spool}/{name}*")):   # NAME: a box run (all its batches) or one batch
    run = Path(run)
    if not (run / "episodes.jsonl").exists():
        continue
    recs = {r["episode"]: r for r in map(json.loads, open(run / "episodes.jsonl"))} if (run / "episodes.jsonl").exists() else {}
    for d in sorted(run.glob("ep[0-9]*")):
        k = int(d.name[2:])
        ep = f"{run.name}-w1-{k:03d}"
        bag = bags / run.name / f"{ep}.bag"
        if not (d / "labels.parquet").exists() or not bag.exists() or k not in recs:
            skipped.append((ep, (d / "labels.parquet").exists(), bag.exists(), k in recs))
            continue
        r = recs[k]
        shutil.copy(d / "labels.parquet", lab / f"{ep}.labels.parquet")
        env += open(d / "env.log").read().splitlines()
        with open(mac / "episode-scenes.log", "a") as f:
            f.write(f"{ep} {r['scene']}/62\n")
        man = mac / "manifests" / f"{r['scene'].split('_')[0]}-manifest.json"
        json.dump({r["scene"]: r["scene_meta"]}, open(man, "w"))
        out = Path(s0) / "_popout" / ep
        out.mkdir(exist_ok=True)
        for f in ("labels.parquet", "objects.json.gz", "timeline.json", "env.log"):
            if (d / f).exists():
                shutil.copy(d / f, out / f)
        movers = [a for a in r["scene_meta"]["actors"] if a.get("ptype") == "popout"]
        fr = json.load(gzip.open(d / "objects.json.gz", "rt"))["frames"] if (d / "objects.json.gz").exists() else []
        found = [any(math.hypot(o[2] - a["xy"][0], o[3] - a["xy"][1]) < 1.5 for f in fr[:40] for o in f[2]) for a in movers]
        print(ep, "mover in Object Info:", [(a["zone"][:2], a["start"], a["id"], ok_) for a, ok_ in zip(movers, found)])
        json.dump(dict(r, run=run.name, bag=bag.name, bag_bytes=bag.stat().st_size, mover_in_objinfo=found), open(out / "episode.json", "w"), ensure_ascii=False)
        ok.append(ep)
with open(mac / "env-w1.log", "a") as f:
    f.write("".join(line + "\n" for line in sorted(env, key=lambda x: float(x.split()[0]))))
open(Path(s0) / "_popout" / f"ok_{name}.txt", "w").write("".join(e + "\n" for e in ok))
print(f"staged {len(ok)} episodes, skipped {len(skipped)}: {skipped[:10]}")
