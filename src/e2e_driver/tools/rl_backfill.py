#!/usr/bin/env python3
"""DAgger actor episodes -> a PPO critic warm-start spool (Alpamayo-Cosmos train/rl/learner.py --warmstart OUT_SPOOL).

    tools/rl_backfill.py SRC_SPOOL OUT_SPOOL [--segments round,hwy] [--scorer DIR]      (SCORER env as default)

Every finished episode SRC/<actor>/epK/ (done.json) gives OUT/<actor>/epK/ steps.jsonl, timeline.json, done.json (last):
- critic_obs: the full rl_critic.FIELDS vector as the PPO episode manager computes it online: rl_critic.Critic driven with
  the recorded 20 Hz Ego rows (rows.json.gz: pose, speed, GPS-shadow flag), the Object Info frames (objects.json.gz) and
  the route progress of the manager's Track replayed from START_CP (back to the checkpoint at each return), taken at the
  recorded decision times with the recorded prev (a, kappa).
- rewards: rl_reward.rewards over timeline.json plus 15 s per return (rl_episode.finish since a042c2f), at the decision
  times of the engine requests' t_wall (the recorded t_sim, 1 ms, when the dump is gone). DAgger's "slow" end of a
  segment other than the 고주로 (END_CP != 12) does not exist for PPO, whose episode drives on to the goal: it becomes a
  cut ("slow_cut", truncated, no terminal reward: the learner bootstraps it).
The warm start reads only steps.jsonl and done.json: engine dumps (req/out keys dropped), rows and objects stay in SRC.
Skipped and counted: no done.json, not in <actor>/episodes.jsonl, other segment (--segments: tools/pod/segments.py names
or full), no objects.json.gz (before DAgger 1c9322f: no near-car fields), frame or decision counts that do not line up.
Offline equals online up to Object Info stored to 1 mm: car speeds (differences over 0.3 s) move by up to ~5e-3 m/s.
The summary's "check" lists recorded critic_obs fields that the recomputation moved by more than 1e-3.
"""
import argparse
import gzip
import json
import os
import sys
from collections import Counter
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE / "pod"))
import eval_manager as em  # noqa: E402
import rl_critic  # noqa: E402
import rl_episode  # noqa: E402
import rl_reward  # noqa: E402
import segments  # noqa: E402


def _cps(args):
    t = args.split()
    return int(t[t.index("--start-cp") + 1]), int(t[t.index("--end-cp") + 1])


SEGMENT = {_cps(segments.args_of(n)): n for n in [*segments.SEGMENTS, "full"]}   # (start_cp, end_cp) -> name
PREV = [rl_critic.FIELDS.index("prev_a"), rl_critic.FIELDS.index("prev_kappa")]


def episode(mgr, d, ep):
    """-> (steps, done, timeline, max |recomputed - recorded| per recorded critic_obs field) for episode dir d (its
    episodes.jsonl record ep), or a skip reason."""
    tl = json.load(open(d / "timeline.json"))
    with gzip.open(d / "rows.json.gz", "rt") as f:
        rows = json.load(f)
    with gzip.open(d / "objects.json.gz", "rt") as f:
        frames = json.load(f)["frames"]
    old = [json.loads(line) for line in open(d / "steps.jsonl")]
    if not len(rows["ego_status"]) == len(rows["gps"]) == len(frames):
        return "frame_count"
    rets = ep.get("returns", [])
    tl["penalties"] = sorted([p for p in tl["penalties"] if p[2] != "return"] + [[x["tau"], em.RETURN_S, "return"] for x in rets])
    if tl["end"] == "slow" and ep["end_cp"] != rl_episode.EXEMPT_END_CP:
        tl["end"] = "slow_cut"
    W, T = np.asarray(tl["clock"], float).T
    t_dec = [float(np.interp(rl_episode.head(d / st["req"])["t_wall"], W, T)) if "req" in st and (d / st["req"]).exists()
             else st["t_sim"] for st in old]
    recs = rl_reward.rewards(tl, t_dec, **tl["reward_args"])
    if len(recs) != len(old):
        return "decision_count"
    critic = rl_critic.Critic(mgr.kmap, mgr.cfg, mgr.lanes, float(mgr.track.s[mgr.cp_i[ep["end_cp"]]]), tl["s_goal"])
    tr, back = mgr.track, {x["tau"]: x["to"] for x in rets}
    tr.i = mgr.cp_i[ep["start_cp"]]
    for (ns, (x, y, yaw, v)), (_, shade), (tau3, _, objs) in zip(rows["ego_status"], rows["gps"], frames):
        s, _ = tr.update(x, y)
        critic.step(ns / 1e9, x, y, yaw, v, s, tr.i, objs, shade)
        if tau3 in back:                                  # the return's spawn: the route search starts at that checkpoint
            tr.i = mgr.cp_i[back[tau3]]
    obs = critic.at([r["t_sim"] for r in recs], [[st["critic_obs"][j] for j in PREV] for st in old])
    steps = [{**{k: v for k, v in st.items() if k not in ("req", "out")}, **r, "critic_obs": o} for st, r, o in zip(old, recs, obs)]
    rec = np.array([st["critic_obs"] for st in old])
    dev = np.abs(np.array(obs)[:, :rec.shape[1]] - rec).max(0)
    return steps, dict(terminated=recs[-1]["terminated"], truncated=recs[-1]["truncated"], end=tl["end"]), tl, dev


def backfill(src, out, scorer, segs=None):
    """-> summary {in, written, skip_<reason>..., check}."""
    src, out = Path(src).expanduser().resolve(), Path(out).expanduser().resolve()
    assert src != out, "OUT_SPOOL must differ from SRC_SPOOL (its steps.jsonl would be overwritten)"
    assert not segs or set(segs) <= set(SEGMENT.values()), f"segments: {', '.join(SEGMENT.values())}"
    mgr = rl_episode.Episodes(argparse.Namespace(scorer=scorer, out=str(out), name="", objinfo_port=0, sync=None,
                                                 limit_s=0.0, end_cp=12, past_m=0.0))   # map, lanes, Track, checkpoints
    n, check, logs = Counter(), {}, {}
    for d in sorted(p for p in src.glob("*/ep[0-9]*") if p.is_dir() and p.name[2:].isdigit()):
        n["in"] += 1
        if d.parent not in logs:
            f = d.parent / "episodes.jsonl"
            logs[d.parent] = {e["episode"]: e for e in map(json.loads, open(f))} if f.exists() else {}
        ep = logs[d.parent].get(int(d.name[2:]))
        why = ("unfinished" if not (d / "done.json").exists() else "no_episodes_record" if ep is None
               else "segment" if segs and SEGMENT.get((ep["start_cp"], ep["end_cp"])) not in segs
               else "no_objects" if not (d / "objects.json.gz").exists() else None)
        res = why or episode(mgr, d, ep)
        if isinstance(res, str):
            n[f"skip_{res}"] += 1
            continue
        steps, done, tl, dev = res
        od = out / d.parent.name / d.name
        od.mkdir(parents=True, exist_ok=True)
        json.dump(tl, open(od / "timeline.json", "w"))
        with open(od / "steps.jsonl", "w") as f:
            f.writelines(json.dumps(st) + "\n" for st in steps)
        rl_episode.write_json(od / "done.json", done)
        n["written"] += 1
        for f, x in zip(rl_critic.FIELDS, dev):
            if x > 1e-3:
                check[f] = round(max(check.get(f, 0.0), float(x)), 4)
    return dict(n, check=check)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("src")
    ap.add_argument("out")
    ap.add_argument("--segments", help="comma list of segments.py names (dyn, round, hwy, tunnel, full); default: all")
    ap.add_argument("--scorer", default=os.environ.get("SCORER", str(HERE.parents[3] / "Alpamayo-Cosmos" / "tools" / "scorer")))
    a = ap.parse_args()
    print(json.dumps(backfill(a.src, a.out, a.scorer, a.segments and a.segments.split(","))))


if __name__ == "__main__":
    main()
