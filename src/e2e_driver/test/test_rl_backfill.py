"""rl_backfill offline = online: two fake PPO episodes of test_rl_episode (the highway run with a car ahead ending 'slow',
and the off-road run that returns to CP10 and pays 15 s) are made to look like DAgger's records (34-value critic_obs, no
return penalty in the timeline, zeroed rewards), plus a copy without objects.json.gz. The backfill must give back the
online 50-value critic_obs, rewards, events and done.json, skip the copy and leave the engine dumps behind.
critic_obs: the returns run's car comes at 1 mm, as objects.json.gz keeps it: offline must equal online exactly. The
highway run's car has full precision online: only the car fields may move, by the 1 mm storage (speeds over 0.3 s).

    SCORER=.../Alpamayo-Cosmos/tools/scorer uv run --python 3.12 --with grpcio --with protobuf \
        --with-requirements $SCORER/requirements.txt python test/test_rl_backfill.py
"""
import json
import os
import shutil
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import test_rl_episode as te  # noqa: E402  (puts tools/ on the path)
import rl_backfill  # noqa: E402
import rl_critic  # noqa: E402



def tol(f):
    """full-precision car online vs 1 mm offline: positions 1e-3 m, speeds (0.3 s differences) ~3.5e-3 m/s, ttc in
    proportion (gap / closing speed: 0.02 s at 35 s); the Ego's own fields exactly."""
    car = f.startswith(("own_", "left_", "right_", "near"))
    return (0.05 if f.endswith("ttc") else 0.01 if f.endswith(("rel_v", "dvx", "dvy")) else 1e-3) if car else 0.0


def returns_lat():
    """test_returns' path: 30 m right of the route from s 1300 until the return's jump back to the checkpoint."""
    seen = dict(s=0.0, back=False)

    def lat(s):
        seen["back"] |= s < seen["s"] - 50.0
        seen["s"] = s
        return -30.0 if s > 1300 and not seen["back"] else 0.0
    return lat


def dagger_like(d):
    """as a DAgger actor recorded it: 34 critic fields, rewards without the returns' 15 s."""
    steps = [json.loads(line) for line in open(os.path.join(d, "steps.jsonl"))]
    with open(os.path.join(d, "steps.jsonl"), "w") as f:
        for st in steps:
            f.write(json.dumps(dict(st, critic_obs=st["critic_obs"][:34], r=0.0, r_penalty=0.0, r_time=0.0, events=[])) + "\n")
    tl = json.load(open(os.path.join(d, "timeline.json")))
    tl["penalties"] = [p for p in tl["penalties"] if p[2] != "return"]
    json.dump(tl, open(os.path.join(d, "timeline.json"), "w"))


def test_backfill():
    runs = dict(hwy=te.episode(lambda s: 15.0 if s > 1721.7 or s < 1118.7 else 25.0))
    car = te.FakeSim.car
    te.FakeSim.car = lambda self: [(*o[:2], *(round(x, 3) for x in o[2:])) for o in car(self)]   # as objects.json.gz
    try:
        runs["ret"] = te.episode(lambda s: 25.0, lat=returns_lat(), returns=True)
    finally:
        te.FakeSim.car = car
    src, out = tempfile.mkdtemp(), tempfile.mkdtemp()
    for name, (mgr, *_rest) in runs.items():
        shutil.copytree(mgr.out, os.path.join(src, name))
        dagger_like(os.path.join(src, name, "ep00000"))
    shutil.copytree(os.path.join(src, "hwy"), os.path.join(src, "noobj"))
    os.remove(os.path.join(src, "noobj", "ep00000", "objects.json.gz"))
    summary = rl_backfill.backfill(src, out, runs["hwy"][0].a.scorer)
    print("summary", summary)
    assert {k: summary[k] for k in ("in", "written", "skip_no_objects")} == {"in": 3, "written": 2, "skip_no_objects": 1}, summary
    assert set(summary) == {"in", "written", "skip_no_objects", "check"}, summary
    for name, (mgr, ep, recs, tl, _) in runs.items():
        d = os.path.join(out, name, "ep00000")
        assert sorted(os.listdir(d)) == ["done.json", "steps.jsonl", "timeline.json"], os.listdir(d)   # no dumps
        new = [json.loads(line) for line in open(os.path.join(d, "steps.jsonl"))]
        assert len(new) == len(recs) and all("req" not in st and "out" not in st for st in new), (len(new), len(recs))
        a, b = np.array([st["critic_obs"] for st in new]), np.array([r["critic_obs"] for r in recs])
        assert a.shape == b.shape == (len(recs), len(rl_critic.FIELDS)), (a.shape, b.shape)
        dev = np.abs(a - b).max(0)
        print(name, "max |offline - online| per field > 0:", {f: round(float(x), 5) for f, x in zip(rl_critic.FIELDS, dev) if x > 0})
        bad = {f: float(x) for f, x in zip(rl_critic.FIELDS, dev) if x > (tol(f) if name == "hwy" else 0.0)}
        assert not bad and (name == "ret" or dev[[f.startswith("near") for f in rl_critic.FIELDS]].max() > 0), bad
        for key in ("k", "t_sim", "t_wall", "s", "v", "phi", "phi_next", "r_time", "r_penalty", "r_shape", "r_terminal", "r",
                    "events", "done", "terminated", "truncated", "end", "since_return_s"):
            assert all(st.get(key) == r.get(key) for st, r in zip(new, recs)), (name, key)
        assert json.load(open(os.path.join(d, "done.json"))) == json.load(open(os.path.join(mgr.out, "ep00000", "done.json")))
        assert json.load(open(os.path.join(d, "timeline.json")))["penalties"] == tl["penalties"], name
        assert abs(sum(st["r"] for st in new) - ep["r"]) < 0.01, (name, ep["r"])
    hit = [st["k"] for st in new if any(e["type"] == "return" for e in st["events"])]
    assert len(runs["ret"][1]["returns"]) == 1 and len(hit) == 1, hit       # the 15 s at the online decision
    print("ok")


if __name__ == "__main__":
    test_backfill()
