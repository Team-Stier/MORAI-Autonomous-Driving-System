"""rl_reward on recorded drives: per-decision rewards add up to -(time + scorer penalties) + the shaping constant, each
penalty lands in the decision interval it falls in, and the repeated speed/lane penalties sit where the scorer counts them.

    SCORER=.../Alpamayo-Cosmos/tools/scorer uv run --python 3.12 --with-requirements $SCORER/requirements.txt \
        python test/test_rl_reward.py

Data (this Mac): the step-0 Ego tables of collected episodes (STEP0, default ~/.cache/step0_ego) and the 10/26 eval lap
live3 (EVAL_RUNS, default ~/morai-eval/runs). shortcut: fails without them; copy two npz files and one lap to run elsewhere.
"""
import gzip
import json
import os
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "tools"))
SCORER = os.environ.get("SCORER", str(HERE.parents[3] / "Alpamayo-Cosmos" / "tools" / "scorer"))
sys.path.insert(0, SCORER)
import rl_reward  # noqa: E402
from bag import episode_from_rows  # noqa: E402
from scorer import KMap, load_config, score  # noqa: E402

STEP0 = Path(os.environ.get("STEP0", "~/.cache/step0_ego")).expanduser()
RUNS = Path(os.environ.get("EVAL_RUNS", "~/morai-eval/runs")).expanduser()
CFG = load_config(Path(SCORER) / "thresholds.yaml")
CFG["map"]["checkpoints"] = str(HERE.parent / "tools" / "checkpoints_20261007.csv")
KM = KMap.from_files(CFG)
V_REF, FAIL_S = 80 / 3.6, 100.0
S9, S12 = KM.cp_s[9], KM.cp_s[12]
S_GOAL = S12 + 100.0


def npz_rows(name, v_scale_past_s=None):
    d = np.load(STEP0 / f"{name}.npz")
    v = d["v"].astype(float)
    if v_scale_past_s:                                    # a faster exit than the collection driver ever drove
        v = np.where(d["path_s"] > v_scale_past_s[0], v * v_scale_past_s[1], v)
    ns = (d["tau"] * 1e9).astype(np.int64)
    return dict(ego_status=[(int(n), (x, y, yaw, vv)) for n, x, y, yaw, vv in zip(ns, d["x"], d["y"], d["yaw"], v)],
                gps=[(int(n), bool(b)) for n, b in zip(ns, d["gps_blackout"])])


def window(rows, t_lo, t_hi):
    return {k: [(n, x) for n, x in r if t_lo * 1e9 <= n <= t_hi * 1e9] for k, r in rows.items()}


def episode(rows, slow=True):
    """the manager's end rule on recorded rows (route s from the scorer): first frame at s >= S_GOAL, or past CP12 below
    60 kph. -> (rows to score: up to 1 s past the end, end, t_end, t_play)"""
    labels, _ = score(episode_from_rows(rows, CFG), KM, CFG)
    t0 = rows["ego_status"][0][0] / 1e9
    s, v = labels.path_s.to_numpy(float), labels.speed_kph.to_numpy(float)
    t_play = float(labels.t[np.flatnonzero(np.nan_to_num(s) >= S9 - 0.5)[0]]) + t0     # the spawn at CP9
    past = np.maximum.accumulate(np.nan_to_num(s) >= S12)
    i = np.flatnonzero((labels.t + t0 > t_play) & ((s >= S_GOAL) | (past & (v < 60) & slow)))[0]
    end, t_end = ("goal" if s[i] >= S_GOAL else "slow"), float(labels.t[i]) + t0
    return window(rows, t_play, t_end + 1.0), end, t_end, t_play


def run(rows, end, t_end, t_play):
    labels, sm = score(episode_from_rows(rows, CFG), KM, CFG)
    t0 = rows["ego_status"][0][0] / 1e9
    clk = [(t + 1000.0, t) for t in np.arange(t0, t_end + 1.0, 0.05)]       # wall = sim + 1000 s
    tl = json.loads(json.dumps(rl_reward.timeline(labels, sm, CFG, t0, end, t_end, S_GOAL, clk)))   # as saved
    t_dec = np.arange(t_play, t_end, 0.1)
    return tl, sm, rl_reward.rewards(tl, t_dec, V_REF, FAIL_S), t_dec, t0


def check_sums(tl, sm, recs, t_dec, end):
    S = np.asarray(tl["s"])
    tot = {p: sum(r[p] for r in recs) for p in ("r", "r_time", "r_penalty", "r_shape", "r_terminal")}
    assert abs(tot["r_time"] + (tl["t_end"] - t_dec[0])) < 1e-6, tot
    in_window = sum(p for t, p, _ in tl["penalties"])
    assert abs(tot["r_penalty"] + in_window) < 1e-9, (tot, in_window)
    s0 = np.interp(t_dec[0], S[:, 0], S[:, 1])
    shape = (S_GOAL if recs[-1]["terminated"] else np.interp(tl["t_end"], S[:, 0], S[:, 1])) / V_REF - s0 / V_REF
    assert abs(tot["r_shape"] - shape) < 1e-6, (tot, shape)
    assert abs(tot["r"] - sum(tot[p] for p in ("r_time", "r_penalty", "r_shape", "r_terminal"))) < 1e-6
    t_next = np.r_[t_dec[1:], tl["t_end"]]
    for r, t, tn in zip(recs, t_dec, t_next):             # each penalty inside its decision's interval (t, t_next]
        for e in r["events"]:
            assert r["k"] == 0 or t - 1e-6 <= e["t_sim"] <= tn + 1e-6, (r["k"], t, e)
    assert [r["done"] for r in recs].count(True) == 1 and recs[-1]["end"] == end
    return tot


def test_cp9_to_exit_recorded():
    """run1020s-w1-020 drove CP9 (s 930.2) through the exemption and slowed below 60 kph after CP12: a 'slow' success."""
    rows, end, t_end, t_play = episode(npz_rows("run1020s-w1-020"))
    tl, sm, recs, t_dec, t0 = run(rows, end, t_end, t_play)
    assert end in ("slow", "goal"), end
    tot = check_sums(tl, sm, recs, t_dec, end)
    s_end = recs[-1]["s"]
    if end == "slow":
        assert recs[-1]["terminated"] and abs(recs[-1]["r_terminal"] + (S_GOAL - np.interp(t_end, *np.asarray(tl["s"])[:, :2].T)) / rl_reward.V60) < 1e-6
    # nothing outside the exemption was over the limit on this drive; any lane penalty is the scorer's
    assert sm["counts"].get("speed", 0) == 0 and abs(-tot["r_penalty"] - sum(
        e["penalty_s"] for e in sm["events"] if not e["after_finish"] and e["t_start"] + t0 <= t_end)) < 1e-6
    print("cp9->exit", end, round(t_end - t_play, 1), "s, s_end", s_end, {k: round(v, 2) for k, v in tot.items()})


def test_fast_exit_speed_penalties():
    """the same drive 1.5x faster past CP12 (to s_goal, read as a time limit): the scorer's 15 s + 15 per 3 s lands at
    the frames its count reaches."""
    rows, end, t_end, t_play = episode(npz_rows("run1020s-w1-020", (S12, 1.5)), slow=False)
    tl, sm, recs, t_dec, t0 = run(rows, "time_limit", t_end, t_play)
    sp = [e for e in sm["events"] if e["type"] == "speed" and e["t_start"] + t0 <= t_end]
    assert sp and max(e["t_end"] - e["t_start"] for e in sp) >= 3.0, sp
    for e in sp:
        n = round((e["t_end"] - e["t_start"]) * 10) + 1                  # frames over the limit
        want = [e["t_start"] + t0] + [e["t_start"] + t0 + 0.1 * (m * 30 - 1) for m in range(1, n // 30 + 1)]
        got = [t for t, p, k in tl["penalties"] if k == "speed" and e["t_start"] + t0 - 1e-6 <= t <= e["t_end"] + t0 + 1e-6]
        assert np.allclose(got, want, atol=1e-6) and len(got) * 15 == e["penalty_s"], (got, want, e)
    tot = check_sums(tl, sm, recs, t_dec, "time_limit")
    assert recs[-1]["truncated"] and not recs[-1]["terminated"] and tot["r_terminal"] == 0
    print("fast exit", [(e["t_start"], e["t_end"], e["penalty_s"]) for e in sp], round(tot["r_penalty"], 1))


def test_lane_repeats():
    """run1020i-w1-004: 75 s of lane contact = 15 x 5 s, one every 3 s of contact."""
    rows = npz_rows("run1020i-w1-004")
    t0, t1 = rows["ego_status"][0][0] / 1e9, rows["ego_status"][-1][0] / 1e9
    tl, sm, recs, t_dec, _ = run(rows, "time_limit", t1 - 1.0, t0)
    lane = [e for e in sm["events"] if e["type"] == "lane" and e["penalty_s"] > 0]
    assert lane, sm["counts"]
    for e in lane:
        got = [t for t, p, k in tl["penalties"] if k == "lane" and e["t_start"] + t0 - 1e-6 <= t <= e["t_end"] + t0 + 1e-6]
        assert len(got) * 5 == e["penalty_s"] and np.allclose(np.diff([e["t_start"] + t0 - 0.1] + got), 3.0), (got, e)
    check_sums(tl, sm, recs, t_dec, "time_limit")
    print("lane", [(e["t_start"], e["t_end"], e["penalty_s"]) for e in lane])


def test_eval_lap_collisions_signals():
    """the 10/26 eval lap live3 (gRPC rows from bc-250: collisions, lights, lane): rewards carry every scorer penalty
    once, and a failure end costs fail_s."""
    rows = {k: [(n, x) for n, x in r] for k, r in json.load(gzip.open(RUNS / "live3" / "lap000.rows.json.gz", "rt")).items()}
    t0, t1 = rows["ego_status"][0][0] / 1e9, rows["ego_status"][-1][0] / 1e9
    tl, sm, recs, t_dec, _ = run(rows, "off_road", t1 - 1.0, t0)
    kinds = {e["type"] for r in recs for e in r["events"]}
    assert {"collision", "signal_violation"} <= kinds, kinds
    tot = check_sums(tl, sm, recs, t_dec, "off_road")
    scored = sum(e["penalty_s"] for e in sm["events"] if not e["after_finish"] and not e.get("repeat")
                 and e["t_start"] + t0 <= t1 - 1.0)
    assert abs(-tot["r_penalty"] - scored) < 1e-6 and tot["r_terminal"] == -FAIL_S and recs[-1]["terminated"]
    print("live3", sm["counts"], round(tot["r_penalty"], 1), round(tot["r"], 1))


def test_rejoin_actor_decisions(tmp=None):
    """rl_reward.py TIMELINE DECISIONS: the actor's own decision times (wall clock, uneven, some outside the episode)
    get the same totals; their other fields are copied through."""
    import subprocess
    import tempfile
    rows, end, t_end, t_play = episode(npz_rows("run1020s-w1-020"))
    tl, sm, recs, t_dec, t0 = run(rows, end, t_end, t_play)
    tmp = Path(tempfile.mkdtemp())
    json.dump(dict(tl, episode="x/0", reward_args=dict(v_ref=V_REF, fail_s=FAIL_S)), open(tmp / "tl.json", "w"))
    rng = np.random.default_rng(0)
    walls = [t_play + 1000.0 - 5.0] + list(t_play + 1000.0 + np.cumsum(rng.uniform(0.08, 0.14, 2000)))
    with open(tmp / "dec.jsonl", "w") as f:
        f.writelines(json.dumps(dict(id=i, t_wall=w)) + "\n" for i, w in enumerate(walls))
    out = [json.loads(l) for l in subprocess.run([sys.executable, str(HERE.parent / "tools" / "rl_reward.py"), str(tmp / "tl.json"),
                                                  str(tmp / "dec.jsonl")], capture_output=True, text=True, check=True).stdout.splitlines()]
    assert out[0]["id"] == 1 and out[-1]["done"] and out[-1]["end"] == end and out[0]["episode"] == "x/0", out[0]
    assert abs(sum(r["r_time"] for r in out) + (t_end - out[0]["t_sim"])) < 2e-3          # t_sim is rounded to ms
    assert abs(sum(r["r_shape"] for r in out) - (S_GOAL / V_REF - out[0]["s"] / V_REF)) < 0.01
    print("rejoin", len(out), "decisions of", len(walls), round(sum(r["r"] for r in out), 2))


if __name__ == "__main__":
    for f in (test_cp9_to_exit_recorded, test_fast_exit_speed_penalties, test_lane_repeats, test_eval_lap_collisions_signals,
              test_rejoin_actor_decisions):
        f()
    print("ok")
