"""rl_episode offline: a fake simulator at RTF 0.7 is reset to CP9 and driven along the route. The test checks the reset
order, the ends (slow below 60 kph past CP12, off road, time limit) with their rewards, and the records.

    SCORER=.../Alpamayo-Cosmos/tools/scorer uv run --python 3.12 --with grpcio --with protobuf \
        --with-requirements $SCORER/requirements.txt python test/test_rl_episode.py
"""
import argparse
import json
import math
import os
import sys
import tempfile
import types

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "tools"))
sys.path.insert(0, HERE)
import eval_manager as em  # noqa: E402
import rl_episode  # noqa: E402
from test_eval_manager import BOX, Wall  # noqa: E402

RTF = 0.7


class FakeSim:
    """drives the route at v(s) m/s with lateral offset lat(s) m; put() = the CP spawn (nearest path vertex)."""

    def __init__(self, path, wall, v, lat, order):
        self.P, self.w, self.v, self.lat, self.order = path, wall, v, lat, order
        self.S = np.r_[0.0, np.cumsum(np.hypot(*np.diff(path, axis=0).T))]
        self.s, self.t, self.playing = 0.0, None, False

    def ego(self):
        t = self.w.time()
        dt, self.t = (0.0 if self.t is None else t - self.t), t
        v = self.v(self.s) if self.playing else 0.0
        self.s += RTF * v * dt
        i = min(int(np.searchsorted(self.S, self.s)), len(self.P) - 2)
        u = (self.s - self.S[i]) / max(self.S[i + 1] - self.S[i], 1e-9)
        h = math.atan2(*(self.P[i + 1] - self.P[i])[::-1])
        x, y = self.P[i] + u * (self.P[i + 1] - self.P[i]) + self.lat(self.s) * np.array([-math.sin(h), math.cos(h)])
        return dict(x=float(x), y=float(y), yaw=h, v=v, hit=[], tl=None, color=-1)

    def put(self, x, y, z, yaw_deg):
        self.order.append("spawn")
        self.s = float(self.S[np.argmin(np.hypot(*(self.P - (x, y)).T))])


def episode(v, lat=lambda s: 0.0, limit_s=rl_episode.LIMIT_S):
    scorer = os.environ.get("SCORER", os.path.join(HERE, *[".."] * 4, "Alpamayo-Cosmos", "tools", "scorer"))
    out = tempfile.mkdtemp()
    a = argparse.Namespace(name="t", start_cp=9, end_cp=12, past_m=100.0, seed=0, limit_s=limit_s, v_ref=80 / 3.6,
                           fail_s=100.0, weather=None, scorer=scorer, out=out, direct=True, loc_restart="true", no_view=True)
    wall, order = Wall(), []

    def msc(action, *args, **kw):
        order.append(action)
        if action == "play":
            mgr.sim.playing = True
        return "ok"
    em.time = rl_episode.time = wall
    em.msc = msc
    em.make_scene = lambda seed: ("ev_test", em.shade_test(BOX), {})
    em.send_weather = lambda host, w, h: order.append("weather")
    rl_episode.subprocess = types.SimpleNamespace(run=lambda *args, **kw: order.append("loc_restart"))
    mgr = rl_episode.Episodes(a)
    mgr.sim = FakeSim(mgr.kmap.path, wall, v, lat, order)
    mgr.run_episode(0)
    ep = json.loads(open(os.path.join(out, "t", "episodes.jsonl")).read())
    recs = [json.loads(l) for l in open(os.path.join(out, "t", "ep0000.rewards.jsonl"))]
    tl = json.load(open(os.path.join(out, "t", "ep0000.timeline.json")))
    return mgr, ep, recs, tl, order


def test_slow_exit():
    """25 m/s on the highway, 15 m/s from 20 m before CP12: ends 'slow' at CP12 with the rest at 60 kph charged."""
    mgr, ep, recs, tl, order = episode(lambda s: 15.0 if s > 1721.7 or s < 1118.7 else 25.0)
    assert order[:6] == ["pause", "load-scene", "spawn", "weather", "loc_restart", "play"], order
    S = mgr.track.s
    s9, s12 = S[mgr.cp_i[9]], S[mgr.cp_i[12]]
    expect = (S[mgr.cp_i[10]] - s9) / 15 + (1721.7 - S[mgr.cp_i[10]]) / 25 + (s12 - 3.0 - 1721.7) / 15   # CP radius 3 m
    assert ep["end"] == "slow" and ep["terminated"] and not ep["truncated"], ep
    assert [c[0] for c in ep["checkpoints"]] == [10, 11, 12] and len(order) - order.count("weather") > 0, ep["checkpoints"]
    assert abs(ep["t_sim"] - expect) < 0.3, (ep["t_sim"], expect)             # simulator clock, not wall (RTF 0.7)
    assert [w[0] for w in ep["weather"]] == [9, 10, 11, 12] and ep["scorer"]["penalty_s"] == 0, ep
    assert abs(ep["r_terminal"] + (mgr.s_goal - recs[-1]["s"] - recs[-1]["v"] * 0.1) / (60 / 3.6)) < 0.5, ep
    assert abs(ep["r_time"] + ep["t_sim"]) < 0.11 and abs(sum(r["r"] for r in recs) - ep["r"]) < 0.05
    assert abs(ep["r_shape"] - (mgr.s_goal - recs[0]["s"]) / (80 / 3.6)) < 0.05, ep   # Phi(terminal) = Phi(s_goal)
    t = np.array([r["t_sim"] for r in recs])
    assert np.allclose(np.diff(t), 0.1, atol=1e-3) and abs(tl["t_end"] - t[0] - ep["t_sim"]) < 0.11
    print("slow", ep["t_sim"], round(expect, 1), {k: ep[k] for k in ("r", "r_time", "r_shape", "r_terminal")})


def test_off_road():
    """leaves the road 30 m to the right from s 1300 (12 m left is the opposite carriageway, still drivable: that run
    ends missed_cp11): off road after the scorer's 1 s, a failure (-fail_s)."""
    mgr, ep, recs, tl, order = episode(lambda s: 25.0, lat=lambda s: -30.0 if s > 1300 else 0.0)
    assert ep["end"] == "off_road" and ep["terminated"] and ep["r_terminal"] == -100.0, ep
    assert 1300 < recs[-1]["s"] < 1340, recs[-1]
    print("off_road", ep["t_sim"], recs[-1]["s"], ep["r"])


def test_time_limit():
    """5 m/s never reaches CP12 within 150 s of simulator time: truncated, no terminal reward."""
    mgr, ep, recs, tl, order = episode(lambda s: 5.0)
    assert ep["end"] == "time_limit" and ep["truncated"] and not ep["terminated"] and ep["r_terminal"] == 0, ep
    assert abs(ep["t_sim"] - 150.0) < 0.2 and abs(ep["r_shape"] - (recs[-1]["s"] + 0.5 - recs[0]["s"]) / (80 / 3.6)) < 0.1, ep
    print("time_limit", ep["t_sim"], recs[-1]["s"], ep["r"])


if __name__ == "__main__":
    for f in (test_slow_exit, test_off_road, test_time_limit):
        f()
    print("ok")
