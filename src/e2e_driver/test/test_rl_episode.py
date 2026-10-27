"""rl_episode offline: a fake simulator at RTF 0.7 is reset to CP9 and driven along the route. The test checks the reset
order, the ends (slow below 60 kph past CP12, off road, time limit) with their rewards, and the learner spool: a fake
engine writes this world's and another world's dumps at 10 Hz of simulator time, a fake car drives 40 m ahead.

    SCORER=.../Alpamayo-Cosmos/tools/scorer uv run --python 3.12 --with grpcio --with protobuf \
        --with-requirements $SCORER/requirements.txt python test/test_rl_episode.py
"""
import argparse
import json
import math
import os
import sys
import tempfile
import time
import types

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "tools"))
sys.path.insert(0, HERE)
import eval_manager as em  # noqa: E402
import rl_critic  # noqa: E402
import rl_episode  # noqa: E402
from test_eval_manager import BOX, Wall  # noqa: E402
sys.path.insert(0, os.path.join(HERE, "..", "src"))
from e2e_driver import wire  # noqa: E402

RTF = 0.7


class FakeSim:
    """drives the route at v(s) m/s with lateral offset lat(s) m; put() = the CP spawn (nearest path vertex)."""

    def __init__(self, path, wall, v, lat, order, dump):
        self.P, self.w, self.v, self.lat, self.order, self.dump = path, wall, v, lat, order, dump
        self.S = np.r_[0.0, np.cumsum(np.hypot(*np.diff(path, axis=0).T))]
        self.s, self.t, self.playing, self.t_dec, self.n = 0.0, None, False, None, 0
        self.car0, self.log = None, []

    def xy(self, s, lat=0.0):
        i = min(int(np.searchsorted(self.S, s)), len(self.P) - 2)
        u = (s - self.S[i]) / max(self.S[i + 1] - self.S[i], 1e-9)
        h = math.atan2(*(self.P[i + 1] - self.P[i])[::-1])
        return self.P[i] + u * (self.P[i + 1] - self.P[i]) + lat * np.array([-math.sin(h), math.cos(h)]), h

    def car(self):
        """an NPC on the route lane, from 150 m ahead of the Ego at play on at 20 m/s (Object Info record)."""
        if not self.playing:
            return []
        t = self.w.time()
        self.car0 = self.car0 or (t, self.s + 150.0)
        s = self.car0[1] + 20.0 * RTF * (t - self.car0[0])
        self.log.append((t, s, self.s))
        p, h = self.xy(s)
        return [(7, 1, float(p[0]), float(p[1]), 28.0, math.degrees(h), 4.5, 1.9, 1.5)]

    def engine(self, t):
        """the actor engine's dumps at 10 Hz of simulator time: world 1 (ours) and world 2 (another manager's)"""
        if not self.playing:
            return
        if self.t_dec is None or RTF * (t - self.t_dec) >= 0.1 - 1e-9:
            self.t_dec, self.n = t, self.n + 1
            for world in (1, 2):
                d = os.path.join(self.dump, f"world{world}")         # the actor engine's --dump layout
                os.makedirs(d, exist_ok=True)
                rid = 100000 * world + self.n
                open(os.path.join(d, f"req_{rid:06d}.bin"), "wb").write(wire.pack(dict(id=rid, t_wall=t, world=world, camera_jpegs=[[b"jpeg"]])))
                open(os.path.join(d, f"out_{rid:06d}.bin"), "wb").write(wire.pack(dict(id=rid, accel=np.full(64, 0.01 * self.n), kappa=np.zeros(64), expert=1)))

    def ego(self):
        t = self.w.time()
        dt, self.t = (0.0 if self.t is None else t - self.t), t
        v = self.v(self.s) if self.playing else 0.0
        self.s += RTF * v * dt
        self.engine(t)
        (x, y), h = self.xy(self.s, self.lat(self.s))
        return dict(x=float(x), y=float(y), yaw=h, v=v, hit=[], tl=None, color=-1)

    def put(self, x, y, z, yaw_deg):
        self.order.append("spawn")
        self.s = float(self.S[np.argmin(np.hypot(*(self.P - (x, y)).T))])


def episode(v, lat=lambda s: 0.0, limit_s=rl_episode.LIMIT_S):
    scorer = os.environ.get("SCORER", os.path.join(HERE, *[".."] * 4, "Alpamayo-Cosmos", "tools", "scorer"))
    out, dump = tempfile.mkdtemp(), tempfile.mkdtemp()
    json.dump(dict(version=3), open(os.path.join(out, "latest.json"), "w"))
    a = argparse.Namespace(name="t", start_cp=9, end_cp=12, past_m=100.0, seed=0, limit_s=limit_s, v_ref=80 / 3.6,
                           fail_s=100.0, weather=None, scorer=scorer, out=out, direct=True, loc_restart="true",
                           world=1, dump_dir=dump, objinfo_port=0, lora_latest=os.path.join(out, "latest.json"))
    wall, order = Wall(), []
    wall.t = time.time()                                  # dump mtimes are real: keep the fake wall clock near them

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
    mgr.sim = FakeSim(mgr.kmap.path, wall, v, lat, order, dump)
    mgr.objects = mgr.sim.car
    mgr.run_episode(0)
    d = os.path.join(out, "t", "ep00000")
    ep = json.loads(open(os.path.join(out, "t", "episodes.jsonl")).read())
    recs = [json.loads(l) for l in open(os.path.join(d, "steps.jsonl"))]
    tl = json.load(open(os.path.join(d, "timeline.json")))
    done = json.load(open(os.path.join(d, "done.json")))
    assert done == dict(terminated=ep["terminated"], truncated=ep["truncated"], end=ep["end"]), done
    assert json.load(open(os.path.join(out, "t", "inflight.json"))) == dict(version=3, episode="ep00000")
    for r in recs:                                        # learner fields, this world's dumps moved in
        assert len(r["critic_obs"]) == len(rl_critic.FIELDS) and all(math.isfinite(x) for x in r["critic_obs"]), r
        req = wire.unpack(open(os.path.join(d, r["req"]), "rb").read())
        assert req["world"] == 1 and abs(req["t_wall"] - r["t_wall"]) < 0.06, (req, r["t_wall"])
    left = os.listdir(os.path.join(dump, "world1"))
    assert not left and all(f"{n:05d}.out.bin" in os.listdir(d) for n in range(len(recs))), left   # ours all moved:
    assert os.listdir(os.path.join(d, "outside")), "decisions past the end"                       # 1 s past the end
    assert len(os.listdir(os.path.join(dump, "world2"))) >= 2 * len(recs), "another world's dumps were touched"
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
    # the first engine decision comes up to one decision interval (0.10-0.115 s on the 20 Hz poll) after play
    assert abs(ep["r_time"] + ep["t_sim"]) < 0.12 and abs(sum(r["r"] for r in recs) - ep["r"]) < 0.05
    assert abs(ep["r_shape"] - (mgr.s_goal - recs[0]["s"]) / (80 / 3.6)) < 0.05, ep   # Phi(terminal) = Phi(s_goal)
    t = np.array([r["t_sim"] for r in recs])
    assert np.allclose(np.diff(t), 0.1, atol=0.02) and abs(tl["t_end"] - t[0] - ep["t_sim"]) < 0.12 and ep["engine_decisions"]
    F = {f: i for i, f in enumerate(rl_critic.FIELDS)}
    r = recs[len(recs) // 2]                                # mid-highway: Ego 25 m/s, the car ahead at 20 m/s
    o = r["critic_obs"]
    _, s_car, s_ego = min(mgr.sim.log, key=lambda e: abs(e[0] - r["t_wall"]))
    gap = s_car - s_ego - rl_critic.EGO_CENTRE - (rl_critic.EGO_LEN + 4.5) / 2
    assert abs(o[F["v"]] - 25) < 1e-6 and abs(o[F["own_front_gap"]] - gap) < 0.3 and abs(o[F["own_front_rel_v"]] + 5) < 0.3, o
    assert abs(o[F["own_front_ttc"]] - gap / 5) < 0.3 and o[F["own_rear_gap"]] == rl_critic.GAP0 and o[F["in_exempt"]] == 1, o
    assert abs(o[F["lane_offset"]]) < 1.0 and abs(o[F["heading_err"]]) < 0.1 and 2.5 < o[F["lane_width"]] < 4.5, o
    assert recs[1]["critic_obs"][F["prev_a"]] > 0 and recs[0]["critic_obs"][F["prev_a"]] == 0, "previous (a, kappa)"
    print("slow", ep["t_sim"], round(expect, 1), {k: ep[k] for k in ("r", "r_time", "r_shape", "r_terminal")},
          dict(zip(rl_critic.FIELDS, o)))


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
