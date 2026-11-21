"""eval_manager lap logic offline: a fake simulator at RTF 0.7 drives the route at 8 m/s, stands still from s 600 m (stuck:
return to the last checkpoint passed, +15 s) and finishes; tools/scorer judges the lap. test_lap: --reload-each-lap;
test_continuous: main() with 2 laps in one pool scene (stub pool_scene, scenario_variants.layout and world_director).
tools/eval.sh's dependencies:

    SCORER=.../Alpamayo-Cosmos/tools/scorer uv run --python 3.12 --with grpcio --with protobuf \
        --with-requirements $SCORER/requirements.txt python test/test_eval_manager.py
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
import eval_manager as em  # noqa: E402

RTF, V = 0.7, 8.0
BOX = {"pos": {"x": -23.668, "y": -519.24}, "rot": {"yaw": "-58.844"}, "size": {"x": 117.959, "y": 7.419}}   # clcol
SCORER = os.environ.get("SCORER", os.path.join(HERE, *[".."] * 4, "Alpamayo-Cosmos", "tools", "scorer"))


class Wall:
    def __init__(self):
        self.t = 1000.0

    def time(self):
        return self.t

    def sleep(self, d):
        self.t += max(d, 0.0)

    def perf_counter(self):
        return self.t


class FakeSim:
    def __init__(self, path, wall):
        self.P, self.w = path, wall
        self.S = np.r_[0.0, np.cumsum(np.hypot(*np.diff(path, axis=0).T))]
        self.s, self.t, self.stuck, self.puts = 0.0, None, 0.0, []

    def ego(self):
        t = self.w.time()
        dt, self.t = (0.0 if self.t is None else t - self.t), t
        v = V
        if self.s >= 600.0 and self.stuck is not None:
            self.stuck, v = self.stuck + RTF * dt, 0.0
        self.s = (self.s + RTF * v * dt) % self.S[-1]                         # on into the next lap
        i = min(int(np.searchsorted(self.S, self.s)), len(self.P) - 2)
        u = (self.s - self.S[i]) / max(self.S[i + 1] - self.S[i], 1e-9)
        x, y = self.P[i] + u * (self.P[i + 1] - self.P[i])
        return dict(x=float(x), y=float(y), yaw=math.atan2(*(self.P[i + 1] - self.P[i])[::-1]), v=v, hit=[], tl=None, color=-1)

    def put(self, x, y, z, yaw_deg):
        self.puts.append((self.stuck, x, y))
        self.s, self.stuck = float(self.S[np.argmin(np.hypot(*(self.P - (x, y)).T))]), None


def test_lap():
    out = tempfile.mkdtemp()
    a = argparse.Namespace(name="t", laps=1, seed=0, weather=None, scorer=SCORER, out=out, port=0, no_view=True, direct=True,
                           objinfo_port=0, sync=None, max_s=0.0, density="alternate")
    wall, sent, dens = Wall(), [], []
    em.time, em.msc = wall, lambda *args, **kw: "ok"
    em.make_scene = lambda seed, density=None: (dens.append(density), ("ev_test", em.shade_test(BOX), {}))[1]
    em.send_weather = lambda host, w, h: sent.append((w, h))
    mgr = em.Manager(a)
    mgr.sim = sim = FakeSim(mgr.kmap.path, wall)
    mgr.run_lap(0)
    lap = json.loads(open(os.path.join(out, "t", "laps.jsonl")).read())
    S = sim.S
    cp_s = [float(S[i]) for i in mgr.cp_i]
    j = max(k for k in range(15) if cp_s[k] < 600.0)
    assert lap["end"] == "finish", lap["end"]
    assert [(r["to"], r["reason"]) for r in lap["returns"]] == [(j, "stuck")], lap["returns"]
    assert abs(sim.puts[0][0] - em.STUCK_S) < 1.0, sim.puts                     # returned after STUCK_S of sim time
    expect = S[-1] / V + em.STUCK_S + (600.0 - cp_s[j]) / V
    assert abs(lap["lap_s"] - expect) < 0.02 * expect, (lap["lap_s"], expect)   # on the simulator clock
    assert abs(lap["rho_end"] - RTF) < 0.02, lap["rho_end"]
    assert [c[0] for c in lap["checkpoints"]] == list(range(1, 15)) and len(sent) == 15, (lap["checkpoints"], sent)
    sc = lap["scorer"]
    assert sc["finished"] and set(sc["checkpoints_reached"]) == set(range(15)), sc
    assert lap["total_s"] == round(lap["lap_s"] + sc["penalty_s"] + 15.0, 2)
    assert "director" not in lap and lap["density"] == "low" and dens == ["low"], (lap["density"], dens)   # lap 0: low
    shade = em.shade_test(BOX)
    inside = [s for s, p in zip(S, mgr.kmap.path) if shade(*p)]
    assert 1875 < min(inside) < 1885 and 1973 < max(inside) < 1983, (min(inside), max(inside))
    print("reload ok", lap["lap_s"], round(expect, 1), sc["penalty_s"], sc["counts"])
    return mgr.kmap.path


def test_continuous(path):
    out, wall, log, dens = tempfile.mkdtemp(), Wall(), [], []
    sim = FakeSim(path, wall)
    put = sim.put
    sim.put = lambda *p: (log.append(("put", round(p[0], 3), round(p[1], 3))), put(*p))

    class Director:
        def __init__(self, sim_, route, pool, rng):
            assert sim_ is sim and pool == {"parking": []}
            self.n = 0

        def new_lap(self, meta, density):
            log.append(("new_lap", meta["r"]))
            dens.append(density)
            return [dict(event="ring_lap", r=meta["r"])]

        def step(self, tau, ego):
            assert set(ego) == {"x", "y", "yaw", "v", "s"}, ego
            self.n += 1
            return [dict(t=round(tau, 2), event="first step")] if self.n == 1 else []

        def close(self):
            log.append(("close",))

    sys.modules["world_director"] = types.SimpleNamespace(Director=Director)
    sys.modules["scenario_variants"] = types.SimpleNamespace(
        Route=lambda p: p, layout=lambda route, rng, pool: dict(gen="test", tunnel="dynamic", actors=[], r=rng.random()))
    em.time, em.Sim = wall, lambda host: sim
    em.msc = lambda action, *args, **kw: (log.append((action,) + args[:1]), "ok")[1]
    em.pool_scene = lambda seed: (f"ev{seed}_pool", em.shade_test(BOX), {"parking": []})
    em.make_scene = None                                   # never called in continuous mode
    em.deploy = em.serve = lambda *args: None
    em.send_weather = lambda host, w, h: None
    sys.argv = ["eval_manager.py", "c", "--laps", "2", "--scorer", SCORER, "--out", out, "--no-view", "--direct"]
    em.main()
    laps = [json.loads(x) for x in open(os.path.join(out, "c", "laps.jsonl"))]
    r = [x[1] for x in log if x[0] == "new_lap"]
    # one load, no pause before it, the Ego put back at START after lap 0's layout is in place, close before the pause
    assert log == [("load-scene", "ev0_pool/62"), ("new_lap", r[0]), ("play",), log[3], ("new_lap", r[1]),
                   ("put", em.START[0], em.START[1]), ("close",), ("pause",)], log
    assert log[3][0] == "put" and len(r) == 2 and r[0] != r[1], log                # lap 0's stuck return, two layouts
    assert [x["scene_meta"]["r"] for x in laps] == r and [x["seed"] for x in laps] == [0, 1], laps
    assert dens == ["low", "high"] and [x["density"] for x in laps] == dens, (dens, [x.get("density") for x in laps])
    assert [x["scene"] for x in laps] == ["ev0_pool"] * 2 and [x["end"] for x in laps] == ["finish"] * 2, laps
    assert len(laps[0]["returns"]) == 1 and laps[1]["returns"] == [], [x["returns"] for x in laps]
    assert all(x["director"][0]["event"] == "ring_lap" for x in laps), [x["director"][:1] for x in laps]   # new_lap's events kept
    assert laps[0]["director"][1]["event"] == "first step" and len(laps[1]["director"]) == 1, laps[1]["director"]
    expect = sim.S[-1] / V
    assert abs(laps[1]["lap_s"] - expect) < 0.02 * expect, (laps[1]["lap_s"], expect)
    c0, c1 = (json.load(open(os.path.join(out, "c", f"lap{k:03d}.clock.json"))) for k in (0, 1))
    assert c1[0][1] >= c0[-1][1] and c1[0][0] >= c0[-1][0], (c0[-1], c1[0])       # a clock per lap, tau goes on
    assert all(x["scorer"].get("finished") and x["scorer"].get("start_ok") for x in laps), [x["scorer"] for x in laps]
    print("continuous ok", [x["lap_s"] for x in laps], [x["total_s"] for x in laps], laps[1]["scorer"]["counts"])


def test_validate_director():
    """world_director movers (s_cross) are judged once the Ego reached s_cross - 60 m, however far it passes them."""
    ego = [(0.1 * i, float(i), -50.0, float(i)) for i in range(501)]
    objs = [(0.1 * i, [[9, 2, 100.0, -48.0, 1, 1, 1]] + ([[7, 0, 300.0 + 0.01 * (i - 240), 0.0, 1, 1, 1]] if i >= 240 else []))
            for i in range(501)]
    meta = dict(actors=[dict(zone="ra_dynamic", id=1, ped=True, move=4.0, active=5.0, s_cross=300.0, xy=[300.0, 0.0]),
                        dict(zone="rb_dynamic", id=2, ped=True, move=4.0, active=5.0, s_cross=800.0, xy=[800.0, 0.0]),
                        dict(zone="static", id=3, xy=[100.0, -48.0])])
    v = em.validate(meta, ego, objs, (1e9, 1e9 + 1), (1e9, 1e9 + 1))
    assert [d["status"] for d in v["dynamic"]] == ["moved", "not_reached"] and v["static"][0]["status"] == "present", v
    assert v["ok"], v
    print("validate ok")


if __name__ == "__main__":
    test_validate_director()
    test_continuous(test_lap())
