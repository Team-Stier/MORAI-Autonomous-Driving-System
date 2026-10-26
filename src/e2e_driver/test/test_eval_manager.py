"""eval_manager lap logic offline: a fake simulator at RTF 0.7 drives the route at 8 m/s, stands still from s 600 m (stuck:
return to the last checkpoint passed, +15 s) and finishes; tools/scorer judges the lap. tools/eval.sh's dependencies:

    SCORER=.../Alpamayo-Cosmos/tools/scorer uv run --python 3.12 --with grpcio --with protobuf \
        --with-requirements $SCORER/requirements.txt python test/test_eval_manager.py
"""
import argparse
import json
import math
import os
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "tools"))
import eval_manager as em  # noqa: E402

RTF, V = 0.7, 8.0
BOX = {"pos": {"x": -23.668, "y": -519.24}, "rot": {"yaw": "-58.844"}, "size": {"x": 117.959, "y": 7.419}}   # clcol


class Wall:
    def __init__(self):
        self.t = 1000.0

    def time(self):
        return self.t

    def sleep(self, d):
        self.t += max(d, 0.0)


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
    scorer = os.environ.get("SCORER", os.path.join(HERE, *[".."] * 4, "Alpamayo-Cosmos", "tools", "scorer"))
    out = tempfile.mkdtemp()
    a = argparse.Namespace(name="t", laps=1, seed=0, weather=None, scorer=scorer, out=out, port=0, no_view=True)
    wall, sent = Wall(), []
    em.time, em.msc = wall, lambda *args, **kw: "ok"
    em.make_scene = lambda seed: ("ev_test", em.shade_test(BOX), {})
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
    assert abs(sim.puts[0][0] - em.STUCK_S) < 1.0, sim.puts                     # returned after 60 s of sim time
    expect = S[-1] / V + em.STUCK_S + (600.0 - cp_s[j]) / V
    assert abs(lap["lap_s"] - expect) < 0.02 * expect, (lap["lap_s"], expect)   # on the simulator clock
    assert abs(lap["rho_end"] - RTF) < 0.02, lap["rho_end"]
    assert [c[0] for c in lap["checkpoints"]] == list(range(1, 15)) and len(sent) == 15, (lap["checkpoints"], sent)
    sc = lap["scorer"]
    assert sc["finished"] and set(sc["checkpoints_reached"]) == set(range(15)), sc
    assert lap["total_s"] == round(lap["lap_s"] + sc["penalty_s"] + 15.0, 2)
    shade = em.shade_test(BOX)
    inside = [s for s, p in zip(S, mgr.kmap.path) if shade(*p)]
    assert 1875 < min(inside) < 1885 and 1973 < max(inside) < 1983, (min(inside), max(inside))
    print("ok", lap["lap_s"], round(expect, 1), sc["penalty_s"], sc["counts"])


if __name__ == "__main__":
    test_lap()
