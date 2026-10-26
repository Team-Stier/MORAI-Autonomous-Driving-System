"""Route.pursuit_kappa checks (numpy only): on a straight stretch it is ~0, and it steers back toward the route."""
import os, sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "src"))
from e2e_driver.route import PoseGuard, Route  # noqa: E402


def test_pursuit():
    r = Route(os.path.join(HERE, "..", "assets", "route.json"))
    d = np.abs(np.diff(np.unwrap(r.heading)))
    i = int(np.argmin(np.convolve(d, np.ones(40), "valid"))) + 5                  # straightest 20 m
    x, y, h = r.path[i, 0], r.path[i, 1], r.heading[i]
    assert abs(r.pursuit_kappa(r.s[i], x, y, h, 8.0)) < 0.01
    left = (x - 2.0 * np.sin(h), y + 2.0 * np.cos(h))                           # 2 m left of the route
    assert r.pursuit_kappa(r.s[i], *left, h, 8.0) < -0.02                         # steer right
    right = (x + 2.0 * np.sin(h), y - 2.0 * np.cos(h))
    assert r.pursuit_kappa(r.s[i], *right, h, 8.0) > 0.02


def test_guard():
    g, v = PoseGuard(), 7.0
    track = [(0.02 * k, 0.14 * k, 0.0) for k in range(100)]                   # 7 m/s along x, 50 Hz
    assert all(g.ok(t, x, y, 0.0, v) for t, x, y in track[:50])
    assert not g.ok(1.0, 150.0, 80.0, 0.0, v)                                  # one swing of 170 m: dropped
    assert g.ok(1.02, 0.14 * 51, 0.0, 0.0, v)                                  # back on the track
    assert g.ok(1.03, 0.14 * 51 + 11.0, 0.0, 0.0, v)                           # an 11 m GPS-return correction passes
    assert g.ok(1.04, 0.14 * 51, 0.0, 0.0, v)
    assert not g.ok(1.05, float("nan"), 0.0, 0.0, v) and not g.ok(1.06, 1.9e17, 0.0, 0.0, v)
    jumped = [g.ok(1.1 + 0.02 * k, 500.0 + 0.14 * k, 0.0, 0.0, v) for k in range(60)]   # teleport: a new steady track
    assert not jumped[0] and not any(jumped[:49]) and all(jumped[51:]), jumped   # taken over after 1 s
    alt = [g.ok(3.0 + 0.02 * k, (508.4 + 0.14 * k) if k % 2 else 300.0, 0.0, 0.0, v) for k in range(100)]
    assert all(alt[1::2]) and not any(alt[0::2])                               # interleaved bad track never wins


if __name__ == "__main__":
    test_pursuit(); test_guard(); print("ok")
