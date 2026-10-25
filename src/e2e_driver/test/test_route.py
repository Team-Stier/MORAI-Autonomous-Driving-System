"""Route.pursuit_kappa checks (numpy only): on a straight stretch it is ~0, and it steers back toward the route."""
import os, sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "src"))
from e2e_driver.route import Route  # noqa: E402


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


if __name__ == "__main__":
    test_pursuit(); print("ok")
