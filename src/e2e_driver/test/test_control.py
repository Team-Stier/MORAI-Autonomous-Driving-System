"""Controller unit checks (numpy only): table inverses, steering sign and scale, standstill hold, stale plan."""
import math, os, sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "src"))
from e2e_driver.control import Controller, Plan  # noqa: E402


def test_control():
    c = Controller()
    a, b = c.pedals(0.94, 4.0); assert abs(a - 0.25) < 1e-9 and b == 0           # low-speed table point
    a, b = c.pedals(0.48, 10.0); assert abs(a - 0.25) < 1e-9 and b == 0          # cruise table point
    a, b = c.pedals(0.0, 10.0); assert abs(a - 0.06) < 1e-9                      # hold speed
    a, b = c.pedals(-0.40, 10.0); assert a == 0 and abs(b) < 1e-9                # cruise coast: no pedal
    a, b = c.pedals(-6.1, 10.0); assert a == 0 and abs(b - 0.38) < 1e-9
    a, b = c.pedals(-30.0, 10.0); assert b == 1.0
    k = 0.05                                                                      # left turn, R = 20 m
    p = Plan(0.0, np.zeros(64), np.full(64, k), 8.0)
    acc, brk, st, info = c.command(p, 0.05, 8.0)
    assert st > 0 and abs(st - math.atan(3.0 * k / 0.77) / math.radians(40)) < 1e-12
    p = Plan(0.0, np.zeros(64), np.zeros(64), 0.0)
    acc, brk, st, info = c.command(p, 0.05, 0.0); assert info["hold"] and brk >= 0.5 and acc == 0
    acc, brk, st, info = c.command(p, 0.8, 0.0); assert brk == 1.0 and info["reason"] == "no fresh plan"
    p = Plan(0.0, np.full(64, 1.0), np.zeros(64), 0.0)
    acc, brk, st, info = c.command(p, 0.05, 0.0); assert not info["hold"] and acc > 0


if __name__ == "__main__":
    test_control(); print("control ok")
