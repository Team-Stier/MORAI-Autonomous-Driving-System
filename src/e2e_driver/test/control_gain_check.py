"""control.py steering gain: a scalar or a [[speed, g], ...] table (python3 test/control_gain_check.py)."""
import math, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from e2e_driver.control import Controller


class Plan:
    t0, dt, k, a = 0.0, 0.1, [0.02] * 64, [0.0] * 64
    def step(self, t): return min(int(t / self.dt), 63)
    def speed(self, t): return 9.0


def steer(gain, v):
    return Controller(steer_gain=gain, mode="map").command(Plan(), 0.0, v)[2]


tab = [[6.5, 0.77], [8.0, 0.67], [11.5, 0.67], [13.0, 0.77]]
assert abs(steer(0.77, 9.0) - steer(tab, 3.0)) < 1e-9              # below the table: scalar 0.77
assert steer(tab, 9.0) > steer(0.77, 9.0)                           # 7-12 m/s: more wheel for the same kappa
assert abs(steer(tab, 9.0) - math.atan(3.0 * 0.02 / 0.67) / math.radians(40)) < 1e-9
print("ok")
