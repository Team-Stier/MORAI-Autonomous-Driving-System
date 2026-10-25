"""The model's terminal control signals -> morai_msgs/CtrlCmd values (accel, brake in [0, 1], steering in [-1, 1]).

The expert outputs 64 steps of (a_k, kappa_k), 0.1 s each (alpamayo_r1 UnicycleAccelCurvatureActionSpace): a_k is the
longitudinal acceleration (m/s^2) and kappa_k the path curvature (1/m, left +) held over [t0 + 0.1k, t0 + 0.1(k+1)),
with v_{k+1} = v_k + a_k dt from the v0 the action space estimates from the ego history. At time t the controller reads
the step under t + lookahead and maps it through the vehicle. Constants were measured on collected bags on the sim clock
(test/fit_vehicle.py, run1024b-w1-001, run1024a-w1-002/003, ~460 s of driving; README "제어 변환"):

  steering  delta = atan(L kappa / g),  steering = clip(delta / delta_max, -1, 1)
              L = 3.0 m           competition_status wheelbase_m
              delta_max = 40 deg  applied front_steer = 40 deg x command exactly, 0.1 s after the command (status angle
                                  is right +, command +1 turns left)
              g = 0.77            kappa = g tan(delta) / L, fits 0.782 / 0.753 / 0.757 (R^2 0.96-0.97); control
                                  morai_mpc.yaml steering_prediction.gain says 0.75
  pedals    dv/dt of the applied pedal (other pedal 0) is a table over pedal and speed (accel_map_low below 4.5 m/s,
            accel_map_cruise above 7.5 m/s, linear in between; brake_map at any speed); pedal 0 is the coast row.
            accel/brake = the table inverse at the measured speed. The response follows the applied pedal without lag,
            and the applied pedal follows the command by ~0.1 s, so both lookaheads are 0.1 s.
  standstill  measured and planned speed (next hold_horizon s) below stop_speed: brake >= hold_brake, so the car does
              not creep where the plan stays stopped
Optional speed feedback a += kp (v_plan(t) - v) (kp = 0: the model's own re-planning every 0.1 s closes the loop).
"""
import math

import numpy as np

DEFAULTS = dict(dt=0.1, wheelbase_m=3.0, max_steer_deg=40.0, steer_gain=0.77, steer_lookahead_s=0.1, accel_lookahead_s=0.1,
                # [pedal, m/s^2]: medians of the three fits (pedal 0 = coast; pedal 0.06 holds speed)
                accel_map_low=[[0.0, -0.55], [0.06, 0.0], [0.15, 0.51], [0.25, 0.94], [0.35, 1.40], [0.45, 2.18],
                               [0.55, 2.58], [0.67, 3.45], [1.0, 5.0]],
                accel_map_cruise=[[0.0, -0.40], [0.06, 0.0], [0.15, 0.27], [0.25, 0.48], [0.35, 0.87], [0.45, 1.08],
                                  [0.55, 1.37], [0.67, 1.75], [1.0, 2.6]],
                brake_map=[[0.0, -0.45], [0.06, -1.1], [0.15, -1.95], [0.25, -2.6], [0.38, -6.1], [0.52, -9.1], [1.0, -18.8]],
                low_speed_mps=4.5, cruise_speed_mps=7.5,
                max_accel_cmd=1.0, max_brake_cmd=1.0, speed_kp=0.0, stop_speed=0.3, hold_brake=0.5, hold_horizon_s=1.0,
                plan_timeout_s=0.5)


class Plan:
    def __init__(self, t0, accel, kappa, v0, dt=0.1):
        self.t0, self.a, self.k, self.v0, self.dt = float(t0), np.asarray(accel, float), np.asarray(kappa, float), float(v0), dt
        self.v = self.v0 + np.r_[0.0, np.cumsum(self.a * dt)]        # speed at each step start, (65,)

    def step(self, t):
        return int(np.clip(math.floor((t - self.t0) / self.dt), 0, len(self.a) - 1))

    def speed(self, t):
        k = self.step(t)
        return float(self.v[k] + self.a[k] * min(max(t - self.t0 - k * self.dt, 0.0), self.dt))


def _inverse(table, a):
    """pedal giving acceleration a on a monotone [pedal, a] table (clamped to its ends)."""
    p, y = np.asarray(table, float).T
    if y[-1] < y[0]:
        p, y = p[::-1], y[::-1]
    return float(np.interp(a, y, p))


class Controller:
    def __init__(self, **params):
        self.p = dict(DEFAULTS, **params)

    def accel_table(self, v):
        p = self.p
        lo, hi = np.asarray(p["accel_map_low"], float), np.asarray(p["accel_map_cruise"], float)
        w = float(np.clip((v - p["low_speed_mps"]) / (p["cruise_speed_mps"] - p["low_speed_mps"]), 0.0, 1.0))
        assert np.array_equal(lo[:, 0], hi[:, 0]), "accel maps need the same pedal points"
        return np.c_[lo[:, 0], (1 - w) * lo[:, 1] + w * hi[:, 1]]

    def pedals(self, a, v):
        """desired acceleration at speed v -> (accel, brake)."""
        acc_t = self.accel_table(abs(v))
        coast = float(acc_t[0, 1])
        if a >= coast:
            return min(_inverse(acc_t, a), self.p["max_accel_cmd"]), 0.0
        br = np.asarray(self.p["brake_map"], float).copy(); br[0, 1] = max(coast, br[1, 1] + 0.01)   # pedal 0 = this speed's coast
        return 0.0, min(_inverse(br, a), self.p["max_brake_cmd"])

    def command(self, plan, t, v):
        """-> (accel, brake, steering, info). plan None or too old -> full brake, wheel straight."""
        p = self.p
        if plan is None or t - plan.t0 > p["plan_timeout_s"]:
            return 0.0, 1.0, 0.0, {"reason": "no fresh plan"}
        kappa = plan.k[plan.step(t + p["steer_lookahead_s"])]
        delta = math.atan(p["wheelbase_m"] * kappa / p["steer_gain"])
        steering = float(np.clip(delta / math.radians(p["max_steer_deg"]), -1.0, 1.0))
        a = plan.a[plan.step(t + p["accel_lookahead_s"])] + p["speed_kp"] * (plan.speed(t) - v)
        accel, brake = self.pedals(a, v)
        ahead = [plan.speed(t + dt) for dt in np.arange(0.0, p["hold_horizon_s"] + 1e-9, plan.dt)]
        hold = abs(v) < p["stop_speed"] and max(ahead) < p["stop_speed"]
        if hold:
            accel, brake = 0.0, max(brake, p["hold_brake"])
        return float(accel), float(brake), steering, {"a": float(a), "kappa": float(kappa), "hold": hold}
