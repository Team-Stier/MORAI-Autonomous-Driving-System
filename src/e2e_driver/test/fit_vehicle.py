"""Evidence for control.py's constants from collected bags (Noetic chroot, rosbag): on the restored sim clock tau (step 0's
_ego table maps bag time to tau, so rates are per sim second), fit
  steering:     kappa = wz / v  vs  tan(delta) / L, delta = applied front_steer (competition_status, sign flipped to left +)
                -> g (curvature per tan(delta)/L); and front_steer vs command x 40 deg
  longitudinal: dv/dtau = Ka accel - Kb brake - c0 - c2 v^2 on the applied pedals (competition_status accel, brake), best lag
Usage: python3 fit_vehicle.py EGO_NPZ BAG [BAG ...]  (prints JSON)"""
import json, sys

import numpy as np
import rosbag

from offline_replay import Clock


def main():
    clk = Clock(sys.argv[1]); st, imu, cmd = [], [], []
    for p in sys.argv[2:]:
        b = rosbag.Bag(p)
        for topic, m, t in b.read_messages(topics=["/vehicle/competition_status", "/sensors/imu/data", "/morai_udp_bridge/control_cmd"]):
            if topic == "/vehicle/competition_status":
                st.append((m.header.stamp.to_nsec(), m.signed_velocity_mps, m.accel, m.brake, -m.steering_angle_rad))
            elif topic == "/sensors/imu/data":
                imu.append((m.header.stamp.to_nsec(), m.angular_velocity.z))
            else:
                cmd.append((t.to_nsec(), m.steering))
        b.close()
    st, imu, cmd = np.array(st, float), np.array(imu, float), np.array(cmd, float)
    g = np.arange(clk.tau_at(st[0, 0]) + 1, clk.tau_at(st[-1, 0]) - 1, 0.05)               # 20 Hz tau grid
    I = lambda a, c: np.interp(g, clk.tau_at(a[:, 0]), a[:, c])
    v, acc, brk, delta, wz, steer = I(st, 1), I(st, 2), I(st, 3), I(st, 4), I(imu, 1), I(cmd, 1)
    out = {}
    m = v > 3.0
    kappa, L = wz[m] / v[m], 3.0
    x = np.tan(delta[m]) / L
    out["steer_gain_g"] = round(float(np.dot(x, kappa) / np.dot(x, x)), 4)
    out["steer_fit_r2"] = round(float(1 - np.sum((kappa - out["steer_gain_g"] * x) ** 2) / np.sum((kappa - kappa.mean()) ** 2)), 4)
    lag = []
    for d in np.arange(0, 0.31, 0.05):                                                     # command -> applied wheel angle
        s = np.interp(g - d, g, steer); mm = np.abs(s) > 0.05
        lag.append((float(np.median(np.abs(delta[mm] - np.radians(40) * s[mm]))), round(float(d), 2)))
    out["steer_cmd_to_angle"] = dict(median_abs_err_rad=round(min(lag)[0], 4), best_lag_s=min(lag)[1],
                                     ratio_deg=round(float(np.degrees(np.median(delta[np.abs(steer) > 0.05] / steer[np.abs(steer) > 0.05]))), 2))
    a = np.gradient(np.convolve(v, np.ones(5) / 5, "same"), 0.05)
    best = None
    for d in np.arange(0, 0.61, 0.05):
        A = np.interp(g - d, g, acc); B = np.interp(g - d, g, brk)
        mm = (v > 1.0) & (np.abs(a) < 8)
        X = np.c_[A[mm], -B[mm], -np.ones(mm.sum()), -v[mm] ** 2]
        coef, res, *_ = np.linalg.lstsq(X, a[mm], rcond=None)
        r2 = 1 - np.sum((a[mm] - X @ coef) ** 2) / np.sum((a[mm] - a[mm].mean()) ** 2)
        if best is None or r2 > best[0]: best = (r2, d, coef)
    r2, d, (ka, kb, c0, c2) = best
    out["pedal_fit"] = dict(r2=round(float(r2), 4), lag_s=round(float(d), 2), accel_gain=round(float(ka), 3), brake_gain=round(float(kb), 3),
                            drag_c0=round(float(c0), 4), drag_c2=round(float(c2), 6), n=int(((v > 1.0) & (np.abs(a) < 8)).sum()))
    # pedal maps: median dv/dtau per (pedal, speed) bin, the other pedal 0; coast (both 0) per speed bin
    vb = [0, 3, 6, 9, 12, 15, 18]
    def table(p, other, edges):
        rows = {}
        for lo, hi in zip(edges[:-1], edges[1:]):
            r = []
            for v0, v1 in zip(vb[:-1], vb[1:]):
                mm = (p >= lo) & (p < hi) & (other < 0.01) & (v >= v0) & (v < v1) & (np.abs(a) < 12)
                r.append(round(float(np.median(a[mm])), 2) if mm.sum() >= 10 else None)
            rows["%.2f-%.2f" % (lo, hi)] = r
        return rows
    out["speed_bins"] = vb
    out["accel_map"] = table(acc, brk, [0.02, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.75])
    out["brake_map"] = table(brk, acc, [0.02, 0.1, 0.2, 0.3, 0.45, 0.6, 1.01])
    mm = (acc < 0.01) & (brk < 0.01)
    out["coast"] = [round(float(np.median(a[mm & (v >= v0) & (v < v1)])), 3) if (mm & (v >= v0) & (v < v1)).sum() >= 10 else None
                    for v0, v1 in zip(vb[:-1], vb[1:])]
    out["seconds"] = round(float(g[-1] - g[0]), 1)
    print(json.dumps(out))


if __name__ == "__main__":
    main()
