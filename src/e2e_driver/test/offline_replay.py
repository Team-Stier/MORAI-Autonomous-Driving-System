"""Offline parity test, step 1 (Noetic chroot: python 3.8, rosbag, cv2 4.2): replay one collected episode through the live
node's own code (vlp16_bev.scan, e2e_driver.inputs / route) and dump the model request at chosen training rows.

The bags carry wall-clock stamps and the sim ran at 0.71-0.95x, while training used the restored sim clock tau. So the
replay maps every stamp to tau with the episode's _ego table, exactly as step 0 did (Clock below = step0_worker.Clock),
and stores tau as float32 like cams_index / lidar_index. Two modes:
  gt      the training signals: LiDAR motion from _ego v and yaw_rate at packet times (step 0), ego history from the
          _ego v / yaw grid, route s = _ego path_s. Must reproduce the training sample exactly (tests the code).
  deploy  the inference signals: competition_status speed + yaw rate (--r-source grid: the node's GridYawRate on
          competition_status yaw; imu: IMU wz) for LiDAR, competition_status speed + yaw for
          the history, /collect_only/localization_ekf (the stack's EKF) + odometer for route s. Differences are the
          price of the deployable signals.
Usage: python3 offline_replay.py --mode gt|deploy --ego _ego/<ep>.npz --rows 100,120,... --out DIR BAG [BAG ...]
"""
import argparse, json, os, sys

import numpy as np
import rosbag

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [os.path.join(HERE, "..", "src"), os.path.join(HERE, "..", "..", "vlp16_bev", "src")]
from e2e_driver import wire                                         # noqa: E402
from e2e_driver.inputs import Inputs                                # noqa: E402
from e2e_driver.route import Odometer, Route, RouteTracker          # noqa: E402
from vlp16_bev.scan import BevBuilder, GridYawRate, SignalTrack     # noqa: E402

CAMS = {"/image/front/compressed_10hz": "front", "/image/left/compressed_10hz": "left",
        "/image/right/compressed_10hz": "right", "/image/camera_7/compressed_10hz": "camera_7"}
LIDAR, STATUS, IMU, EKF = "/velodyne_packets", "/vehicle/competition_status", "/sensors/imu/data", "/collect_only/localization_ekf"
f32 = lambda x: float(np.float32(x))


class Clock:
    """tools/step0/step0_worker.py Clock: bag time (ns) -> tau (linear extrapolation at the ends), GT v / yaw rate."""
    def __init__(self, f):
        z = np.load(f); self.ns = z["bag_ns"].astype(np.float64); self.tau = z["tau"]
        self.v, self.r = z["v"].astype(np.float64), z["yaw_rate"].astype(np.float64)
        k = max(2, len(self.ns) // 20)
        self.slope0 = (self.tau[k] - self.tau[0]) / max(self.ns[k] - self.ns[0], 1)
        self.slope1 = (self.tau[-1] - self.tau[-k]) / max(self.ns[-1] - self.ns[-k], 1)

    def tau_at(self, ns):
        ns = np.asarray(ns, np.float64); t = np.interp(ns, self.ns, self.tau)
        t = np.where(ns < self.ns[0], self.tau[0] + (ns - self.ns[0]) * self.slope0, t)
        return np.where(ns > self.ns[-1], self.tau[-1] + (ns - self.ns[-1]) * self.slope1, t)

    def vr(self, ns):
        return np.interp(ns, self.ns, self.v), np.interp(ns, self.ns, self.r)


def yaw_of(q):
    return float(np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=["gt", "deploy"], required=True)
    ap.add_argument("--ego", required=True); ap.add_argument("--rows", required=True); ap.add_argument("--out", required=True)
    ap.add_argument("--route", default=os.path.join(HERE, "..", "assets", "route.json"))
    ap.add_argument("--r-source", default="grid",
                    help="deploy: LiDAR yaw rate: grid (the node's GridYawRate, hold), trend<window s> (GridYawRate with "
                         "the trend extension, e.g. trend0.3), imu (IMU wz)")
    ap.add_argument("bags", nargs="+")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    clk, z = Clock(a.ego), dict(np.load(a.ego))           # dict: NpzFile re-reads an array on every z[key]
    rows = sorted(int(x) for x in a.rows.split(","))
    route = Route(a.route); tracker, odo = RouteTracker(route), Odometer()
    inp, bev = Inputs(horizon=3.0), BevBuilder()
    imu_r = a.r_source == "imu"
    trend = float(a.r_source[5:]) if a.r_source.startswith("trend") else 0.0
    v_trk, r_trk = SignalTrack(), (SignalTrack() if imu_r else GridYawRate(trend_window=trend))
    r_err = []                                      # deploy: per scan (mean |r - training r|, max, training |r| mean)
    mode = a.mode if a.mode == "gt" else "deploy_" + a.r_source
    grid = list(zip(z["tau"], z["v"].astype(np.float64), z["yaw"])) if a.mode == "gt" else []   # gt: the tau grid is the
    pending, meta, n_scans = list(rows), [], 0                                                  # history signal

    snap, unsnapped = {}, list(rows)   # deploy: route s as the live node sees it at t0 (fixes up to t0 only, then odometer)

    def emit(i):
        t0 = float(z["tau"][i])
        s = float(z["path_s"][i]) if a.mode == "gt" else snap.get(i)
        req, problems = inp.request(t0, route.nav_text(s) if s is not None else "")
        if req is None:
            meta.append(dict(i=i, ok=False, problems=problems)); return
        req.update(row=i, mode=mode, s=s)
        open(os.path.join(a.out, "req_%06d.bin" % i), "wb").write(wire.pack(req))
        meta.append(dict(i=i, ok=True, problems=problems, s=s, nav=req["nav_text"]))

    for path in a.bags:
        b = rosbag.Bag(path)
        for topic, m, t in b.read_messages(topics=list(CAMS) + [LIDAR, STATUS, IMU, EKF]):
            tau_t = float(clk.tau_at(t.to_nsec()))
            while grid and grid[0][0] <= tau_t:
                g = grid.pop(0); inp.add_status(float(g[0]), float(g[1]), float(g[2]))
            while unsnapped and tau_t > float(z["tau"][unsnapped[0]]):
                i = unsnapped.pop(0); snap[i] = tracker.s_at(odo.at(float(z["tau"][i])))
            while pending and tau_t > float(z["tau"][pending[0]]) + 0.25:
                emit(pending.pop(0))
            if topic in CAMS:
                inp.add_camera(CAMS[topic], f32(tau_t), m.data)               # step 0: cams_index tau of the bag time
            elif topic == LIDAR:
                ns = np.array([p.stamp.to_nsec() for p in m.packets], np.float64)
                tt = clk.tau_at(ns)
                if a.mode == "gt":
                    v, r = clk.vr(ns)
                else:
                    if not (v_trk.ready() and r_trk.ready()): continue
                    v, r = v_trk.at(tt), r_trk.at(tt)
                    rt = clk.vr(ns)[1]; r_err.append((float(np.abs(r - rt).mean()), float(np.abs(r - rt).max()), float(np.abs(rt).mean())))
                h, d, npts, pose = bev.scan(b"".join(bytes(p.data) for p in m.packets), tt, v, r)
                inp.add_scan(f32(clk.tau_at(int(ns[-1]))), h, d, pose, bev.epoch); n_scans += 1
            elif topic == STATUS:
                ts = float(clk.tau_at(m.header.stamp.to_nsec()))
                v_trk.add(ts, m.signed_velocity_mps); odo.add(ts, m.signed_velocity_mps)
                if not imu_r: r_trk.add(ts, m.yaw_rad)
                if a.mode == "deploy": inp.add_status(ts, m.signed_velocity_mps, m.yaw_rad)
            elif topic == IMU and imu_r:
                r_trk.add(float(clk.tau_at(m.header.stamp.to_nsec())), m.angular_velocity.z)
            elif topic == EKF and a.mode == "deploy":
                p = m.pose.pose
                tracker.update(float(clk.tau_at(m.header.stamp.to_nsec())), p.position.x, p.position.y, yaw_of(p.orientation), odo.d)
        b.close()
    while pending: emit(pending.pop(0))
    e = np.array(r_err).reshape(-1, 3); turn = e[:, 2] > 0.1
    r_summary = None if not len(e) else dict(
        mean=round(float(e[:, 0].mean()), 5), max=round(float(e[:, 1].max()), 4),
        turn_mean=round(float(e[turn, 0].mean()), 5) if turn.any() else None, turn_scans=int(turn.sum()), scans=len(e))
    json.dump(dict(mode=mode, scans=n_scans, r_err_radps=r_summary, rows=meta), open(os.path.join(a.out, "meta.json"), "w"), indent=1)
    print(mode, "rows ok", sum(r["ok"] for r in meta), "/", len(meta), "scans", n_scans)


if __name__ == "__main__":
    main()
