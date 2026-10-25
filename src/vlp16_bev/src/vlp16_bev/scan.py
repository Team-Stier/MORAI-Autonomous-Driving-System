"""VLP16 scans -> per-scan BEV, the way the training data was made (no ROS imports; python 3.8 / numpy 1.17).

Training (Alpamayo-Cosmos tools/step0/step0_worker.py) read /velodyne_packets from the bags: velodyne_driver's scans
(VLP16_points.launch: 76 packets of 1206 bytes, each stamped with its receipt time, the scan with its last packet). The
stack runs the same driver, so the node reads the same messages.

Motion: lidar_bev.scan_bev_stream with v and r sampled at each packet time, as step 0 did from the _ego table:
  v  competition_status signed_velocity_mps (equal to the GT speed, median difference 0.0002 m/s in run1024b-w1-001)
  r  GridYawRate: the _ego yaw_rate recipe (coc_labeler load_episode) on competition_status yaw, which differs from the
     GT yaw by a constant 0.0235 rad; the node's default extends the newest 0.3 s by the yaw trend (grid_trend)
"""
import math

import numpy as np

from vlp16_bev import lidar_bev as L


class SignalTrack:
    """(t, x) samples of one signal; at(t) = np.interp, i.e. linear inside, the nearest end value outside (as step 0
    sampled the _ego table). Out-of-order samples are dropped."""

    def __init__(self, horizon=5.0):
        self.horizon, self.t, self.x = horizon, [], []

    def add(self, t, x):
        t = float(t)
        if self.t and t <= self.t[-1]:
            return
        self.t.append(t); self.x.append(float(x))
        while self.t[0] < t - self.horizon:
            self.t.pop(0); self.x.pop(0)

    def ready(self):
        return bool(self.t)

    def latest(self):
        return self.t[-1] if self.t else None

    def at(self, t):
        return np.interp(np.asarray(t, np.float64), np.array(self.t), np.array(self.x))


class GridYawRate:
    """yaw rate as the training _ego table has it: yaw on a 0.1 s grid, np.convolve(np.gradient(yaw, 0.1),
    np.ones(5) / 5, "same") (coc_labeler data.load_episode), linearly interpolated at the packet times (step 0).
    A grid point's value needs the yaw 0.3 s after it (central difference +-0.1 s, 5-tap box +-0.2 s), so live only the
    grid points with that support are used and packets newer than the last of them get its value (np.interp holds the
    end): r lags the training value by up to ~0.3 s at turn onset and exit. Grid points are multiples of 0.1 s of the
    stamp clock (the training grid started at the episode's tau 0; only the sampling phase differs)."""

    def __init__(self, dt=0.1, horizon=3.0, trend_window=0.0):
        """trend_window > 0: instead of holding, extend the yaw past the newest sample with a straight-line fit of the
        last trend_window s of yaw, far enough to give the queried times their 0.3 s support, then apply the same recipe."""
        self.dt, self.horizon, self.trend, self.t, self.yaw = dt, horizon, trend_window, [], []

    def add(self, t, yaw):
        t = float(t)
        if self.t and t <= self.t[-1]:
            return
        u = yaw if not self.yaw else self.yaw[-1] + math.atan2(math.sin(yaw - self._raw), math.cos(yaw - self._raw))
        self._raw = yaw
        self.t.append(t); self.yaw.append(u)
        while self.t[0] < t - self.horizon:
            self.t.pop(0); self.yaw.pop(0)

    def ready(self):
        return len(self.t) > 1 and self.t[-1] - self.t[0] >= 7 * self.dt

    def latest(self):
        return self.t[-1] if self.t else None

    def at(self, tq):
        t, y = np.array(self.t), np.array(self.yaw)
        tq = np.asarray(tq, np.float64)
        if self.trend > 0:
            m = t >= t[-1] - self.trend
            if m.sum() >= 2:
                k, c = np.polyfit(t[m] - t[-1], y[m], 1)
                fut = t[-1] + self.dt * np.arange(1, int(math.ceil((float(tq.max()) - t[-1] + 0.4) / self.dt)) + 1)
                t, y = np.r_[t, fut], np.r_[y, c + k * (fut - t[-1])]
        g = self.dt * np.arange(math.ceil(t[0] / self.dt), math.floor(t[-1] / self.dt) + 1)
        r = np.convolve(np.gradient(np.interp(g, t, y), self.dt), np.ones(5) / 5, "same")
        return np.interp(tq, g[3:-3], r[3:-3])                             # only points with their full 0.3 s support


class BevBuilder:
    """one lidar_bev.Odometry for the whole drive (step 0: one per episode) and the per-scan BEV."""

    def __init__(self):
        self.odom, self.epoch = L.Odometry(), 0

    def reset(self):
        self.odom, self.epoch = L.Odometry(), self.epoch + 1

    def scan(self, raw, t, v, r):
        """-> (height (320, 160) uint8, density (320, 160) uint8, npts, pose (x, y, yaw) of the last packet)."""
        if self.odom.last is not None and float(t[0]) < self.odom.last[0]:
            self.reset()                                  # clock went back (bag loop, sim reset): new drive
        return L.scan_bev_stream(raw, t, v, r, self.odom)
