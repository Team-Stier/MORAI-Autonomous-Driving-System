"""VLP16 UDP datagrams -> per-scan BEV, the way the training data was made (no ROS imports; python 3.8 / numpy 1.17).

Training (Alpamayo-Cosmos tools/step0/step0_worker.py) read /velodyne_packets from the bags. velodyne_driver had made those
scans with its default VLP16 settings (VLP16_points.launch: rpm 600, cut_angle -0.01 = off): every scan is the next
npackets = ceil(754 / 10) = 76 packets of exactly 1206 bytes, each stamped with its receipt time, the scan stamped with
its last packet. Every scan in the collected bags has 76 packets (run1024b-w1-001: 1336 scans). The scan boundary
azimuth is wherever the driver happened to start, so any phase is in distribution.

Motion: lidar_bev.scan_bev_stream with v and r sampled at each packet time. Step 0 used the _ego GT speed and yaw rate
(np.interp of the 10 Hz table); here they are competition_status signed_velocity_mps (equal to the GT speed, median
difference 0.0002 m/s in run1024b-w1-001) and IMU angular_velocity.z (left +, correlation 0.99 with the GT yaw rate,
same sign; competition_status angular_velocity_z_radps has the opposite sign).
"""
import numpy as np

from vlp16_bev import lidar_bev as L

PACKET_BYTES = 1206
PACKETS_PER_SCAN = 76


class ScanAssembler:
    """consecutive 1206-byte packets -> scans of 76 (velodyne_driver standard behaviour without a cut angle)."""

    def __init__(self, n=PACKETS_PER_SCAN):
        self.n, self.data, self.stamps = n, [], []
        self.dropped = 0                                  # datagrams of another size (velodyne InputSocket ignores them)

    def add(self, payload, stamp_ns):
        """one datagram and its receipt time (int ns, ROS clock) -> (raw bytes of 76 packets, (76,) int64 ns stamps) when a
        scan completes."""
        if len(payload) != PACKET_BYTES:
            self.dropped += 1
            return None
        self.data.append(bytes(payload)); self.stamps.append(int(stamp_ns))
        if len(self.data) < self.n:
            return None
        out = b"".join(self.data), np.array(self.stamps, np.int64)
        self.data, self.stamps = [], []
        return out


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
