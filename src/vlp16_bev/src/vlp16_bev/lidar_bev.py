"""VLP16 packets -> motion-compensated 320x160 BEV, and the 2-scan BEV pair the model reads. One copy shared by the step-0
worker (tools/step0/step0_worker.py, Noetic chroot: python 3.8, numpy 1.17), the training loader (morai_vla.data) and the
inference code. Pure numpy, no package imports, so the worker can import this file directly.

Motion comes only from speed v (m/s) and yaw rate r (rad/s, left turn +), integrated by `Odometry`. Earlier BEVs used the GT
pose (x, y, yaw) of the Ego status table instead. Those positions are placed by message arrival time and their short-range
steps swing +-30 % (train/README.md, "위치는 GT 속도와 yaw를 τ로 적분해 만든다"), and the competition provides no GT pose.

Training (step 0, tools/step0/step0_worker.py):
  - v and r are the _ego GT `v` and `yaw_rate`, interpolated to each packet's bag_ns.
  - Time is the restored sim clock tau of each packet, not wall time: the sim ran at 0.71-0.95x real time.
Inference:
  - v: /vehicle/competition_status speed, converted to m/s. r: IMU angular_velocity.z. Check the IMU sign and frame against GT
    on one recorded bag first.
  - Time: the packet stamps in seconds. The competition runs at Time Scale 1, so wall time equals sim time.
  - Sample v and r at each packet stamp: interpolate between messages, or hold the latest one.
  - Keep one Odometry per drive and call
        height, density, npts, pose = scan_bev_stream(raw, t, v, r, odom)
    per scan. `raw` is the concatenated 1206-byte packets of one revolution (one VelodyneScan message).
  - Keep (np.stack([height, density]), pose) of the last 4 scans and feed pack_bev(scan_k, scan_k-1, pose_k, pose_k-1), as
    morai_vla.data does.

BEV layout: row = (XF - x) / RES, col = (YW - y) / RES in the ego frame at the scan's last packet (base_link: rear axle at
ground level). height: 0 = empty, else 0.05 m steps from -1 m (max over the cell). density: point count, capped at 255.
"""
import numpy as np

MOUNT = np.array([1.5, 0.0, 1.403])              # VLP16 in base_link (rear axle, ground level)
VERT = np.radians(np.array([-15, 1, -13, 3, -11, 5, -9, 7, -7, 9, -5, 11, -3, 13, -1, 15], np.float64))
RES, XF, YW, H, W = 0.25, 60.0, 20.0, 320, 160   # x 60 .. -20 m forward, y +20 .. -20 m left
BODY = (-1.3, 4.2, 1.15)                          # ego body box in base_link (x from, x to, |y|) -> dropped
DENSITY_GAIN = 16                                 # point count -> uint8 channel of the pair (median occupied cell has 4 points)
BEV_RES, BEV_XF, BEV_YW = RES, XF, YW             # names morai_vla.data re-exports


def decode(raw):
    """concatenated VLP16 packets (bytes or uint8 array, n x 1206) -> (points (N, 3) in the sensor frame, packet index per point)."""
    raw = np.frombuffer(raw, np.uint8) if isinstance(raw, (bytes, bytearray)) else np.asarray(raw, np.uint8)
    raw = raw.reshape(-1, 1206)[:, :1200].reshape(-1, 12, 100)
    az = (raw[:, :, 2].astype(np.uint16) | (raw[:, :, 3].astype(np.uint16) << 8)).astype(np.float64) / 100.0
    ch = raw[:, :, 4:].reshape(-1, 12, 32, 3)
    dist = (ch[..., 0].astype(np.uint16) | (ch[..., 1].astype(np.uint16) << 8)).astype(np.float64) * 0.002
    a = np.radians(az)[..., None] + np.where(np.arange(32) >= 16, np.radians(0.1), 0.0)
    w = np.tile(VERT, 2)[None, None, :]; xy = dist * np.cos(w)
    pts = np.stack([xy * np.cos(a), -xy * np.sin(a), dist * np.sin(w)], -1).reshape(-1, 3)
    pk = np.repeat(np.arange(raw.shape[0]), 12 * 32)
    keep = dist.reshape(-1) > 0.1
    return pts[keep], pk[keep]


def integrate(t, v, r, x0=0.0, y0=0.0, h0=0.0):
    """poses (x, y, yaw) at the sample times t, starting from (x0, y0, h0) at t[0]. Each step uses the mean v and r of its two
    ends and moves along the chord of that constant-curvature arc, so constant (v, r) gives the exact line or circle."""
    t, v, r = (np.asarray(a, np.float64).reshape(-1) for a in (t, v, r))
    dt = np.maximum(np.diff(t), 0.0)
    dh = 0.5 * (r[1:] + r[:-1]) * dt
    ds = 0.5 * (v[1:] + v[:-1]) * dt * np.sinc(dh / (2 * np.pi))                 # chord = arc * sin(dh/2) / (dh/2)
    h = h0 + np.r_[0.0, np.cumsum(dh)]
    hc = h[:-1] + 0.5 * dh
    return x0 + np.r_[0.0, np.cumsum(ds * np.cos(hc))], y0 + np.r_[0.0, np.cumsum(ds * np.sin(hc))], h


class Odometry:
    """dead reckoning over a whole drive: poses in the frame of its first sample. advance() takes samples newer than the last."""
    def __init__(self):
        self.last = None                                                          # (t, v, r, x, y, yaw) of the latest sample

    def advance(self, t, v, r):
        t, v, r = (np.asarray(a, np.float64).reshape(-1) for a in (t, v, r))
        if self.last is None:
            x, y, h = integrate(t, v, r)
        else:
            t0, v0, r0, x0, y0, h0 = self.last
            x, y, h = (a[1:] for a in integrate(np.r_[t0, t], np.r_[v0, v], np.r_[r0, r], x0, y0, h0))
        self.last = (t[-1], v[-1], r[-1], x[-1], y[-1], h[-1])
        return x, y, h


def to_frame(x, y, h, ref):
    """poses (x, y, yaw) expressed in the frame of the pose ref = (x, y, yaw)."""
    dx, dy = np.asarray(x) - ref[0], np.asarray(y) - ref[1]
    c, s = np.cos(ref[2]), np.sin(ref[2])
    return c * dx + s * dy, -s * dx + c * dy, np.asarray(h) - ref[2]


def compensate(q, pk, x, y, h):
    """base_link points q measured at packet pk -> the reference frame in which the packet poses (x, y, h) are given (2-D motion;
    height unchanged)."""
    px, py, ph = x[pk], y[pk], h[pk]
    c, s = np.cos(ph), np.sin(ph)
    return np.stack([px + c * q[:, 0] - s * q[:, 1], py + s * q[:, 0] + c * q[:, 1], q[:, 2]], 1)


def rasterize(P):
    """points in the BEV frame -> (height (H, W) uint8, density (H, W) uint8, number of points kept)."""
    P = P[~((P[:, 0] > BODY[0]) & (P[:, 0] < BODY[1]) & (np.abs(P[:, 1]) < BODY[2]))]
    P = P[(P[:, 0] > XF - H * RES) & (P[:, 0] < XF) & (np.abs(P[:, 1]) < YW)]
    r = ((XF - P[:, 0]) / RES).astype(np.int32); c = ((YW - P[:, 1]) / RES).astype(np.int32)
    hgt = np.zeros((H, W), np.uint8); den = np.zeros((H, W), np.uint16)
    hq = np.clip((P[:, 2] + 1.0) / 0.05, 1, 255).astype(np.uint8)
    np.maximum.at(hgt, (r, c), hq); np.add.at(den, (r, c), 1)
    return hgt, np.minimum(den, 255).astype(np.uint8), len(P)


def scan_bev(raw, x, y, h):
    """one scan -> BEV in the frame where the per-packet poses (x, y, h) are given (normally the scan's last packet)."""
    q, pk = decode(raw)
    return rasterize(compensate(q + MOUNT, pk, x, y, h))


def scan_bev_stream(raw, t, v, r, odom):
    """one scan, with per-packet times t and v, r sampled at them -> (height, density, npts, pose). pose = the Odometry pose of
    the last packet (the BEV frame), to be passed to pack_bev."""
    x, y, h = odom.advance(t, v, r)
    pose = (x[-1], y[-1], h[-1])
    return scan_bev(raw, *to_frame(x, y, h, pose)) + (pose,)


def pack_bev(new, old, pose_new, pose_old):
    """2 scans ((2, H, W) height, density) -> one (3, H, W) uint8 BEV in the newer scan's frame: newer height, older height
    moved by the relative motion of the two poses (any common frame, e.g. one Odometry), newer density x DENSITY_GAIN."""
    (h_new, d_new), (h_old, _) = new, old
    r, c = np.nonzero(h_old)
    x, y = XF - (r + 0.5) * RES, YW - (c + 0.5) * RES
    xo, yo, ho = pose_old
    x2, y2, _ = to_frame(xo + np.cos(ho) * x - np.sin(ho) * y, yo + np.sin(ho) * x + np.cos(ho) * y, 0.0, pose_new)
    r2, c2 = np.floor((XF - x2) / RES).astype(int), np.floor((YW - y2) / RES).astype(int)
    m = (r2 >= 0) & (r2 < h_old.shape[0]) & (c2 >= 0) & (c2 < h_old.shape[1])
    warped = np.zeros_like(h_old)
    np.maximum.at(warped, (r2[m], c2[m]), h_old[r[m], c[m]])
    den = np.minimum(d_new.astype(np.int32) * DENSITY_GAIN, 255).astype(np.uint8)
    return np.stack([h_new, warped, den])
