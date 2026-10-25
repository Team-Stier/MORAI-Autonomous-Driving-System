"""VLP16 scan -> the PointCloud2 velodyne_pointcloud published in the collection stack, for lidar_leveler -> localization
(tunnel_wall reads x, y, z, time). A numpy float32 port of ros-drivers/velodyne @29abd0e (the submodule the collection
built), exactly as VLP16_points.launch ran it:
  RawData::unpack_vlp16 with params/VLP16db.yaml (only vertical corrections are non-zero; distance resolution 0.002 m),
  azimuth interpolated per firing, per-laser timing offsets, min_range 0.4 / max_range 130, no fixed/target frame (sensor
  frame `velodyne`, no motion compensation), PointcloudXYZIRT fields x y z intensity (f32) ring (u16) time (f32) packed
  without padding (point_step 22), height 1, is_dense, time = point time - scan stamp (the scan's last packet).
test/cloud_parity.py compares it with the real velodyne_pointcloud on collected scans.
"""
import ctypes
import ctypes.util
import math

import numpy as np


def _f32_trig():
    """float cosf/sinf as velodyne_pointcloud calls them (glibc libm via ctypes; numpy float64 rounding otherwise, which
    differs by 1 ulp for some angles)."""
    try:
        m = ctypes.CDLL(ctypes.util.find_library("m"))
        m.cosf.restype = m.sinf.restype = ctypes.c_float
        m.cosf.argtypes = m.sinf.argtypes = [ctypes.c_float]
        f = lambda fn, a: np.array([fn(float(v)) for v in np.asarray(a, np.float32)], np.float32)
        return lambda a: f(m.cosf, a), lambda a: f(m.sinf, a)
    except (OSError, AttributeError, TypeError):
        return (lambda a: np.cos(np.asarray(a, np.float32).astype(np.float64)).astype(np.float32),
                lambda a: np.sin(np.asarray(a, np.float32).astype(np.float64)).astype(np.float32))


COSF, SINF = _f32_trig()

VERT = np.array([-0.2617993877991494, 0.017453292519943295, -0.22689280275926285, 0.05235987755982989,
                 -0.19198621771937624, 0.08726646259971647, -0.15707963267948966, 0.12217304763960307,
                 -0.12217304763960307, 0.15707963267948966, -0.08726646259971647, 0.19198621771937624,
                 -0.05235987755982989, 0.22689280275926285, -0.017453292519943295, 0.2617993877991494], np.float32)
COS_V, SIN_V = COSF(VERT), SINF(VERT)                               # calibration.cc cosf / sinf(vert_correction)
RING = np.argsort(np.argsort(VERT.astype(np.float64), kind="stable"), kind="stable").astype(np.uint16)   # rank of the angle
_ROT = (np.float32(0.01) * np.arange(36000, dtype=np.float32)).astype(np.float64) * math.pi / 180.0       # from_degrees
COS_T, SIN_T = COSF(_ROT), SINF(_ROT)                                                                     # cosf / sinf tables
_OFF = np.array([[(55.296e-6 * (2 * b + y // 16)) + (2.304e-6 * (y % 16)) for y in range(32)] for b in range(12)],
                np.float64).astype(np.float32)                                                            # buildTimings
DSR = np.tile(np.arange(16), 2); FIRING = np.repeat(np.arange(2), 16)
_TOFF = (DSR.astype(np.float32) * np.float32(2.304) + FIRING.astype(np.float32) * np.float32(55.296)).astype(np.float32)
DTYPE = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("intensity", "<f4"), ("ring", "<u2"), ("time", "<f4")])
FIELDS = [("x", 0, 7), ("y", 4, 7), ("z", 8, 7), ("intensity", 12, 7), ("ring", 16, 4), ("time", 18, 7)]   # PointField
MIN_RANGE, MAX_RANGE = 0.4, 130.0


def _round_half_away(x):
    """C++ round(float) for x >= 0 (numpy rounds half to even)."""
    x = x.astype(np.float64)
    f = np.floor(x)
    return np.where(x - f >= 0.5, f + 1, f).astype(np.int64)


def _to_sec(d_ns):
    """ros::Duration(ns).toSec(): sec + 1e-9 nsec with nsec in [0, 1e9)."""
    sec = np.floor_divide(d_ns, 1000000000)
    return sec.astype(np.float64) + 1e-9 * (d_ns - sec * 1000000000).astype(np.float64)


def scan_cloud(raw, stamps_ns, scan_stamp_ns):
    """raw: n x 1206 bytes, stamps_ns (n,) packet times (int ns), scan_stamp_ns: the cloud stamp (the last packet's)
    -> structured array (DTYPE), points in velodyne_pointcloud's order."""
    pk = np.frombuffer(raw, np.uint8).reshape(-1, 1206)[:, :1200].reshape(-1, 12, 100)
    head = pk[:, :, 0].astype(np.uint16) | (pk[:, :, 1].astype(np.uint16) << 8)
    rot = (pk[:, :, 2].astype(np.int64) | (pk[:, :, 3].astype(np.int64) << 8))
    n = pk.shape[0]
    valid = np.cumprod(head == 0xEEFF, axis=1).astype(bool)          # a bad block header skips the rest of its packet
    az_diff = np.zeros((n, 12), np.float32); use = valid.copy()
    last = np.zeros(n, np.float32)
    for b in range(12):                                              # the per-packet sequential azimuth_diff logic
        if b < 11:
            raw_diff = rot[:, b + 1] - rot[:, b]
            d = ((36000 + raw_diff) % 36000).astype(np.float32)
            over = raw_diff < 0
            skip = over & ~(last > 0)
            d = np.where(over & (last > 0), last, d)
            use[:, b] &= ~skip
            last = np.where(skip, last, d)
            az_diff[:, b] = d
        else:
            az_diff[:, b] = last
    dist_raw = (pk[:, :, 4:].reshape(n, 12, 32, 3)[..., 0].astype(np.uint16)
                | (pk[:, :, 4:].reshape(n, 12, 32, 3)[..., 1].astype(np.uint16) << 8))
    inten = pk[:, :, 4:].reshape(n, 12, 32, 3)[..., 2].astype(np.float32)
    azimuth = rot.astype(np.float32)[:, :, None]
    az_f = (azimuth + (az_diff[:, :, None] * _TOFF[None, None, :]) / np.float32(110.592)).astype(np.float32)
    az = _round_half_away(az_f) % 36000
    distance = (dist_raw.astype(np.float32) * np.float32(0.002)).astype(np.float32)
    xy = (distance * COS_V[DSR]).astype(np.float32)
    sin_r, cos_r = SIN_T[az], COS_T[az]
    x = (xy * sin_r).astype(np.float32); y = (xy * cos_r).astype(np.float32); z = (distance * SIN_V[DSR]).astype(np.float32)
    tdiff = _to_sec(np.asarray(stamps_ns, np.int64) - int(scan_stamp_ns)).astype(np.float32)
    time = (_OFF[None, :, :] + tdiff[:, None, None]).astype(np.float32)
    keep = use[:, :, None] & (distance.astype(np.float64) >= MIN_RANGE) & (distance.astype(np.float64) <= MAX_RANGE)
    keep = np.broadcast_to(keep, distance.shape)
    out = np.empty(int(keep.sum()), DTYPE)
    out["x"], out["y"], out["z"] = y[keep], -x[keep], z[keep]        # ROS axes: x_coord = y, y_coord = -x
    out["intensity"] = np.clip(np.broadcast_to(inten, distance.shape)[keep], 0, 255)
    out["ring"] = np.broadcast_to(RING[DSR], distance.shape)[keep]
    out["time"] = time[keep]
    return out


def _check():
    """cheap self-checks of the tables (run as a script)."""
    assert list(RING[:4]) == [0, 8, 1, 9] and RING[15] == 15
    assert DTYPE.itemsize == 22 and [DTYPE.fields[f][1] for f, _, _ in FIELDS] == [o for _, o, _ in FIELDS]
    assert _round_half_away(np.array([0.5, 1.4999999, 2.5], np.float32)).tolist() == [1, 1, 3]
    print("cloud tables ok")


if __name__ == "__main__":
    _check()
