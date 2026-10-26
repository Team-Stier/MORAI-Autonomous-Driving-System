#!/usr/bin/env python3
"""Fake competition MORAI host: a recorded drive (a collection bag) played back as the simulator's UDP output at real
time, to test the stack + model at 10 Hz without a simulator (bc-250 reaches RTF 0.5-0.7; the operators' PC runs Time
Scale 1, rules 2-7). Open loop: the stack's control comes back and is counted, the recording does not react to it.

    fake_host.py BAG [--listen 0.0.0.0:30011] [--start S] [--secs N] [--speed 1.0]
    stack host:  lan_tunnel.py stack --connect <this host>:30011       (as for the bc-250 simulator)

Streams, bridge_same_pc.yaml ports, packets built as MORAI-UDP-Bridge-Node protocol.cpp decodes them:
  cameras front/left/right/camera_7  9291/9293/9295/9297  'MOR' + sec, nsec, chunk index, size + JPEG chunk + 'AI'|'EI'
  GPS 9301 NMEA GGA, IMU 9999 '#IMUData$' (115 B), competition status 9099 '#MoraiInfo$' (181 B), VLP-16 2368 raw
  1206-byte packets (the bag's VelodyneScan, spread over the scan period). The bag's cameras are its 10 Hz copies.
Timing: the bag's receive clock is mapped to simulator time tau with rho = sum |d pos| / sum |v| dt over 5 s windows of
the Ego GT (/collect_only/ego_status), as sim_clock does live, and played at 1x tau: what a real-time host sends.
"""
import argparse
import math
import selectors
import socket
import struct
import time
from pathlib import Path

import numpy as np

CAMS = {"front": 9291, "left": 9293, "right": 9295, "camera_7": 9297}
GPS, IMU, STATUS, LIDAR, CTRL = 9301, 9999, 9099, 2368, 9093
CHUNK = 60000


def frame(port, data):
    return struct.pack("<HI", port, len(data)) + data


def camera_packets(sec, nsec, jpeg):
    parts = [jpeg[i:i + CHUNK] for i in range(0, len(jpeg), CHUNK)]
    return [b"MOR" + struct.pack("<IIII", sec, nsec, i, len(p)) + p + (b"EI" if i == len(parts) - 1 else b"AI")
            for i, p in enumerate(parts)]


def nmea(body):
    cs = 0
    for c in body.encode():
        cs ^= c
    return "$%s*%02X\r\n" % (body, cs)


def gga(fix, t):
    """sensor_msgs/NavSatFix -> GGA. A fix (status >= 0) gets quality 1; NO_FIX quality 0 with its coordinates."""
    def coord(v, pos, neg, width):
        if not math.isfinite(v):
            return "", ""
        a = abs(v); d = int(a); mins = (a - d) * 60.0
        return "%0*d%09.6f" % (width, d, mins), (pos if v >= 0 else neg)
    la, ns = coord(fix.latitude, "N", "S", 2)
    lo, ew = coord(fix.longitude, "E", "W", 3)
    hh = time.gmtime(t)
    alt = "%.3f" % fix.altitude if math.isfinite(fix.altitude) else ""
    return nmea("GPGGA,%02d%02d%02d.%02d,%s,%s,%s,%s,%d,12,0.9,%s,M,%s,M,," % (
        hh.tm_hour, hh.tm_min, hh.tm_sec, int((t % 1) * 100), la, ns, lo, ew, 1 if fix.status.status >= 0 else 0,
        alt, "0.000" if alt else "")).encode()


def imu(m):
    b = bytearray(115)
    b[0:9] = b"#IMUData$"
    struct.pack_into("<I", b, 9, 82)
    struct.pack_into("<II", b, 25, m.header.stamp.sec, m.header.stamp.nanosec)
    o, w, a = m.orientation, m.angular_velocity, m.linear_acceleration
    struct.pack_into("<10d", b, 33, o.w, o.x, o.y, o.z, w.x, w.y, w.z, a.x, a.y, a.z)
    b[113:115] = b"\r\n"
    return bytes(b)


def status(m):
    """interfaces/CompetitionVehicleStatus -> '#MoraiInfo$' (speeds back to km/h, angles to degrees)."""
    b = bytearray(181)
    b[0:11] = b"#MoraiInfo$"
    struct.pack_into("<I", b, 11, 152)
    p, deg = 27, 180.0 / math.pi
    struct.pack_into("<iiBBfi", b, p, m.source_sec, m.source_nsec, m.control_mode, m.gear, m.signed_velocity_mps * 3.6, m.map_data_id)
    struct.pack_into("<9f", b, p + 18, m.accel, m.brake, m.size_x_m, m.size_y_m, m.size_z_m, m.overhang_m, m.wheelbase_m,
                     m.rear_overhang_m, 0.0)
    struct.pack_into("<4f", b, p + 62, m.roll_rad * deg, m.pitch_rad * deg, m.yaw_rad * deg, m.velocity_x_mps * 3.6)
    struct.pack_into("<3f", b, p + 86, m.angular_velocity_x_radps * deg, m.angular_velocity_y_radps * deg, m.angular_velocity_z_radps * deg)
    struct.pack_into("<f", b, p + 110, m.steering_angle_rad * deg)
    link = m.link_id.encode()[:37]
    b[p + 114:p + 114 + len(link)] = link
    b[179:181] = b"\r\n"
    return bytes(b)


def tau_map(reader, conns):
    """bag ns -> tau seconds: rho over trailing 5 s windows of the Ego GT, held while standing (sim_clock's rule)."""
    t, x, y, v = [], [], [], []
    for c, ns, raw in reader.messages(connections=conns):
        m = reader.deserialize(raw, c.msgtype)
        t.append(ns * 1e-9); x.append(m.pose.pose.position.x); y.append(m.pose.pose.position.y); v.append(abs(m.twist.twist.linear.x))
    t, x, y, v = map(np.asarray, (t, x, y, v))
    d = np.r_[0, np.hypot(np.diff(x), np.diff(y))]
    vd = np.r_[0, 0.5 * (v[1:] + v[:-1]) * np.diff(t)]
    moving = (v > 0.5) & (np.r_[0, v[:-1]] > 0.5) & (d < 5.0)
    rho, r = np.empty(len(t)), 0.8
    lo = 0
    for i in range(len(t)):
        while t[lo] < t[i] - 5.0:
            lo += 1
        den = vd[lo:i + 1][moving[lo:i + 1]].sum()
        if den > 2.0:
            r = min(max(d[lo:i + 1][moving[lo:i + 1]].sum() / den, 0.2), 1.05)
        rho[i] = r
    tau = np.r_[0, np.cumsum(rho[1:] * np.diff(t))]
    return lambda ns: float(np.interp(ns * 1e-9, t, tau)), float(np.median(rho))


def events(reader, conns, tau):
    """(tau, port, payload) in bag order."""
    prev_scan = None
    for c, ns, raw in reader.messages(connections=conns):
        m, topic = reader.deserialize(raw, c.msgtype), c.topic
        if topic.startswith("/image/"):
            cam = topic.split("/")[2]
            for p in camera_packets(m.header.stamp.sec, m.header.stamp.nanosec, bytes(m.data)):
                yield tau(ns), CAMS[cam], p
        elif topic == "/sensors/gps/fix":
            yield tau(ns), GPS, gga(m, ns * 1e-9)
        elif topic == "/sensors/imu/data":
            yield tau(ns), IMU, imu(m)
        elif topic == "/vehicle/competition_status":
            yield tau(ns), STATUS, status(m)
        elif topic == "/velodyne_packets":
            period = min(0.2, (ns - prev_scan) * 1e-9) if prev_scan else 0.1
            prev_scan, n = ns, len(m.packets)
            for i, pk in enumerate(m.packets):
                yield tau(ns) - (n - 1 - i) * period / max(n, 1), LIDAR, bytes(pk.data)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag")
    ap.add_argument("--listen", default="0.0.0.0:30011")
    ap.add_argument("--start", type=float, default=0.0, help="skip this many seconds of tau")
    ap.add_argument("--secs", type=float, default=0.0, help="stop after this many seconds of tau (0: whole bag)")
    ap.add_argument("--speed", type=float, default=1.0)
    a = ap.parse_args()
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores, get_typestore, get_types_from_msg
    ts = get_typestore(Stores.ROS1_NOETIC)
    with AnyReader([Path(a.bag)], default_typestore=ts) as r:
        gt = [c for c in r.connections if c.topic == "/collect_only/ego_status"]
        tau, rho = tau_map(r, gt)
        want = [c for c in r.connections if c.topic in ("/sensors/gps/fix", "/sensors/imu/data", "/vehicle/competition_status",
                                                        "/velodyne_packets") or (c.topic.startswith("/image/") and c.topic.endswith("_10hz"))]
        print("bag %s: rho median %.2f, streams %s" % (a.bag, rho, sorted(c.topic for c in want)), flush=True)
        host, port = a.listen.rsplit(":", 1)
        srv = socket.socket(); srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1); srv.bind((host, int(port))); srv.listen(1)
        print("waiting for the stack tunnel on", a.listen, flush=True)
        conn, peer = srv.accept(); conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1); conn.setblocking(False)
        print("connected", peer, flush=True)
        sel = selectors.DefaultSelector(); sel.register(conn, selectors.EVENT_READ)
        w0, t0, sent, ctrl, late, nbytes = None, None, {}, [], [], 0
        buf = b""
        for t, p, data in events(r, want, tau):
            if t < a.start:
                continue
            if t0 is None:
                t0, w0 = t, time.perf_counter()
            if a.secs and t - t0 > a.secs:
                break
            due = w0 + (t - t0) / a.speed
            while True:
                rest = due - time.perf_counter()
                for key, _ in sel.select(timeout=max(rest, 0)):
                    try:
                        got = conn.recv(1 << 16)
                    except BlockingIOError:
                        got = b""
                    buf += got
                    while len(buf) >= 6:
                        cp, size = struct.unpack_from("<HI", buf, 0)
                        if len(buf) < 6 + size:
                            break
                        if cp == CTRL:
                            ctrl.append(time.perf_counter() - w0)
                        buf = buf[6 + size:]
                if rest <= 0:
                    break
            late.append(time.perf_counter() - due)
            conn.setblocking(True); conn.sendall(frame(p, data)); conn.setblocking(False)
            sent[p] = sent.get(p, 0) + 1; nbytes += len(data)
        dur = time.perf_counter() - w0
        gaps = np.diff(ctrl) if len(ctrl) > 1 else np.array([0.0])
        print("played %.1f s: packets %s, %.1f MB/s, late p99 %.1f ms; control %d (%.1f Hz, max gap %.0f ms)" % (
            dur, sent, nbytes / dur / 1e6, 1e3 * np.percentile(late, 99), len(ctrl), len(ctrl) / dur, 1e3 * gaps.max()), flush=True)


if __name__ == "__main__":
    main()
