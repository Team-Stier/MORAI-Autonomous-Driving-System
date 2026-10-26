"""fake_host packet encoders against a Python copy of the bridge's decoders (MORAI-UDP-Bridge-Node protocol.cpp:
decodeCameraPacket + CameraFrameAssembler, decodeGgaDatagram, decodeImuPacket timestamped, decodeVehicleStatusPacket):
the messages of a collection bag survive encode -> decode. The real bridge is the end-to-end check (fake_host run).

    uv run --with rosbags --with numpy python test/test_fake_host.py <collection bag> [n per stream]
"""
import math
import struct
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "tools"))
import fake_host as fh  # noqa: E402


def dec_camera(packets):
    chunks, key = {}, None
    for d in packets:
        assert d[:3] == b"MOR" and d[-1:] == b"I" and d[-2:-1] in (b"A", b"E")
        sec, nsec, idx, size = struct.unpack_from("<IIII", d, 3)
        assert key in (None, (sec, nsec)) and 0 < size <= len(d) - 21
        key = (sec, nsec); chunks[idx] = d[19:19 + size]
        final = d[-2:-1] == b"E"
    assert final and chunks.get(0, b"")[:2] == b"\xff\xd8"
    jpeg = b"".join(chunks[i] for i in range(len(chunks)))
    return key, jpeg[:jpeg.index(b"\xff\xd9", 2) + 2]


def dec_gga(d):
    s = d.decode().strip()
    body, cs = s[1:s.rindex("*")], int(s[s.rindex("*") + 1:], 16)
    x = 0
    for c in body.encode():
        x ^= c
    assert x == cs
    f = body.split(",")
    assert f[0][-3:] == "GGA" and len(f) >= 15

    def coord(v, h):
        if not v:
            return float("nan")
        raw = float(v); deg = math.floor(raw / 100); r = deg + (raw - deg * 100) / 60
        return -r if h in "SW" else r
    return int(f[6]), coord(f[2], f[3]), coord(f[4], f[5]), float(f[9]) + float(f[11]) if f[9] else float("nan")


def dec_imu(d):
    assert len(d) == 115 and d[:9] == b"#IMUData$" and d[-2:] == b"\r\n"
    return struct.unpack_from("<II", d, 25), struct.unpack_from("<10d", d, 33)


def dec_status(d):
    assert len(d) == 181 and d[:11] == b"#MoraiInfo$" and struct.unpack_from("<I", d, 11)[0] == 152 and d[179:] == b"\r\n"
    p, f = 27, lambda o: struct.unpack_from("<f", d, 27 + o)[0]
    rad = math.pi / 180
    sec, nsec = struct.unpack_from("<ii", d, p)
    return dict(source_sec=sec, source_nsec=nsec, control_mode=d[p + 8], gear=d[p + 9], signed_velocity_mps=f(10) / 3.6,
                map_data_id=struct.unpack_from("<i", d, p + 14)[0], accel=f(18), brake=f(22), size_x_m=f(26), size_y_m=f(30),
                size_z_m=f(34), overhang_m=f(38), wheelbase_m=f(42), rear_overhang_m=f(46), roll_rad=f(62) * rad,
                pitch_rad=f(66) * rad, yaw_rad=f(70) * rad, velocity_x_mps=f(74) / 3.6, angular_velocity_x_radps=f(86) * rad,
                angular_velocity_y_radps=f(90) * rad, angular_velocity_z_radps=f(94) * rad, steering_angle_rad=f(110) * rad,
                link_id=d[p + 114:p + 152].split(b"\0")[0].decode())


def main():
    from rosbags.highlevel import AnyReader
    bag, n = sys.argv[1], int(sys.argv[2]) if len(sys.argv) > 2 else 50
    counts = {}
    close = lambda a, b, tol: (math.isnan(a) and math.isnan(b)) or abs(a - b) <= tol * max(1.0, abs(b))
    with AnyReader([Path(bag)]) as r:
        conns = [c for c in r.connections if c.topic in ("/sensors/gps/fix", "/sensors/imu/data", "/vehicle/competition_status",
                                                         "/velodyne_packets", "/image/front/compressed_10hz")]
        for c, ns, raw in r.messages(connections=conns):
            if counts.get(c.topic, 0) >= n:
                if all(counts.get(x.topic, 0) >= n for x in conns):
                    break
                continue
            m = r.deserialize(raw, c.msgtype)
            if c.topic.startswith("/image/"):
                key, jpeg = dec_camera(fh.camera_packets(m.header.stamp.sec, m.header.stamp.nanosec, bytes(m.data)))
                assert key == (m.header.stamp.sec, m.header.stamp.nanosec) and jpeg == bytes(m.data)[:len(jpeg)]
            elif c.topic == "/sensors/gps/fix":
                q, la, lo, alt = dec_gga(fh.gga(m, ns * 1e-9))
                assert (q > 0) == (m.status.status >= 0)
                assert close(la, m.latitude, 1e-9) and close(lo, m.longitude, 1e-9) and close(alt, m.altitude, 1e-3), (la, m.latitude, lo, m.longitude)
            elif c.topic == "/sensors/imu/data":
                (sec, nsec), v = dec_imu(fh.imu(m))
                o, w, a = m.orientation, m.angular_velocity, m.linear_acceleration
                assert (sec, nsec) == (m.header.stamp.sec, m.header.stamp.nanosec) and v == (o.w, o.x, o.y, o.z, w.x, w.y, w.z, a.x, a.y, a.z)
            elif c.topic == "/vehicle/competition_status":
                s = dec_status(fh.status(m))
                for k, v in s.items():
                    want = getattr(m, k)
                    assert (v == want) if isinstance(v, (int, str)) else close(v, want, 1e-5), (k, v, want)
            elif c.topic == "/velodyne_packets":
                assert all(len(bytes(pk.data)) == 1206 for pk in m.packets)
            counts[c.topic] = counts.get(c.topic, 0) + 1
    print("ok", counts)


if __name__ == "__main__":
    main()
