#!/usr/bin/env python3
"""shadow_replay.py: a recorded drive replayed into the shadow in a local container, no box (README "쉐도 로컬 재생").
  make REC_DIR T0 T1 OUT.json   (Mac, uv numpy; dagger_label.py beside) a window of shadow records (wall clock T0..T1:
                                 ego, Object Info) as replay input; the first sample is held while the shadow starts
  play IN.json HOLD_S           (container, shadow workspace sourced, ROS master 11311 up) Ego status (9095, 50 Hz) and
                                 Object Info (7505, 20 Hz) as MORAI sends them, /vehicle/competition_status and IMU on
                                 11311 for shadow_relay; holds the first sample HOLD_S (shadow.sh up/restart), plays,
                                 holds the last 3 s
  plans REC_DIR MARKS [STEP_S]  (Mac) what the shadow planned in each replay (MARKS: lines "name wall_start hold"):
                                 car (route s, offset), plan length, end offset, speeds
Open loop: the car follows the record, not the shadow; read the plan per state, not the car's path."""
import json
import math
import socket
import struct
import sys
import time
from pathlib import Path

import numpy as np


def make(rec_dir, t0, t1, out):
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
    import dagger_label as dl
    rows = dl.load_rec(rec_dir, t0, t1)
    json.dump(dict(ego=[[r[1] - t0] + r[2:6] for r in rows["ego"]], obj=[[r[1] - t0, r[2]] for r in rows["obj"]]),
              open(out, "w"))
    print(f"{out}: {len(rows['ego'])} ego, {len(rows['obj'])} object frames, {t1 - t0:.1f} s")


def play(path, hold):
    import rospy
    from interfaces.msg import CompetitionVehicleStatus
    from sensor_msgs.msg import Imu
    d = json.load(open(path))
    E = np.array([[float(v) for v in e] for e in d["ego"]]); O = d["obj"]; OT = np.array([o[0] for o in O])
    rospy.init_node("shadow_replay")
    pc = rospy.Publisher("/vehicle/competition_status", CompetitionVehicleStatus, queue_size=1)
    pi = rospy.Publisher("/sensors/imu/data", Imu, queue_size=1)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    start, k, T = time.time(), 0, E[-1, 0]
    while not rospy.is_shutdown():
        w = time.time() - start - hold
        if w > T + 3:
            break
        t = min(max(w, 0.0), T)
        _, x, y, yaw, v = E[max(0, int(np.searchsorted(E[:, 0], t, side="right")) - 1)]
        pk = bytearray(229); pk[:11] = b"#MoraiInfo$"; struct.pack_into("<I", pk, 11, 200)
        struct.pack_into("<f", pk, 37, v * 3.6)          # the GT node subtracts its yaw offset again: add it here
        struct.pack_into("<6f", pk, 77, x, y, 28.5, 0.0, 0.0, math.degrees(yaw + 0.023509))
        pk[-2:] = b"\r\n"; sock.sendto(bytes(pk), ("127.0.0.1", 9095))
        m = CompetitionVehicleStatus(); m.header.stamp = rospy.Time.now(); m.header.frame_id = "base_link"
        m.signed_velocity_mps = m.velocity_x_mps = v; m.wheelbase_m = 3.0; pc.publish(m)
        im = Imu(); im.header.stamp = m.header.stamp; im.header.frame_id = "imu_link"; im.orientation.w = 1.0; pi.publish(im)
        if k % 2 == 0:                                    # Object Info: centre, heading, size, body-frame km/h
            frame = O[max(0, int(np.searchsorted(OT, t, side="right")) - 1)][1]
            o = bytearray(1400); o[:14] = b"#MoraiObjInfo$"; struct.pack_into("<i", o, 14, 1400 - 32); o[-2:] = b"\r\n"
            for n, (oid, lab, ox, oy, oyaw, vx, vy, va, sx, sy) in enumerate(frame[:20]):
                struct.pack_into("<hh16f", o, 38 + n * 68, int(oid), {"pedestrian": 0, "vehicle": 1}.get(lab, 2), ox, oy,
                                 28.5, math.degrees(oyaw), sx, sy, 1.5, 0, 0, 0, vx * 3.6, vy * 3.6, 0, 0, 0, 0)
            sock.sendto(bytes(o), ("127.0.0.1", 7505))
        k += 1
        time.sleep(0.02)


def plans(rec_dir, marks, step):
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
    import dagger_label as dl
    lane = dl.Lane(str(Path(__file__).resolve().parent.parent.parent / "assets" / "route.json"))
    M = [l.split() for l in open(marks) if len(l.split()) > 2]
    rows = dl.load_rec(rec_dir, 0, 1e12); rec = dl.Rec(rows)
    for i, (name, t0, hold) in enumerate(M):
        a, b = float(t0) + float(hold.removeprefix("hold")), float(M[i + 1][1]) if i + 1 < len(M) else 1e12
        print("==", name, "(t from the play start)")
        last = -9.0
        for r in rows.get("vp", []):
            t = r[1]
            if not a <= t <= b or t - last < step or (e := rec.ego_at(t)) is None:
                continue
            last = t
            s, d, _ = lane.project(e[0], e[1]); xs, ys, vs = np.array(r[3]), np.array(r[4]), np.array(r[5], float)
            es, ed, _ = lane.project(xs[-1], ys[-1])
            print(f"t {t - a:6.1f} car s {s:7.1f} d {d:5.1f} v {e[3]:5.1f} | plan {np.hypot(*np.diff(np.c_[xs, ys], axis=0).T).sum():5.1f} m"
                  f" end s {es:7.1f} d {ed:5.1f} v0 {vs[0]:5.2f} vmin {np.nanmin(vs):5.2f}")


if __name__ == "__main__":
    c = sys.argv[1]
    if c == "make":
        make(sys.argv[2], float(sys.argv[3]), float(sys.argv[4]), sys.argv[5])
    elif c == "play":
        play(sys.argv[2], float(sys.argv[3]))
    elif c == "plans":
        plans(sys.argv[2], sys.argv[3], float(sys.argv[4]) if len(sys.argv) > 4 else 0.5)
    else:
        sys.exit(__doc__)
