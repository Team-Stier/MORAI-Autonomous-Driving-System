#!/usr/bin/env python3
"""diag_rec.py OUT.jsonl [HZ]: the competition stack's state at HZ (5) for stop / stuck diagnosis (eval3.sh DRIVER=stack
runs it per stack as diag_wN.jsonl, in DRIVE_WS's environment). One line per tick: t (ROS time, tau with SIM_CLOCK), wall,
ego [x, y, yaw, v] from /localization/ego, section (/section_annotation state), tl (/traffic_light state), objects: n and
the ones within 40 m of the ego as [id, label, x, y, dist, sx, sy, vx, vy], path: /local_path points and age, vpath:
/local_path/velocity_injected points, first velocity and the lowest velocity of its first 30 m, leveled: age of the last
/velodyne_points/leveled, cmd: [accel, brake, steering] of /control_cmd. Stack topics only (no GT)."""
import json
import math
import sys
import time

import rospy
from interfaces.msg import Objects, Path, SectionAnnotation, TrafficLight
from morai_msgs.msg import CtrlCmd
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2

last = {}


def keep(name):
    def cb(m):
        last[name] = (m, rospy.get_time())
    return cb


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def tick(f):
    now = rospy.get_time()
    r = dict(t=round(now, 3), wall=round(time.time(), 3))
    if "ego" in last:
        m = last["ego"][0]
        p, v = m.pose.pose, m.twist.twist.linear
        ex, ey = p.position.x, p.position.y
        r["ego"] = [round(ex, 2), round(ey, 2), round(yaw(p.orientation), 3), round(math.hypot(v.x, v.y), 2)]
    if "section" in last:
        r["section"] = last["section"][0].state
    if "tl" in last:
        r["tl"] = last["tl"][0].state
    if "objects" in last and "ego" in r:
        objs = last["objects"][0].objects
        near = []
        for o in objs:
            d = math.hypot(o.position.x - ex, o.position.y - ey)
            if d < 40.0:
                near.append([o.id, o.label, round(o.position.x, 2), round(o.position.y, 2), round(d, 1), round(o.bbox_size.x, 2),
                             round(o.bbox_size.y, 2), round(o.velocity.x, 2), round(o.velocity.y, 2)])
        r["objects"] = dict(n=len(objs), near=sorted(near, key=lambda x: x[4])[:12], age=round(now - last["objects"][1], 2))
    if "path" in last:
        r["path"] = dict(n=len(last["path"][0].points), age=round(now - last["path"][1], 2))
    if "vpath" in last:
        pts = last["vpath"][0].points
        v30, s, prev = [], 0.0, None
        for pt in pts:
            q = pt.pose.pose.position
            if prev is not None:
                s += math.hypot(q.x - prev.x, q.y - prev.y)
            prev = q
            if s > 30.0:
                break
            v30.append(pt.velocity)
        r["vpath"] = dict(n=len(pts), v0=round(pts[0].velocity, 2) if pts else None, vmin30=round(min(v30), 2) if v30 else None,
                          age=round(now - last["vpath"][1], 2))
    if "leveled" in last:
        r["leveled_age"] = round(now - last["leveled"][1], 2)
    if "cmd" in last:
        c = last["cmd"][0]
        r["cmd"] = [round(c.accel, 3), round(c.brake, 3), round(c.steering, 3)]
    f.write(json.dumps(r) + "\n")
    f.flush()


def main():
    rospy.init_node("diag_rec", anonymous=True)
    for topic, typ, name in (("/localization/ego", Odometry, "ego"), ("/section_annotation", SectionAnnotation, "section"),
                             ("/traffic_light", TrafficLight, "tl"), ("/objects", Objects, "objects"), ("/local_path", Path, "path"),
                             ("/local_path/velocity_injected", Path, "vpath"), ("/velodyne_points/leveled", PointCloud2, "leveled"),
                             ("/control_cmd", CtrlCmd, "cmd")):
        rospy.Subscriber(topic, typ, keep(name), queue_size=1)
    rate = rospy.Rate(float(sys.argv[2]) if len(sys.argv) > 2 else 5.0)
    with open(sys.argv[1], "a") as f:
        while not rospy.is_shutdown():
            try:
                tick(f)
            except Exception as e:                 # a diagnostic recorder: log and go on
                rospy.logwarn_throttle(10, "diag_rec: %r", e)
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                break


if __name__ == "__main__":
    main()
