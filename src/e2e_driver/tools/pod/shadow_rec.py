#!/usr/bin/env python3
"""shadow_rec.py MODE OUT_DIR: in the shadow master (shadow.sh), everything dagger_label.py needs from the collection
driver, one JSON list per message, receipt time t = host wall clock (time.time(), the clock of the engine dumps'
t_wall), to OUT_DIR/rec_<start epoch>.jsonl:
  ["ego", t, x, y, yaw, v]                         /collect_only/ego_status (GT, rear axle, signed m/s)
  ["vp", t, stamp, xs, ys, vs, [tx, ty, tyaw]]     /local_path/velocity_injected (control's input) back in map with the
                                                   latest map->base_link /tf (the one velocity_planner used), point speeds
  ["lp", t, stamp, xs, ys]                         /local_path (map, path_planner)
  ["cmd", t, src, accel, brake, steer, mark]       commands: src control | yield | aeb | bridge (drive mode: what the
                                                   bridge gets; mark = CtrlCmd.acceleration, -97 yield, -99 AEB)
  ["obj", t, [[id, label, x, y, yaw, vx, vy, va, sx, sy], ...]]   /collect_only/objects_all (GT)
  ["tl", t, state], ["tlgt", t, index, status], ["sec", t, section], ["comp", t, v, steer_rad, accel, brake]
MODE shadow: commands from /shadow/{control,yield,aeb}_cmd; drive: /control_cmd and /morai_udp_bridge/control_cmd."""
import json
import math
import os
import sys
import threading
import time


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def r(v, d=2):
    return round(float(v), d)


def main():
    import rospy
    from interfaces.msg import CompetitionVehicleStatus, Objects, Path, SectionAnnotation, TrafficLight
    from morai_msgs.msg import CtrlCmd, GetTrafficLightStatus
    from nav_msgs.msg import Odometry
    from tf2_msgs.msg import TFMessage

    mode, out = sys.argv[1], sys.argv[2]
    os.makedirs(out, exist_ok=True)
    f = open(os.path.join(out, f"rec_{int(time.time())}.jsonl"), "w", buffering=1 << 16)
    lock, tf, last = threading.Lock(), [None], {"comp": 0.0}
    rospy.init_node("shadow_rec")

    def put(row):
        with lock:
            f.write(json.dumps(row, separators=(",", ":")) + "\n")

    def on_tf(m):
        for x in m.transforms:
            if x.header.frame_id == "map" and x.child_frame_id == "base_link":
                tr = x.transform
                tf[0] = (tr.translation.x, tr.translation.y, yaw_of(tr.rotation))

    def on_vp(m):
        if tf[0] is None:
            return
        tx, ty, th = tf[0]
        c, s = math.cos(th), math.sin(th)
        ps = [p.pose.pose.position for p in m.points]
        put(["vp", r(time.time(), 3), r(m.header.stamp.to_sec(), 3), [r(tx + c * p.x - s * p.y) for p in ps],
             [r(ty + s * p.x + c * p.y) for p in ps], [r(p.velocity) for p in m.points], [r(tx, 3), r(ty, 3), r(th, 4)]])

    def on_lp(m):
        ps = [p.pose.pose.position for p in m.points]
        put(["lp", r(time.time(), 3), r(m.header.stamp.to_sec(), 3), [r(p.x) for p in ps], [r(p.y) for p in ps]])

    def on_ego(m):
        p = m.pose.pose.position
        put(["ego", r(time.time(), 3), r(p.x, 3), r(p.y, 3), r(yaw_of(m.pose.pose.orientation), 4), r(m.twist.twist.linear.x, 3)])

    def on_cmd(src):
        return lambda m: put(["cmd", r(time.time(), 3), src, r(m.accel, 3), r(m.brake, 3), r(m.steering, 4), r(m.acceleration, 1)])

    def on_obj(m):
        put(["obj", r(time.time(), 3), [[o.id, o.label, r(o.bbox_pose.position.x), r(o.bbox_pose.position.y), r(yaw_of(o.bbox_pose.orientation), 3),
                                         r(o.velocity.x), r(o.velocity.y), int(o.velocity_available), r(o.bbox_size.x), r(o.bbox_size.y)]
                                        for o in m.objects]])

    def on_comp(m):
        t = time.time()
        if t - last["comp"] >= 0.05:          # 20 Hz is plenty
            last["comp"] = t
            put(["comp", r(t, 3), r(m.signed_velocity_mps, 3), r(m.steering_angle_rad, 4), r(m.accel, 3), r(m.brake, 3)])

    S = rospy.Subscriber
    S("/tf", TFMessage, on_tf, queue_size=50)
    S("/local_path/velocity_injected", Path, on_vp, queue_size=20)
    S("/local_path", Path, on_lp, queue_size=20)
    S("/collect_only/ego_status", Odometry, on_ego, queue_size=100)
    S("/collect_only/objects_all", Objects, on_obj, queue_size=20)
    S("/traffic_light", TrafficLight, lambda m: put(["tl", r(time.time(), 3), m.state]), queue_size=20)
    S("/collect_only/traffic_light_gt", GetTrafficLightStatus,
      lambda m: put(["tlgt", r(time.time(), 3), m.trafficLightIndex, m.trafficLightStatus]), queue_size=20)
    S("/section_annotation", SectionAnnotation, lambda m: put(["sec", r(time.time(), 3), m.state]), queue_size=20)
    S("/vehicle/competition_status", CompetitionVehicleStatus, on_comp, queue_size=50)
    if mode == "drive":
        S("/control_cmd", CtrlCmd, on_cmd("control"), queue_size=100)
        S("/morai_udp_bridge/control_cmd", CtrlCmd, on_cmd("bridge"), queue_size=500)
    else:
        for src in ("control", "yield", "aeb"):
            S(f"/shadow/{src}_cmd", CtrlCmd, on_cmd(src), queue_size=500)
    while not rospy.is_shutdown():
        time.sleep(1.0)
        with lock:
            f.flush()


if __name__ == "__main__":
    main()
