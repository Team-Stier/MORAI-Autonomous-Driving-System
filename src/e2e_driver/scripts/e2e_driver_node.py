#!/usr/bin/env python3
"""The model node: MORAI sensors (bridge, vlp16_bev, localization) -> model server -> /control_cmd (morai_msgs/CtrlCmd),
which speed_filter limits and forwards to the bridge (/morai_udp_bridge/control_cmd -> MORAI UDP 9093).

Inference loop (~rate_hz): t0 = now -> inputs.Inputs.request (training-identical picks, step-0 JPEGs) -> model server
(TCP, e2e_driver.wire) -> Plan (64 x (a, kappa)). /control_cmd is published once as soon as a plan arrives and then at
control_rate_hz from the newest plan (control.Controller). /e2e_driver/debug (std_msgs/String, JSON) carries per-step
timings: latency_ms = command publish time - receipt stamp of the newest front-camera frame used.
"""
import json
import threading
import time

import rospy
from interfaces.msg import CompetitionVehicleStatus, LidarBevScan
from morai_msgs.msg import CtrlCmd
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage, NavSatFix
from std_msgs.msg import String

import numpy as np

from e2e_driver.control import DEFAULTS, Controller, Plan
from e2e_driver.inputs import CAMERAS, Inputs
from e2e_driver.route import Odometer, Route, RouteTracker
from e2e_driver.wire import Client


def yaw_of(q):
    return float(np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))


class Node:
    def __init__(self):
        P = lambda k, d: rospy.get_param("~" + k, d)
        self.rate, self.control_rate = float(P("rate_hz", 10.0)), float(P("control_rate_hz", 50.0))
        self.lidar = bool(P("use_lidar", True))
        self.inputs = Inputs()
        self.route = Route(P("route_asset", ""))
        self.tracker, self.odo = RouteTracker(self.route), Odometer()
        self.ctrl = Controller(**{k: P("control/" + k, v) for k, v in DEFAULTS.items()})
        self.client = Client(P("server_host", "127.0.0.1"), P("server_port", 47011), float(P("server_timeout_s", 5.0)))
        self.plan, self.v, self.plan_lock = None, 0.0, threading.Lock()
        self.pub = rospy.Publisher(P("command_topic", "/control_cmd"), CtrlCmd, queue_size=1, tcp_nodelay=True)
        self.debug = rospy.Publisher("~debug", String, queue_size=5)
        for cam in CAMERAS:
            rospy.Subscriber(P("camera_topic_" + cam, "/image/%s/compressed" % cam), CompressedImage,
                             lambda m, c=cam: self.inputs.add_camera(c, m.header.stamp.to_sec(), m.data),
                             queue_size=2, buff_size=1 << 22, tcp_nodelay=True)
        rospy.Subscriber(P("lidar_topic", "/lidar_bev/scan"), LidarBevScan, self.on_scan, queue_size=4, tcp_nodelay=True)
        rospy.Subscriber(P("status_topic", "/vehicle/competition_status"), CompetitionVehicleStatus, self.on_status,
                         queue_size=20, tcp_nodelay=True)
        rospy.Subscriber(P("localization_topic", "/localization/ego"), Odometry, self.on_ego, queue_size=20, tcp_nodelay=True)
        rospy.Subscriber(P("gps_topic", "/sensors/gps/fix"), NavSatFix, self.on_gps, queue_size=20, tcp_nodelay=True)
        self.gps_ok = True

    def on_scan(self, m):
        h = np.frombuffer(m.height, np.uint8).reshape(m.rows, m.cols)
        d = np.frombuffer(m.density, np.uint8).reshape(m.rows, m.cols)
        self.inputs.add_scan(m.header.stamp.to_sec(), h, d, (m.pose_x, m.pose_y, m.pose_yaw), m.odom_epoch)

    def on_status(self, m):
        t = m.header.stamp.to_sec()
        self.v = m.signed_velocity_mps
        self.odo.add(t, m.signed_velocity_mps)
        self.inputs.add_status(t, m.signed_velocity_mps, m.yaw_rad)

    def on_gps(self, m):
        self.gps_ok = not (m.status.status < 0 or (m.latitude == 0 and m.longitude == 0))   # blackout: NO_FIX or 0,0

    def on_ego(self, m):
        # While GPS is blacked out the EKF / tunnel_wall pose drifts along the road (23-30 m in run1023h-w1-007): route
        # progress then runs on the competition_status odometer from the last GPS-backed fix instead.
        if not self.gps_ok:
            return
        p = m.pose.pose
        self.tracker.update(m.header.stamp.to_sec(), p.position.x, p.position.y, yaw_of(p.orientation), self.odo.d)

    def publish(self, t):
        with self.plan_lock:
            plan = self.plan
        accel, brake, steering, info = self.ctrl.command(plan, t, self.v)
        cmd = CtrlCmd()
        cmd.longlCmdType, cmd.accel, cmd.brake, cmd.steering = 1, accel, brake, steering
        self.pub.publish(cmd)
        return info

    def control_loop(self, _event):
        self.publish(rospy.get_time())

    def infer_loop(self):
        period, n = 1.0 / self.rate, 0
        while not rospy.is_shutdown():
            t0 = rospy.get_time()
            s = self.tracker.s_at(self.odo.at(t0))
            if s is None:
                rospy.logwarn_throttle(5.0, "e2e_driver: no route progress yet (waiting for /localization/ego)")
                rospy.sleep(0.1); continue
            nav = self.route.nav_text(s)
            req, problems = self.inputs.request(t0, nav, lidar=self.lidar)
            if req is None:
                rospy.logwarn_throttle(5.0, "e2e_driver: inputs not ready: %s", problems)
                rospy.sleep(0.1); continue
            t_req = rospy.get_time()
            try:
                resp = self.client.call(req)
            except Exception as e:
                rospy.logerr_throttle(2.0, "e2e_driver: model server: %s", e)
                rospy.sleep(0.2); continue
            t_resp = rospy.get_time()
            plan = Plan(t0, resp["accel"], resp["kappa"], resp["v0"])
            with self.plan_lock:
                self.plan = plan
            info = self.publish(rospy.get_time())
            t_pub = rospy.get_time()
            n += 1
            self.debug.publish(String(data=json.dumps(dict(
                n=n, t0=t0, s=round(s, 2), nav=nav, problems=problems,
                latency_ms=round(1e3 * (t_pub - req["camera_newest"][0]), 1),
                camera_age_ms=[round(1e3 * (t0 - x), 1) for x in req["camera_newest"]],
                assemble_ms=round(1e3 * (t_req - t0), 1), server_ms=round(1e3 * (t_resp - t_req), 1),
                server=resp.get("timing", {}), v=round(self.v, 3), v0=round(float(resp["v0"]), 3),
                a0=round(float(plan.a[0]), 3), kappa0=round(float(plan.k[0]), 5), cmd=info))))
            rest = t0 + period - rospy.get_time()
            if rest > 0:
                time.sleep(rest)


def main():
    rospy.init_node("e2e_driver")
    node = Node()
    rospy.Timer(rospy.Duration(1.0 / node.control_rate), node.control_loop)
    threading.Thread(target=node.infer_loop, daemon=True).start()
    rospy.spin()


if __name__ == "__main__":
    main()
