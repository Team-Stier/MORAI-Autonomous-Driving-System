#!/usr/bin/env python3
"""The model node: MORAI sensors (bridge, vlp16_bev, localization) -> model server -> /control_cmd (morai_msgs/CtrlCmd),
which speed_filter limits and forwards to the bridge (/morai_udp_bridge/control_cmd -> MORAI UDP 9093).

Inference loop (every 1/rate_hz s): t0 = now -> inputs.Inputs.request (training-identical picks, step-0 JPEGs, re-encoded
by per-camera worker threads as frames arrive) -> model server (TCP, e2e_driver.wire), up to max_inflight requests
outstanding, so the server prepares the next request while its model runs the current one. A receive thread turns each
reply into a Plan (64 x (a, kappa)); /control_cmd is published once as soon as a plan arrives and then at control_rate_hz
from the newest plan (control.Controller). /e2e_driver/debug (std_msgs/String, JSON) carries per-step timings:
latency_ms = command publish time - receipt stamp of the newest front-camera frame used.
Sync mode (~sync, tools/pod eval with MORAI synchronous mode): no loops; sync_clock_node.py sends each simulator step
and the node answers it (on_sync) with the command for that step and, on the 0.1 s grid, the request of that step.
"""
import json
import math
import socket
import threading
import time
from collections import deque

import rosnode
import rospy
from interfaces.msg import CompetitionVehicleStatus, LidarBevScan
from morai_msgs.msg import CtrlCmd
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage, NavSatFix
from std_msgs.msg import Header, String

import numpy as np

from e2e_driver.control import DEFAULTS, Controller, Plan
from e2e_driver.inputs import CAMERAS, Inputs
from e2e_driver.route import Odometer, PoseGuard, Route, RouteTracker, TeleportDetector
from e2e_driver import wire


def yaw_of(q):
    return float(np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))


class Node:
    def __init__(self):
        P = lambda k, d: rospy.get_param("~" + k, d)
        self.rate, self.control_rate = float(P("rate_hz", 10.0)), float(P("control_rate_hz", 50.0))
        self.lidar = bool(P("use_lidar", True))
        self.inputs = Inputs(prefetch=True)
        self.route = Route(P("route_asset", ""))
        self.tracker, self.odo = RouteTracker(self.route), Odometer()
        self.ctrl = Controller(**{k: P("control/" + k, v) for k, v in DEFAULTS.items()})
        self.server = (P("server_host", "127.0.0.1"), int(P("server_port", 47011)))
        self.max_inflight = int(P("max_inflight", 2))
        # eval below real time: a plan observed at t0 drives from t0 + apply_delay_s on tau, never earlier (the 1x one-step
        # latency); a reply after that is applied on arrival and counted late. 0 (the competition): on arrival.
        self.apply_delay, self.late = float(P("apply_delay_s", 0.0)), 0
        self.follow_m, self.pose = float(P("fake_follow_route_m", 0.0)), None    # > 0: route_kappa for the fake backend
        self.expert2_zone = [float(x) for x in P("expert2_zone", [])]              # [s0, s1): the server's 2nd expert set
        self.world = int(P("world", 0))                                               # RL actor: which simulator (dump dir)
        # synchronous evaluation (MORAI sync mode, sync_clock_node.py): no timers; each simulator step arrives on ~sync
        # and is acked on ~sync_ack once its command is out (on_sync)
        self.sync, self.sync_wait = bool(P("sync", False)), float(P("sync_wait_s", 0.5))
        self.status_msg, self.n_status, self.ready, self.cv = None, 0, [], threading.Condition()
        self.guard = PoseGuard()                                                  # localization guard (on_ego)
        self.teleport, self.restart_loc = TeleportDetector(), bool(P("restart_localization_on_teleport", True))
        self.teleport_pub = rospy.Publisher("~teleport", Header, queue_size=2)       # vlp16_bev, sim_clock reset on it
        self.sock, self.pending, self.next_id, self.skipped = None, {}, 0, 0
        self.reset_vit = False                      # the next request tells the server to drop its ViT cache
        self.plan, self.plan_id, self.v, self.plan_lock = None, -1, 0.0, threading.Lock()
        self.applied = deque(maxlen=200)                # (t, accel, brake, steer deg left +) from competition_status (mlp mode)
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

    def teleported(self, t, why):
        """a checkpoint return or scene reload seen in the inputs (the stack is not told): start over from standstill.
        Buffered frames, scans and ego history, route progress and the plan belong to the old place; localization is
        restarted (respawn) from the next GPS fix; vlp16_bev and sim_clock reset on ~teleport."""
        rospy.logwarn("e2e_driver: teleport (%s): inputs, route progress and plan reset", why)
        self.inputs.reset()
        self.reset_vit = True
        self.tracker, self.odo, self.guard = RouteTracker(self.route), Odometer(), PoseGuard()
        with self.plan_lock:
            self.plan, self.plan_id = None, self.next_id                       # replies to earlier requests are dropped
        self.teleport_pub.publish(Header(stamp=rospy.Time.from_sec(t), frame_id=why))
        if self.restart_loc:
            threading.Thread(target=rosnode.kill_nodes, args=(["/localization_node"],), daemon=True).start()

    def on_status(self, m):
        if self.sync:                   # the step applies the newest: status flows at wall rate, some sent before the tick ended
            self.status_msg, self.n_status = m, self.n_status + 1
            return
        self.apply_status(m)

    def apply_status(self, m):
        t = m.header.stamp.to_sec()
        if self.teleport.status(t, m.yaw_rad, m.signed_velocity_mps):
            self.teleported(t, "heading jump")
        self.v = m.signed_velocity_mps
        self.applied.append((t, m.accel, m.brake, -math.degrees(m.steering_angle_rad)))
        self.odo.add(t, m.signed_velocity_mps)
        self.inputs.add_status(t, m.signed_velocity_mps, m.yaw_rad)

    def on_gps(self, m):
        self.gps_ok = not (m.status.status < 0 or (m.latitude == 0 and m.longitude == 0))   # blackout: NO_FIX or 0,0
        if self.teleport.gps(m.header.stamp.to_sec(), m.latitude if self.gps_ok else None, m.longitude):
            self.teleported(m.header.stamp.to_sec(), "GPS jump")

    def on_ego(self, m):
        # While GPS is blacked out the EKF / tunnel_wall pose drifts along the road (23-30 m in run1023h-w1-007): route
        # progress then runs on the competition_status odometer from the last GPS-backed fix instead.
        p = m.pose.pose
        t, x, y, yaw = m.header.stamp.to_sec(), p.position.x, p.position.y, yaw_of(p.orientation)
        if not self.guard.ok(t, x, y, yaw, self.v):
            return
        self.pose = (x, y, yaw)
        if not self.gps_ok:
            return
        self.tracker.update(m.header.stamp.to_sec(), p.position.x, p.position.y, yaw_of(p.orientation), self.odo.d)

    def publish(self, t):
        with self.plan_lock:
            plan = self.plan
        accel, brake, steering, info = self.ctrl.command(plan, t, self.v, self.applied_at(t) if self.ctrl.mlp is not None else None)
        cmd = CtrlCmd()
        cmd.longlCmdType, cmd.accel, cmd.brake, cmd.steering = 1, accel, brake, steering
        self.pub.publish(cmd)
        return info

    def applied_at(self, t):
        """the applied (accel, brake, steer deg) at t - 0.1 s and t (newest status at or before each), for mode mlp."""
        h = list(self.applied)
        if not h:
            return np.zeros(6)
        pick = lambda tt: next((r[1:] for r in reversed(h) if r[0] <= tt), h[0][1:])
        return np.r_[pick(t - 0.1), pick(t)]

    def control_loop(self, _event):
        self.publish(rospy.get_time())

    def connect(self):
        s = socket.create_connection(self.server, timeout=5.0)
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        s.settimeout(None)
        self.pending.clear()
        threading.Thread(target=self.receive_loop, args=(s,), daemon=True).start()
        self.sock = s

    def receive_loop(self, sock):
        try:
            while True:
                self.on_reply(wire.recv(sock), rospy.get_time())
        except Exception as e:
            rospy.logerr("e2e_driver: model server connection: %s", e)
        if self.sock is sock:
            self.sock = None
            self.pending.clear()           # unanswered on a dead connection: else max_inflight blocks the reconnect (10/27)
            with self.cv:
                self.cv.notify_all()

    def on_reply(self, resp, t_resp):
        meta = self.pending.pop(resp.get("id"), None)
        if meta is None:
            return
        plan = Plan(meta["t0"], resp["accel"], resp["kappa"], resp["v0"])
        if self.sync:                                   # on_sync activates it at t0 + apply_delay: never late
            with self.cv:
                self.ready.append((resp, meta, plan)); self.cv.notify_all()
            return
        t_apply = meta["t0"] + self.apply_delay
        late = self.apply_delay > 0 and t_resp > t_apply
        self.late += late
        if self.apply_delay > 0 and not late:
            rospy.Timer(rospy.Duration(max(t_apply - rospy.get_time(), 1e-3)),
                        lambda _e: self.activate(resp, meta, plan, late), oneshot=True)
        else:
            self.activate(resp, meta, plan, late)

    def activate(self, resp, meta, plan, late, t=None):
        with self.plan_lock:
            if resp["id"] < self.plan_id:
                return False                            # an older request answered late: keep the newer plan
            self.plan, self.plan_id = plan, resp["id"]
        info = self.publish(rospy.get_time() if t is None else t)
        t_pub = rospy.get_time() if t is None else t
        self.debug.publish(String(data=json.dumps(dict(
            n=resp["id"], t0=meta["t0"], s=round(meta["s"], 2), nav=meta["nav"], problems=meta["problems"],
            latency_ms=round(1e3 * (t_pub - meta["camera_newest"][0]), 1),
            camera_age_ms=[round(1e3 * (meta["t0"] - x), 1) for x in meta["camera_newest"]],
            assemble_ms=round(1e3 * (meta["w_req"] - meta["w0"]), 1), server_ms=round(1e3 * (time.perf_counter() - meta["w_req"]), 1),
            server=resp.get("timing", {}), pick=meta["pick"], inflight=meta["inflight"], skipped=self.skipped, late=late, late_total=self.late, loc_rejected=self.guard.rejected, v=round(self.v, 3),
            v0=round(float(resp["v0"]), 3), expert=resp.get("expert", 0), a0=round(float(plan.a[0]), 3), kappa0=round(float(plan.k[0]), 5), cmd=info,
            plan_a=[round(float(x), 3) for x in plan.a], plan_k=[round(float(x), 5) for x in plan.k],   # the 64 steps, for path checks
            pose=[round(float(x), 3) for x in meta["pose"]] if meta["pose"] else None))))          # localization x, y, yaw at t0
        return True

    def infer_loop(self):
        period = 1.0 / self.rate
        next_t = rospy.get_time()
        while not rospy.is_shutdown():
            rest = next_t - rospy.get_time()
            if rest > 0:
                rospy.sleep(rest)                               # on tau when /use_sim_time: 10 Hz of simulation time
            next_t = max(next_t + period, rospy.get_time() - period)        # do not burst after a stall
            self.request(rospy.get_time())

    def request(self, t0):
        """one model request on the inputs at t0 (tau) -> its id, or None (no route progress, server behind, inputs not
        ready, server unreachable)."""
        w0, t_wall = time.perf_counter(), time.time()                       # w0 wall for timings
        s = self.tracker.s_at(self.odo.at(t0))
        if s is None:
            rospy.logwarn_throttle(5.0, "e2e_driver: no route progress yet (waiting for /localization/ego)")
            return None
        if len(self.pending) >= self.max_inflight:
            self.skipped += 1                       # the server is behind: skip this tick rather than queue up
            return None
        nav = self.route.nav_text(s)
        req, problems = self.inputs.request(t0, nav, lidar=self.lidar)
        if req is None:
            rospy.logwarn_throttle(5.0, "e2e_driver: inputs not ready: %s", problems)
            return None
        if self.follow_m > 0 and self.pose is not None:
            req["route_kappa"] = float(self.route.pursuit_kappa(s, *self.pose, self.follow_m))
        req["t_wall"], req["world"] = t_wall, self.world   # host clock at t0 (rl_reward joins on it), simulator
        if self.expert2_zone and self.expert2_zone[0] <= s < self.expert2_zone[1]:
            req["expert"] = 1                       # RL_PLAN 5 A: the section expert (model_server --expert2)
        try:
            if self.sock is None:
                self.connect()
            rid, self.next_id = self.next_id, self.next_id + 1
            req["id"] = rid
            pick = req.pop("pick", None)                    # frame selection stats: debug only, not sent
            if self.reset_vit:
                req["reset"], self.reset_vit = True, False
            self.pending[rid] = dict(t0=t0, s=s, nav=nav, problems=problems, camera_newest=req["camera_newest"], pick=pick, pose=self.pose,
                                     w0=w0, w_req=time.perf_counter(), inflight=len(self.pending))
            wire.send(self.sock, req)
            return rid
        except Exception as e:
            rospy.logerr_throttle(2.0, "e2e_driver: model server: %s", e)
            self.sock = None
            (time.sleep if self.sync else rospy.sleep)(0.2)          # the sync clock stands still until the ack
            return None

    def on_sync(self, m):
        """one simulator step at t = m.stamp (sync mode; frame_id hold, or decide | decide_serial on the 0.1 s grid). sync_clock_node set
        /clock to t before the tick, so the bridge stamped the tick's sensors t. Wait (sync_wait_s at most) until every
        camera that sends has a frame of t and a status arrived after the tick, apply that status, activate the plans due
        by t (waiting for their replies: a plan observed at t0 drives from t0 + apply_delay_s, never late), publish the
        command for t once, send the request of t when deciding (serial: and wait for its reply), then ack."""
        t, n0, w0 = m.stamp.to_sec(), self.n_status, time.perf_counter()
        while time.perf_counter() - w0 < self.sync_wait and (self.n_status == n0 or any(
                self.inputs.latest(c) is not None and self.inputs.latest(c) < t - 1e-4 for c in CAMERAS)):
            time.sleep(0.001)
        w1 = time.perf_counter()
        stale = [c for c in CAMERAS if self.inputs.latest(c) is not None and self.inputs.latest(c) < t - 1e-4] + (["status"] if self.n_status == n0 else [])
        if self.status_msg is not None:
            self.apply_status(self.status_msg)
        due = lambda r: r["t0"] + self.apply_delay <= t + 1e-6
        with self.cv:
            waited = self.cv.wait_for(lambda: not any(due(r) for r in list(self.pending.values())), timeout=30.0)
            ready, self.ready = [r for r in self.ready if due(r[1])], [r for r in self.ready if not due(r[1])]
        w2 = time.perf_counter()
        if not any([self.activate(resp, meta, plan, False, t) for resp, meta, plan in sorted(ready, key=lambda r: r[0]["id"])]):
            self.publish(t)
        rid = self.request(t) if m.frame_id.startswith("decide") else None
        w3 = time.perf_counter()
        if rid is not None and m.frame_id == "decide_serial":
            with self.cv:
                waited = self.cv.wait_for(lambda: rid not in self.pending, timeout=30.0) and waited
        ms = lambda a, b: round(1e3 * (b - a), 1)
        self.sync_ack.publish(String(data=json.dumps(dict(
            t=t, rid=rid, plans=len(ready), stale=stale, timeout=not waited, sensors_ms=ms(w0, w1), plans_ms=ms(w1, w2),
            request_ms=ms(w2, w3), reply_ms=ms(w3, time.perf_counter())))))


def main():
    rospy.init_node("e2e_driver")
    node = Node()
    if node.sync:
        node.sync_ack = rospy.Publisher("~sync_ack", String, queue_size=5, tcp_nodelay=True)
        rospy.Subscriber("~sync", Header, node.on_sync, queue_size=1, tcp_nodelay=True)
    else:
        rospy.Timer(rospy.Duration(1.0 / node.control_rate), node.control_loop)
        threading.Thread(target=node.infer_loop, daemon=True).start()
    rospy.spin()


if __name__ == "__main__":
    main()
