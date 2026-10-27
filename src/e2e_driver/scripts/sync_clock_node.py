#!/usr/bin/env python3
"""/clock for a stack whose MORAI world runs in synchronous mode (tools/pod eval with SYNC=1, in place of
sim_clock_node): the world advances only when its evaluation manager ticks it (eval_manager.py --sync), so simulation
time is exact and this node owns it. The manager connects to TCP 127.0.0.1:~port and sends one JSON line per simulator
step, {"dt": s, "decide": bool, "serial": bool}; at the current time t this node then
  1. publishes /e2e_driver/sync (Header stamp t, frame_id hold | decide | decide_serial): the e2e node's step (sensors
     of t in, due plans applied, the command for t published, the request of t sent when deciding and, serial, its
     reply awaited; e2e_driver_node on_sync);
  2. waits for its ~sync_ack, then for that command through speed_filter, then ~tx_wait_s more (the bridge sends the
     newest command every 1 / send_rate_hz: MORAI must have it before the tick);
  3. publishes /clock = t + dt: everything received during the tick is stamped with the time the tick ends at;
  4. replies {"t": t, "ack": <the node's ack>, "tx_ms": ..., "ack_timeout": bool}; the manager ticks.
/clock starts at the host clock rounded to 0.1 s and is republished at 5 Hz (use_sim_time nodes that start later need
it). ~state = [wall, tau, 1, steps] per step, as sim_clock_node's (eval3.sh records /sim_clock/state).
"""
import json
import socket
import threading
import time

import rospy
from morai_msgs.msg import CtrlCmd
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float64MultiArray, Header, String


def main():
    rospy.init_node("sim_clock")                       # sim_clock_node's name: ~state is /sim_clock/state either way
    port, tx_wait = int(rospy.get_param("~port")), float(rospy.get_param("~tx_wait_s", 0.006))
    ack_timeout = float(rospy.get_param("~ack_timeout_s", 60.0))
    clock_pub = rospy.Publisher("/clock", Clock, queue_size=1)
    state = rospy.Publisher("~state", Float64MultiArray, queue_size=5)
    step_pub = rospy.Publisher("/e2e_driver/sync", Header, queue_size=1, tcp_nodelay=True)
    cv, lock = threading.Condition(), threading.Lock()
    s = dict(t=round(time.time(), 1), k=0, ack=None, tx=0)

    def set_clock(t):
        with lock:                                     # the 5 Hz republisher never sends an older value after a newer one
            s["t"] = t
            clock_pub.publish(Clock(clock=rospy.Time.from_sec(t)))

    def on_ack(m):
        with cv:
            s["ack"] = json.loads(m.data); cv.notify_all()

    def on_tx(_m):
        with cv:
            s["tx"] += 1; cv.notify_all()
    rospy.Subscriber("/e2e_driver/sync_ack", String, on_ack, queue_size=5, tcp_nodelay=True)
    rospy.Subscriber("/morai_udp_bridge/control_cmd", CtrlCmd, on_tx, queue_size=5, tcp_nodelay=True)

    def republish():
        while not rospy.is_shutdown():
            set_clock(s["t"]); time.sleep(0.2)
    threading.Thread(target=republish, daemon=True).start()

    def step(dt, decide, serial):
        t = s["t"]
        with cv:
            s["ack"], tx0 = None, s["tx"]
        w0 = time.perf_counter()
        step_pub.publish(Header(stamp=rospy.Time.from_sec(t), frame_id=("decide_serial" if serial else "decide") if decide else "hold"))
        with cv:
            acked = cv.wait_for(lambda: s["ack"] is not None and abs(s["ack"]["t"] - t) < 1e-6, timeout=ack_timeout)
            w1 = time.perf_counter()
            sent = acked and cv.wait_for(lambda: s["tx"] > tx0, timeout=0.5)
            ack = s["ack"]
        if sent:
            time.sleep(tx_wait)
        set_clock(round(t + dt, 6))
        s["k"] += 1
        state.publish(Float64MultiArray(data=[time.time(), t, 1.0, s["k"]]))
        return dict(t=t, ack=ack, ack_timeout=not acked, tx_timeout=acked and not sent,
                    ack_ms=round(1e3 * (w1 - w0), 1), tx_ms=round(1e3 * (time.perf_counter() - w1), 1))

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", port)); srv.listen(1)
    rospy.loginfo("sync_clock: tau %.1f, manager port %d", s["t"], port)
    while not rospy.is_shutdown():
        conn, _ = srv.accept()
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        f = conn.makefile("rw")
        rospy.loginfo("sync_clock: manager connected at tau %.1f", s["t"])
        try:
            for line in f:
                q = json.loads(line)
                f.write(json.dumps(step(float(q["dt"]), bool(q.get("decide")), bool(q.get("serial")))) + "\n"); f.flush()
        except (OSError, ValueError) as e:
            rospy.logwarn("sync_clock: manager connection: %s", e)
        conn.close()


if __name__ == "__main__":
    main()
