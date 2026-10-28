#!/usr/bin/env python3
"""takeover.py watch PORT LOG N_S (shadow master) | pub PORT (stack master): the actor's start intervention (user
10/28: a policy that stays stopped where the expert would go only repeats the same rows, then the 60 s stuck return).
watch: once the car has stood (GT speed < 0.3 m/s) for N_S seconds (the longest red wait + 5 s) and the shadow
collection driver says go (its plan's speed > 0.5 m/s and fresh, no yield or AEB command in the last second; its
velocity_planner already zeroes the plan before a red stop line, and a light check of its own held a left turn on its
arrow: 10/28 run f, 900 s at CP6), its /shadow/control_cmd goes to `pub` over UDP until the car reaches 3 m/s, 5 s
pass or the go breaks; then a new full N_S stop is needed. A teleport (> 5 m jump) or a planner gap (> 1 s: the episode restart) resets the
clock. Each intervention is a line of LOG (JSON: t_start, t_end, wall clock; why it ended) for dagger_label.py.
pub: republishes the newest command (< 0.15 s old) on /morai_udp_bridge/control_cmd at 500 Hz: the bridge sends the
newest message at 50 Hz, so the speed filter's 20 Hz output rarely gets through (the collection AEB overrides alike).
Nothing reaches the car without both halves; shadow.sh starts them only with TAKEOVER=1."""
import json
import math
import socket
import sys
import time

V_STOP, V_DONE, MAX_S, QUIET_S = 0.3, 3.0, 5.0, 1.0


def watch(port, log, n_s):
    import rospy
    from interfaces.msg import Path
    from morai_msgs.msg import CtrlCmd
    from nav_msgs.msg import Odometry

    rospy.init_node("shadow_takeover_watch")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    st = dict(v=0.0, xy=None, stopped=None, plan=(0.0, -1e9), override=-1e9, on=None)

    def now():
        return time.time()

    def on_ego(m):
        p = m.pose.pose.position
        t, v = now(), abs(m.twist.twist.linear.x)
        if st["xy"] is not None and math.hypot(p.x - st["xy"][0], p.y - st["xy"][1]) > 5.0:
            st["stopped"] = None                           # teleport: an episode start
        st["xy"], st["v"] = (p.x, p.y), v
        if v >= V_STOP:
            st["stopped"] = None
        elif st["stopped"] is None:
            st["stopped"] = t

    def on_plan(m):
        t = now()
        if t - st["plan"][1] > 1.0:                        # planner restarted (episode start): count the stop anew
            st["stopped"] = t if st["v"] < V_STOP else None
        st["plan"] = (m.points[0].velocity if m.points else 0.0, t)

    def go():
        t = now()
        v_plan, t_plan = st["plan"]
        return v_plan > 0.5 and t - t_plan < 0.3 and t - st["override"] > QUIET_S

    def end(why):
        rec = dict(t_start=round(st["on"], 3), t_end=round(now(), 3), why=why, v_end=round(st["v"], 2))
        with open(log, "a") as f:
            f.write(json.dumps(rec) + "\n")
        print("takeover", rec, flush=True)
        st["on"], st["stopped"] = None, None

    def on_cmd(m):
        t = now()
        if st["on"] is None:
            if st["stopped"] is not None and t - st["stopped"] >= n_s and go():
                st["on"] = t
                print(f"takeover on after {t - st['stopped']:.1f} s stopped", flush=True)
            else:
                return
        if st["v"] >= V_DONE:
            return end("speed")
        if t - st["on"] > MAX_S:
            return end("time")
        if not go():
            return end("no_go")
        buf = __import__("io").BytesIO()
        m.serialize(buf)
        sock.sendto(buf.getvalue(), ("127.0.0.1", port))

    def on_override(m):
        st["override"] = now()

    rospy.Subscriber("/collect_only/ego_status", Odometry, on_ego, queue_size=10)
    rospy.Subscriber("/local_path/velocity_injected", Path, on_plan, queue_size=1)
    rospy.Subscriber("/shadow/control_cmd", CtrlCmd, on_cmd, queue_size=1)
    rospy.Subscriber("/shadow/yield_cmd", CtrlCmd, on_override, queue_size=1)
    rospy.Subscriber("/shadow/aeb_cmd", CtrlCmd, on_override, queue_size=1)
    rospy.spin()


def pub(port):
    import rospy
    from morai_msgs.msg import CtrlCmd

    rospy.init_node("shadow_takeover_pub")
    p = rospy.Publisher("/morai_udp_bridge/control_cmd", CtrlCmd, queue_size=1)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("127.0.0.1", port))
    sock.setblocking(False)
    last, t_last = None, -1e9
    while not rospy.is_shutdown():
        try:
            while True:
                data = sock.recv(4096)
                last, t_last = CtrlCmd().deserialize(data), time.time()
        except BlockingIOError:
            pass
        if last is not None and time.time() - t_last < 0.15:
            p.publish(last)
            time.sleep(0.002)
        else:
            time.sleep(0.01)


if __name__ == "__main__":
    if sys.argv[1] == "watch":
        watch(int(sys.argv[2]), sys.argv[3], float(sys.argv[4]))
    else:
        pub(int(sys.argv[2]))
