#!/usr/bin/env python3
"""/clock = the estimated simulator time tau (e2e_driver/sim_clock.py) for a stack running with /use_sim_time true
(run.sh starts it first when E2E_SIM_CLOCK=1). Every node then stamps and times on tau: the bridge's receipt stamps,
velodyne packet stamps, localization and speed_filter, the e2e node's frame picks, history grid, time text and plan.
Inputs are taken on their wall-clock receipt (time.time()): /vehicle/competition_status speed and /sensors/gps/fix.
~state = [wall, tau, rho, rho updates] at 10 Hz (drive_summary maps tau back to the wall clock with it). rho is kept in
~/.ros/sim_clock_rho for the next start: the rs2041 spawn reaches its GPS blackout before 5 s of driving."""
import os
import threading
import time

import rospy
from interfaces.msg import CompetitionVehicleStatus
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64MultiArray

from e2e_driver.sim_clock import SimClock

RHO_FILE = os.path.expanduser("~/.ros/sim_clock_rho")


def main():
    rospy.init_node("sim_clock")
    try:
        rho0 = float(open(RHO_FILE).read())
    except (OSError, ValueError):
        rho0 = 0.8                                    # training's prior (load_episode fillna)
    clock, lock = SimClock(rho0=rho0), threading.Lock()

    def on_status(m):
        with lock:
            clock.add_status(time.time(), m.signed_velocity_mps)

    def on_gps(m):
        ok = m.status.status >= 0
        with lock:
            clock.add_gps(time.time(), m.latitude if ok else None, m.longitude if ok else None)

    pub = rospy.Publisher("/clock", Clock, queue_size=1)
    state = rospy.Publisher("~state", Float64MultiArray, queue_size=5)
    rospy.Subscriber("/vehicle/competition_status", CompetitionVehicleStatus, on_status, queue_size=50, tcp_nodelay=True)
    rospy.Subscriber("/sensors/gps/fix", NavSatFix, on_gps, queue_size=50, tcp_nodelay=True)
    rospy.loginfo("sim_clock: rho0 %.3f", rho0)
    n, saved = 0, None
    while not rospy.is_shutdown():
        t = time.time()
        with lock:
            tau, rho, est = clock.advance(t), clock.rho, clock.estimated
        pub.publish(Clock(clock=rospy.Time.from_sec(tau)))
        n += 1
        if n % 20 == 0:
            state.publish(Float64MultiArray(data=[t, tau, rho, est]))
        if n % 2000 == 0 and est and rho != saved:
            try:
                open(RHO_FILE, "w").write("%.4f" % rho); saved = rho
            except OSError:
                pass
        time.sleep(0.005)                             # ~200 Hz: 5 ms stamp resolution on tau


if __name__ == "__main__":
    main()
