"""speed_filter start-assist check (roscore + speed_filter running): a stopped car (competition_status speed 0) holding
/control_cmd accel 0, brake 0.5 for DURATION s; reports whether /morai_udp_bridge/control_cmd ever pushed the accel
(main's assist: accel 0.2, steering 0 for 1 s after stop_tolerance_sec stopped).
Usage: python3 start_assist_check.py [DURATION_S]"""
import json, sys

import rospy
from interfaces.msg import CompetitionVehicleStatus
from morai_msgs.msg import CtrlCmd


def main():
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 40.0
    rospy.init_node("start_assist_check")
    st = rospy.Publisher("/vehicle/competition_status", CompetitionVehicleStatus, queue_size=5)
    cmd = rospy.Publisher("/control_cmd", CtrlCmd, queue_size=5)
    out = []
    rospy.Subscriber("/morai_udp_bridge/control_cmd", CtrlCmd, lambda m: out.append((rospy.get_time(), m.accel, m.brake)), queue_size=100)
    rospy.sleep(1.0)
    r, t0 = rospy.Rate(50), rospy.get_time()
    while not rospy.is_shutdown() and rospy.get_time() - t0 < dur:
        s = CompetitionVehicleStatus(); s.header.stamp, s.header.frame_id = rospy.Time.now(), "base_link"
        s.control_mode, s.gear, s.signed_velocity_mps, s.velocity_x_mps = 2, 4, 0.0, 0.0
        st.publish(s)
        cmd.publish(CtrlCmd(longlCmdType=1, accel=0.0, brake=0.5, steering=0.0))
        r.sleep()
    pushed = [(round(t - t0, 2), a) for t, a, b in out if a > 0]
    print(json.dumps(dict(seconds=dur, outputs=len(out), accel_pushed=len(pushed), first_push_s=pushed[0][0] if pushed else None,
                          max_accel=max([a for _, a, _ in out], default=None))))


if __name__ == "__main__":
    main()
