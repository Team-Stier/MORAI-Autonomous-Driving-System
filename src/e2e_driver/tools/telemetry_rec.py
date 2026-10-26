#!/usr/bin/env python3
"""Dashboard telemetry for lap_video.py (Noetic chroot, rospy): applied values from /vehicle/competition_status and the
node's command /control_cmd, stamped with this host's wall clock (lap_video puts them on the video's tau through the
Mac's wall clock; the stack's own ROS time is the estimated tau, not the evaluation manager's).

    rosrun-less: python3 telemetry_rec.py OUT.csv          rows: wall,kind,speed_kph,steer_deg,accel,brake,yaw_rad
kind s = applied (status steering_angle_rad is right +, written left + in deg; yaw for the actual curvature),
c = command (steering x 40 deg).
"""
import math
import sys
import time

import rospy
from interfaces.msg import CompetitionVehicleStatus
from morai_msgs.msg import CtrlCmd

out = open(sys.argv[1], "a", buffering=1)
if out.tell() == 0:
    out.write("wall,kind,speed_kph,steer_deg,accel,brake,yaw_rad\n")


def status(m):
    out.write("%.3f,s,%.2f,%.2f,%.3f,%.3f,%.5f\n" % (time.time(), 3.6 * m.signed_velocity_mps, -math.degrees(m.steering_angle_rad),
                                                      m.accel, m.brake, m.yaw_rad))


def command(m):
    out.write("%.3f,c,,%.2f,%.3f,%.3f,\n" % (time.time(), 40.0 * m.steering, m.accel, m.brake))


rospy.init_node("telemetry_rec", anonymous=True)
rospy.Subscriber("/vehicle/competition_status", CompetitionVehicleStatus, status, queue_size=50)
rospy.Subscriber("/control_cmd", CtrlCmd, command, queue_size=50)
rospy.spin()
