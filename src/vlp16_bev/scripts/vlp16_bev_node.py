#!/usr/bin/env python3
"""VLP16 UDP packets from MORAI -> /lidar_bev/scan (interfaces/LidarBevScan), replacing velodyne_driver + pointcloud.

A receive thread only reads and stamps datagrams (receipt time, ROS clock), so processing never delays a stamp; a worker
thread groups them into 76-packet scans and rasterises each with the training code (vlp16_bev/lidar_bev.py, a verbatim
copy of Alpamayo-Cosmos train/morai_vla/lidar_bev.py). See vlp16_bev/scan.py for what matches the training data and why.
"""
import queue
import socket
import threading

import numpy as np
import rospy
from interfaces.msg import CompetitionVehicleStatus, LidarBevScan
from sensor_msgs.msg import Imu

from vlp16_bev.scan import BevBuilder, ScanAssembler, SignalTrack


def main():
    rospy.init_node("vlp16_bev")
    bind_ip = rospy.get_param("~bind_ip", "0.0.0.0")
    port = int(rospy.get_param("~port", 2368))
    topic = rospy.get_param("~topic", "/lidar_bev/scan")
    rcvbuf = int(rospy.get_param("~receive_buffer_bytes", 4 << 20))
    max_signal_age = float(rospy.get_param("~max_signal_age_sec", 0.5))

    v_track, r_track = SignalTrack(), SignalTrack()
    rospy.Subscriber(rospy.get_param("~status_topic", "/vehicle/competition_status"), CompetitionVehicleStatus,
                     lambda m: v_track.add(m.header.stamp.to_sec(), m.signed_velocity_mps), queue_size=50, tcp_nodelay=True)
    rospy.Subscriber(rospy.get_param("~imu_topic", "/sensors/imu/data"), Imu,
                     lambda m: r_track.add(m.header.stamp.to_sec(), m.angular_velocity.z), queue_size=50, tcp_nodelay=True)
    pub = rospy.Publisher(topic, LidarBevScan, queue_size=4)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, rcvbuf)
    sock.bind((bind_ip, port))
    sock.settimeout(0.5)
    rospy.loginfo("vlp16_bev: UDP %s:%d -> %s (SO_RCVBUF %d)", bind_ip, port, topic,
                  sock.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF))

    q = queue.Queue(maxsize=4096)

    def receive():
        while not rospy.is_shutdown():
            try:
                data = sock.recv(2048)
            except socket.timeout:
                continue
            try:
                q.put_nowait((data, rospy.get_time()))
            except queue.Full:
                rospy.logwarn_throttle(5.0, "vlp16_bev: packet queue full, dropping")

    threading.Thread(target=receive, daemon=True).start()
    asm, bev = ScanAssembler(), BevBuilder()
    while not rospy.is_shutdown():
        try:
            data, stamp = q.get(timeout=0.5)
        except queue.Empty:
            continue
        out = asm.add(data, stamp)
        if out is None:
            continue
        raw, t = out
        if not (v_track.ready() and r_track.ready()) or t[-1] - min(v_track.latest(), r_track.latest()) > max_signal_age:
            rospy.logwarn_throttle(5.0, "vlp16_bev: no fresh competition_status / IMU, scan skipped")
            continue
        h, d, n, pose = bev.scan(raw, t, v_track.at(t), r_track.at(t))
        m = LidarBevScan()
        m.header.stamp, m.header.frame_id = rospy.Time.from_sec(float(t[-1])), "base_link"
        m.rows, m.cols = h.shape
        m.height, m.density = h.tobytes(), d.tobytes()
        m.pose_x, m.pose_y, m.pose_yaw = (float(p) for p in pose)
        m.npts, m.npackets, m.odom_epoch = int(n), len(t), bev.epoch
        pub.publish(m)


if __name__ == "__main__":
    main()
