#!/usr/bin/env python3
"""/velodyne_packets (velodyne_msgs/VelodyneScan from velodyne_driver, as in collection) -> /lidar_bev/scan
(interfaces/LidarBevScan), the model's per-scan LiDAR BEV. Each scan is rasterised with the training code
(vlp16_bev/lidar_bev.py, a verbatim copy of Alpamayo-Cosmos train/morai_vla/lidar_bev.py); see vlp16_bev/scan.py for the
motion signals and why they match the training data.
"""
import numpy as np
import rospy
from interfaces.msg import CompetitionVehicleStatus, LidarBevScan
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from velodyne_msgs.msg import VelodyneScan

from vlp16_bev.scan import BevBuilder, GridYawRate, SignalTrack


def main():
    rospy.init_node("vlp16_bev")
    max_signal_age = float(rospy.get_param("~max_signal_age_sec", 0.5))
    # grid: the training recipe on competition_status yaw (held ~0.3 s behind live); imu: IMU wz.
    # grid_trend: the same recipe with the missing last ~0.3 s of yaw extended by a line through the last
    # ~yaw_trend_window_sec of yaw (README: grid vs grid_trend vs imu).
    src = rospy.get_param("~yaw_rate_source", "grid")
    imu = src == "imu"
    trend = float(rospy.get_param("~yaw_trend_window_sec", 0.3)) if src == "grid_trend" else 0.0
    new_tracks = lambda: (SignalTrack(), SignalTrack() if imu else GridYawRate(trend_window=trend))
    tracks = list(new_tracks())                                   # [v, r]; replaced on a teleport

    def on_status(m):
        t = m.header.stamp.to_sec()
        tracks[0].add(t, m.signed_velocity_mps)
        if not imu: tracks[1].add(t, m.yaw_rad)
    if imu:
        rospy.Subscriber("/sensors/imu/data", Imu, lambda m: tracks[1].add(m.header.stamp.to_sec(), m.angular_velocity.z),
                         queue_size=50, tcp_nodelay=True)
    rospy.Subscriber(rospy.get_param("~status_topic", "/vehicle/competition_status"), CompetitionVehicleStatus, on_status,
                     queue_size=50, tcp_nodelay=True)
    pub = rospy.Publisher(rospy.get_param("~topic", "/lidar_bev/scan"), LidarBevScan, queue_size=4)
    bev = BevBuilder()

    def on_teleport(_m):                                          # e2e_driver saw the Ego moved in one step
        tracks[:] = new_tracks(); bev.reset()                    # yaw/speed history and odometry: a new drive
    rospy.Subscriber("/e2e_driver/teleport", Header, on_teleport, queue_size=2)

    def on_scan(scan):
        t = np.array([p.stamp.to_nsec() for p in scan.packets], np.int64) * 1e-9
        v_track, r_track = tracks
        if not (v_track.ready() and r_track.ready()) or t[-1] - v_track.latest() > max_signal_age:
            rospy.logwarn_throttle(5.0, "vlp16_bev: no fresh competition_status, scan skipped")
            return
        h, d, n, pose = bev.scan(b"".join(bytes(p.data) for p in scan.packets), t, v_track.at(t), r_track.at(t))
        m = LidarBevScan()
        m.header.stamp, m.header.frame_id = scan.header.stamp, "base_link"
        m.rows, m.cols = h.shape
        m.height, m.density = h.tobytes(), d.tobytes()
        m.pose_x, m.pose_y, m.pose_yaw = (float(p) for p in pose)
        m.npts, m.npackets, m.odom_epoch = int(n), len(t), bev.epoch
        pub.publish(m)
    rospy.Subscriber(rospy.get_param("~packets_topic", "/velodyne_packets"), VelodyneScan, on_scan, queue_size=4,
                     buff_size=1 << 22, tcp_nodelay=True)
    rospy.spin()


if __name__ == "__main__":
    main()
