#!/usr/bin/env python3
import math
import struct


def decode_ego(packet):
    # 25.S4 normal Ego: recorded in spec/ioniq5_bbox.meta.json. Competition has no GT position.
    if (len(packet) != 229 or packet[:11] != b"#MoraiInfo$"
            or struct.unpack_from("<I", packet, 11)[0] != 200 or packet[-2:] != b"\r\n"):
        raise ValueError("expected 229-byte normal Ego status (payload 200), not competition status")
    values = struct.unpack_from("<6f", packet, 77)  # position XYZ, roll/pitch/yaw in degrees
    if not all(math.isfinite(value) for value in values):
        raise ValueError("non-finite Ego position or orientation")
    return values


def main():
    import rospy
    from nav_msgs.msg import Odometry
    from tf.transformations import quaternion_from_euler

    from udp_receiver import receive
    if not rospy.core.is_initialized():
        rospy.init_node("visualizer_ego_udp")
    yaw_offset_rad = float(rospy.get_param("~yaw_offset_rad", -0.023509))
    offset = [float(rospy.get_param("~offset_" + axis + "_m", 0.0)) for axis in "xyz"]
    if not all(math.isfinite(v) for v in offset + [yaw_offset_rad]):
        raise ValueError("invalid GT calibration")
    publisher = rospy.Publisher("/visualizer/gt", Odometry, queue_size=1)

    def handle(packet, stamp):
        x, y, z, roll, pitch, yaw = decode_ego(packet)
        message = Odometry()
        message.header.stamp = stamp
        message.header.frame_id = "map"
        message.child_frame_id = "ego_gt"
        position = message.pose.pose.position
        position.x, position.y, position.z = x + offset[0], y + offset[1], z + offset[2]
        q = quaternion_from_euler(math.radians(roll), math.radians(pitch),
                                  math.radians(yaw) + yaw_offset_rad)
        orientation = message.pose.pose.orientation
        orientation.x, orientation.y, orientation.z, orientation.w = q
        publisher.publish(message)

    receive(9095, handle, 27)


if __name__ == "__main__":
    from udp_receiver import run
    run(main)
