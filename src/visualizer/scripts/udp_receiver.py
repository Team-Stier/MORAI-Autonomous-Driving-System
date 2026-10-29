#!/usr/bin/env python3
"""UDP transport for the visualization input adapters."""
import socket
import struct


def decode_timestamp(packet, offset):
    if len(packet) < offset + 8:
        raise ValueError("missing UDP source timestamp")
    seconds, nanoseconds = struct.unpack_from("<II", packet, offset)
    if (seconds == 0 and nanoseconds == 0) or nanoseconds >= 1000000000:
        raise ValueError("invalid UDP source timestamp")
    return seconds, nanoseconds


def receive(default_port, handle, timestamp_offset):
    import rospy

    while not rospy.is_shutdown():
        try:
            bind_ip = rospy.get_param("~bind_ip", "0.0.0.0")
            port = int(rospy.get_param("~port", default_port))
            source_ip = rospy.get_param("~source_ip", "")
            if not 1 <= port <= 65535:
                raise ValueError("UDP port must be between 1 and 65535")
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                sock.bind((bind_ip, port))
                sock.settimeout(0.2)
                rospy.loginfo("Visualization UDP listening on %s:%d", bind_ip, port)
                while not rospy.is_shutdown():
                    try:
                        packet, source = sock.recvfrom(65535)
                    except socket.timeout:
                        continue
                    if source_ip and source[0] != source_ip:
                        continue
                    try:
                        # Source and ROS must share a clock; Pipeline rejects stale/future stamps.
                        handle(packet, rospy.Time(*decode_timestamp(packet, timestamp_offset)))
                    except Exception as error:
                        rospy.logwarn_throttle(5.0, "Visualization UDP packet rejected: %s", error)
        except Exception as error:
            rospy.logerr_throttle(5.0, "Visualization UDP retrying: %s", error)
            rospy.rostime.wallsleep(1.0)


def run(main):
    import time
    while True:
        try:
            main()
            return
        except Exception as error:
            try:
                import rospy
                rospy.logerr("Visualization input adapter retrying: %s", error)
            except Exception:
                print("Visualization input adapter retrying:", error)
            time.sleep(1.0)
