#!/usr/bin/env python3
"""Temporary Object Info recorder. Publishes /ground_truth/* only.

Wire fields follow MORAI-DriveExample_UDP's ObjectInfoReceiver and the local
24.R2 reference adapter. Position is the simulator reference point; the
interfaces box center is inferred from the vehicle specifications. Raw packets
are retained so timestamp, units and reference offsets can be checked later.
"""
import math
import socket
import struct
import time


def decode_packet(packet):
    if (len(packet) not in (1400, 2160) or packet[:14] != b"#MoraiObjInfo$"
            or packet[-2:] != b"\r\n"):
        raise ValueError("expected timestamped Object Info with 20 x 68/106 byte records")
    declared = struct.unpack_from("<I", packet, 14)[0]
    if declared not in (len(packet) - 40, len(packet) - 32):
        raise ValueError("unsupported Object Info declared length")
    seconds, nanoseconds = struct.unpack_from("<II", packet, 30)
    if (seconds == 0 and nanoseconds == 0) or nanoseconds >= 1000000000:
        raise ValueError("invalid source timestamp")
    stride = (len(packet) - 40) // 20
    records, seen = [], set()
    for index in range(20):
        record = struct.unpack_from("<hh16f", packet, 38 + index * stride)
        if record[0] == 0 or record[1] == -1:
            continue
        if (record[1] not in (0, 1, 2) or not all(math.isfinite(v) for v in record[2:])
                or min(record[6:9]) <= 0 or record[:2] in seen):
            raise ValueError("invalid or duplicate Object Info object")
        seen.add(record[:2])
        records.append(record)
    return seconds, nanoseconds, records


def messages(records, stamp):
    from interfaces.msg import Object, Objects
    from morai_msgs.msg import ObjectStatus, ObjectStatusList
    converted, native = Objects(), ObjectStatusList()
    for message in (converted, native):
        message.header.frame_id, message.header.stamp = "map", stamp
    for row in records:
        sim_id, kind, x, y, z, heading, length, width, height = row[:9]
        yaw = math.radians(heading)
        offset = (row[9] + row[10] - row[11]) / 2 if kind == 1 else 0
        obj = Object()
        obj.id = (kind << 16) | (sim_id & 0xffff)
        obj.label = {0: "pedestrian", 1: "vehicle", 2: "obstacle"}[kind]
        obj.position.x = x + offset * math.cos(yaw)
        obj.position.y = y + offset * math.sin(yaw)
        obj.position.z = z + height / 2
        obj.bbox_pose.position = obj.position
        obj.bbox_pose.orientation.z, obj.bbox_pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        obj.bbox_size.x, obj.bbox_size.y, obj.bbox_size.z = length, width, height
        # UDP vectors use each object's body axes, not map axes. Keep native
        # values below and rotate the interfaces vectors by the GT heading.
        c, s = math.cos(yaw), math.sin(yaw)
        vx, vy, vz = (v / 3.6 for v in row[12:15])
        ax, ay, az = row[15:18]
        obj.velocity.x, obj.velocity.y, obj.velocity.z = c*vx-s*vy, s*vx+c*vy, vz
        obj.acceleration.x, obj.acceleration.y, obj.acceleration.z = c*ax-s*ay, s*ax+c*ay, az
        obj.velocity_available = obj.acceleration_available = True
        converted.objects.append(obj)
        original = ObjectStatus()
        original.unique_id, original.type, original.heading = sim_id, kind, heading
        original.position.x, original.position.y, original.position.z = x, y, z
        original.size.x, original.size.y, original.size.z = length, width, height
        # Preserve native body-axis UDP velocity (km/h) and acceleration.
        original.velocity.x, original.velocity.y, original.velocity.z = row[12:15]
        original.acceleration.x, original.acceleration.y, original.acceleration.z = row[15:18]
        {0: native.pedestrian_list, 1: native.npc_list, 2: native.obstacle_list}[kind].append(original)
    native.num_of_npcs, native.num_of_pedestrian, native.num_of_obstacle = (
        len(native.npc_list), len(native.pedestrian_list), len(native.obstacle_list))
    return converted, native


def main():
    import rospy
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from interfaces.msg import Objects
    from morai_msgs.msg import ObjectStatusList
    from std_msgs.msg import UInt8MultiArray
    if not rospy.core.is_initialized():
        rospy.init_node("object_info_gt_recorder")
    objects_pub = rospy.Publisher("/ground_truth/objects", Objects, queue_size=1)
    native_pub = rospy.Publisher("/ground_truth/object_status", ObjectStatusList, queue_size=1)
    raw_pub = rospy.Publisher("/ground_truth/raw_object_info", UInt8MultiArray, queue_size=10)
    timing_pub = rospy.Publisher("/ground_truth/timing", DiagnosticArray, queue_size=1)
    port = int(rospy.get_param("~port", 7505))
    source_ip = rospy.get_param("~source_ip", "")
    if not 1 <= port <= 65535:
        raise ValueError("GT port must be 1..65535")
    count, last_packet, last_report = 0, None, 0
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4194304)
        sock.bind((rospy.get_param("~bind_ip", "0.0.0.0"), port))
        sock.settimeout(.2)
        rospy.loginfo("GT only: UDP %d -> /ground_truth/*; no publication to /objects", port)
        while not rospy.is_shutdown():
            try:
                packet, address = sock.recvfrom(65535)
                if source_ip and address[0] != source_ip:
                    continue
                received = rospy.Time.now()
                raw = UInt8MultiArray(); raw.data = list(packet); raw_pub.publish(raw)
                seconds, nanoseconds, records = decode_packet(packet)
                stamp = rospy.Time(seconds, nanoseconds)
                objects, native = messages(records, stamp)
                objects_pub.publish(objects); native_pub.publish(native)
                count += 1; last_packet = time.monotonic()
                report = DiagnosticArray(); report.header.stamp = received
                status = DiagnosticStatus(); status.name = "object_info_gt"; status.level = DiagnosticStatus.OK
                status.message = "GT received; specification bbox center is approximate"
                status.values = [KeyValue(k, str(v)) for k, v in [
                    ("packets", count), ("source_stamp", "{}.{:09d}".format(seconds, nanoseconds)),
                    ("receive_minus_source_sec", (received - stamp).to_sec()),
                    ("objects", len(records)), ("sender", address[0]), ("packet_bytes", len(packet))]]
                report.status = [status]; timing_pub.publish(report)
                if count == 1:
                    rospy.loginfo("GT RECEIVED: %d objects, source-to-receive offset %.6f s", len(records), (received-stamp).to_sec())
            except socket.timeout:
                pass
            except Exception as error:
                rospy.logwarn_throttle(2, "GT UDP rejected: %s", error)
            now = time.monotonic()
            if (last_packet is None or now-last_packet > 1) and now-last_report > 2:
                rospy.logwarn("No current GT: enable Object Info UDP to this PC port %d in MORAI", port)
                last_report = now


if __name__ == "__main__":
    while True:
        try:
            import rospy
            if rospy.is_shutdown():
                break
            main()
            break
        except Exception as error:
            print("GT receiver retrying: {}".format(error), flush=True)
            time.sleep(1)
