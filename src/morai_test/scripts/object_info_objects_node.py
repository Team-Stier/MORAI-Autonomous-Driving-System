#!/usr/bin/env python3
"""Manual test source for /objects; header uses the Object Info UDP timestamp.

Receive Object_Info on 7505 by default and publish its map ground truth.
Protocol: MORAI-NetworkModule/24.R2.0/lib/define/ObjectInfo.py.
Bboxes approximate asset specifications, not simulator collider bounds.
"""
import math
import socket
import struct
import time


def decode_stamp(packet, offset):
    seconds, nanoseconds = struct.unpack_from("<ii", packet, offset)
    if seconds < 0 or not 0 <= nanoseconds < 1000000000 or (seconds == 0 and nanoseconds == 0):
        raise ValueError("invalid UDP source timestamp")
    return seconds, nanoseconds


def decode_objects(packet):
    # Twenty slots: legacy 68-byte or current 106-byte records with link ID.
    if (len(packet) not in (1400, 2160) or packet[:14] != b"#MoraiObjInfo$"
            or struct.unpack_from("<i", packet, 14)[0] != len(packet) - 32
            or packet[-2:] != b"\r\n"):
        raise ValueError("expected timestamped MORAI Object Info (1400 or 2160 bytes)")
    seconds, nanoseconds = decode_stamp(packet, 30)
    stride = (len(packet) - 40) // 20
    records, seen = [], set()
    for index in range(20):
        values = struct.unpack_from("<hh16f", packet, 38 + index * stride)
        if values[0] == 0:
            continue
        if (values[1] not in (-1, 0, 1, 2)
                or not all(math.isfinite(value) for value in values[2:])
                or not all(size > 0 for size in values[6:9])
                or values[:2] in seen):
            raise ValueError("invalid or duplicate Object Info record")
        seen.add(values[:2])
        records.append(values)
    return seconds, nanoseconds, records


class ObjectsConverter:
    def __init__(self):
        self.ids = {}
        self.next_id = 1

    def convert(self, records, stamp):
        from interfaces.msg import Object, Objects
        output = Objects()
        output.header.frame_id, output.header.stamp = "map", stamp
        active_ids = {}
        for values in records:
            sim_id, kind, x, y, z, heading, length, width, height = values[:9]
            if kind == -1:
                continue
            yaw = math.radians(heading)
            offset = (values[10] + values[9] - values[11]) / 2 if kind == 1 else 0
            # shortcut: specification box and height/2 approximate the center; use mesh GT for exact bboxes.
            center = (x + math.cos(yaw) * offset, y + math.sin(yaw) * offset, z + height / 2)
            obj = Object()
            key = (kind, sim_id)
            if key not in self.ids:
                self.ids[key] = self.next_id
                self.next_id += 1
            obj.id = active_ids[key] = self.ids[key]
            obj.label = {0: "pedestrian", 1: "vehicle", 2: "obstacle"}[kind]
            obj.position.x, obj.position.y, obj.position.z = map(float, center)
            obj.bbox_pose.position = obj.position
            obj.bbox_pose.orientation.z = math.sin(yaw / 2)
            obj.bbox_pose.orientation.w = math.cos(yaw / 2)
            obj.bbox_size.x, obj.bbox_size.y, obj.bbox_size.z = length, width, height
            # GT absolute motion is already expressed in map axes.
            velocity = [value / 3.6 for value in values[12:15]]
            acceleration = values[15:18]
            obj.velocity.x, obj.velocity.y, obj.velocity.z = map(float, velocity)
            obj.acceleration.x, obj.acceleration.y, obj.acceleration.z = map(float, acceleration)
            obj.velocity_available = obj.acceleration_available = True
            output.objects.append(obj)
        self.ids = active_ids
        return output


def receive(converter):
    import rospy
    from interfaces.msg import Objects

    bind_ip = rospy.get_param("~bind_ip", "0.0.0.0")
    object_port = int(rospy.get_param("~object_port", 7505))
    source_ip = rospy.get_param("~source_ip", "")
    if not 1 <= object_port <= 65535:
        raise ValueError("object_port must be a UDP port between 1 and 65535")
    publisher = rospy.Publisher("/objects", Objects, queue_size=1)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as objects_sock:
        objects_sock.bind((bind_ip, object_port))
        objects_sock.settimeout(0.2)
        rospy.loginfo("MORAI test objects: Object_Info UDP %s:%d -> /objects (map)", bind_ip, object_port)
        while not rospy.is_shutdown():
            try:
                packet, source = objects_sock.recvfrom(65535)
                if source_ip and source[0] != source_ip:
                    continue
                seconds, nanoseconds, records = decode_objects(packet)
                publisher.publish(converter.convert(records, rospy.Time(seconds, nanoseconds)))
            except socket.timeout:
                continue
            except Exception as error:
                rospy.logwarn_throttle(5.0, "MORAI test UDP packet rejected: %s", error)


def main():
    converter = ObjectsConverter()
    while True:
        try:
            import rospy
            if rospy.is_shutdown():
                return
            if not rospy.core.is_initialized():
                rospy.init_node("morai_test_objects")
            receive(converter)
        except Exception as error:
            try:
                rospy.logerr("MORAI test objects retrying: %s", error)
            except Exception:
                print("MORAI test objects retrying:", error, flush=True)
            time.sleep(1.0)


if __name__ == "__main__":
    main()
