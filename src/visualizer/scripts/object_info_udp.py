#!/usr/bin/env python3
"""Receive only; never change MORAI settings.

Wire formats: MORAI-DriveExample_UDP/network/receiver/object_info_receiver.py
(68-byte records) and MORAI SIM Drive 24.R2 UDP Object Info (106-byte records).
The size fields are simulator specifications, not measured collider bboxes.
"""
import math
import struct


def decode_objects(packet):
    # Header 14 + length 4 + aux 12 + timestamp 8; twenty slots and CRLF.
    if (len(packet) not in (1400, 2160) or packet[:14] != b"#MoraiObjInfo$"
            or struct.unpack_from("<I", packet, 14)[0] != len(packet) - 32
            or packet[-2:] != b"\r\n"):
        raise ValueError("expected timestamped MORAI Object Info with 20 records (68 or 106 bytes)")
    record_size = (len(packet) - 40) // 20
    objects = []
    for index in range(20):
        values = struct.unpack_from("<hh7f", packet, 38 + index * record_size)
        if values[0] == 0:  # Unused slot, as in the official UDP example.
            continue
        if all(math.isfinite(value) for value in values[2:]) and all(size > 0 for size in values[6:]):
            objects.append(values)
    return objects


def main():
    import rospy
    from morai_msgs.msg import ObjectStatus, ObjectStatusList
    from udp_receiver import receive

    if not rospy.core.is_initialized():
        rospy.init_node("visualizer_object_info_udp")
    publisher = rospy.Publisher("/visualizer/object_info", ObjectStatusList, queue_size=1)

    def handle(packet, stamp):
        message = ObjectStatusList()
        message.header.frame_id, message.header.stamp = "map", stamp
        for object_id, kind, x, y, z, heading, length, width, height in decode_objects(packet):
            obj = ObjectStatus()
            obj.unique_id, obj.type, obj.heading = object_id, kind, heading
            obj.position.x, obj.position.y, obj.position.z = x, y, z
            obj.size.x, obj.size.y, obj.size.z = length, width, height
            if kind == 0:
                message.pedestrian_list.append(obj)
            elif kind == 2:
                message.obstacle_list.append(obj)
            else:
                message.npc_list.append(obj)
        message.num_of_npcs = len(message.npc_list)
        message.num_of_pedestrian = len(message.pedestrian_list)
        message.num_of_obstacle = len(message.obstacle_list)
        publisher.publish(message)

    receive(7505, handle, 30)


if __name__ == "__main__":
    from udp_receiver import run
    run(main)
