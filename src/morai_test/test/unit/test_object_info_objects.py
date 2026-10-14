#!/usr/bin/env python3
import importlib.util
import math
from pathlib import Path
import struct
import unittest

import rospy
from interfaces.msg import Objects


NODE_PATH = Path(__file__).resolve().parents[2] / "scripts/object_info_objects_node.py"
SPEC = importlib.util.spec_from_file_location("object_info_objects_node", str(NODE_PATH))
NODE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(NODE)


def record(sim_id=42, kind=1):
    return (sim_id, kind, 10.0, 24.0, 30.0, 90.0,
            4.0, 2.0, 2.0, 1.0, 2.0, 1.0,
            0.0, 36.0, 0.0, 0.0, 3.0, 0.0)


def objects_packet(records, stride=68):
    packet = bytearray(40 + 20 * stride)
    packet[:14] = b"#MoraiObjInfo$"
    struct.pack_into("<i", packet, 14, len(packet) - 32)
    struct.pack_into("<ii", packet, 30, 12, 345)
    for index, values in enumerate(records):
        struct.pack_into("<hh16f", packet, 38 + index * stride, *values)
    packet[-2:] = b"\r\n"
    return bytes(packet)


class ObjectInfoObjectsTest(unittest.TestCase):
    def assert_vector(self, vector, expected):
        for actual, wanted in zip((vector.x, vector.y, vector.z), expected):
            self.assertAlmostEqual(actual, wanted, places=6)

    def test_object_wire_layouts_and_invalid_packets(self):
        for stride in (68, 106):
            stamp_sec, stamp_nsec, records = NODE.decode_objects(objects_packet([record()], stride))
            self.assertEqual((stamp_sec, stamp_nsec), (12, 345))
            self.assertEqual(records, [record()])
        packet = objects_packet([record()])
        malformed = [packet[:-1], b"!" + packet[1:], packet[:-2] + b"!!"]
        bad_length = bytearray(packet)
        struct.pack_into("<i", bad_length, 14, 1)
        malformed.append(bytes(bad_length))
        for seconds, nanoseconds in ((0, 0), (-1, 0), (12, 1000000000)):
            bad_stamp = bytearray(packet)
            struct.pack_into("<ii", bad_stamp, 30, seconds, nanoseconds)
            malformed.append(bytes(bad_stamp))
        self.assertEqual(NODE.decode_stamp(struct.pack("<ii", 0, 1), 0), (0, 1))
        for field, value in ((2, math.nan), (6, 0.0), (7, -1.0)):
            values = list(record())
            values[field] = value
            malformed.append(objects_packet([values]))
        for bad_packet in malformed:
            with self.assertRaises(ValueError):
                NODE.decode_objects(bad_packet)

    def test_transform_units_bbox_and_ids_across_reordering(self):
        converter = NODE.ObjectsConverter()
        stamp = rospy.Time(12, 345)
        message = converter.convert([record(), record(7, 0)], stamp)
        self.assertIsInstance(message, Objects)
        self.assertEqual(message.header.frame_id, "map")
        self.assertEqual(message.header.stamp, stamp)
        obj = message.objects[0]
        self.assertEqual(obj.label, "vehicle")
        self.assert_vector(obj.position, (10, 25, 31))
        self.assert_vector(obj.bbox_pose.position, (10, 25, 31))
        self.assert_vector(obj.bbox_size, (4, 2, 2))
        self.assert_vector(obj.velocity, (0, 10, 0))
        self.assert_vector(obj.acceleration, (0, 3, 0))
        self.assertTrue(obj.velocity_available)
        self.assertTrue(obj.acceleration_available)
        orientation = obj.bbox_pose.orientation
        self.assert_vector(orientation, (0, 0, math.sqrt(0.5)))
        self.assertAlmostEqual(abs(orientation.w), math.sqrt(0.5))
        ids = [item.id for item in message.objects]
        self.assertEqual(len(set(ids)), 2)
        self.assertTrue(all(0 <= value <= 0xffffffff for value in ids))
        reordered = converter.convert([record(7, 0), record()], stamp)
        self.assertEqual([item.id for item in reordered.objects], list(reversed(ids)))
        self.assertEqual(reordered.objects[0].label, "pedestrian")

    def test_empty_ego_exclusion_and_three_axis_map_motion(self):
        converter = NODE.ObjectsConverter()
        stamp = rospy.Time(12, 345)
        for records in ([], [record(1, -1)]):
            message = converter.convert(records, stamp)
            self.assertEqual(message.objects, [])
            self.assertEqual(message.header.frame_id, "map")
            self.assertEqual(message.header.stamp, stamp)
        values = (9, 2, 0, 2, 0, 0, 4, 2, 2, 0, 0, 0,
                  0, 3.6, -7.2, 0, 1, 3)
        obj = converter.convert([values], stamp).objects[0]
        self.assertEqual(obj.label, "obstacle")
        self.assert_vector(obj.position, (0, 2, 1))
        self.assert_vector(obj.velocity, (0, 1, -2))
        self.assert_vector(obj.acceleration, (0, 1, 3))
        orientation = obj.bbox_pose.orientation
        self.assertAlmostEqual(orientation.x, 0)
        self.assertAlmostEqual(orientation.y, 0)
        self.assertAlmostEqual(orientation.z, 0)
        self.assertAlmostEqual(orientation.w, 1)


if __name__ == "__main__":
    unittest.main()
