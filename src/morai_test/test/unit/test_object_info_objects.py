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
    struct.pack_into("<i", packet, 14, len(packet) - 40)
    struct.pack_into("<ii", packet, 30, 12, 345)
    for index, values in enumerate(records):
        struct.pack_into("<hh16f", packet, 38 + index * stride, *values)
    packet[-2:] = b"\r\n"
    return bytes(packet)


class ObjectInfoObjectsTest(unittest.TestCase):
    def test_payload_length_includes_eight_byte_source_timestamp(self):
        # 실제 25.S4 수신 패킷: 전체 2160바이트, length 필드 2128바이트.
        for stride in (68, 106):
            packet = bytearray(objects_packet([record()], stride))
            struct.pack_into("<i", packet, 14, len(packet) - 32)
            self.assertEqual(NODE.decode_objects(bytes(packet)), (12, 345, [record()]))
            for wrong_length in (len(packet) - 33, len(packet) - 31):
                struct.pack_into("<i", packet, 14, wrong_length)
                with self.assertRaises(ValueError):
                    NODE.decode_objects(bytes(packet))

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
        self.assertFalse(obj.velocity_available)
        self.assertFalse(obj.acceleration_available)
        orientation = obj.bbox_pose.orientation
        self.assert_vector(orientation, (0, 0, math.sqrt(0.5)))
        self.assertAlmostEqual(abs(orientation.w), math.sqrt(0.5))
        ids = [item.id for item in message.objects]
        self.assertEqual(len(set(ids)), 2)
        self.assertTrue(all(0 <= value <= 0xffffffff for value in ids))
        reordered = converter.convert([record(7, 0), record()], stamp)
        self.assertEqual([item.id for item in reordered.objects], list(reversed(ids)))
        self.assertEqual(reordered.objects[0].label, "pedestrian")

    def test_explicit_scaled_asset_size_preserves_motion_stamp_and_unlisted_objects(self):
        converter = NODE.ObjectsConverter({"2:2": [1.49725926, 1.49725926, 1.30324721]})
        stamp = rospy.Time(12, 345)
        values = list(record(2, 2))
        values[6:9] = [.75, .75, .5]
        obj, untouched = converter.convert([tuple(values), record(3, 2)], stamp).objects
        self.assert_vector(obj.bbox_size, (1.49725926, 1.49725926, 1.30324721))
        self.assert_vector(obj.position, (10, 24, 30 + 1.30324721 / 2))
        self.assert_vector(obj.velocity, (0, 10, 0))
        self.assert_vector(untouched.bbox_size, (4, 2, 2))
        # kind가 다른 같은 raw ID에는 보정을 적용하지 않는다.
        self.assert_vector(converter.convert([record(2, 1)], stamp).objects[0].bbox_size, (4, 2, 2))

    def test_invalid_explicit_scaled_asset_size_is_rejected(self):
        for size in ([1, 2], [1, -2, 3], [1, math.nan, 3], [1, math.inf, 3]):
            with self.assertRaises(ValueError):
                NODE.ObjectsConverter({"2:2": size}).convert([record(2, 2)], rospy.Time(12, 345))

    def test_empty_ego_exclusion_and_raw_motion_without_attitude(self):
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


def native_actor(sim_id="42", kind="OBJECT_TYPE_VEHICLE"):
    return {"actor_info": {"id": {"value": sim_id}, "object_type": kind},
            "bounding_box": {"location": {"x": 11, "y": 22, "z": 33},
                             "extent": {"x": 1, "y": 2, "z": 3},
                             "rotation": {"z": -90}},
            "transform": {"rotation": {"z": -90}},
            "velocity": {"x": 50}, "global_velocity": {"y": -50},
            "acceleration": {"x": -2}}


class NativeObjectsTest(unittest.TestCase):
    def snapshot(self, *actors):
        return {"timestamp": "1700000000000123", "states": list(actors)}

    def xyz(self, value):
        return value.x, value.y, value.z

    def test_southbound_vehicle_uses_global_velocity_and_rotated_body_acceleration(self):
        result = NODE.ObjectsConverter().convert_native(self.snapshot(native_actor()), 0)
        obj = result.objects[0]
        self.assertEqual(result.header.frame_id, "map")
        self.assertEqual(result.header.stamp, rospy.Time(1700000000, 123000))
        self.assertEqual(self.xyz(obj.position), (11, 22, 33))
        self.assertEqual(obj.bbox_pose.position, obj.position)
        self.assertEqual(self.xyz(obj.velocity), (0, -50 / 3.6, 0))
        self.assertAlmostEqual(obj.acceleration.x, 0)
        self.assertAlmostEqual(obj.acceleration.y, 2)
        self.assertAlmostEqual(obj.acceleration.z, 0)
        self.assertTrue(obj.velocity_available)
        self.assertTrue(obj.acceleration_available)

    def test_full_roll_and_pitch_rotate_all_three_acceleration_axes(self):
        import copy
        for rpy, body, expected in [({"x": 90}, (1, 2, 3), (1, -3, 2)),
                                    ({"y": 90}, (1, 2, 3), (3, 2, -1))]:
            state = native_actor()
            state["transform"]["rotation"] = rpy
            state["acceleration"] = dict(zip(("x", "y", "z"), body))
            state["global_velocity"] = {"x": 36, "y": 72, "z": 108}
            before = copy.deepcopy(state)
            obj = NODE.ObjectsConverter().convert_native(self.snapshot(state), 0).objects[0]
            for actual, wanted in zip(self.xyz(obj.acceleration), expected):
                self.assertAlmostEqual(actual, wanted)
            self.assertEqual(self.xyz(obj.velocity), (10, 20, 30))
            self.assertEqual(state, before)

    def test_grid_heading_offset_rotates_bbox_and_acceleration_not_global_velocity(self):
        state = native_actor()
        state["transform"]["rotation"] = {}
        state["bounding_box"]["rotation"] = {}
        obj = NODE.ObjectsConverter().convert_native(self.snapshot(state), math.pi / 2).objects[0]
        self.assertAlmostEqual(obj.acceleration.x, 0)
        self.assertAlmostEqual(obj.acceleration.y, -2)
        self.assertAlmostEqual(obj.bbox_pose.orientation.z, math.sqrt(.5))
        self.assertAlmostEqual(obj.bbox_pose.orientation.w, math.sqrt(.5))
        self.assertEqual(self.xyz(obj.velocity), (0, -50 / 3.6, 0))

    def test_bbox_axes_ego_exclusion_and_stable_ids(self):
        states = [native_actor("Ego"), native_actor("42"),
                  native_actor("3", "OBJECT_TYPE_PEDESTRIAN"),
                  native_actor("2", "OBJECT_TYPE_OBSTACLE")]
        converter = NODE.ObjectsConverter()
        result = converter.convert_native(self.snapshot(*states), 0)
        self.assertEqual([o.label for o in result.objects], ["vehicle", "pedestrian", "obstacle"])
        self.assertEqual([self.xyz(o.bbox_size) for o in result.objects], [(4, 2, 6), (6, 2, 4), (2, 4, 6)])
        ids = [o.id for o in result.objects]
        reordered = converter.convert_native(self.snapshot(*reversed(states)), 0)
        self.assertEqual([o.id for o in reordered.objects], list(reversed(ids)))

    def test_explicit_scaled_bbox_and_zero_motion_are_preserved(self):
        state = native_actor("2", "OBJECT_TYPE_OBSTACLE")
        state["global_velocity"] = state["acceleration"] = {}
        obj = NODE.ObjectsConverter({"2:2": [1.497, 1.497, 1.303]}).convert_native(self.snapshot(state), 0).objects[0]
        self.assertEqual(self.xyz(obj.bbox_size), (1.497, 1.497, 1.303))
        self.assertEqual(self.xyz(obj.position), (11, 22, 33))
        self.assertEqual(self.xyz(obj.velocity), (0, 0, 0))
        self.assertTrue(obj.velocity_available)
        self.assertTrue(obj.acceleration_available)

    def test_native_pedestrian_uses_body_velocity_with_full_attitude(self):
        for rotation, offset, expected in [({"z": 180}, 0, (-3, 0, 0)),
                                            ({"z": 90}, 0, (0, 3, 0)),
                                            ({}, math.pi / 2, (0, 3, 0)),
                                            ({"y": 90}, 0, (0, 0, -3))]:
            state = native_actor("3", "OBJECT_TYPE_PEDESTRIAN")
            state["global_velocity"] = {}
            state["velocity"] = {"x": 10.8}
            state["transform"]["rotation"] = rotation
            obj = NODE.ObjectsConverter().convert_native(self.snapshot(state), offset).objects[0]
            for actual, wanted in zip(self.xyz(obj.velocity), expected):
                self.assertAlmostEqual(actual, wanted)
            self.assertTrue(obj.velocity_available)

    def test_native_stationary_pedestrian_preserves_zero_and_source_stamp(self):
        state = native_actor("3", "OBJECT_TYPE_PEDESTRIAN")
        state["global_velocity"] = state["velocity"] = {}
        result = NODE.ObjectsConverter().convert_native(self.snapshot(state), -.023509)
        self.assertEqual(self.xyz(result.objects[0].velocity), (0, 0, 0))
        self.assertEqual(result.header.stamp, rospy.Time(1700000000, 123000))

    def test_invalid_native_frames_never_become_valid_zero_motion(self):
        import copy
        invalid = []
        state = native_actor()
        del state["global_velocity"]
        invalid.append(self.snapshot(state))
        state = native_actor()
        state["acceleration"]["x"] = math.nan
        invalid.append(self.snapshot(state))
        state = native_actor()
        state["bounding_box"]["extent"]["x"] = 0
        invalid.append(self.snapshot(state))
        invalid.append(self.snapshot(native_actor(), native_actor()))
        bad_stamp = self.snapshot(native_actor())
        bad_stamp["timestamp"] = "0"
        invalid.append(bad_stamp)
        for frame in invalid:
            with self.assertRaises((ValueError, KeyError)):
                NODE.ObjectsConverter().convert_native(copy.deepcopy(frame), 0)


if __name__ == "__main__":
    unittest.main()
