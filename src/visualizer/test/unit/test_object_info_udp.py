#!/usr/bin/env python3
import importlib.util
import math
from pathlib import Path
import struct
import unittest

module = importlib.util.spec_from_file_location(
    "object_info_udp", Path(__file__).resolve().parents[2] / "scripts/object_info_udp.py")
object_info_udp = importlib.util.module_from_spec(module)
module.loader.exec_module(object_info_udp)


def packet_with_object(record_size=106):
    packet = bytearray(40 + 20 * record_size)
    packet[:14] = b"#MoraiObjInfo$"
    struct.pack_into("<I", packet, 14, 20 * record_size)
    struct.pack_into("<II", packet, 30, 100, 123456789)
    struct.pack_into("<hh7f", packet, 38, 51, 1, -12.5, 22.25, 28.5, 90, 4.5, 2, 1.5)
    packet[-2:] = b"\r\n"
    return packet


class ObjectInfoUdpTest(unittest.TestCase):
    def test_both_official_record_formats_and_empty_slots(self):
        expected = (51, 1, -12.5, 22.25, 28.5, 90, 4.5, 2, 1.5)
        for record_size in (68, 106):
            packet = packet_with_object(record_size)
            # A second active slot catches incorrect record stride.
            struct.pack_into("<hh7f", packet, 38 + record_size, 52, 0, 1, 2, 3, 180, 0.5, 0.5, 1.8)
            objects = object_info_udp.decode_objects(packet)
            self.assertEqual(objects[0], expected)
            self.assertEqual(objects[1][:6], (52, 0, 1, 2, 3, 180))
            self.assertEqual(len(objects), 2)
            packet[38:-2] = bytes(20 * record_size)
            self.assertEqual(object_info_udp.decode_objects(packet), [])

    def test_reject_malformed_and_skip_invalid_objects(self):
        packet = packet_with_object()
        for malformed in (packet[:-1], b"bad" + packet[3:], packet[:-2] + b"xx"):
            with self.assertRaises(ValueError):
                object_info_udp.decode_objects(malformed)
        wrong_length = bytearray(packet)
        struct.pack_into("<I", wrong_length, 14, 1)
        with self.assertRaises(ValueError):
            object_info_udp.decode_objects(wrong_length)
        for value in (math.nan, math.inf):
            invalid = bytearray(packet)
            struct.pack_into("<f", invalid, 42, value)
            self.assertEqual(object_info_udp.decode_objects(invalid), [])
        struct.pack_into("<f", packet, 58, -1)
        self.assertEqual(object_info_udp.decode_objects(packet), [])


if __name__ == "__main__":
    unittest.main()
