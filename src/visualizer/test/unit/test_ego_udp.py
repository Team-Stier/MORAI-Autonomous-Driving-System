#!/usr/bin/env python3
import importlib.util
import math
from pathlib import Path
import struct
import unittest

module = importlib.util.spec_from_file_location(
    "ego_udp", Path(__file__).resolve().parents[2] / "scripts/ego_udp.py")
ego_udp = importlib.util.module_from_spec(module)
module.loader.exec_module(ego_udp)


class EgoUdpTest(unittest.TestCase):
    def test_normal_ego_and_invalid_packets(self):
        packet = bytearray(229)
        packet[:11] = b"#MoraiInfo$"
        struct.pack_into("<I", packet, 11, 200)
        packet[-2:] = b"\r\n"
        expected = (-12.5, -221.25, 28.5, -359.5, 0.25, 62.5)
        struct.pack_into("<6f", packet, 77, *expected)
        self.assertEqual(ego_udp.decode_ego(packet), expected)
        invalid = [packet[:181], b"bad" + packet[3:], packet[:-2] + b"xx"]
        wrong_length = bytearray(packet)
        struct.pack_into("<I", wrong_length, 11, 152)
        invalid.append(wrong_length)
        for value in (math.nan, math.inf):
            nonfinite = bytearray(packet)
            struct.pack_into("<f", nonfinite, 77, value)
            invalid.append(nonfinite)
        for malformed in invalid:
            with self.assertRaises(ValueError):
                ego_udp.decode_ego(malformed)


if __name__ == "__main__":
    unittest.main()
