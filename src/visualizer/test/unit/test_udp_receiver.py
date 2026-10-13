#!/usr/bin/env python3
import importlib.util
from pathlib import Path
import struct
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, patch

spec = importlib.util.spec_from_file_location(
    "udp_receiver", Path(__file__).resolve().parents[2] / "scripts/udp_receiver.py")
udp_receiver = importlib.util.module_from_spec(spec)
spec.loader.exec_module(udp_receiver)


class UdpReceiverTest(unittest.TestCase):
    def test_preserves_wire_seconds_and_nanoseconds_instead_of_receive_time(self):
        state = {"shutdown": False}
        rospy = SimpleNamespace(
            is_shutdown=lambda: state["shutdown"], get_param=lambda key, default: default,
            Time=MagicMock(side_effect=lambda seconds, nanoseconds: (seconds, nanoseconds)),
            rostime=SimpleNamespace(wallsleep=MagicMock()), loginfo=MagicMock(),
            logwarn_throttle=MagicMock(), logerr_throttle=MagicMock())
        rospy.Time.now.return_value = (1790632545, 0)
        packet = bytearray(38)
        struct.pack_into("<II", packet, 30, 1790632544, 95000001)
        sock = MagicMock()
        sock.__enter__.return_value = sock
        sock.recvfrom.return_value = (packet, ("127.0.0.1", 9000))
        received = []

        def handle(packet, stamp):
            received.append(stamp)
            state["shutdown"] = True

        with patch.dict("sys.modules", {"rospy": rospy}), patch.object(udp_receiver.socket, "socket", return_value=sock):
            udp_receiver.receive(7505, handle, 30)
        self.assertEqual([(1790632544, 95000001)], received)

    def test_retries_bind_failure_and_keeps_receiving_after_bad_packet(self):
        state = {"shutdown": False}
        rospy = SimpleNamespace(
            is_shutdown=lambda: state["shutdown"],
            get_param=lambda key, default: default,
            Time=lambda seconds, nanoseconds: (seconds, nanoseconds),
            rostime=SimpleNamespace(wallsleep=MagicMock()),
            loginfo=MagicMock(), logwarn_throttle=MagicMock(), logerr_throttle=MagicMock())
        failed, connected = MagicMock(), MagicMock()
        failed.__enter__.return_value = failed
        connected.__enter__.return_value = connected
        failed.bind.side_effect = OSError("address already in use")
        bad, good = bytearray(38), bytearray(38)
        bad[:3], good[:4] = b"bad", b"good"
        for packet in (bad, good):
            struct.pack_into("<II", packet, 30, 123, 456)
        connected.recvfrom.side_effect = [(bad, ("127.0.0.1", 9000)),
                                         (good, ("127.0.0.1", 9000))]
        received = []

        def handle(packet, stamp):
            if packet.startswith(b"bad"):
                raise ValueError("invalid packet")
            received.append((packet, stamp))
            state["shutdown"] = True

        with patch.dict("sys.modules", {"rospy": rospy}), \
                patch.object(udp_receiver.socket, "socket", side_effect=[failed, connected]):
            udp_receiver.receive(7505, handle, 30)
        self.assertEqual([(good, (123, 456))], received)
        self.assertEqual(1, rospy.rostime.wallsleep.call_count)
        self.assertEqual(1, rospy.logwarn_throttle.call_count)

    def test_rejects_missing_zero_and_noncanonical_source_timestamp(self):
        for offset in (27, 30):
            packet = bytearray(offset + 8)
            for invalid in ((0, 0), (123, 1000000000)):
                struct.pack_into("<II", packet, offset, *invalid)
                with self.assertRaises(ValueError):
                    udp_receiver.decode_timestamp(packet, offset)
            with self.assertRaises(ValueError):
                udp_receiver.decode_timestamp(packet[:-1], offset)
            struct.pack_into("<II", packet, offset, 0, 1)
            self.assertEqual((0, 1), udp_receiver.decode_timestamp(packet, offset))


if __name__ == "__main__":
    unittest.main()
