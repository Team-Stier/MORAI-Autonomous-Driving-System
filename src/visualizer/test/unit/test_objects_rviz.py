#!/usr/bin/env python3
"""#OBB_REGRESSION: 회전 박스를 AABB로 바꾸거나 이전 검출을 남기는 회귀를 검증한다."""
import importlib.util
import math
from pathlib import Path
import unittest
import shutil
import subprocess
import tempfile

from interfaces.msg import Object, Objects
from visualization_msgs.msg import Marker
import rospy


SCRIPT = Path(__file__).resolve().parents[2] / "scripts" / "objects_rviz.py"


class ObjectsRvizTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not SCRIPT.exists():
            return
        spec = importlib.util.spec_from_file_location("objects_rviz", SCRIPT)
        cls.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.module)

    def module_ready(self):
        self.assertTrue(SCRIPT.exists(), "OBB MarkerArray 변환기가 아직 구현되지 않았다")
        return self.module

    def sample(self):
        message = Objects()
        message.header.frame_id = "base_link_level"
        message.header.stamp = rospy.Time(123, 456)
        obj = Object()
        obj.id = 42
        obj.bbox_pose.position.x = 7.0
        obj.bbox_pose.position.y = -2.0
        obj.bbox_pose.orientation.z = math.sin(math.pi / 8)
        obj.bbox_pose.orientation.w = math.cos(math.pi / 8)
        obj.bbox_size.x, obj.bbox_size.y, obj.bbox_size.z = 4.0, 1.0, 2.0
        obj.position = obj.bbox_pose.position
        message.objects = [obj]
        return message

    def test_rotated_box_preserves_pose_and_local_axis_lengths(self):
        module = self.module_ready()
        message = self.sample()
        array = module.make_markers(message)
        box = next(m for m in array.markers if m.ns == "obb")
        self.assertEqual(box.pose, message.objects[0].bbox_pose)
        self.assertEqual(box.scale, message.objects[0].bbox_size)
        self.assertEqual(box.header, message.header)
        self.assertFalse(box.frame_locked)

    def test_heading_and_center_use_observed_pose(self):
        module = self.module_ready()
        message = self.sample()
        markers = module.make_markers(message).markers
        arrow = next(m for m in markers if m.ns == "heading")
        center = next(m for m in markers if m.ns == "center")
        self.assertEqual(arrow.pose, message.objects[0].bbox_pose)
        self.assertEqual(center.pose.position, message.objects[0].bbox_pose.position)

    def test_empty_frame_clears_old_boxes_and_markers_expire(self):
        module = self.module_ready()
        for message in [self.sample(), Objects()]:
            markers = module.make_markers(message).markers
            self.assertEqual(markers[0].action, Marker.DELETEALL)
            for marker in markers[1:]:
                self.assertGreater(marker.lifetime.to_sec(), 0)
                self.assertLessEqual(marker.lifetime.to_sec(), 0.5)
        self.assertEqual(len(module.make_markers(Objects()).markers), 1)

    def test_conversion_does_not_mutate_objects(self):
        module = self.module_ready()
        message = self.sample()
        import copy
        expected = copy.deepcopy(message)
        array = module.make_markers(message)
        array.markers[1].pose.position.x = 999
        self.assertEqual(message, expected)

    def test_receive_history_is_bounded_without_ros_timer(self):
        module = self.module_ready()
        self.assertTrue(hasattr(module, "ReceiveMetrics"), "수신 통계의 독립 시간 관리가 필요하다")
        metrics = module.ReceiveMetrics()
        for index in range(2500):
            message = self.sample()
            message.header.seq = index
            message.header.stamp = rospy.Time(index + 1)
            metrics.record("raw", message, index / 100.0)
        self.assertLessEqual(len(metrics.arrivals["raw"]), 1000)
        self.assertLessEqual(metrics.arrivals["raw"][-1] - metrics.arrivals["raw"][0], 5.0)
        summary = metrics.snapshot(40.0, rospy.Time(40))
        self.assertEqual(summary["raw"]["hz"], 0.0)

    def test_stamp_join_handles_reordered_callbacks(self):
        module = self.module_ready()
        self.assertTrue(hasattr(module, "ReceiveMetrics"), "stamp 결합 통계가 필요하다")
        metrics = module.ReceiveMetrics()
        message = self.sample()
        metrics.record("objects", message, 10.0)
        metrics.record("leveled", message, 10.01)
        metrics.record("raw", message, 10.02)
        summary = metrics.snapshot(11.0, rospy.Time(124))
        self.assertEqual(summary["objects_without_leveled_stamp"], 0)
        self.assertEqual(summary["leveled_without_raw_stamp"], 0)
        self.assertEqual(summary["objects_receive_delay_ms_mean"], None)
        self.assertEqual(summary["receive_order_inversions"], 1)

    def test_installed_entrypoint_loads_install_overlay(self):
        # #INSTALL_LAYOUT: 실제 bash 진입점을 복사된 설치 구조에서 실행한다.
        # ROS 노드 대신 기록용 roslaunch 실행 파일로 최종 인자 전달만 관측한다.
        with tempfile.TemporaryDirectory(prefix="visualizer_launch_test_") as directory:
            root = Path(directory)
            package = root / "install" / "share" / "visualizer"
            package.mkdir(parents=True)
            shutil.copyfile(SCRIPT.parent.parent / "launch.sh", package / "launch.sh")
            bin_path = root / "bin"
            bin_path.mkdir()
            launch = bin_path / "roslaunch"
            launch.write_text('#!/bin/bash\nprintf "%s\\n" "$@"\n', encoding="utf-8")
            launch.chmod(0o755)
            (root / "install" / "setup.bash").write_text(
                'export PATH="' + str(bin_path) + ':$PATH"\n', encoding="utf-8")
            result = subprocess.run(["bash", str(package / "launch.sh"), "rviz:=false"],
                                    text=True, capture_output=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(result.stdout.splitlines(), ["visualizer", "live_lidar.launch", "rviz:=false"])


if __name__ == "__main__":
    unittest.main()
