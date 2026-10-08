#!/usr/bin/env python3
"""#VISUALIZATION_ONLY: Objects를 관측 좌표 그대로 표시하고 수신 경계의 상태만 측정한다."""
import copy
import csv
import json
import threading
import time
from collections import deque, OrderedDict

import rospy
from geometry_msgs.msg import Point
from interfaces.msg import Objects
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


def make_markers(message):
    """#OBB_POSE: bbox local 축의 길이와 quaternion을 그대로 RViz에 전달한다.

    Cube의 scale은 부모 frame의 AABB extent가 아니라 bbox_size이다. 테두리도
    bbox local 좌표에서 만든 뒤 같은 pose를 적용한다. 센서 offset·roll/pitch를
    다시 적용하거나 yaw를 새로 추정하지 않는다. frame과 stamp는 관측 header다.
    """
    clear = Marker(action=Marker.DELETEALL)
    clear.header = copy.deepcopy(message.header)
    markers = [clear]
    for index, obj in enumerate(message.objects):
        def marker(namespace, marker_type, alpha=1.0):
            value = Marker()
            value.header = copy.deepcopy(message.header)
            value.ns, value.id, value.type = namespace, index, marker_type
            value.action = Marker.ADD
            value.pose = copy.deepcopy(obj.bbox_pose)
            value.color.r, value.color.g, value.color.b, value.color.a = 0.15, 1.0, 0.4, alpha
            # #NO_GHOST: 새 프레임의 DELETEALL로 객체 감소·빈 목록을 즉시 반영한다.
            # 수신 자체가 끊겨도 마지막 박스는 0.35초 뒤 사라진다. frame_locked를
            # 끄므로 과거 관측을 최신 TF에 붙여 이동시키지 않는다.
            value.lifetime = rospy.Duration(0.35)
            value.frame_locked = False
            return value

        box = marker("obb", Marker.CUBE, 0.12)
        box.scale = copy.deepcopy(obj.bbox_size)
        markers.append(box)

        edges = marker("edges", Marker.LINE_LIST)
        edges.scale.x = 0.025
        sx, sy, sz = obj.bbox_size.x / 2, obj.bbox_size.y / 2, obj.bbox_size.z / 2
        corners = [Point(x=x, y=y, z=z) for z in (-sz, sz)
                   for y in (-sy, sy) for x in (-sx, sx)]
        for a, b in [(0, 1), (0, 2), (1, 3), (2, 3), (4, 5), (4, 6),
                     (5, 7), (6, 7), (0, 4), (1, 5), (2, 6), (3, 7)]:
            edges.points.extend([corners[a], corners[b]])
        markers.append(edges)

        arrow = marker("heading", Marker.ARROW)
        arrow.scale.x, arrow.scale.y, arrow.scale.z = max(0.4, sx), 0.07, 0.07
        arrow.color.r, arrow.color.g, arrow.color.b = 1.0, 0.7, 0.0
        # #ORIENTATION_AXIS: 화살표는 OBB local +x축을 가리킨다. 검출 형상의 축이며
        # 추적 속도나 차량의 실제 전후 방향을 뜻하지 않는다(OBB의 180도 모호성).
        markers.append(arrow)
        center = marker("center", Marker.SPHERE)
        center.scale.x = center.scale.y = center.scale.z = 0.12
        center.color.r, center.color.g, center.color.b = 1.0, 0.2, 0.2
        markers.append(center)
        label = marker("object_id", Marker.TEXT_VIEW_FACING)
        label.pose.position.z += sz + 0.25
        label.pose.orientation.x = label.pose.orientation.y = label.pose.orientation.z = 0.0
        label.pose.orientation.w = 1.0
        label.scale.z = 0.22
        label.text = str(obj.id)
        markers.append(label)
    return MarkerArray(markers=markers)


class ReceiveMetrics:
    """#RECEIVE_METRICS: monotonic 수신 시각과 원본 stamp를 서로 구분한다.

    raw→level→objects의 같은 stamp를 대조한다. 수신 순서는 TCP 연결마다 다를 수
    있어 양쪽 기록을 보관한다. 검출 지연에는 ROS 전송·스케줄링도 포함되므로 OBB
    계산시간으로 표기하지 않는다. sequence gap은 이 구독자에서 관측한 누락이며
    센서 UDP 유실과 동일하지 않다. production Core에는 계측 코드를 넣지 않는다.
    """
    def __init__(self):
        self.lock = threading.RLock()
        # #BOUNDED_HISTORY: ROS 시간이 멈춰도 수신 이력은 증가하지 않는다.
        # 5초 창은 입력 시에도 정리하며 예상보다 높은 입력률에는 1000개로 제한한다.
        self.arrivals = {key: deque(maxlen=1000) for key in ("raw", "leveled", "objects")}
        self.records = {key: OrderedDict() for key in self.arrivals}
        self.latest = {}
        self.gaps = {key: 0 for key in self.arrivals}
        self.previous_seq = {}
        self.delays = deque(maxlen=100)
        self.matched = set()
        self.objects_count = None
        self.receive_order_inversions = 0

    def record(self, key, message, now):
        with self.lock:
            self.arrivals[key].append(now)
            while self.arrivals[key] and now - self.arrivals[key][0] > 5.0:
                self.arrivals[key].popleft()
            self.latest[key] = (message.header, now)
            if key == "objects":
                self.objects_count = len(message.objects)
            previous = self.previous_seq.get(key)
            if previous is not None and message.header.seq > previous + 1:
                self.gaps[key] += message.header.seq - previous - 1
            self.previous_seq[key] = message.header.seq
            records = self.records[key]
            stamp = message.header.stamp.to_nsec()
            records[stamp] = now
            while len(records) > 200:
                records.popitem(last=False)
            # #STAMP_JOIN: 같은 stamp의 입력/출력 수신 간격을 측정한다. 출력 콜백이
            # 먼저 실행된 경우에도 결합하되 음수 간격을 계산시간 0으로 둔갑시키지
            # 않고 별도 역전 횟수로 기록한다. 서로 다른 TCP 연결의 수신 순서다.
            if stamp not in self.matched and stamp in self.records["leveled"] and stamp in self.records["objects"]:
                delay = self.records["objects"][stamp] - self.records["leveled"][stamp]
                if delay >= 0:
                    self.delays.append(delay * 1000)
                else:
                    self.receive_order_inversions += 1
                self.matched.add(stamp)
            self.matched.intersection_update(self.records["objects"])

    def snapshot(self, now, ros_now):
        summary = {}
        with self.lock:
            for key, arrivals in self.arrivals.items():
                while arrivals and now - arrivals[0] > 5.0:
                    arrivals.popleft()
                rate = (len(arrivals) - 1) / (arrivals[-1] - arrivals[0]) if len(arrivals) > 1 else 0.0
                header, receipt = self.latest.get(key, (None, None))
                summary[key] = {
                    "hz": round(rate, 2), "observed_sequence_gaps": self.gaps[key],
                    "frame_id": header.frame_id if header else None,
                    "last_stamp_ns": header.stamp.to_nsec() if header else None,
                    "receive_age_ms": round((now - receipt) * 1000, 1) if receipt else None,
                    "stamp_age_ms": round((ros_now - header.stamp).to_sec() * 1000, 1) if header else None,
                }
            summary["object_count"] = self.objects_count
            summary["objects_receive_delay_ms_mean"] = round(sum(self.delays) / len(self.delays), 2) if self.delays else None
            summary["obb_compute_ms"] = None
            summary["receive_order_inversions"] = self.receive_order_inversions
            # #UNMATCHED: 결합 가능 시간이 지난 stamp만 세므로 콜백 순서 차이를
            # timestamp 불일치로 단정하지 않는다. 누락/검사 거부도 이 값에 포함된다.
            summary["objects_without_leveled_stamp"] = sum(
                1 for stamp, receipt in self.records["objects"].items()
                if now - receipt > 0.5 and stamp not in self.records["leveled"])
            summary["leveled_without_raw_stamp"] = sum(
                1 for stamp, receipt in self.records["leveled"].items()
                if now - receipt > 0.5 and stamp not in self.records["raw"])
        return summary


class LiveView:
    """#ROS_DISPLAY_ADAPTER: 표시와 관측만 담당하며 검출 Core·제어 명령에 접근하지 않는다."""
    def __init__(self):
        self.metrics = ReceiveMetrics()
        self.publisher = rospy.Publisher("/rviz/objects", MarkerArray, queue_size=1)
        self.metrics_publisher = rospy.Publisher("/rviz/perception_metrics", String, queue_size=1)
        self.hud_publisher = rospy.Publisher("/rviz/perception_hud", Marker, queue_size=1)
        self.log_lock = threading.Lock()
        self.log_stream = None
        self.csv = None
        log_path = rospy.get_param("~frame_log", "")
        if log_path:
            self.log_stream = open(log_path, "x", newline="", encoding="utf-8")
            self.csv = csv.writer(self.log_stream)
            self.csv.writerow(["received_wall_sec", "stamp_ns", "frame_id", "object_count", "stamp_age_ms"])
        # #POINTCLOUD_BUFFER: VLP16 직렬화 점군은 rospy 기본 64KiB보다 크다.
        # queue_size=1이 읽기 버퍼에 남은 이전 메시지를 버릴 수 있게 4MiB를
        # 기본으로 둔다. 더 큰 센서로 바꿀 때 실제 메시지 크기보다 크게 조정한다.
        buffer_bytes = int(rospy.get_param("~cloud_buffer_bytes", 4 * 1024 * 1024))
        if buffer_bytes <= 0:
            raise ValueError("cloud_buffer_bytes must be positive")
        self.subscribers = [
            rospy.Subscriber("/velodyne_points", PointCloud2, self.on_raw, queue_size=1, buff_size=buffer_bytes),
            rospy.Subscriber("/velodyne_points/leveled", PointCloud2, self.on_leveled, queue_size=1, buff_size=buffer_bytes),
            rospy.Subscriber("/objects", Objects, self.on_objects, queue_size=1, buff_size=buffer_bytes),
        ]
        # #WALL_HEALTH: /clock가 없거나 MORAI가 pause되어도 수신 단절을 보고한다.
        # ROS Timer 대신 종료 가능한 wall-time 대기를 쓰며, pose의 관측 stamp와
        # Marker lifetime은 ROS 규약을 그대로 따른다.
        self.stop_event = threading.Event()
        self.reporter = threading.Thread(target=self.report_loop, daemon=True)
        rospy.on_shutdown(self.close)
        self.reporter.start()

    def close(self):
        self.stop_event.set()
        self.reporter.join(timeout=2.0)
        with self.log_lock:
            if self.log_stream:
                self.log_stream.close()
                self.csv = None

    def report_loop(self):
        while not self.stop_event.wait(1.0) and not rospy.is_shutdown():
            self.on_timer()

    def on_raw(self, message):
        self.metrics.record("raw", message, time.monotonic())

    def on_leveled(self, message):
        self.metrics.record("leveled", message, time.monotonic())

    def on_objects(self, message):
        self.metrics.record("objects", message, time.monotonic())
        with self.log_lock:
            if self.csv:
                age_ms = (rospy.Time.now() - message.header.stamp).to_sec() * 1000
                self.csv.writerow([time.time(), message.header.stamp.to_nsec(), message.header.frame_id,
                                   len(message.objects), age_ms])
                self.log_stream.flush()
        self.publisher.publish(make_markers(message))

    def on_timer(self):
        summary = self.metrics.snapshot(time.monotonic(), rospy.Time.now())
        text = json.dumps(summary, ensure_ascii=False)
        self.metrics_publisher.publish(String(data=text))
        rospy.loginfo("perception_receive_metrics %s", text)
        hud = Marker()
        hud.header.frame_id = "base_link_level"
        hud.header.stamp = rospy.Time.now()
        hud.ns, hud.id, hud.type, hud.action = "status", 0, Marker.TEXT_VIEW_FACING, Marker.ADD
        hud.pose.position.x, hud.pose.position.y, hud.pose.position.z = 3.0, 0.0, 3.2
        hud.pose.orientation.w = 1.0
        hud.scale.z = 0.35
        hud.color.r = hud.color.g = hud.color.b = hud.color.a = 1.0
        hud.lifetime = rospy.Duration(2.0)
        counts = summary["object_count"] if summary["object_count"] is not None else "--"
        ages = [summary[k]["receive_age_ms"] for k in self.metrics.arrivals]
        stale = any(age is None or age > 500 for age in ages)
        hud.text = ("Objects: {} | {}\nRaw {:.1f} / Level {:.1f} / Objects {:.1f} Hz".format(
            counts, "WAITING / STALE" if stale else "LIVE",
            summary["raw"]["hz"], summary["leveled"]["hz"], summary["objects"]["hz"]))
        self.hud_publisher.publish(hud)


if __name__ == "__main__":
    rospy.init_node("objects_rviz")
    view = LiveView()
    rospy.spin()
