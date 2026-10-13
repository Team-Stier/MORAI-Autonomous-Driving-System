#!/usr/bin/env python3
"""Run after building: python3 src/visualizer/test/unit/check_visualizer_ros.py"""
import base64
from copy import deepcopy
import json
import math
import os
from pathlib import Path
import signal
import socket
import struct
import subprocess
import tempfile
import time
import xml.etree.ElementTree as ET
import xmlrpc.client


HORIZONS = ("0_5s", "1_0s", "1_5s", "2_0s", "2_5s", "3_0s", "3_5s", "4_0s", "4_5s", "5_0s")


def wait_for(condition, timeout=35, publish=None):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if condition():
            return
        if publish:
            publish()
        time.sleep(0.05)
    raise AssertionError("visualizer check timed out")


def main():
    # Private master and UDP ports keep fixture traffic outside the driving stack.
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        os.environ["ROS_MASTER_URI"] = "http://127.0.0.1:" + str(probe.getsockname()[1])
    root = Path(__file__).resolve().parents[4]
    with tempfile.TemporaryFile(mode="w+") as log:
        launch = subprocess.Popen(
            ["roslaunch", "--sigint-timeout=2", "--sigterm-timeout=2", "visualizer", "visualizer.launch",
             "udp_port:=19095", "object_info_port:=17505", "rviz:=false"],
            stdout=log, stderr=log, start_new_session=True)
        try:
            def ready():
                try:
                    return xmlrpc.client.ServerProxy(os.environ["ROS_MASTER_URI"]).getUri("check")[0] == 1
                except OSError:
                    return False
            wait_for(ready)
            import rospy
            import tf2_ros
            from geometry_msgs.msg import PoseStamped, TransformStamped
            from interfaces.msg import Object, Objects, Path as LocalPath, PathPoint, TrafficLight
            from morai_msgs.msg import ObjectStatusList
            from nav_msgs.msg import Odometry, Path as GlobalPath
            from tf.transformations import quaternion_matrix
            from visualization_msgs.msg import Marker

            rospy.init_node("check_visualizer", anonymous=True, disable_signals=True)
            expected_nodes = {"/" + path.parent.name + "_node" for path in
                              (root / "src/visualizer/config").glob("*/node.yaml")}
            master = xmlrpc.client.ServerProxy(os.environ["ROS_MASTER_URI"])
            wait_for(lambda: expected_nodes <= {node for channels in master.getSystemState("check")[2]
                                               for _, nodes in channels for node in nodes})

            def nodes_are_alive():
                try:
                    for name in expected_nodes | {"/visualizer_ego_udp", "/visualizer_object_info_udp"}:
                        result = master.lookupNode("check", name)
                        if result[0] != 1:
                            return False
                        if xmlrpc.client.ServerProxy(result[2]).getPid("check")[0] != 1:
                            return False
                    return True
                except OSError:
                    return False

            wait_for(nodes_are_alive)
            # Exceed the input timeout without sending data, then reject malformed UDP.
            time.sleep(0.7)
            assert nodes_are_alive(), "missing input stopped a visualizer node"
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udp:
                for port in (19095, 17505):
                    udp.sendto(b"invalid packet", ("127.0.0.1", port))
            seen, counts = {}, {}

            def record(key, message):
                seen[key] = message
                counts[key] = counts.get(key, 0) + 1

            topics = [("map", "/visualizer/map", Marker), ("route", "/visualizer/route", Marker),
                      ("ego", "/visualizer/ego", Marker), ("gt", "/visualizer/gt", Odometry),
                      ("gt_marker", "/visualizer/ego_gt", Marker), ("local", "/visualizer/local_path", Marker),
                      ("objects", "/visualizer/objects", Marker),
                      ("object_info", "/visualizer/object_info", ObjectStatusList),
                      ("morai_objects", "/visualizer/morai_objects", Marker),
                      ("traffic_light", "/visualizer/traffic_light", Marker)]
            topics.extend(("prediction_" + str(i), "/visualizer/predicted_objects_" + suffix + "_markers", Marker)
                          for i, suffix in enumerate(HORIZONS))
            subscribers = [rospy.Subscriber(topic, cls, lambda message, key=key: record(key, message), queue_size=1)
                           for key, topic, cls in topics]
            wait_for(lambda: "map" in seen)
            map_marker = seen["map"]
            assert map_marker.type == Marker.LINE_LIST and map_marker.header.frame_id == "map"
            assert len(map_marker.points) > 1000 and len(map_marker.colors) == len(map_marker.points)
            assert {(round(c.r), round(c.g), round(c.b)) for c in map_marker.colors} == {(1, 1, 0), (1, 1, 1), (0, 1, 1)}
            assert map_marker.header.stamp.is_zero() and map_marker.lifetime.is_zero()
            subscribers.append(rospy.Subscriber("/visualizer/map", Marker,
                lambda message: record("map_late", message), queue_size=1))
            wait_for(lambda: "map_late" in seen)
            assert seen["map_late"] == map_marker, "late subscriber did not receive the static map"

            osm = ET.parse(root / "src/map/kcity_lanelet2.osm").getroot()
            nodes = {}
            for node in osm.findall("node"):
                tags = {tag.get("k"): tag.get("v") for tag in node.findall("tag")}
                nodes[node.get("id")] = [float(tags["local_x"]), float(tags["local_y"]),
                                         float(tags.get("local_z", tags.get("ele", "0")))]
            ways = {way.get("id"): [nodes[p.get("ref")] for p in way.findall("nd")] for way in osm.findall("way")}
            lane = next(relation for relation in osm.findall("relation") if relation.get("id") == "95900")
            bounds = {member.get("role"): ways[member.get("ref")] for member in lane.findall("member")
                      if member.get("role") in ("left", "right")}
            centers = [[(a + b) / 2 for a, b in zip(left, right)]
                       for left, right in zip(bounds["left"][:5], bounds["right"][:5])]
            x, y, _ = centers[0]
            ego_pub = rospy.Publisher("/localization/ego", Odometry, queue_size=1)
            route_pub = rospy.Publisher("/visualizer/global_path", GlobalPath, queue_size=1, latch=True)
            local_pub = rospy.Publisher("/local_path/velocity_injected", LocalPath, queue_size=1)
            objects_pub = rospy.Publisher("/objects", Objects, queue_size=1)
            light_pub = rospy.Publisher("/traffic_light", TrafficLight, queue_size=1)
            prediction_pubs = [rospy.Publisher("/visualizer/predicted_objects_" + suffix, Objects, queue_size=1)
                               for suffix in HORIZONS]
            broadcaster = tf2_ros.TransformBroadcaster()
            wait_for(lambda: all(pub.get_num_connections() for pub in
                     [ego_pub, route_pub, local_pub, objects_pub, light_pub] + prediction_pubs))

            light = TrafficLight(state=TrafficLight.RED_LEFT)
            light.header.frame_id = "camera_7_optical_frame"
            def publish_light():
                light.header.stamp = rospy.Time.now()
                light_pub.publish(light)
            wait_for(lambda: "traffic_light" in seen
                     and seen["traffic_light"].header.stamp == light.header.stamp, publish=publish_light)
            status = seen["traffic_light"]
            assert status.type == Marker.TEXT_VIEW_FACING and status.header.frame_id == "base_link"
            assert status.header.stamp == light.header.stamp and status.frame_locked
            assert status.text == "Traffic light: RED + LEFT" and status.color.r == 1
            assert status.lifetime.to_sec() == 1.0
            before = counts["traffic_light"]
            light.state = 255
            publish_light()
            time.sleep(0.1)
            assert counts["traffic_light"] == before, "invalid traffic-light enum emitted a marker"

            route = GlobalPath()
            route.header.stamp, route.header.frame_id = rospy.Time.now(), "map"
            for center in centers:
                pose = PoseStamped()
                pose.header = deepcopy(route.header)
                pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = center
                pose.pose.orientation.w = 1
                route.poses.append(pose)
            wait_for(lambda: "route" in seen, publish=lambda: route_pub.publish(route))
            route_marker = seen["route"]
            assert route_marker.type == Marker.TRIANGLE_LIST and route_marker.points
            assert len(route_marker.colors) == len(route_marker.points)
            assert any(abs(color.a - 0.35) < 1e-6 for color in route_marker.colors), "lanelet fill missing"
            assert any(abs(color.a - 1.0) < 1e-6 for color in route_marker.colors), "lanelet boundaries missing"
            assert not any(abs(color.g - 0.55) < 1e-6 for color in route_marker.colors), "orange centerline remains"

            packet = bytearray(base64.b64decode(json.loads((root / "spec/ioniq5_bbox.meta.json").read_text())
                ["runtime_verification"]["pose_crosscheck"]["ego_packet_base64"]))
            struct.pack_into("<6f", packet, 77, x + 0.5, y + 0.5, 28.5, 0, 0, 90)
            object_packet = bytearray(2160)
            object_packet[:14] = b"#MoraiObjInfo$"
            struct.pack_into("<I", object_packet, 14, 2120)
            struct.pack_into("<hh7f", object_packet, 38, 51, 1, x - 4, y + 12, 28.5, 90, 4.5, 2, 1.5)
            object_packet[-2:] = b"\r\n"
            stamp = rospy.Time.now()
            ego = Odometry()
            ego.header.stamp, ego.header.frame_id, ego.child_frame_id = stamp, "map", "base_link"
            ego.pose.pose.position.x, ego.pose.pose.position.y = x, y
            ego.pose.pose.orientation.z = ego.pose.pose.orientation.w = math.sqrt(0.5)
            transform = TransformStamped()
            transform.header, transform.child_frame_id = deepcopy(ego.header), "base_link"
            transform.transform.translation.x, transform.transform.translation.y = x, y
            transform.transform.rotation = deepcopy(ego.pose.pose.orientation)
            local = LocalPath()
            local.header.stamp, local.header.frame_id = stamp, "base_link"
            for distance, speed_kph in ((5, 0), (7.5, 12.5), (10, 25), (15, 50), (20, 75), (25, 100), (30, 125)):
                point = PathPoint()
                point.pose.header = deepcopy(local.header)
                point.pose.pose.position.x = distance
                point.pose.pose.orientation.w = 1
                point.velocity = speed_kph / 3.6
                local.points.append(point)
            objects = Objects()
            objects.header.stamp, objects.header.frame_id = stamp, "map"
            obj = Object(id=42, label="vehicle")
            obj.bbox_pose.position.x, obj.bbox_pose.position.y, obj.bbox_pose.position.z = x + 8, y - 2, 1.75
            obj.position = deepcopy(obj.bbox_pose.position)
            obj.bbox_pose.orientation.w = 1
            obj.bbox_size.x, obj.bbox_size.y, obj.bbox_size.z = 4, 2, 1.5
            objects.objects.append(obj)
            predictions = []
            for horizon in range(10):
                prediction = deepcopy(objects)
                prediction.objects[0].bbox_pose.position.x += horizon + 1
                prediction.objects[0].position = deepcopy(prediction.objects[0].bbox_pose.position)
                predictions.append(prediction)
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udp:
                def publish_batch():
                    nonlocal stamp
                    stamp = rospy.Time.now()
                    struct.pack_into("<II", packet, 27, stamp.secs, stamp.nsecs)
                    struct.pack_into("<II", object_packet, 30, stamp.secs, stamp.nsecs)
                    ego.header.stamp = stamp
                    transform.header.stamp = stamp
                    local.header.stamp = stamp
                    for point in local.points:
                        point.pose.header.stamp = stamp
                    objects.header.stamp = stamp
                    for prediction in predictions:
                        prediction.header.stamp = stamp
                    ego_pub.publish(ego)
                    broadcaster.sendTransform(transform)
                    local_pub.publish(local)
                    objects_pub.publish(objects)
                    for publisher, prediction in zip(prediction_pubs, predictions):
                        publisher.publish(prediction)
                    udp.sendto(packet, ("127.0.0.1", 19095))
                    udp.sendto(object_packet, ("127.0.0.1", 17505))
                keys = ["ego", "gt", "gt_marker", "local", "objects", "object_info", "morai_objects"]
                keys += ["prediction_" + str(i) for i in range(10)]
                try:
                    wait_for(lambda: all(key in seen for key in keys)
                             and all(seen[key].header.stamp == stamp for key in ["ego", "gt", "gt_marker", "local", "objects", "object_info", "morai_objects"]
                                     + ["prediction_" + str(i) for i in range(10)])
                             and seen["morai_objects"].header.stamp == seen["object_info"].header.stamp,
                             publish=publish_batch)
                except AssertionError:
                    print("Missing topics:", sorted(set(keys) - set(seen)))
                    print("Fixture stamp:", stamp.to_sec())
                    print("Observed stamps:", {key: value.header.stamp.to_sec() for key, value in seen.items()})
                    raise
                assert abs(seen["gt"].pose.pose.position.x - (x + 0.5)) < 1e-4
                for key in ("ego", "gt_marker"):
                    assert seen[key].type == Marker.LINE_LIST and len(seen[key].points) == 30
                    assert seen[key].header.frame_id == "map"
                assert seen["ego"].color != seen["gt_marker"].color
                assert seen["local"].header.frame_id == "base_link" and seen["local"].frame_locked
                assert seen["local"].header.stamp == stamp and len(seen["local"].points) == 7
                for color, expected in zip(seen["local"].colors,
                        ((1, 0, 0), (1, 0.5, 0), (1, 1, 0), (0, 1, 0), (0, 1, 1), (0, 0, 1), (0, 0, 1))):
                    assert all(abs(a - b) < 1e-6 for a, b in zip((color.r, color.g, color.b), expected))
                detected = seen["objects"]
                assert detected.type == Marker.LINE_LIST and len(detected.points) == 24
                assert detected.header.frame_id == "map" and detected.header.stamp == stamp
                assert not detected.frame_locked and detected.lifetime.to_sec() == 0.5
                assert (detected.points[0].x, detected.points[0].y, detected.points[0].z) == (x + 6, y - 3, 1)
                colors = []
                frozen_predictions = []
                for horizon in range(10):
                    marker = seen["prediction_" + str(horizon)]
                    assert marker.type == Marker.LINE_LIST and len(marker.points) == 24
                    assert marker.ns == "predicted_objects_" + str(horizon) and marker.id == 0
                    assert marker.header.frame_id == "map" and marker.header.stamp == stamp
                    assert not marker.frame_locked and marker.lifetime.to_sec() == 0.5
                    assert marker.points[0].x == x + 7 + horizon and marker.points[0].y == y - 3
                    assert marker.points[0].z == 1, "prediction bbox height was flattened"
                    colors.append((marker.color.r, marker.color.g, marker.color.b))
                    frozen_predictions.append(deepcopy(marker))
                assert len(set(colors)) == 10
                assert all(a[0] < b[0] and a[1] > b[1] for a, b in zip(colors, colors[1:]))
                unknown = seen["morai_objects"]
                assert unknown.type == Marker.LINE_LIST and unknown.header.frame_id == "base_link"
                assert seen["object_info"].header.frame_id == "map"
                assert all(math.isfinite(value) for point in unknown.points for value in (point.x, point.y, point.z))
                gt_pose = seen["gt"].pose.pose
                q = gt_pose.orientation
                inverse_rotation = quaternion_matrix((q.x, q.y, q.z, q.w))[:3, :3].T
                actor_position = seen["object_info"].npc_list[0].position
                expected_origin = inverse_rotation.dot((actor_position.x - gt_pose.position.x,
                    actor_position.y - gt_pose.position.y, actor_position.z - gt_pose.position.z))
                assert any(math.dist((point.x, point.y, point.z), expected_origin) < 1e-3 for point in unknown.points)

                def publish_object_packet():
                    gt_stamp = rospy.Time.now()
                    struct.pack_into("<II", packet, 27, gt_stamp.secs, gt_stamp.nsecs)
                    struct.pack_into("<II", object_packet, 30, gt_stamp.secs, gt_stamp.nsecs)
                    udp.sendto(packet, ("127.0.0.1", 19095))
                    udp.sendto(object_packet, ("127.0.0.1", 17505))

                struct.pack_into("<hh7f", object_packet, 38, 51, 1, x - 4, y + 12, 28.5, 90, 4.635, 1.892, 2.434)
                wait_for(lambda: len(seen["morai_objects"].points) == 24
                         and seen["morai_objects"].header.stamp == seen["object_info"].header.stamp,
                         publish=publish_object_packet)
                known = seen["morai_objects"]
                assert known.header.frame_id == "base_link"
                edge_lengths = {round(math.dist((a.x, a.y, a.z), (b.x, b.y, b.z)), 3)
                                for a, b in zip(known.points[::2], known.points[1::2])}
                assert edge_lengths == {4.635, 2.161, 1.644}, "simulator specification used instead of actual bbox"
                assert abs(min(point.z for point in known.points) + 0.3685) < 1e-4
                object_packet[38:-2] = bytes(2120)
                wait_for(lambda: seen["morai_objects"].action == Marker.DELETE,
                         publish=publish_object_packet)

            ego.header.stamp = rospy.Time.now()
            ego.pose.pose.position.x, ego.pose.pose.position.y = x + 50, y + 50
            ego.pose.pose.orientation.z, ego.pose.pose.orientation.w = 0, 1
            transform.header = deepcopy(ego.header)
            transform.transform.translation.x, transform.transform.translation.y = x + 50, y + 50
            transform.transform.rotation = deepcopy(ego.pose.pose.orientation)
            def publish_moved_ego():
                ego_pub.publish(ego)
                broadcaster.sendTransform(transform)
            wait_for(lambda: seen["ego"].header.stamp == ego.header.stamp, publish=publish_moved_ego)
            for horizon, original in enumerate(frozen_predictions):
                marker = seen["prediction_" + str(horizon)]
                assert marker.points == original.points and marker.header == original.header

            clear_stamp = rospy.Time.now()
            empty = Objects()
            empty.header.stamp, empty.header.frame_id = clear_stamp, "map"
            for prediction in predictions[1:]:
                prediction.header.stamp = clear_stamp
            def publish_independent_clear():
                prediction_pubs[0].publish(empty)
                for publisher, prediction in zip(prediction_pubs[1:], predictions[1:]):
                    publisher.publish(prediction)
            wait_for(lambda: seen["prediction_0"].action == Marker.DELETE
                     and all(seen["prediction_" + str(i)].action == Marker.ADD
                             and seen["prediction_" + str(i)].header.stamp == clear_stamp for i in range(1, 10)),
                     publish=publish_independent_clear)
            empty.header.stamp = rospy.Time.now()
            wait_for(lambda: all(seen["prediction_" + str(i)].action == Marker.DELETE for i in range(10)),
                     publish=lambda: [publisher.publish(empty) for publisher in prediction_pubs])

            time.sleep(0.1)
            before = counts["objects"]
            old_frame = deepcopy(objects)
            old_frame.header.stamp = rospy.Time.now()
            old_frame.header.frame_id = "base_link"
            objects_pub.publish(old_frame)
            prediction_counts = [counts["prediction_" + str(i)] for i in range(10)]
            for publisher in prediction_pubs:
                publisher.publish(old_frame)
            time.sleep(0.15)
            assert counts["objects"] == before, "old base_link object input emitted a map marker"
            assert prediction_counts == [counts["prediction_" + str(i)] for i in range(10)], \
                "old base_link prediction input emitted a map marker"
            before = counts["objects"]
            stale = deepcopy(objects)
            stale.header.stamp = rospy.Time.now() - rospy.Duration(5)
            objects_pub.publish(stale)
            time.sleep(0.15)
            assert counts["objects"] == before, "stale object input emitted a marker"
            empty.header.stamp = rospy.Time.now()
            wait_for(lambda: seen["objects"].action == Marker.DELETE, publish=lambda: objects_pub.publish(empty))
            local_clear = LocalPath(header=deepcopy(empty.header))
            local_clear.header.frame_id = local.header.frame_id
            wait_for(lambda: seen["local"].action == Marker.DELETE, publish=lambda: local_pub.publish(local_clear))
            route_clear = GlobalPath()
            route_clear.header.stamp, route_clear.header.frame_id = rospy.Time.now(), "map"
            wait_for(lambda: seen["route"].action == Marker.DELETE, publish=lambda: route_pub.publish(route_clear))
            assert counts["map"] == 1 and counts["map_late"] == 1, "static map was rebuilt or republished"
            assert launch.poll() is None
            assert nodes_are_alive(), "an input failure stopped a visualizer node"
            print("PASS: all configured nodes, native Marker topics, latched map/colors, route mesh/clear, ego/GT geometry, local speed gradient, "
                  "object wireframes, calibrated GT bbox/clear, 10 independent prediction horizons, source stamp after "
                  "ego motion, stale input gating, traffic-light state/range, and node survival after missing/malformed input")
            for subscriber in subscribers:
                subscriber.unregister()
        finally:
            os.killpg(launch.pid, signal.SIGINT)
            try:
                launch.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(launch.pid, signal.SIGKILL)
                launch.wait()
            log.seek(0)
            output = log.read()
            Path("/tmp/visualizer-launch-check.log").write_text(output)
            print(output[-5000:])


if __name__ == "__main__":
    main()
