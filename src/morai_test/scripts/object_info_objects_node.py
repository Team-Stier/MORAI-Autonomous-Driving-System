#!/usr/bin/env python3
"""Manual /objects adapter: native map-state mode or UDP packet inspection.

Native mode uses the existing local MORAI SDK and native source timestamps.
UDP has only heading, so its raw body motion is not marked as map-available.
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
            or struct.unpack_from("<i", packet, 14)[0] not in (len(packet) - 40, len(packet) - 32)
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
    def __init__(self, bbox_size_overrides=None):
        self.bbox_size_overrides = bbox_size_overrides or {}
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
            # Explicit simulator metadata for scaled assets; absent keys retain UDP sizes.
            override = self.bbox_size_overrides.get(f"{kind}:{sim_id}")
            if override is not None:
                if len(override) != 3 or not all(math.isfinite(float(v)) and float(v) > 0 for v in override):
                    raise ValueError("bbox override must contain three positive finite full lengths")
                length, width, height = map(float, override)
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
            # UDP motion is body-frame; unavailable without full attitude metadata.
            velocity = [value / 3.6 for value in values[12:15]]
            acceleration = values[15:18]
            obj.velocity.x, obj.velocity.y, obj.velocity.z = map(float, velocity)
            obj.acceleration.x, obj.acceleration.y, obj.acceleration.z = map(float, acceleration)
            obj.velocity_available = obj.acceleration_available = False
            output.objects.append(obj)
        self.ids = active_ids
        return output

    def convert_native(self, snapshot, yaw_offset_rad):
        from interfaces.msg import Object, Objects
        import rospy

        def vector(value):
            result = tuple(float(value.get(axis, 0.0)) for axis in ("x", "y", "z"))
            if not all(math.isfinite(v) for v in result):
                raise ValueError("non-finite native vector")
            return result

        def rotation(value):
            roll, pitch, heading = map(math.radians, vector(value))
            return roll, pitch, heading + yaw_offset_rad

        ticks = int(snapshot["timestamp"])
        if ticks <= 0 or not math.isfinite(yaw_offset_rad):
            raise ValueError("invalid native source time or yaw offset")
        output = Objects()
        output.header.frame_id = "map"
        seconds, microseconds = divmod(ticks, 1000000)
        output.header.stamp = rospy.Time(seconds, microseconds * 1000)
        active_ids = {}
        kinds = {"OBJECT_TYPE_PEDESTRIAN": (0, "pedestrian"),
                 "OBJECT_TYPE_VEHICLE": (1, "vehicle"),
                 "OBJECT_TYPE_OBSTACLE": (2, "obstacle")}
        for state in snapshot["states"]:
            actor = state["actor_info"]
            sim_id = actor["id"]["value"]
            if sim_id == "Ego" or actor["object_type"] not in kinds:
                continue
            kind, label = kinds[actor["object_type"]]
            key = (kind, sim_id)
            if key in active_ids:
                raise ValueError("duplicate native actor")
            bbox = state["bounding_box"]
            extent = vector(bbox["extent"])
            # #GT_BBOX_AXES
            # 25.S4 차량 extent는 폭·길이·높이, 보행자는 폭·높이·길이 순서다.
            # /objects의 전방 길이·좌측 폭·위 높이로 정규화하고, 장애물은
            # bbox 자체 x/y/z 축을 유지한다. 중심은 native bbox 위치다.
            axes = (1, 0, 2) if kind == 1 else (2, 0, 1) if kind == 0 else (0, 1, 2)
            size = tuple(2 * extent[axis] for axis in axes)
            size = tuple(map(float, self.bbox_size_overrides.get(f"{kind}:{sim_id}", size)))
            if len(size) != 3 or not all(math.isfinite(v) and v > 0 for v in size):
                raise ValueError("invalid native bbox size")
            if key not in self.ids:
                self.ids[key] = self.next_id
                self.next_id += 1
            obj = Object()
            obj.id = active_ids[key] = self.ids[key]
            obj.label = label
            obj.position.x, obj.position.y, obj.position.z = vector(bbox["location"])
            obj.bbox_pose.position = obj.position
            obj.bbox_size.x, obj.bbox_size.y, obj.bbox_size.z = size
            roll, pitch, yaw = rotation(bbox["rotation"])
            cr, sr = math.cos(roll / 2), math.sin(roll / 2)
            cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
            cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
            q = obj.bbox_pose.orientation
            q.x, q.y = sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy
            q.z, q.w = cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy
            # #GT_MAP_MOTION
            # global_velocity는 map 축 절대 속도[km/h]여서 단위만 바꾼다.
            # 차체 축 acceleration[m/s²]에는 같은 snapshot의 전체 자세
            # Rz(heading+grid_offset) Ry(pitch) Rx(roll)을 적용한다.
            # 경사에서도 3축을 보존하며 header는 native 원본 us 시각이다.
            obj.velocity.x, obj.velocity.y, obj.velocity.z = (v / 3.6 for v in vector(state["global_velocity"]))
            x, y, z = vector(state["acceleration"])
            roll, pitch, yaw = rotation(state["transform"]["rotation"])
            cr, sr = math.cos(roll), math.sin(roll)
            cp, sp = math.cos(pitch), math.sin(pitch)
            cy, sy = math.cos(yaw), math.sin(yaw)
            y, z = cr * y - sr * z, sr * y + cr * z
            x, z = cp * x + sp * z, -sp * x + cp * z
            obj.acceleration.x, obj.acceleration.y, obj.acceleration.z = cy * x - sy * y, sy * x + cy * y, z
            # #GT_PEDESTRIAN_MOTION
            # 25.S4 보행자는 global_velocity가 빈 메시지인 동안에도 velocity에
            # 차체 전방 속도[km/h]를 제공한다. 같은 관측의 전체 자세와 격자
            # 보정으로 map 절대 속도[m/s]로 회전한다. 차량·장애물은 기존
            # global_velocity 경로를 유지하며 관측 시각·bbox·가속도는 바꾸지 않는다.
            if kind == 0:
                vx, vy, vz = (v / 3.6 for v in vector(state["velocity"]))
                vy, vz = cr * vy - sr * vz, sr * vy + cr * vz
                vx, vz = cp * vx + sp * vz, -sp * vx + cp * vz
                obj.velocity.x, obj.velocity.y, obj.velocity.z = cy * vx - sy * vy, sy * vx + cy * vy, vz
            obj.velocity_available = obj.acceleration_available = True
            output.objects.append(obj)
        self.ids = active_ids
        return output


def receive_native(converter):
    import importlib.util
    import os
    import rospy
    from interfaces.msg import Objects

    # 기존 native SDK의 명시된 경로만 사용한다. 새 노드나 환경 설치는 없다.
    module_path = rospy.get_param("~native_api_module")
    os.environ["MORAI_NATIVE_DESCRIPTOR_DIR"] = rospy.get_param("~native_descriptor_dir")
    os.environ["MORAI_NATIVE_GRPC_ENDPOINT"] = rospy.get_param("~native_endpoint", "127.0.0.1:7789")
    spec = importlib.util.spec_from_file_location("morai_native_objects_api", module_path)
    api = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(api)
    yaw_offset = float(rospy.get_param("~native_yaw_offset_rad", -0.023509))
    timeout = float(rospy.get_param("~native_timeout_sec", 0.15))
    frequency = float(rospy.get_param("~native_rate_hz", 20.0))
    if not all(math.isfinite(v) and v > 0 for v in (timeout, frequency)):
        raise ValueError("invalid native timeout or rate")
    publisher = rospy.Publisher("/objects", Objects, queue_size=1)
    rate = rospy.Rate(frequency)
    while not rospy.is_shutdown():
        snapshot = api.rpc("GetAllActorsState", {"vehicle": True, "obstacle": True, "pedestrian": True}, timeout=timeout)
        publisher.publish(converter.convert_native(snapshot, yaw_offset))
        rate.sleep()


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
            converter.bbox_size_overrides = rospy.get_param("~bbox_size_overrides", {})
            source = rospy.get_param("~source", "udp")
            if source == "native_grpc":
                receive_native(converter)
            elif source == "udp":
                rospy.logwarn("UDP body motion is unavailable as map motion; use native_grpc for closed-loop tests")
                receive(converter)
            else:
                raise ValueError("unsupported GT source")
        except Exception as error:
            try:
                rospy.logerr("MORAI test objects retrying: %s", error)
            except Exception:
                print("MORAI test objects retrying:", error, flush=True)
            time.sleep(1.0)


if __name__ == "__main__":
    main()
