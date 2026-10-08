#!/usr/bin/env python3
"""test_yolo_traffic_light.cpp가 쓰는 ONNX fixture를 만든다. onnx 패키지 없이 protobuf를 직접 쓴다.

모델은 입력 images [1,3,1280,1280] float32를 선언하고, Constant 노드가 고정 YOLO 출력 output0을 낸다.
"""
import os
import struct
import sys


def varint(value):
    out = bytearray()
    while True:
        bits = value & 0x7F
        value >>= 7
        if value:
            out.append(bits | 0x80)
        else:
            out.append(bits)
            return bytes(out)


def field_varint(number, value):
    return varint(number << 3) + varint(value)


def field_bytes(number, payload):
    if isinstance(payload, str):
        payload = payload.encode()
    return varint((number << 3) | 2) + varint(len(payload)) + payload


def tensor_type(shape):
    dims = b"".join(field_bytes(1, field_varint(1, size)) for size in shape)
    tensor = field_varint(1, 1) + field_bytes(2, dims)  # elem_type FLOAT, shape
    return field_bytes(1, tensor)


def value_info(name, shape):
    return field_bytes(1, name) + field_bytes(2, tensor_type(shape))


def constant_model(shape, values):
    raw = struct.pack("<%df" % len(values), *values)
    tensor = b"".join(field_varint(1, size) for size in shape) + field_varint(2, 1) + field_bytes(9, raw)
    attribute = field_bytes(1, "value") + field_bytes(5, tensor) + field_varint(20, 4)
    node = field_bytes(2, "output0") + field_bytes(3, "fixture_output") + field_bytes(4, "Constant") + \
        field_bytes(5, attribute)
    graph = field_bytes(1, node) + field_bytes(2, "fixture") + \
        field_bytes(11, value_info("images", [1, 3, 1280, 1280])) + field_bytes(12, value_info("output0", shape))
    opset = field_bytes(1, "") + field_varint(2, 17)
    return field_varint(1, 9) + field_bytes(2, "traffic_light_fixture") + field_bytes(7, graph) + field_bytes(8, opset)


def channels_first(candidates, classes=4):
    """candidates: (cx, cy, w, h, class_index, score). 반환은 [1, 4+classes, N] 행 우선 값이다."""
    count = len(candidates)
    values = [0.0] * ((4 + classes) * count)
    for index, (cx, cy, w, h, class_index, score) in enumerate(candidates):
        for channel, value in enumerate((cx, cy, w, h)):
            values[channel * count + index] = value
        values[(4 + class_index) * count + index] = score
    return [1, 4 + classes, count], values


def main(directory):
    # go(0)와 left_turn(1)이 서로 겹치지 않게 검출된다.
    shape, values = channels_first([(600.0, 640.0, 60.0, 60.0, 0, 0.9), (680.0, 640.0, 60.0, 60.0, 1, 0.8)])
    fixtures = {
        "yolo_green_left.onnx": constant_model(shape, values),
        # 클래스 4개 계약과 맞지 않는 7채널 출력이다.
        "yolo_wrong_channels.onnx": constant_model([1, 7, 1], [0.0] * 7),
    }
    for name, payload in fixtures.items():
        with open(os.path.join(directory, name), "wb") as stream:
            stream.write(payload)
        print(os.path.join(directory, name))


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else os.path.dirname(os.path.abspath(__file__)))
