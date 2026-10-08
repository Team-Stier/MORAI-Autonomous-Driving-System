#!/usr/bin/env python3
"""ClusterMatch NPC 모델 C++ 초기화 데이터(.inc)를 models/npc_models.bin으로 변환한다.

입력은 npc_models.inc와 그것이 include하는 npc_points_*.inc, 검증용 npc_models.json이다.
모델별 점은 npc_models.json의 sample_float32_sha256과 대조한다.

출력 형식(little-endian):
  magic "NPCM", u32 version=1, u32 model_count, u32 point_count
  model_count × { u16 label_len, label, u16 semantic_len, semantic_label, u32 begin, u32 count,
                  u8 collider_available, f64 center[3], f64 orientation_xyzw[4], f64 size[3] }
  point_count × f32 xyz[3]
"""
import argparse
import hashlib
import json
import re
import struct
import sys

NUMBER = r"[-+0-9.eE]+"
ASSET = re.compile(r'\{"([^"]+)", (\d+), (\d+), (true|false), \{(' + NUMBER + r'), (' + NUMBER + r'), (' + NUMBER + r')\}, '
                   r'\{(' + NUMBER + r'), (' + NUMBER + r'), (' + NUMBER + r'), (' + NUMBER + r')\}, '
                   r'\{(' + NUMBER + r'), (' + NUMBER + r'), (' + NUMBER + r')\}\},')
ALIAS = re.compile(r'\{"([^"]+)", "([^"]+)"\},')
POINT = re.compile(r"\{(" + NUMBER + r")f, (" + NUMBER + r")f, (" + NUMBER + r")f\},")


def read_points(paths):
    points = bytearray()
    count = 0
    for path in paths:
        with open(path, encoding="utf-8") as stream:
            for line in stream:
                match = POINT.fullmatch(line.strip())
                if not match:
                    raise ValueError(f"{path}: unexpected point line: {line.strip()[:80]}")
                points += struct.pack("<3f", *(float(value) for value in match.groups()))
                count += 1
    return bytes(points), count


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--models-inc", required=True)
    parser.add_argument("--points-inc", nargs="+", required=True)
    parser.add_argument("--metadata", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    text = open(args.models_inc, encoding="utf-8").read()
    assets = ASSET.findall(text)
    aliases = dict(ALIAS.findall(text))
    points, point_count = read_points(args.points_inc)
    metadata = json.load(open(args.metadata, encoding="utf-8"))
    if len(assets) != metadata["model_count"] or point_count != metadata["point_count"]:
        raise ValueError(f"expected {metadata['model_count']} models and {metadata['point_count']} points, "
                         f"found {len(assets)} and {point_count}")
    digests = {model["actor_label"]: model["sample_float32_sha256"] for model in metadata["models"]}

    output = bytearray(b"NPCM" + struct.pack("<III", 1, len(assets), point_count))
    for label, begin, count, collider, *values in assets:
        begin, count = int(begin), int(count)
        if begin + count > point_count:
            raise ValueError(f"{label}: point range exceeds data")
        digest = hashlib.sha256(points[begin * 12:(begin + count) * 12]).hexdigest()
        if digests.get(label) != digest:
            raise ValueError(f"{label}: float32 sha256 {digest} does not match metadata")
        semantic = aliases.get(label, "")
        for name in (label, semantic):
            encoded = name.encode("utf-8")
            output += struct.pack("<H", len(encoded)) + encoded
        output += struct.pack("<IIB", begin, count, collider == "true")
        output += struct.pack("<10d", *(float(value) for value in values))
    output += points
    with open(args.output, "wb") as stream:
        stream.write(output)
    print(f"wrote {args.output}: {len(assets)} models, {point_count} points, {len(output)} bytes")
    return 0


if __name__ == "__main__":
    sys.exit(main())
