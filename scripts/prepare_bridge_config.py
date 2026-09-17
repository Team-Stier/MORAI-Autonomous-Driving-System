#!/usr/bin/env python3
import os
import sys

import yaml


def merge(base, override):
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            merge(base[key], value)
        else:
            base[key] = value


if len(sys.argv) != 4:
    raise SystemExit("usage: prepare_bridge_config.py BASE OVERRIDE OUTPUT")

with open(sys.argv[1], encoding="utf-8") as stream:
    config = yaml.safe_load(stream) or {}
with open(sys.argv[2], encoding="utf-8") as stream:
    merge(config, yaml.safe_load(stream) or {})

network = config.setdefault("network_launch", {})
remote = network.setdefault("remote_pc", {})
if os.environ.get("MORAI_BRIDGE_IP"):
    remote["bridge_ip"] = os.environ["MORAI_BRIDGE_IP"]
if os.environ.get("MORAI_SIMULATOR_IP"):
    remote["simulator_ip"] = os.environ["MORAI_SIMULATOR_IP"]

with open(sys.argv[3], "w", encoding="utf-8") as stream:
    yaml.safe_dump(config, stream, sort_keys=False)
