#!/usr/bin/env bash
set -eo pipefail
PACKAGE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$PACKAGE_DIR/../.." && pwd)"
source /opt/ros/noetic/setup.bash
source "$WORKSPACE_DIR/devel/setup.bash"
ONNXRUNTIME_ROOT="${ONNXRUNTIME_ROOT:-/opt/onnxruntime-gpu/1.19.2}"
export LD_LIBRARY_PATH="$ONNXRUNTIME_ROOT/lib:$ONNXRUNTIME_ROOT/cuda/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
python3 - "$PACKAGE_DIR" <<'PY'
import json
from pathlib import Path
import sys
from urllib.parse import urljoin
from urllib.request import urlopen

import yaml

package_dir = Path(sys.argv[1])
try:
    config_dir = package_dir / "config" / "traffic_light"
    selection = yaml.safe_load((config_dir / "selection.yaml").read_text())["recognize"]
    selected_cores = set(selection["section_core_map"].values())
    selected_cores.update(rule["core"] for rule in selection["health_core_map"])
    if "yolo_traffic_light_mac.hpp" in selected_cores:
        parameters = yaml.safe_load(
            (config_dir / "parameters" / "yolo_traffic_light_mac.yaml").read_text())
        health_url = urljoin(parameters["inference_url"], "/health")
        with urlopen(health_url, timeout=parameters["timeout_ms"] / 1000.0) as response:
            health = json.load(response)
        if (health.get("backend") != "CoreMLExecutionProvider"
                or health.get("compute_units") != "CPUAndGPU"):
            raise ValueError("Mac inference backend must be CoreMLExecutionProvider with CPUAndGPU")
except Exception as error:
    print(f"traffic_light readiness failed: {error}", file=sys.stderr)
    print("On macOS, run: bash src/traffic_light/start_mac_gpu.sh", file=sys.stderr)
    sys.exit(1)
PY
exec roslaunch traffic_light traffic_light.launch "$@"
