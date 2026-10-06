#!/usr/bin/env bash
set -eo pipefail
PACKAGE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$PACKAGE_DIR/../.." && pwd)"
source /opt/ros/noetic/setup.bash
source "$WORKSPACE_DIR/devel/setup.bash"
ORT_ROOT=/opt/onnxruntime-gpu/1.19.2
export LD_LIBRARY_PATH="$ORT_ROOT/lib:$ORT_ROOT/cuda/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
exec roslaunch traffic_light traffic_light.launch "$@"
