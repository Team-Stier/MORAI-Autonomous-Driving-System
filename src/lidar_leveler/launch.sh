#!/usr/bin/env bash
set -eo pipefail
PACKAGE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$PACKAGE_DIR/../.." && pwd)"
source /opt/ros/noetic/setup.bash
source "$WORKSPACE_DIR/devel/setup.bash"
exec roslaunch lidar_leveler lidar_leveler.launch "$@"
