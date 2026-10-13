#!/usr/bin/env bash
set -eo pipefail
PACKAGE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$PACKAGE_DIR/../.." && pwd)"
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
source /opt/ros/noetic/setup.bash
source "$WORKSPACE_DIR/devel/setup.bash"
exec roslaunch --wait visualizer visualizer.launch "$@"
