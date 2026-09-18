#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/noetic/setup.bash
WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source "$WORKSPACE/devel/setup.bash"
exec roslaunch morai_path_tracking controller_stack.launch
