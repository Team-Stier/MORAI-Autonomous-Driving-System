#!/usr/bin/env bash
set -euo pipefail

PACKAGE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$PACKAGE_DIR/../.." && pwd)"

source /opt/ros/noetic/setup.bash
source "$WORKSPACE_DIR/devel/setup.bash"
exec roslaunch localization localization.launch "$@"
