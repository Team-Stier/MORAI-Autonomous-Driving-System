#!/usr/bin/env bash
# #VISUALIZATION_ONLY: 환경 로드와 표시 노드 실행만 수행한다. 차량 명령은 발행하지 않는다.
set -eo pipefail
PACKAGE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$PACKAGE_DIR/../.." && pwd)"
source /opt/ros/noetic/setup.bash
# #SOURCE_OR_INSTALL: 소스에서는 devel overlay, install/share에서는 설치 overlay다.
if [[ -r "$WORKSPACE_DIR/devel/setup.bash" ]]; then
    source "$WORKSPACE_DIR/devel/setup.bash"
elif [[ -r "$WORKSPACE_DIR/setup.bash" ]]; then
    source "$WORKSPACE_DIR/setup.bash"
else
    printf 'visualizer: workspace overlay is unavailable at %s\n' "$WORKSPACE_DIR" >&2
    exit 1
fi
exec roslaunch visualizer live_lidar.launch "$@"
