#!/usr/bin/env bash
set -eo pipefail

WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WORKSPACE"

source /opt/ros/noetic/setup.bash
/opt/ros/noetic/bin/rosnode kill -a >/dev/null 2>&1 || true

BRIDGE_PROCESSES=(
    "MORAI-UDP-Bridge-Node/scripts/network_launch.py"
    "/opt/ros/noetic/bin/roslaunch"
    "/opt/ros/noetic/bin/rosmaster"
    "/opt/ros/noetic/lib/rosout/rosout"
    "/morai_udp_bridge_node"
)
for pattern in "${BRIDGE_PROCESSES[@]}"; do
    pkill -TERM -f "$pattern" 2>/dev/null || true
done
for _ in {1..20}; do
    alive=false
    for pattern in "${BRIDGE_PROCESSES[@]}"; do
        pgrep -f "$pattern" >/dev/null && alive=true
    done
    "$alive" || break
    sleep 0.1
done
for pattern in "${BRIDGE_PROCESSES[@]}"; do
    pkill -KILL -f "$pattern" 2>/dev/null || true
done

if [[ ! -f devel/setup.bash ]]; then
    catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo
fi
source devel/setup.bash
set -u

python3 scripts/prepare_bridge_config.py \
    src/MORAI-UDP-Bridge-Node/config/config.yaml \
    config/bridge.yaml /tmp/morai-bridge-workspace.yaml

exec src/MORAI-UDP-Bridge-Node/launch.sh \
    config:=/tmp/morai-bridge-workspace.yaml
