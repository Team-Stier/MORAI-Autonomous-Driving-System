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

if [[ ! -f devel/setup.bash || ! -f devel/lib/libdriver_nodelet.so || ! -f devel/lib/libtransform_nodelet.so ]]; then
    catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo
fi
source devel/setup.bash
set -u

python3 scripts/prepare_bridge_config.py \
    src/MORAI-UDP-Bridge-Node/config/config.yaml \
    config/bridge.yaml /tmp/morai-bridge-workspace.yaml

# The bridge launch runs in the background, so acquire sudo before it configures the wired NIC.
sudo -v

children=()
cleanup() {
    trap - EXIT
    trap '' INT TERM
    for child in "${children[@]}"; do
        kill -TERM -- "-$child" 2>/dev/null || true
    done
    for _ in {1..20}; do
        alive=false
        for child in "${children[@]}"; do
            kill -0 -- "-$child" 2>/dev/null && alive=true
        done
        "$alive" || break
        sleep 0.1
    done
    for child in "${children[@]}"; do
        kill -KILL -- "-$child" 2>/dev/null || true
    done
    wait "${children[@]}" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

setsid bash src/MORAI-UDP-Bridge-Node/launch.sh \
    config:=/tmp/morai-bridge-workspace.yaml &
bridge_pid=$!
children+=("$bridge_pid")

bridge_ready=false
for _ in {1..300}; do
    kill -0 "$bridge_pid" 2>/dev/null || break
    rosnode list 2>/dev/null | grep -qx /morai_udp_bridge && {
        bridge_ready=true
        break
    }
    sleep 0.1
done
if ! "$bridge_ready"; then
    echo "MORAI UDP bridge did not become ready" >&2
    exit 1
fi

setsid roslaunch velodyne_pointcloud VLP16_points.launch &
velodyne_pid=$!
children+=("$velodyne_pid")

velodyne_ready=false
for _ in {1..300}; do
    kill -0 "$velodyne_pid" 2>/dev/null || break
    loaded="$(rosservice call /velodyne_nodelet_manager/list 2>/dev/null || true)"
    if [[ "$loaded" == *velodyne_nodelet_manager_driver* &&
          "$loaded" == *velodyne_nodelet_manager_transform* &&
          "$loaded" == *velodyne_nodelet_manager_laserscan* ]]; then
        velodyne_ready=true
        break
    fi
    sleep 0.1
done
if ! "$velodyne_ready"; then
    echo "Velodyne nodelets did not become ready" >&2
    exit 1
fi

wait -n "${children[@]}"
