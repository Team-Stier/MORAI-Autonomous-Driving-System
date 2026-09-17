#!/usr/bin/env bash
set -eo pipefail

WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WORKSPACE"

source /opt/ros/noetic/setup.bash
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
