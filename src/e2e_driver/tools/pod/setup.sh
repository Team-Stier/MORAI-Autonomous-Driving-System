#!/usr/bin/env bash
# Pod boot (RunPod, image ros:noetic-ros-base-focal, network volume at /workspace, env NVIDIA_DRIVER_CAPABILITIES=all;
# create_pod.sh makes this the start command). Idempotent: packages and MORAI go to the container disk on every boot,
# the catkin build and the venvs to the volume on the first boot only. Volume layout: README.md.
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd); V=${V:-/workspace}; WS=$V/MORAI-Autonomous-Driving-System
export DEBIAN_FRONTEND=noninteractive UV_PYTHON_INSTALL_DIR=$V/uv-python UV_CACHE_DIR=$V/uv-cache
log() { echo "[setup $(date +%T)] $*"; }

log packages
apt-get update -qq
# MORAI Linux: Vulkan loader (the NVIDIA ICD comes with the driver capabilities), X libs for Xvfb, ko_KR locale (the
# simulator's date parsing); the 10/26 pod list on focal names. Triton (engine) needs gcc and python3-dev.
apt-get install -y -qq --no-install-recommends openssh-server xvfb libvulkan1 vulkan-tools libgtk-3-0 libglu1-mesa \
  libxcursor1 libxrandr2 libxinerama1 libxi6 libxss1 libxkbcommon-x11-0 libasound2 locales python3-pip python3-yaml \
  rsync psmisc procps iproute2 bc util-linux gcc python3-dev ros-noetic-compressed-image-transport > /dev/null
locale-gen ko_KR.UTF-8 > /dev/null

# ssh: GraphQL-deployed pods get no PUBLIC_KEY; README "Access" puts the key in through ssh.runpod.io
mkdir -p ~/.ssh /run/sshd && chmod 700 ~/.ssh
[[ -n ${PUBLIC_KEY:-} ]] && ! grep -qF "$PUBLIC_KEY" ~/.ssh/authorized_keys 2>/dev/null && echo "$PUBLIC_KEY" >> ~/.ssh/authorized_keys
service ssh start > /dev/null

log MORAI
[[ -e /home/stier ]] || ln -s /root /home/stier              # the bc-250 copy's prefs and SaveFile name /home/stier
if [[ ! -x /root/Workspaces/scratches/morai/MoraiLauncher_Lin_Data/Project/Simulator/Simulator_v.S4.251001.MolitComp03_Linux/Simulator.real ]]; then
  mkdir -p /root/Workspaces/scratches && tar -xf $V/morai/morai.tar -C /root/Workspaces/scratches
fi
mkdir -p /root/.config && tar -xf $V/morai/config.tar -C /root/.config  # unity3d/MORAI (login prefs), morai-stier/sim-args
mkdir -p /root/.config/morai-stier && echo "-force-vulkan -force-gfx-mt -job-worker-count 8" > /root/.config/morai-stier/sim-args   # 10/26: Mono crash fix
cp $WS/src/e2e_driver/tools/msc.py /root/morai-eval-msc.py

log ROS build
source /opt/ros/noetic/setup.bash
PKGS="morai_msgs;interfaces;node_pipeline;velodyne;velodyne_driver;velodyne_laserscan;velodyne_msgs;velodyne_pcl;velodyne_pointcloud;morai_udp_bridge;tf_publisher;lidar_leveler;localization;section_annotator;vlp16_bev;e2e_driver;speed_filter"   # run.sh EVAL_PACKAGES
cd $WS
rosdep update --rosdistro noetic > /dev/null 2>&1 || true
# rosdep keys of the evaluation packages only (others in src need what the stack does not run, e.g. onnxruntime)
paths=$(for p in ${PKGS//;/ }; do d=$(grep -rl --include=package.xml "<name>$p</name>" src | head -1); dirname "$d"; done | sort -u)
rosdep install -y -q --from-paths $paths --ignore-src --rosdistro noetic > /dev/null
[[ -r devel/setup.bash ]] || catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCATKIN_WHITELIST_PACKAGES="$PKGS" -j"$(nproc --all | awk '{print ($1 > 16) ? 16 : $1}')" > $V/catkin.log 2>&1

log venvs
python3 -m pip install -q uv
export PATH=$HOME/.local/bin:$PATH
[[ -x $V/venv-rl/bin/python ]] || { uv venv -q --python 3.12 $V/venv-rl && uv pip install -q --python $V/venv-rl/bin/python \
  grpcio protobuf -r $V/Alpamayo-Cosmos/tools/scorer/requirements.txt; }
# the engine: winpc's ~/morai-eval/venv-engine frozen as is (torch 2.14.1+cu130, flashinfer 0.7.2.post1, cutlass-dsl, model deps)
[[ -x $V/venv-engine/bin/python ]] || { uv venv -q --python 3.12 $V/venv-engine && uv pip install -q --python $V/venv-engine/bin/python \
  --no-deps --index-strategy unsafe-best-match --extra-index-url https://download.pytorch.org/whl/cu130 -r $HERE/engine-requirements.txt; }
log done
