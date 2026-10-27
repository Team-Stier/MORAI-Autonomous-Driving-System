#!/usr/bin/env bash
# Pod boot (RunPod, image ros:noetic-ros-base-focal, network volume at /workspace, env NVIDIA_DRIVER_CAPABILITIES=all;
# create_pod.sh / eval_pod.sh make this the start command). Idempotent. Volume layout: README.md.
#   first boot (a CPU pod is enough): apt lists and .deb files land in $V/apt, rosdep, the catkin build and the venvs on
#     the volume; $V/apt/packages.txt = every package installed after it
#   later boots: install packages.txt from the cached .deb files (no download, no rosdep), unpack MORAI to the container
#     disk, sshd; the build and venvs are reused. REBUILD=1 runs catkin_make again (C++ changes)
# Writes $V/setup.done.<container hostname> at the end (eval_pod.sh waits for it).
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd); V=${V:-/workspace}; WS=$V/MORAI-Autonomous-Driving-System
export DEBIAN_FRONTEND=noninteractive UV_PYTHON_INSTALL_DIR=$V/uv-python UV_CACHE_DIR=$V/uv-cache
log() { echo "[setup $(date +%T) +${SECONDS}s] $*"; }

log packages
mkdir -p $V/apt/archives/partial $V/apt/lists/partial
# apt on the volume for every apt call (rosdep's too); the image's docker-clean hook only empties /var/cache/apt
printf 'Dir::Cache::Archives "%s/apt/archives";\nDir::State::Lists "%s/apt/lists";\nAPT::Keep-Downloaded-Packages "true";\nBinary::apt::APT::Keep-Downloaded-Packages "true";\n' \
  "$V" "$V" > /etc/apt/apt.conf.d/99volume
FIRST=0; [[ -s $V/apt/packages.txt ]] || FIRST=1
if (( FIRST )); then
  apt-get update -qq
  # MORAI Linux: Vulkan loader (the NVIDIA ICD comes with the driver capabilities), X libs for Xvfb, ko_KR locale (the
  # simulator's date parsing). Triton (engine) needs gcc and python3-dev. viewer.py: python3-xlib, python3-pil; videos: ffmpeg
  apt-get install -y -qq --no-install-recommends openssh-server xvfb libvulkan1 vulkan-tools libgtk-3-0 libglu1-mesa \
    libxcursor1 libxrandr2 libxinerama1 libxi6 libxss1 libxkbcommon-x11-0 libasound2 locales python3-pip python3-yaml \
    rsync psmisc procps iproute2 bc util-linux gcc python3-dev ros-noetic-compressed-image-transport \
    python3-xlib python3-pil ffmpeg curl ca-certificates git > /dev/null
else
  apt-get install -y -qq --no-install-recommends $(cat $V/apt/packages.txt) > /dev/null
fi
locale-gen ko_KR.UTF-8 > /dev/null
# ssh (GraphQL-deployed pods get the key only from the PUBLIC_KEY env that eval_pod.sh sets)
mkdir -p ~/.ssh /run/sshd && chmod 700 ~/.ssh
[[ -n ${PUBLIC_KEY:-} ]] && ! grep -qF "$PUBLIC_KEY" ~/.ssh/authorized_keys 2>/dev/null && echo "$PUBLIC_KEY" >> ~/.ssh/authorized_keys
service ssh start > /dev/null

log MORAI
[[ -e /home/stier ]] || ln -s /root /home/stier              # the bc-250 copy's prefs and SaveFile name /home/stier
if [[ ! -x /root/Workspaces/scratches/morai/MoraiLauncher_Lin_Data/Project/Simulator/Simulator_v.S4.251001.MolitComp03_Linux/Simulator.real ]]; then
  mkdir -p /root/Workspaces/scratches
  for f in $V/morai/part*.tar $V/morai/morai.tar; do [[ -e $f ]] && tar -xf "$f" -C /root/Workspaces/scratches & done; wait
fi
mkdir -p /root/.config && tar -xf $V/morai/config.tar -C /root/.config  # unity3d/MORAI (login prefs), morai-stier/sim-args
mkdir -p /root/.config/morai-stier && echo "-force-vulkan -force-gfx-mt -job-worker-count 8" > /root/.config/morai-stier/sim-args   # 10/26: Mono crash fix
cp $WS/src/e2e_driver/tools/msc.py /root/morai-eval-msc.py
# compile caches of the engine (inductor via TORCHINDUCTOR_CACHE_DIR in eval3.sh, triton, flashinfer / cutlass under ~/.cache)
mkdir -p $V/cache/home-cache $V/cache/triton
[[ -L /root/.cache ]] || { mv /root/.cache /root/.cache.image 2>/dev/null || true; ln -s $V/cache/home-cache /root/.cache; }
[[ -L /root/.triton ]] || ln -sfn $V/cache/triton /root/.triton

log ROS build
set +u; source /opt/ros/noetic/setup.bash; set -u            # ROS's setup reads unset variables
PKGS="morai_msgs;interfaces;node_pipeline;velodyne;velodyne_driver;velodyne_laserscan;velodyne_msgs;velodyne_pcl;velodyne_pointcloud;morai_udp_bridge;tf_publisher;lidar_leveler;localization;section_annotator;vlp16_bev;e2e_driver;speed_filter"   # run.sh EVAL_PACKAGES
cd $WS
if (( FIRST )); then
  rosdep update --rosdistro noetic > /dev/null 2>&1 || true
  # rosdep keys of the evaluation packages only (others in src need what the stack does not run, e.g. onnxruntime)
  paths=$(for p in ${PKGS//;/ }; do d=$(grep -rl --include=package.xml "<name>$p</name>" src | head -1); dirname "$d"; done | sort -u)
  rosdep install -y -q --from-paths $paths --ignore-src --rosdistro noetic > /dev/null
fi
[[ -r devel/setup.bash && ${REBUILD:-0} == 0 ]] || catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCATKIN_WHITELIST_PACKAGES="$PKGS" -j"$(nproc --all | awk '{print ($1 > 16) ? 16 : $1}')" > $V/catkin.log 2>&1

log venvs
if [[ ! -e $V/venv-rl/.ok || ! -e $V/venv-engine/.ok ]]; then     # .ok: the install finished
  python3 -m pip install -q uv
  export PATH=$HOME/.local/bin:$PATH
fi
# managers, scorer, summary, videos (lap_video make), plan overlays
[[ -e $V/venv-rl/.ok ]] || { uv venv -q --clear --python 3.12 $V/venv-rl && uv pip install -q --python $V/venv-rl/bin/python \
  grpcio protobuf pillow pandas matplotlib -r $V/Alpamayo-Cosmos/tools/scorer/requirements.txt && touch $V/venv-rl/.ok; }
# the engine: winpc's ~/morai-eval/venv-engine frozen as is (torch 2.14.1+cu130, flashinfer 0.7.2.post1, cutlass-dsl, model deps)
[[ -e $V/venv-engine/.ok ]] || { uv venv -q --clear --python 3.12 $V/venv-engine && uv pip install -q --python $V/venv-engine/bin/python \
  --no-deps --index-strategy unsafe-best-match --extra-index-url https://download.pytorch.org/whl/cu130 -r $HERE/engine-requirements.txt \
  && touch $V/venv-engine/.ok; }
if (( FIRST )); then
  dpkg-query -W -f='${Package}\n' > $V/apt/packages.txt     # all of them: the image's own are no-ops on later boots
  log "packages for later boots: $(wc -l < $V/apt/packages.txt) in $V/apt/packages.txt"
fi
touch "$V/setup.done.$(hostname)"
log done
