#!/usr/bin/env bash
# drive_setup.sh (pod, after setup.sh): the competition stack's workspace DRIVE_WS (default /workspace/drive; eval_pod.sh
# rsyncs a checkout of BRANCH there, submodules and LFS models included, and writes its REF) built for stacks.sh
# DRIVER=stack. Idempotent: ONNX Runtime GPU 1.19.2 at /opt/onnxruntime-gpu/1.19.2 with the CUDA 12 / cuDNN 9 libraries
# of the NVIDIA wheels in its cuda/lib (traffic_light's launch.sh puts both on LD_LIBRARY_PATH), rosdep keys of
# DRIVE_WS/src, then catkin_make (every package but visualizer, which the stack does not run). Writes DRIVE_WS/BUILT
# ("<sha of the synced snapshot's SHA file> <REF>") on success; the log is $V/drive_build.log.
set -uo pipefail
V=${V:-/workspace}; D=${DRIVE_WS:-$V/drive}; ORT=/opt/onnxruntime-gpu/1.19.2
export DEBIAN_FRONTEND=noninteractive
log() { echo "[drive_setup $(date +%T) +${SECONDS}s] $*"; }
if [[ ! -e $ORT/cuda/.ok ]]; then
  log "ONNX Runtime GPU"
  mkdir -p "$ORT" && curl -sSL https://github.com/microsoft/onnxruntime/releases/download/v1.19.2/onnxruntime-linux-x64-gpu-1.19.2.tgz |
    tar -xz -C "$ORT" --strip-components=1 || { log "ORT download failed"; exit 1; }
  python3 -m pip install -q --target "$ORT/cuda/pkgs" nvidia-cuda-runtime-cu12==12.4.* nvidia-cublas-cu12==12.4.* \
    nvidia-cudnn-cu12==9.1.* nvidia-cufft-cu12==11.2.* nvidia-curand-cu12==10.3.* nvidia-cuda-nvrtc-cu12==12.4.* > $V/ort_pip.log 2>&1 ||
    { log "CUDA wheels failed ($V/ort_pip.log)"; exit 1; }
  mkdir -p "$ORT/cuda/lib" && find "$ORT/cuda/pkgs/nvidia" -name 'lib*.so*' -exec ln -sf {} "$ORT/cuda/lib/" \; && touch "$ORT/cuda/.ok"
fi
REF=$(cat "$D/REF" 2> /dev/null || echo "?")
cd "$D" || { log "no $D"; exit 1; }
set +u; source /opt/ros/noetic/setup.bash; set -u
log "rosdep ($REF)"
if ! rosdep check --from-paths src --ignore-src --rosdistro noetic --skip-keys visualizer > /dev/null 2>&1; then
  [[ -e /tmp/apt.updated ]] || { apt-get update -qq && touch /tmp/apt.updated; }
  rosdep update --rosdistro noetic > /dev/null 2>&1 || true
  rosdep install -y -q --from-paths src --ignore-src --rosdistro noetic --skip-keys "rviz visualizer" > $V/drive_rosdep.log 2>&1 ||
    log "rosdep install reported errors ($V/drive_rosdep.log); building anyway"
fi
log "catkin_make ($REF)"
rm -f BUILT
find build -type f -name '*.o' -size 0c -delete 2> /dev/null
# --force-cmake: an incremental build over another ref does not regenerate np_generated/<pkg>/cores/<core>.cpp when a
# parameters/<core>.hpp loader is added, so the Core builds silently without parameters (worker-control-transition 10/29)
build() { ONNXRUNTIME_ROOT=$ORT catkin_make --force-cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo -DONNXRUNTIME_ROOT=$ORT -DCATKIN_BLACKLIST_PACKAGES=visualizer \
  -j"$(nproc --all | awk '{print ($1 > 16) ? 16 : $1}')" >> $V/drive_build.log 2>&1; }
: > $V/drive_build.log
# one retry: parallel CMake generation of node_pipeline's generated files can race (BROADCAST-16)
if build || { log "catkin_make failed once, retrying ($(command grep -m1 -E 'error|Error' $V/drive_build.log | cut -c1-160))"; build; }; then
  echo "$(cat SHA 2> /dev/null || echo nosha) $REF" > BUILT; log "built $(cat BUILT)"
else
  log "build FAILED ($REF): $(command grep -m3 -E 'error|Error' $V/drive_build.log | cut -c1-200)"; exit 1
fi
