#!/usr/bin/env bash
# stacks.sh up N | down | config n: N closed-loop ROS stacks on one host, stack n for MORAI world n (worlds.sh).
# run.sh cannot be used N times: it kills every ROS process on the host and rebuilds the workspace. So this starts the
# same chain as run.sh after its build (keep the two in step): roscore, sim_clock (/use_sim_time), bridge, velodyne,
# vlp16_bev, tf_publisher, lidar_leveler, localization, section_annotator, e2e_driver, speed_filter.
# Stack n: ROS master 11310+n, every UDP port +(n-1)*1000 (as mk-clone.sh shifts world n's presets: the bridge config and
# the VLP-16 port), model server port 47011 (one batched engine) or 47011+n-1 with ENGINE_PORT_STEP=1, CPU affinity
# STACK_CPUS (all stacks share it), node param world = n (in its requests). Logs and pids: /root/stack/n/.
# DRIVER=stack (campaign default): the competition stack of the drive workspace DRIVE_WS (drive_setup.sh; run.sh's
# chain without visualizer: bridge, velodyne, tf_publisher, lidar_leveler, localization, section_annotator,
# object_detector, traffic_light, path_planner, velocity_planner, control, speed_filter) on wall time (no /use_sim_time,
# no sim_clock), the bridge config from DRIVE_WS's bridge submodule with same-PC addresses. No GT topic reaches it.
# SIM_CLOCK=1 (default with DRIVER=stack): MORAI Linux on a pod runs below real time (rho 0.5-0.85 under Xvfb, the
# competition PC at ~1), so the stack runs on the simulator time instead: /use_sim_time true and WS's
# e2e_driver sim_clock_node.py (/clock = tau estimated from /vehicle/competition_status speed and /sensors/gps/fix, both
# stack inputs; ~state = [wall, tau, rho, n]). The bridge stamps receipts with ros::Time::now, so every node follows tau.
# DRIVER=e2e: the old e2e_driver chain of WS.
# E2E_FAKE_FOLLOW_ROUTE_M passes through (fake backend).
# SYNC=1 (MORAI synchronous mode, eval_manager.py --sync): sync_clock_node.py instead of sim_clock (manager port
# 47100+n), the node in sync mode, and the bridge's command timeout off
# (30 s: a step can take longer than 0.25 s of wall time) with sends at 500 Hz (the command reaches MORAI within 2 ms).
set -uo pipefail
V=${V:-/workspace}; WS=${WS:-$V/MORAI-Autonomous-Driving-System}; R=${STACK_ROOT:-/root/stack}
DRIVER=${DRIVER:-stack}; DRIVE_WS=${DRIVE_WS:-$V/drive}
DRIVE_PKGS="tf_publisher lidar_leveler localization section_annotator object_detector traffic_light path_planner velocity_planner control speed_filter"

config() {  # n -> $R/n/bridge.yaml: the same-PC bridge config with world n's ports
  local n=$1 d=$R/$1; mkdir -p "$d/ros" "$d/tmp"
  local src=$WS/src/e2e_driver/config/bridge_same_pc.yaml
  [[ $DRIVER == stack ]] && src=$DRIVE_WS/src/MORAI-UDP-Bridge-Node/config/config.yaml
  python3 - "$src" "$d/bridge.yaml" $(( (n - 1) * 1000 )) "${SYNC:-0}" <<'PY'
import sys, yaml
src, dst, off, sync = sys.argv[1], sys.argv[2], int(sys.argv[3]), sys.argv[4] == "1"
c = yaml.safe_load(open(src))
c["network_launch"]["launch_option"] = 1                   # same PC: the world runs on this host
c["network_launch"]["same_pc"].update(bridge_ip="127.0.0.1", simulator_ip="127.0.0.1")
if sync:
    c["control"].update(command_timeout_sec=30.0, send_rate_hz=500.0)
for cam in c["cameras"].values():
    cam["port"] += off
for k in ("gps", "imu", "status"):
    c[k]["port"] += off
c["control"]["destination_port"] += off
yaml.safe_dump(c, open(dst, "w"), sort_keys=False)
PY
}

up1() {
  local n=$1 d=$R/$1 off=$(( ($1 - 1) * 1000 ))
  config "$n"
  [[ -e $d/stack.pid ]] && kill -0 "$(cat "$d/stack.pid")" 2>/dev/null && { echo "stack $n already up"; return; }
  if [[ $DRIVER == stack ]]; then
  ROS_MASTER_URI=http://127.0.0.1:$((11310 + n)) ROS_IP=127.0.0.1 ROS_HOME=$d/ros TMPDIR=$d/tmp MORAI_BRIDGE_CONFIG=$d/bridge.yaml \
  ONNXRUNTIME_ROOT=${ONNXRUNTIME_ROOT:-/opt/onnxruntime-gpu/1.19.2} VLP_PORT=$((2368 + off)) MASTER_PORT=$((11310 + n)) PKGS="$DRIVE_PKGS" \
  SIM_CLOCK=${SIM_CLOCK:-1} HWS=$WS \
  setsid taskset -c "${STACK_CPUS:-0-$(($(nproc) - 1))}" bash -c '
    cd "'"$DRIVE_WS"'" && source /opt/ros/noetic/setup.bash && source devel/setup.bash
    roscore -p $MASTER_PORT & for i in $(seq 100); do rosnode list > /dev/null 2>&1 && break; sleep 0.1; done
    if [[ $SIM_CLOCK == 1 ]]; then
      rosparam set /use_sim_time true
      (source "$HWS/devel/setup.bash" && exec rosrun e2e_driver sim_clock_node.py) &
      for i in $(seq 100); do rostopic list 2> /dev/null | grep -q "^/clock$" && break; sleep 0.1; done
    fi
    bash src/MORAI-UDP-Bridge-Node/launch.sh &
    roslaunch velodyne_pointcloud VLP16_points.launch port:=$VLP_PORT &
    for p in $PKGS; do bash src/$p/launch.sh & done
    wait' > "$d/stack.log" 2>&1 < /dev/null &
  echo $! > "$d/stack.pid"
  echo "stack $n ($DRIVER $(cat "$DRIVE_WS/REF" 2> /dev/null)): master $((11310 + n)), udp +$off, log $d/stack.log"
  return
  fi
  # TMPDIR: the bridge writes its runtime config to $TMPDIR/morai-udp-bridge-<uid>.yaml, one name for every stack
  ROS_MASTER_URI=http://127.0.0.1:$((11310 + n)) ROS_IP=127.0.0.1 ROS_HOME=$d/ros TMPDIR=$d/tmp MORAI_BRIDGE_CONFIG=$d/bridge.yaml \
  E2E_SERVER_PORT=$((47011 + (n - 1) * ${ENGINE_PORT_STEP:-0})) E2E_WORLD=$n OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  VLP_PORT=$((2368 + off)) MASTER_PORT=$((11310 + n)) E2E_SYNC=${SYNC:-0} \
  CLOCK="$([[ ${SYNC:-0} == 1 ]] && echo "sync_clock_node.py _port:=$((47100 + n))" || echo sim_clock_node.py)" \
  setsid taskset -c "${STACK_CPUS:-0-$(($(nproc) - 1))}" bash -c '
    source /opt/ros/noetic/setup.bash; source devel/setup.bash
    roscore -p $MASTER_PORT & for i in $(seq 100); do rosnode list > /dev/null 2>&1 && break; sleep 0.1; done
    rosparam set /use_sim_time true
    rosrun e2e_driver $CLOCK &
    bash src/MORAI-UDP-Bridge-Node/launch.sh &
    roslaunch velodyne_pointcloud VLP16_points.launch port:=$VLP_PORT &
    for p in vlp16_bev tf_publisher lidar_leveler localization section_annotator e2e_driver speed_filter; do bash src/$p/launch.sh & done
    wait' > "$d/stack.log" 2>&1 < /dev/null &
  echo $! > "$d/stack.pid"
  echo "stack $n ($DRIVER): master $((11310 + n)), udp +$off, server $((47011 + (n - 1) * ${ENGINE_PORT_STEP:-0})), log $d/stack.log"
}

down1() {
  local d=$R/$1
  [[ -e $d/stack.pid ]] || return 0
  local w; w=$(cat "$d/stack.pid")
  pkill -INT -s "$w" 2>/dev/null
  for i in $(seq 30); do pgrep -s "$w" > /dev/null || break; sleep 0.5; done
  pkill -KILL -s "$w" 2>/dev/null; mv "$d/stack.pid" "$d/stack.pid.old"; echo "stack $1 down"
}

cd "$WS" || exit 1
case ${1:-} in
  up) for n in $(seq "${2:-3}"); do up1 "$n"; done ;;
  down) for d in "$R"/*/; do [[ -e $d/stack.pid ]] && down1 "$(basename "$d")"; done; true ;;
  config) config "$2"; cat "$R/$2/bridge.yaml" ;;
  *) sed -n 2,8p "$0"; exit 1 ;;
esac
