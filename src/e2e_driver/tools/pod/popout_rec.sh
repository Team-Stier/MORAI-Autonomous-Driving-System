#!/usr/bin/env bash
# popout_rec.sh up | start RUN | stop | down: rosbags of world 1 while the collection driver drives it (shadow.sh up 1 drive),
# in the R2 layout for tools/step0: the morai-parallel-collect TOPICS, cameras at 10 Hz (camera-10hz.py), /velodyne_packets
# from the eval workspace's VLP16 launch (the stacks' port 2368). up: those two feeders in the drive master (11321);
# start RUN: a new bag $V/bags/RUN/RUN-w1-KKK.bag for the episode in $V/spool/RUN/inflight.json (rl_episode --loc-restart),
# the previous one closed first; stop: close it (rl_episode --end-hook). README "근거리 돌출 녹화".
set -uo pipefail
cpus=${SHADOW_CPUS:-0-$(($(nproc) - 1))}
V=${V:-/workspace}; WS=$V/MORAI-Autonomous-Driving-System; R=/root/shadow/1/rec; mkdir -p "$R"
bg() {  # LOG cmd...: in the drive master as its own session (setsid -> taskset -> bash -> cmd: one pid, as shadow.sh)
  local log=$1; shift
  ROS_MASTER_URI=http://127.0.0.1:11321 ROS_IP=127.0.0.1 ROS_HOME=$R setsid taskset -c "$cpus" bash -c \
    "cd $WS && source /opt/ros/noetic/setup.bash && source devel/setup.bash && exec $*" > "$log" 2>&1 < /dev/null &
}
CAMS="/image/front/compressed /image/left/compressed /image/right/compressed /image/camera_7/compressed"
TOPICS="$(printf '%s_10hz ' $CAMS) /velodyne_packets /sensors/imu/data /sensors/gps/fix /vehicle/competition_status
  /morai_udp_bridge/control_cmd /tf /tf_static /section_annotation /collect_only/ego_status"

stop() {
  [[ -e $R/bag.pid ]] || return 0
  pkill -INT -s "$(cat "$R/bag.pid")" 2> /dev/null
  for i in $(seq 60); do pgrep -s "$(cat "$R/bag.pid")" > /dev/null || break; sleep 0.5; done   # .active -> .bag
  pkill -KILL -s "$(cat "$R/bag.pid")" 2> /dev/null; rm -f "$R/bag.pid"
}

case ${1:-} in
  up)
    bg "$R/velodyne.log" roslaunch velodyne_pointcloud VLP16_points.launch port:=2368; echo $! > "$R/feed.pids"
    bg "$R/cam10.log" python3 -u $V/shadow/gt/camera-10hz.py camera-10hz $CAMS; echo $! >> "$R/feed.pids" ;;
  start)
    stop
    k=$(python3 -c "import json,sys; print(int(json.load(open(sys.argv[1]))['episode'][2:]))" "$V/spool/$2/inflight.json") || exit 1
    mkdir -p "$V/bags/$2"
    bg "$R/bag.log" rosbag record -q -O $V/bags/$2/$2-w1-$(printf %03d "$k").bag $(echo $TOPICS); echo $! > "$R/bag.pid" ;;
  stop) stop ;;
  down)
    stop
    for p in $(cat "$R/feed.pids" 2> /dev/null); do pkill -INT -s "$p" 2> /dev/null; done; sleep 2
    for p in $(cat "$R/feed.pids" 2> /dev/null); do pkill -KILL -s "$p" 2> /dev/null; done; rm -f "$R/feed.pids" ;;
  *) sed -n 2,7p "$0"; exit 1 ;;
esac
