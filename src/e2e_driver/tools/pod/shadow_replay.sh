#!/usr/bin/env bash
# shadow_replay.sh IN.json... (inside the morai-stack container, after shadow.sh build): shadow 1 on each replay input in
# turn (shadow_replay.py play; a fake stack master 11311 for the relay), restarted between inputs; records
# $V/shadow/rec_w1, marks (name wall_start hold) $V/shadow/marks.txt for `shadow_replay.py plans`. README "쉐도 로컬 재생".
set -uo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
export V=${V:-/workspace} WS=${WS:-$V/shadow/ws} SHADOW_CPUS=${SHADOW_CPUS:-0-5} ROS_MASTER_URI=http://127.0.0.1:11311
source /opt/ros/noetic/setup.bash; source "$WS/devel/setup.bash"
roscore -p 11311 > /tmp/replay_roscore.log 2>&1 &
sleep 4
M=$V/shadow/marks.txt; : > "$M"; first=1
for f in "$@"; do
  hold=$([[ $first == 1 ]] && echo 35 || echo 30)          # the shadow's first start takes longer than a restart
  python3 "$HERE/shadow_replay.py" play "$f" "$hold" > "$V/shadow/play_$(basename "$f" .json).log" 2>&1 &
  rp=$!; echo "$(basename "$f" .json) $(date +%s.%N) $hold" >> "$M"
  sleep 1
  if [[ $first == 1 ]]; then bash "$HERE/shadow.sh" up 1; else bash "$HERE/shadow.sh" restart 1; fi >> "$V/shadow/replay_up.log" 2>&1
  wait $rp; first=0
done
bash "$HERE/shadow.sh" down >> "$V/shadow/replay_up.log" 2>&1
kill %1 2> /dev/null
echo "done: $M"
