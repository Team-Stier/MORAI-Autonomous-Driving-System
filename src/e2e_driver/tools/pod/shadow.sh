#!/usr/bin/env bash
# shadow.sh build | up n [drive] | restart n | down | check n: the R2 collection driver beside world n as a DAgger expert.
# Driver: the collection fork (MORAI-Autonomous-Driving-System-collect e2cd63a, the R2 image morai-stack:e2cd63a) with
# its collection tune (stop_timeout_sec 600) and the morai-parallel-collect GT/R2 nodes (Ego status, GT localization,
# Object Info filter, GT AEB, GT yield, TL GT driver), and of the fork only what the planners need: tf_publisher,
# section_annotator, path_planner, velocity_planner, control.
# shadow (default): its own ROS master 11320+n on wall time (as the collection stacks ran), NO bridge in that master and
#   every command remapped: control /control_cmd -> /shadow/control_cmd, AEB and yield /morai_udp_bridge/control_cmd ->
#   /shadow/aeb_cmd, /shadow/yield_cmd. It never reaches the simulator. Inputs: Ego status 9095 and TL GT 7509 (+ world
#   offset) straight from world n (nothing else binds them), Object Info from udp_tee.py (7505 -> 17505 here and 27505
#   for rl_episode: two sockets on one port split the packets), /vehicle/competition_status and /sensors/imu/data from
#   stack n's master (shadow_relay.py, restamped to wall time). shadow_rec.py records what dagger_label.py needs.
# drive: the collection driver drives world 1 itself (label accuracy check; stack 1 must be down): the fork's own bridge
#   (make-bridge.py config) and speed_filter, no remaps: the R2 Mac stack's driver.
# restart: the driver nodes anew (an episode's spawn is a jump the planners must not see; rl_episode --loc-restart),
#   back once /local_path is out again (25 s at most). Logs: /root/shadow/n/. Env: SHADOW_CPUS (all).
# TAKEOVER=1 (shadow mode): the start intervention, takeover.py watch (here) + pub (stack n's master, TAKEOVER_S 25):
#   the shadow's control command drives a car stopped that long where the shadow says go, until 3 m/s or 5 s.
set -uo pipefail
HERE=$(cd "$(dirname "$0")" && pwd); V=${V:-/workspace}
SW=$V/shadow/ws; ST=$V/shadow/gt; R=/root/shadow; WS_EVAL=${WS:-$V/MORAI-Autonomous-Driving-System}
cpus=${SHADOW_CPUS:-0-$(($(nproc) - 1))}

inm() {  # MASTER_PORT cmd...: in that ROS master (wall time), shadow workspace
  local p=$1; shift
  (cd "$SW" && ROS_MASTER_URI=http://127.0.0.1:$p ROS_IP=127.0.0.1 bash -c "source /opt/ros/noetic/setup.bash; source devel/setup.bash; $*")
}

build() {  # the fork snapshot ($V/shadow/shadow_src.tar from the Mac) -> $SW, built (the packages listed below only)
  mkdir -p "$SW" /workspaces && tar -xf $V/shadow/shadow_src.tar -C "$SW" || return 1
  sed -i 's/^stop_timeout_sec: .*/stop_timeout_sec: 600.0/' "$SW/src/velocity_planner/config/velocity_planner/parameters/stop_line_state.yaml"
  local f; for f in "$HERE"/shadow_patches/*.patch; do patch -s -N -p1 -d "$SW" < "$f" || { echo "patch $f failed"; return 1; }; done   # early lane changes
  ln -sfn "$SW" /workspaces/MORAI-Autonomous-Driving-System      # the GT nodes' fixed paths (relay module, global path)
  local p pk=""; for p in path_planner velocity_planner control node_pipeline section_annotator tf_publisher speed_filter morai_test interfaces MORAI-ROS_morai_msgs MORAI-UDP-Bridge-Node; do pk+=" $SW/src/$p"; done
  (set +u; source /opt/ros/noetic/setup.bash; apt-get update -qq > /dev/null 2>&1; rosdep update --rosdistro noetic > /dev/null 2>&1   # the bundle lacks lanelet2 (10/28)
   rosdep install -y -q --rosdistro noetic --ignore-src --from-paths $pk) > $V/shadow/rosdep.log 2>&1 || { echo "rosdep failed: $V/shadow/rosdep.log"; return 1; }
  (set +u; cd "$SW" && source /opt/ros/noetic/setup.bash && taskset -c "$cpus" catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo -j"$(nproc)" \
    -DCATKIN_WHITELIST_PACKAGES="morai_msgs;interfaces;node_pipeline;morai_test;morai_udp_bridge;tf_publisher;section_annotator;path_planner;velocity_planner;control;speed_filter") \
    > $V/shadow/build.log 2>&1 && touch $V/shadow/BUILT
}

up() {  # n [drive]
  local n=$1 mode=${2:-shadow} d=$R/$1 off=$(( ($1 - 1) * 1000 )) m=$((11320 + $1))
  mkdir -p "$d/ros" "$d/tmp"; echo "$mode" > "$d/mode"
  for i in $(seq 900); do [[ -e $V/shadow/BUILT ]] && break; sleep 2; done
  [[ -e $V/shadow/BUILT ]] || { echo "shadow $n: no build ($V/shadow/build.log)"; return 1; }
  ROS_HOME=$d/ros setsid taskset -c "$cpus" bash -c "source /opt/ros/noetic/setup.bash; exec roscore -p $m" > "$d/roscore.log" 2>&1 < /dev/null &
  echo $! > "$d/roscore.pid"
  for i in $(seq 100); do inm $m "rosnode list" > /dev/null 2>&1 && break; sleep 0.2; done
  setsid taskset -c "$cpus" python3 -u "$HERE/udp_tee.py" $((7505 + off)) $((17505 + off)) $((27505 + off)) > "$d/tee.log" 2>&1 < /dev/null &
  echo $! > "$d/tee.pid"
  if [[ $mode == shadow ]]; then   # stack n's competition status and IMU, from its master into this one
    ROS_MASTER_URI=http://127.0.0.1:$((11310 + n)) ROS_IP=127.0.0.1 setsid taskset -c "$cpus" bash -c \
      "cd $WS_EVAL && source /opt/ros/noetic/setup.bash && source devel/setup.bash && exec python3 -u $HERE/shadow_relay.py out $((47200 + n))" > "$d/relay_out.log" 2>&1 < /dev/null &
    echo $! > "$d/relay_out.pid"                      # setsid -> taskset -> bash -> python: one pid, its own session
  fi
  ROS_MASTER_URI=http://127.0.0.1:$m ROS_IP=127.0.0.1 ROS_HOME=$d/ros setsid taskset -c "$cpus" bash -c \
    "cd $SW && source /opt/ros/noetic/setup.bash && source devel/setup.bash && exec python3 -u $HERE/shadow_rec.py $mode $V/shadow/rec_w$n" > "$d/rec.log" 2>&1 < /dev/null &
  echo $! > "$d/rec.pid"
  if [[ $mode == shadow && ${TAKEOVER:-0} == 1 ]]; then   # the start intervention (takeover.py): the only shadow -> car path
    ROS_MASTER_URI=http://127.0.0.1:$m ROS_IP=127.0.0.1 ROS_HOME=$d/ros setsid taskset -c "$cpus" bash -c \
      "cd $SW && source /opt/ros/noetic/setup.bash && source devel/setup.bash && exec python3 -u $HERE/takeover.py watch $((47210 + n)) $V/shadow/rec_w$n/takeover.jsonl ${TAKEOVER_S:-25}" > "$d/takeover_watch.log" 2>&1 < /dev/null &
    echo $! > "$d/takeover_watch.pid"
    ROS_MASTER_URI=http://127.0.0.1:$((11310 + n)) ROS_IP=127.0.0.1 setsid taskset -c "$cpus" bash -c \
      "cd $WS_EVAL && source /opt/ros/noetic/setup.bash && source devel/setup.bash && exec python3 -u $HERE/takeover.py pub $((47210 + n))" > "$d/takeover_pub.log" 2>&1 < /dev/null &
    echo $! > "$d/takeover_pub.pid"
  fi
  nodes "$n"
}

nodes() {  # n: the driver nodes as one session (restart kills and starts this session only)
  local n=$1 d=$R/$1 off=$(( ($1 - 1) * 1000 )) m=$((11320 + $1)) mode aeb='' yld='' ctl=''
  mode=$(cat "$d/mode")
  [[ $mode == shadow ]] && { aeb=/morai_udp_bridge/control_cmd:=/shadow/aeb_cmd; yld=/morai_udp_bridge/control_cmd:=/shadow/yield_cmd; ctl=/control_cmd:=/shadow/control_cmd; }
  [[ $mode == drive ]] && python3 "$ST/make-bridge.py" "$SW" "$d/bridge.yaml" 127.0.0.1 127.0.0.1 9093 > /dev/null
  cd "$SW" || return 1
  ROS_MASTER_URI=http://127.0.0.1:$m ROS_IP=127.0.0.1 ROS_HOME=$d/ros TMPDIR=$d/tmp MORAI_BRIDGE_CONFIG=$d/bridge.yaml \
  setsid taskset -c "$cpus" bash -c "
    source /opt/ros/noetic/setup.bash; source devel/setup.bash
    st() { local name=\$1; shift; \"\$@\" > $d/\$name.log 2>&1 < /dev/null & }
    T=$ST
    [[ $mode == shadow ]] && st relay_in python3 -u $HERE/shadow_relay.py in $((47200 + n))
    [[ $mode == drive ]] && { st bridge bash src/MORAI-UDP-Bridge-Node/launch.sh; st speed_filter rosrun speed_filter speed_filter_node; }
    st ego_status python3 -u \$T/collect-only-ego-status-node.py _enable_collect_only:=true _port:=$((9095 + off))
    st gt_localization python3 -u \$T/gt-localization-node.py
    st tl_gt python3 -u \$T/collect-only-tl-gt-node.py _enable_collect_only:=true _port:=$((7509 + off))
    st tl_gt_driver python3 -u \$T/tl-gt-driver-node.py
    st objects python3 -u \$T/gt-objects-filter-node.py _object_port:=$((17505 + off))
    st gt_aeb python3 -u \$T/gt-aeb-node.py /collect_only/objects_all $aeb
    st gt_yield python3 -u \$T/gt-yield-node.py /collect_only/objects_all $yld
    st tf rosrun tf_publisher tf_publisher_node
    st static_tf rosrun tf_publisher static_tf_publisher_node
    st section rosrun section_annotator section_annotator_node
    st path_planner rosrun path_planner path_planner_node
    st velocity_planner rosrun velocity_planner velocity_planner_node
    st control rosrun control control_node $ctl
    wait" > "$d/nodes.log" 2>&1 < /dev/null &
  echo $! > "$d/nodes.pid"
}

stop_nodes() {  # n
  local d=$R/$1 w
  [[ -e $d/nodes.pid ]] || return 0
  w=$(cat "$d/nodes.pid")
  pkill -INT -s "$w" 2> /dev/null
  for i in $(seq 20); do pgrep -s "$w" > /dev/null || break; sleep 0.5; done
  pkill -KILL -s "$w" 2> /dev/null; rm -f "$d/nodes.pid"
}

restart() {  # n: driver nodes anew, back once the planner publishes /local_path again (or 25 s)
  local n=$1 t0=$SECONDS
  stop_nodes "$n"; nodes "$n"
  inm $((11320 + n)) "timeout 25 rostopic echo -n 1 /local_path/header" > /dev/null 2>&1
  echo "$(date +%T) shadow $n restarted, /local_path after $((SECONDS - t0)) s" >> "$R/$n/restarts.log"
}

down() {
  local d n
  for d in "$R"/*/; do
    n=$(basename "$d"); [[ -e $d/roscore.pid ]] || continue
    stop_nodes "$n"
    for f in rec relay_out tee takeover_watch takeover_pub roscore; do [[ -e $d/$f.pid ]] && { pkill -INT -s "$(cat "$d/$f.pid")" 2> /dev/null; kill "$(cat "$d/$f.pid")" 2> /dev/null; rm -f "$d/$f.pid"; }; done
    pkill -f "rosmaster.*-p $((11320 + n))" 2> /dev/null
    echo "shadow $n down"
  done
}

pubs() {  # TOPIC < rostopic info output: one line per publisher ("none" for none or an unknown topic)
  awk -v t="$1" '/^Publishers:/{p=1; next} /^Subscribers:/{p=0} p && /\*/{print t, "publisher", $2; c++} END{if(!c) print t, "publisher none"}'
}

check() {  # n: who publishes the commands, in both masters (the shadow must have none on the real command topics)
  local n=$1 t
  echo "== stack $n master $((11310 + n)) (drives the car)"
  for t in /control_cmd /morai_udp_bridge/control_cmd; do
    (cd "$WS_EVAL" && ROS_MASTER_URI=http://127.0.0.1:$((11310 + n)) bash -c "source /opt/ros/noetic/setup.bash; source devel/setup.bash; rostopic info $t 2>&1") | pubs "$t"
  done
  echo "== shadow $n master $((11320 + n))"
  for t in /control_cmd /morai_udp_bridge/control_cmd /shadow/control_cmd /shadow/aeb_cmd /shadow/yield_cmd; do
    inm $((11320 + n)) "rostopic info $t 2>&1" | pubs "$t"
  done
  inm $((11320 + n)) "rosnode list" | sed 's/^/node /'
}

case ${1:-} in
  build) build ;;
  up) up "$2" "${3:-shadow}" ;;
  restart) restart "$2" ;;
  down) down ;;
  check) check "$2" ;;
  *) sed -n 2,19p "$0"; exit 1 ;;
esac
