#!/usr/bin/env bash
# eval3.sh NAME DELIVER [SEED ...]: closed-loop evaluation of one WoWmayo delivery on this pod (after setup.sh), one map
# lap per seed, all at once: seed k drives world k (default seeds 7 8 9 = three worlds). eval_pod.sh runs it from the Mac.
#   DELIVER  a QAD delivery dir on the volume, or b2:<path> (synced to $V/models/<basename> while the worlds start)
# Parts, started together: worlds.sh up N | the batched engine (model_server --sims N on 47011) | stacks.sh up N with
# E2E_APPLY_DELAY_S 0.1 (the stacks run on tau; a plan observed at tau drives from tau + 0.1 s, a later reply is late).
# Then per world n: viewer.py on Xvfb :n (MJPEG 127.0.0.1:8090+n) + lap_video.py record, /e2e_driver/debug and
# /sim_clock/state recorders in stack n, eval_manager.py --laps 1 (scene variant of the seed, returns and weather over
# gRPC 7788+n). After the laps: everything down, eval3_summary.py (summary.json), 1x plain videos <= 30 MB, plan overlays
# (test/plan_overlay.py), all of $V/results/NAME (frames stay on the container disk) to B2 runs/eval_vps/NAME.
# Timeline: $V/results/NAME/timeline.txt (epoch, clock, step). Env: VIEW_FPS (10), VIEW_WIDTH (1280), APPLY_DELAY_S (0.1),
# SIM_FPS (60: uncapped; worlds.sh)
# SYNC=1: MORAI synchronous mode (README "sync"): stacks with sync_clock_node, the managers tick their worlds
# (--sync, TICK_MS 100 | 50, SERIAL=1: each reply before the tick), the engine batches the worlds' requests together
# (--gather GATHER 0.1 s, --prep PREP process). SIMS: engine slots (N). Parts already up (worlds, stacks, an engine
# from sync_bench.sh or an earlier KEEP_UP=1 run) are reused; KEEP_UP=1 leaves them up at the end (another wave).
# CPU_LAYOUT=shared (SYNC default): worlds and stacks share all CPUs but the engine's 8, every world's camera spinners on 4 CPUs.
# LAP_RESTART=1 (DRIVER=stack default): the stack's nodes start afresh before every lap after the first (stacks.sh
# renodes via eval_manager --lap-hook), as each competition run starts from a fresh stack; roscore, sim_clock and the
# recorders stay. START_CP=J / END_CP=K: section laps (eval_manager --start-cp / --end-cp; lap_s from the first move at CP J to CP K).
# DRIVER=stack (campaign default; DELIVER is ignored, give -): the competition stack of DRIVE_WS (stacks.sh, built by
# drive_setup.sh) drives instead of e2e_driver + engine, on wall time (free run, SYNC=0 only). No engine; each stack's
# /morai_udp_bridge/control_cmd is recorded (cmd_wN.csv, the first-steps signal) and the CPUs are split between the
# worlds (12 each, half of a small host) and the stacks (the rest).
set -uo pipefail
DRIVER=${DRIVER:-stack}; export DRIVER; [[ $DRIVER == stack ]] && { SYNC=0; export SENSOR_PRESET=${SENSOR_PRESET:-sub}; }   # sub: the submitted sensors (user 10/29), camp_cam7 before
[[ ${SYNC:-0} == 1 ]] && CPU_LAYOUT=${CPU_LAYOUT:-shared}            # sync_bench 10/28: shared + pinned spinners
HERE=$(cd "$(dirname "$0")" && pwd); V=${V:-/workspace}; WS=$V/MORAI-Autonomous-Driving-System; E2E=$WS/src/e2e_driver
NAME=$1; DELIVER=$2; shift 2; SEEDS=("$@"); (( ${#SEEDS[@]} )) || SEEDS=(7 8 9)
N=${#SEEDS[@]}; OUT=$V/results/$NAME; FR=/root/frames/$NAME; SCORER=$V/Alpamayo-Cosmos/tools/scorer; PY=$V/venv-rl/bin/python
for f in /root/frames/*/; do [[ $f == "$FR/" ]] || rm -rf "$f"; done   # earlier runs' frames (their videos are made)
mkdir -p "$OUT" "$FR" /root/actor
stamp() { echo "$(date +%s.%N | cut -c1-14) $(date +%T) $*" | tee -a "$OUT/timeline.txt"; }
wdir() { [[ $1 == 1 ]] && echo /root/Workspaces/scratches/morai || echo /root/inst/$1/morai; }
inrosn() { local n=$1; shift; (cd "$WS" && ROS_MASTER_URI=http://127.0.0.1:$((11310 + n)) ROS_IP=127.0.0.1 ROS_HOME=/root/stack/$n/ros \
  bash -c "source /opt/ros/noetic/setup.bash; source devel/setup.bash; $*"); }
b2env() { eval "$(tr '\0' '\n' < /proc/1/environ | grep -E '^B2_APPLICATION_KEY(_ID)?=' | sed 's/^/export /')"; }
stamp "eval3 $NAME: $DELIVER, seeds ${SEEDS[*]}, SIM_FPS ${SIM_FPS:-60}, apply delay ${APPLY_DELAY_S:-0.1} s, sync ${SYNC:-0} (tick ${TICK_MS:-100} ms, serial ${SERIAL:-0}, CPUs ${CPU_LAYOUT:-split}), $(nvidia-smi --query-gpu=name --format=csv,noheader), $(nproc --all) CPUs visible, quota $(cat /sys/fs/cgroup/cpu.max)"

# CPU sets: 12 per world (the Mono fix, worlds.sh), then 8 for the stacks and 8 for the engine; fewer visible CPUs
# shrink the worlds' share
T=$(nproc --all); PER=12; S=8; (( T < 12 * N + 16 )) && { S=4; PER=$(( (T - 8) / N )); }   # 16 CPUs, 1 world: 8 + 4 + 4
for n in $(seq "$N"); do export CPUS_$n=$(( (n - 1) * PER ))-$(( n * PER - 1 )); done
export STACK_CPUS=$((PER * N))-$((PER * N + S - 1)) ENGINE_CPUS=$((PER * N + S))-$((PER * N + 2 * S - 1))
if [[ $DRIVER == stack ]]; then
  PER=$(( T >= 12 * N + 8 ? 12 : T / (2 * N) )); (( PER > 0 )) || PER=1
  for n in $(seq "$N"); do export CPUS_$n=$(( (n - 1) * PER ))-$(( n * PER - 1 )); done
  export STACK_CPUS=$((PER * N))-$((T - 1))
elif [[ ${CPU_LAYOUT:-split} == shared ]]; then
  for n in $(seq "$N"); do export CPUS_$n=0-$((T - 9)); done
  export STACK_CPUS=0-$((T - 9)) ENGINE_CPUS=$((T - 8))-$((T - 1)) SKIP_PIN=1
fi
SYNC_ARGS=(); SERVER_ARGS=()
if [[ ${SYNC:-0} == 1 ]]; then
  export SYNC
  SERVER_ARGS=(--gather "${GATHER:-0.1}" --prep "${PREP:-process}")
  SYNC_ARGS=(--tick-ms "${TICK_MS:-100}"); [[ ${SERIAL:-0} == 1 ]] && SYNC_ARGS+=(--serial)
fi

if [[ $DRIVER != stack && $DELIVER == b2:* ]]; then
  src=${DELIVER#b2:}; DELIVER=$V/models/$(basename "$src"); mkdir -p "$DELIVER"
  ( b2env; $V/venv-tools/bin/b2 sync --threads 32 --no-progress "b2://morai2026-alpamayo/${src%/}/" "$DELIVER/" > "$OUT/b2_deliver.log" 2>&1 ) &
  DSYNC=$!
fi
bash "$HERE/worlds.sh" up "$N" > "$OUT/worlds.log" 2>&1 &
WORLDS=$!
E2E_APPLY_DELAY_S=${APPLY_DELAY_S:-0.1} bash "$HERE/stacks.sh" up "$N" > "$OUT/stacks.log" 2>&1
[[ -n ${DSYNC:-} ]] && { wait $DSYNC && stamp "delivery synced from B2 ($(du -sh "$DELIVER" | cut -f1))" || { stamp "B2 sync failed"; exit 1; }; }
if [[ $DRIVER == stack ]]; then
  : > /root/actor/server.pids; ENGINE=
elif [[ -e /root/actor/server.pids ]] && kill -0 "$(cat /root/actor/server.pids)" 2> /dev/null; then
  ENGINE=$(cat /root/actor/server.pids); ln -sf "$(readlink -f /proc/$ENGINE/fd/1)" "$OUT/server.log"
  stamp "engine already up (pid $ENGINE): $(tr '\0' ' ' < /proc/$ENGINE/cmdline | grep -o -- '--sims.*')"
else
OMP_NUM_THREADS=4 MALLOC_MMAP_MAX_=0 MALLOC_TRIM_THRESHOLD_=1073741824 HF_HUB_OFFLINE=1 HF_HOME=$V/models/hf \
TORCHINDUCTOR_CACHE_DIR=$V/cache/inductor TRITON_CACHE_DIR=$V/cache/triton \
  setsid taskset -c "$ENGINE_CPUS" $V/venv-engine/bin/python -u $E2E/server/model_server.py --backend engine --ckpt "$DELIVER" \
  --vlm Qwen/Qwen3-VL-4B-Instruct --sims "${SIMS:-$N}" "${SERVER_ARGS[@]}" --port 47011 > "$OUT/server.log" 2>&1 < /dev/null &
ENGINE=$!; echo $ENGINE > /root/actor/server.pids          # measure.sh's engine
fi
stamp "worlds, stacks and engine starting"
(nvidia-smi --query-gpu=timestamp,utilization.gpu,memory.used --format=csv,noheader,nounits -lms 1000 > "$OUT/gpu.csv" 2>&1 &
 echo $! > /root/actor/gpu.pid)

wait $WORLDS || { stamp "worlds failed"; tail -5 "$OUT/worlds.log"; bash "$HERE/stacks.sh" down; [[ -s /root/actor/server.pids ]] && kill "$(cat /root/actor/server.pids)" 2> /dev/null; exit 1; }
stamp "worlds ready"
if [[ $DRIVER == stack ]]; then   # a world kept up from an earlier run may hold another sensor preset
  for n in $(seq "$N"); do
    L=$(wdir "$n")/MoraiLauncher_Lin_Data/SaveFile/LinkConnection.txt
    python3 /root/morai-eval-msc.py "$L" pause "" 30 > /dev/null
    stamp "w$n sensors $(SENSOR_PRESET=$SENSOR_PRESET bash "$HERE/worlds.sh" preset "$n") $(python3 /root/morai-eval-msc.py "$L" load-sensors "$(SENSOR_PRESET=$SENSOR_PRESET bash "$HERE/worlds.sh" preset "$n")" 60 | cut -c1-24)"
  done
fi
if [[ $DRIVER == stack ]]; then
  [[ ${CPU_LAYOUT:-split} == shared ]] && SPIN_CPUS=$((T - 12))-$((T - 9)) bash "$HERE/pin.sh" >> "$OUT/worlds.log" 2>&1
  for n in $(seq "$N"); do   # every node of the chain registered (2 min at most: a node that died is in stack.log)
    for i in $(seq 60); do
      l=$(inrosn "$n" "rosnode list 2> /dev/null"); miss=
      for x in $([[ ${SIM_CLOCK:-1} == 1 ]] && echo sim_clock) morai_udp_bridge velodyne tf_publisher_node lidar_leveler_node localization_node section_annotator_node object_detector_node \
               traffic_light_node path_planner_node velocity_planner_node control_node speed_filter_node; do echo "$l" | grep -q "$x" || miss+=" $x"; done
      [[ -z $miss ]] && break; sleep 2
    done
    if [[ -n $miss ]]; then   # one fresh start of the nodes, then the run is env-invalid (a fresh pod is replaced)
      stamp "stack $n missing$miss: nodes restarted once ($(bash "$HERE/stacks.sh" renodes "$n" 2>&1 | tail -1))"
      l=$(inrosn "$n" "rosnode list 2> /dev/null"); miss=
      for x in morai_udp_bridge velodyne tf_publisher_node lidar_leveler_node localization_node section_annotator_node object_detector_node \
               traffic_light_node path_planner_node velocity_planner_node control_node speed_filter_node; do echo "$l" | grep -q "$x" || miss+=" $x"; done
      if [[ -n $miss ]]; then
        echo "w$n stack nodes missing after a restart:$miss at $(date +%T)" > "$OUT/ENV_INVALID"; stamp "ENV_INVALID: $(cat "$OUT/ENV_INVALID")"
        bash "$HERE/stacks.sh" down > /dev/null; exit 1
      fi
    fi
    stamp "stack $n $(cat "${DRIVE_WS:-$V/drive}/BUILT" 2> /dev/null): ${miss:+missing$miss}${miss:-all nodes up}"
    # a node that registered but failed its startup waits without processing (no /traffic_light, laps judged as if it
    # ran): one fresh start of the nodes, then env-invalid like a missing node (a fresh pod is replaced)
    sleep 15; bad=$(bash "$HERE/stacks.sh" startfail "$n" 2> /dev/null | grep -xE '[a-z_]+' | tr '\n' ' ')
    if [[ -n $bad ]]; then
      stamp "stack $n startup failed: $bad- nodes restarted once ($(bash "$HERE/stacks.sh" renodes "$n" 2>&1 | tail -1))"
      sleep 15; bad=$(bash "$HERE/stacks.sh" startfail "$n" 2> /dev/null | grep -xE '[a-z_]+' | tr '\n' ' ')
      if [[ -n $bad ]]; then
        echo "w$n startup failed after a restart: $bad($(tac /root/stack/$n/nodes.log | grep -m1 -o 'startup failed; waiting without processing: .\{0,80\}')) at $(date +%T)" > "$OUT/ENV_INVALID"
        stamp "ENV_INVALID: $(cat "$OUT/ENV_INVALID")"; bash "$HERE/stacks.sh" down > /dev/null; exit 1
      fi
    fi
  done
else
if ! grep -q "listening on" "$OUT/server.log"; then   # idle worlds burn ~3.8 cores each: the engine gets the CPU quota (10/28)
  sims=$(pgrep -f "[S]imulator.real"); kill -STOP $sims
  until grep -q "listening on" "$OUT/server.log" || ! kill -0 $ENGINE 2> /dev/null; do sleep 1; done
  kill -CONT $sims
fi
# every world's camera spinners on 4 CPUs (shared layout): unpinned they burn ~4 cores per world (10/28: N=6 0.52 -> 0.76 sim-s/s)
[[ ${CPU_LAYOUT:-split} == shared ]] && SPIN_CPUS=$((T - 12))-$((T - 9)) bash "$HERE/pin.sh" >> "$OUT/worlds.log" 2>&1
grep -q "listening on" "$OUT/server.log" || { stamp "engine failed"; tail -20 "$OUT/server.log"; bash "$HERE/stacks.sh" down; bash "$HERE/worlds.sh" down; exit 1; }
stamp "engine ready: $(grep -o 'loaded in.*' "$OUT/server.log" | cut -c1-120)"
for n in $(seq "$N"); do until inrosn "$n" "rosnode list 2> /dev/null" | grep -q e2e_driver; do sleep 2; done; done
fi
stamp "stacks ready"
# leftovers of an earlier run on this pod that was stopped before its end (a version switch kills an endless-lap eval3,
# so the cleanup after "laps done" never ran): its RViz stayed on :20+n above the new one and its viewers kept the ports,
# so the lap videos showed the old, frozen RViz (user 10/29)
for p in $(cat /root/actor/rviz.pids 2> /dev/null); do pkill -INT -s "$p" 2> /dev/null; done
kill $(cat /root/actor/rec.pids 2> /dev/null) 2> /dev/null; rm -f /root/actor/rviz.pids /root/actor/rec.pids /root/actor/managers.pids
pkill -f "[v]iewer.py 127.0.0.1 8[01]9[0-9] "; pkill -f "[l]ap_video.py record"; pkill -f "[d]iag_rec.py "; pkill -f "[r]ostopic echo"; sleep 2
pkill -KILL -f "[r]viz -d /root/stack/"; pkill -KILL -f "[v]isualizer.launch"; pkill -KILL -f "[v]iewer.py 127.0.0.1 8[01]9[0-9] " ; pkill -KILL -f "[d]iag_rec.py "
# RVIZ=1: the repo's visualizer nodes and RViz (its config at RVIZ_FPS, 5) per world on Xvfb :20+n in the stack's ROS
# master, CPUs of the stacks; outside the competition chain (/visualizer/* only feeds RViz). Captured next to the world.
if [[ $DRIVER == stack && ${RVIZ:-0} == 1 ]]; then
  for n in $(seq "$N"); do
    rd=:$((20 + n)); pgrep -f "^Xvfb $rd " > /dev/null || { setsid Xvfb $rd -screen 0 960x720x24 > /root/stack/$n/xvfb_rviz.log 2>&1 < /dev/null & sleep 1; }
    sed -E "s/^( *Frame Rate:).*/\1 ${RVIZ_FPS:-5}/" "${DRIVE_WS:-$V/drive}/src/visualizer/rviz/visualizer.rviz" > /root/stack/$n/visualizer.rviz
    for cmd in "roslaunch visualizer visualizer.launch rviz:=false" "rosrun rviz rviz -d /root/stack/$n/visualizer.rviz"; do
      setsid bash -c "$(declare -f inrosn); WS=${DRIVE_WS:-$V/drive} inrosn $n 'export DISPLAY=$rd LIBGL_ALWAYS_SOFTWARE=1; exec taskset -c $STACK_CPUS $cmd'" \
        >> /root/stack/$n/rviz.log 2>&1 < /dev/null &
      echo $! >> /root/actor/rviz.pids
    done
  done
  stamp "visualizer + RViz up (${RVIZ_FPS:-5} fps, displays :21..)"
fi

for n in $(seq "$N"); do
  O=$OUT/w$n; mkdir -p "$O"
  DISPLAY=:$n setsid python3 $E2E/tools/viewer.py 127.0.0.1 $((8090 + n)) "${VIEW_FPS:-10}" "${VIEW_WIDTH:-1280}" > "$O/viewer.log" 2>&1 < /dev/null &
  echo $! >> /root/actor/rec.pids
  setsid $PY $E2E/tools/lap_video.py record http://127.0.0.1:$((8090 + n))/ "$FR/w$n" > /dev/null 2>&1 < /dev/null &
  echo $! >> /root/actor/rec.pids
  if [[ $DRIVER == stack && ${RVIZ:-0} == 1 ]]; then   # the RViz window, recorded like the world (lap_deliver.sh puts them side by side)
    VIEW_CLASS=rviz DISPLAY=:$((20 + n)) setsid python3 $E2E/tools/viewer.py 127.0.0.1 $((8190 + n)) "${VIEW_FPS:-10}" 960 > "$O/viewer_rviz.log" 2>&1 < /dev/null &
    echo $! >> /root/actor/rec.pids
    setsid $PY $E2E/tools/lap_video.py record http://127.0.0.1:$((8190 + n))/ "$FR/r$n" > /dev/null 2>&1 < /dev/null &
    echo $! >> /root/actor/rec.pids
  fi
  if [[ $DRIVER == stack ]]; then
  WS=${DRIVE_WS:-$V/drive} inrosn "$n" "PYTHONUNBUFFERED=1 exec rostopic echo -p /morai_udp_bridge/control_cmd" > "$O/cmd_w$n.csv" 2> /dev/null < /dev/null &
  echo $! >> /root/actor/rec.pids
  inrosn "$n" "exec rostopic echo -p /sim_clock/state" > "$O/clk_w$n.csv" 2> /dev/null < /dev/null &   # SIM_CLOCK: [wall, tau, rho, n]
  echo $! >> /root/actor/rec.pids
  WS=${DRIVE_WS:-$V/drive} inrosn "$n" "exec python3 $HERE/diag_rec.py $O/diag_w$n.jsonl 5" > "$O/diag_rec.log" 2>&1 < /dev/null &   # stop diagnosis
  echo $! >> /root/actor/rec.pids
  else
  inrosn "$n" "PYTHONUNBUFFERED=1 exec rostopic echo -p /e2e_driver/debug/data" > "$O/dbg_w$n.csv" 2> /dev/null < /dev/null &
  echo $! >> /root/actor/rec.pids
  inrosn "$n" "exec rostopic echo -p /sim_clock/state" > "$O/clk_w$n.csv" 2> /dev/null < /dev/null &
  echo $! >> /root/actor/rec.pids
  fi
  hook=(); [[ $DRIVER == stack && ${LAP_RESTART:-1} == 1 ]] && hook=(--lap-hook "bash $HERE/stacks.sh renodes $n")
  (cd $E2E/tools && exec $PY -u eval_manager.py eval_map --seed "${SEEDS[n-1]}" --scorer "$SCORER" --out "$O" \
    --port $((8094 + n)) --no-view --direct --host local --grpc 127.0.0.1:$((7788 + n)) --launcher "$(wdir "$n")/MoraiLauncher_Lin_Data" \
    --objinfo-port $((7505 + 1000 * (n - 1))) --laps "${LAPS:-1}" --start-cp "${START_CP:-0}" --end-cp "${END_CP:-0}" ${SCENE_FILE:+--scene-file "$SCENE_FILE"} \
    ${hook[@]+"${hook[@]}"} ${SYNC_ARGS[@]+"${SYNC_ARGS[@]}"} $([[ ${SYNC:-0} == 1 ]] && echo --sync $((47100 + n))) > "$O/eval.log" 2>&1 < /dev/null) &
  echo $! >> /root/actor/managers.pids
done
stamp "drives started ($N managers)"
# LAP_DELIVER=1: each lap as it ends -> video, report, B2 and OUT/lap_KK.ready (lap_deliver.sh), for eval_pod.sh to send
# The loop ends once the managers are gone; it waits up to 10 min for them to appear first (19:42 camp-p62: it checked
# before eval_manager was up, ended at once and no lap was delivered).
[[ ${LAP_DELIVER:-0} == 1 ]] && (declare -A done_; seen=0; t0=$SECONDS; while :; do
  live=0; pgrep -f "[e]val_manager.py eval_map" > /dev/null && { live=1; seen=1; }
  for n in $(seq "$N"); do
    for ((k = ${done_[$n]:-0}; k < $( (wc -l < "$OUT/w$n/eval_map/laps.jsonl") 2> /dev/null || echo 0); k++)); do
      bash "$HERE/lap_deliver.sh" "$OUT" "$n" "$k" "$NAME" && stamp "lap $((k + 1)) of w$n delivered: $(cut -f1 "$OUT/lap_$(printf %02d $((k + 1))).ready" | cut -c1-160)"
      done_[$n]=$((k + 1))
    done
  done
  (( live || (! seen && SECONDS - t0 < 600) )) || break; sleep 10
done; touch "$OUT/LAPS_DELIVERED") &
(for i in $(seq 900); do   # every world's first model step (sync: steps.jsonl; real time: a /e2e_driver/debug row)
  ok=1; for n in $(seq "$N"); do
    if [[ $DRIVER == stack ]]; then (( $(wc -l < "$OUT/w$n/cmd_w$n.csv") > 1 )) || ok=0
    elif [[ ${SYNC:-0} == 1 ]]; then [[ -s $OUT/w$n/eval_map/steps.jsonl ]] || ok=0; else (( $(wc -l < "$OUT/w$n/dbg_w$n.csv") > 1 )) || ok=0; fi
  done; (( ok )) && break; sleep 1; done
  stamp "first steps") &
(sleep 240; bash "$HERE/measure.sh" 120 > "$OUT/cpu.txt" 2>&1) &
# env guard (DRIVER=stack, HARNESS v2.2): 60 s after the first steps and then every 5 min, over 30 s of wall time, the
# environment only (not the stack's own faults: a planner that rejects its input also drops /control_cmd):
#   /localization/ego >= GUARD_EGO_HZ (40; 50 nominal) per simulator second (rostopic hz on /clock), the manager's rho
#   (its GT clock estimate, for this check only) >= GUARD_RHO_MIN (0.5), and the "pipeline Rejected" warnings of the
#   upstream nodes (localization, lidar_leveler) <= GUARD_REJECT_PER_MIN (130/min)
# or OUT/ENV_INVALID (the reason) is written, the managers stop and every later lap is marked env-invalid.
# /morai_udp_bridge/control_cmd per simulator second is logged too, flagged outside 30 +- 3 Hz (the competition PC's rate).
[[ $DRIVER == stack && ${GUARD:-1} == 1 ]] && (
  until grep -q "first steps" "$OUT/timeline.txt"; do sleep 2; done; sleep 60
  while pgrep -f "[e]val_manager.py eval_map" > /dev/null; do
    for n in $(seq "$N"); do
      log=/root/stack/$n/ros/log/latest/rosout.log
      st=/root/stack/$n/restart.stamp   # LAP_RESTART: no window within 90 s of a node restart
      [[ -e $st ]] && (( $(date +%s) - $(stat -c %Y "$st") < 90 )) && { echo "$(date +%T) w$n skipped (nodes restarted)" >> "$OUT/env_guard.log"; continue; }
      c0=$(wc -l < "$OUT/w$n/cmd_w$n.csv"); r0=$(grep -cE "WARN /(localization|lidar_leveler)_node .*Rejected" "$log" 2> /dev/null); t0=$(date +%s.%N)
      ego=$(WS=${DRIVE_WS:-$V/drive} inrosn "$n" "timeout 30 rostopic hz -w 200 /localization/ego 2>&1" | grep average | tail -1 | awk '{print $3}')
      c1=$(wc -l < "$OUT/w$n/cmd_w$n.csv"); r1=$(grep -cE "WARN /(localization|lidar_leveler)_node .*Rejected" "$log" 2> /dev/null); t1=$(date +%s.%N)
      srho=$(tail -1 "$OUT/w$n/clk_w$n.csv" 2> /dev/null | awk -F, '{print $(NF-1)}')
      mrho=$(curl -s -m 3 http://127.0.0.1:$((8094 + n))/status | grep -o 'rho [0-9.]*' | awk '{print $2}')
      # the first window is the host check: the world's rho must reach GUARD_RHO_FIRST (0.85) or the pod is replaced
      # (eval_pod.sh on ENV_INVALID; a 3090 / i7-11700F that passed the CPU and Vulkan gates ran at 0.5-0.66, 10/29)
      rmin=${GUARD_RHO_MIN:-0.5}; [[ ! -e $OUT/env_guard.log ]] && rmin=${GUARD_RHO_FIRST:-0.85}
      read -r cmd rpm flag bad < <(python3 -c "import sys; c,r,dt,srho,mrho,ego=map(float,sys.argv[1:]); cmd=c/dt/max(srho,0.05); rpm=r/dt*60
print(round(cmd,1), round(rpm), '-' if 27 <= cmd <= 33 else 'CMD_OUT_OF_30+-3', int(ego < ${GUARD_EGO_HZ:-40} or mrho < $rmin or rpm > ${GUARD_REJECT_PER_MIN:-130}))" \
        $((c1 - c0)) $((r1 - r0)) "$(echo "$t1 - $t0" | bc)" "${srho:-1}" "${mrho:-1}" "${ego:-0}")
      line="w$n ego ${ego:-?} Hz(sim) rho ${mrho:-?} (clock ${srho:-?}) upstream rejected ${rpm}/min cmd ${cmd} Hz(sim) $flag"
      # a node restart (LAP_RESTART) inside the window: its numbers are the restart's, not the environment's
      [[ -e $st ]] && (( $(stat -c %Y "$st") >= ${t0%.*} )) && { echo "$(date +%T) $line (discarded: nodes restarted in the window)" >> "$OUT/env_guard.log"; continue; }
      echo "$(date +%T) $line" >> "$OUT/env_guard.log"
      # after the host check, one bad window is a warning and the second in a row ends the run (a single ego dip to
      # 37.5 Hz at rho 1.00 ended an endless run, 10/29)
      if [[ $bad == 1 && $(grep -cv "skipped\|discarded" "$OUT/env_guard.log") -gt 1 && ! -e $OUT/w$n/guard_warn ]]; then
        touch "$OUT/w$n/guard_warn"; echo "$(date +%T) w$n warning: one bad window (two in a row end the run)" >> "$OUT/env_guard.log"; continue
      fi
      [[ $bad == 1 ]] || rm -f "$OUT/w$n/guard_warn"
      if [[ $bad == 1 ]]; then
        echo "$line at $(date +%T)" > "$OUT/ENV_INVALID"; stamp "ENV_INVALID: $(cat "$OUT/ENV_INVALID")"
        kill $(cat /root/actor/managers.pids) 2> /dev/null; break 2
      fi
    done
    sleep 270
  done) &
for p in $(cat /root/actor/managers.pids); do wait "$p"; done
stamp "laps done"

kill $(cat /root/actor/rec.pids /root/actor/gpu.pid) 2> /dev/null; pkill -f "[r]ostopic echo" ; sleep 2
for p in $(cat /root/actor/rviz.pids 2> /dev/null); do pkill -INT -s "$p" 2> /dev/null; done; rm -f /root/actor/rviz.pids
[[ ${KEEP_UP:-0} == 1 ]] || { bash "$HERE/stacks.sh" down > /dev/null; kill "$(cat /root/actor/server.pids)" 2> /dev/null; bash "$HERE/worlds.sh" down; }
for n in $(seq "$N"); do cp /root/stack/$n/stack.log "$OUT/w$n/stack.log" 2> /dev/null; cp /root/inst/$n/Player.log "$OUT/w$n/Player.log" 2> /dev/null; done
mv /root/actor/managers.pids /root/actor/managers.pids.old; mv /root/actor/rec.pids /root/actor/rec.pids.old
stamp "$([[ ${KEEP_UP:-0} == 1 ]] && echo "laps recorded, worlds, stacks and engine kept up" || echo "all down")"

$PY "$HERE/eval3_summary.py" "$OUT" | tee "$OUT/summary.txt"
if [[ ${LAP_DELIVER:-0} == 1 ]]; then    # videos and reports were made lap by lap
  until [[ -e $OUT/LAPS_DELIVERED ]]; do sleep 5; done
  cat "$OUT"/lap_*/report.txt > "$OUT/laps.txt" 2> /dev/null
else
for n in $(seq "$N"); do
  O=$OUT/w$n
  $PY $E2E/tools/lap_video.py make "$FR/w$n" "$O/eval_map/lap000.clock.json" "$FR/w$n.mp4" 30 > "$O/video.json" 2>&1 &&
    bash "$HERE/phone_video.sh" "$FR/w$n.mp4" "$O/${NAME}_w$n.mp4" >> "$O/video.json" 2>&1 &
  [[ $DRIVER == stack ]] || $PY $E2E/test/plan_overlay.py "$O" "w$n" "$SCORER" 0 > "$O/plans.log" 2>&1 &
done
wait
fi
stamp "videos and overlays: $(ls "$OUT"/w*/*.mp4 "$OUT"/lap_*/*.mp4 2> /dev/null | wc -l) videos, $(ls "$OUT"/w*/plans/*.png 2> /dev/null | wc -l) figures"
(b2env; $V/venv-tools/bin/b2 sync --threads 16 --no-progress "$OUT/" "b2://morai2026-alpamayo/runs/eval_vps/$NAME/" > "$OUT/../b2_$NAME.log" 2>&1) &&
  stamp "B2 runs/eval_vps/$NAME" || stamp "B2 upload failed (see results/b2_$NAME.log)"
touch "$OUT/DONE"
