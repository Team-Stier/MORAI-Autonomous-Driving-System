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
set -uo pipefail
[[ ${SYNC:-0} == 1 ]] && CPU_LAYOUT=${CPU_LAYOUT:-shared}            # sync_bench 10/28: shared + pinned spinners
HERE=$(cd "$(dirname "$0")" && pwd); V=${V:-/workspace}; WS=$V/MORAI-Autonomous-Driving-System; E2E=$WS/src/e2e_driver
NAME=$1; DELIVER=$2; shift 2; SEEDS=("$@"); (( ${#SEEDS[@]} )) || SEEDS=(7 8 9)
N=${#SEEDS[@]}; OUT=$V/results/$NAME; FR=/root/frames/$NAME; SCORER=$V/Alpamayo-Cosmos/tools/scorer; PY=$V/venv-rl/bin/python
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
if [[ ${CPU_LAYOUT:-split} == shared ]]; then
  for n in $(seq "$N"); do export CPUS_$n=0-$((T - 9)); done
  export STACK_CPUS=0-$((T - 9)) ENGINE_CPUS=$((T - 8))-$((T - 1)) SKIP_PIN=1
fi
SYNC_ARGS=(); SERVER_ARGS=()
if [[ ${SYNC:-0} == 1 ]]; then
  export SYNC
  SERVER_ARGS=(--gather "${GATHER:-0.1}" --prep "${PREP:-process}")
  SYNC_ARGS=(--tick-ms "${TICK_MS:-100}"); [[ ${SERIAL:-0} == 1 ]] && SYNC_ARGS+=(--serial)
fi

if [[ $DELIVER == b2:* ]]; then
  src=${DELIVER#b2:}; DELIVER=$V/models/$(basename "$src"); mkdir -p "$DELIVER"
  ( b2env; $V/venv-tools/bin/b2 sync --threads 32 --no-progress "b2://morai2026-alpamayo/${src%/}/" "$DELIVER/" > "$OUT/b2_deliver.log" 2>&1 ) &
  SYNC=$!
fi
bash "$HERE/worlds.sh" up "$N" > "$OUT/worlds.log" 2>&1 &
WORLDS=$!
E2E_APPLY_DELAY_S=${APPLY_DELAY_S:-0.1} bash "$HERE/stacks.sh" up "$N" > "$OUT/stacks.log" 2>&1
[[ -n ${SYNC:-} ]] && { wait $SYNC && stamp "delivery synced from B2 ($(du -sh "$DELIVER" | cut -f1))" || { stamp "B2 sync failed"; exit 1; }; }
if [[ -e /root/actor/server.pids ]] && kill -0 "$(cat /root/actor/server.pids)" 2> /dev/null; then
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

wait $WORLDS || { stamp "worlds failed"; tail -5 "$OUT/worlds.log"; bash "$HERE/stacks.sh" down; kill "$(cat /root/actor/server.pids)"; exit 1; }
stamp "worlds ready"
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
stamp "stacks ready"

for n in $(seq "$N"); do
  O=$OUT/w$n; mkdir -p "$O"
  DISPLAY=:$n setsid python3 $E2E/tools/viewer.py 127.0.0.1 $((8090 + n)) "${VIEW_FPS:-10}" "${VIEW_WIDTH:-1280}" > "$O/viewer.log" 2>&1 < /dev/null &
  echo $! >> /root/actor/rec.pids
  setsid $PY $E2E/tools/lap_video.py record http://127.0.0.1:$((8090 + n))/ "$FR/w$n" > /dev/null 2>&1 < /dev/null &
  echo $! >> /root/actor/rec.pids
  inrosn "$n" "PYTHONUNBUFFERED=1 exec rostopic echo -p /e2e_driver/debug/data" > "$O/dbg_w$n.csv" 2> /dev/null < /dev/null &
  echo $! >> /root/actor/rec.pids
  inrosn "$n" "exec rostopic echo -p /sim_clock/state" > "$O/clk_w$n.csv" 2> /dev/null < /dev/null &
  echo $! >> /root/actor/rec.pids
  (cd $E2E/tools && exec $PY -u eval_manager.py eval_map --seed "${SEEDS[n-1]}" --scorer "$SCORER" --out "$O" \
    --port $((8094 + n)) --no-view --direct --host local --grpc 127.0.0.1:$((7788 + n)) --launcher "$(wdir "$n")/MoraiLauncher_Lin_Data" \
    --objinfo-port $((7505 + 1000 * (n - 1))) --laps "${LAPS:-1}" ${SYNC_ARGS[@]+"${SYNC_ARGS[@]}"} $([[ ${SYNC:-0} == 1 ]] && echo --sync $((47100 + n))) > "$O/eval.log" 2>&1 < /dev/null) &
  echo $! >> /root/actor/managers.pids
done
stamp "drives started ($N managers)"
# LAP_DELIVER=1: each lap as it ends -> video, report, B2 and OUT/lap_KK.ready (lap_deliver.sh), for eval_pod.sh to send
[[ ${LAP_DELIVER:-0} == 1 ]] && (declare -A done_; while :; do
  live=0; pgrep -f "[e]val_manager.py eval_map" > /dev/null && live=1
  for n in $(seq "$N"); do
    for ((k = ${done_[$n]:-0}; k < $(wc -l < "$OUT/w$n/eval_map/laps.jsonl" 2> /dev/null || echo 0); k++)); do
      bash "$HERE/lap_deliver.sh" "$OUT" "$n" "$k" "$NAME" && stamp "lap $((k + 1)) of w$n delivered: $(cut -f1 "$OUT/lap_$(printf %02d $((k + 1))).ready" | cut -c1-160)"
      done_[$n]=$((k + 1))
    done
  done
  (( live )) || break; sleep 10
done; touch "$OUT/LAPS_DELIVERED") &
[[ ${SYNC:-0} == 1 ]] && (for i in $(seq 600); do   # the first simulator step of every world (sync: steps.jsonl)
  ok=1; for n in $(seq "$N"); do [[ -s $OUT/w$n/eval_map/steps.jsonl ]] || ok=0; done; (( ok )) && break; sleep 1; done
  stamp "first steps ($(cat "$OUT"/w*/eval_map/steps.jsonl 2> /dev/null | wc -l) so far)") &
(sleep 240; bash "$HERE/measure.sh" 120 > "$OUT/cpu.txt" 2>&1) &
for p in $(cat /root/actor/managers.pids); do wait "$p"; done
stamp "laps done"

kill $(cat /root/actor/rec.pids /root/actor/gpu.pid) 2> /dev/null; pkill -f "[r]ostopic echo" ; sleep 2
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
  $PY $E2E/test/plan_overlay.py "$O" "w$n" "$SCORER" 0 > "$O/plans.log" 2>&1 &
done
wait
fi
stamp "videos and overlays: $(ls "$OUT"/w*/*.mp4 "$OUT"/lap_*/*.mp4 2> /dev/null | wc -l) videos, $(ls "$OUT"/w*/plans/*.png 2> /dev/null | wc -l) figures"
(b2env; $V/venv-tools/bin/b2 sync --threads 16 --no-progress "$OUT/" "b2://morai2026-alpamayo/runs/eval_vps/$NAME/" > "$OUT/../b2_$NAME.log" 2>&1) &&
  stamp "B2 runs/eval_vps/$NAME" || stamp "B2 upload failed (see results/b2_$NAME.log)"
touch "$OUT/DONE"
