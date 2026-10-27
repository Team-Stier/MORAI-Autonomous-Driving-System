#!/usr/bin/env bash
# sync_bench.sh NAME DELIVER COMBO...: speed of the synchronous evaluation (README "sync") on this pod, per combination
# COMBO = N:TICK_MS:MODE (worlds, simulator step 100 | 50 ms, o = the reply awaited during the next tick, s = before it),
# e.g. 8:100:o 8:50:o 6:100:o 4:100:o 4:100:s ...: worlds and stacks up to N are started if missing (all at once), those
# above N stopped (descending N needs no restarts); one engine (--sims of the largest N, --gather GATHER, --prep PREP)
# serves them all. DELIVER: a delivery dir on the volume.
# Per combination OUT/nN_tT_M/: N eval_manager laps (seeds 101..), MEAS_S (150) wall s after every world's first step,
# then SIGINT (the lap so far is kept); wN/steps.jsonl (per step: tick, stack and wait times), dbg_wN.csv (model_ms),
# gpu.csv, cpu.txt (measure.sh). Then sync_bench_summary.py OUT -> OUT/bench.json + table. Worlds, stacks and the
# engine stay up: eval3.sh (SYNC=1) reuses them. CPU_LAYOUT shared (default here) | split as eval3.sh, for the largest N.
set -uo pipefail
HERE=$(cd "$(dirname "$0")" && pwd); V=${V:-/workspace}; WS=$V/MORAI-Autonomous-Driving-System; E2E=$WS/src/e2e_driver
NAME=$1; DELIVER=$2; shift 2; COMBOS=("$@")
OUT=$V/results/$NAME; SCORER=$V/Alpamayo-Cosmos/tools/scorer; PY=$V/venv-rl/bin/python
mkdir -p "$OUT" /root/actor
stamp() { echo "$(date +%s.%N | cut -c1-14) $(date +%T) $*" | tee -a "$OUT/timeline.txt"; }
wdir() { [[ $1 == 1 ]] && echo /root/Workspaces/scratches/morai || echo /root/inst/$1/morai; }
inrosn() { local n=$1; shift; (cd "$WS" && ROS_MASTER_URI=http://127.0.0.1:$((11310 + n)) ROS_IP=127.0.0.1 ROS_HOME=/root/stack/$n/ros \
  bash -c "source /opt/ros/noetic/setup.bash; source devel/setup.bash; $*"); }
MAXN=$(printf '%s\n' "${COMBOS[@]}" | cut -d: -f1 | sort -n | tail -1)
T=$(nproc --all)
if [[ ${CPU_LAYOUT:-shared} == shared ]]; then
  for n in $(seq "$MAXN"); do export CPUS_$n=0-$((T - 9)); done
  export STACK_CPUS=0-$((T - 9)) ENGINE_CPUS=$((T - 8))-$((T - 1)) SKIP_PIN=1
else
  PER=12; (( T < 12 * MAXN + 16 )) && PER=$(( (T - 16) / MAXN ))
  for n in $(seq "$MAXN"); do export CPUS_$n=$(( (n - 1) * PER ))-$(( n * PER - 1 )); done
  export STACK_CPUS=$((PER * MAXN))-$((PER * MAXN + 7)) ENGINE_CPUS=$((PER * MAXN + 8))-$((PER * MAXN + 15))
fi
export SYNC=1
stamp "sync_bench $NAME: $DELIVER, ${COMBOS[*]}, $(nvidia-smi --query-gpu=name,memory.total --format=csv,noheader), $T CPUs, quota $(cat /sys/fs/cgroup/cpu.max), $(( $(cat /sys/fs/cgroup/memory.max 2> /dev/null || echo 0) / 1000000000 )) GB, layout ${CPU_LAYOUT:-shared}"

if [[ -e /root/actor/server.pids ]] && kill -0 "$(cat /root/actor/server.pids)" 2> /dev/null; then
  stamp "engine already up"
else
  OMP_NUM_THREADS=4 MALLOC_MMAP_MAX_=0 MALLOC_TRIM_THRESHOLD_=1073741824 HF_HUB_OFFLINE=1 HF_HOME=$V/models/hf \
  TORCHINDUCTOR_CACHE_DIR=$V/cache/inductor TRITON_CACHE_DIR=$V/cache/triton \
    setsid taskset -c "$ENGINE_CPUS" $V/venv-engine/bin/python -u $E2E/server/model_server.py --backend engine --ckpt "$DELIVER" \
    --vlm Qwen/Qwen3-VL-4B-Instruct --sims "$MAXN" --gather "${GATHER:-0.1}" --prep "${PREP:-process}" --port 47011 \
    > "$OUT/server.log" 2>&1 < /dev/null &
  echo $! > /root/actor/server.pids
  stamp "engine starting (--sims $MAXN, gather ${GATHER:-0.1}, prep ${PREP:-process})"
fi
ENGINE=$(cat /root/actor/server.pids)

for combo in "${COMBOS[@]}"; do
  IFS=: read -r N TICK MODE <<< "$combo"
  C=$OUT/n${N}_t${TICK}_$MODE; mkdir -p "$C"
  for n in $(seq $((N + 1)) "$MAXN"); do
    pkill -f "inst/$n/Player[.]log" && echo "$(date +%T) w$n stopped" >> "$OUT/worlds.log"
    [[ -e /root/stack/$n/stack.pid ]] && { pkill -INT -s "$(cat /root/stack/$n/stack.pid)"; mv /root/stack/$n/stack.pid /root/stack/$n/stack.pid.old; }
  done
  bash "$HERE/worlds.sh" up "$N" >> "$OUT/worlds.log" 2>&1 || { stamp "worlds failed (worlds.log)"; exit 1; }
  bash "$HERE/stacks.sh" up "$N" >> "$OUT/stacks.log" 2>&1
  for n in $(seq "$N"); do until inrosn "$n" "rosnode list 2> /dev/null" | grep -q e2e_driver; do sleep 2; done; done
  if ! grep -q "listening on" "/proc/$ENGINE/fd/1" 2> /dev/null; then   # idle worlds burn ~3.8 cores each: the engine load and
    sims=$(pgrep -f "[S]imulator.real"); kill -STOP $sims             # compile get the pod's CPU quota (10/28: 8 worlds = 30 cores)
    stamp "worlds paused while the engine loads"
    until grep -q "listening on" "/proc/$ENGINE/fd/1" 2> /dev/null || ! kill -0 "$ENGINE" 2> /dev/null; do sleep 1; done
    kill -CONT $sims
  fi
  # every world's camera spinners on 4 CPUs: unpinned they burn ~4 cores per world and N=6 hit the quota (0.52 -> 0.76 sim-s/s)
  [[ ${CPU_LAYOUT:-shared} == shared ]] && SPIN_CPUS=$((T - 12))-$((T - 9)) bash "$HERE/pin.sh" >> "$OUT/worlds.log" 2>&1
  kill -0 "$ENGINE" 2> /dev/null || { stamp "engine died"; exit 1; }
  stamp "$combo: $N worlds, stacks and engine up"
  nvidia-smi --query-gpu=timestamp,utilization.gpu,memory.used,power.draw --format=csv,noheader,nounits -lms 1000 > "$C/gpu.csv" 2>&1 &
  GPU=$!
  pids=(); recs=()
  for n in $(seq "$N"); do
    inrosn "$n" "PYTHONUNBUFFERED=1 exec rostopic echo -p /e2e_driver/debug/data" > "$C/dbg_w$n.csv" 2> /dev/null < /dev/null &
    recs+=($!)
    (cd $E2E/tools && exec $PY -u eval_manager.py "w$n" --laps 1 --seed $((100 + n)) --scorer "$SCORER" --out "$C" \
      --port $((8094 + n)) --no-view --direct --host local --grpc 127.0.0.1:$((7788 + n)) --launcher "$(wdir "$n")/MoraiLauncher_Lin_Data" \
      --sync $((47100 + n)) --tick-ms "$TICK" $([[ $MODE == s ]] && echo --serial) > "$C/eval_w$n.log" 2>&1 < /dev/null) &
    pids+=($!)
  done
  echo "${pids[*]}" > /root/actor/managers.pids
  for i in $(seq 180); do   # every world stepping (scene loads take 10-60 s)
    ok=1; for n in $(seq "$N"); do [[ -s $C/w$n/steps.jsonl ]] || ok=0; done
    (( ok )) && break; sleep 2
  done
  stamp "$combo: driving ($(for n in $(seq "$N"); do [[ -s $C/w$n/steps.jsonl ]] && echo -n "w$n "; done)stepping)"
  (sleep $(( ${MEAS_S:-150} / 2 - 15 )); bash "$HERE/measure.sh" 30 > "$C/cpu.txt" 2>&1) &
  sleep "${MEAS_S:-150}"
  for p in "${pids[@]}"; do pkill -INT -P "$p" 2> /dev/null; kill -INT "$p" 2> /dev/null; done
  for p in "${pids[@]}"; do wait "$p"; done
  kill "${recs[@]}" $GPU 2> /dev/null; wait 2> /dev/null
  stamp "$combo: done ($(cat "$C"/w*/steps.jsonl 2> /dev/null | wc -l) steps)"
done
$PY "$HERE/sync_bench_summary.py" "$OUT" | tee "$OUT/bench.txt"
touch "$OUT/DONE"
