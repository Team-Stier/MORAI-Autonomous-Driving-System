#!/usr/bin/env bash
# dagger_box.sh NAME [SHADOW_EPISODES 2] [DRIVE_EPISODES 1] [SEED 28]: the DAgger shadow debug run on this pod after
# setup.sh (dagger_pod.sh starts it; README "DAgger 쉐도"). One world, real time:
#   1. shadow build (the collection fork snapshot) while actor.sh brings up world 1, the engine (wowmayo_final, --dump)
#      and stack 1; SHADOW=1: the collection driver as a shadow, restarted at every episode start
#   2. policy: SHADOW_EPISODES full laps (CP0 -> CP14 + 100 m or RANGE, random scene per episode, seed SEED): rl_episode spool
#      $V/spool/NAME_w1, shadow records $V/shadow/rec_w1, the stack's /e2e_driver/debug (late, model_ms), the command
#      publishers of both masters (shadow.sh check) and the CPU per part (measure.sh)
#   3. expert: the engine and stack 1 down, the collection driver drives world 1 itself (shadow.sh up 1 drive) for
#      DRIVE_EPISODES laps (seed SEED + 500): $V/spool/NAME_drive_w1 (no engine dumps), records $V/shadow/rec_w1
#   4. labels (dagger_label.py, all CPUs) for both; the drive's with the driven future (--actual)
#   5. B2: spool -> runs/dagger/shadow_debug/spool/, records, logs and results -> runs/dagger/shadow_debug/NAME/
# Out: $V/results/NAME (timeline.txt, logs, check.txt, cpu.txt, dbg_w1.csv, summary.json); DONE at the end.
set -uo pipefail
HERE=$(cd "$(dirname "$0")" && pwd); V=${V:-/workspace}; WS=$V/MORAI-Autonomous-Driving-System; E2E=$WS/src/e2e_driver
NAME=$1; K=${2:-2}; D=${3:-1}; SEED=${4:-28}; OUT=$V/results/$NAME; mkdir -p "$OUT" /root/actor
RANGE=${RANGE:---start-cp 0 --end-cp 14 --past-m 100 --returns --limit-s 900}   # full laps, failures return to the last CP
# (as the video laps), 15 min; e.g. "--start-cp 10 --end-cp 12" for the highway only
stamp() { echo "$(date +%s.%N | cut -c1-14) $(date +%T) $*" | tee -a "$OUT/timeline.txt"; }
inrosn() { local n=$1; shift; (cd "$WS" && ROS_MASTER_URI=http://127.0.0.1:$((11310 + n)) ROS_IP=127.0.0.1 bash -c "source /opt/ros/noetic/setup.bash; source devel/setup.bash; $*"); }
b2env() { eval "$(tr '\0' '\n' < /proc/1/environ | grep -E '^B2_APPLICATION_KEY(_ID)?=' | sed 's/^/export /')"; }
T=$(nproc --all)
if (( T >= 24 )); then export CPUS_1=0-9 STACK_CPUS=10-13 ENGINE_CPUS=14-$((T - 5)) SHADOW_CPUS=$((T - 4))-$((T - 1))
else export CPUS_1=0-7 STACK_CPUS=8-10 ENGINE_CPUS=11-13 SHADOW_CPUS=14-$((T - 1)); fi   # 16 CPUs: tight, measure.sh shows it
export E2E_APPLY_DELAY_S=0.1                          # as eval3: a plan observed at tau drives from tau + 0.1 s
RED_CAP=${RED_CAP:-5 20}                              # red waits end after 5-20 s (rl_episode --red-cap, user 10/28)
export TAKEOVER=${TAKEOVER:-1} TAKEOVER_S=$(( ${RED_CAP##* } + 5 ))   # start intervention after the longest red + 5 s
stamp "dagger_box $NAME: $K shadow laps, $D drive laps ($RANGE), seed $SEED, $(nvidia-smi --query-gpu=name --format=csv,noheader), $T CPUs, quota $(cat /sys/fs/cgroup/cpu.max); world $CPUS_1 stack $STACK_CPUS engine $ENGINE_CPUS shadow $SHADOW_CPUS"

SHADOW_CPUS=0-$((T - 1)) bash "$HERE/shadow.sh" build > "$OUT/shadow_build.log" 2>&1 &   # all CPUs until the engine loads
BUILD=$!
DELIVER=$V/models/wowmayo_final
SHADOW=1 ENGINE_ARGS="--backend engine --ckpt $DELIVER --vlm Qwen/Qwen3-VL-4B-Instruct --sims 1" \
  RL_ARGS="--limit-s 1100 $RANGE --episodes $K --seed $SEED --red-cap $RED_CAP" \
  bash "$HERE/actor.sh" up "$NAME" 1 > "$OUT/actor.log" 2>&1
wait $BUILD; [[ -e $V/shadow/BUILT ]] && stamp "shadow built" || { stamp "shadow build failed"; tail -30 $V/shadow/build.log; }
grep -q "listening on" /root/actor/server0.log && stamp "engine ready: $(grep -o 'loaded in.*' /root/actor/server0.log | cut -c1-100)" || stamp "engine failed"
[[ -e /root/actor/managers.pids ]] || { stamp "actor failed: $(tail -3 "$OUT/actor.log")"; touch "$OUT/DONE"; exit 1; }
stamp "policy laps started (shadow up)"
inrosn 1 "PYTHONUNBUFFERED=1 exec rostopic echo -p /e2e_driver/debug/data" > "$OUT/dbg_w1.csv" 2> /dev/null < /dev/null &
DBG=$!
(sleep 120; bash "$HERE/shadow.sh" check 1 > "$OUT/check.txt" 2>&1; bash "$HERE/measure.sh" 120 > "$OUT/cpu.txt" 2>&1) &
for p in $(cat /root/actor/managers.pids); do while kill -0 "$p" 2> /dev/null; do sleep 10; done; done
stamp "policy laps done: $(grep -c '^episode' /root/actor/episodes_w1.log) ($(grep '^episode' /root/actor/episodes_w1.log | cut -c1-120 | tr '\n' '|'))"
kill $DBG 2> /dev/null
cp /root/actor/*.log /root/stack/1/stack.log /root/shadow/1/*.log "$OUT/" 2> /dev/null; mkdir -p "$OUT/policy_logs"; mv "$OUT"/*.log "$OUT/policy_logs/" 2> /dev/null
mv /root/actor/managers.pids /root/actor/managers.pids.old
kill $(cat /root/actor/server.pids) 2> /dev/null; mv /root/actor/server.pids /root/actor/server.pids.old
bash "$HERE/stacks.sh" down > /dev/null; bash "$HERE/shadow.sh" down > /dev/null

if [[ ${FRAME_CHECK:-0} == 1 ]]; then                 # dagger-engine 2e4a2bb: frames prepared ahead, engine output identity
  mkdir -p /tmp/fs_reqs; i=0
  for f in $V/spool/${NAME}_w1/ep*/[0-9]*.req.bin; do ln -sf "$f" /tmp/fs_reqs/req_$(printf %06d $i).bin; i=$((i + 1)); done
  (cd $E2E && DELIVER=$DELIVER HF_HUB_OFFLINE=1 HF_HOME=$V/models/hf MORAI_VLA_PYLIB=$V/morai-student/pylib TORCHINDUCTOR_CACHE_DIR=$V/cache/inductor \
   TRITON_CACHE_DIR=$V/cache/triton timeout 1800 taskset -c "$ENGINE_CPUS" $V/venv-engine/bin/python test/test_frame_stream.py "$DELIVER/final" /tmp/fs_reqs \
   > "$OUT/frame_stream_check.log" 2>&1; stamp "test_frame_stream exit $?: $(grep -E 'engine ok|frame stream ok' "$OUT/frame_stream_check.log" | tr '\n' ' ' | cut -c1-200)") &
  FSC=$!
fi
if [[ ${LORA_CHECK:-0} == 1 ]]; then                  # dagger-engine: LLM LoRA swap check on this 5090 while the engine is down
  mkdir -p /tmp/lora_reqs; i=0
  for f in $V/spool/${NAME}_w1/ep*/[0-9]*.req.bin; do ln -sf "$f" /tmp/lora_reqs/req_$(printf %06d $i).bin; i=$((i + 1)); done
  (cd $E2E/server && HF_HUB_OFFLINE=1 HF_HOME=$V/models/hf MORAI_VLA_PYLIB=$V/morai-student/pylib TORCHINDUCTOR_CACHE_DIR=$V/cache/inductor \
   TRITON_CACHE_DIR=$V/cache/triton timeout 2700 taskset -c "$ENGINE_CPUS" $V/venv-engine/bin/python ../test/lora_llm_check.py "$DELIVER" \
   /tmp/lora_reqs 1 - "$OUT/lora_llm_check.json" > "$OUT/lora_llm_check.log" 2>&1; stamp "lora_llm_check exit $? ($i requests)") &
  LORA=$!
fi
if (( D > 0 )); then                                  # the collection driver drives world 1 itself
  export SHADOW_CPUS=10-$((T - 1))                     # the engine's and the stack's CPUs are free now
  bash "$HERE/shadow.sh" up 1 drive || stamp "shadow drive up failed"
  sleep 20
  d=/root/Workspaces/scratches/morai
  SCORER=$V/Alpamayo-Cosmos/tools/scorer setsid $V/venv-rl/bin/python -u $E2E/tools/rl_episode.py "${NAME}_drive_w1" \
    --scorer $V/Alpamayo-Cosmos/tools/scorer --out $V/spool --host local --direct --grpc 127.0.0.1:7789 --world 1 \
    --objinfo-port 27505 --launcher $d/MoraiLauncher_Lin_Data --port 8092 --seed $((SEED + 500)) \
    --loc-restart "bash $HERE/shadow.sh restart 1" --return-hook "bash $HERE/shadow.sh restart 1" --limit-s 1100 $RANGE \
    --episodes "$D" --red-cap $RED_CAP \
    > /root/actor/drive_w1.log 2>&1 < /dev/null &
  MGR=$!; stamp "expert drive started"
  while kill -0 $MGR 2> /dev/null; do sleep 10; done
  stamp "expert drive done: $(grep '^episode' /root/actor/drive_w1.log | cut -c1-120 | tr '\n' '|')"
  mkdir -p "$OUT/drive_logs"; cp /root/actor/drive_w1.log /root/shadow/1/*.log "$OUT/drive_logs/" 2> /dev/null
  bash "$HERE/shadow.sh" down > /dev/null
fi
bash "$HERE/worlds.sh" down > /dev/null
[[ -n ${LORA:-} ]] && wait $LORA
[[ -n ${FSC:-} ]] && wait $FSC

cfg=$(find "$DELIVER" -maxdepth 2 -name config.json | head -1)
LBL="$V/venv-engine/bin/python -u $E2E/tools/dagger_label.py --rec $V/shadow/rec_w1 --gt $V/shadow/gt --config $cfg --jobs $((T - 2))"
$LBL $V/spool/${NAME}_w1/ep[0-9]* > "$OUT/labels_policy.jsonl" 2> "$OUT/labels_policy.err" && stamp "labels (policy): $(cat "$OUT/labels_policy.jsonl" | tr '\n' ' ' | cut -c1-300)" || stamp "labels (policy) failed: $(tail -3 "$OUT/labels_policy.err")"
(( D > 0 )) && { $LBL --actual $V/spool/${NAME}_drive_w1/ep[0-9]* > "$OUT/labels_drive.jsonl" 2> "$OUT/labels_drive.err" && stamp "labels (drive): $(cat "$OUT/labels_drive.jsonl" | tr '\n' ' ' | cut -c1-300)" || stamp "labels (drive) failed: $(tail -3 "$OUT/labels_drive.err")"; }
gzip -k $V/shadow/rec_w1/*.jsonl 2> /dev/null
(b2env; B=$V/venv-tools/bin/b2; P=b2://morai2026-alpamayo/runs/dagger/shadow_debug
 $B sync --threads 32 --no-progress $V/spool/ $P/spool/ > "$OUT/b2_spool.log" 2>&1 &&
 $B sync --threads 16 --no-progress --exclude-regex '.*\.jsonl$' $V/shadow/rec_w1/ $P/$NAME/rec_w1/ > "$OUT/b2_rec.log" 2>&1 &&
 cp $V/shadow/build.log "$OUT/" && stamp "B2 spool and records" &&
 $B sync --threads 16 --no-progress "$OUT/" $P/$NAME/results/ > "$OUT/b2_results.log" 2>&1) && echo B2_OK > "$OUT/B2_OK" || stamp "B2 upload failed (results/$NAME/b2_*.log)"
if [[ -n ${LATE_SEED:-} ]]; then                      # late with frames prepared ahead: eval3, one world, one lap (boot2's set-up)
  stamp "eval3 ${NAME}_late (seed $LATE_SEED) for late"
  LAPS=1 LAP_DELIVER=0 VIEW_FPS=${VIEW_FPS:-15} bash "$HERE/eval3.sh" "${NAME}_late" "$DELIVER" "$LATE_SEED" > "$V/results/${NAME}_late.log" 2>&1
  stamp "eval3 late: $(head -3 $V/results/${NAME}_late/summary.txt 2> /dev/null | tr '\n' ' ' | cut -c1-300)"
  rsync -a --exclude '*.mp4' "$V/results/${NAME}_late/" "$OUT/late_eval3/" 2> /dev/null
  (b2env; $V/venv-tools/bin/b2 sync --threads 16 --no-progress "$OUT/late_eval3/" b2://morai2026-alpamayo/runs/dagger/shadow_debug/$NAME/late_eval3/ > /dev/null 2>&1)
fi
touch "$OUT/DONE"
