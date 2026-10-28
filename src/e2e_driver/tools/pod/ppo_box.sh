#!/usr/bin/env bash
# ppo_box.sh up RUN BOX SEGMENT SEED_BASE | down RUN BOX: one real-time PPO actor on this box after setup.sh (ppo_pods.sh
# runs it from the Mac; README "PPO 액터"). up: the delivery BASE_B2 from B2 -> ppo_relay.sh RUN in the background (LoRA
# down, spool up every EVERY s) -> actor.sh up SEGMENT 1: one world, the batched RL engine (--rl-sigma SIGMA --expert2
# --lora-watch $V/out/lora/latest.json), the node's second expert set on route s EXPERT2_ZONE, SEGMENT's preset (--returns),
# scenes of seed SEED_BASE + 1 -> waits on the episode manager. When it ends (or down): the last relay pass, everything
# down, logs to B2 runs/ppo/RUN/boxes/BOX/, DONE (and B2_OK when every finished episode is on B2); the idle watchdog then
# stops the box. down: the manager killed, then the same (exit 0 = B2_OK).
#   Env (up): SIGMA, BASE_B2 (required; a deliver dir, final/ + quant_manifest.json; a trailing /final is dropped),
#   EXPERT2_ZONE ("[800, 1841.7]"), RL_ARGS (no --seed), EVERY (30), APPLY_DELAY_S (0.1, as eval3 and dagger_box).
set -uo pipefail
HERE=$(cd "$(dirname "$0")" && pwd); V=${V:-/workspace}; RUN=${2:?RUN}; BOX=${3:?BOX}; OUT=$V/results/$BOX; mkdir -p "$OUT"
stamp() { echo "$(date +%s.%N | cut -c1-14) $(date +%T) $*" | tee -a "$OUT/timeline.txt"; }
b2env() { eval "$(tr '\0' '\n' < /proc/1/environ | grep -E '^B2_APPLICATION_KEY(_ID)?=' | sed 's/^/export /')"; }

finish() {  # after the manager: the last relay pass, the rest down, the logs up
  [[ -e $OUT/relay.pid ]] && pkill -s "$(cat "$OUT/relay.pid")" 2> /dev/null   # the loop and its b2 (setsid session)
  bash "$HERE/ppo_relay.sh" "$RUN" once >> "$OUT/relay.log" 2>&1 && touch "$OUT/B2_OK"
  stamp "last relay pass: $([[ -e $OUT/B2_OK ]] && echo "every finished episode on B2" || echo "FAILED (relay.log)")"
  bash "$HERE/actor.sh" down > /dev/null 2>&1
  cp /root/actor/*.log /root/stack/1/stack.log "$OUT/" 2> /dev/null
  (b2env; $V/venv-tools/bin/b2 sync --no-progress "$OUT/" "b2://morai2026-alpamayo/runs/ppo/$RUN/boxes/$BOX/" > /dev/null 2>&1) ||
    stamp "box logs to B2 failed"
  touch "$OUT/DONE"
}

case ${1:-} in
  up)
    SEG=${4:?SEGMENT}; SB=${5:?SEED_BASE}; : "${SIGMA:?SIGMA}" "${BASE_B2:?BASE_B2}"
    Z=${EXPERT2_ZONE:-"[800, 1841.7]"}; src=${BASE_B2%/}; src=${src%/final}; DELIVER=$V/models/$(basename "$src")
    T=$(nproc --all)                                    # dagger_box's CPU sets without the shadow
    if (( T >= 24 )); then export CPUS_1=0-9 STACK_CPUS=10-13 ENGINE_CPUS=14-$((T - 1))
    else export CPUS_1=0-7 STACK_CPUS=8-10 ENGINE_CPUS=11-$((T - 1)); fi
    stamp "ppo_box $BOX: run $RUN, segment $SEG, seed $((SB + 1)), sigma $SIGMA, delivery b2:$src, expert2 zone $Z, $(nvidia-smi --query-gpu=name --format=csv,noheader), $T CPUs, quota $(cat /sys/fs/cgroup/cpu.max)"
    (b2env; $V/venv-tools/bin/b2 sync --threads 32 --no-progress "b2://morai2026-alpamayo/$src/" "$DELIVER/" > "$OUT/b2_deliver.log" 2>&1) &&
      [[ -d $DELIVER/final ]] || { stamp "delivery b2:$src not synced (b2_deliver.log)"; touch "$OUT/DONE"; exit 1; }
    stamp "delivery synced ($(du -sh "$DELIVER" | cut -f1))"
    EVERY=${EVERY:-30} setsid bash "$HERE/ppo_relay.sh" "$RUN" > "$OUT/relay.log" 2>&1 < /dev/null &
    echo $! > "$OUT/relay.pid"
    E2E_APPLY_DELAY_S=${APPLY_DELAY_S:-0.1} E2E_EXPERT2_ZONE=$Z SEGMENT=$SEG SEED_BASE=$SB RL_ARGS=${RL_ARGS:-} \
    ENGINE_ARGS="--backend engine --ckpt $DELIVER --vlm Qwen/Qwen3-VL-4B-Instruct --sims 1 --rl-sigma $SIGMA --expert2 --lora-watch $V/out/lora/latest.json" \
      bash "$HERE/actor.sh" up "$SEG" 1 > "$OUT/actor.log" 2>&1
    if [[ -e /root/actor/managers.pids ]] && grep -q "listening on" /root/actor/server0.log; then
      stamp "actor up: $(grep -o 'listening on.*' /root/actor/server0.log | cut -c1-160)"
      for p in $(cat /root/actor/managers.pids); do while kill -0 "$p" 2> /dev/null; do sleep 10; done; done
      stamp "episode manager ended: $(grep -c '^episode' /root/actor/episodes_w1.log) episodes ($(tail -1 /root/actor/episodes_w1.log | cut -c1-160))"
    else
      stamp "actor failed: $(tail -2 "$OUT/actor.log" | tr '\n' ' ') $(tail -2 /root/actor/server0.log 2> /dev/null | tr '\n' ' ' | cut -c1-200)"
    fi
    finish ;;
  down)
    [[ -e /root/actor/managers.pids ]] && kill $(cat /root/actor/managers.pids) 2> /dev/null
    for i in $(seq 180); do   # up notices the manager within 10 s and finishes (15 min cap)
      [[ -e $OUT/DONE ]] || ! pgrep -f "[p]po_box.sh up $RUN $BOX " > /dev/null && break; sleep 5
    done
    [[ -e $OUT/DONE ]] || finish
    [[ -e $OUT/B2_OK ]] ;;
  *) sed -n 2,10p "$0"; exit 1 ;;
esac
