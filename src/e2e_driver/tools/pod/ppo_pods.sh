#!/usr/bin/env bash
# ppo_pods.sh up RUN LAUNCH N | down RUN   (Mac) the real-time PPO actors of run RUN, one vast RTX 5090 box and one MORAI
# world each, PAID (README "PPO 액터"). up: box k (0..N-1) = eval_pod.sh PROVIDER=vast BUNDLE=v1 CREATE_ONLY=1 (rent, B2
# bundle, setup.sh, this checkout, the competition account from ~/.morai-secrets) -> ppo_box.sh up on it: segment k of
# `SEGMENTS=round,hwy FULL_SHARE=0 segments.py N`, SEED_BASE 2000 + 100 LAUNCH + 10 k (LAUNCH 0-9, a new one per launch:
# no scene repeats). All boxes at once; logs ~/morai-eval/ppo/RUN/, B2 b2://morai2026-alpamayo/runs/ppo/RUN/.
#   Env for up, required: OFFERS (vast offer ids, `vastai search offers`; box k takes the k-th, ids after the first N are
#   spares every box tries next), SIGMA (engine --rl-sigma), BASE_B2 (deliver dir in the bucket, e.g. runs/stageC/deliver).
#   Optional: EXPERT2_ZONE ("[800, 1841.7]"), RL_ARGS (more rl_episode arguments, never --seed), EVERY (relay period, 30 s),
#   APPLY_DELAY_S (0.1), LOAD_MIN (eval_pod.sh).
# down: per box ppo_box.sh down (manager down, last relay pass: every finished episode on B2), its logs here, then vast
#   destroy. A box whose last pass failed (or without ssh) stays up and is named; FORCE=1 destroys it anyway.
set -uo pipefail
HERE=$(cd "$(dirname "$0")" && pwd); W=/workspace/MORAI-Autonomous-Driving-System/src/e2e_driver/tools/pod
CMD=${1:-}; RUN=${2:-}; [[ $CMD =~ ^(up|down)$ && -n $RUN ]] || { sed -n 2,14p "$0"; exit 1; }
D=$HOME/morai-eval/ppo/$RUN; mkdir -p "$D"
log() { echo "$(date +%T) $*" | tee -a "$D/ppo_pods.log"; }
id_of() { sed -n 's/.*vast create: instance \([0-9]*\).*/\1/p' "$HOME/morai-eval/pod/$1/eval_pod.log" 2> /dev/null | tail -1; }

case $CMD in
  up)
    LAUNCH=${3:?LAUNCH}; N=${4:?N}
    : "${OFFERS:?OFFERS=<vast offer ids>}" "${SIGMA:?SIGMA=<engine --rl-sigma>}" "${BASE_B2:?BASE_B2=<deliver dir in the bucket>}"
    (( LAUNCH >= 0 && LAUNCH <= 9 && N >= 1 && N <= 10 )) ||
      { log "LAUNCH 0-9, N 1-10: seeds 2000 + 100 LAUNCH + 10 k + 1 stay in the PPO range 2000-2999"; exit 1; }
    [[ " ${RL_ARGS:-} " == *" --seed"* ]] && { log "RL_ARGS: no --seed (SEED_BASE sets the scenes)"; exit 1; }
    read -r -a O <<< "$OFFERS"; (( ${#O[@]} >= N )) || { log "OFFERS: ${#O[@]} ids for $N boxes"; exit 1; }
    split=$(SEGMENTS=round,hwy FULL_SHARE=0 python3 "$HERE/segments.py" "$N") || exit 1
    SEGS=($(echo "$split" | sed -n 's/^actor [0-9]*: SEGMENT=\([a-z]*\) .*/\1/p'))
    log "run $RUN launch $LAUNCH: $N boxes, $(echo "$split" | head -1), sigma $SIGMA, delivery b2:$BASE_B2"
    env=$(printf 'SIGMA=%q BASE_B2=%q EXPERT2_ZONE=%q RL_ARGS=%q EVERY=%q APPLY_DELAY_S=%q' "$SIGMA" "$BASE_B2" \
      "${EXPERT2_ZONE:-[800, 1841.7]}" "${RL_ARGS:-}" "${EVERY:-30}" "${APPLY_DELAY_S:-0.1}")
    for ((k = 0; k < N; k++)); do
      BOX=ppo-$RUN-l$LAUNCH-b$k; SB=$((2000 + 100 * LAUNCH + 10 * k)); C=$HOME/morai-eval/pod/$BOX/ssh_config
      ( PROVIDER=vast BUNDLE=v1 CREATE_ONLY=1 OFFER="${O[k]} ${O[*]:N}" bash "$HERE/eval_pod.sh" "$BOX" "b2:$BASE_B2" \
          > "$D/$BOX.create.log" 2>&1 || { log "$BOX: no box ($D/$BOX.create.log)"; exit 1; }
        ssh -F "$C" pod "mkdir -p /workspace/results && $env setsid nohup bash $W/ppo_box.sh up $RUN $BOX ${SEGS[k]} $SB \
          > /workspace/results/$BOX.log 2>&1 < /dev/null &" &&
          log "$BOX: instance $(id_of "$BOX"), ${SEGS[k]}, seed $((SB + 1)); ssh -F $C pod tail -f /workspace/results/$BOX/timeline.txt" ) &
      sleep 5                                            # one vastai create after another
    done
    wait ;;
  down)
    for c in "$HOME"/morai-eval/pod/ppo-"$RUN"-l*/ssh_config; do
      [[ -e $c ]] || continue
      BOX=$(basename "$(dirname "$c")"); ID=$(id_of "$BOX")
      [[ -z $ID || -e $D/$BOX.destroyed ]] && continue
      grep -q "vast instance $ID destroyed" "$(dirname "$c")/eval_pod.log" && continue   # eval_pod.sh's own trap took it
      ( ok=0; ssh -F "$c" pod "bash $W/ppo_box.sh down $RUN $BOX" > /dev/null 2>&1 && ok=1
        mkdir -p "$D/$BOX"; rsync -rlt -e "ssh -F $c" "pod:/workspace/results/$BOX/" "pod:/workspace/results/$BOX.log" "$D/$BOX/" 2> /dev/null
        if [[ $ok == 1 || ${FORCE:-0} == 1 ]]; then
          vastai destroy instance "$ID" -y > /dev/null && touch "$D/$BOX.destroyed" &&
            log "$BOX: instance $ID destroyed ($([[ $ok == 1 ]] && echo "spool on B2" || echo "FORCE: last relay pass NOT confirmed"))"
        else
          log "$BOX: last relay pass failed or no ssh: instance $ID left up ($D/$BOX/timeline.txt); FORCE=1 destroys it"
        fi ) &
    done
    wait
    log "credit \$$(vastai show user --raw | python3 -c 'import json,sys; print(round(json.load(sys.stdin)["credit"], 2))')" ;;
esac
