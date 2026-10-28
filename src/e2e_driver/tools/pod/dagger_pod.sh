#!/usr/bin/env bash
# dagger_pod.sh NAME [SHADOW_EPISODES 2] [DRIVE_EPISODES 1] [SEED 1990]   (Mac) the DAgger shadow debug run on a fresh vast
# 5090, end to end, PAID (README "DAgger 쉐도"): eval_pod.sh PROVIDER=vast BUNDLE=v1 CREATE_ONLY=1 (box, B2 bundle,
# setup, this checkout) -> the collection fork snapshot (MORAI-Autonomous-Driving-System-collect at e2cd63a, clean) and the
# morai-parallel-collect GT/R2 nodes -> dagger_box.sh -> results without engine dumps to ~/morai-eval/dagger/NAME ->
# destroy once B2 holds the results (else after pulling them here); also on error, Ctrl-C or MAX_MIN (150) after create.
#   Env: OFFER (vast offer ids, tried in order; eval_pod.sh), KEEP=1 leaves the box up (stop it yourself),
#   ATTACH=<instance id> ATTACH_NAME=<its first run's name>: run again on that box (e2e_driver code synced anew).
#   RANGE: rl_episode sections for dagger_box.sh (default the full lap); LATE_SEED: then one eval3 lap of that seed (late);
#   B2_SUB: B2 runs/dagger/<B2_SUB>/ (default shadow_debug).
set -uo pipefail
HERE=$(cd "$(dirname "$0")" && pwd); REPO=$(cd "$HERE/../../../.." && pwd)
FORK=$REPO/../MORAI-Autonomous-Driving-System-collect; GT=$HOME/.claude/skills/morai-parallel-collect/scripts/wsl
NAME=$1; K=${2:-2}; D=${3:-1}; SEED=${4:-1990}
LOCAL=$HOME/morai-eval/dagger/$NAME; mkdir -p "$LOCAL"; SSHC=$HOME/morai-eval/pod/$NAME/ssh_config
log() { echo "$(date +%T) $*" | tee -a "$LOCAL/dagger_pod.log"; }
[[ $(git -C "$FORK" rev-parse --short HEAD) == e2cd63a && -z $(git -C "$FORK" status --porcelain) ]] ||
  { log "collection fork is not a clean e2cd63a checkout: $FORK"; exit 1; }
COPYFILE_DISABLE=1 tar -C "$FORK" -cf "$LOCAL/shadow_src.tar" --no-mac-metadata --no-xattrs --no-fflags --exclude=.git --exclude=./build --exclude=./devel --exclude=./output \
  --exclude=./src/object_detector --exclude=./src/traffic_light --exclude=./src/visualizer --exclude=./src/velodyne \
  --exclude=./src/localization --exclude=./src/lidar_leveler --exclude='*.backup' . || exit 1
log "fork snapshot $(du -h "$LOCAL/shadow_src.tar" | cut -f1)"

if [[ -n ${ATTACH:-} ]]; then                         # a box this script set up before (ssh config of ATTACH_NAME)
  ID=$ATTACH; SSHC=$HOME/morai-eval/pod/${ATTACH_NAME:?}/ssh_config
  rsync -rlt -e "ssh -F $SSHC" --exclude .git --exclude build --exclude devel --exclude __pycache__ --exclude '*.onnx' \
    --exclude object_detector/models --exclude traffic_light/models "$REPO/src/e2e_driver/" pod:/workspace/MORAI-Autonomous-Driving-System/src/e2e_driver/
  rsync -rlt -e "ssh -F $SSHC" --exclude __pycache__ "$REPO/../Alpamayo-Cosmos/tools/scorer" pod:/workspace/Alpamayo-Cosmos/tools/
else
  PROVIDER=vast BUNDLE=v1 CREATE_ONLY=1 bash "$HERE/eval_pod.sh" "$NAME" /workspace/models/wowmayo_final
  ID=$(sed -n 's/.*vast create: instance \([0-9]*\).*/\1/p' "$HOME/morai-eval/pod/$NAME/eval_pod.log" | tail -1)
fi
T0=$(date +%s)
[[ -n $ID ]] || { log "no box"; exit 1; }
P() { ssh -F "$SSHC" pod "$@"; }
bye() {
  [[ -z $ID ]] && return
  mkdir -p "$LOCAL/results"
  rsync -rlt -e "ssh -F $SSHC" pod:/workspace/results/$NAME/ "$LOCAL/results/" 2> /dev/null
  rsync -rlt -e "ssh -F $SSHC" --exclude '*.bin' --exclude 'outside' pod:/workspace/spool/ "$LOCAL/spool/" 2> /dev/null
  rsync -rlt -e "ssh -F $SSHC" --include '*.gz' --exclude '*' pod:/workspace/shadow/rec_w1/ "$LOCAL/rec_w1/" 2> /dev/null
  log "results here: $(du -sh "$LOCAL" | cut -f1); B2 $([[ -e $LOCAL/results/B2_OK ]] && echo ok || echo 'NOT confirmed')"
  if [[ ${KEEP:-0} == 1 ]]; then log "KEEP=1: instance $ID left running"; return; fi
  vastai destroy instance "$ID" -y > /dev/null
  log "vast instance $ID destroyed; credit \$$(vastai show user --raw | python3 -c 'import json,sys; print(round(json.load(sys.stdin)["credit"], 2))')"
  ID=
}
trap bye EXIT; trap 'exit 130' INT TERM
log "instance $ID up; $(vastai show instance "$ID" --raw | python3 -c 'import json,sys; d=json.load(sys.stdin); print(d["dph_total"], "$/h", d.get("cpu_name"), d.get("geolocation"))')"
P "mkdir -p /workspace/shadow/gt /workspace/results/$NAME" && scp -q -F "$SSHC" "$LOCAL/shadow_src.tar" pod:/workspace/shadow/ &&
  rsync -rlt -e "ssh -F $SSHC" --include '*.py' --exclude '*' "$GT/" pod:/workspace/shadow/gt/ || { log "push failed"; exit 1; }
P "${RANGE:+RANGE='$RANGE' }${LATE_SEED:+LATE_SEED=$LATE_SEED }${B2_SUB:+B2_SUB=$B2_SUB }setsid nohup bash /workspace/MORAI-Autonomous-Driving-System/src/e2e_driver/tools/pod/dagger_box.sh $NAME $K $D $SEED > /workspace/results/$NAME.log 2>&1 < /dev/null &"
for i in $(seq 30); do P "test -s /workspace/results/$NAME/timeline.txt" && break; sleep 2; done
log "dagger_box running: ssh -F $SSHC pod tail -f /workspace/results/$NAME/timeline.txt"
seen=0
state() { P "test -e /workspace/results/$NAME/DONE && echo done || (pgrep -f '[d]agger_box.sh $NAME' > /dev/null && echo run || echo dead)" 2> /dev/null; }
gone=0
while :; do   # an ssh hiccup is not an end (10/28: "connection refused" for a minute read as dagger_box gone)
  st=$(state); [[ $st == done ]] && break
  [[ $st == dead ]] && { gone=$((gone + 1)); (( gone >= 2 )) && break; } || gone=0
  (( $(date +%s) - T0 > ${MAX_MIN:-150} * 60 )) && { log "MAX_MIN reached: giving up"; exit 1; }
  n=$(P "wc -l < /workspace/results/$NAME/timeline.txt" 2> /dev/null || echo "$seen")
  (( n > seen )) && { P "tail -n $((n - seen)) /workspace/results/$NAME/timeline.txt" | cut -c26- | while read -r l; do log "box: $l"; done; seen=$n; }
  sleep 30
done
P "tail -n +$((seen + 1)) /workspace/results/$NAME/timeline.txt" | cut -c26- | while read -r l; do log "box: $l"; done
P "test -e /workspace/results/$NAME/DONE" && log "done" || log "dagger_box stopped early: /workspace/results/$NAME.log"
