#!/usr/bin/env bash
# eval_pod.sh NAME DELIVER [SEED ...]   (Mac) one closed-loop evaluation on a fresh RunPod GPU pod, end to end, PAID:
#   pod create (GraphQL podFindAndDeployOnDemand on the network volume, GPUs tried in order; start command = setup.sh, then
#   the idle watchdog) -> ssh -> rsync of this checkout to the volume (only changed files) -> eval3.sh NAME DELIVER SEEDS
#   (3 seeds = 3 MORAI worlds at once) -> results to ~/morai-eval/pod/NAME (no frames; B2 runs/eval_pod/NAME has all) ->
#   pod terminate (also on error or Ctrl-C; the volume stays). Stamps go to the pod's results/NAME/timeline.txt.
#   DELIVER: a dir on the volume (/workspace/models/wowmayo_final) or b2:<path> (runs/stageC/deliver)
#   Env: RUNPOD_API_KEY (default: ~/.runpod/config.toml), VOLUMES ("1tra2qsxbl:EU-CZ-1", id:data-centre list), GPUS
#   (5090, then RTX PRO 6000 Server, Workstation; one per line), RETRY_MIN (30: no stock -> again every minute), VCPU (32;
#   16 is enough for one world), MEM_GB (60), BUDGET_USD (watchdog cap per pod, 6),
#   KEEP=1 leaves the pod running at the end (stop it yourself), CREATE_ONLY=1 stops after setup (first boot, tests),
#   EVAL_MAX_MIN (110): create to end of eval3 before giving up; SIM_FPS / APPLY_DELAY_S pass through to eval3.sh (worlds.sh TargetFrameRate cap; tau delay of a plan, 0.1).
#   SYNC=1 (MORAI synchronous mode) with TICK_MS, SERIAL, CPU_LAYOUT, GATHER, PREP pass through too (eval3.sh).
set -uo pipefail
HERE=$(cd "$(dirname "$0")" && pwd); REPO=$(cd "$HERE/../../../.." && pwd); AC=$(cd "$REPO/../Alpamayo-Cosmos" && pwd)
NAME=$1; DELIVER=$2; shift 2; SEEDS=("$@")
KEY=${RUNPOD_API_KEY:-$(sed -n 's/^api_key = "\(.*\)"/\1/p' ~/.runpod/config.toml)}
read -r -a VOLUMES <<< "${VOLUMES:-1tra2qsxbl:EU-CZ-1}"     # VOLUME_ID:DATA_CENTER ..., tried in order
IFS=$'\n' read -r -d '' -a GPUS <<< "${GPUS:-NVIDIA GeForce RTX 5090
NVIDIA RTX PRO 6000 Blackwell Server Edition
NVIDIA RTX PRO 6000 Blackwell Workstation Edition}"
LOCAL=$HOME/morai-eval/pod/$NAME; mkdir -p "$LOCAL"; SSHC=$LOCAL/ssh_config
T0=$(date +%s.%N | cut -c1-14)
log() { echo "$(date +%T) $*" | tee -a "$LOCAL/eval_pod.log"; }
gql() { curl -s https://api.runpod.io/graphql -H 'content-type: application/json' -H "Authorization: Bearer $KEY" -d "$1"; }
POD=
bye() {
  [[ -z $POD ]] && return
  if [[ ${KEEP:-0} == 1 ]]; then log "KEEP=1: pod $POD left running ($(price) \$/h)"; return; fi
  curl -s -X DELETE "https://rest.runpod.io/v1/pods/$POD" -H "Authorization: Bearer $KEY" > /dev/null
  log "pod $POD terminated (volume $VOLUME kept); balance \$$(gql '{"query":"query { myself { clientBalance } }"}' | python3 -c 'import json,sys; print(round(json.load(sys.stdin)["data"]["myself"]["clientBalance"], 2))')"
  POD=
}
trap bye EXIT; trap 'exit 130' INT TERM

start="bash /workspace/MORAI-Autonomous-Driving-System/src/e2e_driver/tools/pod/setup.sh > /workspace/setup.\$(hostname).log 2>&1; \
DPH_USD=\$DPH_USD BUDGET_USD=${BUDGET_USD:-6} nohup bash /workspace/Alpamayo-Cosmos/tools/vast/idle_watchdog.sh > /dev/null 2>&1 & sleep infinity"
for try in $(seq $(( ${RETRY_MIN:-30} + 1 ))); do         # stock is Low: every GPU x volume, then again a minute later
for vd in "${VOLUMES[@]}"; do VOLUME=${vd%%:*}; DC=${vd#*:}
for g in "${GPUS[@]}"; do
  dph=2.49; [[ $g == *5090* ]] && dph=1.19; [[ $g == *Workstation* ]] && dph=2.59   # the watchdog's rate (Secure, 10/27)
  q=$(python3 - "$g" "$VOLUME" "$DC" "$start" "$(cat ~/.ssh/id_ed25519.pub)" "$dph" "wowmayo-eval-$NAME" <<'PY'
import json, os, sys
g, vol, dc, start, pub, dph, name = sys.argv[1:8]
env = dict(NVIDIA_DRIVER_CAPABILITIES="all", NVIDIA_VISIBLE_DEVICES="all", PUBLIC_KEY=pub, DPH_USD=dph,
           B2_APPLICATION_KEY_ID="{{ RUNPOD_SECRET_b2_key_id }}", B2_APPLICATION_KEY="{{ RUNPOD_SECRET_b2_app_key }}")
inp = (f'cloudType: SECURE, gpuCount: 1, gpuTypeId: {json.dumps(g)}, minVcpuCount: {int(os.environ.get("VCPU", 32))}, '
       f'minMemoryInGb: {int(os.environ.get("MEM_GB", 60))}, name: {json.dumps(name)}, imageName: "ros:noetic-ros-base-focal", '
       f'containerDiskInGb: 60, volumeInGb: 0, networkVolumeId: {json.dumps(vol)}, dataCenterId: {json.dumps(dc)}, '
       f'volumeMountPath: "/workspace", ports: "22/tcp", supportPublicIp: true, dockerArgs: {json.dumps("bash -c " + json.dumps(start))}, '
       'env: [' + ", ".join(f"{{key: {json.dumps(k)}, value: {json.dumps(v)}}}" for k, v in env.items()) + ']')
print(json.dumps({"query": "mutation { podFindAndDeployOnDemand(input: {%s}) { id costPerHr machine { gpuDisplayName } } }" % inp}))
PY
)
  r=$(gql "$q"); POD=$(echo "$r" | python3 -c 'import json,sys; d=json.load(sys.stdin); print(((d.get("data") or {}).get("podFindAndDeployOnDemand") or {}).get("id") or "")')
  [[ -n $POD ]] && { T0=$(date +%s.%N | cut -c1-14); log "pod create: $POD $g ($(echo "$r" | python3 -c 'import json,sys; print(json.load(sys.stdin)["data"]["podFindAndDeployOnDemand"]["costPerHr"])') \$/h, $DC)"; break 3; }
  (( try == 1 )) && log "no $g in $DC: $(echo "$r" | cut -c1-160)"
done; done
(( try == 1 )) && log "no GPU now: trying again every minute for ${RETRY_MIN:-30} min"; sleep 60
done
[[ -n $POD ]] || { log "no GPU pod available"; exit 1; }
price() { gql "{\"query\":\"query { pod(input:{podId:\\\"$POD\\\"}) { costPerHr } }\"}" | python3 -c 'import json,sys; print(json.load(sys.stdin)["data"]["pod"]["costPerHr"])'; }

for i in $(seq 120); do
  read -r ip port < <(curl -s "https://rest.runpod.io/v1/pods/$POD" -H "Authorization: Bearer $KEY" |
    python3 -c 'import json,sys; d=json.load(sys.stdin); print(d.get("publicIp") or "-", (d.get("portMappings") or {}).get("22", "-"))')
  [[ $ip != - && $port != - ]] && break; sleep 5
done
printf 'Host pod\n  HostName %s\n  Port %s\n  User root\n  StrictHostKeyChecking no\n  UserKnownHostsFile /dev/null\n  LogLevel ERROR\n  ServerAliveInterval 30\n' "$ip" "$port" > "$SSHC"
P() { ssh -F "$SSHC" pod "$@"; }
late() { (( $(date +%s) - ${T0%.*} > $1 )) && { log "$2: gave up"; exit 1; }; true; }   # the trap terminates the pod
until P true 2> /dev/null; do late 900 "no ssh 15 min after create (setup.sh died before sshd? /workspace/setup.<host>.log)"; sleep 3; done
log "ssh up ($ip:$port)"
rsync -rlt -e "ssh -F $SSHC" --exclude .git --exclude build --exclude devel --exclude __pycache__ --exclude '*.onnx' \
  --exclude object_detector/models --exclude traffic_light/models "$REPO/" pod:/workspace/MORAI-Autonomous-Driving-System/
rsync -rlt -e "ssh -F $SSHC" --exclude __pycache__ "$AC/tools/scorer" "$AC/tools/vast" pod:/workspace/Alpamayo-Cosmos/tools/
log "code synced"
until P "test -e /workspace/setup.\$(hostname).log && grep -q 'setup.*done\|rror' /workspace/setup.\$(hostname).log"; do late 1500 "setup"; sleep 3; done
P "tail -3 /workspace/setup.\$(hostname).log; nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader; nproc --all" | tee -a "$LOCAL/eval_pod.log"
P "grep -q 'setup.*done' /workspace/setup.\$(hostname).log" || { log "setup failed"; exit 1; }
log "setup done"
[[ ${CREATE_ONLY:-0} == 1 ]] && { KEEP=1; exit 0; }

P "mkdir -p /workspace/results/$NAME && echo '$T0 $(date -r "${T0%.*}" +%T) pod create ($POD)' > /workspace/results/$NAME/timeline.txt && \
   (SIM_FPS=${SIM_FPS:-60} APPLY_DELAY_S=${APPLY_DELAY_S:-0.1} SYNC=${SYNC:-0} TICK_MS=${TICK_MS:-100} SERIAL=${SERIAL:-0} \
    CPU_LAYOUT=${CPU_LAYOUT:-split} GATHER=${GATHER:-0.1} PREP=${PREP:-process} setsid nohup bash /workspace/MORAI-Autonomous-Driving-System/src/e2e_driver/tools/pod/eval3.sh $NAME $DELIVER ${SEEDS[*]} \
   > /workspace/results/$NAME.log 2>&1 < /dev/null &)"
log "eval3 running: ssh -F $SSHC pod tail -f /workspace/results/$NAME.log"
until P "test -e /workspace/results/$NAME/DONE || ! pgrep -f '[e]val3.sh $NAME' > /dev/null"; do late $((${EVAL_MAX_MIN:-110} * 60)) "eval3 (EVAL_MAX_MIN)"; sleep 20; done
rsync -rlt -e "ssh -F $SSHC" --exclude '*.mp4.raw' pod:/workspace/results/$NAME/ pod:/workspace/results/$NAME.log "$LOCAL/"
P "test -e /workspace/results/$NAME/DONE" && log "done: $LOCAL" || log "eval3 stopped early: $LOCAL/$NAME.log"
cat "$LOCAL/summary.txt" 2> /dev/null
