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
#   BUNDLE=TAG: no network volume, any data centre: the start command fetches the B2 bundle (bundle.py fetch TAG, ~1 GB
#   shards in parallel) to /workspace and runs setup.sh; the competition account comes from the Mac's ~/.morai-secrets
#   (config.tar, V2AccountInfo.byte; never in B2). Then N = the most worlds this host takes (VRAM, CPU quota, RAM;
#   README "sync") and the seeds run in waves of N, the parts kept up between them; REDRAW (1): scenes that failed the
#   replay check are drawn anew once (new seeds, one more wave).
#   ATTACH=<pod/instance id> (ATTACH_T0 = its create epoch): run the eval on a box this script set up before.
#   PROVIDER=vast OFFER=<vastai offer id> (with BUNDLE): a vast instance instead (B2 keys from the vast account's env
#   vars, ssh keys from the account; vastai CLI on the Mac). LAPS=K: K laps per world (seed S -> scenes S*1000+k);
#   LAP_DELIVER=1: each lap's 1x phone video (< 24 MB) and one-line report go to B2 runs/eval_vps/NAME/lap_KK/ and, from
#   the Mac, to the user's Discord DM (bot token in ~/.claude/channels/discord/.env, never printed; a failed attachment
#   falls back to a 7-day B2 link).
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
  if [[ ${PROVIDER:-runpod} == vast ]]; then
    vastai destroy instance "$POD" -y > /dev/null; log "vast instance $POD destroyed; credit \$$(vastai show user --raw | python3 -c 'import json,sys; print(round(json.load(sys.stdin)["credit"], 2))')"
    POD=; return
  fi
  curl -s -X DELETE "https://rest.runpod.io/v1/pods/$POD" -H "Authorization: Bearer $KEY" > /dev/null
  log "pod $POD terminated (volume $VOLUME kept); balance \$$(gql '{"query":"query { myself { clientBalance } }"}' | python3 -c 'import json,sys; print(round(json.load(sys.stdin)["data"]["myself"]["clientBalance"], 2))')"
  POD=
}
trap bye EXIT; trap 'exit 130' INT TERM

[[ -n ${BUNDLE:-} ]] && VOLUMES=(none:any)
hold=" sleep infinity"; [[ ${PROVIDER:-runpod} == vast ]] && hold=   # RunPod: the start command is the container's process
wdprep=; [[ ${PROVIDER:-runpod} == vast ]] && wdprep="python3 -m pip install -q vastai > /dev/null 2>&1; "   # the watchdog's vast stop
# BUNDLE: a stdlib-python bootstrap (the image has no curl) takes bundle.py from B2 and fetches the shards first
boot=$(printf '%s' "import base64,json,os,urllib.request as u
e=dict(l.split('=',1) for l in open('/proc/1/environ').read().split(chr(0)) if '=' in l); e.update(os.environ)
h={'Authorization':'Basic '+base64.b64encode((e['B2_APPLICATION_KEY_ID'].strip()+':'+e['B2_APPLICATION_KEY'].strip()).encode()).decode()}
a=json.load(u.urlopen(u.Request('https://api.backblazeb2.com/b2api/v2/b2_authorize_account',headers=h)))
open('/bundle.py','wb').write(u.urlopen(u.Request(a['downloadUrl']+'/file/morai2026-alpamayo/podimg/${BUNDLE:-}/bundle.py',headers={'Authorization':a['authorizationToken']})).read())" | base64 | tr -d '\n')
start="${BUNDLE:+mkdir -p /workspace && python3 -c \"\$(echo $boot | base64 -d)\" && python3 /bundle.py fetch $BUNDLE > /workspace/fetch.log 2>&1 && }\
bash /workspace/MORAI-Autonomous-Driving-System/src/e2e_driver/tools/pod/setup.sh > /workspace/setup.\$(hostname).log 2>&1; \
${wdprep}DPH_USD=\$DPH_USD BUDGET_USD=${BUDGET_USD:-6} nohup bash /workspace/Alpamayo-Cosmos/tools/vast/idle_watchdog.sh > /dev/null 2>&1 &$hold"
if [[ -n ${ATTACH:-} ]]; then   # a box this script started before (its eval3 died early): POD, ssh config and setup are there
  POD=$ATTACH; T0=${ATTACH_T0:-$T0}
  price() { vastai show instance "$POD" --raw | python3 -c 'import json,sys; print(round(json.load(sys.stdin)["dph_total"], 3))'; }
  log "attached to $POD"
elif [[ ${PROVIDER:-runpod} == vast ]]; then   # one offer, chosen beforehand (vastai search offers); Vulkan needs graphics caps
  r=$(vastai create instance "${OFFER:?OFFER=<vast offer id>}" --image ros:noetic-ros-base-focal --disk 150 --ssh --direct \
      --env "-e NVIDIA_DRIVER_CAPABILITIES=all -e NVIDIA_VISIBLE_DEVICES=all" --label "wowmayo-eval-$NAME" --onstart-cmd "$start" --raw)
  POD=$(echo "$r" | python3 -c 'import json,sys; print(json.load(sys.stdin).get("new_contract") or "")' 2> /dev/null)
  [[ -n $POD ]] || { log "vast create failed: $r"; exit 1; }
  T0=$(date +%s.%N | cut -c1-14); log "vast create: instance $POD (offer $OFFER)"
  price() { vastai show instance "$POD" --raw | python3 -c 'import json,sys; print(round(json.load(sys.stdin)["dph_total"], 3))'; }
  for i in $(seq 180); do
    read -r ip port st < <(vastai show instance "$POD" --raw | python3 -c 'import json,sys; d=json.load(sys.stdin); p=(d.get("ports") or {}).get("22/tcp") or [{}]
print(d.get("public_ipaddr") or "-", p[0].get("HostPort") or "-", d.get("actual_status"))')
    [[ $ip != - && $port != - && $st == running ]] && break; sleep 5
  done
  log "vast instance $POD: $st, $(price) \$/h, ssh $ip:$port"
else
for try in $(seq $(( ${RETRY_MIN:-30} + 1 ))); do         # stock is Low: every GPU x volume, then again a minute later
for vd in "${VOLUMES[@]}"; do VOLUME=${vd%%:*}; DC=${vd#*:}
for g in "${GPUS[@]}"; do
  dph=2.49; [[ $g == *5090* ]] && dph=1.19; [[ $g == *Workstation* ]] && dph=2.59   # the watchdog's rate (Secure, 10/27)
  q=$(python3 - "$g" "$VOLUME" "$DC" "$start" "$(cat ~/.ssh/id_ed25519.pub)" "$dph" "wowmayo-eval-$NAME" <<'PY'
import json, os, sys
g, vol, dc, start, pub, dph, name = sys.argv[1:8]
env = dict(NVIDIA_DRIVER_CAPABILITIES="all", NVIDIA_VISIBLE_DEVICES="all", PUBLIC_KEY=pub, DPH_USD=dph,
           B2_APPLICATION_KEY_ID="{{ RUNPOD_SECRET_b2_key_id }}", B2_APPLICATION_KEY="{{ RUNPOD_SECRET_b2_app_key }}")
where = (f'containerDiskInGb: 60, volumeInGb: 0, networkVolumeId: {json.dumps(vol)}, dataCenterId: {json.dumps(dc)}, volumeMountPath: "/workspace", '
         if vol != "none" else 'containerDiskInGb: 150, volumeInGb: 0, ')          # BUNDLE: ~50 GB of files + frames
inp = (f'cloudType: SECURE, gpuCount: 1, gpuTypeId: {json.dumps(g)}, minVcpuCount: {int(os.environ.get("VCPU", 32))}, '
       f'minMemoryInGb: {int(os.environ.get("MEM_GB", 60))}, name: {json.dumps(name)}, imageName: "ros:noetic-ros-base-focal", '
       f'{where}ports: "22/tcp", supportPublicIp: true, dockerArgs: {json.dumps("bash -c " + json.dumps(start))}, '
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
fi
[[ -z ${ATTACH:-} ]] && printf 'Host pod\n  HostName %s\n  Port %s\n  User root\n  StrictHostKeyChecking no\n  UserKnownHostsFile /dev/null\n  LogLevel ERROR\n  ServerAliveInterval 30\n' "$ip" "$port" > "$SSHC"
P() { ssh -F "$SSHC" pod "$@"; }
late() { (( $(date +%s) - ${T0%.*} > $1 )) && { log "$2: gave up"; exit 1; }; true; }   # the trap terminates the pod
[[ -z ${ATTACH:-} ]] && {
until P true 2> /dev/null; do late 900 "no ssh 15 min after create (setup.sh died before sshd? /workspace/setup.<host>.log)"; sleep 3; done
log "ssh up ($ip:$port)"
if [[ -n ${BUNDLE:-} ]]; then   # the competition account: from the Mac only (setup.sh waits for config.tar)
  M=/root/Workspaces/scratches/morai/MoraiLauncher_Lin_Data/Version
  until P "grep -q '^\[fetch\] ' /workspace/fetch.log"; do late 900 "bundle fetch (/workspace/fetch.log)"; sleep 5; done   # all shards in
  scp -q -F "$SSHC" ~/.morai-secrets/V2AccountInfo.byte pod:$M/ && P "mkdir -p /workspace/morai" &&
    scp -q -F "$SSHC" ~/.morai-secrets/config.tar pod:/workspace/morai/config.tar.part && P "mv /workspace/morai/config.tar.part /workspace/morai/config.tar"
  log "bundle fetched: $(P "tail -1 /workspace/fetch.log")"
fi
rsync -rlt -e "ssh -F $SSHC" --exclude .git --exclude build --exclude devel --exclude __pycache__ --exclude '*.onnx' \
  --exclude object_detector/models --exclude traffic_light/models "$REPO/" pod:/workspace/MORAI-Autonomous-Driving-System/
rsync -rlt -e "ssh -F $SSHC" --exclude __pycache__ "$AC/tools/scorer" "$AC/tools/vast" pod:/workspace/Alpamayo-Cosmos/tools/
log "code synced"
until P "test -e /workspace/setup.\$(hostname).log && grep -q 'setup.*done\|rror' /workspace/setup.\$(hostname).log"; do late 1500 "setup"; sleep 3; done
P "tail -3 /workspace/setup.\$(hostname).log; nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader; nproc --all" | tee -a "$LOCAL/eval_pod.log"
P "grep -q 'setup.*done' /workspace/setup.\$(hostname).log" || { log "setup failed"; exit 1; }
log "setup done"
}
[[ ${CREATE_ONLY:-0} == 1 ]] && { KEEP=1; exit 0; }

dm() {  # TEXT [FILE] -> HTTP code: a Discord DM to the user (memory discord-urgent-dm), the bot token never printed
  local T j; T=$(sed -n 's/^DISCORD_BOT_TOKEN=//p' ~/.claude/channels/discord/.env)
  j=$(python3 -c 'import json,sys; print(json.dumps({"content": sys.argv[1][:1900], "allowed_mentions": {"parse": []}}))' "$1")
  if [[ -n ${2:-} ]]; then curl -sS -m 600 -o /dev/null -w '%{http_code}' -H "Authorization: Bot $T" -F "payload_json=$j" \
    -F "files[0]=@$2" https://discord.com/api/v10/channels/1563731925322174534/messages
  else curl -sS -m 60 -o /dev/null -w '%{http_code}' -H "Authorization: Bot $T" -H 'Content-Type: application/json' -d "$j" \
    https://discord.com/api/v10/channels/1563731925322174534/messages; fi
}
deliver() {  # WN: every lap_KK.ready on the pod not sent yet -> its video and report to the user's DM
  local R=/workspace/results/$1 f line url vid code
  for f in $(P "ls $R/lap_*.ready 2> /dev/null"); do
    [[ -e $LOCAL/$(basename "$f").sent ]] && continue
    IFS=$'\t' read -r line url vid < <(P "cat $f")
    mkdir -p "$LOCAL/laps"; scp -q -F "$SSHC" "pod:$vid" "$LOCAL/laps/" 2> /dev/null
    code=$( [[ -s $LOCAL/laps/$(basename "$vid") ]] && dm "$line" "$LOCAL/laps/$(basename "$vid")" || echo none)
    [[ $code == 200 ]] || code="$code, link: $(dm "$line
영상(7일 링크): $url")"
    echo "$code" > "$LOCAL/$(basename "$f").sent"; log "lap $(basename "$f" .ready) sent to Discord ($code)"
  done
}

# worlds this host takes (sync_bench 10/28, 5090: N=7 used 29.2 GB VRAM, 23.5 cores, ~38 GB RAM): VRAM 3.13 N + 7.3 GB,
# CPU 3.1 N + 2.5 cores of 90 % of the quota, RAM 4.8 N + 4.5 GB of 85 %; the seeds run in waves of N
NMAX=$(P "python3 -c 'import subprocess as s; q=open(\"/sys/fs/cgroup/cpu.max\").read().split(); c=int(q[0])/int(q[1]) if q[0] != \"max\" else len(open(\"/proc/cpuinfo\").read().split(\"processor\"))-1; m=open(\"/sys/fs/cgroup/memory.max\").read().strip(); r=(int(m) if m != \"max\" else int(open(\"/proc/meminfo\").read().split()[1])*1024)/2**30; v=int(s.check_output([\"nvidia-smi\",\"--query-gpu=memory.total\",\"--format=csv,noheader,nounits\"]).split()[0])/1024; print(max(1, min(int((v-0.8-7.3)/3.13), int((0.9*c-2.5)/3.1), int((0.85*r-4.5)/4.8))), round(c,1), round(r), round(v,1))'")
read -r N CPUQ RAMG VRAMG <<< "$NMAX"; [[ ${SYNC:-0} == 1 ]] || N=${#SEEDS[@]}
log "host: CPU quota $CPUQ cores, RAM $RAMG GiB, VRAM $VRAMG GiB -> $N worlds at once; ${#SEEDS[@]} seeds"
P "mkdir -p /workspace/results/$NAME && echo '$T0 $(date -r "${T0%.*}" +%T) pod create ($POD)' > /workspace/results/$NAME/timeline.txt"
redraw=${REDRAW:-1}; next=$(( $(printf '%s\n' "${SEEDS[@]}" | sort -n | tail -1) + 1 ))
for ((w = 0; w < ${#SEEDS[@]}; w += N)); do
  wave=("${SEEDS[@]:w:N}"); WN=$NAME; (( w > 0 || ${#SEEDS[@]} > N )) && WN=${NAME}_wave$((w / N + 1))
  P "mkdir -p /workspace/results/$WN && cp -n /workspace/results/$NAME/timeline.txt /workspace/results/$WN/ && \
     (SIM_FPS=${SIM_FPS:-60} APPLY_DELAY_S=${APPLY_DELAY_S:-0.1} LAPS=${LAPS:-1} LAP_DELIVER=${LAP_DELIVER:-0} VIEW_FPS=${VIEW_FPS:-10} SYNC=${SYNC:-0} TICK_MS=${TICK_MS:-100} SERIAL=${SERIAL:-0} \
      KEEP_UP=1 CPU_LAYOUT=${CPU_LAYOUT:-} GATHER=${GATHER:-0.1} PREP=${PREP:-process} setsid nohup bash \
      /workspace/MORAI-Autonomous-Driving-System/src/e2e_driver/tools/pod/eval3.sh $WN $DELIVER ${wave[*]} > /workspace/results/$WN.log 2>&1 < /dev/null &)"
  sleep 10   # setsid may not have exec'd eval3 yet when the next ssh looks for it
  log "eval3 $WN (seeds ${wave[*]}) running: ssh -F $SSHC pod tail -f /workspace/results/$WN.log"
  until P "test -e /workspace/results/$WN/DONE || ! pgrep -f '[e]val3.sh $WN' > /dev/null"; do
    late $((${EVAL_MAX_MIN:-110} * 60)) "eval3 (EVAL_MAX_MIN)"; [[ ${LAP_DELIVER:-0} == 1 ]] && deliver "$WN"; sleep 20
  done
  [[ ${LAP_DELIVER:-0} == 1 ]] && deliver "$WN"
  mkdir -p "$LOCAL/$WN"; rsync -rlt -e "ssh -F $SSHC" --exclude '*.mp4.raw' pod:/workspace/results/$WN/ pod:/workspace/results/$WN.log "$LOCAL/$WN/"
  P "test -e /workspace/results/$WN/DONE" && log "done: $LOCAL/$WN" || { log "eval3 stopped early: $LOCAL/$WN/$WN.log"; break; }
  cat "$LOCAL/$WN/summary.txt" 2> /dev/null
  # scenes that did not play as generated (GT replay check failed): drawn anew once, as one more wave
  bad=$(python3 -c 'import json,sys; print(sum(d.get("replay_ok") is False for d in json.load(open(sys.argv[1]))["drives"]))' "$LOCAL/$WN/summary.json" 2> /dev/null || echo 0)
  if (( redraw && bad > 0 )); then
    log "$bad scene(s) failed the replay check: seeds $next.. drawn anew"; redraw=0
    for ((i = 0; i < bad; i++)); do SEEDS+=($((next + i))); done
  fi
done
