#!/usr/bin/env bash
# eval_pod.sh NAME DELIVER [SEED ...]   (Mac; no SEED: a new random one) one closed-loop evaluation on a fresh RunPod GPU pod, end to end, PAID:
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
#   PROVIDER=vast OFFER="<vastai offer id> ..." (with BUNDLE; the next offer when one is not running after LOAD_MIN, 6): a vast instance instead (B2 keys from the vast account's env
#   vars, ssh keys from the account; vastai CLI on the Mac). LAPS=K: K laps per world (seed S -> scenes S*1000+k);
#   LAP_DELIVER=1: each lap's 1x phone video (< 24 MB) and one-line report go to B2 runs/eval_vps/NAME/lap_KK/ and, from
#   the Mac, to the user's Discord DM (bot token in ~/.claude/channels/discord/.env, never printed; a failed attachment
#   falls back to a 7-day B2 link). MSG_PREFIX starts each lap message (e.g. "[R2 미세조정 납품본] ").
#   DELIVER b2:<path> (a qad_export.py deliver dir: final/ + quant_manifest.json) is fetched on the box (eval3.sh).
# Campaign (HARNESS.md): DRIVER=stack (default) drives the competition stack of BRANCH (drive_prep.sh: any ref of the
#   user's checkout or origin, default main) instead of the e2e engine; DELIVER is then ignored (give -). The checkout
#   goes to the pod's /workspace/drive, drive_setup.sh builds it (ONNX Runtime GPU, rosdep, catkin_make) after setup.sh.
#   ATTACH=<pod> with BRANCH=<ref>: the stacks go down, the new ref is synced and built, then the eval runs on it.
#   ACCOUNT=1 (with ATTACH): copy ~/.morai-secrets to the pod and unpack it (an account that came after the boot).
#   Defaults: GPUS RTX 5080 / 3080 / 3090 / 4090 / A5000 / 3070 (Vulkan), CLOUD=COMMUNITY, VCPU 12, MEM_GB 28, DISK_GB 100. Pod create and
#   terminate are logged to PODS_LOG (/home/opc/morai-campaign/shared/pods.log). VERSION=vN: Discord prefix "[vN] "
#   (before MSG_PREFIX) and every delivered lap's lap_KK.json to RESULTS_DIR/vN/ (campaign_results.py aggregates).
#   NO_EVAL=1: stop after the deploy (pod kept, as CREATE_ONLY).
#   HARNESS v2: ~/morai-eval/pod/<name>/driver.pid = this driver's PID (stop a run with kill "$(cat .../driver.pid)" only;
#   never pkill/pgrep patterns: other sessions' drivers run on this host). CPU gate after ssh up (a new pod): CPU model
#   not EPYC 7xxx / Xeon and a 10 M-iteration python loop (best of 3) <= CPU_BENCH_MAX_S (0.80; i5-13600K 0.52) or the
#   pod is deleted and another created (CPU_RETRIES, 6). ENV_INVALID from eval3.sh (its guard): the run ends; a new
#   pod (not ATTACH) is then replaced the same way.
set -uo pipefail
# run from a private copy: an edit of this file (git checkout, another session) cannot break a run in progress
if [[ -z ${EVAL_POD_HERE:-} ]]; then
  self=$(mktemp /tmp/eval_pod.XXXXXX); cp "$0" "$self"
  EVAL_POD_HERE=$(cd "$(dirname "$0")" && pwd) exec bash "$self" "$@"
fi
# GPUS / VOLUMES become arrays below and bash exports no arrays: the given lists are kept for a re-exec (CPU gate)
export EVAL_POD_GPUS=${EVAL_POD_GPUS:-${GPUS:-}} EVAL_POD_VOLUMES=${EVAL_POD_VOLUMES:-${VOLUMES:-}}
BUNDLE=${BUNDLE:-}; DRIVER=${DRIVER:-stack}; BRANCH=${BRANCH:-}; [[ $DRIVER == stack && -z ${ATTACH:-} && -z $BRANCH ]] && BRANCH=main
PODS_LOG=${PODS_LOG:-/home/opc/morai-campaign/shared/pods.log}; RESULTS_DIR=${RESULTS_DIR:-/home/opc/morai-campaign/shared/results}
[[ -n ${VERSION:-} ]] && MSG_PREFIX="[${VERSION}] ${MSG_PREFIX:-}"
podlog() { echo "$(date '+%F %T %Z') $*" >> "$PODS_LOG"; }
HERE=$EVAL_POD_HERE; REPO=$(cd "$HERE/../../../.." && pwd); AC=$(cd "$REPO/../Alpamayo-Cosmos" && pwd)
ORIG_ARGS=("$@"); NAME=$1; DELIVER=$2; shift 2; SEEDS=("$@")
# no seed given: a new one per run (scenes S*1000+k, in laps.jsonl, every lap report and the results name NAME_sS); give
# one only to replay scenes. draw: eval seeds are 0-99 (DAgger actors 1000+, PPO 2000+), never 7, 28 or 29 (the scenes of
# earlier runs, user 10/28) or a seed of this run
draw() { python3 -c 'import random,sys; print(random.choice([s for s in range(100) if s not in {7, 28, 29, *map(int, sys.argv[1:])}]))' "$@"; }
(( ${#SEEDS[@]} )) || SEEDS=($(draw))
RUN=${NAME}_s$(IFS=-; echo "${SEEDS[*]}")   # results and B2 runs/eval_vps/RUN; LOCAL (ssh_config) keeps NAME
KEY=${RUNPOD_API_KEY:-$(sed -n -E "s/^api_?key *= *[\"']?([^\"']*)[\"']?.*/\1/p" ~/.runpod/config.toml)}
read -r -a VOLUMES <<< "${EVAL_POD_VOLUMES:-1tra2qsxbl:EU-CZ-1}"     # VOLUME_ID:DATA_CENTER ..., tried in order
IFS=$'\n' read -r -d '' -a GPUS <<< "${EVAL_POD_GPUS:-NVIDIA GeForce RTX 5080
NVIDIA GeForce RTX 3080
NVIDIA GeForce RTX 3090
NVIDIA GeForce RTX 4090
NVIDIA RTX A5000
NVIDIA GeForce RTX 3070}"
LOCAL=$HOME/morai-eval/pod/$NAME; mkdir -p "$LOCAL"; SSHC=$LOCAL/ssh_config; echo $$ > "$LOCAL/driver.pid"
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
  podlog "terminate $POD ($NAME)"
  log "pod $POD terminated (volume ${VOLUME:-none} kept); balance \$$(gql '{"query":"query { myself { clientBalance } }"}' | python3 -c 'import json,sys; print(round(json.load(sys.stdin)["data"]["myself"]["clientBalance"], 2))')"
  POD=
}
trap bye EXIT; trap 'exit 130' INT TERM

[[ -n ${BUNDLE:-} ]] && VOLUMES=(none:any)
hold=" sleep infinity"; [[ ${PROVIDER:-runpod} == vast ]] && hold=   # RunPod: the start command is the container's process
wdprep=; [[ ${PROVIDER:-runpod} == vast ]] && wdprep="python3 -m pip install -q vastai > /dev/null 2>&1; "   # the watchdog's vast stop
# BUNDLE: a stdlib-python bootstrap (the image has no curl) takes bundle.py from B2 and fetches the shards first
boot=$(printf '%s' "import base64,json,os,zlib,urllib.request as u
e=dict(l.split('=',1) for l in open('/proc/1/environ').read().split(chr(0)) if '=' in l); e.update(os.environ)
h={'Authorization':'Basic '+base64.b64encode((e['B2_APPLICATION_KEY_ID'].strip()+':'+e['B2_APPLICATION_KEY'].strip()).encode()).decode()}
a=json.load(u.urlopen(u.Request('https://api.backblazeb2.com/b2api/v2/b2_authorize_account',headers=h)))
open('/bundle.py','wb').write(zlib.decompress(base64.b64decode('$(python3 -c 'import base64,sys,zlib; print(base64.b64encode(zlib.compress(open(sys.argv[1],"rb").read(),9)).decode())' "$HERE/bundle.py")')))" | base64 | tr -d '\n')
# sshd before the bundle (ssh and the fetch log are watchable while ~27 GB come in; setup.sh starts it again)
early="${BUNDLE:+(apt-get update -qq && DEBIAN_FRONTEND=noninteractive apt-get install -y -qq --no-install-recommends openssh-server rsync > /dev/null; \
mkdir -p ~/.ssh /run/sshd && chmod 700 ~/.ssh && echo \"\$PUBLIC_KEY\" >> ~/.ssh/authorized_keys && service ssh start) > /workspace/early.log 2>&1; }"
start="${BUNDLE:+mkdir -p /workspace && $early python3 -c \"\$(echo $boot | base64 -d)\" && STREAMS=\${STREAMS:-16} python3 /bundle.py fetch $BUNDLE > /workspace/fetch.log 2>&1 && }\
ACCOUNT_WAIT_S=$([[ -s ~/.morai-secrets/config.tar ]] && echo 600 || echo 4) bash /workspace/MORAI-Autonomous-Driving-System/src/e2e_driver/tools/pod/setup.sh > /workspace/setup.\$(hostname).log 2>&1; \
${wdprep}DPH_USD=\$DPH_USD BUDGET_USD=${BUDGET_USD:-6} nohup bash /workspace/Alpamayo-Cosmos/tools/vast/idle_watchdog.sh > /dev/null 2>&1 &$hold"
if [[ -n ${ATTACH:-} ]]; then   # a box this script started before (its eval3 died early): POD, ssh config and setup are there
  POD=$ATTACH; T0=${ATTACH_T0:-$T0}
  if [[ ${PROVIDER:-runpod} == vast ]]; then price() { vastai show instance "$POD" --raw | python3 -c 'import json,sys; print(round(json.load(sys.stdin)["dph_total"], 3))'; }
  else price() { gql "{\"query\":\"query { pod(input:{podId:\\\"$POD\\\"}) { costPerHr } }\"}" | python3 -c 'import json,sys; print(json.load(sys.stdin)["data"]["pod"]["costPerHr"])'; }; fi
  log "attached to $POD"
elif [[ ${PROVIDER:-runpod} == vast ]]; then   # offers chosen beforehand (vastai search offers); Vulkan needs graphics caps
  for off in ${OFFER:?OFFER=<vast offer id(s), tried in order>}; do   # a host may sit in 'loading' (image pull) for long
    r=$(vastai create instance "$off" --image ros:noetic-ros-base-focal --disk 150 --ssh --direct \
        --env "-e NVIDIA_DRIVER_CAPABILITIES=all -e NVIDIA_VISIBLE_DEVICES=all" --label "wowmayo-eval-$NAME" --onstart-cmd "$start" --raw)
    POD=$(echo "$r" | python3 -c 'import json,sys; print(json.load(sys.stdin).get("new_contract") or "")' 2> /dev/null)
    [[ -n $POD ]] || { log "vast create $off failed: $r"; continue; }
    T0=$(date +%s.%N | cut -c1-14); log "vast create: instance $POD (offer $off)"
    for i in $(seq $(( ${LOAD_MIN:-6} * 12 ))); do
      read -r ip port st < <(vastai show instance "$POD" --raw | python3 -c 'import json,sys; d=json.load(sys.stdin); p=(d.get("ports") or {}).get("22/tcp") or [{}]
print(d.get("public_ipaddr") or "-", p[0].get("HostPort") or "-", d.get("actual_status"))')
      [[ $ip != - && $port != - && $st == running ]] && break; sleep 5
    done
    [[ $st == running && $port != - ]] && break
    log "instance $POD still '$st' after ${LOAD_MIN:-6} min: destroyed, next offer"; vastai destroy instance "$POD" -y > /dev/null; POD=
  done
  [[ -n $POD ]] || { log "no vast offer came up"; exit 1; }
  price() { vastai show instance "$POD" --raw | python3 -c 'import json,sys; print(round(json.load(sys.stdin)["dph_total"], 3))'; }
  log "vast instance $POD: $st, $(price) \$/h, ssh $ip:$port"
else
for try in $(seq $(( ${RETRY_MIN:-30} + 1 ))); do         # stock is Low: every GPU x volume, then again a minute later
for vd in "${VOLUMES[@]}"; do VOLUME=${vd%%:*}; DC=${vd#*:}
for g in "${GPUS[@]}"; do
  dph=0.5; [[ $g == *5090* ]] && dph=1.19; [[ $g == *PRO\ 6000* ]] && dph=2.59   # the watchdog's rate (an upper estimate)
  q=$(python3 - "$g" "$VOLUME" "$DC" "$start" "$(cat ~/.ssh/id_ed25519.pub)" "$dph" "wowmayo-eval-$NAME" <<'PY'
import json, os, sys
g, vol, dc, start, pub, dph, name = sys.argv[1:8]
env = dict(NVIDIA_DRIVER_CAPABILITIES="all", NVIDIA_VISIBLE_DEVICES="all", PUBLIC_KEY=pub, DPH_USD=dph,
           B2_APPLICATION_KEY_ID="{{ RUNPOD_SECRET_b2_key_id }}", B2_APPLICATION_KEY="{{ RUNPOD_SECRET_b2_app_key }}")
where = (f'containerDiskInGb: 60, volumeInGb: 0, networkVolumeId: {json.dumps(vol)}, dataCenterId: {json.dumps(dc)}, volumeMountPath: "/workspace", '
         if vol != "none" else f'containerDiskInGb: {int(os.environ.get("DISK_GB", 100))}, volumeInGb: 0, ')          # BUNDLE: ~50 GB of files + frames
inp = (f'cloudType: {os.environ.get("CLOUD", "COMMUNITY")}, gpuCount: 1, gpuTypeId: {json.dumps(g)}, minVcpuCount: {int(os.environ.get("VCPU", 12))}, '
       f'minMemoryInGb: {int(os.environ.get("MEM_GB", 28))}, name: {json.dumps(name)}, imageName: "ros:noetic-ros-base-focal", '
       f'{where}ports: "22/tcp", supportPublicIp: true, dockerArgs: {json.dumps("bash -c " + json.dumps(start))}, '
       'env: [' + ", ".join(f"{{key: {json.dumps(k)}, value: {json.dumps(v)}}}" for k, v in env.items()) + ']')
print(json.dumps({"query": "mutation { podFindAndDeployOnDemand(input: {%s}) { id costPerHr machine { gpuDisplayName } } }" % inp}))
PY
)
  r=$(gql "$q"); POD=$(echo "$r" | python3 -c 'import json,sys; d=json.load(sys.stdin); print(((d.get("data") or {}).get("podFindAndDeployOnDemand") or {}).get("id") or "")')
  [[ -n $POD ]] && { T0=$(date +%s.%N | cut -c1-14); podlog "create $POD $g ${CLOUD:-COMMUNITY} $(echo "$r" | python3 -c 'import json,sys; print(json.load(sys.stdin)["data"]["podFindAndDeployOnDemand"]["costPerHr"])') \$/h ($NAME)"; log "pod create: $POD $g ($(echo "$r" | python3 -c 'import json,sys; print(json.load(sys.stdin)["data"]["podFindAndDeployOnDemand"]["costPerHr"])') \$/h, $DC)"; break 3; }
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
read -r bench cpu < <(P "python3 -c 'import time
b=9.9
for _ in range(3):
    t=time.perf_counter(); s=0
    for i in range(10_000_000): s+=i*i
    b=min(b, time.perf_counter()-t)
m=[l.split(\":\",1)[1].strip() for l in open(\"/proc/cpuinfo\") if l.startswith(\"model name\")][0]
print(round(b, 3), m)'")
log "CPU: $cpu, bench ${bench:-?} s (max ${CPU_BENCH_MAX_S:-0.80})"
if [[ -z $bench || $cpu == *EPYC\ 7* || $cpu == *Xeon* ]] || python3 -c "import sys; sys.exit(0 if float(sys.argv[1]) > float(sys.argv[2]) else 1)" "$bench" "${CPU_BENCH_MAX_S:-0.80}"; then
  podlog "reject $POD ($NAME): CPU $cpu bench ${bench:-?} s"
  (( ${CPU_RETRY:-0} < ${CPU_RETRIES:-6} )) || { log "CPU gate: $((CPU_RETRY)) pods rejected, giving up"; KEEP=0 bye; exit 1; }
  KEEP=0 bye; log "CPU gate: pod rejected (deleted even with KEEP=1), creating another"
  CPU_RETRY=$(( ${CPU_RETRY:-0} + 1 )) exec bash "$0" "${ORIG_ARGS[@]}"
fi
if [[ -n ${BUNDLE:-} ]]; then   # the competition account: from the Mac only (setup.sh waits for config.tar)
  M=/root/Workspaces/scratches/morai/MoraiLauncher_Lin_Data/Version
  until P "grep -q '^\[fetch\] ' /workspace/fetch.log"; do late ${FETCH_MAX_S:-3000} "bundle fetch (/workspace/fetch.log)"; sleep 10; done   # all shards in
  if [[ -s ~/.morai-secrets/config.tar ]]; then
  scp -q -F "$SSHC" ~/.morai-secrets/V2AccountInfo.byte pod:$M/ && P "mkdir -p /workspace/morai" &&
    scp -q -F "$SSHC" ~/.morai-secrets/config.tar pod:/workspace/morai/config.tar.part && P "mv /workspace/morai/config.tar.part /workspace/morai/config.tar"
  else log "no ~/.morai-secrets yet: the pod builds without the MORAI account (ACCOUNT=1 ATTACH=$POD later)"; fi
  log "bundle fetched: $(P "tail -1 /workspace/fetch.log")"
fi
rsync -rlt -e "ssh -F $SSHC" --exclude .git --exclude build --exclude devel --exclude __pycache__ --exclude '*.onnx' \
  --exclude object_detector/models --exclude traffic_light/models "$REPO/" pod:/workspace/MORAI-Autonomous-Driving-System/
rsync -rlt -e "ssh -F $SSHC" --exclude __pycache__ "$AC/tools/scorer" "$AC/tools/vast" pod:/workspace/Alpamayo-Cosmos/tools/
log "code synced"
until P "test -e /workspace/setup.\$(hostname).log && grep -q 'setup.*done\|rror' /workspace/setup.\$(hostname).log"; do late $(( ${FETCH_MAX_S:-3000} + 900 )) "setup"; sleep 3; done
P "tail -3 /workspace/setup.\$(hostname).log; nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader; nproc --all" | tee -a "$LOCAL/eval_pod.log"
P "grep -q 'setup.*done' /workspace/setup.\$(hostname).log" || { log "setup failed"; exit 1; }
log "setup done"
}
M=/root/Workspaces/scratches/morai/MoraiLauncher_Lin_Data/Version
if [[ ${ACCOUNT:-0} == 1 ]]; then   # the account after the boot: as setup.sh would have unpacked it
  scp -q -F "$SSHC" ~/.morai-secrets/V2AccountInfo.byte pod:$M/ && P "mkdir -p /workspace/morai" &&
    scp -q -F "$SSHC" ~/.morai-secrets/config.tar pod:/workspace/morai/config.tar && P "tar -xf /workspace/morai/config.tar -C /root/.config && \
    echo '-force-vulkan -force-gfx-mt -job-worker-count 8' > /root/.config/morai-stier/sim-args" && log "MORAI account installed" || { log "account copy failed"; exit 1; }
fi
[[ ${CREATE_ONLY:-0} == 1 && $DRIVER != stack ]] && { KEEP=1; exit 0; }
if [[ $DRIVER == stack && -n $BRANCH ]]; then   # the drive checkout: prepared here, synced, built on the pod
  rsync -rlt -e "ssh -F $SSHC" --exclude .git --exclude build --exclude devel --exclude __pycache__ \
    "$REPO/src/e2e_driver/tools/" pod:/workspace/MORAI-Autonomous-Driving-System/src/e2e_driver/tools/ &&   # an ATTACH gets the harness too
  rsync -rlt -e "ssh -F $SSHC" --exclude __pycache__ "$AC/tools/scorer" "$AC/tools/vast" pod:/workspace/Alpamayo-Cosmos/tools/ || { log "harness sync failed"; exit 1; }
  read -r SHA _ SNAPDIR < <(bash "$HERE/drive_prep.sh" "$BRANCH") && [[ -s $SNAPDIR/SHA ]] || { log "drive_prep $BRANCH failed"; exit 1; }
  P "bash /workspace/MORAI-Autonomous-Driving-System/src/e2e_driver/tools/pod/stacks.sh down > /dev/null 2>&1; mkdir -p /workspace/drive"
  rsync -rlt --delete -e "ssh -F $SSHC" --exclude .git --exclude /build --exclude /devel --exclude /REF --exclude /BUILT \
    "$SNAPDIR/" pod:/workspace/drive/ && P "echo '$BRANCH' > /workspace/drive/REF" || { log "drive sync failed"; exit 1; }
  log "drive $BRANCH (${SHA:0:10}) synced, building"
  P "touch /workspace/keepalive; (while sleep 120; do touch /workspace/keepalive; done) & k=\$!; bash /workspace/MORAI-Autonomous-Driving-System/src/e2e_driver/tools/pod/drive_setup.sh > /workspace/drive_setup.log 2>&1; r=\$?; kill \$k; exit \$r" ||
    { log "drive build failed: $(P "tail -3 /workspace/drive_setup.log")"; exit 1; }
  BUILT=$(P "cat /workspace/drive/BUILT"); log "drive built: $BUILT"
  [[ ${BUILT%% *} == "$SHA" ]] || { log "built sha ${BUILT%% *} is not $SHA"; exit 1; }
  echo "$BUILT" > "$LOCAL/built_sha"
fi
[[ ${CREATE_ONLY:-0} == 1 || ${NO_EVAL:-0} == 1 ]] && { KEEP=1; exit 0; }

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
    line="${MSG_PREFIX:-}$line"
    code=$( [[ -s $LOCAL/laps/$(basename "$vid") ]] && dm "$line" "$LOCAL/laps/$(basename "$vid")" || echo none)
    [[ $code == 200 ]] || code="$code, link: $(dm "$line
영상(7일 링크): $url")"
    if [[ -n ${VERSION:-} ]]; then   # the lap's numbers for campaign_results.py
      local K; K=$(basename "$f" .ready); mkdir -p "$RESULTS_DIR/$VERSION"
      scp -q -F "$SSHC" "pod:$R/$K/$K.json" "$RESULTS_DIR/$VERSION/${RUN}_$K.json" 2> /dev/null &&
        python3 "$HERE/campaign_results.py" "$RESULTS_DIR" "$VERSION" >> "$LOCAL/eval_pod.log" 2>&1
    fi
    echo "$code" > "$LOCAL/$(basename "$f").sent"; log "lap $(basename "$f" .ready) sent to Discord ($code)"
  done
}

# worlds this host takes (sync_bench 10/28, 5090: N=7 used 29.2 GB VRAM, 23.5 cores, ~38 GB RAM): VRAM 3.13 N + 7.3 GB,
# CPU 3.1 N + 2.5 cores of 90 % of the quota, RAM 4.8 N + 4.5 GB of 85 %; the seeds run in waves of N
NMAX=$(P "python3 -c 'import subprocess as s; q=open(\"/sys/fs/cgroup/cpu.max\").read().split(); c=int(q[0])/int(q[1]) if q[0] != \"max\" else len(open(\"/proc/cpuinfo\").read().split(\"processor\"))-1; m=open(\"/sys/fs/cgroup/memory.max\").read().strip(); r=(int(m) if m != \"max\" else int(open(\"/proc/meminfo\").read().split()[1])*1024)/2**30; v=int(s.check_output([\"nvidia-smi\",\"--query-gpu=memory.total\",\"--format=csv,noheader,nounits\"]).split()[0])/1024; print(max(1, min(int((v-0.8-7.3)/3.13), int((0.9*c-2.5)/3.1), int((0.85*r-4.5)/4.8))), round(c,1), round(r), round(v,1))'")
read -r N CPUQ RAMG VRAMG <<< "$NMAX"; [[ ${SYNC:-0} == 1 ]] || N=${#SEEDS[@]}
log "host: CPU quota $CPUQ cores, RAM $RAMG GiB, VRAM $VRAMG GiB -> $N worlds at once; seeds ${SEEDS[*]}"
P "mkdir -p /workspace/results/$RUN && echo '$T0 $(date -d "@${T0%.*}" +%T 2> /dev/null || date -r "${T0%.*}" +%T) pod create ($POD)' > /workspace/results/$RUN/timeline.txt"
redraw=${REDRAW:-1}
for ((w = 0; w < ${#SEEDS[@]}; w += N)); do
  wave=("${SEEDS[@]:w:N}"); WN=$RUN; (( w > 0 || ${#SEEDS[@]} > N )) && WN=${RUN}_wave$((w / N + 1))
  P "mkdir -p /workspace/results/$WN && cp -n /workspace/results/$RUN/timeline.txt /workspace/results/$WN/ && \
     (DRIVER=$DRIVER SIM_FPS=${SIM_FPS:-60} APPLY_DELAY_S=${APPLY_DELAY_S:-0.1} EVENT_T0=${EVENT_T0:-} LAPS=${LAPS:-1} LAP_DELIVER=${LAP_DELIVER:-0} VIEW_FPS=${VIEW_FPS:-10} SYNC=${SYNC:-0} TICK_MS=${TICK_MS:-100} SERIAL=${SERIAL:-0} \
      KEEP_UP=1 CPU_LAYOUT=${CPU_LAYOUT:-} GATHER=${GATHER:-0.1} PREP=${PREP:-process} setsid nohup bash \
      /workspace/MORAI-Autonomous-Driving-System/src/e2e_driver/tools/pod/eval3.sh $WN $DELIVER ${wave[*]} > /workspace/results/$WN.log 2>&1 < /dev/null &)"
  sleep 10   # setsid may not have exec'd eval3 yet when the next ssh looks for it
  log "eval3 $WN (seeds ${wave[*]}) running: ssh -F $SSHC pod tail -f /workspace/results/$WN.log"
  until P "test -e /workspace/results/$WN/DONE || ! pgrep -f '[e]val3.sh $WN' > /dev/null"; do
    late $((${EVAL_MAX_MIN:-110} * 60)) "eval3 (EVAL_MAX_MIN)"; P "touch /workspace/keepalive"; [[ ${LAP_DELIVER:-0} == 1 ]] && deliver "$WN"; sleep 20
  done
  [[ ${LAP_DELIVER:-0} == 1 ]] && deliver "$WN"
  mkdir -p "$LOCAL/$WN"; rsync -rlt -e "ssh -F $SSHC" --exclude '*.mp4.raw' pod:/workspace/results/$WN/ pod:/workspace/results/$WN.log "$LOCAL/$WN/"
  if P "test -e /workspace/results/$WN/ENV_INVALID"; then
    log "ENV_INVALID: $(P "cat /workspace/results/$WN/ENV_INVALID")"; podlog "env-invalid $POD ($NAME): $(P "cat /workspace/results/$WN/ENV_INVALID")"
    [[ -z ${ATTACH:-} && ${KEEP:-0} != 1 ]] && (( ${CPU_RETRY:-0} < ${CPU_RETRIES:-6} )) && { bye; CPU_RETRY=$(( ${CPU_RETRY:-0} + 1 )) exec bash "$0" "${ORIG_ARGS[@]}"; }
    break
  fi
  P "test -e /workspace/results/$WN/DONE" && log "done: $LOCAL/$WN" || { log "eval3 stopped early: $LOCAL/$WN/$WN.log"; break; }
  cat "$LOCAL/$WN/summary.txt" 2> /dev/null
  # scenes that did not play as generated (GT replay check failed): drawn anew once, as one more wave
  bad=$(python3 -c 'import json,sys; print(sum(d.get("replay_ok") is False for d in json.load(open(sys.argv[1]))["drives"]))' "$LOCAL/$WN/summary.json" 2> /dev/null || echo 0)
  if (( redraw && bad > 0 )); then
    for ((i = 0; i < bad; i++)); do SEEDS+=($(draw "${SEEDS[@]}")); done
    log "$bad scene(s) failed the replay check: seeds ${SEEDS[*]: -bad} drawn anew"; redraw=0
  fi
done
