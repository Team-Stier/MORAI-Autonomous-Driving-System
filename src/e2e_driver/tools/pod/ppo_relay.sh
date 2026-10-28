#!/usr/bin/env bash
# ppo_relay.sh RUN [once]: the PPO actor box <-> learner relay through B2 (b2://morai2026-alpamayo/runs/ppo/RUN/), one
# pass every EVERY (30) s until killed; once: a single pass, exit 0 = every episode finished by then is on B2 (README
# "PPO 액터"). ppo_box.sh runs it.
#   LoRA down: lora/latest.json to a hidden .part, the version file it names (when not here yet), then latest.json renamed
#     into $V/out/lora/ (engine --lora-watch, rl_episode --lora-latest): a latest.json here always has its file here.
#   spool up, episode-atomic: the done.json files are listed at the pass start; per listed episode not up yet: its dir
#     without done.json (b2 sync, all files final: done.json is written last), and only after that sync's exit 0 the
#     done.json alone. B2 never holds a done.json whose episode is incomplete. Then each actor's inflight.json and
#     episodes.jsonl when their mtime moved (b2 keeps the file mtime: the learner's actor_timeout reads the actor's write
#     time). An uploaded episode's *.bin go from this disk (~6 MB/s of dumps per world would fill it in hours).
set -uo pipefail
V=${V:-/workspace}; RUN=${1:?RUN}; SP=${SPOOL:-$V/spool}; L=${LORA_DIR:-$V/out/lora}; S=${RELAY_STATE:-/root/ppo_relay}
BK=morai2026-alpamayo; B=b2://$BK/runs/ppo/$RUN; B2=${B2CLI:-$V/venv-tools/bin/b2}
[[ -z ${B2_APPLICATION_KEY:-} && -r /proc/1/environ ]] &&
  eval "$(tr '\0' '\n' < /proc/1/environ | grep -E '^B2_APPLICATION_KEY(_ID)?=' | sed 's/^/export /')"
mkdir -p "$L" "$S" "$SP"; touch "$S/up.list"
log() { echo "$(date '+%F %T') $*"; }
up() { "$B2" file upload --no-progress "$BK" "$1" "runs/ppo/$RUN/spool/${1#"$SP"/}" > /dev/null; }   # one file, same path

lora() {
  "$B2" file download --no-progress "$B/lora/latest.json" "$L/.latest.json.part" > /dev/null 2>&1 || return 0   # none yet
  cmp -s "$L/.latest.json.part" "$L/latest.json" && return 0
  local f; f=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["path"])' "$L/.latest.json.part") || return 0
  [[ $f =~ ^v[0-9]+\.safetensors$ ]] || { log "lora: bad path '$f' in latest.json"; return 0; }
  [[ -s $L/$f ]] || { "$B2" file download --no-progress "$B/lora/$f" "$L/.$f.part" > /dev/null 2>&1 && mv "$L/.$f.part" "$L/$f"; } ||
    { log "lora: $f not fetched, latest.json held back"; return 0; }
  mv "$L/.latest.json.part" "$L/latest.json" && log "lora: $f"
}

spool() {
  local rc=0 e f st
  for e in $(cd "$SP" && find . -mindepth 3 -maxdepth 3 -name done.json | sed 's|^\./||; s|/done\.json$||' | sort); do
    grep -qxF "$e" "$S/up.list" && continue
    if "$B2" sync --threads 16 --no-progress --exclude-regex 'done\.json$' "$SP/$e/" "$B/spool/$e/" > "$S/sync.log" 2>&1 &&
       up "$SP/$e/done.json"; then
      echo "$e" >> "$S/up.list"; find "$SP/$e" -name '*.bin' -delete; log "spool: $e"
    else
      rc=1; log "spool: $e not up: $(tail -1 "$S/sync.log")"
    fi
  done
  for f in "$SP"/*/inflight.json "$SP"/*/episodes.jsonl; do
    [[ -e $f ]] || continue
    st=$S/stamp/${f#"$SP"/}; mkdir -p "$(dirname "$st")"
    [[ -e $st && ! $f -nt $st ]] && continue
    touch -r "$f" "$st.new" && up "$f" && mv "$st.new" "$st" || { rc=1; log "spool: ${f#"$SP"/} not up"; }   # stamp first: a newer write goes next pass
  done
  return $rc
}

while :; do
  lora; spool; rc=$?
  [[ ${2:-} == once ]] && exit $rc
  sleep "${EVERY:-30}"
done
