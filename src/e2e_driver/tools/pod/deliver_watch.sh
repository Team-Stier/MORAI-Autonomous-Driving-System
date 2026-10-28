#!/usr/bin/env bash
# deliver_watch.sh (Mac, nohup): waits for a new delivery and puts it on the lap-video evaluation at once. The R2 fine-tune's
# watcher (Alpamayo-Cosmos tools/vast/r2ft_autowatch.py) writes "STAGE 납품 완료: b2://morai2026-alpamayo/<run>/deliver/final"
# to EVENTS (~/.r2ft-watch.events); only lines after this script starts count. Then: the cheapest vast 5090 offers
# (README quick start filter, up to MAX_DPH $/h, OFFERS_N tried in order), eval_pod.sh BUNDLE=v1 on b2:<run>/deliver (the
# qad_export deliver dir: final/ + quant_manifest.json, as the delivered WoWmayo), LAPS (8) on SEED (default: a new random one per
# run; set it only to replay the scenes of an earlier run), each lap to the user's Discord DM starting with LABEL, B2 sync and self-deletion at the end.
#   DRY_RUN=1 prints the command instead of renting.        nohup bash deliver_watch.sh > ~/morai-eval/deliver_watch.log 2>&1 &
HERE=$(cd "$(dirname "$0")" && pwd); EVENTS=${EVENTS:-$HOME/.r2ft-watch.events}
n0=$(wc -l < "$EVENTS" 2> /dev/null || echo 0)
echo "$(date +%T) watching $EVENTS from line $((n0 + 1)) for a delivery"
until line=$(tail -n +$((n0 + 1)) "$EVENTS" 2> /dev/null | grep -m1 "STAGE 납품 완료: b2://"); do sleep 30; done
src=$(echo "$line" | grep -o "b2://[^ ]*" | sed 's#^b2://morai2026-alpamayo/##; s#/final/*$##')
echo "$(date +%T) delivery seen: $line -> b2:$src"
offers=$(vastai search offers "gpu_name=RTX_5090 num_gpus=1 rentable=true verified=true cpu_ghz>=4.5 inet_down>=900 reliability>0.98 cpu_ram>=30 cpu_cores_effective>=16 dph_total<=${MAX_DPH:-1.0}" -o 'dph+' --raw |
  python3 -c "import json,sys; print(' '.join(str(o['id']) for o in json.load(sys.stdin)[:${OFFERS_N:-4}]))")
NAME=$(date +%Y%m%d-%H%M)_${RUNTAG:-r2ft}_rt${LAPS:-8}
EVENT_T0=$(date -j -f "%Y-%m-%d %H:%M:%S" "$(echo "$line" | cut -c1-19)" +%s 2> /dev/null || date +%s)
# a box booted beforehand for this (PRE = ~/morai-eval/pod/prelaunch: "NAME instance create_epoch"; worlds, stacks and the
# engine's compile cache for the new manifest are up): only the weights are fetched there
read -r pname pid pt0 < "${PRE:-$HOME/morai-eval/pod/prelaunch}" 2> /dev/null
if [[ -n ${pid:-} ]] && [[ $(vastai show instance "$pid" --raw 2> /dev/null | python3 -c 'import json,sys; print(json.load(sys.stdin).get("actual_status"))' 2> /dev/null) == running ]]; then
  NAME=$pname; att=(ATTACH=$pid ATTACH_T0=$pt0); echo "$(date +%T) using the prelaunched box $pid ($pname)"
  for i in $(seq 120); do   # it may still be booting: its setup must be done before the eval is attached
    ssh -F "$HOME/morai-eval/pod/$pname/ssh_config" -o ConnectTimeout=15 pod 'ls /workspace/setup.done.* > /dev/null 2>&1' 2> /dev/null && break; sleep 10
  done
else att=(); fi
cmd=(env "${att[@]}" EVENT_T0=$EVENT_T0 MSG_PREFIX="${LABEL:-[R2 미세조정 납품본] }" PROVIDER=vast OFFER="$offers" BUNDLE=v1 LAPS=${LAPS:-8} LAP_DELIVER=1 VIEW_FPS=15
     EVAL_MAX_MIN=${EVAL_MAX_MIN:-330} bash "$HERE/eval_pod.sh" "$NAME" "b2:$src" ${SEED:+"$SEED"})   # no SEED: eval_pod.sh draws a new one
echo "$(date +%T) offers [$offers] -> ${cmd[*]}"
[[ -n $offers ]] || { echo "no 5090 offer under \$${MAX_DPH:-1.0}/h"; exit 1; }
[[ ${DRY_RUN:-0} == 1 ]] && exit 0
"${cmd[@]}" > "$HOME/morai-eval/pod/$NAME.out" 2>&1
echo "$(date +%T) eval_pod.sh ended ($?): $(tail -2 "$HOME/morai-eval/pod/$NAME/eval_pod.log" | tr '\n' ' ')"
