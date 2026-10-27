#!/usr/bin/env bash
# deliver_watch.sh (Mac, nohup): waits for a new delivery and puts it on the lap-video evaluation at once. The R2 fine-tune's
# watcher (Alpamayo-Cosmos tools/vast/r2ft_autowatch.py) writes "STAGE 납품 완료: b2://morai2026-alpamayo/<run>/deliver/final"
# to EVENTS (~/.r2ft-watch.events); only lines after this script starts count. Then: the cheapest vast 5090 offers
# (README quick start filter, up to MAX_DPH $/h, OFFERS_N tried in order), eval_pod.sh BUNDLE=v1 on b2:<run>/deliver (the
# qad_export deliver dir: final/ + quant_manifest.json, as the delivered WoWmayo), LAPS (8) on SEED (28: the scenes of the
# 10/28 rt8 run), each lap to the user's Discord DM starting with LABEL, B2 sync and self-deletion at the end.
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
cmd=(env MSG_PREFIX="${LABEL:-[R2 미세조정 납품본] }" PROVIDER=vast OFFER="$offers" BUNDLE=v1 LAPS=${LAPS:-8} LAP_DELIVER=1 VIEW_FPS=15
     EVAL_MAX_MIN=${EVAL_MAX_MIN:-330} bash "$HERE/eval_pod.sh" "$NAME" "b2:$src" "${SEED:-28}")
echo "$(date +%T) offers [$offers] -> ${cmd[*]}"
[[ -n $offers ]] || { echo "no 5090 offer under \$${MAX_DPH:-1.0}/h"; exit 1; }
[[ ${DRY_RUN:-0} == 1 ]] && exit 0
"${cmd[@]}" > "$HOME/morai-eval/pod/$NAME.out" 2>&1
echo "$(date +%T) eval_pod.sh ended ($?): $(tail -2 "$HOME/morai-eval/pod/$NAME/eval_pod.log" | tr '\n' ' ')"
