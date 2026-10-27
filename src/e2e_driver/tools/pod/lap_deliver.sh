#!/usr/bin/env bash
# lap_deliver.sh OUT n K NAME: lap K (from 0) of world n is recorded (eval3.sh LAP_DELIVER=1 calls it as each lap ends):
# 1x phone video under 24 MB (Discord's attachment limit; lap_video.py make + phone_video.sh), the one-line report
# (lap_report.py), the lap's files to B2 runs/eval_vps/NAME/lap_KK/ and a 7-day download link for the video, then
# OUT/lap_KK.ready ("report<TAB>link<TAB>video path") for eval_pod.sh on the Mac to send.
set -uo pipefail
HERE=$(cd "$(dirname "$0")" && pwd); V=${V:-/workspace}; E2E=$V/MORAI-Autonomous-Driving-System/src/e2e_driver
OUT=$1; n=$2; k=$3; NAME=$4; K=$(printf %02d $((k + 1))); k3=$(printf %03d "$k")
O=$OUT/w$n; FR=/root/frames/$NAME; PY=$V/venv-rl/bin/python; D=$OUT/lap_$K; mkdir -p "$D"
b2env() { eval "$(tr '\0' '\n' < /proc/1/environ | grep -E '^B2_APPLICATION_KEY(_ID)?=' | sed 's/^/export /')"; }
$PY $E2E/tools/lap_video.py make "$FR/w$n" "$O/eval_map/lap$k3.clock.json" "$FR/w${n}_lap$K.mp4" 30 > "$D/video.log" 2>&1 &&
  bash "$HERE/phone_video.sh" "$FR/w${n}_lap$K.mp4" "$D/${NAME}_lap$K.mp4" 21 24000000 >> "$D/video.log" 2>&1
line=$($PY "$HERE/lap_report.py" "$O" "$k" 2> "$D/report.err")
[[ -s $D/${NAME}_lap$K.mp4 ]] || line="$line | 영상 없음"
echo "$line" > "$D/report.txt"
cp "$O"/eval_map/lap$k3.* "$O/lap_$K.json" "$O/eval_map/laps.jsonl" "$D/" 2> /dev/null
url=-
(b2env; $V/venv-tools/bin/b2 sync --threads 8 --no-progress "$D/" "b2://morai2026-alpamayo/runs/eval_vps/$NAME/lap_$K/" > "$D/b2.log" 2>&1) &&
  url=$(b2env; $V/venv-tools/bin/b2 file url --with-auth --duration 604800 "b2://morai2026-alpamayo/runs/eval_vps/$NAME/lap_$K/${NAME}_lap$K.mp4" 2>> "$D/b2.log")
printf '%s\t%s\t%s\n' "$line" "${url:--}" "$D/${NAME}_lap$K.mp4" > "$OUT/lap_$K.ready"
