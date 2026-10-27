#!/usr/bin/env bash
# eval_pods.sh NAME DELIVER SEED ...   (Mac) one pod per seed, all at once (eval_pod.sh NAME_sSEED DELIVER SEED): one MORAI
# world next to the engine per GPU keeps the 10 Hz simulator-time steps (3 worlds on one 5090 / RTX PRO 6000 were late in
# 85-99 % of the steps, 10/27). Results: ~/morai-eval/pod/NAME_sSEED each, and NAME/summary.json over all of them.
HERE=$(cd "$(dirname "$0")" && pwd); NAME=$1; DELIVER=$2; shift 2
for s in "$@"; do VCPU=${VCPU:-16} MEM_GB=${MEM_GB:-48} bash "$HERE/eval_pod.sh" "${NAME}_s$s" "$DELIVER" "$s" > "$HOME/morai-eval/pod/${NAME}_s$s.out" 2>&1 & sleep 5; done
wait
D=$HOME/morai-eval/pod/$NAME; mkdir -p "$D"; k=0
for s in "$@"; do k=$((k + 1)); ln -sfn "../${NAME}_s$s/w1" "$D/w$k"; done
head -1 "$HOME/morai-eval/pod/${NAME}_s$1/timeline.txt" > "$D/timeline.txt" 2> /dev/null
uv run -q --no-project --with numpy python "$HERE/eval3_summary.py" "$D" | tee "$D/summary.txt"
