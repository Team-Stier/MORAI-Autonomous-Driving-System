#!/usr/bin/env bash
# The model server on the GPU, outside the chroot. Usage: server.sh up <model_server.py args...> | down
#   server.sh up --backend hf --ckpt /path/to/student/final          (the only thing to change for the real student)
#   server.sh up --backend fake --preprocess-config "$R"              (no model; full preprocessing; R = a config.json dir)
#   server.sh up                       the model settings of config/runtime.env (E2E_MODEL_BACKEND/CKPT/VLM/ARGS), as run.sh
#                                      passes them; the engine backend runs in $ENGINE_VENV
set -uo pipefail
cd "$(dirname "$0")"; source ./env.sh
mkdir -p "$EVAL/run"
case ${1:-} in
  up)
    shift
    [[ -e $EVAL/run/server.pid ]] && kill -0 "$(cat "$EVAL/run/server.pid")" 2>/dev/null && { echo "server already up"; exit 2; }
    if (( $# == 0 )); then                 # runtime.env, parsed as run.sh does (the environment overrides the file)
      while IFS='=' read -r key value; do
        [[ $key =~ ^[A-Z_][A-Z0-9_]*$ ]] || continue
        value="${value%%#*}"; value="${value%"${value##*[![:space:]]}"}"
        [[ -n "${!key+x}" || -z $value ]] || export "$key=$value"
      done < "$WS/src/e2e_driver/config/runtime.env"
      read -r -a extra <<< "${E2E_MODEL_ARGS:-}"
      set -- --backend "${E2E_MODEL_BACKEND:-hf}" --vlm "${E2E_MODEL_VLM:-Qwen/Qwen3-VL-4B-Instruct}" \
        ${E2E_MODEL_CKPT:+--ckpt "$E2E_MODEL_CKPT"} "${extra[@]}"
    fi
    [[ " $* " == *" --backend engine "* ]] && VENV=$ENGINE_VENV
    # 4 torch threads and heap-only malloc (no fresh mmap + page faults for the 35 MB pixel_values copies): server round
    # trip 75 -> 58 ms (p95 118 -> 71) next to a running simulator; numerics unchanged (elementwise ops, parity re-checked)
    OMP_NUM_THREADS=${OMP_NUM_THREADS:-4} MALLOC_MMAP_MAX_=${MALLOC_MMAP_MAX_:-0} MALLOC_TRIM_THRESHOLD_=${MALLOC_TRIM_THRESHOLD_:-1073741824} \
    MORAI_VLA_PYLIB=$HOME/morai-student/pylib HF_HUB_OFFLINE=1 setsid "$VENV/bin/python" -u "$WS/src/e2e_driver/server/model_server.py" "$@" \
      > "$EVAL/run/server.log" 2>&1 < /dev/null &
    echo $! > "$EVAL/run/server.pid"; echo "server pid $(cat "$EVAL/run/server.pid"), log $EVAL/run/server.log" ;;
  down)
    [[ -e $EVAL/run/server.pid ]] && { kill "$(cat "$EVAL/run/server.pid")" 2>/dev/null; mv "$EVAL/run/server.pid" "$EVAL/run/server.pid.old"; }
    echo "server down" ;;
  *) echo "usage: server.sh up <args> | down" >&2; exit 1 ;;
esac
