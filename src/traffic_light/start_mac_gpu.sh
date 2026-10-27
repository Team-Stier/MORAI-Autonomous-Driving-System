#!/usr/bin/env bash
set -euo pipefail

PACKAGE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_PATH=/tmp/morai-mac-gpu-inference.log
LOCK_PATH=/tmp/morai-mac-gpu-inference.start.lock
HEALTH_URL=http://127.0.0.1:18765/health

healthy() {
  local response
  response="$(curl -fsS --max-time 1 "$HEALTH_URL" 2>/dev/null)" || return 1
  [[ "$response" == *'"backend": "CoreMLExecutionProvider"'* &&
     "$response" == *'"compute_units": "CPUAndGPU"'* ]]
}

if healthy; then
  printf 'Mac GPU server is ready: %s\n' "$HEALTH_URL"
  exit 0
fi
if [[ "$(uname -s)" != Darwin || "$(uname -m)" != arm64 ]]; then
  printf 'Run this script on the Apple Silicon Mac host.\n' >&2
  exit 1
fi

if mkdir "$LOCK_PATH" 2>/dev/null; then
  trap 'rmdir "$LOCK_PATH"' EXIT
  if ! lsof -nP -iTCP:18765 -sTCP:LISTEN -t >/dev/null 2>&1; then
    UV_BIN="$(command -v uv || true)"
    UV_BIN="${UV_BIN:-$HOME/.local/bin/uv}"
    if [[ ! -x "$UV_BIN" ]]; then
      printf 'uv is required to start the Mac GPU server.\n' >&2
      exit 1
    fi
    /usr/bin/python3 -c '
import subprocess
import sys
with open(sys.argv[3], "ab") as log:
    subprocess.Popen([sys.argv[1], "run", "--python", "3.12", "--with",
                      "onnxruntime==1.31.0", "--with", "numpy==2.5.3", "python",
                      "-u", sys.argv[2]], stdin=subprocess.DEVNULL,
                     stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
' "$UV_BIN" "$PACKAGE_DIR/scripts/mac_gpu_inference.py" "$LOG_PATH"
  fi
fi

deadline=$((SECONDS + 30))
while (( SECONDS < deadline )); do
  if healthy; then
    printf 'Mac GPU server is ready: %s\n' "$HEALTH_URL"
    exit 0
  fi
  (( SECONDS + 1 < deadline )) || break
  sleep 1
done
printf 'Mac GPU server is not ready. Inspect %s; an existing process was preserved.\n' \
  "$LOG_PATH" >&2
exit 1
