#!/usr/bin/env bash
# Create the model server venv from server/requirements.lock (Python 3.12; uv if present, else python3.12 -m venv).
# Usage: tools/setup_server_env.sh [VENV_DIR, default: <repo>/.venv-model]   then set E2E_MODEL_PYTHON in config/e2e_runtime.env
set -euo pipefail
HERE=$(cd "$(dirname "$0")/.." && pwd); ROOT=$(cd "$HERE/../.." && pwd)
VENV=${1:-$ROOT/.venv-model}
if command -v uv > /dev/null; then uv venv --python 3.12 "$VENV" && uv pip install --python "$VENV/bin/python" -r "$HERE/server/requirements.lock"
else python3.12 -m venv "$VENV" && "$VENV/bin/pip" install -r "$HERE/server/requirements.lock"; fi
"$VENV/bin/python" -c "import torch, transformers, alpamayo_r1; print('ok', torch.__version__, torch.cuda.is_available())"
