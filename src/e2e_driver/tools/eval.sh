#!/usr/bin/env bash
# Closed-loop evaluation manager (eval_manager.py; two_pc.sh up and prep first). Dependencies through uv: grpcio,
# protobuf and Alpamayo-Cosmos tools/scorer's requirements (SCORER, default: the sibling checkout).
HERE=$(cd "$(dirname "$0")" && pwd)
SCORER=${SCORER:-$(cd "$HERE/../../../.." && pwd)/Alpamayo-Cosmos/tools/scorer}
exec uv run -q --python 3.12 --with grpcio --with protobuf --with-requirements "$SCORER/requirements.txt" \
  python "$HERE/eval_manager.py" --scorer "$SCORER" "$@"
