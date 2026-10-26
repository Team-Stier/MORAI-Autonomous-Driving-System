#!/usr/bin/env bash
# RL episode manager (rl_episode.py; the stack up first, e.g. two_pc.sh up + prep, or the pod's stack.sh). Dependencies
# through uv: grpcio, protobuf and Alpamayo-Cosmos tools/scorer's requirements (SCORER, default: the sibling checkout).
HERE=$(cd "$(dirname "$0")" && pwd)
SCORER=${SCORER:-$(cd "$HERE/../../../.." && pwd)/Alpamayo-Cosmos/tools/scorer}
exec uv run -q --python 3.12 --with grpcio --with protobuf --with-requirements "$SCORER/requirements.txt" \
  python "$HERE/rl_episode.py" --scorer "$SCORER" "$@"
