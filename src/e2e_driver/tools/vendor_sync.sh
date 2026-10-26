#!/usr/bin/env bash
# Refresh the pinned copy of the training package the model server imports (server/vendor/morai_vla) from a training
# tree, and its manifest. Run when the delivered checkpoint's training code changes; then test/test_vendor.py.
# Usage: tools/vendor_sync.sh <train dir containing morai_vla/>
set -euo pipefail
SRC=$1/morai_vla; DST=$(cd "$(dirname "$0")/../server/vendor" && pwd)
[[ -f $SRC/model.py ]] || { echo "no morai_vla in $1" >&2; exit 1; }
mkdir -p "$DST/morai_vla"
find "$DST/morai_vla" -maxdepth 1 -name '*.py' -exec rm -- {} +
cp "$SRC"/*.py "$DST/morai_vla/"
(cd "$DST" && { echo "# morai_vla pinned from $1 on $(date +%F) (sha256)"; shasum -a 256 morai_vla/*.py 2>/dev/null || sha256sum morai_vla/*.py; } > MANIFEST.sha256)
cat "$DST/MANIFEST.sha256"
