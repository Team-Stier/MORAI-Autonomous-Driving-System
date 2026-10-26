#!/usr/bin/env bash
# MORAI's official Linux programs for eval_manager.py, from the morai-docs repository (gh, latest main), checked
# against the SHA-256 that repository records (assets/catalog.json) and copied to the simulator host:
#   checkpoint Spawn program (practice, 10/13)  assets/competition/20261013T155954549530Z/spawn/MORAI_Spawn_Linux
#   Time / Weather Changer (9/28)               assets/competition/tools/WeatherTimeChanger_Linux
set -eu
REPO=Team-Stier/morai-stier-codex-plugin; BC=${BC:-bc-250}
OUT=$(mktemp -d)
while read -r sha path; do
  gh api -H "Accept: application/vnd.github.raw" "repos/$REPO/contents/$path" > "$OUT/${path##*/}"
  echo "$sha  $OUT/${path##*/}" | shasum -a 256 -c -
done <<'E'
5ecc827f5fcbd1cbe00aabb5b01aa01e6d1f87293345d61c20ba5d139210a2f6 assets/competition/20261013T155954549530Z/spawn/MORAI_Spawn_Linux
8c47f48a8d54b79b657920a661f582bd587c9f9065b6b55c9e911d42a0369609 assets/competition/tools/WeatherTimeChanger_Linux
E
ssh "$BC" 'mkdir -p ~/morai-eval-official'
scp -q "$OUT/MORAI_Spawn_Linux" "$OUT/WeatherTimeChanger_Linux" "$BC:morai-eval-official/"
ssh "$BC" 'chmod +x ~/morai-eval-official/MORAI_Spawn_Linux ~/morai-eval-official/WeatherTimeChanger_Linux'
echo "installed on $BC:~/morai-eval-official (downloads left in $OUT)"
