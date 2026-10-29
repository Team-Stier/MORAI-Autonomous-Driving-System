#!/usr/bin/env bash
# drive_prep.sh REF (host): the drive checkout for a pod. CLONE (/home/opc/morai-campaign/drive-src, a clone of
# Team-Stier/MORAI-Autonomous-Driving-System with git-lfs) fetches origin and every local branch of the user's
# checkout SRC (/home/opc/MORAI-Autonomous-Driving-System: the worker and merge worktrees' branches) as local/<branch>,
# checks REF out detached (a sha, origin/<b>, local/<b>, or a bare branch name tried as local/ then origin/), updates the
# submodules to the recorded pins and pulls the LFS models, then copies the tree (no .git) to the immutable snapshot
# SNAP/<sha> (/home/opc/morai-campaign/drive-snap) with its SHA file (git rev-parse HEAD, read under the lock). Prints
# "<sha> <ref> <snapshot dir>"; eval_pod.sh rsyncs the snapshot, so concurrent runs never see each other's checkout.
# Only one prep at a time (flock); a snapshot already made is reused.
set -euo pipefail
REF=${1:?REF}; CLONE=${CLONE:-/home/opc/morai-campaign/drive-src}; SRC=${SRC:-/home/opc/MORAI-Autonomous-Driving-System}
SNAP=${SNAP:-/home/opc/morai-campaign/drive-snap}
exec 9> "$CLONE.lock"; flock 9
cd "$CLONE"
git fetch -q origin '+refs/heads/*:refs/remotes/origin/*' 2> /dev/null || true
git fetch -q "$SRC" '+refs/heads/*:refs/remotes/local/*'
sha=
for r in "local/$REF" "origin/$REF" "$REF"; do sha=$(git rev-parse -q --verify "$r^{commit}" 2> /dev/null) && break; done
[[ -n $sha ]] || { echo "unknown ref $REF" >&2; exit 1; }
if [[ ! -s $SNAP/$sha/SHA ]]; then
  git checkout -q --detach -f "$sha" && git clean -qfdx -e build -e devel
  git submodule update -q --init --recursive --force
  git lfs pull > /dev/null
  head=$(git rev-parse HEAD); [[ $head == "$sha" ]] || { echo "checkout is $head, not $sha" >&2; exit 1; }
  mkdir -p "$SNAP" && rm -rf "$SNAP/$sha.part" && rsync -a --exclude .git --exclude /build --exclude /devel ./ "$SNAP/$sha.part/" &&
    echo "$head" > "$SNAP/$sha.part/SHA" && mv "$SNAP/$sha.part" "$SNAP/$sha"
fi
# disk (the host is shared and nearly full): snapshots unused for SNAP_IDLE_MIN (20) go, the newest SNAP_KEEP (4) always
# stay (an `if`, not `&&`: under set -e a false test as the loop's last status failed the whole prep, 10/29)
touch "$SNAP/$sha"
ls -1dt "$SNAP"/*/ 2> /dev/null | tail -n +$(( ${SNAP_KEEP:-4} + 1 )) | while read -r old; do
  if [[ -n $(find "$old" -maxdepth 0 -mmin +"${SNAP_IDLE_MIN:-20}") && $old != "$SNAP/$sha/" ]]; then rm -rf "$old"; fi
done || true
echo "$sha $REF $SNAP/$sha"
