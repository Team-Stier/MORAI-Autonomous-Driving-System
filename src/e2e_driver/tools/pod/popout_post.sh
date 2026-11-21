#!/usr/bin/env bash
# popout_post.sh RUN | final NAME: one recorded batch (rl_episode run RUN) through step0 and the synthetic labels while the
# next batch records (nice), then B2 at once (user 10/29: labels on the box, each batch uploaded as it is done):
#   popout_step0.py stage -> ego_tables.py -> step0_worker.py (cams, LiDAR) -> build_index (this batch) ->
#   train/scripts/popout_relabel.py (stop / restart_hold virtual episodes, keep list) -> b2 sync step0_popout/
# final NAME: all of the box's batches merged into the _dyn_relabel layout, dyn_stage.py + dyn_load_check.py (unchanged
# loader), results to B2. Batches run one at a time (flock).
set -uo pipefail
V=${V:-/workspace}; AC=$V/Alpamayo-Cosmos; S0=$V/step0_popout; PY=$V/venv-rl/bin/python; J=${POST_JOBS:-4}
b2env() { eval "$(tr '\0' '\n' < /proc/1/environ | grep -E '^B2_APPLICATION_KEY(_ID)?=' | sed 's/^/export /')"; }
up() { (b2env; nice $V/venv-tools/bin/b2 sync --threads 16 --no-progress --exclude-regex '(.*/\._.*|\._.*|.*\.tmp.*|.*_ext/.*|_logs/.*|.*_(dyn|rh)[0-9]{5}(/.*|\.npz)|_dyn_relabel/.*)' "$@") ; }
exec 9> /tmp/popout_post.lock; flock 9
mkdir -p $S0/_logs
if [[ $1 == final ]]; then
  NAME=$2; D=$S0/_dyn_relabel; mkdir -p $D/ego
  for r in $S0/_popout_relabel/$NAME*/; do for f in $r/ego/*.npz; do [[ -e $f ]] && ln -sf "$f" $D/ego/; done; done
  $PY - "$S0" "$NAME" <<'PY'
import glob, sys, pandas as pd
s0, name = sys.argv[1:3]
for f in ("dyn_index", "dyn_future", "dyn_relabel_index", "keep"):
    parts = [pd.read_parquet(p) for p in sorted(glob.glob(f"{s0}/_popout_relabel/{name}*/{f}.parquet"))]
    if parts: pd.concat(parts, ignore_index=True).to_parquet(f"{s0}/_dyn_relabel/{f}.parquet", index=False)
PY
  $PY $AC/train/scripts/dyn_stage.py $S0 || exit 1
  # dyn_load_check.py as is on the stop rows (it keys the future by (src, i) and wants the original history): a root of
  # links with only those rows; restart_hold rows: the unchanged loader reads them, history standing, future = label
  C=/tmp/popout_check; mkdir -p $C/_dyn_relabel; for f in $S0/*; do [[ $(basename $f) == _dyn_relabel ]] || ln -sfn $f $C/; done
  ln -sfn $D/ego $C/_dyn_relabel/ego
  $PY - "$S0" "$C" <<'PY'
import sys, pandas as pd
s0, c = sys.argv[1:3]
vi = pd.read_parquet(f"{s0}/_dyn_relabel/dyn_index.parquet"); vi = vi[vi.kind != "restart_hold"]; vi.to_parquet(f"{c}/_dyn_relabel/dyn_index.parquet", index=False)
f = pd.read_parquet(f"{s0}/_dyn_relabel/dyn_future.parquet").merge(vi[["src_episode", "i"]].rename(columns={"src_episode": "episode"}).drop_duplicates(), on=["episode", "i"])
f.drop_duplicates(["episode", "i"]).to_parquet(f"{c}/_dyn_relabel/dyn_future.parquet", index=False)
PY
  (cd $AC/train && PYTHONPATH=. $V/venv-engine/bin/python scripts/dyn_load_check.py $C 40) || exit 1
  (cd $AC/train && PYTHONPATH=. $V/venv-engine/bin/python - "$S0" <<'PY'
import sys, numpy as np, pandas as pd
from morai_vla.data import MoraiDataset
root = sys.argv[1]; vi = pd.read_parquet(f"{root}/_dyn_relabel/dyn_index.parquet"); vi = vi[vi.kind == "restart_hold"]
pick = vi.sample(min(30, len(vi)), random_state=0); pick[["episode", "i"]].to_parquet("/tmp/rh_keep.parquet", index=False)
ds = MoraiDataset(root, split=["train", "val", "val_asset"], index="_dyn_relabel/dyn_index.parquet", keep="/tmp/rh_keep.parquet", augment=False)
assert len(ds), "no usable restart_hold row"
for k in range(len(ds)):
    s = ds.sample(k); h = s["ego_history_xyz"][0].numpy(); f = s["ego_future_xyz"][0].numpy()
    assert np.abs(h).max() < 1e-3, "history not standing"
    assert len(s["camera_frames"]) == 4
print(f"restart_hold OK: {len(ds)} of {len(pick)} rows usable, standing history, future moves {float(np.linalg.norm(f[-1])):.1f} m")
PY
  ) && echo LOAD_CHECK_OK
  exit
fi
RUN=$1
nice $PY -u $V/MORAI-Autonomous-Driving-System/src/e2e_driver/tools/pod/popout_step0.py stage $V/spool "$RUN" $S0 || exit 1
nice $PY -u $AC/tools/step0/ego_tables.py $S0/_ego --only $S0/_popout/ok_$RUN.txt --jobs $J || exit 1
for k in $(seq 0 $((J - 1))); do
  (cd $AC/tools/step0 && STEP0_ROOT=$V/bags STEP0_OUT=$S0 nice bash -c "source /opt/ros/noetic/setup.bash; exec python3 step0_worker.py $k $J") &
done; wait
eps=$(cat $S0/_popout/ok_$RUN.txt); E=/tmp/ego_$RUN; mkdir -p $E
for e in $eps; do [[ -e $S0/$e/meta.json ]] && ln -sf $S0/_ego/$e.npz $E/; done
(cd $AC/train && nice $PY -m morai_vla.build_index $S0/_popout_index/$RUN --ego $E --held-out 40100027,40100029,50100001 --glitch net > /dev/null) || exit 1
nice $PY -u $AC/train/scripts/popout_relabel.py $S0 $S0/_popout_index/$RUN $S0/_popout_relabel/$RUN $(ls $E | sed 's/\.npz$//') || exit 1
# main 10/29 "B2 깨끗하게": an episode whose segment mover never spawned or never moved is not uploaded; it goes to
# step0_popout_quarantine/ (episode folder, _ego, _popout) and leaves this box's step0 tree
Q=$V/quarantine/step0_popout; mkdir -p $Q/_ego $Q/_popout
$PY - "$S0" "$RUN" > /tmp/bad_$RUN.txt <<'PY'
import json, sys
s0, run = sys.argv[1:3]
zone = "ra" if run[-4:-2] == "ra" else "rb"
summ = json.load(open(f"{s0}/_popout_relabel/{run}/summary.json"))["episodes"]
for ep in open(f"{s0}/_popout/ok_{run}.txt").read().split():
    rec = json.load(open(f"{s0}/_popout/{ep}/episode.json"))
    mv = [(a, f) for a, f in zip([a for a in rec["scene_meta"]["actors"] if a.get("ptype") == "popout"], rec.get("mover_in_objinfo", []))]
    st = dict(zip([a["zone"] for a, _ in mv], summ.get(ep, {}).get("tau_start", [])))
    if not all(f for a, f in mv if a["zone"].startswith(zone)) or st.get(f"{zone}_dynamic") is None or rec.get("t_sim", 99) < 2.0:
        print(ep)
PY
for e in $(cat /tmp/bad_$RUN.txt); do
  mv $S0/$e $Q/ 2> /dev/null; mv $S0/_ego/$e.npz $Q/_ego/ 2> /dev/null; mv $S0/_popout/$e $Q/_popout/ 2> /dev/null
done
[[ -s /tmp/bad_$RUN.txt ]] && { echo "quarantined: $(tr '\n' ' ' < /tmp/bad_$RUN.txt)"; up $V/quarantine/step0_popout/ b2://morai2026-alpamayo/step0_popout_quarantine/; }
up $S0/ b2://morai2026-alpamayo/step0_popout/ && echo "B2 $RUN ok"
