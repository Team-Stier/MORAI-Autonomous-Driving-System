#!/usr/bin/env bash
# offline parity: replay one episode (gt, deploy) through the node code, compare with the training loader
set -u
EP=${1:-run1024b-w1-001}; BAGDIR=${2:-/mnt/2026_MORAI_collect_20261018/mac-win-1}
W=$(cd "$(dirname "$0")/../.." && pwd); O=~/morai-eval/data/parity/$EP
source ~/morai-student/env.sh > /dev/null
ROWS=$(python - <<PY
import pandas as pd
d = pd.read_parquet("$D/_index/index.parquet", columns=["episode", "i"])
print(",".join(map(str, d[d.episode == "$EP"].i.to_numpy()[::15])))
PY
)
BAGS=$(ls /mnt/d/share/${BAGDIR#/mnt/}/${EP}*.bag | sed 's|^/mnt/d/share|/mnt|' | tr '\n' ' ')
for m in gt deploy; do
  $W/tools/winpc/chroot.sh replay "python3 src/e2e_driver/test/offline_replay.py --mode $m --ego /mnt/2026_MORAI_step0/_ego/$EP.npz --rows $ROWS --out /data/parity/$EP/$m $BAGS" 2>&1 | tail -1
done
cd $W/test && python offline_compare.py --root $D --episode $EP --config $R --out $O/report.json $O/gt $O/deploy 2>&1 | grep -v Warn > $O/compare.log
echo done
