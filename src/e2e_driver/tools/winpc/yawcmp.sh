#!/usr/bin/env bash
# LiDAR yaw-rate comparison on one episode: replay deploy mode with each r source, compare with the training loader
set -u
EP=${1:-run1024b-w1-001}
W=$(cd "$(dirname "$0")/../.." && pwd); O=~/morai-eval/data/yawcmp/$EP
source ~/morai-student/env.sh > /dev/null
ROWS=$(python3 -c "import json;print(','.join(str(r['i']) for r in json.load(open('$HOME/morai-eval/data/parity/$EP/gt/meta.json'))['rows']))")
BAGS=$(ls /mnt/d/share/2026_MORAI_collect_20261018/*/${EP}*.bag | sed 's|^/mnt/d/share|/mnt|' | tr '\n' ' ')
for r in grid trend0.3 trend0.6 imu; do
  $W/tools/winpc/chroot.sh replay "python3 src/e2e_driver/test/offline_replay.py --mode deploy --r-source $r --ego /mnt/2026_MORAI_step0/_ego/$EP.npz --rows $ROWS --out /data/yawcmp/$EP/$r $BAGS" 2>&1 | tail -1
done
cd $W/test && python offline_compare.py --root $D --episode $EP --config $R --out $O/report.json $O/grid $O/trend0.3 $O/trend0.6 $O/imu 2>&1 | grep -v Warn > $O/compare.log
python - <<PY
import json, numpy as np
z = dict(np.load("/mnt/d/share/2026_MORAI_step0/_ego/$EP.npz")); r = json.load(open("$O/report.json"))
print("| r source | BEV IoU median | IoU p10 | turn-row IoU median (n) | r error mean / max (rad/s) | r error mean in turns |")
for src in ("grid", "trend0.3", "trend0.6", "imu"):
    m = json.load(open("$O/%s/meta.json" % src))["r_err_radps"]; rows = [x for x in r["deploy_" + src]["per_row"] if "bev_occupied_iou" in x]
    iou = np.array([x["bev_occupied_iou"] for x in rows]); turn = np.array([abs(z["yaw_rate"][x["i"]]) > 0.1 for x in rows])
    print("| %s | %.4f | %.4f | %.4f (%d) | %.4f / %.3f | %.4f |" % (src, np.median(iou), np.percentile(iou, 10), np.median(iou[turn]), turn.sum(), m["mean"], m["max"], m["turn_mean"]))
PY
