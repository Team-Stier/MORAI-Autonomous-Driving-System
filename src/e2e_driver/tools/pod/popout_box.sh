#!/usr/bin/env bash
# popout_box.sh NAME [EPISODES_PER_ZONE 30] [BATCH 5] [SEED 29]: near pop-out collection on this box after setup.sh
# (dagger_pod.sh BOX=popout_box.sh starts it; README "근거리 돌출 녹화", Alpamayo-Cosmos reports/popout_collect_plan_20261029.md).
# One world, real time, no engine and no stack:
#   1. shadow build (collection fork snapshot) while world 1 comes up
#   2. the collection driver drives world 1 itself (shadow.sh up 1 drive) through rl_episode --scene-preset popout, the
#      dynamic-zone (CP4 -> s 561) and tunnel (CP12 -> s 1984.6) segments in turns of BATCH episodes; popout_rec.sh records
#      one rosbag per episode, rl_episode writes labels.parquet + env.log + objects.json.gz (Object Info 20 Hz)
#   3. step0 (popout_step0.py): the R2 chain on this box (labels -> tools/step0 ego_tables -> step0_worker cams/LiDAR)
#   4. B2 step0_popout/: episode folders, _ego, _popout/<ep>/ (labels, objects, episode record, scene), results/NAME
#   (3-4 per batch in popout_post.sh while the next batch records; final: _dyn_relabel merge + dyn_load_check)
# Out: $V/results/NAME (timeline.txt, logs); DONE at the end. Episodes are NAME{ra,rb}BB-w1-KKK.
set -uo pipefail
HERE=$(cd "$(dirname "$0")" && pwd); V=${V:-/workspace}; WS=$V/MORAI-Autonomous-Driving-System; E2E=$WS/src/e2e_driver
NAME=$1; N=${2:-30}; B=${3:-5}; SEED=${4:-29}; OUT=$V/results/$NAME; mkdir -p "$OUT"
stamp() { echo "$(date +%s.%N | cut -c1-14) $(date +%T) $*" | tee -a "$OUT/timeline.txt"; }
b2env() { eval "$(tr '\0' '\n' < /proc/1/environ | grep -E '^B2_APPLICATION_KEY(_ID)?=' | sed 's/^/export /')"; }
T=$(nproc --all)
[[ ${POP_MIX:-} == tstar ]] && { POP_STARTS=${POP_STARTS-}; ZONES=${ZONES:-ra rb}; RB_SPAWN=${RB_SPAWN:-80 100}; }   # T* from the start
export NO_YIELD=1 POP_STARTS=${POP_STARTS-ra:hidden;rb:hidden,far}   # main 10/29 balance: hidden-heavy, no tunnel near                                      # user 10/29: RDDF only, a collision ends the episode; labels synthesized later
W=$(( T >= 16 ? 10 : T * 3 / 5 ))                      # world CPUs; the driver, bridge and recorder get the rest
export CPUS_1=0-$((W - 1)) SHADOW_CPUS=$W-$((T - 1))
export POP_MIX=${POP_MIX:-}                             # tstar: + 25 % crossings from afar (scenario_variants)
RB_SPAWN=${RB_SPAWN:-30 50}                           # tstar: 80 100 (the tunnel entered at 13-15 m/s)
declare -A RANGE=([ra]="--start-cp 4 --end-cp 5 --past-m 90 --red-cap -1 -1 --spawn-before 30 50" [rb]="--start-cp 12 --end-cp 13 --past-m 125 --spawn-before $RB_SPAWN")
stamp "popout_box $NAME: $N episodes per zone in batches of $B, seed $SEED, $(nvidia-smi --query-gpu=name,memory.total --format=csv,noheader), $T CPUs, quota $(cat /sys/fs/cgroup/cpu.max 2> /dev/null); world $CPUS_1 driver $SHADOW_CPUS"

(touch $V/keepalive; while sleep 60; do touch $V/keepalive; done) &   # idle watchdog: the build and step0 leave the GPU idle
KA=$!
SHADOW_CPUS=0-$((T - 1)) bash "$HERE/shadow.sh" build > "$OUT/shadow_build.log" 2>&1 &
BUILD=$!
bash "$HERE/worlds.sh" up 1 > "$OUT/worlds.log" 2>&1 || { stamp "world failed: $(tail -3 "$OUT/worlds.log")"; touch "$OUT/DONE"; exit 1; }
stamp "world up"
wait $BUILD; [[ -e $V/shadow/BUILT ]] && stamp "shadow built" || { stamp "shadow build failed: $(tail -3 $V/shadow/build.log)"; touch "$OUT/DONE"; exit 1; }
bash "$HERE/shadow.sh" up 1 drive || { stamp "shadow drive up failed"; touch "$OUT/DONE"; exit 1; }
bash "$HERE/popout_rec.sh" up; sleep 20
d=/root/Workspaces/scratches/morai
POSTS=
for b in $(seq 0 $(( (N + B - 1) / B - 1 ))); do
  for z in ${ZONES:-ra rb rb}; do                      # main 10/29: tunnel batches twice the dynamic zone's
    if [[ -e $V/tstar && -z $POP_MIX ]]; then          # touch $V/tstar: the T* collector from the next batch on (main 10/29)
      export POP_MIX=tstar; RANGE[rb]="--start-cp 12 --end-cp 13 --past-m 125 --spawn-before 80 100"; stamp "switched to T* collection"
    fi
    bb=$b; [[ -e $V/spool/$NAME$z$(printf %02d "$b") ]] && bb=$((b + 50))   # a zone run twice per round: batch + 50
    run=$NAME$z$(printf %02d "$bb"); k=$(( N - b * B < B ? N - b * B : B ))
    SCORER=$V/Alpamayo-Cosmos/tools/scorer setsid $V/venv-rl/bin/python -u $E2E/tools/rl_episode.py "$run" \
      --scorer $V/Alpamayo-Cosmos/tools/scorer --out $V/spool --host local --direct --grpc 127.0.0.1:7789 --world 1 \
      --objinfo-port 27505 --launcher $d/MoraiLauncher_Lin_Data --port 8092 --seed $(( SEED * 100 + bb )) \
      --loc-restart "bash $HERE/shadow.sh restart 1; bash $HERE/popout_rec.sh start $run" --end-hook "bash $HERE/popout_rec.sh stop" \
      --limit-s 40 --end-on-hit --no-slow-end --scene-preset popout ${RANGE[$z]} --episodes "$k" > "$OUT/$run.log" 2>&1 < /dev/null
    stamp "$run: $(grep -c '^episode' "$OUT/$run.log") episodes ($(grep '^episode' "$OUT/$run.log" | sed 's/^episode [0-9]* \([^:]*\): \([a-z_0-9]*\) \([0-9.]*\) s.*/\2 \3/' | tr '\n' ',' | cut -c1-200)), bags $(ls $V/bags/$run 2> /dev/null | wc -l)"
    bash "$HERE/popout_post.sh" "$run" > "$OUT/post_$run.log" 2>&1 &   # step0 + labels + B2 while the next batch records
    POSTS+=" $!"
    (b2env; nice $V/venv-tools/bin/b2 sync --no-progress --threads 4 "$OUT/" b2://morai2026-alpamayo/step0_popout/_popout/results/$NAME/ > /dev/null 2>&1) &   # logs survive a lost Mac launcher
  done
done
bash "$HERE/popout_rec.sh" down; bash "$HERE/shadow.sh" down > /dev/null; bash "$HERE/worlds.sh" down > /dev/null
mkdir -p "$OUT/drive_logs"; cp /root/shadow/1/*.log /root/shadow/1/rec/*.log "$OUT/drive_logs/" 2> /dev/null
stamp "recording done: $(ls $V/bags/*/*.bag 2> /dev/null | wc -l) bags, $(du -sh $V/bags | cut -f1)"

for p in $POSTS; do wait $p; done
stamp "batches processed: $(grep -l '^B2 .* ok' $OUT/post_*.log 2> /dev/null | wc -l) of $(ls $OUT/post_*.log | wc -l) on B2 ($(tail -qn1 $OUT/post_*.log | grep -o '"kinds": {[^}]*}' | tr '\n' ' ' | cut -c1-300))"
POST_JOBS=$T bash "$HERE/popout_post.sh" final "$NAME" > "$OUT/post_final.log" 2>&1
stamp "final: $(grep -E 'dyn_stage|OK:|LOAD_CHECK|Error|assert' "$OUT/post_final.log" | tail -3 | tr '\n' ' ' | cut -c1-300)"
(b2env; $V/venv-tools/bin/b2 sync --threads 16 --no-progress --exclude-regex '(.*\.tmp.*|.*_ext/.*|.*_(dyn|rh)[0-9]{5}(/.*|\.npz)|_dyn_relabel/ego/.*)' $V/step0_popout/ b2://morai2026-alpamayo/step0_popout/ > "$OUT/b2_step0.log" 2>&1 &&
 $V/venv-tools/bin/b2 sync --threads 16 --no-progress "$OUT/" b2://morai2026-alpamayo/step0_popout/_popout/results/$NAME/ > "$OUT/b2_results.log" 2>&1) && echo B2_OK > "$OUT/B2_OK" && stamp "B2 step0_popout ok" || stamp "B2 upload failed (results/$NAME/b2_*.log)"
kill $KA 2> /dev/null
touch "$OUT/DONE"
