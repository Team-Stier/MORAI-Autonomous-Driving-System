#!/usr/bin/env bash
# The delivery drive, from the Mac, once both hosts are ours: one lap with control.mode map (the fixed tables; the user
# dropped the MLP on 10/27). MODES="map mlp" would add a lap with the mlp node on the same seed (engine stays up).
#   delivery_drive.sh DELIVER_DIR(on winpc) NAME [SEED]
# Per drive: eval.sh NAME_<mode> --laps 1 --seed SEED, the simulator window recorded on the Mac (lap_video.py record,
# 15 fps 1280 wide), and on winpc telemetry_rec.py + /e2e_driver/debug + /sim_clock/state; everything lands in
# ~/morai-eval/drives/NAME on the Mac. The last step stops the stack, server, tunnels, viewer and simulator, so the hosts
# can go back right away; the 1x videos (lap_video.py make) are made afterwards on the Mac.
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
DELIVER=$1; NAME=$2; SEED=${3:-7}
OUT=$HOME/morai-eval/drives/$NAME; mkdir -p "$OUT"
W=morai-eval/ws/MORAI-Autonomous-Driving-System/src/e2e_driver
L='~/Workspaces/scratches/morai/MoraiLauncher_Lin_Data/SaveFile/LinkConnection.txt'
msc() { ssh bc-250 "python3 ~/msc.py $L $1 '${2:-}' ${3:-30}" | tail -1; }
log() { echo "$(date +%T) $*" | tee -a "$OUT/drive.log"; }

log "sync and delivery $DELIVER"
rsync -a --exclude __pycache__ --exclude '*.pyc' "$HERE/../" "winpc-wsl:$W/"
ssh winpc-wsl "cd $W && sed -i 's|^E2E_MODEL_CKPT=.*|E2E_MODEL_CKPT=$DELIVER|' config/runtime.env && \
  sed -i 's|^  mode: .*|  mode: map|' config/e2e_driver.yaml && grep -n '^E2E_MODEL_CKPT\|^  mode:' config/runtime.env config/e2e_driver.yaml" | tee -a "$OUT/drive.log"

if ! ssh bc-250 'pgrep -f "[S]imulator[.](x86_64|real)" > /dev/null'; then
  log "simulator: execute"
  msc status "" 8 | grep -q platform || { log "launcher sends no MSC status: restart it (README two-PC rehearsal)"; exit 1; }
  msc execute "" 120
  for i in $(seq 60); do msc status "" 5 | grep -q "'platform': 2.*'status': 1," && break; sleep 3; done
fi
log "5080 before the server (Windows desktop baseline): $(ssh winpc-wsl 'nvidia-smi --query-gpu=memory.used --format=csv,noheader')"
log "two_pc up + prep"
(SIM=bc250 FAKE_FOLLOW_ROUTE_M=0 bash "$HERE/two_pc.sh" up > "$OUT/up.log" 2>&1 &)
SIM=bc250 bash "$HERE/two_pc.sh" prep 2>&1 | tail -6 | tee -a "$OUT/drive.log"
ssh winpc-wsl 'timeout 900 bash -c "until grep -q \"listening on\|Error\|Traceback\" ~/morai-eval/run/server.log; do sleep 3; done"; grep "loaded in\|Error" ~/morai-eval/run/server.log | cut -c1-600' | tee -a "$OUT/drive.log"
log "5080 after the server warm-up: $(ssh winpc-wsl 'nvidia-smi --query-gpu=memory.used --format=csv,noheader; nvidia-smi --query-compute-apps=pid,used_memory --format=csv,noheader' | tr '\n' ' ')"

for MODE in ${MODES:-map}; do
  N=${NAME}_$MODE
  if [[ $MODE == mlp ]]; then
    log "stack restart with control.mode mlp (engine stays)"
    ssh winpc-wsl "cd $W && sed -i 's|^  mode: .*|  mode: mlp|' config/e2e_driver.yaml && cd tools/winpc && ./stack.sh down && FAKE_FOLLOW_ROUTE_M=0 ./stack.sh up"
  fi
  ssh winpc-wsl 'timeout 120 bash -c "until grep -q \"Started e2e_driver\" ~/morai-eval/run/stack.log; do sleep 2; done"'; sleep 8
  log "drive $N: recorders"
  ssh winpc-wsl "cd $W/tools/winpc && rm -f ~/morai-eval/data/stop_$N && (setsid nohup ./chroot.sh rec \"source devel/setup.bash; \
    python3 src/e2e_driver/tools/telemetry_rec.py /data/tel_$N.csv & PYTHONUNBUFFERED=1 rostopic echo -p /e2e_driver/debug/data > /data/dbg_$N.csv & \
    rostopic echo -p /sim_clock/state > /data/clk_$N.csv & until [ -e /data/stop_$N ]; do sleep 1; done; kill %1 %2 %3\" > /dev/null 2>&1 < /dev/null &)"
  uv run --no-project -q --with numpy python "$HERE/lap_video.py" record http://192.168.0.15:8090/ "$OUT/frames_$MODE" > /dev/null 2>&1 &
  REC=$!
  ssh winpc-wsl "(setsid nohup nvidia-smi --query-gpu=timestamp,memory.used --format=csv,noheader,nounits -lms 1000 > ~/morai-eval/data/vram_$N.csv 2>&1 < /dev/null & echo \$! > ~/morai-eval/run/vram.pid)"
  log "drive $N: eval lap seed $SEED"
  VIEW_FPS=15 VIEW_WIDTH=1280 bash "$HERE/eval.sh" "$N" --laps 1 --seed "$SEED" 2>&1 | grep -v "ev_poll_posix" | tee -a "$OUT/drive.log"
  kill $REC 2>/dev/null
  ssh winpc-wsl "touch ~/morai-eval/data/stop_$N; kill \$(cat ~/morai-eval/run/vram.pid) 2>/dev/null"; sleep 3
  log "5080 during the lap: max $(ssh winpc-wsl "cut -d, -f2 ~/morai-eval/data/vram_$N.csv | sort -n | tail -1") MiB; server peaks: $(ssh winpc-wsl 'grep -o "gpu {[^}]*}" ~/morai-eval/run/server.log | tail -1')"
  for f in tel dbg clk vram; do scp -q "winpc-wsl:morai-eval/data/${f}_$N.csv" "$OUT/"; done
  scp -q winpc-wsl:morai-eval/run/stack.log "$OUT/stack_$MODE.log"
  cp -r "$HOME/morai-eval/runs/$N" "$OUT/eval_$MODE"
  log "drive $N done: $(tail -1 "$OUT/eval_$MODE/laps.jsonl" | cut -c1-200)"
done

log "hosts back: stack, server, tunnels, viewer, simulator down"
scp -q winpc-wsl:morai-eval/run/server.log "$OUT/server.log"
ssh winpc-wsl "cd $W && sed -i 's|^  mode: .*|  mode: map|' config/e2e_driver.yaml"
SIM=bc250 bash "$HERE/two_pc.sh" down 2>&1 | tail -2
ssh bc-250 'python3 ~/morai-eval-wtc.py down > /dev/null 2>&1; kill $(cat ~/morai-eval-viewer.pid 2>/dev/null) 2>/dev/null; true'
ssh bc-250 'p=$(pgrep -f "[S]imulator[.](x86_64|real)" | head -1); [ -n "$p" ] && kill $p; true'
ssh winpc-wsl 'nvidia-smi --query-gpu=utilization.gpu,memory.used --format=csv,noheader; echo "compute apps: $(nvidia-smi --query-compute-apps=pid --format=csv,noheader | wc -l)"' | tee -a "$OUT/drive.log"
log "done: $OUT"
