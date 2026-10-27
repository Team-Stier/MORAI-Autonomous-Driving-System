#!/usr/bin/env bash
# prewarm.sh MANIFEST (on a booted box, setup.sh done): ready for a delivery that is not out yet. World 1 and stack 1 come up
# with eval3.sh's real-time CPU layout and apply delay (eval3.sh reuses them), and the engine is compiled for the new
# delivery's layer formats: a stand-in delivery = the bundled WoWmayo final/ (same model, same shapes) + the new
# quant_manifest.json (its formats and amax), run until "listening" (the warm-up compiles and autotunes into $V/cache), then
# stopped. When the delivery lands only its weights are fetched (eval3.sh DELIVER=b2:...) and the engine finds the cache.
set -uo pipefail
HERE=$(cd "$(dirname "$0")" && pwd); V=${V:-/workspace}; E2E=$V/MORAI-Autonomous-Driving-System/src/e2e_driver
P=$V/models/prewarm; mkdir -p $P; cp "$1" $P/quant_manifest.json; ln -sfn $V/models/wowmayo_final/final $P/final
T=$(nproc --all); PER=12; S=8; (( T < 28 )) && { S=4; PER=$((T - 8)); }        # eval3.sh's split for one world
export CPUS_1=0-$((PER - 1)) STACK_CPUS=$PER-$((PER + S - 1)) ENGINE_CPUS=$((PER + S))-$((PER + 2 * S - 1))
log() { echo "$(date +%T) $*"; }
bash "$HERE/worlds.sh" up 1 > /root/prewarm_worlds.log 2>&1 &
W=$!
E2E_APPLY_DELAY_S=${APPLY_DELAY_S:-0.1} bash "$HERE/stacks.sh" up 1 > /root/prewarm_stacks.log 2>&1
OMP_NUM_THREADS=4 MALLOC_MMAP_MAX_=0 MALLOC_TRIM_THRESHOLD_=1073741824 HF_HUB_OFFLINE=1 HF_HOME=$V/models/hf \
TORCHINDUCTOR_CACHE_DIR=$V/cache/inductor TRITON_CACHE_DIR=$V/cache/triton \
  taskset -c "$ENGINE_CPUS" $V/venv-engine/bin/python -u $E2E/server/model_server.py --backend engine --ckpt $P \
  --vlm Qwen/Qwen3-VL-4B-Instruct --sims 1 --port 47019 > /root/prewarm_engine.log 2>&1 < /dev/null &
E=$!
log "world, stack and stand-in engine starting"
wait $W && log "world ready" || log "world failed: $(tail -2 /root/prewarm_worlds.log)"
until grep -q "listening on" /root/prewarm_engine.log || ! kill -0 $E 2> /dev/null; do sleep 2; done
grep -q "listening on" /root/prewarm_engine.log && log "engine compiled: $(grep -o 'loaded in.*' /root/prewarm_engine.log | cut -c1-100)" ||
  log "engine failed: $(tail -3 /root/prewarm_engine.log)"
kill $E 2> /dev/null; wait $E 2> /dev/null; log "stand-in engine stopped; waiting for the delivery"
