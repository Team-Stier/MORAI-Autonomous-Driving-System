#!/usr/bin/env bash
# actor.sh up NAME [N] | down: one RL actor pod (RL_PLAN.md 3.1): N (3) MORAI worlds, the model server, N ROS stacks and
# an episode manager per world (rl_episode.py, records under $V/rl-out/NAME_wn). After setup.sh.
#   ENGINE_ARGS   model_server.py arguments (default: the batched RL engine of branch rl/actor-engine on the volume's
#                 delivery, all N nodes on one port, slots in connection order; add the RL flags --rl-sigma S --expert2
#                 --lora-watch FILE, side-session-notes rl-actor-engine-20261027.md; "--backend fake --sims N
#                 --fake-speed 9" for a plumbing test without a GPU model)
#   CPUS_n / STACK_CPUS / ENGINE_CPUS   CPU affinities (worlds.sh; stacks and engine default to the CPUs after the worlds)
#   RL_ARGS       extra rl_episode.py arguments (e.g. --episodes 20 --start-cp 9 --end-cp 12)
set -uo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
V=${V:-/workspace}; WS=$V/MORAI-Autonomous-Driving-System; E2E=$WS/src/e2e_driver
N=${3:-3}; NAME=${2:-actor}; R=/root/actor; mkdir -p $R
export STACK_CPUS=${STACK_CPUS:-$((12 * N))-$((12 * N + 7))} ENGINE_CPUS=${ENGINE_CPUS:-$((12 * N + 8))-$((12 * N + 15))}
ENGINE_ARGS=${ENGINE_ARGS:---backend engine --ckpt $V/models/deliver --vlm Qwen/Qwen3-VL-4B-Instruct --sims $N}

engine() {  # n server(s): one batched engine on 47011, or one per stack with ENGINE_PORT_STEP=1 (server.sh's settings)
  local k
  for k in $(seq 0 $(( ${ENGINE_PORT_STEP:-0} ? N - 1 : 0 ))); do
    OMP_NUM_THREADS=4 MALLOC_MMAP_MAX_=0 MALLOC_TRIM_THRESHOLD_=1073741824 HF_HUB_OFFLINE=1 HF_HOME=$V/models/hf \
    MORAI_VLA_PYLIB=$V/morai-student/pylib setsid taskset -c "$ENGINE_CPUS" $V/venv-engine/bin/python -u $E2E/server/model_server.py \
      $ENGINE_ARGS --port $((47011 + k)) > $R/server$k.log 2>&1 < /dev/null &
    echo $! >> $R/server.pids
  done
}

case ${1:-} in
  up)
    bash "$HERE/worlds.sh" up "$N" || exit 1
    engine
    bash "$HERE/stacks.sh" up "$N"
    sleep 20                                              # nodes up, engine warm-up running
    for n in $(seq "$N"); do
      d=$([[ $n == 1 ]] && echo /root/Workspaces/scratches/morai || echo /root/inst/$n/morai)
      SCORER=$V/Alpamayo-Cosmos/tools/scorer setsid $V/venv-rl/bin/python -u $E2E/tools/rl_episode.py "${NAME}_w$n" \
        --scorer $V/Alpamayo-Cosmos/tools/scorer --out $V/rl-out --host local --direct --grpc 127.0.0.1:$((7788 + n)) \
        --launcher $d/MoraiLauncher_Lin_Data --port $((8091 + n)) --seed $n \
        --loc-restart "source /opt/ros/noetic/setup.bash; ROS_MASTER_URI=http://127.0.0.1:$((11310 + n)) rosnode kill /localization_node" \
        ${RL_ARGS:-} > $R/episodes_w$n.log 2>&1 < /dev/null &
      echo $! >> $R/managers.pids
      sleep 5                                             # managers deploy the same scene tools to ~ in turn
    done
    echo "status: curl localhost:$((8092))/status (world n: 8091+n); logs $R" ;;
  down)
    for f in managers server; do [[ -e $R/$f.pids ]] && { kill $(cat $R/$f.pids) 2>/dev/null; mv $R/$f.pids $R/$f.pids.old; }; done
    bash "$HERE/stacks.sh" down; bash "$HERE/worlds.sh" down ;;
  *) sed -n 2,8p "$0"; exit 1 ;;
esac
