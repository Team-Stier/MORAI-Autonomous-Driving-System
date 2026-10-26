#!/usr/bin/env bash
# Two-PC operation from the Mac: MORAI on bc-250 (SIM=bc250) or the W1 copy on winpc (SIM=w1), stack + model on winpc WSL.
#   two_pc.sh up [server args]   tunnels (bc250), model server (default: fake, route-following), ROS stack
#   two_pc.sh prep [SCENE]       bc250: load the map, then the named collection presets (w1_collect), then SCENE paused
#   two_pc.sh drive NAME SECS [SCENE]
#        probes + raw GT + debug/localization recorders, play; whenever the Ego has stood still STUCK_S (10) s while
#        asked to drive, reload SCENE (pause, load-scene, restart localization, play) with the rest of the stack left
#        running; summary at the end (test/drive_summary.py). EPISODE_S > 0 also reloads every EPISODE_S s.
#   two_pc.sh reset [SCENE]      one reload as above
#   two_pc.sh down
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
SIM=${SIM:-bc250}; BC=${BC:-bc-250}; SCENE_DEFAULT=rs2041_001/62; STUCK_S=${STUCK_S:-10}
W=morai-eval/ws/MORAI-Autonomous-Driving-System/src/e2e_driver/tools/winpc
L='~/Workspaces/scratches/morai/MoraiLauncher_Lin_Data/SaveFile/LinkConnection.txt'
msc() {  # <action> [option] [timeout]
  if [[ $SIM == bc250 ]]; then
    ssh "$BC" "python3 ~/morai-eval-msc.py $L $1 '${2:-}' ${3:-30}" | tail -1
  else
    ssh -o BatchMode=yes winpc-win "powershell -NoProfile -ExecutionPolicy Bypass -File C:/morai-par/windows_msc.ps1 -Action $1 ${2:+-Option $2} -Timeout ${3:-30} -LinkFile C:/morai/multi-03-20261007/MoraiLauncher_Win_Data/SaveFile/LinkConnection.txt" 2>&1 | tr -d '\000' | tail -1
  fi
}
wsl() { ssh winpc-wsl "$@"; }
loc_restart() {  # localization does not follow a teleport (or a new run): it respawns from the next GPS fix
  wsl "cd $W && ./chroot.sh ctl 'source devel/setup.bash; rosnode kill /localization_node'" > /dev/null 2>&1; sleep 4
}
reset() {  # <scene>: reload it paused, restart localization, play
  msc pause > /dev/null; msc load-scene "$1" 90 > /dev/null; loc_restart; msc play > /dev/null
}

case ${1:-} in
  up)
    shift
    if [[ $SIM == bc250 ]]; then
      scp -q "$HERE/lan_tunnel.py" "$BC:morai-eval-lan_tunnel.py"; scp -q "$HERE/msc.py" "$BC:morai-eval-msc.py"
      ssh "$BC" 'setsid nohup python3 ~/morai-eval-lan_tunnel.py sim --listen 192.168.0.15:30011 > ~/morai-eval-tunnel.log 2>&1 < /dev/null & echo $! > ~/morai-eval-tunnel.pid'
      wsl 'cd ~/morai-eval/ws/MORAI-Autonomous-Driving-System/src/e2e_driver/tools && setsid nohup python3 -u lan_tunnel.py stack --connect 192.168.0.15:30011 > ~/morai-eval/run/lan_stack.log 2>&1 < /dev/null & echo $! > ~/morai-eval/run/lan_stack.pid'
    else                                                   # W1 copy on winpc: the collection tunnel (morai-parallel-collect)
      wsl 'setsid nohup python3 ~/.claude-morai-wslros-20261018/bin/udp-tunnel.py 30001 0 9093 > ~/morai-eval/run/tunnel.log 2>&1 < /dev/null & echo $! > ~/morai-eval/run/tunnel.pid'
      STACKS=wsl bash ~/.claude/skills/morai-parallel-collect/scripts/parallel.sh win_tunnel 7505,7509,9092,9095,9099,9291,9293,9295,9297,9301,9999,2368 30001 | tail -1
    fi
    args=${*:-"--backend fake --fake-speed 7 --preprocess-config \$(ls -d ~/.cache/huggingface/hub/models--nvidia--Alpamayo-1.5-10B/snapshots/*)"}
    wsl "cd $W && ./server.sh up $args && FAKE_FOLLOW_ROUTE_M=${FAKE_FOLLOW_ROUTE_M:-8} ./stack.sh up" ;;
  prep)
    SCENE=${2:-$SCENE_DEFAULT}
    if [[ $SIM == bc250 ]]; then                          # the W1 copy's own defaults are the collection presets
      msc pause; msc map R_KR_PR_K-city_2025/2023_Hyundai_Ioniq5 150
      msc load-network w1_collect 60; msc load-sensors w1_collect 60
    fi
    msc pause; msc load-scene "$SCENE" 90 ;;
  drive)
    NAME=$2; SECS=$3; SCENE=${4:-$SCENE_DEFAULT}
    wsl "cd $W && rm -f ~/morai-eval/data/stop_$NAME
      (setsid nohup ./chroot.sh probes 'bash src/e2e_driver/tools/winpc/probes.sh $NAME' > ~/morai-eval/run/probe_$NAME.log 2>&1 < /dev/null &)
      (setsid nohup ./chroot.sh rec 'source devel/setup.bash; rostopic echo -p /e2e_driver/debug/data > /data/dbg_$NAME.csv & rostopic echo -p /localization/ego/pose/pose/position > /data/loc_$NAME.csv & rostopic echo -p /section_annotation/state > /data/sec_$NAME.csv & until [ -e /data/stop_$NAME ]; do sleep 1; done; kill %1 %2 %3' > /dev/null 2>&1 < /dev/null &)
      (setsid nohup python3 ~/morai-eval/ws/MORAI-Autonomous-Driving-System/src/e2e_driver/test/gtraw.py ~/morai-eval/data/gtraw_$NAME.txt ~/morai-eval/data/stop_$NAME > /dev/null 2>&1 < /dev/null &)"
    loc_restart; sleep 4; msc play; t0=$(date +%s); te=$t0; still=0; resets=0
    while (( $(date +%s) - t0 < SECS )); do
      sleep 5
      v=$(wsl "tail -1 ~/morai-eval/data/dbg_$NAME.csv" | python3 -c 'import json,sys; print(abs(json.loads(sys.stdin.read().split(",", 1)[1])["v"]))' 2>/dev/null || echo 0)
      if python3 -c "import sys; sys.exit(0 if float('$v') < 0.5 else 1)"; then still=$((still + 5)); else still=0; fi
      if (( still >= STUCK_S )) || (( ${EPISODE_S:-0} > 0 && $(date +%s) - te >= ${EPISODE_S:-0} )); then
        resets=$((resets + 1)); echo "$(date +%T) reset $resets (still ${still}s, episode $(( $(date +%s) - te ))s)"
        reset "$SCENE"; still=0; te=$(date +%s)
      fi
    done
    wsl "touch ~/morai-eval/data/stop_$NAME; sleep 20"
    msc pause > /dev/null
    echo "resets $resets"
    wsl "cd ~/morai-eval/ws/MORAI-Autonomous-Driving-System/src/e2e_driver && ~/claude-bench/venv-teacher/bin/python test/drive_summary.py $NAME" ;;
  reset) reset "${2:-$SCENE_DEFAULT}" ;;
  down)
    wsl "cd $W && ./stack.sh down; ./server.sh down; for f in lan_stack tunnel; do [ -e ~/morai-eval/run/\$f.pid ] && kill \$(cat ~/morai-eval/run/\$f.pid) 2>/dev/null; done; true"
    if [[ $SIM == bc250 ]]; then ssh "$BC" 'kill $(cat ~/morai-eval-tunnel.pid) 2>/dev/null; true'
    else ssh winpc-win 'powershell -NoProfile -ExecutionPolicy Bypass -File C:/morai-par/stop_tunnels.ps1' 2>&1 | tr -d '\000' | tail -1; fi ;;
  *) sed -n 2,9p "$0"; exit 1 ;;
esac
