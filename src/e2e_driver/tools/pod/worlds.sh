#!/usr/bin/env bash
# worlds.sh up N | down | status: N MORAI worlds, world n on its own Xvfb :n (one window per screen for viewer.py), without the launcher; the simulator logs itself in from the
# copied prefs and Version/V2AccountInfo.byte (10/26 pod, cloud-rl-pod-scripts-20261026 inst.sh). World n:
#   - files: world 1 is /root/Workspaces/scratches/morai, world n >= 2 a clone /root/inst/n/morai (mk-clone.sh): MSC
#     10508/10326 + (n-1)*1000 and every UDP port in the w1_collect presets and scene network files + (n-1)*1000
#   - gRPC 7788+n: the Unity pref gRPCPort in world n's own HOME (/root/inst/n/home, a copy of /root/.config made at its
#     start), so the worlds start together, STAGGER_S (5) apart (one shared prefs file made each wait for the previous
#     login: 8 worlds took minutes)
#   - CPU affinity CPUS_n (default 12 logical CPUs from 12(n-1)): with all of a big host's CPUs visible Mono dies at load
#     (mono-threads.c:568); sim-args -force-vulkan -force-gfx-mt -job-worker-count 8 (setup.sh) are the other half of
#     that fix. The camera busy-wait threads are pinned to one core per world afterwards (pin.sh, 6.5 -> 4.6 cores).
#   - then: map K-City / Ioniq5, pause, w1_collect network + sensors, all worlds in parallel. Scenes: rl_episode.py per
#     episode, eval_manager.py per lap.
set -uo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
SRC=/root/Workspaces/scratches/morai; PREFS=/root/.config/unity3d/MORAI/Simulator/prefs
SIMDIR=MoraiLauncher_Lin_Data/Project/Simulator/Simulator_v.S4.251001.MolitComp03_Linux
dir() { [[ $1 == 1 ]] && echo $SRC || echo /root/inst/$1/morai; }
M() { python3 /root/morai-eval-msc.py "$(dir "$1")/MoraiLauncher_Lin_Data/SaveFile/LinkConnection.txt" "${@:2}"; }

pref() {  # file name value: set (or add) <pref name=NAME type="int"> in a Unity prefs file
  python3 - "$@" <<'PY'
import re, sys
p, name, value = sys.argv[1:4]
s = open(p, encoding="utf-8").read()
pref = f'<pref name="{name}" type="int">{value}</pref>'
s = re.sub(rf'<pref name="{name}" type="int">\d+</pref>', pref, s) if f'name="{name}"' in s else s.replace("</unity_prefs>", f"\t{pref}\n</unity_prefs>")
open(p, "w", encoding="utf-8").write(s)
PY
}

start1() {
  local n=$1 d; d=$(dir "$n")
  [[ $n == 1 || -e $d ]] || bash "$HERE/mk-clone.sh" "$n"
  local v=CPUS_$n; local cpus=${!v:-$(( (n - 1) * 12 ))-$(( n * 12 - 1 ))}
  local log=/root/inst/$n/Player.log h=/root/inst/$n/home; mkdir -p "$h"; rm -f "$log"
  cp -a /root/.config "$h/"
  pref "$h/.config/unity3d/MORAI/Simulator/prefs" gRPCPort $((7788 + n))
  pgrep -f "^Xvfb :$n " > /dev/null || { setsid Xvfb :$n -screen 0 1280x800x24 > /root/inst/$n/xvfb.log 2>&1 < /dev/null & sleep 1; }
  ( cd "$d/$SIMDIR" && exec env HOME="$h" DISPLAY=:$n LANG=ko_KR.UTF-8 LC_ALL=ko_KR.UTF-8 setsid taskset -c "$cpus" ./Simulator.x86_64 -logFile "$log" > /dev/null 2>&1 < /dev/null ) &
}

login1() {
  local n=$1 log=/root/inst/$1/Player.log cpus; cpus=$(taskset -pc "$(pgrep -f "inst/$n/Player[.]log" | head -1)" 2> /dev/null | sed 's/.*: //')
  for i in $(seq 60); do grep -q 'Login success' "$log" 2>/dev/null && break; sleep 3; done
  grep -q 'Login success' "$log" || { echo "w$n: no login"; tail -3 "$log"; return 1; }
  echo "$(date +%T) w$n logged in (cpus $cpus, gRPC $((7788 + n)), DISPLAY :$n)"
}

prep1() {
  local n=$1 log=/root/inst/$1/Player.log i
  for i in $(seq 40); do M "$n" status "" 4 2>/dev/null | grep -q "platform.: 2.*status.: 1," && break; sleep 3; done
  echo "$(date +%T) w$n lobby ($(grep -m1 -o 'Run server.*' "$log" | cut -c1-60))"
  M "$n" map R_KR_PR_K-city_2025/2023_Hyundai_Ioniq5 300 | tail -1 | cut -c1-50
  M "$n" pause "" 30 > /dev/null
  echo "w$n net $(M "$n" load-network w1_collect 60 | cut -c1-30) sens $(M "$n" load-sensors w1_collect 60 | cut -c1-30)"
}

case ${1:-} in
  up)
    # SIM_FPS: Unity TargetFrameRate (prefs 60). MORAI steps ~0.06 s of simulator time per frame below ~17 fps, so the
    # cap sets the RTF (~0.06 x fps) with the same physics step and camera rate per simulator second (10/27 pod)
    pref "$PREFS" TargetFrameRate "${SIM_FPS:-60}"
    new=()
    for n in $(seq "${2:-3}"); do   # a world already up stays as it is
      pgrep -f "inst/$n/Player[.]log" > /dev/null && { echo "w$n already up"; continue; }
      start1 "$n"; new+=("$n"); sleep "${STAGGER_S:-5}"
    done
    pids=(); for n in "${new[@]}"; do (login1 "$n" && prep1 "$n") & pids+=($!); done
    for p in "${pids[@]}"; do wait "$p" || exit 1; done
    [[ ${SKIP_PIN:-0} == 1 ]] || bash "$HERE/pin.sh" ;;
  down) pkill -f "[S]imulator.real"; sleep 3; pkill -x Xvfb; true ;;
  status) for p in $(pgrep -f "[S]imulator.real"); do echo "$p $(readlink /proc/$p/cwd) $(taskset -pc $p | sed 's/.*: //')"; done ;;
  *) sed -n 2,12p "$0"; exit 1 ;;
esac
