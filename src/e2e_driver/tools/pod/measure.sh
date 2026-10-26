#!/usr/bin/env bash
# measure.sh SECS: CPU cores per part while the actor drives (the ROS stacks' share was never measured, RL_PLAN 3.1):
# each simulator, each stack (its whole session: roscore, bridge, velodyne, vlp16_bev, localization, e2e_driver, ...),
# each model server, the episode managers, and the container total (cgroup), plus GPU and memory. Children that exit
# during the window are missed (shortcut: fine for steady driving; use cgroup v2 per-part slices if parts churn).
T=${1:-60}; hz=$(getconf CLK_TCK)
parts() {  # "name pid..." per line
  for p in $(pgrep -f "[S]imulator.real"); do echo "sim:$(readlink /proc/$p/cwd | grep -o 'inst/[0-9]*' || echo inst/1) $p"; done
  for f in /root/stack/*/stack.pid; do [[ -e $f ]] && echo "stack:$(basename "$(dirname "$f")") $(pgrep -s "$(cat "$f")" | tr '\n' ' ')"; done
  [[ -e /root/actor/server.pids ]] && for p in $(cat /root/actor/server.pids); do echo "engine:$p $p $(pgrep -P "$p" | tr '\n' ' ')"; done
  [[ -e /root/actor/managers.pids ]] && echo "managers $(cat /root/actor/managers.pids | tr '\n' ' ')"
}
jiffies() { local s=0; for p in "$@"; do s=$((s + $(awk '{print $14+$15}' /proc/$p/stat 2>/dev/null || echo 0))); done; echo $s; }
declare -A j0 members
while read -r name pids; do members[$name]=$pids; j0[$name]=$(jiffies $pids); done < <(parts)
c0=$(awk '/usage_usec/{print $2}' /sys/fs/cgroup/cpu.stat); t0=$(date +%s.%N)
sleep "$T"
c1=$(awk '/usage_usec/{print $2}' /sys/fs/cgroup/cpu.stat); dt=$(echo "$(date +%s.%N) - $t0" | bc)
for name in $(printf '%s\n' "${!members[@]}" | sort); do
  printf '%-14s %5.2f cores\n' "$name" "$(echo "($(jiffies ${members[$name]}) - ${j0[$name]}) / $hz / $dt" | bc -l)"
done
printf 'cgroup total   %5.2f cores (quota %s)\n' "$(echo "($c1 - $c0) / 1000000 / $dt" | bc -l)" "$(cat /sys/fs/cgroup/cpu.max)"
echo "gpu $(nvidia-smi --query-gpu=utilization.gpu,memory.used,power.draw --format=csv,noheader | tr -d ' ')  mem $(( $(cat /sys/fs/cgroup/memory.current) / 1000000000 )) GB"
