#!/bin/bash
# mon.sh <secs>: container CPU cores used (cgroup), each Simulator.real's CPU cores, GPU util/mem/power, memory; 5 s steps.
T=${1:-60}; c0=$(awk '/usage_usec/{print $2}' /sys/fs/cgroup/cpu.stat); t0=$(date +%s.%N)
declare -A j0; for p in $(pgrep -f "[S]imulator.real"); do j0[$p]=$(awk '{print $14+$15}' /proc/$p/stat); done
sleep $T
c1=$(awk '/usage_usec/{print $2}' /sys/fs/cgroup/cpu.stat); t1=$(date +%s.%N); dt=$(echo "$t1 - $t0" | bc)
printf 'cgroup_cores=%.1f ' $(echo "($c1 - $c0) / 1000000 / $dt" | bc -l)
for p in $(pgrep -f "[S]imulator.real"); do printf 'sim%s_cores=%.1f ' $p $(echo "($(awk '{print $14+$15}' /proc/$p/stat) - ${j0[$p]:-0}) / 100 / $dt" | bc -l); done
echo "gpu=$(nvidia-smi --query-gpu=utilization.gpu,memory.used,power.draw,clocks.sm --format=csv,noheader | tr -d ' ') mem_GB=$(( $(cat /sys/fs/cgroup/memory.current) / 1000000000 )) sim_rss_GB=$(ps -o rss= -p $(pgrep -d, -f '[S]imulator.real') | awk '{s+=$1} END {printf "%.1f", s/1e6}')"
