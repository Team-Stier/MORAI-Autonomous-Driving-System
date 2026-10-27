#!/bin/bash
# pin.sh: for every Simulator.real, pin its busy-wait camera threads (>90% CPU over 3 s, named like the process, not the
# main thread) to the last physical core of its current affinity (both hyperthreads), the rest stay. As bc-250 sim-pin.sh.
hz=$(getconf CLK_TCK)
for P in $(pgrep -f "[S]imulator.real"); do
  aff=$(taskset -pc $P | sed 's/.*: //'); lo=${aff%%,*}; hi=${aff##*,}; sc="${lo##*-},${hi##*-}"
  snap() { for T in $(ls /proc/$P/task); do echo "$T $(awk '{print $14+$15}' /proc/$P/task/$T/stat 2>/dev/null)"; done | sort; }
  a=$(snap); sleep 3; b=$(snap)
  spin=$(join <(echo "$a") <(echo "$b") | awk -v hz=$hz -v m=$P '($3-$2)/hz/3 > 0.9 && $1!=m {print $1}' |
         while read T; do grep -q '^Simulat' /proc/$P/task/$T/comm && echo $T; done)
  for T in $spin; do taskset -pc $sc $T > /dev/null; done
  echo "sim $P: spinners [$(echo $spin)] -> $sc"
done
