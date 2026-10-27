#!/bin/bash
# pin.sh: for every Simulator.real, pin its busy-wait camera threads (>90% CPU over 3 s, named like the process, not the
# main thread) to the last physical core of its current affinity (both hyperthreads), the rest stay. As bc-250 sim-pin.sh.
# SPIN_CPUS (a CPU list): every simulator's spinners there instead (caps what idle worlds burn of the pod's CPU quota).
hz=$(getconf CLK_TCK)
for P in $(pgrep -f "[S]imulator.real"); do
  aff=$(taskset -pc $P | sed 's/.*: //'); lo=${aff%%,*}; hi=${aff##*,}; sc="${lo##*-},${hi##*-}"
  snap() { for T in $(ls /proc/$P/task); do echo "$T $(awk '{print $14+$15}' /proc/$P/task/$T/stat 2>/dev/null)"; done | sort; }
  a=$(snap); sleep 3; b=$(snap)
  spin=$(join <(echo "$a") <(echo "$b") | awk -v hz=$hz -v m=$P '($3-$2)/hz/3 > 0.9 && $1!=m {print $1}' |
         while read T; do grep -q '^Simulat' /proc/$P/task/$T/comm && echo $T; done)
  if [[ -n ${SPIN_CPUS:-} ]]; then   # many worlds share the quota, so spinners may run < 0.9 cores: take the camera
    sc=$SPIN_CPUS                    # threads by their total CPU time (> 1/3 of the main thread's) instead
    m=$(awk '{print $14+$15}' /proc/$P/stat 2>/dev/null)
    spin=$(for T in $(ls /proc/$P/task); do [[ $T != "$P" ]] && grep -q '^Simulat' /proc/$P/task/$T/comm &&
           awk -v T=$T -v m=$(awk '{print $14+$15}' /proc/$P/task/$P/stat) '$14+$15 > m / 3 {print T}' /proc/$P/task/$T/stat; done)
  fi
  for T in $spin; do taskset -pc $sc $T > /dev/null; done
  echo "sim $P: spinners [$(echo $spin)] -> $sc"
done
