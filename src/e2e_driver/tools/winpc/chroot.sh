#!/usr/bin/env bash
# Run a command in the Noetic chroot over a private overlay (upper: $EVAL/root/<name>), this workspace at
# /workspaces/MORAI-Autonomous-Driving-System, D:/share at /mnt, $EVAL/data at /data. Host network (ROS master 127.0.0.1:11311), own pid ns.
# Usage: chroot.sh <name> <bash command>
set -euo pipefail
source "$(dirname "$0")/env.sh"
name=$1; shift
d=$EVAL/root/$name
mkdir -p "$d"/{upper,work,merged}
exec unshare --user --map-root-user --mount --pid --fork bash -c '
  set -eu; d=$1; lower=$2; ws=$3; share=$4; cmd=$5; data=$6
  mount --make-rprivate /
  mount -t overlay overlay -o "lowerdir=$lower,upperdir=$d/upper,workdir=$d/work,userxattr" "$d/merged"
  mount --bind "$ws" "$d/merged/workspaces/MORAI-Autonomous-Driving-System"
  mount --bind "$share" "$d/merged/mnt"
  mkdir -p "$d/merged/data" "$data"; mount --bind "$data" "$d/merged/data"           # outputs visible outside: $EVAL/data
  mount -t proc proc "$d/merged/proc"
  for n in null zero random urandom tty; do touch "$d/merged/dev/$n"; mount --bind "/dev/$n" "$d/merged/dev/$n"; done
  mkdir -p "$d/merged/dev/shm"; mount -t tmpfs tmpfs "$d/merged/dev/shm"; mount -t tmpfs tmpfs "$d/merged/run"
  exec chroot "$d/merged" /usr/bin/env -i HOME=/root PATH=/usr/sbin:/usr/bin:/sbin:/bin TERM=dumb \
    ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 EVAL_CMD="$cmd" \
    /bin/bash -c "mkdir -p /root/.ros; source /opt/ros/noetic/setup.bash; cd /workspaces/MORAI-Autonomous-Driving-System; eval \"\$EVAL_CMD\""
' bash "$d" "$LOWER" "$WS" "$SHARE" "$*" "$EVAL/data"
