#!/usr/bin/env bash
# up: the ROS stack (run.sh: bridge, vlp16_bev, localization, section_annotator, e2e_driver, speed_filter) in the chroot.
# down: stop it (run.sh cleans up its children on TERM). Log: $EVAL/run/stack.log
set -uo pipefail
cd "$(dirname "$0")"; source ./env.sh
mkdir -p "$EVAL/run"
case ${1:-} in
  up)
    [[ -e $EVAL/run/stack.pid ]] && kill -0 "$(cat "$EVAL/run/stack.pid")" 2>/dev/null && { echo "stack already up"; exit 2; }
    ss -ltn | grep -q ':11311 ' && { echo "port 11311 busy: another ROS master runs on the WSL host network" >&2; exit 3; }
    setsid ./chroot.sh stack 'exec bash run.sh' > "$EVAL/run/stack.log" 2>&1 < /dev/null &
    echo $! > "$EVAL/run/stack.pid"; echo "stack pid $(cat "$EVAL/run/stack.pid"), log $EVAL/run/stack.log" ;;
  down)
    [[ -e $EVAL/run/stack.pid ]] || exit 0
    w=$(cat "$EVAL/run/stack.pid")
    # setsid made $w a session leader; everything run.sh started in the chroot (own pid namespace) keeps that session
    pkill -INT -s "$w" 2>/dev/null
    for i in $(seq 1 30); do pgrep -s "$w" > /dev/null || break; sleep 0.5; done
    pkill -KILL -s "$w" 2>/dev/null; mv "$EVAL/run/stack.pid" "$EVAL/run/stack.pid.old"; echo "stack down" ;;
  *) echo "usage: stack.sh up|down" >&2; exit 1 ;;
esac
