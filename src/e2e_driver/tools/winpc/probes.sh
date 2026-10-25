#!/usr/bin/env bash
# measurement nodes for a live run (inside the chroot): GT receiver (judge-only Ego status, measurement only), live_probe,
# latency_probe; stop with: touch $EVAL/data/stop_$NAME
NAME=$1; source devel/setup.bash
python3 /data/collect-only-ego-status-node.py _enable_collect_only:=true > /tmp/gt.log 2>&1 &
gt=$!
sleep 2
python3 src/e2e_driver/test/live_probe.py _out:=/data/live_$NAME.json _route:=src/e2e_driver/assets/route.json > /tmp/live.log 2>&1 &
p1=$!
rosrun e2e_driver latency_probe.py _out:=/data/latency_$NAME.json > /tmp/lat.log 2>&1 &
p2=$!
until [[ -e /data/stop_$NAME ]]; do sleep 1; done
kill -INT $p1 $p2; for p in $p1 $p2; do for i in $(seq 1 60); do kill -0 $p 2>/dev/null || break; sleep 0.5; done; done
kill $gt
