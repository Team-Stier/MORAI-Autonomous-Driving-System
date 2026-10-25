#!/usr/bin/env bash
# Bag-in-the-loop wiring test (inside the chroot, from the workspace root; the model server must already listen):
# roscore with /use_sim_time, the stack's nodes except the bridge, a --clock replay of one collected episode
# (cameras from the *_10hz topics, GPS, IMU, competition_status), its VLP16 packets re-sent over UDP, latency_probe.
# Usage: bash src/e2e_driver/test/bag_loop.sh START_S DURATION_S OUT_JSON BAG [BAG ...]
set -u
START=$1 DUR=$2 OUT=$3; shift 3
source devel/setup.bash
roscore > /tmp/roscore.log 2>&1 &
for i in $(seq 1 100); do rosnode list > /dev/null 2>&1 && break; sleep 0.1; done
rosparam set /use_sim_time true
for p in vlp16_bev localization section_annotator e2e_driver speed_filter; do
  bash src/$p/launch.sh > /tmp/$p.log 2>&1 &
done
sleep 6
rosrun e2e_driver latency_probe.py _out:=$OUT > /tmp/probe.log 2>&1 &
probe=$!
sleep 1
python3 src/e2e_driver/test/udp_replay.py --start $START --duration $DUR "$@" > /tmp/udp_replay.log 2>&1 &
rosbag play --clock -q -s $START -u $DUR "$@" \
  /image/front/compressed_10hz:=/image/front/compressed /image/left/compressed_10hz:=/image/left/compressed \
  /image/right/compressed_10hz:=/image/right/compressed /image/camera_7/compressed_10hz:=/image/camera_7/compressed \
  --topics /image/front/compressed_10hz /image/left/compressed_10hz /image/right/compressed_10hz \
  /image/camera_7/compressed_10hz /sensors/gps/fix /sensors/imu/data /vehicle/competition_status
sleep 2
kill -INT $probe; sleep 4
for f in /tmp/vlp16_bev.log /tmp/localization.log /tmp/e2e_driver.log /tmp/speed_filter.log /tmp/udp_replay.log /tmp/probe.log; do
  echo "== $f"; grep -v "^$" $f | tail -4
done
cat $OUT
