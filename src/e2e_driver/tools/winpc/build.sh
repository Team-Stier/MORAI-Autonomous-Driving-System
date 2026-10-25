#!/usr/bin/env bash
# catkin build of the evaluation packages (same whitelist and flags as run.sh), in the chroot.
set -euo pipefail
cd "$(dirname "$0")"
exec ./chroot.sh build 'catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCATKIN_WHITELIST_PACKAGES="morai_msgs;interfaces;node_pipeline;morai_udp_bridge;tf_publisher;lidar_leveler;localization;section_annotator;vlp16_bev;e2e_driver;speed_filter" -j8'
