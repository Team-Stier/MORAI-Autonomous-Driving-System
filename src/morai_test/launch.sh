#!/usr/bin/env bash
set -e
workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source /opt/ros/noetic/setup.bash
if [[ ! -x "$workspace_dir/devel/lib/morai_test/object_info_objects_node.py" ]]; then
  (cd "$workspace_dir" && catkin_make --pkg morai_test -j2)
fi
source "$workspace_dir/devel/setup.bash"
exec rosrun morai_test object_info_objects_node.py "$@"
