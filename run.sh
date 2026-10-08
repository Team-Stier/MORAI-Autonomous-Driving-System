#!/usr/bin/env bash
set -o pipefail

WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WORKSPACE" || exit 1

FAILURES=()
CHILD_PIDS=()
CHILD_NAMES=()
ALL_CHILD_PIDS=()

log() {
    printf '[run.sh] %s\n' "$*"
}

fail() {
    FAILURES+=("$1")
    printf '[run.sh] ERROR: %s\n' "$1" >&2
}

run_step() {
    local name="$1"
    shift
    log "$name"
    "$@"
    local result=$?
    if [[ $result -ne 0 ]]; then
        fail "$name (exit $result)"
    fi
    return 0
}

run_privileged() {
    if [[ "$EUID" -eq 0 ]]; then
        "$@"
    elif command -v sudo >/dev/null 2>&1; then
        sudo "$@"
    else
        return 127
    fi
}

if [[ -r /opt/ros/noetic/setup.bash ]]; then
    source /opt/ros/noetic/setup.bash || fail "source ROS Noetic environment"
else
    fail "ROS Noetic is not installed at /opt/ros/noetic"
fi
export PATH="/usr/bin:${PATH:-}"

stop_existing_ros() {
    log "Stopping existing ROS processes"
    if command -v rosnode >/dev/null 2>&1; then
        if command -v timeout >/dev/null 2>&1; then
            timeout 5s rosnode kill -a >/dev/null 2>&1 || true
        else
            rosnode kill -a >/dev/null 2>&1 || true
        fi
    fi

    local workspace_pattern
    workspace_pattern="$(printf '%s' "$WORKSPACE" | sed 's|/|[/]|g')"
    local patterns=(
        '[/]opt/ros/noetic/'
        '[/]roscore'
        '[/]rosmaster'
        '[/]roslaunch'
        '[/]rosout/rosout'
        '[/]rviz'
        "${workspace_pattern}[/]devel[/]lib[/]"
    )
    local pattern attempt alive

    for pattern in "${patterns[@]}"; do
        pkill -TERM -f "$pattern" 2>/dev/null || true
    done
    for attempt in {1..20}; do
        alive=false
        for pattern in "${patterns[@]}"; do
            if pgrep -f "$pattern" >/dev/null 2>&1; then
                alive=true
                break
            fi
        done
        [[ "$alive" == true ]] || break
        sleep 0.1
    done
    for pattern in "${patterns[@]}"; do
        pkill -KILL -f "$pattern" 2>/dev/null || true
    done
}

stop_existing_ros

needs_install=false
if ! command -v rosdep >/dev/null 2>&1 ||
   ! rosdep check --from-paths src --ignore-src --rosdistro noetic >/dev/null 2>&1 ||
   [[ ! -x /opt/ros/noetic/bin/rviz ]] ||
   ! rospack find compressed_image_transport >/dev/null 2>&1; then
    needs_install=true
fi

if [[ "$needs_install" == true ]]; then
    run_step "Refresh system package index" run_privileged apt-get update
    if command -v rosdep >/dev/null 2>&1; then
        run_step "Install workspace ROS dependencies" rosdep install \
            --from-paths src --ignore-src --rosdistro noetic -y
    else
        fail "rosdep is unavailable; cannot install workspace ROS dependencies"
    fi
    run_step "Install runtime ROS tools" run_privileged apt-get install -y \
        ros-noetic-rviz ros-noetic-compressed-image-transport
fi

pull_lfs_assets() {
    if ! command -v git-lfs >/dev/null 2>&1; then
        run_privileged apt-get install -y git-lfs || return $?
    fi
    git -C "$WORKSPACE" lfs install --local >/dev/null && git -C "$WORKSPACE" lfs pull
}

run_step "Fetch Git LFS assets" pull_lfs_assets
run_step "Build catkin workspace" catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo -j2
if [[ -r devel/setup.bash ]]; then
    source devel/setup.bash || fail "source workspace overlay"
else
    fail "workspace overlay devel/setup.bash is unavailable"
fi

start_process() {
    local name="$1"
    shift
    if ! command -v setsid >/dev/null 2>&1; then
        fail "$name: setsid is unavailable"
        return
    fi
    setsid "$@" &
    local pid=$!
    CHILD_PIDS+=("$pid")
    CHILD_NAMES+=("$name")
    ALL_CHILD_PIDS+=("$pid")
    log "Started $name (pid $pid)"
}

start_package() {
    local package="$1"
    local launch_script="$WORKSPACE/src/$package/launch.sh"
    if [[ ! -f "$launch_script" ]]; then
        fail "$package: src/$package/launch.sh is missing"
        return
    fi
    start_process "$package" bash "$launch_script"
}

start_process roscore roscore
for attempt in {1..100}; do
    rosnode list >/dev/null 2>&1 && break
    sleep 0.1
done
rosnode list >/dev/null 2>&1 || fail "ROS master did not start; package launch attempts will continue"

start_package MORAI-UDP-Bridge-Node
start_process velodyne roslaunch velodyne_pointcloud VLP16_points.launch
start_package tf_publisher
start_package lidar_leveler
start_package localization
start_package section_annotator
start_package object_detector
start_package traffic_light
start_package path_planner
start_package velocity_planner
start_package control
start_package speed_filter

stop_child_groups() {
    local signal="$1"
    local pid
    for pid in "${ALL_CHILD_PIDS[@]}"; do
        kill -"$signal" -- "-$pid" 2>/dev/null || true
    done
}

cleanup() {
    trap - EXIT INT TERM
    stop_child_groups TERM
    local attempt alive pid
    for attempt in {1..20}; do
        alive=false
        for pid in "${ALL_CHILD_PIDS[@]}"; do
            if kill -0 -- "-$pid" 2>/dev/null; then
                alive=true
                break
            fi
        done
        [[ "$alive" == true ]] || break
        sleep 0.1
    done
    stop_child_groups KILL
    wait "${ALL_CHILD_PIDS[@]}" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

report_failures() {
    local failure
    [[ ${#FAILURES[@]} -gt 0 ]] || return 0
    log "Bringup has ${#FAILURES[@]} failed step(s):"
    for failure in "${FAILURES[@]}"; do
        printf '[run.sh]   - %s\n' "$failure" >&2
    done
}
report_failures

while [[ ${#CHILD_PIDS[@]} -gt 0 ]]; do
    local_pids=()
    local_names=()
    for index in "${!CHILD_PIDS[@]}"; do
        pid="${CHILD_PIDS[$index]}"
        name="${CHILD_NAMES[$index]}"
        state="$(ps -o stat= -p "$pid" 2>/dev/null | tr -d '[:space:]')"
        if [[ -n "$state" && "$state" != *Z* ]]; then
            local_pids+=("$pid")
            local_names+=("$name")
            continue
        fi
        wait "$pid"
        result=$?
        kill -TERM -- "-$pid" 2>/dev/null || true
        sleep 0.1
        kill -KILL -- "-$pid" 2>/dev/null || true
        fail "$name exited unexpectedly (exit $result); other packages remain running"
    done
    CHILD_PIDS=("${local_pids[@]}")
    CHILD_NAMES=("${local_names[@]}")
    [[ ${#CHILD_PIDS[@]} -eq 0 ]] || sleep 1
done

report_failures
if [[ ${#FAILURES[@]} -gt 0 ]]; then
    exit 1
fi
exit 0
