#!/usr/bin/env bash
set -u

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="$ROOT_DIR/ros_logs"
LOG_FILE="$LOG_DIR/verify_rgbd_pipeline_$STAMP.log"

mkdir -p "$LOG_DIR"

run_step() {
  local title="$1"
  shift
  {
    echo
    echo "===== $title ====="
    echo "\$ $*"
  } | tee -a "$LOG_FILE"
  timeout 12s "$@" >>"$LOG_FILE" 2>&1
  local rc=$?
  echo "[exit_code=$rc]" | tee -a "$LOG_FILE"
}

cd "$ROOT_DIR" || exit 1
source install/setup.bash
export ROS_LOG_DIR="$LOG_DIR"

{
  echo "RGB-D pipeline verification"
  echo "time=$STAMP"
  echo "root=$ROOT_DIR"
  echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID-}"
  echo "ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY-}"
} | tee "$LOG_FILE"

run_step "ROS topic list" ros2 topic list
run_step "ROS node list" ros2 node list
run_step "Topic hz /camera/points" ros2 topic hz /camera/points
run_step "Topic hz /scan" ros2 topic hz /scan
run_step "TF base_link <- camera optical frame" ros2 run tf2_ros tf2_echo base_link home_bot/camera_link/rgbd_camera
run_step "TF odom <- base_link" ros2 run tf2_ros tf2_echo odom base_link
run_step "Inspect transformed point cloud" "$ROOT_DIR/tools/inspect_cloud.py"
run_step "Inspect generated scan" "$ROOT_DIR/tools/inspect_scan.py"

echo | tee -a "$LOG_FILE"
echo "Wrote $LOG_FILE" | tee -a "$LOG_FILE"
