#!/usr/bin/env bash
# 录制相机标定数据 -> camera/record/cameras_<时间戳>.bag
#
# 录制四路相机图像 + IMU，供 kalibr_calibrate_cameras / kalibr_calibrate_imu_camera 使用。
# 录制中按 Ctrl+C 停止，bag 自动落盘到本目录上一级的 record/ 子目录。
#
# 用法（容器内）:
#   bash camera/script/record_cameras.sh
# 或先 source ROS 后直接执行:
#   camera/script/record_cameras.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# script/ 上一级即 camera/，bag 放到 camera/record/
RECORD_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)/record"
mkdir -p "${RECORD_DIR}"

STAMP="$(date +%Y%m%d_%H%M%S)"
BAG="${RECORD_DIR}/cameras_${STAMP}.bag"

# 标定数据 topic：四路相机图像 + IMU。
# 仅做相机内/外参标定（kalibr_calibrate_cameras）时可删掉 imu 一行。
TOPICS=(
  /head_ring/cam_a/image
  /head_ring/cam_b/image
  /head_ring/cam_c/image
  /head_ring/cam_d/image
  /head_ring/imu
)

echo "== 录制相机标定数据 =="
echo "  输出 : ${BAG}"
echo "  topic: ${TOPICS[*]}"
echo "  停止 : Ctrl+C"
echo

source /opt/ros/noetic/setup.bash

rosbag record -O "${BAG}" "${TOPICS[@]}"

echo
echo "录制完成: ${BAG}"
