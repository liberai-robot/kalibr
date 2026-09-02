#!/usr/bin/env bash
# IMU-相机联合标定脚本（kalibr_calibrate_imu_camera）
#
# 参考:
#   rosrun kalibr kalibr_calibrate_imu_camera --target ... --imu ... \
#     --imu-models calibrated --cams ... --bag ...
#
# 用法（容器内）:
#   bash camera/script/calibrate_imu_camera.sh <bag路径> <camchain路径> [bag-freq] [timeoffset-padding]
#
# 参数:
#   <bag路径>      含图像+IMU 的 ROS bag（四路图像 + /head_ring/imu）
#   <camchain路径> 相机标定结果 camchain yaml（kalibr_calibrate_cameras 输出）
#   [bag-freq]     特征提取频率 [Hz]，默认 30
#   [timeoffset-padding] 相机-IMU 时延搜索范围/样条 buffer 余量 [s]，默认 0.1

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CAMERA_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

# 文件夹内默认值
TARGET="${CAMERA_DIR}/config/charuco_target.yaml"   # 标定板（ChArUco）
IMU="${CAMERA_DIR}/config/imu.yaml"                 # IMU 噪声参数 + rostopic
IMU_MODELS=( calibrated )                    # IMU 模型

BAG="${1:?用法: calibrate_imu_camera.sh <bag路径> <camchain路径> [bag-freq] [timeoffset-padding]}"
CAM="${2:?用法: calibrate_imu_camera.sh <bag路径> <camchain路径> [bag-freq] [timeoffset-padding]}"
BAG_FREQ="${3:-30}"
TIME_PADDING="${4:-0.1}"

source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash

echo "== kalibr IMU-相机联合标定 =="
echo "  target    : ${TARGET}"
echo "  imu       : ${IMU}"
echo "  imu-models: ${IMU_MODELS[*]}"
echo "  cams      : ${CAM}"
echo "  bag       : ${BAG}"
echo "  freq      : ${BAG_FREQ} Hz"
echo "  timeoffset-padding: ${TIME_PADDING} s"
echo

rosrun kalibr kalibr_calibrate_imu_camera \
  --target "${TARGET}" \
  --imu "${IMU}" \
  --imu-models "${IMU_MODELS[@]}" \
  --cams "${CAM}" \
  --bag "${BAG}" \
  --bag-freq "${BAG_FREQ}" \
  --timeoffset-padding "${TIME_PADDING}"

echo
echo "标定完成。输出: camchain-imucam-*.yaml / results-imucam-*.txt / report-imucam-*.pdf"
