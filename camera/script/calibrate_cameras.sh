#!/usr/bin/env bash
# 相机标定脚本（kalibr_calibrate_cameras，ChArUco 标定板 + 等距鱼眼模型）
#
# 参考:
#   rosrun kalibr kalibr_calibrate_cameras --target ... --models ... \
#     --topics ... --bag ... --bag-freq 4.0
#
# 用法（容器内）:
#   bash camera/script/calibrate_cameras.sh <bag路径> [bag-freq]
#
# 参数:
#   <bag路径>   要标定的 ROS bag（如 camera/record/cameras_<时间戳>.bag）
#   [bag-freq]  特征提取频率 [Hz]，默认 4

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CAMERA_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

# 标定板：camera/config/charuco_target.yaml（容器内挂载点 /catkin_ws/src/kalibr
# 对应宿主机 /home/ysc/workspace/kalibr）
TARGET="${CAMERA_DIR}/config/charuco_target.yaml"

# 四路相机均用等距鱼眼模型 pinhole-equi（与 topics 一一对应）
MODELS=( pinhole-equi pinhole-equi pinhole-equi pinhole-equi )

TOPICS=(
  /head_ring/cam_a/image
  /head_ring/cam_b/image
  /head_ring/cam_c/image
  /head_ring/cam_d/image
)

BAG="${1:?用法: calibrate_cameras.sh <bag路径> [bag-freq]}"
BAG_FREQ="${2:-4}"

source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash

# 焦距自动初始化失败时启用手动输入（注意：此变量仅作开关，其值不会被读取，
# 实际焦距仍会在标定运行时通过 stdin 交互输入，见 PinholeProjection.hpp）
export KALIBR_MANUAL_FOCAL_LENGTH_INIT=1

echo "== kalibr 相机标定 =="
echo "  target: ${TARGET}"
echo "  models: ${MODELS[*]}"
echo "  topics: ${TOPICS[*]}"
echo "  bag   : ${BAG}"
echo "  freq  : ${BAG_FREQ} Hz"
echo

rosrun kalibr kalibr_calibrate_cameras \
  --target "${TARGET}" \
  --models "${MODELS[@]}" \
  --topics "${TOPICS[@]}" \
  --bag "${BAG}" \
  --bag-freq "${BAG_FREQ}"

echo
echo "标定完成。输出: camchain-*.yaml / results-*.txt / report-*.pdf"
