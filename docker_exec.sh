#!/usr/bin/env bash
# 从宿主机直接进入 kalibr 运行环境 (Docker 容器内, 已 source ROS Noetic + kalibr devel)
#
# docker exec 不会自动 source ROS 环境, 本脚本封装之:
#   - 无参数时进入交互式 shell (自动 source, 免去每次手敲 source)
#   - 带参数时在容器内执行单条命令后退出
#
# 用法:
#   ./docker_exec.sh                        # 交互式 shell (推荐)
#   ./docker_exec.sh <命令...>              # 执行单条命令
#   KALIBR_CONTAINER=xxx ./docker_exec.sh   # 指定容器名 (默认 kalibr)
#
# 示例:
#   ./docker_exec.sh
#   ./docker_exec.sh rosrun kalibr kalibr_create_target_pdf --type charuco --squares-x 8 --squares-y 6 --square-length 0.04 --marker-length 0.028 --dictionary DICT_6X6_250 /tmp/charuco.pdf
#   ./docker_exec.sh printenv ROS_DISTRO

set -euo pipefail

CONTAINER="${KALIBR_CONTAINER:-kalibr}"
ROS_SETUP='source /opt/ros/noetic/setup.bash && source /catkin_ws/devel/setup.bash'

if [ $# -eq 0 ]; then
  echo "[docker_exec] 进入容器 ${CONTAINER} (ROS + kalibr devel 已 source)，退出: exit / Ctrl-D"
  exec docker exec -it "${CONTAINER}" bash -lc "${ROS_SETUP} && exec bash"
fi

# 执行单条命令 (逐参数 shell 转义, 保留引号语义)
cmd=$(printf '%q ' "$@")
exec docker exec "${CONTAINER}" bash -lc "${ROS_SETUP} && ${cmd}"
