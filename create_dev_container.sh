#!/usr/bin/env bash
# 创建 kalibr 开发容器 (ROS1 / Ubuntu 20.04)
#
# 依次自动执行:
#   1. 检查镜像 kalibr:latest 是否存在, 不存在则用 Dockerfile_ros1_20_04 构建
#   2. 容器: 不存在则创建 (宿主机工作区映射到 /catkin_ws/src/kalibr, X11 显示转发,
#      USB 透传, 共享宿主机网络 --network host --ipc host);
#      已存在但未运行则自动启动
#   3. 首次创建后容器内 catkin build 编译挂载的源码 (含新增 C++ 包)
#   4. 容器内安装 eclipse-zenoh==1.7.0 (camera 桥接节点订阅 zenoh 的依赖;
#      1.8.0 起 cp38 wheel 损坏, 锁 1.7.0 兼容 Python 3.8)
#   另: 自动放行本地 X 连接 (xhost), 使 GUI/显示转发直接可用
#
# 用法:
#   ./create_dev_container.sh [容器名]
#
# 进入容器:
#   docker exec -it kalibr bash
#   # 或 KALIBR_CONTAINER=kalibr ./docker_exec.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE="kalibr:latest"
DOCKERFILE="Dockerfile_ros1_20_04"
CONTAINER="${1:-kalibr}"
CONTAINER_SRC="/catkin_ws/src/kalibr"

# ---- 前置: 自动放行本地 X 连接 (显示转发), 无 xhost / 无 DISPLAY 时跳过 ----
if command -v xhost >/dev/null 2>&1 && [ -n "${DISPLAY:-}" ]; then
  echo "[前置] 放行本地 X 连接: xhost +local:"
  xhost +local: >/dev/null 2>&1 || true
fi

# ---- 1. 镜像: 存在则跳过, 否则构建 ----
if docker image inspect "${IMAGE}" >/dev/null 2>&1; then
  echo "[1/4] 镜像 ${IMAGE} 已存在, 跳过构建"
else
  echo "[1/4] 镜像 ${IMAGE} 不存在, 开始构建 ..."
  docker build -f "${SCRIPT_DIR}/${DOCKERFILE}" -t "${IMAGE}" "${SCRIPT_DIR}"
fi

# ---- 2. 容器: 不存在则创建, 已存在则确保运行 ----
if docker container inspect "${CONTAINER}" >/dev/null 2>&1; then
  RUNNING="$(docker container inspect --format '{{.State.Running}}' "${CONTAINER}")"
  if [ "${RUNNING}" = "true" ]; then
    echo "[2/4] 容器 ${CONTAINER} 已存在且正在运行, 跳过启动"
  else
    echo "[2/4] 容器 ${CONTAINER} 已存在但未运行, 自动启动 ..."
    docker start "${CONTAINER}"
  fi
else
  echo "[2/4] 创建容器 ${CONTAINER} ..."
  docker run -dit \
    --name "${CONTAINER}" \
    -v "${SCRIPT_DIR}:${CONTAINER_SRC}" \
    -e DISPLAY="${DISPLAY:-:0}" \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v /dev/bus/usb:/dev/bus/usb \
    --ipc=host \
    --network host \
    "${IMAGE}"

  # ---- 3. 首次编译: 镜像 devel 可能落后于挂载的源码 (含新增 C++ 包), 编译一次 ----
  echo "[3/4] 容器内 catkin build (编译挂载的源码, 首次较慢)..."
  docker exec "${CONTAINER}" bash -lc 'source /opt/ros/noetic/setup.bash && cd /catkin_ws && catkin build'

  # ---- 4. 桥接节点依赖: zenoh (camera_ros_bridge_node.py 订阅宿主机采集端用) ----
  echo "[4/4] 容器内安装 eclipse-zenoh==1.7.0 ..."
  docker exec "${CONTAINER}" bash -lc 'pip install "eclipse-zenoh==1.7.0"'
fi

echo
echo "完成。"
echo "  进入容器 : docker exec -it ${CONTAINER} bash"
echo "            (或 KALIBR_CONTAINER=${CONTAINER} ./docker_exec.sh)"
echo "  源码挂载 : ${SCRIPT_DIR} -> ${CONTAINER_SRC}"
echo "  显示转发 : DISPLAY=${DISPLAY:-:0} + /tmp/.X11-unix"
echo "  USB 透传 : /dev/bus/usb -> /dev/bus/usb"
echo "  网络     : 共享宿主机网络 (--network host)"
echo "  zenoh    : eclipse-zenoh==1.7.0 已安装 (camera 桥接节点依赖)"
