# CLAUDE.md

本文件为 Claude Code 在此仓库中工作时提供指引。

## 项目简介

Kalibr —— 相机 / 惯性（IMU）标定工具箱（ROS 包），用于多相机、相机-IMU、IMU-IMU 及卷帘快门相机的内参 / 外参标定。

## 重要：可执行程序必须在 Docker 容器内运行

本仓库编译出的所有 kalibr 可执行程序 **必须** 在名为 `beautiful_wozniak` 的 Docker 容器内运行，**不要** 在宿主机上直接运行。

- 容器名：`beautiful_wozniak`
- 镜像：`kalibr:latest`（Ubuntu 20.04 + ROS Noetic）
- 容器内工作空间：`/catkin_ws`
- 容器内源码位置：`/catkin_ws/src/kalibr`
- 编译产物（可执行程序）位于容器内 `/catkin_ws/devel/lib/kalibr/`

可执行程序（`kalibr_*` 工具）包括：

```
kalibr_calibrate_cameras     # 多相机标定
kalibr_calibrate_imu_camera  # 相机-IMU 标定
kalibr_calibrate_rs_cameras  # 卷帘快门相机标定
kalibr_create_target_pdf     # 生成标定板 PDF
kalibr_bagcreater            # 由图片/IMU 数据生成 ROS bag
kalibr_bagextractor          # 从 ROS bag 提取数据
kalibr_camera_focus
kalibr_camera_validator
kalibr_visualize_calibration
kalibr_visualize_distortion
```

## 如何运行

使用 `docker exec` 进入容器运行工具。容器入口只在 `docker run` 时执行，`docker exec` 不会自动 source ROS 环境，因此每次需先 source 环境：

```bash
docker exec beautiful_wozniak bash -lc \
  'source /opt/ros/noetic/setup.bash && source /catkin_ws/devel/setup.bash && rosrun kalibr kalibr_calibrate_cameras --help'
```

将 `kalibr_calibrate_cameras` 替换为需要运行的任一 `kalibr_*` 工具即可。

## 环境说明

- ROS 发行版：Noetic（Ubuntu 20.04）。
- 容器入口会设置 `KALIBR_MANUAL_FOCAL_LENGTH_INIT=1`（标定初始化失败时允许手动指定焦距）。
- **宿主机目录未挂载进容器**：宿主机上的 `/home/ysc/workspace/kalibr` 是用于构建镜像的源码，镜像构建时通过 `ADD . $WORKSPACE/src/kalibr` 拷贝进镜像，两者是独立副本。因此：
  - 在宿主机上修改源码，需要**重新构建镜像**（`docker build -f Dockerfile_ros1_20_04 -t kalibr .`）后重建容器才能生效。
  - 标定数据（ROS bag、标定板 YAML 等）需要 `docker cp` 拷贝进容器，或显式挂载后使用。
- 部分工具（可视化、GUI）需要 X 显示，当前容器未转发显示，运行 GUI 类工具时可能报 `GDK-IS_DISPLAY` 错误；纯命令行工具可正常使用。

## 构建镜像

源码来自当前目录（`Dockerfile_ros1_20_04` 对应 ROS Noetic / Ubuntu 20.04）：

```bash
docker build -f Dockerfile_ros1_20_04 -t kalibr .
```

## 常见命令速查

```bash
# 查看某工具用法
docker exec beautiful_wozniak bash -lc \
  'source /opt/ros/noetic/setup.bash && source /catkin_ws/devel/setup.bash && rosrun kalibr kalibr_calibrate_imu_camera --help'

# 生成标定板（AprilGrid）
docker exec beautiful_wozniak bash -lc \
  'source /opt/ros/noetic/setup.bash && source /catkin_ws/devel/setup.bash && rosrun kalibr kalibr_create_target_pdf --type apriltag --nx 6 --ny 6 --tsize 0.088 --tspace 0.3'
```
