# camera — OAK-FFC 4 目头环 → ROS 桥接节点

本目录提供 OAK-FFC 4 目头环（4 路相机 + IMU）到 ROS 的**桥接节点**，作为 kalibr 标定的数据采集前端。

采集端（OAK 设备 + depthai + zenoh 的原始驱动 `camera_node`）在**另一个工程** `slam_device/head_ring` 中管理，本目录只保留桥接节点与配置。

## 文件说明

| 文件 | 说明 |
|---|---|
| `camera_ros_bridge_node.py` | **ROS 桥接节点**：订阅采集端 `camera_node` 的 zenoh 消息，解码转发为 ROS topic（供 kalibr 标定用） |
| `charuco_online_detector_node.py` | **ChArUco 在线检测节点**：订阅桥接节点发布的图像 topic，用 `aslam_cameras_charuco` 绑定实时检测 ChArUco 角点并 imshow 显示（标定前预览） |
| `script/record_cameras.sh` | **录包脚本**：录制四路相机 + IMU 到 `record/cameras_<时间戳>.bag`（供 kalibr 标定） |
| `script/calibrate_cameras.sh` | **标定脚本**：`kalibr_calibrate_cameras`（ChArUco + 等距模型），bag 由参数传入，`--bag-freq` 默认 4 |
| `script/calibrate_imu_camera.sh` | **联合标定脚本**：`kalibr_calibrate_imu_camera`，bag/camchain 由参数传入，target/imu/imu-models 用文件夹内默认值 |
| `script/visualize_imu_camera.py` | **可视化脚本**：3D 绘制 camchain-imucam.yaml 中相机与 IMU 的相对位姿（输入为标定文件路径） |
| `record/` | 录制 bag 输出目录（脚本运行时自动创建） |
| `config/head_ring.yaml` | 桥接配置（`zenoh.prefix`/`camera.topics`/zenoh 端点，桥接节点读取；采集端相机参数由另一工程管理） |
| `config/imu.yaml` | IMU 噪声参数（加速度计/陀螺仪噪声密度 + 随机游走 + `rostopic`） |
| `config/charuco_target.yaml` | ChArUco 标定板几何（kalibr 标定用） |

## 1. 架构

分两段，两端依赖不同、跑在不同地方：

- **采集端（宿主机，另一工程 `slam_device/head_ring`）**：`camera_node` 直接访问 OAK 硬件，依赖 depthai v3 + zenoh 1.x，向 zenoh 发布图像/IMU。
- **桥接端（本目录，ROS 容器）**：`camera_ros_bridge_node.py` 不碰 OAK 硬件，只订阅 zenoh，依赖 ROS Noetic + zenoh 1.x。

> 为什么拆两段：设备固件是 depthai 3.x 线，而 ROS Noetic 锁 Python 3.8、只能用 depthai 2.x，两者不兼容；zenoh 1.x 同样需要 Python ≥ 3.9。因此把「读设备」留在宿主机（depthai 3.x），把「转 ROS」放进容器，中间用 zenoh 桥接。

## 2. 依赖安装（桥接端，容器内）

桥接端只需要在 ROS 容器里装 zenoh，已由仓库根的 [`create_dev_container.sh`](../create_dev_container.sh) 建容器时自动安装。手动安装：

```bash
pip install "eclipse-zenoh==1.7.0"
# 1.8.0 起 cp38 wheel 损坏 (undefined symbol: PyCMethod_New)，最高可用 1.7.0
```

采集端的 udev/depthai 安装见另一工程 `slam_device/head_ring`。

## 3. 运行

两端分别启动：

```bash
# ① 宿主机：采集端（另一工程 slam_device/head_ring，zenoh 发布，无 ROS）
python -m head_ring.camera_node --config <slam_device>/head_ring.yaml

# ② ROS 容器：桥接端（本目录，zenoh 订阅 -> ROS 发布）
source /opt/ros/noetic/setup.bash
python camera_ros_bridge_node.py --config camera/config/head_ring.yaml
```

验证消息已进 ROS：`rostopic hz /head_ring/cam_0/image`（约 30Hz）、`rostopic hz /head_ring/imu`（约 200Hz）。

## 4. 发布 topic

prefix 来自配置 `zenoh.prefix`（示例为 `head_ring`）：

| topic | 类型 | 说明 |
|---|---|---|
| `/head_ring/cam_0/image` … `/head_ring/cam_3/image` | `sensor_msgs/Image` | 四路相机图像，编码 `mono8`（灰度）/ `bgr8`（彩色），frame_id = `cam_0_optical` |
| `/head_ring/imu` | `sensor_msgs/Imu` | IMU（`angular_velocity` rad/s、`linear_acceleration` m/s²；BNO086 另含 `orientation`），frame_id = `imu` |

topic 命名与采集端 zenoh 一致（`{prefix}/cam_x/image`、`{prefix}/imu`），可直接用 `rosbag record` 录制。

## 5. 配置说明（`config/head_ring.yaml`）

桥接节点只读取 `zenoh.prefix`、`camera.topics` 与 zenoh 端点；采集端相机参数（`type`/`resolution`/`fps`/帧同步等）由另一工程 `slam_device/head_ring` 管理：

```yaml
zenoh:
  prefix: head_ring                    # zenoh key / topic 前缀（桥接节点读这里）
  connect: []                          # 跨机器指定 endpoints，例如 ["tcp/192.168.1.10:7447"]
  listen: []
camera:
  topics: [cam_0, cam_1, cam_2, cam_3]  # 四路相机 topic 名，顺序即 cam_index
```

关键点：
- `topics` 顺序即 `cam_index` 顺序，topic 为 `cam_0`…`cam_3`。
- 桥接节点按 `topics`（`cam_0`…`cam_3`）订阅 zenoh、发布 ROS；编码由消息内 `FrameMeta.encoding` 决定，自动转成 `mono8`/`bgr8`。

## 6. 时间戳说明

depthai 的 `ImgFrame.getTimestamp()`、IMU 时间戳同处**设备内部时钟域**（自设备启动起算的单调时钟），与宿主机 Unix epoch 相差一个固定偏移。

桥接节点用**订阅到的第一帧图像**的 `pub_ns - ts_ns` 固定估算该偏移（`pub_ns` 为宿主机发布时刻，`ts_ns` 为设备时间戳），之后不再更新，再对相机与 IMU 的时间戳统一加偏移换算成 `rospy.Time`（`header.stamp` 的 secs/nsecs）。四路相机与 IMU 共用同一设备时钟域，换算后彼此相对时序不变，满足 kalibr 相机-IMU 同步标定要求。

## 7. 录制 bag 供 kalibr 标定

可直接用录包脚本，自动按 `cameras_<时间戳>.bag` 命名并落盘到 `record/` 子目录：

```bash
bash camera/script/record_cameras.sh   # 录制中 Ctrl+C 停止
```

等价的手动命令（脚本内部即执行此命令）：

```bash
source /opt/ros/noetic/setup.bash
rosbag record -O record/cameras_$(date +%Y%m%d_%H%M%S).bag \
  /head_ring/cam_0/image /head_ring/cam_1/image \
  /head_ring/cam_2/image /head_ring/cam_3/image \
  /head_ring/imu
```

录制时手持头环做充分的三轴旋转/平移运动（标定第三段及剧烈运动建议把 `exposure_max_us` 设到 2000~4000 压运动模糊）。之后用 kalibr 标定：

```bash
bash camera/script/calibrate_cameras.sh record/cameras_<时间戳>.bag [bag-freq]
# bag-freq 默认 4（特征提取频率），即等价于下面这条手动命令：
rosrun kalibr kalibr_calibrate_cameras \
  --target camera/config/charuco_target.yaml \
  --bag record/cameras_<时间戳>.bag \
  --models pinhole-equi pinhole-equi pinhole-equi pinhole-equi \
  --topics /head_ring/cam_0/image /head_ring/cam_1/image \
           /head_ring/cam_2/image /head_ring/cam_3/image \
  --bag-freq 4
```

## 8. 常见问题

| 现象 | 排查 |
|---|---|
| `import zenoh` 报 `PyCMethod_New` | zenoh 1.8.0 起 cp38 wheel 损坏，降级 `pip install "eclipse-zenoh==1.7.0"` |
| 桥接节点收不到消息 | 确认宿主机采集端 `camera_node` 在跑；容器需 `--network host --ipc=host`（否则 zenoh 无法发现/共享 SHM） |

采集端（OAK 设备/udev/depthai/帧同步/IMU）相关的问题见另一工程 `slam_device/head_ring` 的文档。

## 9. ChArUco 在线检测（预览）

标定前可用 `charuco_online_detector_node.py` 实时查看四路相机对 ChArUco 标定板的检测效果，确认标定板摆放、曝光、对焦是否合适：

```bash
source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash
python charuco_online_detector_node.py \
  --target config/charuco_target.yaml \
  --topics /head_ring/cam_0/image /head_ring/cam_1/image \
           /head_ring/cam_2/image /head_ring/cam_3/image
```

四路画面拼接成一幅大图（单窗口 `head_ring`）显示，绿色文字显示检出的 ChArUco 角点数，每个角点旁标注角点 id，红色 `no detection` 表示未检出；按 `q` 退出。

| 参数 | 默认 | 说明 |
|---|---|---|
| `--target` | `config/charuco_target.yaml` | 标定板几何（与 kalibr 标定共用同一份 `charuco_target.yaml`） |
| `--topics` | 四路 `/head_ring/cam_*/image` | 要订阅的图像 topic |
| `--min-corners` | `4` | 有效观测所需最少角点数 |
| `--no-subpix` | 关 | 关闭角点亚像素细化 |
| `--tile-width` | `640` | 拼接大图中每路画面的显示宽度 |

> 检测直接调用 `aslam_cameras_charuco` 的 `GridCalibrationTargetCharuco::computeObservation`（`detectMarkers` → `interpolateCornersCharuco` → 亚像素细化），与 kalibr 离线标定同源。
>
> 需要容器内已编译 `aslam_cameras_charuco` 包（并包含 `computeObservation` 的 Python 绑定）。宿主机改动 C++ 源码后需重新构建镜像并重建容器才能生效（见项目根 `CLAUDE.md`）。

> GUI 需 X 显示转发：容器未转发显示时会报 `GDK-IS_DISPLAY` 错误；纯检测逻辑不受影响。
