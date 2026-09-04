#!/usr/bin/env python3
"""ChArUco 在线检测节点 (rospy + aslam_cameras_charuco 绑定)。

订阅 camera_ros_bridge_node.py 发布的图像 topic（默认 /head_ring/cam_0/image ~
cam_3/image），对每一帧调用 aslam_cameras_charuco 的 GridCalibrationTargetCharuco
做 ChArUco 角点检测，把多路画面拼接成一幅大图用 imshow 实时显示，并在每个检出的
角点旁标注角点 id，用于标定前预览检测效果、确认标定板在四路相机中都能稳定被检出。

检测直接复用 kalibr 的 C++ 实现（GridCalibrationTargetCharuco::computeObservation），
保证在线预览与离线标定用的是同一套算法。

依赖:
  - 容器内需已编译 aslam_cameras_charuco 包（含 computeObservation 的 Python 绑定）
  - 运行前 source devel: source /catkin_ws/devel/setup.bash

用法:
  source /opt/ros/noetic/setup.bash && source /catkin_ws/devel/setup.bash
  python3 charuco_online_detector_node.py \
      --target config/charuco_target.yaml \
      --topics /head_ring/cam_0/image /head_ring/cam_1/image \
               /head_ring/cam_2/image /head_ring/cam_3/image
"""

import argparse

import cv2
import numpy as np
import rospy
import yaml
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

import aslam_cameras_charuco as acv_charuco


def load_target(path):
    """读取标定板 YAML（kalibr 的 charuco_target.yaml 格式），返回构造参数元组。"""
    with open(path) as f:
        cfg = yaml.safe_load(f) or {}

    squares_x = int(cfg["squaresX"])
    squares_y = int(cfg["squaresY"])
    square_len = float(cfg["squareLength"])
    marker_len = float(cfg["markerLength"])
    dictionary = cfg["dictionary"]

    if isinstance(dictionary, str):
        dictionary = acv_charuco.ARUCO_DICTIONARIES[dictionary]

    return squares_x, squares_y, square_len, marker_len, int(dictionary)


class CharucoDetector:
    """封装 aslam_cameras_charuco 的检测，返回便于绘制的角点与观测标志。"""

    def __init__(self, target):
        self.target = target

    def detect(self, image):
        """对单帧图像检测，返回 (points, observed, n) 或 (None, None, 0)。

        points:   (size, 2) 图像角点坐标（索引 = 棋盘内角点 id）
        observed: (size,)   bool，对应角点是否被观测到
        """
        gray = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        gray = np.ascontiguousarray(gray)

        success, points, observed = self.target.computeObservation(gray)
        if not success:
            return None, None, 0

        points = np.asarray(points, dtype=np.float64)
        observed = np.asarray(observed).ravel() != 0
        return points, observed, int(observed.sum())


def _camera_name_from_topic(topic):
    """由 topic 名推导相机名: /head_ring/cam_0/image -> cam_0。"""
    parts = topic.rstrip('/').split('/')
    return parts[-2] if len(parts) >= 2 else topic.replace('/', '_')


def _grid_layout(n):
    """按接近方形的网格布局 n 幅图，返回 (rows, cols)。"""
    cols = int(np.ceil(np.sqrt(n)))
    rows = int(np.ceil(n / cols))
    return rows, cols


def build_tile(displays, tile_w):
    """把多幅 BGR 图像按网格拼成一幅大图。

    displays: [(name, img)] 已绘制好角点/文字的 BGR 图像列表。
    """
    if not displays:
        return None

    rows, cols = _grid_layout(len(displays))
    h0, w0 = displays[0][1].shape[:2]
    tile_h = max(1, int(round(tile_w * h0 / w0)))

    canvas = np.zeros((rows * tile_h, cols * tile_w, 3), dtype=np.uint8)
    for k, (_, img) in enumerate(displays):
        r, c = divmod(k, cols)
        resized = cv2.resize(img, (tile_w, tile_h))
        canvas[r * tile_h:(r + 1) * tile_h, c * tile_w:(c + 1) * tile_w] = resized
    return canvas


def run(args):
    rospy.init_node("charuco_online_detector_node", anonymous=False)
    bridge = CvBridge()

    if not acv_charuco.isCompiled:
        rospy.logfatal("aslam_cameras_charuco 未编译，请先 catkin build 并 source devel")
        return

    squares_x, squares_y, square_len, marker_len, dictionary_id = load_target(args.target)

    options = acv_charuco.CharucoOptions()
    options.doSubpixRefinement = not args.no_subpix
    options.showExtractionVideo = False  # 本节点自行 imshow，关闭 C++ 侧窗口
    options.minCornersForValidObs = args.min_corners

    target = acv_charuco.GridCalibrationTargetCharuco(
        squares_x, squares_y, square_len, marker_len, dictionary_id, options)
    detector = CharucoDetector(target)

    topics = args.topics
    names = [_camera_name_from_topic(t) for t in topics]
    cv2.namedWindow("head_ring", cv2.WINDOW_NORMAL)

    # 每个相机缓存「最新一帧 + 最新检测结果」，回调与显示循环同线程（rospy 单线程），
    # 无需加锁。
    slots = {name: {"frame": None, "det": None, "n": 0} for name in names}

    def make_callback(name):
        def cb(msg):
            try:
                slots[name]["frame"] = bridge.imgmsg_to_cv2(
                    msg, desired_encoding="passthrough")
                points, observed, n = detector.detect(slots[name]["frame"])
                slots[name]["det"] = (points, observed)
                slots[name]["n"] = n
            except Exception as e:  # noqa: BLE001 - 单帧失败不应拖垮节点
                rospy.logwarn_throttle(10, "%s 检测失败: %s", name, e)
        return cb

    for topic, name in zip(topics, names):
        rospy.Subscriber(topic, Image, make_callback(name), queue_size=1)
        rospy.loginfo("订阅图像: %s -> 相机 %s", topic, name)

    rospy.loginfo("ChArUco 在线检测启动: %dx%d 方格, 拼接显示, 按 'q' 退出",
                  squares_x, squares_y)

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        displays = []
        for name in names:
            slot = slots[name]
            frame = slot["frame"]
            if frame is None:
                continue
            disp = frame if frame.ndim == 3 else cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

            points, observed = slot["det"]
            if points is not None:
                for i in range(points.shape[0]):
                    if not observed[i]:
                        continue
                    x, y = int(round(points[i, 0])), int(round(points[i, 1]))
                    cv2.circle(disp, (x, y), 4, (0, 255, 0), 1)
                    cv2.drawMarker(disp, (x, y), (0, 255, 0), cv2.MARKER_CROSS, 6, 1)
                    cv2.putText(disp, str(i), (x + 5, y - 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
                status = f"{slot['n']} corners"
                color = (0, 255, 0)
            else:
                status = "no detection"
                color = (0, 0, 255)

            cv2.putText(disp, f"{name}: {status}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
            displays.append((name, disp))

        if displays:
            cv2.imshow("head_ring", build_tile(displays, args.tile_width))

        if cv2.waitKey(1) & 0xFF == ord('q'):
            rospy.loginfo("收到 'q'，退出")
            break
        rate.sleep()

    cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(
        description="ChArUco 在线检测节点（aslam_cameras_charuco 绑定 -> imshow 显示）")
    parser.add_argument("--target", default="config/charuco_target.yaml",
                        help="标定板 YAML（charuco_target.yaml 格式）")
    parser.add_argument("--topics", nargs="+",
                        default=[
                            "/head_ring/cam_0/image",
                            "/head_ring/cam_1/image",
                            "/head_ring/cam_2/image",
                            "/head_ring/cam_3/image",
                        ],
                        help="要订阅的图像 topic（默认四路头环）")
    parser.add_argument("--min-corners", type=int, default=4,
                        help="有效观测所需的最少 ChArUco 角点数（默认 %(default)s）")
    parser.add_argument("--no-subpix", action="store_true",
                        help="关闭角点亚像素细化")
    parser.add_argument("--tile-width", type=int, default=640,
                        help="拼接大图中每路画面的显示宽度（默认 %(default)s）")
    args = parser.parse_args()

    try:
        run(args)
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
