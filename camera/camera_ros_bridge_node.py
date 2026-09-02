#!/usr/bin/env python3
"""OAK-FFC 4 目头环 -> ROS 桥接节点 (rospy + zenoh)。

不再直接访问 OAK 设备 (避开 depthai 2.x/3.x 版本冲突): 从宿主机 camera_node
(zenoh 发布) 订阅图像/IMU 消息, 解码后转发为 ROS 标准 topic:
  {prefix}/cam_a/image ... {prefix}/cam_d/image   sensor_msgs/Image  (mono8 / bgr8)
  {prefix}/imu                                   sensor_msgs/Imu

线上消息格式 (与 head_ring/messages.py 保持一致):
  图像: payload = 原始帧 (NV12/I420/MONO8/MJPEG/BGR8), attachment = FrameMeta 定长 struct
  IMU : payload = ImuSample 定长 struct

时间戳: ts_ns (设备时钟) + 由图像消息 pub_ns - ts_ns 中位数估算的偏移 -> ROS wall-clock。
四路相机与 IMU 共用同一设备时钟域, 相对时序保留, 满足 kalibr 相机-IMU 同步标定要求
(kalibr 仍会用 --time-calibration 进一步估计相机-IMU 时延)。

用法:
  source /opt/ros/noetic/setup.bash
  python camera_ros_bridge_node.py --config camera/config/head_ring.yaml
"""

import argparse
import json
import struct
import threading
import time

import cv2
import numpy as np
import rospy
import yaml
import zenoh
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, Imu

# ---------------------------------------------------------------------------
# 线上消息格式 (与 head_ring/messages.py 保持一致)
# ---------------------------------------------------------------------------
_FRAME_FMT = "<4sBBBxIIQQQQII"
_FRAME_SIZE = struct.calcsize(_FRAME_FMT)
_IMU_FMT = "<4sBxxxQQ3f3f4ff"
_IMU_SIZE = struct.calcsize(_IMU_FMT)
_FRAME_MAGIC = b"HRF1"
_IMU_MAGIC = b"HRI1"

ENC_NV12 = 0
ENC_BGR8 = 1
ENC_MONO8 = 2
ENC_MJPEG = 3
ENC_I420 = 4


def _unpack_frame_meta(data: bytes):
    """attachment 字节 -> (enc, cam_index, width, height, group_seq, frame_seq,
    ts_ns, pub_ns, exposure_us, iso)。"""
    (magic, ver, enc, idx, w, h, gseq, fseq, ts, pub, exp, iso) = \
        struct.unpack(_FRAME_FMT, data[:_FRAME_SIZE])
    if magic != _FRAME_MAGIC:
        raise ValueError(f"bad frame magic: {magic!r}")
    return enc, idx, w, h, gseq, fseq, ts, pub, exp, iso


def _unpack_imu_sample(data: bytes):
    """payload 字节 -> (seq, ts_ns, accel, gyro, quat, rot_accuracy)。"""
    vals = struct.unpack(_IMU_FMT, data[:_IMU_SIZE])
    magic, ver, seq, ts = vals[0], vals[1], vals[2], vals[3]
    if magic != _IMU_MAGIC:
        raise ValueError(f"bad imu magic: {magic!r}")
    return seq, ts, vals[4:7], vals[7:10], vals[10:14], vals[14]


def _decode_frame(enc: int, payload: bytes, width: int, height: int) -> np.ndarray:
    """把原始帧解码成 OpenCV 图像: 灰度 -> 单通道, 彩色/压缩 -> BGR 三通道。"""
    if enc == ENC_MONO8:
        return np.frombuffer(payload, dtype=np.uint8).reshape(height, width)
    if enc == ENC_BGR8:
        return np.frombuffer(payload, dtype=np.uint8).reshape(height, width, 3)
    if enc == ENC_I420:
        yuv = np.frombuffer(payload, dtype=np.uint8).reshape(height * 3 // 2, width)
        return cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_I420)
    if enc == ENC_NV12:
        yuv = np.frombuffer(payload, dtype=np.uint8).reshape(height * 3 // 2, width)
        return cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_NV12)
    if enc == ENC_MJPEG:
        return cv2.imdecode(np.frombuffer(payload, dtype=np.uint8), cv2.IMREAD_COLOR)
    raise ValueError(f"未知编码: {enc}")


# ---------------------------------------------------------------------------
# 设备时钟 -> ROS 时间偏移估算
# ---------------------------------------------------------------------------
class ClockOffset:
    """由图像消息的 pub_ns - ts_ns 中位数估算设备时钟 -> 宿主机时钟的偏移 (ns)。

    IMU 消息不含 pub_ns, 但相机与 IMU 共用同一设备时钟域, 用图像消息估出的偏移
    统一换算即可; 四路相对时序精确保留。
    """

    def __init__(self, sample_count: int = 30):
        self._lock = threading.Lock()
        self._buf = []
        self._offset = None
        self._sample_count = sample_count

    def observe(self, ts_ns: int, pub_ns: int):
        with self._lock:
            self._buf.append(pub_ns - ts_ns)
            if len(self._buf) >= self._sample_count:
                self._buf.sort()
                self._offset = self._buf[len(self._buf) // 2]
                # 保留最近几个继续滑动跟踪 (设备时钟缓慢漂移)
                self._buf = self._buf[-5:]

    def to_ros_time(self, ts_ns: int) -> rospy.Time:
        with self._lock:
            offset = self._offset
        if offset is None:
            return rospy.Time.from_sec(ts_ns * 1e-9)  # 偏移未就绪, 暂用设备时钟
        return rospy.Time.from_sec((ts_ns + offset) * 1e-9)


# ---------------------------------------------------------------------------
# 配置 / zenoh session
# ---------------------------------------------------------------------------
def load_config(path):
    """读取 head_ring.yaml, 只取 bridge 需要的字段: prefix / sockets / zenoh 端点。"""
    with open(path) as f:
        cfg = yaml.safe_load(f) or {}

    zen = cfg.get("zenoh", {})
    prefix = zen.get("prefix") or "head_ring"
    sockets = (cfg.get("camera", {}).get("sockets")
               or ["CAM_A", "CAM_B", "CAM_C", "CAM_D"])
    return {
        "prefix": prefix,
        "sockets": sockets,
        "connect": zen.get("connect", []),
        "listen": zen.get("listen", []),
    }


def open_session(cfg: dict) -> zenoh.Session:
    conf = zenoh.Config()
    conf.insert_json5("transport/shared_memory/enabled", "true")
    if cfg.get("connect"):
        conf.insert_json5("connect/endpoints", json.dumps(list(cfg["connect"])))
    if cfg.get("listen"):
        conf.insert_json5("listen/endpoints", json.dumps(list(cfg["listen"])))
    return zenoh.open(conf)


# ---------------------------------------------------------------------------
# 主流程
# ---------------------------------------------------------------------------
class Stats:
    def __init__(self, cam_names):
        self.lock = threading.Lock()
        self.frames = {n: 0 for n in cam_names}
        self.imu = 0
        self.t0 = time.monotonic()

    def bump_frame(self, name):
        with self.lock:
            self.frames[name] += 1

    def bump_imu(self):
        with self.lock:
            self.imu += 1

    def report(self):
        with self.lock:
            dt = time.monotonic() - self.t0
            if dt <= 0:
                return
            per_cam = " ".join(f"{n}:{c / dt:.1f}Hz" for n, c in self.frames.items())
            rospy.loginfo("bridge %s  imu:%.0fHz", per_cam, self.imu / dt)
            self.frames = {n: 0 for n in self.frames}
            self.imu = 0
            self.t0 = time.monotonic()


def _stats_loop(stats: Stats, stop: threading.Event):
    while not stop.is_set():
        time.sleep(5)
        stats.report()


def run(cfg: dict):
    prefix = cfg["prefix"]
    cam_names = [s.lower() for s in cfg["sockets"]]  # CAM_A -> cam_a

    rospy.init_node("camera_ros_bridge_node", anonymous=False)

    bridge = CvBridge()
    offset = ClockOffset()
    stats = Stats(cam_names)

    image_pubs = {
        name: rospy.Publisher(f"/{prefix}/{name}/image", Image, queue_size=4)
        for name in cam_names
    }
    imu_pub = rospy.Publisher(f"/{prefix}/imu", Imu, queue_size=100)
    frame_ids = {name: f"{name}_optical" for name in cam_names}
    rospy.loginfo("桥接发布: /%s/cam_x/image (Image), /%s/imu (Imu)", prefix, prefix)

    session = open_session(cfg)

    def on_image(sample, name):
        try:
            payload = sample.payload.to_bytes()
            att = sample.attachment
            if att is None:
                return
            enc, idx, w, h, gseq, fseq, ts_ns, pub_ns, exp, iso = \
                _unpack_frame_meta(att.to_bytes())
            offset.observe(ts_ns, pub_ns)
            img = _decode_frame(enc, payload, w, h)
        except Exception as e:
            rospy.logwarn_throttle(10, "图像处理失败 %s: %s", name, e)
            return

        encoding = "mono8" if img.ndim == 2 else "bgr8"
        msg = bridge.cv2_to_imgmsg(img, encoding=encoding)
        msg.header.stamp = offset.to_ros_time(ts_ns)
        msg.header.frame_id = frame_ids[name]
        image_pubs[name].publish(msg)
        stats.bump_frame(name)

    def on_imu(sample):
        try:
            seq, ts_ns, accel, gyro, quat, rot_acc = \
                _unpack_imu_sample(sample.payload.to_bytes())
        except Exception as e:
            rospy.logwarn_throttle(10, "IMU 处理失败: %s", e)
            return

        msg = Imu()
        msg.header.stamp = offset.to_ros_time(ts_ns)
        msg.header.frame_id = "imu"

        qx, qy, qz, qw = quat
        if not (qx == 0.0 and qy == 0.0 and qz == 0.0 and qw == 0.0):
            # BNO086 旋转向量 (i,j,k,real) -> ROS 四元数 (x,y,z,w); 无旋转向量时保持
            # 默认 (0,0,0,0) 且协方差 -1 表示未知
            msg.orientation.x = qx
            msg.orientation.y = qy
            msg.orientation.z = qz
            msg.orientation.w = qw
        msg.orientation_covariance[0] = -1.0

        msg.angular_velocity.x = gyro[0]      # rad/s
        msg.angular_velocity.y = gyro[1]
        msg.angular_velocity.z = gyro[2]
        msg.linear_acceleration.x = accel[0]  # m/s^2
        msg.linear_acceleration.y = accel[1]
        msg.linear_acceleration.z = accel[2]
        # kalibr 从 imu yaml 读噪声参数, 消息内协方差填 -1 (未知)
        msg.angular_velocity_covariance[0] = -1.0
        msg.linear_acceleration_covariance[0] = -1.0

        imu_pub.publish(msg)
        stats.bump_imu()

    for name in cam_names:
        session.declare_subscriber(f"{prefix}/{name}/image", lambda s, n=name: on_image(s, n))
    session.declare_subscriber(f"{prefix}/imu", on_imu)
    rospy.loginfo("订阅 zenoh: %s/cam_x/image, %s/imu", prefix, prefix)

    stop = threading.Event()
    stats_thread = threading.Thread(target=_stats_loop, args=(stats, stop), daemon=True)
    stats_thread.start()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        session.close()


def main():
    parser = argparse.ArgumentParser(description="OAK-FFC 4 目头环 -> ROS 桥接节点 (zenoh 订阅)")
    parser.add_argument("--config", default=None, help="YAML 配置文件路径 (config/head_ring.yaml)")
    args = parser.parse_args()

    if not args.config:
        parser.error("缺少 --config 参数: 请指定 config/head_ring.yaml 配置文件")
    run(load_config(args.config))


if __name__ == "__main__":
    main()
