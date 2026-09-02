#!/usr/bin/env python3
"""可视化 camchain-imucam.yaml 中相机与 IMU 的相对位姿。

读取 kalibr_calibrate_imu_camera 输出的 camchain-imucam.yaml，对每路相机解析
T_cam_imu（IMU→相机变换），在 3D 图中画出 IMU 系（原点，RGB=X/Y/Z 轴）与各相机系
（位置 + 朝向），并在终端打印每路相机的平移、旋转与时延。

用法:
  python3 visualize_imu_camera.py <camchain-imucam.yaml> [--save out.png]
"""

import argparse
import os
import sys

import numpy as np
import yaml

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  (启用 3D projection)


def load_cameras(path):
    """解析 camchain yaml，返回 {cam_idx: dict}（只取 cam0/cam1/... 节点）。"""
    with open(path) as f:
        data = yaml.safe_load(f) or {}

    cams = {}
    for key, val in data.items():
        if not key.startswith("cam"):
            continue
        try:
            idx = int(key[3:])
        except ValueError:
            continue
        cams[idx] = val
    return cams


def inv_transform(T):
    """4x4 齐次变换的逆。"""
    R = T[:3, :3]
    t = T[:3, 3]
    Ti = np.eye(4)
    Ti[:3, :3] = R.T
    Ti[:3, 3] = -R.T @ t
    return Ti


def rotation_vector(R):
    """旋转矩阵 -> (单位轴, 角度[度])，与 kalibr 内部 RotationVector 一致。"""
    theta = np.arccos(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0))
    if np.sin(theta) < 1e-8:
        return np.array([0.0, 0.0, 1.0]), 0.0
    axis = np.array([
        R[2, 1] - R[1, 2],
        R[0, 2] - R[2, 0],
        R[1, 0] - R[0, 1],
    ]) / (2.0 * np.sin(theta))
    return axis, np.degrees(theta)


def draw_frame(ax, T, size, label, color):
    """在 3D 图中画一个坐标系：原点 + RGB 三轴。T 为该系在世界系下的位姿。"""
    R = T[:3, :3]
    t = T[:3, 3]
    ax.scatter(*t, color=color, s=40, depthshade=False)
    for i, c in enumerate(["r", "g", "b"]):
        ax.quiver(*t, *(R[:, i] * size), color=c,
                  arrow_length_ratio=0.15, linewidth=1.5)
    ax.text(*(t + size * 0.2), label, color=color, fontsize=9)


def main():
    parser = argparse.ArgumentParser(description="可视化 camchain-imucam.yaml 的相机-IMU 相对位姿")
    parser.add_argument("chain", help="camchain-imucam.yaml 路径")
    parser.add_argument("--axis-size", type=float, default=0.03,
                        help="坐标轴长度 [m]（默认 %(default)s）")
    parser.add_argument("--save", metavar="PATH", default=None,
                        help="保存图片到 PATH（不弹窗，用于无显示环境）")
    args = parser.parse_args()

    cams = load_cameras(args.chain)
    if not cams:
        sys.exit(f"{args.chain} 中未找到 cam0/cam1/... 节点")

    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection="3d")

    # IMU 系（原点）
    ax.scatter(0, 0, 0, color="k", s=80, depthshade=False)
    ax.text(0, 0, args.axis_size * 0.35, "imu", color="k", fontsize=11)
    for i, c in enumerate(["r", "g", "b"]):
        ax.quiver(0, 0, 0, *(np.eye(3)[:, i] * args.axis_size),
                  color=c, arrow_length_ratio=0.15, linewidth=2)

    colors = plt.cm.tab10(np.linspace(0, 1, 10))
    print("相机-IMU 相对位姿（相机在 IMU 系下的位姿 = inv(T_cam_imu)）:")
    for idx in sorted(cams):
        cam = cams[idx]
        T_cam_imu = np.asarray(cam.get("T_cam_imu"), dtype=float)
        if T_cam_imu.shape != (4, 4):
            print(f"  cam{idx}: 缺少有效的 T_cam_imu，跳过")
            continue

        # 相机在 IMU 系下的位姿 = inv(T_cam_imu)
        T_imu_cam = inv_transform(T_cam_imu)
        color = colors[idx % len(colors)]
        draw_frame(ax, T_imu_cam, args.axis_size, f"cam{idx}", color)

        t = T_imu_cam[:3, 3]
        axis, angle = rotation_vector(T_imu_cam[:3, :3])
        ts = cam.get("timeshift_cam_imu")
        ts_str = "N/A" if ts is None else f"{ts * 1000.0:+.2f} ms"
        print(f"  cam{idx}: pos=[{t[0]:+.4f}, {t[1]:+.4f}, {t[2]:+.4f}] m | "
              f"rot {angle:7.2f}° @ [{axis[0]:+.3f}, {axis[1]:+.3f}, {axis[2]:+.3f}] | "
              f"timeshift={ts_str}")

    s = args.axis_size * 2.5
    ax.set_xlim(-s, s)
    ax.set_ylim(-s, s)
    ax.set_zlim(-s, s)
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")
    ax.set_title(os.path.basename(args.chain))
    plt.tight_layout()

    if args.save:
        plt.savefig(args.save, dpi=150)
        print(f"\n已保存: {args.save}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
