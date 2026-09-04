#!/usr/bin/env python3
"""由「相机标定 camchain（全 4 路）+ 相机-IMU 联合标定结果（仅 cam0/cam1）」合成完整 imu-cam 结果。

背景：cam2/cam3 在标定时难以看到标定板，因此联合标定（kalibr_calibrate_imu_camera）
只用了 cam0/cam1（对应 *-camchain-cam01.yaml 精简链）。本脚本把 cam0/cam1 已标定的
T_cam_imu，通过相机链外参 T_cn_cnm1 逐级复合到 cam2/cam3，得到四路完整的
camchain-imucam 结果文件。

复合规则（已与 kalibr 输出验证一致，T_cn_cnm1 为「相机 N-1 → 相机 N」的变换）:
    T_camN_imu = T_cn_cnm1 @ T_cam(N-1)_imu
即
    T_cam2_imu = T_2_1 @ T_1_0 @ T_cam0_imu
    T_cam3_imu = T_3_2 @ T_2_1 @ T_1_0 @ T_cam0_imu

timeshift_cam_imu：cam2/cam3 未直接标定；四路相机硬件帧同步、共享同一相机-IMU 时延，
故默认取 cam0（基准相机）的 timeshift 作为近似（可用 --timeshift 覆盖）。

依赖：numpy + pyyaml（容器内已具备）。需在容器内运行，或宿主机 `pip install numpy pyyaml`。

用法:
  python3 compose_imucam_full.py \
      --cams   record/cameras_20260904_031343-camchain.yaml \
      --imucam record/cameras_20260904_063501-camchain-imucam.yaml \
      --output record/cameras_20260904_063501-camchain-imucam-full.yaml
"""

import argparse
import os

import numpy as np
import yaml


def _cam_key(key):
    """'cam0' -> 0, 'cam1' -> 1 ...；非 cam 键返回 -1 以便过滤。"""
    s = str(key)
    if s.startswith("cam") and s[3:].isdigit():
        return int(s[3:])
    return -1


def main():
    parser = argparse.ArgumentParser(
        description="合成完整 imu-cam 标定结果（cam0/cam1 已标定 + 链外参复合 cam2/cam3）")
    parser.add_argument("--cams", required=True,
                        help="相机标定结果（全 4 路 camchain.yaml，提供 cam2/cam3 的内参与链外参）")
    parser.add_argument("--imucam", required=True,
                        help="相机-IMU 联合标定结果（仅 cam0/cam1 的 camchain-imucam.yaml）")
    parser.add_argument("--output", default=None,
                        help="输出路径（默认: 与 --imucam 同名并追加 -full 后缀）")
    parser.add_argument("--timeshift", type=float, default=None,
                        help="cam2/cam3 的 timeshift_cam_imu 覆盖值 [s]（默认取 cam0 的值）")
    args = parser.parse_args()

    with open(args.cams) as f:
        cams = yaml.safe_load(f) or {}
    with open(args.imucam) as f:
        imucam = yaml.safe_load(f) or {}

    # 相机按 cam0..camN 排序
    cam_ids = sorted((k for k in cams if _cam_key(k) >= 0), key=_cam_key)
    imu_ids = sorted((k for k in imucam if _cam_key(k) >= 0), key=_cam_key)

    if not cam_ids:
        raise SystemExit(f"{args.cams} 中未找到 cam0/cam1/... 节点")
    if not imu_ids:
        raise SystemExit(f"{args.imucam} 中未找到 cam0/cam1/... 节点")

    # 基准相机（cam0）的 timeshift 作为 cam2/cam3 的近似
    ref_cam = imu_ids[0]
    ref_timeshift = args.timeshift
    if ref_timeshift is None:
        ref_timeshift = imucam[ref_cam].get("timeshift_cam_imu")

    # 相机标定里 cam0 是参考，无 T_cn_cnm1；其余相机都有。
    # T_parent 始终跟踪「上一台相机」的 T_cam_imu，用于逐级复合。
    out = {}
    T_parent = None

    for cid in cam_ids:
        cam = cams[cid]
        entry = {}

        # 基础相机参数（内参/畸变/resolution/topic/overlaps）来自相机标定
        for key in ("cam_overlaps", "camera_model", "distortion_coeffs",
                    "distortion_model", "intrinsics", "resolution", "rostopic"):
            if key in cam:
                entry[key] = cam[key]

        # 链外参 T_cn_cnm1（cam1/cam2/cam3 有，cam0 无）
        if "T_cn_cnm1" in cam:
            entry["T_cn_cnm1"] = cam["T_cn_cnm1"]

        if cid in imucam:
            # cam0/cam1：直接采用已标定的 T_cam_imu 与 timeshift
            entry["T_cam_imu"] = imucam[cid]["T_cam_imu"]
            entry["timeshift_cam_imu"] = imucam[cid].get("timeshift_cam_imu")
            T_parent = np.asarray(imucam[cid]["T_cam_imu"], dtype=float)
        else:
            # cam2/cam3：由链外参复合
            if T_parent is None or "T_cn_cnm1" not in cam:
                raise SystemExit(f"无法复合 {cid}: 缺少前置相机 T_cam_imu 或链外参")
            T_cn = np.asarray(cam["T_cn_cnm1"], dtype=float)
            T_imu = T_cn @ T_parent
            entry["T_cam_imu"] = T_imu.tolist()
            entry["timeshift_cam_imu"] = ref_timeshift
            T_parent = T_imu

        out[cid] = entry

    # 默认输出: 与 imucam 同名并追加 -full
    #   例: .../cameras_xxx-camchain-imucam.yaml -> .../cameras_xxx-camchain-imucam-full.yaml
    output = args.output or (os.path.splitext(args.imucam)[0] + "-full.yaml")
    with open(output, "w") as f:
        yaml.safe_dump(out, f, default_flow_style=None, sort_keys=False,
                       allow_unicode=True)

    print(f"已生成完整 imu-cam 结果: {output}")
    print("各相机 timeshift_cam_imu [s]:")
    for cid in cam_ids:
        note = "" if cid in imu_ids else "  (复合, 取 cam0 近似)"
        print(f"  {cid}: {out[cid].get('timeshift_cam_imu')}{note}")


if __name__ == "__main__":
    main()
