#!/usr/bin/env python3
"""Compute earth_to_map calibration from a bag with VIO and ground truth.

Matches the first N VIO poses to ground truth by timestamp and computes
the rigid transform (SVD) from VIO odom frame to the GT (world/earth) frame.

Usage:
    pixi run python3 scripts/calibrate_earth_to_map.py \
        --bag dataset/piloted/flight-02p-ellipse/rosbag2_vio_detections \
        --n-samples 200
"""
import argparse
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry


def parse_args():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--n-samples", type=int, default=1000,
                        help="Number of samples for calibration")
    parser.add_argument("--skip", type=int, default=1000,
                        help="Skip first N VIO samples (wait for convergence)")
    parser.add_argument("--vio-topic", default="/ov_msckf/odomimu")
    parser.add_argument("--gt-topic", default="/ground_truth/odometry")
    parser.add_argument("--storage", default="mcap")
    parser.add_argument("--use-gt-pose", action="store_true",
                        help="Compute transform from paired GT+VIO full poses (not SVD)")
    return parser.parse_args()


def main():
    args = parse_args()

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(args.bag), storage_id=args.storage),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"))

    gt_pts, gt_quats, vio_pts, vio_quats = [], [], [], []
    gt_ts, vio_ts = [], []

    while reader.has_next():
        topic, data, ts = reader.read_next()
        if topic == args.gt_topic:
            msg = deserialize_message(data, Odometry)
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            gt_pts.append([p.x, p.y, p.z])
            gt_quats.append([q.x, q.y, q.z, q.w])
            gt_ts.append(t)
        elif topic == args.vio_topic:
            msg = deserialize_message(data, Odometry)
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            vio_pts.append([p.x, p.y, p.z])
            vio_quats.append([q.x, q.y, q.z, q.w])
            vio_ts.append(t)

    gt_pts = np.array(gt_pts)
    gt_quats = np.array(gt_quats)
    vio_pts = np.array(vio_pts)
    vio_quats = np.array(vio_quats)
    gt_ts = np.array(gt_ts)
    vio_ts = np.array(vio_ts)

    start = min(args.skip, len(vio_pts) - 1)
    n_calib = min(args.n_samples, len(vio_pts) - start)

    if args.use_gt_pose:
        transforms = []
        for i in range(start, start + n_calib):
            idx = np.argmin(np.abs(gt_ts - vio_ts[i]))
            R_gt = Rotation.from_quat(gt_quats[idx]).as_matrix()
            R_vio = Rotation.from_quat(vio_quats[i]).as_matrix()
            R_e2o = R_vio @ R_gt.T
            t_e2o = vio_pts[i] - R_e2o @ gt_pts[idx]
            transforms.append((R_e2o, t_e2o))

        R_avg = Rotation.from_matrix(np.array([t[0] for t in transforms])).mean().as_matrix()
        t_avg = np.mean([t[1] for t in transforms], axis=0)

        R_e2m = R_avg
        t_e2m = t_avg
        rpy = Rotation.from_matrix(R_e2m).as_euler("xyz")

        R_o2e = R_avg.T
        t_o2e = -R_avg.T @ t_avg
        vio_in_earth = (R_o2e @ vio_pts[start:start+n_calib].T).T + t_o2e
        gt_matched = []
        for i in range(start, start + n_calib):
            idx = np.argmin(np.abs(gt_ts - vio_ts[i]))
            gt_matched.append(gt_pts[idx])
        gt_matched = np.array(gt_matched)
        residual = np.sqrt(np.mean(np.sum((vio_in_earth - gt_matched) ** 2, axis=1)))
    else:
        matched_gt, matched_vio = [], []
        for i in range(start, start + n_calib):
            idx = np.argmin(np.abs(gt_ts - vio_ts[i]))
            matched_gt.append(gt_pts[idx])
            matched_vio.append(vio_pts[i])

        matched_gt = np.array(matched_gt)
        matched_vio = np.array(matched_vio)

        vio_c = matched_vio.mean(axis=0)
        gt_c = matched_gt.mean(axis=0)
        H = (matched_vio - vio_c).T @ (matched_gt - gt_c)
        U, _, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = Vt.T @ U.T
        t = gt_c - R @ vio_c

        R_e2m = R.T
        t_e2m = -R.T @ t
        rpy = Rotation.from_matrix(R_e2m).as_euler("xyz")

        vio_in_earth = (R @ matched_vio.T).T + t
        residual = np.sqrt(np.mean(np.sum((vio_in_earth - matched_gt) ** 2, axis=1)))

    print(f"Calibration from {n_calib} samples, residual: {residual:.4f}m")
    print()
    print("    earth_to_map:")
    print(f"      x: {t_e2m[0]:.6f}")
    print(f"      y: {t_e2m[1]:.6f}")
    print(f"      z: {t_e2m[2]:.6f}")
    print(f"      roll: {rpy[0]:.6f}")
    print(f"      pitch: {rpy[1]:.6f}")
    print(f"      yaw: {rpy[2]:.6f}")


if __name__ == "__main__":
    main()
