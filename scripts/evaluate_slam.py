#!/usr/bin/env python3
"""Evaluate SLAM correction vs raw VIO against ground truth.

Reads slam_odom.csv and slam_ground_truth.csv, computes RMSE in 3D, XY, and Z.
The corrected output is in earth/world frame; raw VIO is transformed to earth
frame using the provided earth_to_map calibration.

Usage:
    pixi run python3 scripts/evaluate_slam.py --log-dir .
    pixi run python3 scripts/evaluate_slam.py --log-dir . \
        --e2m-xyz -0.278 -1.527 0.047 --e2m-rpy 0.071 -0.116 3.003
"""
import argparse
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation


def parse_args():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--log-dir", type=Path, default=Path("."))
    parser.add_argument("--e2m-xyz", type=float, nargs=3,
                        default=[-0.133262, -1.518612, -0.155813],
                        help="earth_to_map translation (x y z)")
    parser.add_argument("--e2m-rpy", type=float, nargs=3,
                        default=[0.008312, -0.001632, -3.125066],
                        help="earth_to_map rotation (roll pitch yaw)")
    parser.add_argument("--max-time", type=float, default=None,
                        help="Evaluate only the first N seconds")
    parser.add_argument("--align", action="store_true",
                        help="Also report Umeyama SE3-aligned RMSE (fair metric for SLAM)")
    parser.add_argument("--origin-align", action="store_true",
                        help="Also report RMSE with only the first pose (origin) pinned to truth")
    return parser.parse_args()


def align_rmse(src, dst):
    """RMSE after optimal SE3 (no-scale) Umeyama alignment of src onto dst."""
    src_c = src.mean(axis=0)
    dst_c = dst.mean(axis=0)
    H = (src - src_c).T @ (dst - dst_c)
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T
    t = dst_c - R @ src_c
    src_a = (R @ src.T).T + t
    d = src_a - dst
    return {
        "3d": np.sqrt(np.mean(np.sum(d ** 2, axis=1))),
        "xy": np.sqrt(np.mean(np.sum(d[:, :2] ** 2, axis=1))),
        "z": np.sqrt(np.mean(d[:, 2] ** 2)),
    }


def origin_rmse(src_pts, src_quats, dst_pts, dst_quats):
    """RMSE after pinning only the first pose (origin) of src onto dst (drift from a known start)."""
    R = Rotation.from_quat(dst_quats[0]).as_matrix() @ Rotation.from_quat(src_quats[0]).as_matrix().T
    t = dst_pts[0] - R @ src_pts[0]
    d = (R @ src_pts.T).T + t - dst_pts
    return {
        "3d": np.sqrt(np.mean(np.sum(d ** 2, axis=1))),
        "xy": np.sqrt(np.mean(np.sum(d[:, :2] ** 2, axis=1))),
        "z": np.sqrt(np.mean(d[:, 2] ** 2)),
    }


def main():
    args = parse_args()

    odom = pd.read_csv(args.log_dir / "slam_odom.csv").apply(pd.to_numeric, errors="coerce").dropna()
    gt = pd.read_csv(args.log_dir / "slam_ground_truth.csv")

    gt_ts = gt["sec"] + gt["nsec"] * 1e-9
    odom_ts = odom["sec"] + odom["nsec"] * 1e-9
    gt_xyz = gt[["x", "y", "z"]].values

    R_e2m = Rotation.from_euler("xyz", args.e2m_rpy).as_matrix()
    t_e2m = np.array(args.e2m_xyz)

    gt_quat = gt[["qx", "qy", "qz", "qw"]].values

    errors = {"raw_3d": [], "cor_3d": [], "raw_xy": [], "cor_xy": [],
              "raw_z": [], "cor_z": [], "raw_rot": [], "cor_rot": []}
    cor_pts, raw_pts, gp_pts = [], [], []
    cor_quats, raw_quats, gp_quats = [], [], []

    t0 = odom_ts.iloc[0]
    for i in range(len(odom)):
        if args.max_time is not None and (odom_ts.iloc[i] - t0) > args.max_time:
            break
        idx = np.argmin(np.abs(gt_ts.values - odom_ts.iloc[i]))
        cor = np.array([odom.iloc[i].cor_x, odom.iloc[i].cor_y, odom.iloc[i].cor_z])
        raw = np.array([odom.iloc[i].raw_x, odom.iloc[i].raw_y, odom.iloc[i].raw_z])
        raw_earth = R_e2m.T @ (raw - t_e2m)
        gp = gt_xyz[idx]
        cor_pts.append(cor)
        raw_pts.append(raw)
        gp_pts.append(gp)

        errors["cor_3d"].append(np.linalg.norm(cor - gp))
        errors["raw_3d"].append(np.linalg.norm(raw_earth - gp))
        errors["cor_xy"].append(np.linalg.norm(cor[:2] - gp[:2]))
        errors["raw_xy"].append(np.linalg.norm(raw_earth[:2] - gp[:2]))
        errors["cor_z"].append(abs(cor[2] - gp[2]))
        errors["raw_z"].append(abs(raw_earth[2] - gp[2]))

        R_gt = Rotation.from_quat(gt_quat[idx])
        raw_q = [odom.iloc[i].raw_qx, odom.iloc[i].raw_qy, odom.iloc[i].raw_qz, odom.iloc[i].raw_qw]
        R_raw_odom = Rotation.from_quat(raw_q)
        R_raw_earth = Rotation.from_matrix(R_e2m.T) * R_raw_odom
        cor_q = [odom.iloc[i].cor_qx, odom.iloc[i].cor_qy, odom.iloc[i].cor_qz, odom.iloc[i].cor_qw]
        R_cor = Rotation.from_quat(cor_q)

        errors["raw_rot"].append(np.degrees((R_gt.inv() * R_raw_earth).magnitude()))
        errors["cor_rot"].append(np.degrees((R_gt.inv() * R_cor).magnitude()))
        cor_quats.append(cor_q)
        raw_quats.append(raw_q)
        gp_quats.append(gt_quat[idx])

    print(f"Samples: {len(odom)} odom, {len(gt)} GT")
    print()
    print("=== RMSE (Translation) ===")
    for name in ["3d", "xy", "z"]:
        e_raw = np.array(errors[f"raw_{name}"])
        e_cor = np.array(errors[f"cor_{name}"])
        rmse_raw = np.sqrt(np.mean(e_raw ** 2))
        rmse_cor = np.sqrt(np.mean(e_cor ** 2))
        pct = (1 - rmse_cor / rmse_raw) * 100
        print(f"{name.upper():3s}  Raw={rmse_raw:.3f}m  Corrected={rmse_cor:.3f}m"
              f"  Improvement={pct:+.1f}%")

    e_raw_rot = np.array(errors["raw_rot"])
    e_cor_rot = np.array(errors["cor_rot"])
    rmse_raw_rot = np.sqrt(np.mean(e_raw_rot ** 2))
    rmse_cor_rot = np.sqrt(np.mean(e_cor_rot ** 2))
    pct_rot = (1 - rmse_cor_rot / rmse_raw_rot) * 100
    print(f"ROT  Raw={rmse_raw_rot:.2f}deg  Corrected={rmse_cor_rot:.2f}deg"
          f"  Improvement={pct_rot:+.1f}%")

    print()
    n = len(errors["cor_3d"])
    q = n // 4
    e_raw_3d = np.array(errors["raw_3d"])
    e_cor_3d = np.array(errors["cor_3d"])
    print("=== Over time (3D) ===")
    print(f"First quarter:  raw={e_raw_3d[:q].mean():.3f}m"
          f"  cor={e_cor_3d[:q].mean():.3f}m")
    print(f"Last quarter:   raw={e_raw_3d[-q:].mean():.3f}m"
          f"  cor={e_cor_3d[-q:].mean():.3f}m")

    if args.align:
        cor_a = align_rmse(np.array(cor_pts), np.array(gp_pts))
        raw_a = align_rmse(np.array(raw_pts), np.array(gp_pts))
        print()
        print("=== RMSE (Umeyama SE3-aligned) ===")
        for name in ["3d", "xy", "z"]:
            pct = (1 - cor_a[name] / raw_a[name]) * 100
            print(f"{name.upper():3s}  Raw={raw_a[name]:.3f}m  Corrected={cor_a[name]:.3f}m"
                  f"  Improvement={pct:+.1f}%")

    if args.origin_align:
        cor_o = origin_rmse(np.array(cor_pts), cor_quats, np.array(gp_pts), gp_quats)
        raw_o = origin_rmse(np.array(raw_pts), raw_quats, np.array(gp_pts), gp_quats)
        print()
        print("=== RMSE (origin-anchored: only first pose pinned to truth) ===")
        for name in ["3d", "xy", "z"]:
            pct = (1 - cor_o[name] / raw_o[name]) * 100
            print(f"{name.upper():3s}  Raw={raw_o[name]:.3f}m  Corrected={cor_o[name]:.3f}m"
                  f"  Improvement={pct:+.1f}%")


if __name__ == "__main__":
    main()
