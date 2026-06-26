#!/usr/bin/env python3
"""Compare SLAM-estimated gate poses to surveyed truth (position + orientation).

Truth comes from either a full-pose CSV (--true-csv, e.g. from extract_gates_gt.py)
or a config's fixed_objects (--true-config, position + yaw only). Reports per-gate
raw position/orientation error, then a Umeyama SE3 decomposition: the global offset
(the gauge SLAM cannot observe without a known map) vs the post-alignment residual
(relative map accuracy). In localization the gates are fixed, so this is ~0 by design.

Usage:
    pixi run python3 scripts/evaluate_gates.py \
        --estimated runs/flight-01p-ellipse-slam/slam_estimated_objects.csv \
        --true-csv runs/flight-01p-ellipse-slam/gates_ground_truth.csv
"""
import argparse
from pathlib import Path

import numpy as np
import pandas as pd
import yaml
from scipy.spatial.transform import Rotation


def parse_args():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--estimated", type=Path, required=True,
                        help="slam_estimated_objects.csv from a run")
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--true-csv", type=Path,
                       help="full-pose GT csv (id,type,x,y,z,qx,qy,qz,qw)")
    group.add_argument("--true-config", type=Path,
                       help="YAML whose fixed_objects holds the surveyed gates (position+yaw)")
    parser.add_argument("--odom", type=Path,
                        help="slam_odom.csv (with --gt-traj, also report origin-anchored error)")
    parser.add_argument("--gt-traj", type=Path,
                        help="slam_ground_truth.csv (start pose for origin anchoring)")
    return parser.parse_args()


def load_true_csv(path):
    df = pd.read_csv(path)
    pos = {str(r.id): np.array([r.x, r.y, r.z]) for r in df.itertuples()}
    quat = {str(r.id): np.array([r.qx, r.qy, r.qz, r.qw]) for r in df.itertuples()}
    return pos, quat


def load_true_config(path):
    with open(path) as f:
        cfg = yaml.safe_load(f)
    fixed = cfg["/**"]["ros__parameters"].get("fixed_objects", {}) or {}
    pos = {gid: np.array(d["pose"][:3]) for gid, d in fixed.items()}
    quat = {gid: Rotation.from_euler("z", d["pose"][3]).as_quat() for gid, d in fixed.items()}
    return pos, quat


def origin_transform(odom_path, gt_path):
    """Rigid transform pinning the estimated start pose onto the true start pose."""
    odom = pd.read_csv(odom_path).apply(pd.to_numeric, errors="coerce").dropna()
    gt = pd.read_csv(gt_path)
    gt_ts = (gt["sec"] + gt["nsec"] * 1e-9).values
    o_ts = odom["sec"].iloc[0] + odom["nsec"].iloc[0] * 1e-9
    g = gt.iloc[int(np.argmin(np.abs(gt_ts - o_ts)))]
    est_p = np.array([odom.iloc[0].cor_x, odom.iloc[0].cor_y, odom.iloc[0].cor_z])
    est_R = Rotation.from_quat([odom.iloc[0].cor_qx, odom.iloc[0].cor_qy,
                                odom.iloc[0].cor_qz, odom.iloc[0].cor_qw]).as_matrix()
    true_R = Rotation.from_quat([g.qx, g.qy, g.qz, g.qw]).as_matrix()
    R = true_R @ est_R.T
    t = np.array([g.x, g.y, g.z]) - R @ est_p
    return R, t


def main():
    args = parse_args()
    true_pos, true_quat = (load_true_csv(args.true_csv) if args.true_csv
                           else load_true_config(args.true_config))

    est_df = pd.read_csv(args.estimated)
    est_pos = {str(r.id): np.array([r.x, r.y, r.z]) for r in est_df.itertuples()}
    est_quat = {str(r.id): np.array([r.qx, r.qy, r.qz, r.qw]) for r in est_df.itertuples()}

    ids = sorted(set(true_pos) & set(est_pos))
    if not ids:
        print(f"No matching gate IDs. true={sorted(true_pos)} est={sorted(est_pos)}")
        return

    T = np.array([true_pos[i] for i in ids])
    E = np.array([est_pos[i] for i in ids])

    print(f"Matched {len(ids)} gates: {ids}")
    print()
    print("=== Position error (raw, no alignment) ===")
    for i, gid in enumerate(ids):
        print(f"  {gid}: {np.linalg.norm(E[i] - T[i]):.3f} m")
    print(f"  RMS: {np.sqrt(np.mean(np.sum((E - T) ** 2, axis=1))):.3f} m")

    # Umeyama SE3 (no scale) alignment of estimated onto true.
    Ec, Tc = E.mean(axis=0), T.mean(axis=0)
    H = (E - Ec).T @ (T - Tc)
    U, _, Vt = np.linalg.svd(H)
    Rm = Vt.T @ U.T
    if np.linalg.det(Rm) < 0:
        Vt[-1, :] *= -1
        Rm = Vt.T @ U.T
    R_global = Rotation.from_matrix(Rm)
    E_aligned = (Rm @ (E - Ec).T).T + Tc
    residual = np.sqrt(np.mean(np.sum((E_aligned - T) ** 2, axis=1)))
    offset = np.linalg.norm(Tc - Ec)

    print()
    print("=== Umeyama SE3 decomposition (position) ===")
    print(f"  Global offset:    translation={offset:.3f} m   "
          f"rotation={np.degrees(R_global.magnitude()):.2f} deg")
    print(f"  Relative map RMS: {residual:.3f} m  (post-alignment)")

    if args.odom and args.gt_traj:
        R_o, t_o = origin_transform(args.odom, args.gt_traj)
        E_origin = (R_o @ E.T).T + t_o
        origin_rms = np.sqrt(np.mean(np.sum((E_origin - T) ** 2, axis=1)))
        print()
        print("=== Origin-anchored (only start pose pinned to truth) ===")
        for i, gid in enumerate(ids):
            print(f"  {gid}: {np.linalg.norm(E_origin[i] - T[i]):.3f} m")
        print(f"  RMS: {origin_rms:.3f} m")

    print()
    print("=== Orientation error ===")
    raw_deg, aligned_deg = [], []
    for gid in ids:
        R_t = Rotation.from_quat(true_quat[gid])
        R_e = Rotation.from_quat(est_quat[gid])
        raw = np.degrees((R_t.inv() * R_e).magnitude())
        aligned = np.degrees((R_t.inv() * R_global * R_e).magnitude())
        raw_deg.append(raw)
        aligned_deg.append(aligned)
        print(f"  {gid}: raw={raw:.2f} deg   after-global-rot={aligned:.2f} deg")
    print(f"  mean: raw={np.mean(raw_deg):.2f} deg   after-global-rot={np.mean(aligned_deg):.2f} deg")
    print("  (gates near-coplanar -> rotation gauge weakly observable)")


if __name__ == "__main__":
    main()
