#!/usr/bin/env python3
"""Extract ground-truth gate poses from a bag's /ground_truth/gates (MarkerArray).

Writes id,type,x,y,z,qx,qy,qz,qw (earth frame, full 6-DOF) — the surveyed gate
poses dataset_to_rosbag wrote from mocap. Marker id i -> gate_i, matching the
detection IDs and the SLAM-estimated object IDs.

Usage:
    pixi run python3 scripts/extract_gates_gt.py \
        --bag data/flight-01p-ellipse --output gates_ground_truth.csv
"""
import argparse
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from visualization_msgs.msg import MarkerArray


def parse_args():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=Path("gates_ground_truth.csv"))
    parser.add_argument("--topic", default="/ground_truth/gates")
    parser.add_argument("--storage", default="mcap")
    return parser.parse_args()


def main():
    args = parse_args()

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(args.bag), storage_id=args.storage),
        rosbag2_py.ConverterOptions("cdr", "cdr"))

    msg = None
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == args.topic:
            msg = deserialize_message(data, MarkerArray)
            break

    if msg is None:
        print(f"No message found on {args.topic}")
        return

    with open(args.output, "w") as f:
        f.write("id,type,x,y,z,qx,qy,qz,qw\n")
        for m in msg.markers:
            gid = f"gate_{m.id}"
            p, q = m.pose.position, m.pose.orientation
            f.write(f"{gid},gate,{p.x},{p.y},{p.z},{q.x},{q.y},{q.z},{q.w}\n")

    print(f"Wrote {len(msg.markers)} gates to {args.output}")
    for m in msg.markers:
        p, q = m.pose.position, m.pose.orientation
        print(f"  gate_{m.id}: pos=({p.x:.3f}, {p.y:.3f}, {p.z:.3f})  "
              f"quat=({q.x:.4f}, {q.y:.4f}, {q.z:.4f}, {q.w:.4f})")


if __name__ == "__main__":
    main()
