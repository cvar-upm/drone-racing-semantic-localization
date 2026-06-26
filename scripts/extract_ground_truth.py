#!/usr/bin/env python3
"""Extract ground truth odometry from a bag to CSV.

Usage:
    pixi run python3 scripts/extract_ground_truth.py \
        --bag dataset/piloted/flight-02p-ellipse/rosbag2_vio_detections \
        --output slam_ground_truth.csv
"""
import argparse
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry


def parse_args():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=Path("slam_ground_truth.csv"))
    parser.add_argument("--topic", default="/ground_truth/odometry")
    parser.add_argument("--storage", default="mcap")
    return parser.parse_args()


def main():
    args = parse_args()

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(args.bag), storage_id=args.storage),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"))

    count = 0
    with open(args.output, "w") as f:
        f.write("sec,nsec,x,y,z,qx,qy,qz,qw\n")
        while reader.has_next():
            topic, data, ts = reader.read_next()
            if topic == args.topic:
                msg = deserialize_message(data, Odometry)
                p = msg.pose.pose.position
                q = msg.pose.pose.orientation
                s = msg.header.stamp
                f.write(f"{s.sec},{s.nanosec},{p.x},{p.y},{p.z},"
                        f"{q.x},{q.y},{q.z},{q.w}\n")
                count += 1

    print(f"Wrote {count} GT samples to {args.output}")


if __name__ == "__main__":
    main()
