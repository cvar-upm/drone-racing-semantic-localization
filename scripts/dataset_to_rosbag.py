#!/usr/bin/env python3
import argparse
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import pandas as pd
import yaml
from scipy.spatial.transform import Rotation

import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message

from builtin_interfaces.msg import Time
from geometry_msgs.msg import (
    Point,
    Pose,
    PoseStamped,
    PoseWithCovariance,
    Quaternion,
    Twist,
    TwistWithCovariance,
    Vector3,
)
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image, Imu
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker, MarkerArray

from as2_msgs.msg import PoseStampedWithID, PoseStampedWithIDArray


@dataclass
class DatasetPaths:
    flight_id: str
    dataset_dir: Path
    imu_csv: Path
    drone_state_csv: Path
    camera_csv: Path
    mocap_csv: Path
    image_dir: Path
    metadata_yaml: Path
    original_bag: Path


def parse_args():
    parser = argparse.ArgumentParser(
        description="Convert TII drone racing dataset to ROS2 bag"
    )
    parser.add_argument(
        "--dataset-dir",
        type=Path,
        required=True,
        help="Path to dataset directory (e.g., dataset/piloted/flight-01p-ellipse)",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Output bag directory (default: <dataset-dir>/rosbag_vio)",
    )
    parser.add_argument(
        "--storage",
        choices=["mcap", "sqlite3"],
        default="mcap",
        help="Bag storage format (default: mcap)",
    )
    parser.add_argument(
        "--compress",
        action="store_true",
        help="Enable zstd compression (mcap only)",
    )
    parser.add_argument(
        "--detection-range",
        type=float,
        default=15.0,
        help="Max range (m) for synthesized gate detections (default: 15.0)",
    )
    parser.add_argument(
        "--prepend-static",
        type=float,
        default=0.0,
        help="Prepend N seconds of static data (repeat first frame/IMU) before real data",
    )
    parser.add_argument(
        "--fov",
        type=float,
        default=None,
        help="Camera FOV in degrees for detection filtering (default: None = omnidirectional)",
    )
    parser.add_argument(
        "--no-images",
        action="store_true",
        help="Skip camera images",
    )
    parser.add_argument(
        "--camera-skip",
        type=int,
        default=0,
        help="Skip N frames between each written frame (e.g., 3 to keep 1 of 4 → ~30Hz from 120Hz)",
    )
    return parser.parse_args()


def resolve_paths(dataset_dir: Path) -> DatasetPaths:
    flight_id = dataset_dir.name
    imu_csv = dataset_dir / "csv_raw" / "ros2bag_dump" / f"imu_{flight_id}.csv"
    drone_state_csv = (
        dataset_dir / "csv_raw" / "ros2bag_dump" / f"drone_state_{flight_id}.csv"
    )
    camera_csv = dataset_dir / "csv_raw" / f"camera_{flight_id}.csv"
    mocap_csv = dataset_dir / "csv_raw" / f"mocap_{flight_id}.csv"
    image_dir = dataset_dir / f"camera_{flight_id}"
    metadata_yaml = dataset_dir / f"metadata_{flight_id}.yaml"
    original_bag = dataset_dir / f"ros2bag_{flight_id}"

    paths = DatasetPaths(
        flight_id=flight_id,
        dataset_dir=dataset_dir,
        imu_csv=imu_csv,
        drone_state_csv=drone_state_csv,
        camera_csv=camera_csv,
        mocap_csv=mocap_csv,
        image_dir=image_dir,
        metadata_yaml=metadata_yaml,
        original_bag=original_bag,
    )

    for field_name in [
        "imu_csv",
        "drone_state_csv",
        "camera_csv",
        "mocap_csv",
        "image_dir",
        "metadata_yaml",
    ]:
        p = getattr(paths, field_name)
        if not p.exists():
            print(f"WARNING: {field_name} not found: {p}")

    return paths


CALIBRATED_INTRINSICS = {
    "fx": 291.520, "fy": 390.011, "cx": 316.447, "cy": 240.442,
    "distortion_model": "equidistant",
    "d": [0.0478, -0.0282, 0.0376, -0.0184],
}


def make_camera_info(width: int = 640, height: int = 480) -> CameraInfo:
    fx = CALIBRATED_INTRINSICS["fx"]
    fy = CALIBRATED_INTRINSICS["fy"]
    cx = CALIBRATED_INTRINSICS["cx"]
    cy = CALIBRATED_INTRINSICS["cy"]

    msg = CameraInfo()
    msg.width = width
    msg.height = height
    msg.distortion_model = CALIBRATED_INTRINSICS["distortion_model"]
    msg.d = CALIBRATED_INTRINSICS["d"]
    msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
    msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    return msg


def write_camera_messages(
    writer: rosbag2_py.SequentialWriter,
    camera_csv: Path,
    image_dir: Path,
    metadata: dict,
    camera_skip: int = 0,
) -> int:
    from concurrent.futures import ThreadPoolExecutor
    from collections import deque

    print(f"Writing camera images from {image_dir.name}/...")
    df = pd.read_csv(camera_csv)
    width = metadata.get("camera", {}).get("image_width", 640)
    height = metadata.get("camera", {}).get("image_height", 480)
    camera_info_template = make_camera_info(width, height)

    if camera_skip > 0:
        df = df.iloc[::camera_skip + 1].reset_index(drop=True)

    def load_and_serialize(row):
        img_path = image_dir / row["filename"]
        if not img_path.exists():
            return None

        ts = int(row["timestamp"])
        header = make_header(ts, "camera")

        img = cv2.imread(str(img_path))
        if img is None:
            return None
        img_msg = Image()
        img_msg.header = header
        img_msg.height = img.shape[0]
        img_msg.width = img.shape[1]
        img_msg.encoding = "bgr8"
        img_msg.is_bigendian = False
        img_msg.step = img.shape[1] * 3
        img_msg.data = img.tobytes()

        ci = CameraInfo()
        ci.header = header
        ci.width = camera_info_template.width
        ci.height = camera_info_template.height
        ci.distortion_model = camera_info_template.distortion_model
        ci.d = camera_info_template.d
        ci.k = camera_info_template.k
        ci.r = camera_info_template.r
        ci.p = camera_info_template.p

        return (
            ts_us_to_ns(ts),
            serialize_message(img_msg),
            serialize_message(ci),
        )

    count = 0
    total = len(df)
    prefetch = 128

    with ThreadPoolExecutor(max_workers=8) as pool:
        futures = deque()
        row_iter = df.iterrows()

        for _ in range(prefetch):
            try:
                _, row = next(row_iter)
                futures.append(pool.submit(load_and_serialize, row))
            except StopIteration:
                break

        while futures:
            future = futures.popleft()

            try:
                _, row = next(row_iter)
                futures.append(pool.submit(load_and_serialize, row))
            except StopIteration:
                pass

            result = future.result()
            if result is None:
                continue

            ts_ns, img_data, ci_data = result
            writer.write("/camera/image_raw", img_data, ts_ns)
            writer.write("/camera/camera_info", ci_data, ts_ns)
            count += 1

            if count % 1000 == 0:
                print(f"  Camera: {count}/{total} images...")

    print(f"  Camera: {count} images")
    return count


def ts_us_to_ros(timestamp_us: int) -> Time:
    t = Time()
    t.sec = int(timestamp_us // 1_000_000)
    t.nanosec = int((timestamp_us % 1_000_000) * 1000)
    return t


def ts_us_to_ns(timestamp_us: int) -> int:
    return int(timestamp_us) * 1000


def make_header(timestamp_us: int, frame_id: str) -> Header:
    h = Header()
    h.stamp = ts_us_to_ros(timestamp_us)
    h.frame_id = frame_id
    return h


def create_writer(
    output_dir: Path, storage_id: str, compress: bool = False
) -> rosbag2_py.SequentialWriter:
    writer = rosbag2_py.SequentialWriter()

    storage_config_uri = ""
    if compress and storage_id == "mcap":
        config_file = tempfile.NamedTemporaryFile(
            mode="w", suffix=".yaml", delete=False
        )
        config_file.write("output:\n  compression: zstd\n  chunk_size: 4194304\n")
        config_file.close()
        storage_config_uri = config_file.name

    storage_options = rosbag2_py.StorageOptions(
        uri=str(output_dir),
        storage_id=storage_id,
        storage_config_uri=storage_config_uri,
    )
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )
    writer.open(storage_options, converter_options)

    topics = [
        ("/imu", "sensor_msgs/msg/Imu"),
        ("/camera/image_raw", "sensor_msgs/msg/Image"),
        ("/camera/camera_info", "sensor_msgs/msg/CameraInfo"),
        ("/ground_truth/pose", "geometry_msgs/msg/PoseStamped"),
        ("/ground_truth/odometry", "nav_msgs/msg/Odometry"),
        ("/ground_truth/gates", "visualization_msgs/msg/MarkerArray"),
        ("/detections/gates", "as2_msgs/msg/PoseStampedWithIDArray"),
        ("/detections/markers/gates", "visualization_msgs/msg/MarkerArray"),
        ("/tf_static", "tf2_msgs/msg/TFMessage"),
    ]

    for topic_name, topic_type in topics:
        topic = rosbag2_py.TopicMetadata(
            name=topic_name,
            type=topic_type,
            serialization_format="cdr",
        )
        writer.create_topic(topic)

    return writer


def write_imu_from_bag(writer: rosbag2_py.SequentialWriter, original_bag: Path) -> int:
    from rosidl_runtime_py.utilities import get_message

    print(f"Writing IMU from original bag {original_bag.name}...")
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(original_bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )

    topics = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topics}
    reader.set_filter(rosbag2_py.StorageFilter(topics=["/sensors/imu"]))

    imu_msg_type = get_message(type_map["/sensors/imu"])
    count = 0
    while reader.has_next():
        topic, data, ts = reader.read_next()
        src = deserialize_message(data, imu_msg_type)
        msg = Imu()
        msg.header = make_header(int(src.timestamp), "imu_link")
        msg.linear_acceleration = Vector3(
            x=float(src.accel_x), y=float(src.accel_y), z=float(src.accel_z)
        )
        msg.angular_velocity = Vector3(
            x=float(src.gyro_x), y=float(src.gyro_y), z=float(src.gyro_z)
        )
        msg.orientation_covariance[0] = -1.0
        writer.write("/imu", serialize_message(msg), ts_us_to_ns(int(src.timestamp)))
        count += 1

    del reader
    print(f"  IMU: {count} messages (from original bag, includes pre-flight data)")
    return count


def write_imu_messages(
    writer: rosbag2_py.SequentialWriter, imu_csv: Path, original_bag: Path = None
) -> int:
    if original_bag is not None and original_bag.exists():
        return write_imu_from_bag(writer, original_bag)
    print(f"Writing IMU from {imu_csv.name}...")
    df = pd.read_csv(imu_csv)
    count = 0

    for _, row in df.iterrows():
        msg = Imu()
        msg.header = make_header(int(row["timestamp"]), "imu_link")
        msg.linear_acceleration = Vector3(
            x=float(row["accel_x"]),
            y=float(row["accel_y"]),
            z=float(row["accel_z"]),
        )
        msg.angular_velocity = Vector3(
            x=float(row["gyro_x"]),
            y=float(row["gyro_y"]),
            z=float(row["gyro_z"]),
        )
        msg.orientation_covariance[0] = -1.0

        writer.write(
            "/imu",
            serialize_message(msg),
            ts_us_to_ns(int(row["timestamp"])),
        )
        count += 1

    print(f"  IMU: {count} messages")
    return count


def write_ground_truth_from_bag(
    writer: rosbag2_py.SequentialWriter, original_bag: Path
) -> int:
    from rosidl_runtime_py.utilities import get_message

    print(f"Writing ground truth from original bag {original_bag.name}...")
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(original_bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )

    topics = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topics}
    reader.set_filter(
        rosbag2_py.StorageFilter(topics=["/perception/drone_state"])
    )

    msg_type = get_message(type_map["/perception/drone_state"])
    count = 0
    while reader.has_next():
        topic, data, ts = reader.read_next()
        src = deserialize_message(data, msg_type)
        t = int(src.timestamp)

        pose_msg = PoseStamped()
        pose_msg.header = make_header(t, "earth")
        pose_msg.pose = src.pose

        odom_msg = Odometry()
        odom_msg.header = make_header(t, "earth")
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose = PoseWithCovariance(pose=src.pose)
        odom_msg.twist = TwistWithCovariance(twist=src.velocity)

        ts_ns = ts_us_to_ns(t)
        writer.write("/ground_truth/pose", serialize_message(pose_msg), ts_ns)
        writer.write("/ground_truth/odometry", serialize_message(odom_msg), ts_ns)
        count += 1

    del reader
    print(f"  Ground truth: {count} messages (from original bag)")
    return count


def write_ground_truth(
    writer: rosbag2_py.SequentialWriter,
    drone_state_csv: Path,
    original_bag: Path = None,
) -> int:
    if original_bag is not None and original_bag.exists():
        return write_ground_truth_from_bag(writer, original_bag)
    print(f"Writing ground truth from {drone_state_csv.name}...")
    df = pd.read_csv(drone_state_csv)
    count = 0

    for _, row in df.iterrows():
        ts = int(row["timestamp"])

        pose_msg = PoseStamped()
        pose_msg.header = make_header(ts, "earth")
        pose_msg.pose = Pose(
            position=Point(
                x=float(row["pose_position_x"]),
                y=float(row["pose_position_y"]),
                z=float(row["pose_position_z"]),
            ),
            orientation=Quaternion(
                x=float(row["pose_orientation_x"]),
                y=float(row["pose_orientation_y"]),
                z=float(row["pose_orientation_z"]),
                w=float(row["pose_orientation_w"]),
            ),
        )

        odom_msg = Odometry()
        odom_msg.header = make_header(ts, "earth")
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose = PoseWithCovariance(pose=pose_msg.pose)
        odom_msg.twist = TwistWithCovariance(
            twist=Twist(
                linear=Vector3(
                    x=float(row["velocity_linear_x"]),
                    y=float(row["velocity_linear_y"]),
                    z=float(row["velocity_linear_z"]),
                ),
                angular=Vector3(
                    x=float(row["velocity_angular_x"]),
                    y=float(row["velocity_angular_y"]),
                    z=float(row["velocity_angular_z"]),
                ),
            )
        )

        ts_ns = ts_us_to_ns(ts)
        writer.write("/ground_truth/pose", serialize_message(pose_msg), ts_ns)
        writer.write("/ground_truth/odometry", serialize_message(odom_msg), ts_ns)
        count += 1

    print(f"  Ground truth: {count} messages (pose + odometry)")
    return count


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    r = Rotation.from_euler("xyz", [roll, pitch, yaw])
    q = r.as_quat()
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])


def rotmat_to_quaternion(rot: np.ndarray) -> Quaternion:
    r = Rotation.from_matrix(rot)
    q = r.as_quat()
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])


def write_gate_ground_truth(
    writer: rosbag2_py.SequentialWriter, mocap_csv: Path
) -> int:
    print(f"Writing gate ground truth from {mocap_csv.name}...")
    df = pd.read_csv(mocap_csv, nrows=1)
    row = df.iloc[0]
    ts = int(row["timestamp"])

    marker_array = MarkerArray()
    for i in range(1, 5):
        prefix = f"gate{i}_int_"
        marker = Marker()
        marker.header = make_header(ts, "earth")
        marker.ns = "gates"
        marker.id = i
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose = Pose(
            position=Point(
                x=float(row[f"{prefix}x"]),
                y=float(row[f"{prefix}y"]),
                z=float(row[f"{prefix}z"]),
            ),
            orientation=euler_to_quaternion(
                float(row[f"{prefix}roll"]),
                float(row[f"{prefix}pitch"]),
                float(row[f"{prefix}yaw"]),
            ),
        )
        marker.scale = Vector3(x=0.1, y=1.4, z=1.4)
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.8
        marker.lifetime.sec = 0
        marker_array.markers.append(marker)

    writer.write(
        "/ground_truth/gates",
        serialize_message(marker_array),
        ts_us_to_ns(ts),
    )

    tf_msg = TFMessage()
    for marker in marker_array.markers:
        tf = TransformStamped()
        tf.header = make_header(ts, "earth")
        tf.child_frame_id = f"gate_{marker.id}"
        tf.transform.translation = Vector3(
            x=marker.pose.position.x,
            y=marker.pose.position.y,
            z=marker.pose.position.z,
        )
        tf.transform.rotation = marker.pose.orientation
        tf_msg.transforms.append(tf)
    writer.write("/tf_static", serialize_message(tf_msg), ts_us_to_ns(ts))

    print(f"  Gate ground truth: 4 gates published (markers + tf_static)")
    return 1


def write_gate_detections(
    writer: rosbag2_py.SequentialWriter,
    camera_csv: Path,
    mocap_csv: Path,
    detection_range: float,
    fov: float = None,
) -> int:
    fov_half_angle = np.radians(fov / 2.0) if fov is not None else None
    fov_str = f", FOV < {fov}°" if fov else ", omnidirectional"
    print(f"Synthesizing gate detections (range < {detection_range}m{fov_str})...")
    cam_df = pd.read_csv(camera_csv)
    mocap_df = pd.read_csv(mocap_csv)
    mocap_timestamps = mocap_df["timestamp"].values
    count = 0

    for _, cam_row in cam_df.iterrows():
        cam_ts = int(cam_row["timestamp"])
        idx = np.argmin(np.abs(mocap_timestamps - cam_ts))
        mrow = mocap_df.iloc[idx]

        drone_rot = np.array(
            [
                [mrow["drone_rot[0]"], mrow["drone_rot[1]"], mrow["drone_rot[2]"]],
                [mrow["drone_rot[3]"], mrow["drone_rot[4]"], mrow["drone_rot[5]"]],
                [mrow["drone_rot[6]"], mrow["drone_rot[7]"], mrow["drone_rot[8]"]],
            ]
        )
        drone_pos = np.array([mrow["drone_x"], mrow["drone_y"], mrow["drone_z"]])

        det_msg = PoseStampedWithIDArray()
        header = make_header(cam_ts, "base_link")

        for i in range(1, 5):
            prefix = f"gate{i}_int_"
            gate_pos = np.array(
                [mrow[f"{prefix}x"], mrow[f"{prefix}y"], mrow[f"{prefix}z"]]
            )
            gate_rot = np.array(
                [
                    [
                        mrow[f"{prefix}rot[0]"],
                        mrow[f"{prefix}rot[1]"],
                        mrow[f"{prefix}rot[2]"],
                    ],
                    [
                        mrow[f"{prefix}rot[3]"],
                        mrow[f"{prefix}rot[4]"],
                        mrow[f"{prefix}rot[5]"],
                    ],
                    [
                        mrow[f"{prefix}rot[6]"],
                        mrow[f"{prefix}rot[7]"],
                        mrow[f"{prefix}rot[8]"],
                    ],
                ]
            )

            rel_pos = drone_rot @ (gate_pos - drone_pos)
            dist = np.linalg.norm(rel_pos)

            if dist > detection_range:
                continue

            if fov_half_angle is not None:
                angle_from_forward = np.arctan2(
                    np.sqrt(rel_pos[1]**2 + rel_pos[2]**2), rel_pos[0]
                )
                if angle_from_forward > fov_half_angle:
                    continue

            rel_rot = drone_rot @ gate_rot.T
            q = rotmat_to_quaternion(rel_rot)

            det = PoseStampedWithID()
            det.id = f"gate_{i}"
            det.pose = PoseStamped()
            det.pose.header = header
            det.pose.pose = Pose(
                position=Point(x=float(rel_pos[0]), y=float(rel_pos[1]), z=float(rel_pos[2])),
                orientation=q,
            )
            det_msg.poses.append(det)

        if len(det_msg.poses) > 0:
            writer.write(
                "/detections/gates",
                serialize_message(det_msg),
                ts_us_to_ns(cam_ts),
            )

            marker_array = MarkerArray()
            for det in det_msg.poses:
                marker = Marker()
                marker.header = header
                marker.ns = det.id
                marker.id = int(det.id.split("_")[1])
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                marker.pose = det.pose.pose
                marker.scale = Vector3(x=0.1, y=1.4, z=1.4)
                gate_colors = {1: (1.0, 0.0, 0.0), 2: (0.0, 1.0, 0.0),
                               3: (0.0, 0.0, 1.0), 4: (1.0, 1.0, 0.0)}
                r, g, b = gate_colors.get(marker.id, (1.0, 1.0, 1.0))
                marker.color.r = r
                marker.color.g = g
                marker.color.b = b
                marker.color.a = 0.6
                marker.lifetime.nanosec = 200_000_000
                marker_array.markers.append(marker)
            writer.write(
                "/detections/markers/gates",
                serialize_message(marker_array),
                ts_us_to_ns(cam_ts),
            )

            count += 1

    print(f"  Gate detections: {count} messages")
    return count


def main():
    args = parse_args()
    paths = resolve_paths(args.dataset_dir.resolve())

    if args.output_dir is None:
        output_dir = paths.dataset_dir / "rosbag_vio"
    else:
        output_dir = args.output_dir.resolve()

    if output_dir.exists():
        print(f"ERROR: Output directory already exists: {output_dir}")
        sys.exit(1)

    metadata = {}
    if paths.metadata_yaml.exists():
        with open(paths.metadata_yaml) as f:
            metadata = yaml.safe_load(f)

    print(f"Dataset: {paths.flight_id}")
    print(f"Output:  {output_dir}")
    print(f"Storage: {args.storage}")
    print()

    writer = create_writer(output_dir, args.storage, args.compress)

    counts = {}
    counts["imu"] = write_imu_messages(writer, paths.imu_csv, paths.original_bag)
    counts["ground_truth"] = write_ground_truth(
        writer, paths.drone_state_csv, paths.original_bag
    )

    if not args.no_images:
        counts["camera"] = write_camera_messages(
            writer, paths.camera_csv, paths.image_dir, metadata,
            camera_skip=args.camera_skip,
        )
    else:
        print("Skipping camera images (--no-images)")
        counts["camera"] = 0

    counts["gate_gt"] = write_gate_ground_truth(writer, paths.mocap_csv)
    counts["gate_detections"] = write_gate_detections(
        writer, paths.camera_csv, paths.mocap_csv, args.detection_range, fov=args.fov
    )

    del writer

    print()
    print("=== Summary ===")
    for k, v in counts.items():
        print(f"  {k}: {v}")
    print(f"  Output: {output_dir}")


if __name__ == "__main__":
    main()
