#!/usr/bin/env python3
"""Evaluate LIO-SAM drift metrics from a rosbag2 directory."""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def stamp_to_sec(stamp: Any) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def unwrap_delta_yaw(first_yaw: float, last_yaw: float) -> float:
    delta = last_yaw - first_yaw
    while delta > math.pi:
        delta -= 2.0 * math.pi
    while delta < -math.pi:
        delta += 2.0 * math.pi
    return delta


@dataclass
class PoseSample:
    stamp: float
    x: float
    y: float
    z: float
    yaw: float


class PoseMetric:
    def __init__(self) -> None:
        self.count = 0
        self.first: PoseSample | None = None
        self.last: PoseSample | None = None
        self.path_length = 0.0
        self.monotonic_time = True

    def add(self, sample: PoseSample) -> None:
        # 先頭・末尾poseと累積移動距離を同時に保持し、bag全体を1 passで評価する。
        if self.last is not None:
            if sample.stamp < self.last.stamp:
                self.monotonic_time = False
            self.path_length += math.dist(
                (self.last.x, self.last.y, self.last.z), (sample.x, sample.y, sample.z)
            )
        if self.first is None:
            self.first = sample
        self.last = sample
        self.count += 1

    def to_dict(self) -> dict[str, Any]:
        if self.first is None or self.last is None:
            return {"count": self.count, "available": False}
        displacement = math.dist(
            (self.first.x, self.first.y, self.first.z), (self.last.x, self.last.y, self.last.z)
        )
        return {
            "available": True,
            "count": self.count,
            "start_time": self.first.stamp,
            "end_time": self.last.stamp,
            "duration": self.last.stamp - self.first.stamp,
            "displacement": displacement,
            "path_length": self.path_length,
            "delta_yaw": unwrap_delta_yaw(self.first.yaw, self.last.yaw),
            "monotonic_time": self.monotonic_time,
            "start": self.first.__dict__,
            "end": self.last.__dict__,
        }


class CloudInfoMetric:
    def __init__(self) -> None:
        self.count = 0
        self.monotonic_time = True
        self.last_stamp: float | None = None
        self.last_guess: tuple[float, float, float] | None = None
        self.max_translation_jump = 0.0
        self.first_guess: tuple[float, float, float] | None = None
        self.final_guess: tuple[float, float, float] | None = None

    def add(self, msg: Any) -> None:
        # IMU preintegration由来の初期値ジャンプを検出し、scan matching前の破綻を切り分ける。
        stamp = stamp_to_sec(msg.header.stamp)
        guess = (
            float(msg.initial_guess_x),
            float(msg.initial_guess_y),
            float(msg.initial_guess_z),
        )
        if self.last_stamp is not None and stamp < self.last_stamp:
            self.monotonic_time = False
        if self.last_guess is not None:
            self.max_translation_jump = max(self.max_translation_jump, math.dist(self.last_guess, guess))
        if self.first_guess is None:
            self.first_guess = guess
        self.final_guess = guess
        self.last_guess = guess
        self.last_stamp = stamp
        self.count += 1

    def to_dict(self) -> dict[str, Any]:
        return {
            "available": self.count > 0,
            "count": self.count,
            "monotonic_time": self.monotonic_time,
            "max_translation_jump": self.max_translation_jump,
            "first_initial_guess_xyz": self.first_guess,
            "final_initial_guess_xyz": self.final_guess,
        }


def pose_sample_from_odom(msg: Any) -> PoseSample:
    pose = msg.pose.pose
    orientation = pose.orientation
    return PoseSample(
        stamp=stamp_to_sec(msg.header.stamp),
        x=float(pose.position.x),
        y=float(pose.position.y),
        z=float(pose.position.z),
        yaw=yaw_from_quaternion(
            float(orientation.x), float(orientation.y), float(orientation.z), float(orientation.w)
        ),
    )


def pose_sample_from_tf(transform: Any) -> PoseSample:
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    return PoseSample(
        stamp=stamp_to_sec(transform.header.stamp),
        x=float(translation.x),
        y=float(translation.y),
        z=float(translation.z),
        yaw=yaw_from_quaternion(float(rotation.x), float(rotation.y), float(rotation.z), float(rotation.w)),
    )


def make_reader(bag_path: Path) -> rosbag2_py.SequentialReader:
    storage_options = rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    return reader


def collect_metrics(
    args: argparse.Namespace,
    bag_path: Path,
    *,
    include_ground_truth_tf: bool,
    include_slam_outputs: bool,
) -> dict[str, Any]:
    reader = make_reader(bag_path)
    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    message_types = {topic: get_message(type_name) for topic, type_name in topic_types.items()}

    tf_metric = PoseMetric()
    lio_odom_metric = PoseMetric()
    imu_odom_metric = PoseMetric()
    cloud_info_metric = CloudInfoMetric()

    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic not in message_types:
            continue
        msg = deserialize_message(data, message_types[topic])

        if include_ground_truth_tf and topic == args.tf_topic:
            # TFMessage内から指定parent/childだけを抽出し、ground truth移動量として扱う。
            for transform in msg.transforms:
                if transform.header.frame_id == args.tf_parent and transform.child_frame_id == args.tf_child:
                    tf_metric.add(pose_sample_from_tf(transform))
        elif include_slam_outputs and topic == args.lio_odom_topic:
            lio_odom_metric.add(pose_sample_from_odom(msg))
        elif include_slam_outputs and topic == args.imu_odom_topic:
            imu_odom_metric.add(pose_sample_from_odom(msg))
        elif include_slam_outputs and topic == args.cloud_info_topic:
            cloud_info_metric.add(msg)

    return {
        "bag": str(bag_path),
        "topics": topic_types,
        "ground_truth_tf": tf_metric.to_dict(),
        "lio_mapping_odometry": lio_odom_metric.to_dict(),
        "imu_odometry": imu_odom_metric.to_dict(),
        "cloud_info_initial_guess": cloud_info_metric.to_dict(),
    }


def evaluate(args: argparse.Namespace) -> dict[str, Any]:
    slam_metrics = collect_metrics(
        args,
        args.bag,
        include_ground_truth_tf=args.ground_truth_bag is None,
        include_slam_outputs=True,
    )

    if args.ground_truth_bag is not None:
        # replay後bagではLIO-SAMのTFとsimulation TFが同じframe名で混ざるため、元bagからGTだけを読む。
        ground_truth_metrics = collect_metrics(
            args,
            args.ground_truth_bag,
            include_ground_truth_tf=True,
            include_slam_outputs=False,
        )
        slam_metrics["ground_truth_bag"] = str(args.ground_truth_bag)
        slam_metrics["ground_truth_tf"] = ground_truth_metrics["ground_truth_tf"]
        slam_metrics["ground_truth_topics"] = ground_truth_metrics["topics"]

    return slam_metrics


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path, help="rosbag2 directory containing metadata.yaml")
    parser.add_argument("--tf-topic", default="/tf")
    parser.add_argument("--tf-parent", default="odom")
    parser.add_argument("--tf-child", default="base_footprint")
    parser.add_argument("--ground-truth-bag", type=Path, default=None)
    parser.add_argument("--lio-odom-topic", default="/lio_sam/mapping/odometry")
    parser.add_argument("--imu-odom-topic", default="/odometry/imu")
    parser.add_argument("--cloud-info-topic", default="/lio_sam/feature/cloud_info")
    return parser.parse_args(argv)


def main(argv: list[str]) -> int:
    args = parse_args(argv)
    if not (args.bag / "metadata.yaml").is_file():
        print(f"metadata.yaml not found under {args.bag}", file=sys.stderr)
        return 2
    if args.ground_truth_bag is not None and not (args.ground_truth_bag / "metadata.yaml").is_file():
        print(f"metadata.yaml not found under {args.ground_truth_bag}", file=sys.stderr)
        return 2
    print(json.dumps(evaluate(args), ensure_ascii=False, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
