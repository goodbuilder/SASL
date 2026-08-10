#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import subprocess
from pathlib import Path
from typing import Mapping, Sequence


PX4_NED_AXIS_CONTRACT = "px4_local_position_ned:north,east,down"
STEREODISTANCE_TOPIC_CONTRACT = "stereodistance_position_topics:FRD_range(forward,right,down,range)"
CALIBRATION_RDF_AXIS_CONTRACT = "stereo_center_opencv_optical:RDF(right,down,forward)"


DEFAULT_REQUIRED_TOPICS = {
    "/stereo/nominal_pose_plot": "std_msgs/msg/Float32MultiArray",
    "/stereo/detections": "std_msgs/msg/Float32MultiArray",
    "/stereo/disparity": "std_msgs/msg/Float32",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Check whether live ROS topics needed for estimated-to-truth calibration "
            "CSV recording are present with expected message types."
        )
    )
    parser.add_argument("--truth-topic", default="/fmu/out/vehicle_local_position")
    parser.add_argument("--sim-truth-topic", default="")
    parser.add_argument("--nominal-pose-topic", default="/stereo/nominal_pose_plot")
    parser.add_argument("--detections-topic", default="/stereo/detections")
    parser.add_argument("--disparity-topic", default="/stereo/disparity")
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument(
        "--topic-list-file",
        type=Path,
        help="Optional saved output of `ros2 topic list -t` for offline checks/tests.",
    )
    return parser.parse_args()


def parse_topic_list_t(output: str) -> dict[str, set[str]]:
    topics: dict[str, set[str]] = {}
    for raw_line in output.splitlines():
        line = raw_line.strip()
        if not line or "[" not in line or "]" not in line:
            continue
        topic = line.split("[", 1)[0].strip()
        msg_type = line.rsplit("[", 1)[1].rsplit("]", 1)[0].strip()
        if topic and msg_type:
            topics.setdefault(topic, set()).add(msg_type)
    return topics


def topic_list_from_ros2() -> tuple[dict[str, set[str]], str]:
    completed = subprocess.run(
        ["ros2", "topic", "list", "-t"],
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )
    return parse_topic_list_t(completed.stdout), completed.stdout


def expected_topics(args: argparse.Namespace) -> dict[str, str]:
    topics = {
        args.nominal_pose_topic: DEFAULT_REQUIRED_TOPICS["/stereo/nominal_pose_plot"],
        args.detections_topic: DEFAULT_REQUIRED_TOPICS["/stereo/detections"],
        args.disparity_topic: DEFAULT_REQUIRED_TOPICS["/stereo/disparity"],
    }
    if args.sim_truth_topic:
        topics[args.sim_truth_topic] = "std_msgs/msg/Float32MultiArray"
    else:
        topics[args.truth_topic] = "px4_msgs/msg/VehicleLocalPosition"
    return topics


def evaluate_topics(
    available: Mapping[str, set[str]],
    expected: Mapping[str, str],
) -> dict[str, object]:
    checks = {}
    for topic, expected_type in expected.items():
        types = sorted(available.get(topic, set()))
        checks[topic] = {
            "expected_type": expected_type,
            "available_types": types,
            "present": bool(types),
            "type_ok": expected_type in types,
        }
    return {
        "pass_readiness": all(bool(item["present"]) and bool(item["type_ok"]) for item in checks.values()),
        "topic_checks": checks,
        "px4_truth_axis_contract": PX4_NED_AXIS_CONTRACT,
        "stereodistance_topic_contract": STEREODISTANCE_TOPIC_CONTRACT,
        "calibration_rdf_axis_contract": CALIBRATION_RDF_AXIS_CONTRACT,
    }


def write_outputs(output_dir: Path, summary: Mapping[str, object], raw_topic_list: str) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "topic_readiness.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    (output_dir / "ros2_topic_list_t.txt").write_text(raw_topic_list, encoding="utf-8")
    lines = ["Calibration topic readiness", ""]
    lines.append(f"pass_readiness: {summary['pass_readiness']}")
    lines.append(f"px4_truth_axis_contract: {PX4_NED_AXIS_CONTRACT}")
    lines.append(f"stereodistance_topic_contract: {STEREODISTANCE_TOPIC_CONTRACT}")
    lines.append(f"calibration_rdf_axis_contract: {CALIBRATION_RDF_AXIS_CONTRACT}")
    lines.append("")
    lines.append("topic_checks:")
    for topic, check in summary["topic_checks"].items():
        lines.append(
            f"- {topic}: present={check['present']} type_ok={check['type_ok']} "
            f"expected={check['expected_type']} available={check['available_types']}"
        )
    (output_dir / "topic_readiness.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    if args.topic_list_file:
        raw_topic_list = args.topic_list_file.read_text(encoding="utf-8")
        available = parse_topic_list_t(raw_topic_list)
    else:
        available, raw_topic_list = topic_list_from_ros2()
    summary = evaluate_topics(available, expected_topics(args))
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    if args.output_dir:
        write_outputs(args.output_dir, summary, raw_topic_list)
    return 0 if summary["pass_readiness"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
