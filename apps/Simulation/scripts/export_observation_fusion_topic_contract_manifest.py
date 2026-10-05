#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Mapping


DEFAULT_TOPICS = {
    "px4_truth_topic": "/fmu/out/vehicle_local_position",
    "sim_truth_topic": "/ground_truth_position",
    "left_image_topic": "/stereo/left/image_raw",
    "right_image_topic": "/stereo/right/image_raw",
    "bbox_topic": "/stereo/detections",
    "disparity_topic": "/stereo/disparity",
    "nominal_pose_topic": "/stereo/nominal_pose_plot",
    "estimated_position_topic": "/stereo/estimated_position",
    "corrected_position_topic": "/stereo/corrected_position",
}


def topic_contract_rows(topics: Mapping[str, str]) -> list[dict[str, object]]:
    return [
        {
            "interface": "calibration_truth",
            "topic": topics["px4_truth_topic"],
            "message_hint": "px4_msgs/msg/VehicleLocalPosition",
            "runtime_input": False,
            "calibration_or_validation_only": True,
            "input_frame": "PX4 local position NED",
            "axis_order": "x/y/z aliases north/east/down",
            "output_fields": "truth_north_m,truth_east_m,truth_down_ned_m",
            "contract": (
                "offline truth source only; stereo-center RDF truth must be produced by a declared "
                "T_source_to_stereo_rdf or by an already stereo-frame truth producer; direct source-frame "
                "axis aliasing is forbidden"
            ),
        },
        {
            "interface": "simulation_truth",
            "topic": topics["sim_truth_topic"],
            "message_hint": "std_msgs/msg/Float32MultiArray",
            "runtime_input": False,
            "calibration_or_validation_only": True,
            "input_frame": "simulation public FRD+range",
            "axis_order": "forward,right,down,range",
            "output_fields": "truth_right_m,truth_down_m,truth_forward_m,truth_range_m",
            "contract": "synthetic/Gazebo truth only; never used by runtime correction or observation fusion",
        },
        {
            "interface": "stereo_bbox_observation",
            "topic": topics["bbox_topic"],
            "message_hint": "std_msgs/msg/Float32MultiArray or detector-specific bbox array",
            "runtime_input": True,
            "calibration_or_validation_only": False,
            "input_frame": "left/right image pixel coordinates",
            "axis_order": "left bbox center or extent; right bbox center or extent",
            "output_fields": "group_id,burst_id,left_u_px,left_v_px,measured_disparity_px",
            "contract": "left/right bbox centers convert to image observation input; disparity is left_u_px-right_u_px unless explicitly measured",
        },
        {
            "interface": "left_camera_image",
            "topic": topics["left_image_topic"],
            "message_hint": "sensor_msgs/msg/Image",
            "runtime_input": True,
            "calibration_or_validation_only": False,
            "input_frame": "OpenCV image plane",
            "axis_order": "u right, v down",
            "output_fields": "left_u_px,left_v_px via detector or tracker",
            "contract": "image source for detector/tracker; image frame maps to RDF right/down bearing",
        },
        {
            "interface": "right_camera_image",
            "topic": topics["right_image_topic"],
            "message_hint": "sensor_msgs/msg/Image",
            "runtime_input": True,
            "calibration_or_validation_only": False,
            "input_frame": "OpenCV image plane",
            "axis_order": "u right, v down",
            "output_fields": "right_u_px,right_v_px via detector or tracker",
            "contract": "right image source for disparity; baseline is RDF right",
        },
        {
            "interface": "measured_disparity",
            "topic": topics["disparity_topic"],
            "message_hint": "std_msgs/msg/Float32",
            "runtime_input": True,
            "calibration_or_validation_only": False,
            "input_frame": "stereo image pair",
            "axis_order": "scalar positive px",
            "output_fields": "measured_disparity_px",
            "contract": "positive left-minus-right disparity in pixels; no true disparity at runtime",
        },
        {
            "interface": "raw_stereodistance_public_estimate",
            "topic": topics["estimated_position_topic"],
            "message_hint": "std_msgs/msg/Float32MultiArray",
            "runtime_input": False,
            "calibration_or_validation_only": False,
            "input_frame": "StereoDistance public FRD+range",
            "axis_order": "forward,right,down,range",
            "output_fields": "raw_forward_m,raw_right_m,raw_down_m,raw_range_m",
            "contract": "public estimate alias only; observation fusion prefers image u/v/disparity and does not consume this topic as input",
        },
        {
            "interface": "corrected_stereodistance_public_output",
            "topic": topics["corrected_position_topic"],
            "message_hint": "std_msgs/msg/Float32MultiArray",
            "runtime_input": False,
            "calibration_or_validation_only": False,
            "input_frame": "StereoDistance public FRD+range",
            "axis_order": "forward,right,down,range",
            "output_fields": "corrected_public_forward_m,corrected_public_right_m,corrected_public_down_m,corrected_public_range_m",
            "contract": "runtime output alias from corrected stereo-center RDF; publish as public FRD+range",
        },
    ]


def build_manifest(topics: Mapping[str, str]) -> dict[str, object]:
    rows = topic_contract_rows(topics)
    return {
        "schema": "observation_fusion_topic_contract_manifest/v1",
        "purpose": "ROS2 topic and CSV boundary contracts for StereoDistance observation fusion deployment",
        "runtime_truth_used": False,
        "coordinate_contracts_explicit": True,
        "frames": {
            "px4_truth": "NED north/east/down truth source; calibration/validation only; explicit T_source_to_stereo_rdf required before RDF evaluation",
            "image": "OpenCV image u right, v down",
            "internal_measurement": "stereo-center RDF right/down/forward",
            "public_stereodistance": "FRD+range forward/right/down/range",
        },
        "topics": dict(topics),
        "interfaces": rows,
        "runtime_input_interfaces": [
            row["interface"] for row in rows if bool(row["runtime_input"])
        ],
        "forbidden_runtime_inputs": [
            "PX4/RTK truth",
            "Gazebo synthetic truth",
            "true disparity",
            "true target origin/image location",
        ],
    }


def write_csv(path: Path, rows: list[Mapping[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(dict.fromkeys(key for row in rows for key in row.keys()))
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_summary(path: Path, manifest: Mapping[str, object]) -> None:
    lines = [
        "Observation fusion topic contract manifest",
        f"schema: {manifest['schema']}",
        "runtime_truth_used: False",
        "frames:",
    ]
    for name, contract in dict(manifest["frames"]).items():
        lines.append(f"- {name}: {contract}")
    lines.append("interfaces:")
    for row in manifest["interfaces"]:
        lines.append(
            f"- {row['interface']}: topic={row['topic']}, runtime_input={row['runtime_input']}, "
            f"axis_order={row['axis_order']}"
        )
    lines.append(
        "Coordinate contract: PX4 NED truth is offline source data only; RDF truth requires an "
        "explicit T_source_to_stereo_rdf or an already stereo-frame producer; runtime input is "
        "image u/v/disparity; internal output is RDF; public output is FRD+range."
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Export ROS2 topic/CSV contracts for observation-fusion runtime deployment.")
    for name, default in DEFAULT_TOPICS.items():
        parser.add_argument(f"--{name.replace('_', '-')}", default=default)
    parser.add_argument("--output-dir", type=Path, required=True)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    topics = {name: getattr(args, name) for name in DEFAULT_TOPICS}
    manifest = build_manifest(topics)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / "observation_fusion_topic_contract_manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    write_csv(args.output_dir / "observation_fusion_topic_contract_manifest.csv", manifest["interfaces"])
    write_summary(args.output_dir / "observation_fusion_topic_contract_manifest_summary.txt", manifest)
    print(json.dumps(manifest, ensure_ascii=False, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
