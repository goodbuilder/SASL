#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import sys
import time
from pathlib import Path
from typing import Mapping, Sequence


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import validate_external_bbox_topic_contract as bbox_validator  # noqa: E402


def write_rows_csv(path: Path, rows: Sequence[Sequence[object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(bbox_validator.FIELD_NAMES)
        for row in rows:
            writer.writerow(list(row))


def sample_ros_topic(
    *,
    topic: str,
    sample_seconds: float,
    max_samples: int,
) -> list[list[float]]:
    try:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import Float32MultiArray
    except Exception as exc:  # pragma: no cover - depends on ROS2 runtime
        raise SystemExit(f"ROS2 Python packages are required for live topic sampling: {exc}") from exc

    rows: list[list[float]] = []
    rclpy.init()
    node = Node("external_bbox_topic_preflight_sampler")

    def callback(msg: object) -> None:
        if len(rows) >= max_samples:
            return
        rows.append([float(value) for value in list(getattr(msg, "data", []))])

    node.create_subscription(Float32MultiArray, topic, callback, 10)
    deadline = time.monotonic() + max(0.0, float(sample_seconds))
    try:
        while rclpy.ok() and time.monotonic() < deadline and len(rows) < max_samples:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return rows


def write_text(path: Path, summary: Mapping[str, object]) -> None:
    validation = dict(summary.get("validation", {}))
    lines = [
        "External BBox topic preflight",
        f"status: {summary.get('status', '')}",
        f"source_mode: {summary.get('source_mode', '')}",
        f"topic: {summary.get('topic', '')}",
        f"sample_csv: {summary.get('sample_csv', '')}",
        f"sample_count: {summary.get('sample_count', 0)}",
        f"min_valid_pairs: {summary.get('min_valid_pairs', 0)}",
        f"valid_pair_count: {summary.get('valid_pair_count', 0)}",
        f"validation_status: {validation.get('status', '')}",
        f"valid_pair_count: {validation.get('valid_pair_count', 0)}",
        f"disparity_min_px: {validation.get('disparity_min_px', '')}",
        f"disparity_max_px: {validation.get('disparity_max_px', '')}",
        f"runtime_truth_used: {summary.get('runtime_truth_used', False)}",
        f"coordinate_contract: {summary.get('coordinate_contract', '')}",
        f"failure_count: {summary.get('failure_count', 0)}",
    ]
    for failure in summary.get("failures", []):
        lines.append(f"- {failure}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def run(
    *,
    output_dir: Path,
    input_csv: Path | None = None,
    topic: str = "/stereo/external_detections",
    sample_seconds: float = 5.0,
    max_samples: int = 200,
    width_px: int = 1280,
    height_px: int = 720,
    min_disparity_px: float = 0.5,
    min_valid_pairs: int = 1,
) -> dict[str, object]:
    output_dir.mkdir(parents=True, exist_ok=True)
    source_mode = "csv" if input_csv is not None else "ros_topic"
    sample_csv = output_dir / "external_bbox_topic_samples.csv"
    failures: list[str] = []
    if input_csv is not None:
        sample_csv = input_csv
    else:
        rows = sample_ros_topic(topic=topic, sample_seconds=sample_seconds, max_samples=max_samples)
        write_rows_csv(sample_csv, rows)
        if not rows:
            failures.append("topic:no_samples")
    if float(min_disparity_px) <= 0.0:
        failures.append(f"config:min_disparity_px_not_positive:{float(min_disparity_px):g}")
    validation = bbox_validator.validate_csv(
        input_csv=sample_csv,
        width_px=width_px,
        height_px=height_px,
        min_disparity_px=min_disparity_px,
    )
    if validation.get("status") != "PASS":
        failures.extend(f"validation:{failure}" for failure in validation.get("failures", []))
    valid_pair_count = int(validation.get("valid_pair_count", 0) or 0)
    if valid_pair_count < int(min_valid_pairs):
        failures.append(f"valid_pair_count_below_min:{valid_pair_count}<{int(min_valid_pairs)}")
    summary = {
        "mode": "external_bbox_topic_preflight",
        "status": "PASS" if not failures else "FAIL",
        "source_mode": source_mode,
        "topic": topic,
        "sample_seconds": float(sample_seconds),
        "max_samples": int(max_samples),
        "min_valid_pairs": int(min_valid_pairs),
        "sample_csv": str(sample_csv),
        "sample_count": validation.get("row_count", 0),
        "valid_pair_count": valid_pair_count,
        "validation_json": str(output_dir / "external_bbox_topic_contract_validation.json"),
        "validation_txt": str(output_dir / "external_bbox_topic_contract_validation.txt"),
        "validation": validation,
        "runtime_truth_used": False,
        "coordinate_contract": (
            "preflight samples external BBox Float32MultiArray image pixel data only; validator requires "
            "positive left_cx-right_cx disparity; downstream StereoDistance converts to stereo-center "
            "RDF(right,down,forward) and publishes public FRD+range(forward,right,down,range); PX4/RTK/NED "
            "truth is not a runtime input"
        ),
        "failure_count": len(failures),
        "failures": failures,
    }
    (output_dir / "external_bbox_topic_contract_validation.json").write_text(
        json.dumps(validation, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    bbox_validator.write_text(output_dir / "external_bbox_topic_contract_validation.txt", validation)
    (output_dir / "external_bbox_topic_preflight_summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    write_text(output_dir / "external_bbox_topic_preflight_summary.txt", summary)
    return summary


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Sample or validate an external stereo BBox topic before enabling it in StereoDistance. "
            "Use --input-csv for offline smoke or --topic for live ROS2 Float32MultiArray sampling."
        )
    )
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--input-csv", type=Path)
    source.add_argument("--topic", default=None)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--sample-seconds", type=float, default=5.0)
    parser.add_argument("--max-samples", type=int, default=200)
    parser.add_argument("--width-px", type=int, default=1280)
    parser.add_argument("--height-px", type=int, default=720)
    parser.add_argument("--min-disparity-px", type=float, default=0.5)
    parser.add_argument("--min-valid-pairs", type=int, default=1)
    return parser.parse_args(argv)


def main() -> None:
    args = parse_args()
    summary = run(
        output_dir=args.output_dir,
        input_csv=args.input_csv,
        topic=args.topic or "/stereo/external_detections",
        sample_seconds=args.sample_seconds,
        max_samples=args.max_samples,
        width_px=args.width_px,
        height_px=args.height_px,
        min_disparity_px=args.min_disparity_px,
        min_valid_pairs=args.min_valid_pairs,
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False))
    if summary["status"] != "PASS":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
