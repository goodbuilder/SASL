#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import time
import sys
from pathlib import Path
from typing import Mapping


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import validate_observation_fusion_public_topic_payload as payload_validator  # noqa: E402


def payload_arrays(rows: list[Mapping[str, object]]) -> tuple[list[list[float]], list[str]]:
    failures: list[str] = []
    payloads: list[list[float]] = []
    for index, row in enumerate(rows):
        row_failures = payload_validator.validate_row(row, row_index=index, tolerance_m=1.0e-6)
        if row_failures:
            failures.extend(row_failures)
            continue
        payload = json.loads(str(row["float32_multi_array_data_json"]))
        payloads.append([float(value) for value in payload])
    return payloads, failures


def publish_payloads(payloads: list[list[float]], *, topic: str, rate_hz: float, once: bool) -> None:
    try:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import Float32MultiArray
    except Exception as exc:  # pragma: no cover - depends on ROS2 runtime
        raise SystemExit(f"ROS2 Python packages are required unless --dry-run is set: {exc}") from exc

    rclpy.init()
    node = Node("observation_fusion_public_payload_publisher")
    publisher = node.create_publisher(Float32MultiArray, topic, 10)
    period_s = 1.0 / max(rate_hz, 0.1)
    try:
        while rclpy.ok():
            for payload in payloads:
                msg = Float32MultiArray()
                msg.data = payload
                publisher.publish(msg)
                rclpy.spin_once(node, timeout_sec=0.0)
                if once:
                    return
                time.sleep(period_s)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def write_text(path: Path, summary: Mapping[str, object]) -> None:
    lines = [
        "Observation fusion public topic payload publisher",
        f"input_csv: {summary['input_csv']}",
        f"topic: {summary['topic']}",
        f"dry_run: {summary['dry_run']}",
        f"status: {summary['status']}",
        f"payload_count: {summary['payload_count']}",
        f"failure_count: {summary['failure_count']}",
        "runtime_truth_used: False",
        "Coordinate contract: publishes Float32MultiArray [forward,right,down,range] only.",
    ]
    for payload in summary.get("payloads", []):
        lines.append(f"payload: {payload}")
    for failure in summary.get("failures", []):
        lines.append(f"- {failure}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def run(
    *,
    input_csv: Path,
    output_dir: Path,
    topic: str,
    rate_hz: float,
    once: bool,
    dry_run: bool,
) -> dict[str, object]:
    _, rows = payload_validator.read_rows(input_csv)
    payloads, failures = payload_arrays(rows)
    summary = {
        "input_csv": str(input_csv),
        "topic": topic,
        "dry_run": dry_run,
        "status": "PASS" if payloads and not failures else "FAIL",
        "payload_count": len(payloads),
        "failure_count": len(failures),
        "failures": failures,
        "payloads": payloads,
        "runtime_truth_used": False,
        "coordinate_contracts_explicit": True,
    }
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "observation_fusion_public_topic_payload_publish_summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    write_text(output_dir / "observation_fusion_public_topic_payload_publish_summary.txt", summary)
    if summary["status"] != "PASS":
        return summary
    if not dry_run:
        publish_payloads(payloads, topic=topic, rate_hz=rate_hz, once=once)
    return summary


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Publish or dry-run replay public corrected-position payload CSV.")
    parser.add_argument("--input-csv", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--topic", default="/stereo/corrected_position")
    parser.add_argument("--rate-hz", type=float, default=10.0)
    parser.add_argument("--once", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    summary = run(
        input_csv=args.input_csv,
        output_dir=args.output_dir,
        topic=args.topic,
        rate_hz=args.rate_hz,
        once=args.once,
        dry_run=args.dry_run,
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False))
    if summary["status"] != "PASS":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
