#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path
from typing import Mapping


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import validate_observation_fusion_runtime_output_schema as output_validator  # noqa: E402


PUBLIC_TOPIC = "/stereo/corrected_position"
MESSAGE_TYPE = "std_msgs/msg/Float32MultiArray"
PUBLIC_TOPIC_CONTRACT = "stereodistance_position_topics:FRD_range(forward,right,down,range)"


def finite_float(value: object) -> float | None:
    try:
        parsed = float(str(value).strip())
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def read_rows(path: Path) -> tuple[list[str], list[dict[str, str]]]:
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        return list(reader.fieldnames or []), list(reader)


def payload_rows(rows: list[Mapping[str, object]], *, tolerance_m: float) -> tuple[list[dict[str, object]], list[str]]:
    failures: list[str] = []
    output_rows: list[dict[str, object]] = []
    for index, row in enumerate(rows):
        row_failures = output_validator.validate_row(row, row_index=index, tolerance_m=tolerance_m)
        if row_failures:
            failures.extend(row_failures)
            continue
        values = {
            "forward_m": finite_float(row.get("corrected_public_forward_m")),
            "right_m": finite_float(row.get("corrected_public_right_m")),
            "down_m": finite_float(row.get("corrected_public_down_m")),
            "range_m": finite_float(row.get("corrected_public_range_m")),
        }
        if any(value is None for value in values.values()):
            failures.append(f"row{index}:corrected_public_payload_not_finite")
            continue
        payload = [values["forward_m"], values["right_m"], values["down_m"], values["range_m"]]
        output_rows.append(
            {
                "group_id": str(row.get("group_id", f"row{index}")),
                "topic": PUBLIC_TOPIC,
                "message_type": MESSAGE_TYPE,
                "axis_order": "forward,right,down,range",
                "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
                "forward_m": values["forward_m"],
                "right_m": values["right_m"],
                "down_m": values["down_m"],
                "range_m": values["range_m"],
                "float32_multi_array_data_json": json.dumps(payload, separators=(",", ":")),
                "runtime_truth_used": False,
            }
        )
    return output_rows, failures


def write_csv(path: Path, rows: list[Mapping[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    fieldnames = list(dict.fromkeys(key for row in rows for key in row.keys()))
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_text(path: Path, summary: Mapping[str, object]) -> None:
    lines = [
        "Observation fusion public topic payload export",
        f"input_csv: {summary['input_csv']}",
        f"output_csv: {summary['output_csv']}",
        f"topic: {summary['topic']}",
        f"message_type: {summary['message_type']}",
        f"status: {summary['status']}",
        f"input_row_count: {summary['input_row_count']}",
        f"payload_row_count: {summary['payload_row_count']}",
        f"failure_count: {summary['failure_count']}",
        "runtime_truth_used: False",
        "Coordinate contract: corrected stereo-center RDF is exported as public FRD+range Float32MultiArray [forward,right,down,range].",
    ]
    for failure in summary.get("failures", []):
        lines.append(f"- {failure}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Export corrected runtime CSV rows as public StereoDistance topic payloads.")
    parser.add_argument("--input-csv", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--tolerance-m", type=float, default=1.0e-6)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    _, rows = read_rows(args.input_csv)
    payload, failures = payload_rows(rows, tolerance_m=args.tolerance_m)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    output_csv = args.output_dir / "observation_fusion_public_topic_payload.csv"
    write_csv(output_csv, payload)
    summary = {
        "input_csv": str(args.input_csv),
        "output_csv": str(output_csv),
        "topic": PUBLIC_TOPIC,
        "message_type": MESSAGE_TYPE,
        "status": "PASS" if payload and not failures else "FAIL",
        "input_row_count": len(rows),
        "payload_row_count": len(payload),
        "failure_count": len(failures),
        "failures": failures,
        "runtime_truth_used": False,
        "coordinate_contracts_explicit": True,
    }
    (args.output_dir / "observation_fusion_public_topic_payload_summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    write_text(args.output_dir / "observation_fusion_public_topic_payload_summary.txt", summary)
    print(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False))
    if summary["status"] != "PASS":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
