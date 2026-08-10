#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path
from typing import Mapping, Sequence


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import stereodistance_profile_promotion_check as promotion_check  # noqa: E402


PUBLIC_TOPIC = "/stereo/corrected_position"
PUBLIC_AXIS_ORDER = ("forward_m", "right_m", "down_m", "range_m")
PUBLIC_TOPIC_CONTRACT = "std_msgs/Float32MultiArray:data=[forward_m,right_m,down_m,range_m]"
RDF_CONTRACT = "runtime_input:stereo_center_RDF(right,down,forward)"
NODE_FLOAT32_EQUIVALENCE_TOLERANCE_M = 1.0e-3


class CapturePublisher:
    def __init__(self) -> None:
        self.messages: list[object] = []

    def publish(self, msg: object) -> None:
        self.messages.append(msg)


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Export a dry-run StereoDistance public corrected-position payload from an active runtime profile."
    )
    parser.add_argument("--profile-json", type=Path, required=True)
    parser.add_argument("--sample-csv", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--split", default="validation")
    parser.add_argument("--max-samples", type=int, default=25)
    return parser.parse_args(argv)


def finite(value: object) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def public_payload_from_corrected(corrected: Mapping[str, object]) -> list[float]:
    values = [
        float(corrected["forward_m"]),
        float(corrected["right_m"]),
        float(corrected["down_m"]),
        float(corrected["range_m"]),
    ]
    if not all(math.isfinite(value) for value in values):
        raise ValueError("public payload contains non-finite values")
    return values


def node_apply_corrected_payload(profile_json: Path, raw_payload: Mapping[str, object]) -> tuple[list[float] | None, dict[str, object]]:
    node = promotion_check.StereoDistanceNode.__new__(promotion_check.StereoDistanceNode)
    node.enable_bbox_center_adjustment = True
    node.bbox_center_adjustment_path = str(profile_json)
    node.get_logger = lambda: type(
        "Logger",
        (),
        {
            "warning": lambda self, msg: None,
            "info": lambda self, msg: None,
        },
    )()
    node.bbox_center_adjustment = promotion_check.StereoDistanceNode._load_bbox_center_adjustment(node)
    if node.bbox_center_adjustment is None:
        return None, {"fallback_level": 3, "fallback_reason": "node_profile_load_failed"}

    profile_mode = getattr(node.bbox_center_adjustment, "mode", "")
    if profile_mode == "estimated_to_truth_sim3":
        left = None
        right = None
        if "left_u_px" in raw_payload and "left_v_px" in raw_payload:
            left = {"cx": float(raw_payload["left_u_px"]), "cy": float(raw_payload["left_v_px"])}
        corrected, status = promotion_check.StereoDistanceNode._apply_estimated_to_truth_sim3_profile(
            node,
            dict(raw_payload),
            left,
            right,
        )
    elif profile_mode == "estimated_to_truth_disparity_poly":
        corrected, status = promotion_check.StereoDistanceNode._apply_estimated_to_truth_disparity_profile(
            node,
            dict(raw_payload),
        )
    elif profile_mode == "estimated_to_truth_affine_rdf":
        corrected, status = promotion_check.StereoDistanceNode._apply_estimated_to_truth_affine_profile(
            node,
            dict(raw_payload),
        )
    else:
        return None, {"fallback_level": 3, "fallback_reason": f"unsupported_node_profile_mode:{profile_mode}"}

    if not isinstance(corrected, Mapping):
        return None, status

    estimated_pub = CapturePublisher()
    corrected_pub = CapturePublisher()
    bbox_pub = CapturePublisher()
    status_pub = CapturePublisher()
    node.estimated_position_pub = estimated_pub
    node.corrected_position_pub = corrected_pub
    node.bbox_adjustment_pub = bbox_pub
    node.profile_status_pub = status_pub
    metrics = {
        "estimated_right_m": raw_payload["right_m"],
        "estimated_down_m": raw_payload["down_m"],
        "estimated_forward_m": raw_payload["forward_m"],
        "estimated_range_m": raw_payload["range_m"],
        "corrected_right_m": corrected["right_m"],
        "corrected_down_m": corrected["down_m"],
        "corrected_forward_m": corrected["forward_m"],
        "corrected_range_m": corrected["range_m"],
        "bbox_adjustment": None,
        "profile_status": status,
    }
    promotion_check.StereoDistanceNode._publish_estimated_position(node, metrics)
    if not corrected_pub.messages:
        return None, status
    payload = [float(value) for value in corrected_pub.messages[-1].data]
    return payload, status


def sample_rows(profile_json: Path, sample_csv: Path, *, split: str, max_samples: int) -> tuple[list[dict[str, object]], int]:
    profile_payload = promotion_check.load_profile_payload(profile_json)
    profile = promotion_check.load_runtime_profile(profile_payload)
    rows: list[dict[str, object]] = []
    invalid = 0
    with sample_csv.open(newline="", encoding="utf-8") as handle:
        for row_index, row in enumerate(csv.DictReader(handle)):
            if split and row.get("split", "") != split:
                continue
            raw = promotion_check.raw_rdf_from_row(row)
            disparity = promotion_check.disparity_from_row(row)
            if raw is None or disparity is None:
                invalid += 1
                continue
            raw_payload = {
                "right_m": raw[0],
                "down_m": raw[1],
                "forward_m": raw[2],
                "range_m": math.sqrt(raw[0] * raw[0] + raw[1] * raw[1] + raw[2] * raw[2]),
                "disparity_px": disparity,
                "raw_disparity_px": disparity,
            }
            left_point = promotion_check.left_image_point_from_row(row)
            if left_point is not None:
                raw_payload["left_u_px"] = left_point[0]
                raw_payload["left_v_px"] = left_point[1]
            result = profile.apply(raw_payload)
            corrected = result.corrected
            if not isinstance(corrected, Mapping):
                invalid += 1
                continue
            try:
                public_payload = public_payload_from_corrected(corrected)
            except (KeyError, TypeError, ValueError):
                invalid += 1
                continue
            node_payload, node_status = node_apply_corrected_payload(profile_json, raw_payload)
            node_equivalent = (
                node_payload is not None
                and len(node_payload) == len(public_payload)
                and all(
                    abs(float(a) - float(b)) <= NODE_FLOAT32_EQUIVALENCE_TOLERANCE_M
                    for a, b in zip(node_payload, public_payload)
                )
            )
            if not node_equivalent:
                invalid += 1
                continue
            if not all(finite(raw_payload[key]) for key in ("right_m", "down_m", "forward_m", "range_m", "disparity_px")):
                invalid += 1
                continue
            rows.append(
                {
                    "row_index": row_index,
                    "split": row.get("split", ""),
                    "topic": PUBLIC_TOPIC,
                    "message_type": "std_msgs/Float32MultiArray",
                    "data_axis_order": ",".join(PUBLIC_AXIS_ORDER),
                    "data": json.dumps(public_payload, ensure_ascii=False, allow_nan=False),
                    "public_forward_m": public_payload[0],
                    "public_right_m": public_payload[1],
                    "public_down_m": public_payload[2],
                    "public_range_m": public_payload[3],
                    "raw_right_m": raw_payload["right_m"],
                    "raw_down_m": raw_payload["down_m"],
                    "raw_forward_m": raw_payload["forward_m"],
                    "raw_range_m": raw_payload["range_m"],
                    "raw_disparity_px": raw_payload["disparity_px"],
                    "profile_fallback_level": result.status.get("fallback_level"),
                    "profile_fallback_reason": result.status.get("fallback_reason"),
                    "node_public_payload_equivalent": node_equivalent,
                    "node_float32_equivalence_tolerance_m": NODE_FLOAT32_EQUIVALENCE_TOLERANCE_M,
                    "node_profile_fallback_level": node_status.get("fallback_level"),
                    "node_profile_fallback_reason": node_status.get("fallback_reason"),
                    "runtime_truth_used": False,
                    "input_coordinate_contract": RDF_CONTRACT,
                    "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
                }
            )
            if len(rows) >= max_samples:
                break
    return rows, invalid


def write_csv(path: Path, rows: Sequence[Mapping[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "row_index",
        "split",
        "topic",
        "message_type",
        "data_axis_order",
        "data",
        "public_forward_m",
        "public_right_m",
        "public_down_m",
        "public_range_m",
        "raw_right_m",
        "raw_down_m",
        "raw_forward_m",
        "raw_range_m",
        "raw_disparity_px",
        "profile_fallback_level",
        "profile_fallback_reason",
        "node_public_payload_equivalent",
        "node_float32_equivalence_tolerance_m",
        "node_profile_fallback_level",
        "node_profile_fallback_reason",
        "runtime_truth_used",
        "input_coordinate_contract",
        "public_topic_contract",
    ]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def write_text(path: Path, payload: Mapping[str, object]) -> None:
    lines = [
        "StereoDistance public corrected-position payload dry-run",
        f"status: {payload.get('status', '')}",
        f"profile_json: {payload.get('profile_json', '')}",
        f"sample_csv: {payload.get('sample_csv', '')}",
        f"topic: {payload.get('topic', '')}",
        f"message_type: {payload.get('message_type', '')}",
        f"data_axis_order: {payload.get('data_axis_order', '')}",
        f"sample_count: {payload.get('sample_count', 0)}",
        f"invalid_count: {payload.get('invalid_count', 0)}",
        f"node_public_payload_equivalence_status: {payload.get('node_public_payload_equivalence_status', '')}",
        f"node_float32_equivalence_tolerance_m: {payload.get('node_float32_equivalence_tolerance_m', '')}",
        f"runtime_truth_used: {payload.get('runtime_truth_used', False)}",
        f"input_coordinate_contract: {payload.get('input_coordinate_contract', '')}",
        f"public_topic_contract: {payload.get('public_topic_contract', '')}",
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def export_payloads(
    *,
    profile_json: Path,
    sample_csv: Path,
    output_dir: Path,
    split: str = "validation",
    max_samples: int = 25,
) -> dict[str, object]:
    rows, invalid_count = sample_rows(profile_json, sample_csv, split=split, max_samples=max_samples)
    status = "PASS" if rows and invalid_count == 0 else "FAIL"
    summary = {
        "mode": "stereodistance_public_corrected_position_payload_dry_run",
        "status": status,
        "profile_json": str(profile_json),
        "sample_csv": str(sample_csv),
        "topic": PUBLIC_TOPIC,
        "message_type": "std_msgs/Float32MultiArray",
        "data_axis_order": list(PUBLIC_AXIS_ORDER),
        "sample_count": len(rows),
        "invalid_count": invalid_count,
        "node_public_payload_equivalence_status": "PASS"
        if rows and all(row.get("node_public_payload_equivalent") is True for row in rows)
        else "FAIL",
        "node_float32_equivalence_tolerance_m": NODE_FLOAT32_EQUIVALENCE_TOLERANCE_M,
        "runtime_truth_used": False,
        "input_coordinate_contract": RDF_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "payload_csv": str(output_dir / "public_corrected_position_payload.csv"),
    }
    output_dir.mkdir(parents=True, exist_ok=True)
    write_csv(output_dir / "public_corrected_position_payload.csv", rows)
    write_json(output_dir / "public_corrected_position_payload_summary.json", summary)
    write_text(output_dir / "public_corrected_position_payload_summary.txt", summary)
    return summary


def run(argv: Sequence[str] | None = None) -> dict[str, object]:
    args = parse_args(argv)
    return export_payloads(
        profile_json=args.profile_json,
        sample_csv=args.sample_csv,
        output_dir=args.output_dir,
        split=args.split,
        max_samples=args.max_samples,
    )


def main() -> None:
    summary = run()
    print(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False))
    if summary["status"] != "PASS":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
