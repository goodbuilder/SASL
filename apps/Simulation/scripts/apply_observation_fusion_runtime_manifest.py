#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import sys
from collections import defaultdict
from pathlib import Path
from typing import Iterable, Mapping


SCRIPT_DIR = Path(__file__).resolve().parent
ROOT_DIR = SCRIPT_DIR.parent
PACKAGE_DIR = ROOT_DIR / "stereo_ground_sim"
if str(PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(PACKAGE_DIR))

from stereo_ground_sim.runtime_deployment_selector import (  # noqa: E402
    MultiBurstObservationFusion,
    RepeatedObservationFusion,
    RuntimeStereoCameraContract,
    load_observation_fusion_adapter_from_manifest,
)


RDF_AXIS_CONTRACT = "stereo_center_opencv_optical:RDF(right,down,forward)"
IMAGE_CONTRACT = "left image pixel center u/v and measured stereo disparity in pixels"
PUBLIC_TOPIC_CONTRACT = "stereodistance_position_topics:FRD_range(forward,right,down,range)"


def read_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def group_rows(rows: Iterable[Mapping[str, object]], group_field: str) -> dict[str, list[Mapping[str, object]]]:
    grouped: dict[str, list[Mapping[str, object]]] = defaultdict(list)
    for index, row in enumerate(rows):
        group_value = row.get(group_field, "default")
        group_key = str(group_value) if group_value not in (None, "") else "default"
        if group_field not in row:
            group_key = "default"
        grouped[group_key].append(row)
    if not grouped:
        grouped["default"] = []
    return dict(grouped)


def burst_groups(rows: Iterable[Mapping[str, object]], burst_field: str) -> list[list[Mapping[str, object]]]:
    grouped: dict[str, list[Mapping[str, object]]] = defaultdict(list)
    for index, row in enumerate(rows):
        burst_value = row.get(burst_field, index)
        burst_key = str(burst_value) if burst_value not in (None, "") else str(index)
        grouped[burst_key].append(row)
    return [grouped[key] for key in sorted(grouped.keys())]


def sample_to_row(
    *,
    group_id: str,
    tier_name: str,
    method: str,
    adapter_class: str,
    sample: object | None,
) -> dict[str, object]:
    base = {
        "group_id": group_id,
        "tier": tier_name,
        "method": method,
        "adapter_class": adapter_class,
        "sample_valid": sample is not None,
        "runtime_truth_used": False,
        "image_contract": IMAGE_CONTRACT,
        "rdf_axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
    }
    if sample is None:
        base.update(
            {
                "raw_right_m": "",
                "raw_down_m": "",
                "raw_forward_m": "",
                "raw_range_m": "",
                "measured_disparity_px": "",
                "left_u_px": "",
                "left_v_px": "",
            }
        )
        return base
    base.update(
        {
            "raw_right_m": sample.raw_right_m,
            "raw_down_m": sample.raw_down_m,
            "raw_forward_m": sample.raw_forward_m,
            "raw_range_m": sample.raw_range_m,
            "measured_disparity_px": sample.measured_disparity_px,
            "left_u_px": "" if sample.left_u_px is None else sample.left_u_px,
            "left_v_px": "" if sample.left_v_px is None else sample.left_v_px,
        }
    )
    return base


def apply_adapter_to_groups(
    *,
    adapter: RepeatedObservationFusion | MultiBurstObservationFusion,
    tier_payload: Mapping[str, object],
    rows: Iterable[Mapping[str, object]],
    group_field: str,
    burst_field: str,
) -> list[dict[str, object]]:
    grouped = group_rows(rows, group_field)
    output_rows: list[dict[str, object]] = []
    tier_name = str(tier_payload.get("tier", ""))
    method = str(tier_payload.get("method", ""))
    adapter_class = type(adapter).__name__
    for group_id in sorted(grouped.keys()):
        group = grouped[group_id]
        if isinstance(adapter, RepeatedObservationFusion):
            sample = adapter.fuse_rows(group)
        elif isinstance(adapter, MultiBurstObservationFusion):
            sample = adapter.fuse_row_groups(burst_groups(group, burst_field))
        else:
            raise TypeError(f"unsupported adapter instance: {type(adapter).__name__}")
        output_rows.append(
            sample_to_row(
                group_id=group_id,
                tier_name=tier_name,
                method=method,
                adapter_class=adapter_class,
                sample=sample,
            )
        )
    return output_rows


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


def write_summary(path: Path, rows: list[Mapping[str, object]], *, manifest_path: Path, tier_name: str) -> None:
    valid_count = sum(1 for row in rows if row.get("sample_valid") is True)
    lines = [
        "Observation fusion runtime manifest apply",
        f"manifest: {manifest_path}",
        f"tier: {tier_name}",
        f"valid_samples: {valid_count}/{len(rows)}",
        "runtime_truth_used: False",
        "",
        "Coordinate contract: image u/v/disparity -> stereo-center RDF right/down/forward.",
        "Public StereoDistance FRD and PX4/RTK truth are not runtime adapter inputs.",
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Apply observation-fusion runtime manifest to image observation CSV.")
    parser.add_argument("--manifest-json", type=Path, required=True)
    parser.add_argument("--input-csv", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--tier", default=None)
    parser.add_argument("--group-field", default="group_id")
    parser.add_argument("--burst-field", default="burst_id")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    adapter, tier_payload = load_observation_fusion_adapter_from_manifest(args.manifest_json, tier=args.tier)
    rows = read_rows(args.input_csv)
    output_rows = apply_adapter_to_groups(
        adapter=adapter,
        tier_payload=tier_payload,
        rows=rows,
        group_field=args.group_field,
        burst_field=args.burst_field,
    )
    args.output_dir.mkdir(parents=True, exist_ok=True)
    output_csv = args.output_dir / "observation_fusion_runtime_samples.csv"
    write_csv(output_csv, output_rows)
    summary = {
        "manifest_json": str(args.manifest_json),
        "input_csv": str(args.input_csv),
        "tier": tier_payload.get("tier", ""),
        "adapter_class": type(adapter).__name__,
        "group_count": len(output_rows),
        "valid_sample_count": sum(1 for row in output_rows if row.get("sample_valid") is True),
        "runtime_truth_used": False,
        "coordinate_contracts_explicit": True,
        "output_csv": str(output_csv),
    }
    (args.output_dir / "observation_fusion_runtime_apply_summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    write_summary(
        args.output_dir / "observation_fusion_runtime_apply_summary.txt",
        output_rows,
        manifest_path=args.manifest_json,
        tier_name=str(tier_payload.get("tier", "")),
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
