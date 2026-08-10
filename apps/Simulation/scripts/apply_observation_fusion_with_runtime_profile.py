#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path
from typing import Mapping

import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
ROOT_DIR = SCRIPT_DIR.parent
PACKAGE_DIR = ROOT_DIR / "stereo_ground_sim"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))
if str(PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(PACKAGE_DIR))

import apply_observation_fusion_runtime_manifest as fusion_apply  # noqa: E402
import audit_runtime_profile_fresh_seed_replay as profile_replay  # noqa: E402


RDF_AXIS_CONTRACT = "stereo_center_opencv_optical:RDF(right,down,forward)"
IMAGE_CONTRACT = "left image pixel center u/v and measured stereo disparity in pixels"
PUBLIC_TOPIC_CONTRACT = "stereodistance_position_topics:FRD_range(forward,right,down,range)"


def rdf_to_public_frd(
    *,
    right_m: float,
    down_m: float,
    forward_m: float,
) -> dict[str, float]:
    range_m = float(np.linalg.norm([right_m, down_m, forward_m]))
    return {
        "forward_m": float(forward_m),
        "right_m": float(right_m),
        "down_m": float(down_m),
        "range_m": range_m,
    }


def profile_payload_from_fused_row(row: Mapping[str, object]) -> dict[str, float] | None:
    try:
        right = float(row["raw_right_m"])
        down = float(row["raw_down_m"])
        forward = float(row["raw_forward_m"])
        disparity = float(row["measured_disparity_px"])
        left_u = float(row["left_u_px"])
        left_v = float(row["left_v_px"])
    except (KeyError, TypeError, ValueError):
        return None
    if not all(math.isfinite(value) for value in (right, down, forward, disparity, left_u, left_v)):
        return None
    return {
        "right_m": right,
        "down_m": down,
        "forward_m": forward,
        "range_m": float(np.linalg.norm([right, down, forward])),
        "disparity_px": disparity,
        "left_u_px": left_u,
        "left_v_px": left_v,
    }


def apply_profile_to_fused_rows(
    *,
    profile: profile_replay.RuntimeProfile,
    fused_rows: list[Mapping[str, object]],
) -> list[dict[str, object]]:
    output_rows: list[dict[str, object]] = []
    for row in fused_rows:
        out = dict(row)
        payload = profile_payload_from_fused_row(row)
        if payload is None:
            out.update(
                {
                    "profile_valid": False,
                    "corrected_right_m": "",
                    "corrected_down_m": "",
                    "corrected_forward_m": "",
                    "corrected_range_m": "",
                    "raw_public_forward_m": "",
                    "raw_public_right_m": "",
                    "raw_public_down_m": "",
                    "raw_public_range_m": "",
                    "corrected_public_forward_m": "",
                    "corrected_public_right_m": "",
                    "corrected_public_down_m": "",
                    "corrected_public_range_m": "",
                    "profile_fallback_level": "",
                    "profile_fallback_reason": "invalid_fused_payload",
                }
            )
            output_rows.append(out)
            continue
        raw_public = rdf_to_public_frd(
            right_m=payload["right_m"],
            down_m=payload["down_m"],
            forward_m=payload["forward_m"],
        )
        result = profile.apply(payload)
        if result.corrected is None:
            out.update(
                {
                    "profile_valid": False,
                    "corrected_right_m": "",
                    "corrected_down_m": "",
                    "corrected_forward_m": "",
                    "corrected_range_m": "",
                    "raw_public_forward_m": raw_public["forward_m"],
                    "raw_public_right_m": raw_public["right_m"],
                    "raw_public_down_m": raw_public["down_m"],
                    "raw_public_range_m": raw_public["range_m"],
                    "corrected_public_forward_m": "",
                    "corrected_public_right_m": "",
                    "corrected_public_down_m": "",
                    "corrected_public_range_m": "",
                    "profile_fallback_level": result.status.get("fallback_level", ""),
                    "profile_fallback_reason": result.status.get("fallback_reason", "profile_returned_none"),
                }
            )
        else:
            corrected = result.corrected
            corrected_right = float(corrected["right_m"])
            corrected_down = float(corrected["down_m"])
            corrected_forward = float(corrected["forward_m"])
            corrected_public = rdf_to_public_frd(
                right_m=corrected_right,
                down_m=corrected_down,
                forward_m=corrected_forward,
            )
            out.update(
                {
                    "profile_valid": True,
                    "corrected_right_m": corrected_right,
                    "corrected_down_m": corrected_down,
                    "corrected_forward_m": corrected_forward,
                    "corrected_range_m": float(np.linalg.norm([corrected_right, corrected_down, corrected_forward])),
                    "raw_public_forward_m": raw_public["forward_m"],
                    "raw_public_right_m": raw_public["right_m"],
                    "raw_public_down_m": raw_public["down_m"],
                    "raw_public_range_m": raw_public["range_m"],
                    "corrected_public_forward_m": corrected_public["forward_m"],
                    "corrected_public_right_m": corrected_public["right_m"],
                    "corrected_public_down_m": corrected_public["down_m"],
                    "corrected_public_range_m": corrected_public["range_m"],
                    "profile_fallback_level": result.status.get("fallback_level", 0),
                    "profile_fallback_reason": result.status.get("fallback_reason", ""),
                }
            )
        out.update(
            {
                "runtime_truth_used": False,
                "image_contract": IMAGE_CONTRACT,
                "rdf_axis_contract": RDF_AXIS_CONTRACT,
                "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
            }
        )
        output_rows.append(out)
    return output_rows


def write_summary(
    path: Path,
    rows: list[Mapping[str, object]],
    *,
    manifest_path: Path,
    profile_path: Path,
    tier_name: str,
) -> None:
    valid_fused = sum(1 for row in rows if row.get("sample_valid") is True)
    valid_profile = sum(1 for row in rows if row.get("profile_valid") is True)
    lines = [
        "Observation fusion + runtime profile apply",
        f"manifest: {manifest_path}",
        f"profile: {profile_path}",
        f"tier: {tier_name}",
        f"valid_fused_samples: {valid_fused}/{len(rows)}",
        f"valid_profile_outputs: {valid_profile}/{len(rows)}",
        "runtime_truth_used: False",
        "",
        "Coordinate contract: image u/v/disparity -> raw stereo-center RDF -> corrected stereo-center RDF.",
        "Public output aliases are FRD+range: forward/right/down/range.",
        "Public StereoDistance FRD and PX4/RTK truth are not runtime inputs.",
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Apply observation-fusion manifest and runtime correction profile to image observation CSV.")
    parser.add_argument("--manifest-json", type=Path, required=True)
    parser.add_argument("--profile-json", type=Path, required=True)
    parser.add_argument("--input-csv", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--tier", default=None)
    parser.add_argument("--group-field", default="group_id")
    parser.add_argument("--burst-field", default="burst_id")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    adapter, tier_payload = fusion_apply.load_observation_fusion_adapter_from_manifest(args.manifest_json, tier=args.tier)
    rows = fusion_apply.read_rows(args.input_csv)
    fused_rows = fusion_apply.apply_adapter_to_groups(
        adapter=adapter,
        tier_payload=tier_payload,
        rows=rows,
        group_field=args.group_field,
        burst_field=args.burst_field,
    )
    profile = profile_replay.load_profile(args.profile_json)
    output_rows = apply_profile_to_fused_rows(profile=profile, fused_rows=fused_rows)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    output_csv = args.output_dir / "observation_fusion_corrected_runtime_samples.csv"
    fusion_apply.write_csv(output_csv, output_rows)
    summary = {
        "manifest_json": str(args.manifest_json),
        "profile_json": str(args.profile_json),
        "input_csv": str(args.input_csv),
        "tier": tier_payload.get("tier", ""),
        "adapter_class": type(adapter).__name__,
        "group_count": len(output_rows),
        "valid_fused_sample_count": sum(1 for row in output_rows if row.get("sample_valid") is True),
        "valid_profile_output_count": sum(1 for row in output_rows if row.get("profile_valid") is True),
        "runtime_truth_used": False,
        "coordinate_contracts_explicit": True,
        "output_csv": str(output_csv),
    }
    (args.output_dir / "observation_fusion_profile_apply_summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    write_summary(
        args.output_dir / "observation_fusion_profile_apply_summary.txt",
        output_rows,
        manifest_path=args.manifest_json,
        profile_path=args.profile_json,
        tier_name=str(tier_payload.get("tier", "")),
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
