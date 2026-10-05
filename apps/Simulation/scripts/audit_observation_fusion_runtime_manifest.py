#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
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
    RuntimeImageObservation,
    RuntimeStereoCameraContract,
    load_observation_fusion_adapter_from_manifest,
)


RDF_AXIS_CONTRACT = "stereo_center_opencv_optical:RDF(right,down,forward)"
IMAGE_CONTRACT = "left image pixel center u/v and measured stereo disparity in pixels"
PUBLIC_TOPIC_CONTRACT = "stereodistance_position_topics:FRD_range(forward,right,down,range)"


def observation_row_for_rdf(
    *,
    camera: RuntimeStereoCameraContract,
    right_m: float,
    down_m: float,
    forward_m: float,
    u_offset_px: float = 0.0,
    v_offset_px: float = 0.0,
    disparity_scale: float = 1.0,
) -> dict[str, float]:
    disparity_px = camera.fx_px * camera.baseline_m / float(forward_m) * float(disparity_scale)
    left_u_px = camera.cx_px + (float(right_m) + camera.baseline_m * 0.5) * camera.fx_px / float(forward_m)
    left_v_px = camera.cy_px + float(down_m) * camera.fy_px / float(forward_m)
    return {
        "left_u_px": left_u_px + float(u_offset_px),
        "left_v_px": left_v_px + float(v_offset_px),
        "measured_disparity_px": max(float(disparity_px), 1.0e-6),
    }


def repeated_rows(
    *,
    camera: RuntimeStereoCameraContract,
    min_observations: int,
    right_m: float,
    down_m: float,
    forward_m: float,
) -> list[dict[str, float]]:
    rows: list[dict[str, float]] = []
    offsets = [-0.2, 0.0, 0.2]
    for index in range(max(1, int(min_observations))):
        offset = offsets[index % len(offsets)]
        rows.append(
            observation_row_for_rdf(
                camera=camera,
                right_m=right_m,
                down_m=down_m,
                forward_m=forward_m,
                u_offset_px=offset,
                v_offset_px=-offset,
            )
        )
    return rows


def multiburst_row_groups(
    *,
    camera: RuntimeStereoCameraContract,
    min_bursts: int,
    min_observations_per_burst: int,
    right_m: float,
    down_m: float,
    forward_m: float,
) -> list[list[dict[str, float]]]:
    burst_offsets = [-1.0, 0.0, 1.0, -0.5, 0.5]
    groups: list[list[dict[str, float]]] = []
    for burst_index in range(max(1, int(min_bursts))):
        common_offset = burst_offsets[burst_index % len(burst_offsets)]
        burst: list[dict[str, float]] = []
        for obs_index in range(max(1, int(min_observations_per_burst))):
            within_offset = 0.1 * ((obs_index % 3) - 1)
            burst.append(
                observation_row_for_rdf(
                    camera=camera,
                    right_m=right_m,
                    down_m=down_m,
                    forward_m=forward_m,
                    u_offset_px=common_offset + within_offset,
                    v_offset_px=within_offset,
                )
            )
        groups.append(burst)
    return groups


def absolute_errors(sample: object, *, right_m: float, down_m: float, forward_m: float) -> dict[str, float]:
    return {
        "right_abs_error_m": abs(float(sample.raw_right_m) - float(right_m)),
        "down_abs_error_m": abs(float(sample.raw_down_m) - float(down_m)),
        "forward_abs_error_m": abs(float(sample.raw_forward_m) - float(forward_m)),
    }


def audit_tier(
    *,
    manifest_path: Path,
    tier_name: str,
    camera: RuntimeStereoCameraContract,
    right_m: float,
    down_m: float,
    forward_m: float,
    tolerance_m: float,
) -> dict[str, object]:
    adapter, tier_payload = load_observation_fusion_adapter_from_manifest(
        manifest_path,
        tier=tier_name,
        camera=camera,
    )
    if isinstance(adapter, RepeatedObservationFusion):
        rows = repeated_rows(
            camera=camera,
            min_observations=adapter.min_observations,
            right_m=right_m,
            down_m=down_m,
            forward_m=forward_m,
        )
        sample = adapter.fuse_rows(rows)
        adapter_class = "RepeatedObservationFusion"
        observation_shape = f"{len(rows)}"
    elif isinstance(adapter, MultiBurstObservationFusion):
        row_groups = multiburst_row_groups(
            camera=camera,
            min_bursts=adapter.min_bursts,
            min_observations_per_burst=adapter.min_observations_per_burst,
            right_m=right_m,
            down_m=down_m,
            forward_m=forward_m,
        )
        sample = adapter.fuse_row_groups(row_groups)
        adapter_class = "MultiBurstObservationFusion"
        observation_shape = f"{len(row_groups)}x{len(row_groups[0]) if row_groups else 0}"
    else:
        raise TypeError(f"unsupported adapter instance: {type(adapter).__name__}")
    row: dict[str, object] = {
        "tier": tier_name,
        "method": tier_payload.get("method", ""),
        "adapter_class": adapter_class,
        "observation_shape": observation_shape,
        "sample_valid": sample is not None,
        "runtime_truth_used": False,
        "image_contract": IMAGE_CONTRACT,
        "rdf_axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
    }
    if sample is None:
        row.update(
            {
                "status": "FAIL",
                "raw_right_m": float("nan"),
                "raw_down_m": float("nan"),
                "raw_forward_m": float("nan"),
                "right_abs_error_m": float("nan"),
                "down_abs_error_m": float("nan"),
                "forward_abs_error_m": float("nan"),
                "max_abs_axis_error_m": float("nan"),
            }
        )
        return row
    errors = absolute_errors(sample, right_m=right_m, down_m=down_m, forward_m=forward_m)
    max_error = max(errors.values())
    row.update(
        {
            "status": "PASS" if max_error <= float(tolerance_m) else "FAIL",
            "raw_right_m": sample.raw_right_m,
            "raw_down_m": sample.raw_down_m,
            "raw_forward_m": sample.raw_forward_m,
            **errors,
            "max_abs_axis_error_m": max_error,
        }
    )
    return row


def manifest_tier_names(manifest_path: Path) -> list[str]:
    payload = json.loads(manifest_path.read_text(encoding="utf-8"))
    tiers = payload.get("tiers", [])
    if not isinstance(tiers, list):
        raise ValueError("manifest tiers must be a list")
    return [str(tier.get("tier", "")) for tier in tiers if isinstance(tier, Mapping)]


def write_csv(path: Path, rows: Iterable[Mapping[str, object]]) -> None:
    row_list = list(rows)
    path.parent.mkdir(parents=True, exist_ok=True)
    if not row_list:
        path.write_text("", encoding="utf-8")
        return
    fieldnames = list(dict.fromkeys(key for row in row_list for key in row.keys()))
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(row_list)


def write_summary(path: Path, rows: list[Mapping[str, object]], manifest_path: Path) -> None:
    pass_count = sum(1 for row in rows if row.get("status") == "PASS")
    lines = [
        "Observation fusion runtime manifest audit",
        f"manifest: {manifest_path}",
        f"pass_count: {pass_count}/{len(rows)}",
        "runtime_truth_used: False",
        "",
    ]
    for row in rows:
        lines.append(
            "- {tier}: {status}, adapter={adapter}, shape={shape}, max_axis_error={error:.6f}m".format(
                tier=row.get("tier", ""),
                status=row.get("status", ""),
                adapter=row.get("adapter_class", ""),
                shape=row.get("observation_shape", ""),
                error=float(row.get("max_abs_axis_error_m", float("nan"))),
            )
        )
    lines.extend(
        [
            "",
            "Coordinate contract: image u/v/disparity -> stereo-center RDF right/down/forward.",
            "Public StereoDistance FRD and PX4/RTK truth are not runtime adapter inputs.",
        ]
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Audit observation-fusion manifest runtime loader and RDF reconstruction.")
    parser.add_argument("--manifest-json", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--right-m", type=float, default=4.0)
    parser.add_argument("--down-m", type=float, default=1.5)
    parser.add_argument("--forward-m", type=float, default=800.0)
    parser.add_argument("--tolerance-m", type=float, default=0.2)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    camera = RuntimeStereoCameraContract()
    rows = [
        audit_tier(
            manifest_path=args.manifest_json,
            tier_name=tier_name,
            camera=camera,
            right_m=args.right_m,
            down_m=args.down_m,
            forward_m=args.forward_m,
            tolerance_m=args.tolerance_m,
        )
        for tier_name in manifest_tier_names(args.manifest_json)
    ]
    args.output_dir.mkdir(parents=True, exist_ok=True)
    write_csv(args.output_dir / "observation_fusion_runtime_manifest_audit.csv", rows)
    write_summary(args.output_dir / "observation_fusion_runtime_manifest_audit_summary.txt", rows, args.manifest_json)
    payload = {
        "manifest_json": str(args.manifest_json),
        "pass_count": sum(1 for row in rows if row.get("status") == "PASS"),
        "tier_count": len(rows),
        "runtime_truth_used": False,
        "coordinate_contracts_explicit": True,
        "rows": rows,
    }
    (args.output_dir / "observation_fusion_runtime_manifest_audit.json").write_text(
        json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
