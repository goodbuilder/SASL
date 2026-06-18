#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import sys
from collections import deque
from pathlib import Path
from typing import Mapping, Sequence

import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
ROOT_DIR = SCRIPT_DIR.parent
PACKAGE_DIR = ROOT_DIR / "stereo_ground_sim"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))
if str(PACKAGE_DIR) not in sys.path:
    sys.path.insert(0, str(PACKAGE_DIR))

from stereo_ground_sim.runtime_deployment_selector import (  # noqa: E402
    MultiBurstObservationFusion,
    RepeatedObservationFusion,
    RuntimeStereoCameraContract,
    load_observation_fusion_adapter_from_manifest,
)
from stereo_ground_sim.stereo_distance_node import StereoDistanceNode  # noqa: E402


class _Logger:
    def warning(self, msg: str) -> None:
        pass

    def info(self, msg: str) -> None:
        pass


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Smoke-test StereoDistance node observation-fusion runtime path.")
    parser.add_argument("--manifest-json", type=Path, required=True)
    parser.add_argument("--profile-json", type=Path)
    parser.add_argument("--disparity-residual-profile-json", type=Path)
    parser.add_argument("--disparity-residual-feature-selector-json", type=Path)
    parser.add_argument("--disparity-residual-feature-catalog-json", type=Path)
    parser.add_argument("--disparity-residual-feature-selector-window-size", type=int, default=1)
    parser.add_argument("--disparity-residual-feature-selector-min-samples", type=int, default=1)
    parser.add_argument("--tier", default="")
    parser.add_argument("--output-json", type=Path)
    parser.add_argument("--output-txt", type=Path)
    return parser.parse_args(argv)


def required_shape(adapter: object) -> tuple[int, int]:
    if isinstance(adapter, MultiBurstObservationFusion):
        return int(adapter.min_bursts), int(adapter.min_observations_per_burst)
    if isinstance(adapter, RepeatedObservationFusion):
        return 1, int(adapter.min_observations)
    return 1, 1


def make_observation_centers(
    *,
    camera: RuntimeStereoCameraContract,
    burst_count: int,
    observations_per_burst: int,
    right_m: float = 4.0,
    down_m: float = 1.5,
    forward_m: float = 800.0,
) -> list[tuple[float, float, float]]:
    disparity_px = camera.fx_px * camera.baseline_m / forward_m
    left_u_px = camera.cx_px + (right_m + camera.baseline_m * 0.5) * camera.fx_px / forward_m
    left_v_px = camera.cy_px + down_m * camera.fy_px / forward_m
    rows: list[tuple[float, float, float]] = []
    common_offsets = (-1.0, 0.0, 1.0, -0.5, 0.5)
    for burst_index in range(max(1, burst_count)):
        common_offset = common_offsets[burst_index % len(common_offsets)]
        for obs_index in range(max(1, observations_per_burst)):
            within_offset = 0.1 * ((obs_index % 3) - 1)
            rows.append((left_u_px + common_offset + within_offset, left_v_px + within_offset, disparity_px))
    return rows


def load_profile(node: StereoDistanceNode, profile_json: Path | None) -> object | None:
    if profile_json is None:
        return None
    node.enable_bbox_center_adjustment = True
    node.bbox_center_adjustment_path = str(profile_json)
    return StereoDistanceNode._load_bbox_center_adjustment(node)


def load_residual_profile(node: StereoDistanceNode, residual_profile_json: Path | None) -> object | None:
    if residual_profile_json is None:
        return None
    node.enable_disparity_residual_profile = True
    node.disparity_residual_profile_path = str(residual_profile_json)
    return StereoDistanceNode._load_disparity_residual_profile(node)


def load_residual_feature_selector(
    node: StereoDistanceNode,
    selector_json: Path | None,
    catalog_json: Path | None,
) -> object | None:
    if selector_json is None:
        return None
    node.enable_disparity_residual_feature_selector = True
    node.disparity_residual_feature_selector_model_path = str(selector_json)
    node.disparity_residual_feature_selector_catalog_path = "" if catalog_json is None else str(catalog_json)
    return StereoDistanceNode._load_disparity_residual_feature_selector(node)


def run_smoke(
    *,
    manifest_json: Path,
    profile_json: Path | None = None,
    disparity_residual_profile_json: Path | None = None,
    disparity_residual_feature_selector_json: Path | None = None,
    disparity_residual_feature_catalog_json: Path | None = None,
    disparity_residual_feature_selector_window_size: int = 1,
    disparity_residual_feature_selector_min_samples: int = 1,
    tier: str = "",
) -> dict[str, object]:
    failures: list[str] = []
    camera = RuntimeStereoCameraContract()
    adapter, tier_payload = load_observation_fusion_adapter_from_manifest(
        manifest_json,
        tier=tier.strip() or None,
        camera=camera,
    )
    burst_count, observations_per_burst = required_shape(adapter)
    node = StereoDistanceNode.__new__(StereoDistanceNode)
    node.horizontal_fov_rad = camera.horizontal_fov_rad
    node.baseline_m = camera.baseline_m
    node.yolo_load_error = ""
    node.yolo_class_name = "drone"
    node.get_logger = lambda: _Logger()
    node.observation_fusion_adapter = adapter
    node.observation_fusion_buffer = deque(maxlen=max(1, burst_count * observations_per_burst))
    node.bbox_center_adjustment = load_profile(node, profile_json)
    node.disparity_residual_profile = load_residual_profile(node, disparity_residual_profile_json)
    node.disparity_residual_feature_selector_window_size = max(1, int(disparity_residual_feature_selector_window_size))
    node.disparity_residual_feature_selector_min_samples = max(1, int(disparity_residual_feature_selector_min_samples))
    node.disparity_residual_feature_selector_window = deque(
        maxlen=node.disparity_residual_feature_selector_window_size
    )
    node.disparity_residual_feature_selector = load_residual_feature_selector(
        node,
        disparity_residual_feature_selector_json,
        disparity_residual_feature_catalog_json,
    )
    image = np.zeros((int(camera.height_px), int(camera.width_px), 3), dtype=np.uint8)

    metrics: Mapping[str, object] | None = None
    for left_u_px, left_v_px, disparity_px in make_observation_centers(
        camera=camera,
        burst_count=burst_count,
        observations_per_burst=observations_per_burst,
    ):
        left = {"cx": left_u_px, "cy": left_v_px, "bbox": (left_u_px - 5.0, left_v_px - 5.0, left_u_px + 5.0, left_v_px + 5.0)}
        right_u_px = left_u_px - disparity_px
        right = {"cx": right_u_px, "cy": left_v_px, "bbox": (right_u_px - 5.0, left_v_px - 5.0, right_u_px + 5.0, left_v_px + 5.0)}
        metrics = StereoDistanceNode._calculate_metrics(node, image, left, right, 800.0, "node_smoke")

    if metrics is None:
        failures.append("metrics:missing")
        metrics = {}
    status_msg = StereoDistanceNode._profile_status_to_msg(node, metrics)
    status_payload = json.loads(status_msg.data)
    fusion_status = str(metrics.get("observation_fusion_status", ""))
    estimated_range = metrics.get("estimated_range_m")
    corrected_range = metrics.get("corrected_range_m")
    residual_status = str(metrics.get("disparity_residual_status", "disabled"))
    feature_summary_status = str(metrics.get("disparity_residual_feature_summary_status", "disabled"))
    feature_summary = metrics.get("disparity_residual_feature_summary", {})
    feature_selector_summary_status = str(
        metrics.get("disparity_residual_feature_selector_summary_status", "disabled")
    )
    feature_selector_window_count = int(metrics.get("disparity_residual_feature_selector_window_count", 0))
    feature_selector_status = str(metrics.get("disparity_residual_feature_selector_status", "disabled"))
    feature_selected_label = str(metrics.get("disparity_residual_feature_selected_label", ""))
    if not fusion_status.startswith("fused:"):
        failures.append("observation_fusion:not_fused")
    if not math.isfinite(float(estimated_range)):
        failures.append("estimated_range:not_finite")
    if profile_json is not None and corrected_range is None:
        failures.append("corrected_range:missing_with_profile")
    if profile_json is not None and corrected_range is not None and not math.isfinite(float(corrected_range)):
        failures.append("corrected_range:not_finite")
    if status_payload.get("observation_fusion_adapter") != type(adapter).__name__:
        failures.append("profile_status:adapter_mismatch")
    if disparity_residual_profile_json is not None and not residual_status.startswith("applied:"):
        failures.append("disparity_residual:not_applied")
    if disparity_residual_feature_selector_json is not None and not feature_selector_status.startswith("selected:"):
        failures.append("feature_selector:not_selected")
    if disparity_residual_feature_selector_json is not None and not residual_status.startswith("applied:"):
        failures.append("feature_selector:residual_not_applied")
    if disparity_residual_profile_json is not None and not str(
        status_payload.get("disparity_residual_status", "")
    ).startswith("applied:"):
        failures.append("profile_status:disparity_residual_not_applied")
    if int(status_payload.get("observation_fusion_required_count", -1)) != max(1, burst_count * observations_per_burst):
        failures.append("profile_status:required_count")
    if int(status_payload.get("observation_fusion_buffer_count", -1)) != max(1, burst_count * observations_per_burst):
        failures.append("profile_status:buffer_count")
    if not feature_summary_status.startswith("ready:"):
        failures.append("feature_summary:not_ready")
    if status_payload.get("disparity_residual_feature_summary_status") != feature_summary_status:
        failures.append("profile_status:feature_summary_status")
    if status_payload.get("disparity_residual_feature_selector_summary_status") != feature_selector_summary_status:
        failures.append("profile_status:feature_selector_summary_status")
    if int(status_payload.get("disparity_residual_feature_selector_window_count", -1)) != feature_selector_window_count:
        failures.append("profile_status:feature_selector_window_count")
    if status_payload.get("disparity_residual_feature_selector_status") != feature_selector_status:
        failures.append("profile_status:feature_selector_status")
    if not isinstance(feature_summary, Mapping) or "raw_forward_m_mean" not in feature_summary:
        failures.append("feature_summary:missing_raw_forward")

    return {
        "mode": "stereodistance_observation_fusion_node_smoke",
        "status": "PASS" if not failures else "FAIL",
        "manifest_json": str(manifest_json),
        "profile_json": "" if profile_json is None else str(profile_json),
        "disparity_residual_profile_json": ""
        if disparity_residual_profile_json is None
        else str(disparity_residual_profile_json),
        "disparity_residual_feature_selector_json": ""
        if disparity_residual_feature_selector_json is None
        else str(disparity_residual_feature_selector_json),
        "disparity_residual_feature_catalog_json": ""
        if disparity_residual_feature_catalog_json is None
        else str(disparity_residual_feature_catalog_json),
        "disparity_residual_feature_selector_window_size": node.disparity_residual_feature_selector_window_size,
        "disparity_residual_feature_selector_min_samples": node.disparity_residual_feature_selector_min_samples,
        "tier": tier_payload.get("tier", tier),
        "adapter_class": type(adapter).__name__,
        "burst_count": burst_count,
        "observations_per_burst": observations_per_burst,
        "observation_count": max(1, burst_count * observations_per_burst),
        "observation_fusion_status": fusion_status,
        "estimated_range_m": float(estimated_range) if estimated_range is not None else math.nan,
        "corrected_range_m": None if corrected_range is None else float(corrected_range),
        "disparity_residual_status": residual_status,
        "disparity_residual_feature_summary_status": feature_summary_status,
        "disparity_residual_feature_summary": feature_summary,
        "disparity_residual_feature_selector_summary_status": feature_selector_summary_status,
        "disparity_residual_feature_selector_window_count": feature_selector_window_count,
        "disparity_residual_feature_selector_status": feature_selector_status,
        "disparity_residual_feature_selected_label": feature_selected_label,
        "profile_status": status_payload,
        "runtime_truth_used": False,
        "coordinate_contract": (
            "runtime image left_u_px/left_v_px/measured_disparity_px -> stereo-center RDF(right,down,forward); "
            "node feature summary uses current raw/corrected stereo-center RDF only; public StereoDistance output remains FRD+range"
        ),
        "failure_count": len(failures),
        "failures": failures,
    }


def write_text(path: Path, payload: Mapping[str, object]) -> None:
    lines = [
        "StereoDistance observation-fusion node smoke",
        f"status: {payload.get('status', '')}",
        f"manifest_json: {payload.get('manifest_json', '')}",
        f"profile_json: {payload.get('profile_json', '')}",
        f"disparity_residual_profile_json: {payload.get('disparity_residual_profile_json', '')}",
        f"tier: {payload.get('tier', '')}",
        f"adapter_class: {payload.get('adapter_class', '')}",
        f"observation_count: {payload.get('observation_count', '')}",
        f"observation_fusion_status: {payload.get('observation_fusion_status', '')}",
        f"estimated_range_m: {payload.get('estimated_range_m', '')}",
        f"corrected_range_m: {payload.get('corrected_range_m', '')}",
        f"disparity_residual_status: {payload.get('disparity_residual_status', '')}",
        f"disparity_residual_feature_selector_summary_status: {payload.get('disparity_residual_feature_selector_summary_status', '')}",
        f"disparity_residual_feature_selector_window_count: {payload.get('disparity_residual_feature_selector_window_count', '')}",
        f"disparity_residual_feature_selector_status: {payload.get('disparity_residual_feature_selector_status', '')}",
        f"disparity_residual_feature_selected_label: {payload.get('disparity_residual_feature_selected_label', '')}",
        f"runtime_truth_used: {payload.get('runtime_truth_used', False)}",
        f"coordinate_contract: {payload.get('coordinate_contract', '')}",
        f"failure_count: {payload.get('failure_count', 0)}",
    ]
    failures = payload.get("failures", [])
    if isinstance(failures, list) and failures:
        lines.append("")
        lines.append("Failures:")
        lines.extend(f"- {failure}" for failure in failures)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def run(argv: Sequence[str] | None = None) -> dict[str, object]:
    args = parse_args(argv)
    payload = run_smoke(
        manifest_json=args.manifest_json,
        profile_json=args.profile_json,
        disparity_residual_profile_json=args.disparity_residual_profile_json,
        disparity_residual_feature_selector_json=args.disparity_residual_feature_selector_json,
        disparity_residual_feature_catalog_json=args.disparity_residual_feature_catalog_json,
        disparity_residual_feature_selector_window_size=args.disparity_residual_feature_selector_window_size,
        disparity_residual_feature_selector_min_samples=args.disparity_residual_feature_selector_min_samples,
        tier=args.tier,
    )
    if args.output_json is not None:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    if args.output_txt is not None:
        write_text(args.output_txt, payload)
    return payload


def main() -> None:
    payload = run()
    print(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False))
    if payload["status"] != "PASS":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
