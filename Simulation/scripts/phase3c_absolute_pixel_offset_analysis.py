#!/usr/bin/env python3
"""Offline Phase 3C-B3-A absolute pixel-offset analysis."""
from __future__ import annotations

import argparse
import json
import math
from collections import defaultdict
from pathlib import Path
from typing import Any, Iterable

import numpy as np

BASE_COMMIT = "37d48b309779c7ac008579df09a1941711e1f059"
CANDIDATES = ((0.0, 0.0), (0.5, 0.5), (0.5, -0.5), (-0.5, 0.5), (-0.5, -0.5))
REQUIRED_RADII = (0.5, 1.0, 2.0)
REQUIRED_MARKERS = ("center", "right_positive", "down_positive", "nearer", "farther")
REQUIRED_CAMERAS = ("left", "right")
MODES = ("binary_component_centroid", "weighted_rgb_dominance", "high_confidence_core")
REQUIRED_CONFIGS = {(threshold, "weighted_rgb_dominance") for threshold in (40.0, 60.0, 80.0, 100.0, 120.0)} | {(80.0, mode) for mode in MODES}


def _finite(value: Any) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _field_vector(cell: dict[str, Any], corrected: bool = False) -> tuple[float, float] | None:
    prefix = "corrected_" if corrected else ""
    du = _finite(cell.get(f"{prefix}delta_u_px", cell.get(f"{prefix}du_px")))
    dv = _finite(cell.get(f"{prefix}delta_v_px", cell.get(f"{prefix}dv_px")))
    return (du, dv) if du is not None and dv is not None else None


def residual_vector(rendered: Any, analytic: Any) -> dict[str, float] | tuple[float, float]:
    if isinstance(rendered, dict):
        du = float(rendered["u_px"]) - float(analytic["u_px"])
        dv = float(rendered["v_px"]) - float(analytic["v_px"])
        return {"du_px": du, "dv_px": dv, "norm_px": math.hypot(du, dv)}
    du, dv = float(rendered[0]) - float(analytic[0]), float(rendered[1]) - float(analytic[1])
    return (du, dv)


def _vectors(cells: Iterable[dict[str, Any]], corrected: bool = False) -> list[tuple[float, float]]:
    return [vector for cell in cells if (vector := _field_vector(cell, corrected)) is not None]


def continuous_median_offset(cell_observations: Iterable[dict[str, Any]]) -> tuple[float, float]:
    vectors = _vectors(cell_observations)
    return (math.nan, math.nan) if not vectors else (float(np.median([v[0] for v in vectors])), float(np.median([v[1] for v in vectors])))


def apply_offset_correction(cell_observations: Iterable[dict[str, Any]], candidate: tuple[float, float]) -> list[dict[str, Any]]:
    output = []
    for source in cell_observations:
        cell = dict(source)
        vector = _field_vector(cell)
        if vector is not None:
            cell["corrected_delta_u_px"], cell["corrected_delta_v_px"] = vector[0] - candidate[0], vector[1] - candidate[1]
            cell["corrected_residual_norm_px"] = math.hypot(cell["corrected_delta_u_px"], cell["corrected_delta_v_px"])
        output.append(cell)
    return output


def half_pixel_candidate_scores(cell_observations: Iterable[dict[str, Any]], candidates: Iterable[tuple[float, float]] = CANDIDATES) -> dict[str, Any]:
    cells = list(cell_observations)
    scores = []
    for candidate in candidates:
        corrected = apply_offset_correction(cells, candidate)
        norms = [c["corrected_residual_norm_px"] for c in corrected if _finite(c.get("corrected_residual_norm_px")) is not None]
        scores.append({"candidate_px": list(candidate), "median_residual_norm_px": float(np.median(norms)) if norms else None, "count": len(norms)})
    finite_scores = [item for item in scores if item["median_residual_norm_px"] is not None]
    if not finite_scores:
        return {"scores": scores, "best_candidate_px": None, "tie": False}
    best = min(item["median_residual_norm_px"] for item in finite_scores)
    winners = [item for item in finite_scores if abs(item["median_residual_norm_px"] - best) <= 1e-12]
    return {"scores": scores, "best_candidate_px": winners[0]["candidate_px"] if len(winners) == 1 else None, "tie": len(winners) > 1, "best_median_residual_norm_px": best}


def grouped_offset_summary(cell_observations: Iterable[dict[str, Any]], group_fields: Iterable[str], *, corrected: bool = False) -> dict[str, Any]:
    groups = defaultdict(list)
    for cell in cell_observations:
        groups[tuple(cell.get(field) for field in group_fields)].append(cell)
    result = {}
    for key, cells in groups.items():
        vectors = _vectors(cells, corrected)
        median = (math.nan, math.nan) if not vectors else (float(np.median([v[0] for v in vectors])), float(np.median([v[1] for v in vectors])))
        result["|".join(str(value) for value in key)] = {"group": list(key), "median_delta_px": list(median), "cell_count": len(vectors)}
    return result


def _axis_range(vectors: Iterable[tuple[float, float]]) -> float:
    values = list(vectors)
    return max((max(v[axis] for v in values) - min(v[axis] for v in values) for axis in (0, 1)), default=0.0) if values else 0.0


def evaluate_radius_dependence(corrected_reference_cells: Iterable[dict[str, Any]]) -> dict[str, Any]:
    cells = list(corrected_reference_cells)
    group_medians = grouped_offset_summary(cells, ("radius_m",), corrected=True)
    global_range = _axis_range(tuple(value["median_delta_px"]) for value in group_medians.values() if all(_finite(v) is not None for v in value["median_delta_px"]))
    strata = {}
    for key, values in grouped_offset_summary(cells, ("marker", "camera"), corrected=True).items():
        marker, camera = key.split("|", 1)
        by_radius = defaultdict(list)
        for cell in cells:
            if cell.get("marker") == marker and cell.get("camera") == camera and _field_vector(cell, True) is not None:
                by_radius[round(float(cell["radius_m"]), 6)].append(cell)
        radius_medians = {str(radius): list(continuous_median_offset(items)) for radius, items in sorted(by_radius.items())}
        evaluable = all(str(radius) in radius_medians for radius in REQUIRED_RADII)
        range_u = _axis_range((tuple(value) for value in radius_medians.values())) if evaluable else None
        strata[key] = {"marker": marker, "camera": camera, "radius_medians": radius_medians, "evaluable": evaluable, "max_axis_range_px": range_u, "dependent": bool(evaluable and range_u > 0.1)}
    dependent = sum(item["dependent"] for item in strata.values())
    return {"group_medians": group_medians, "strata": strata, "global_max_axis_range_px": global_range, "evaluable_strata_count": sum(item["evaluable"] for item in strata.values()), "dependent_strata_count": dependent, "detected": global_range > 0.1 and dependent >= 6}


def evaluate_segmentation_dependence(eligible_sweep_cells: Iterable[dict[str, Any]]) -> dict[str, Any]:
    cells = [cell for cell in eligible_sweep_cells if math.isclose(float(cell.get("radius_m", -1)), 1.0)]
    configs = grouped_offset_summary(cells, ("threshold", "centroid_mode"), corrected=True)
    global_range = _axis_range(tuple(value["median_delta_px"]) for value in configs.values())
    strata = {}
    for key in {(cell.get("marker"), cell.get("camera")) for cell in cells}:
        marker, camera = key
        by_config = defaultdict(list)
        for cell in cells:
            if (cell.get("marker"), cell.get("camera")) == key and _field_vector(cell, True) is not None:
                by_config[(float(cell["threshold"]), cell["centroid_mode"])].append(cell)
        medians = {f"{config[0]:g}|{config[1]}": list(continuous_median_offset(items)) for config, items in sorted(by_config.items())}
        range_value = _axis_range(tuple(tuple(value) for value in medians.values())) if medians else 0.0
        strata[f"{marker}|{camera}"] = {"marker": marker, "camera": camera, "configuration_medians": medians, "evaluable": bool(medians), "max_axis_range_px": range_value, "dependent": range_value > 0.1}
    dependent = sum(item["dependent"] for item in strata.values())
    return {"configuration_medians": configs, "cells": strata, "global_max_axis_range_px": global_range, "evaluable_cell_count": sum(item["evaluable"] for item in strata.values()), "dependent_cell_count": dependent, "detected": global_range > 0.1 and dependent >= 5}


def evaluate_configuration_eligibility(cells: Iterable[dict[str, Any]]) -> dict[str, Any]:
    required_keys = {(round(radius, 6), marker, camera) for radius in REQUIRED_RADII for marker in REQUIRED_MARKERS for camera in REQUIRED_CAMERAS}
    details = {}
    for threshold in (40.0, 60.0, 80.0, 100.0, 120.0):
        for mode in MODES:
            config = (threshold, mode)
            selected = [c for c in cells if float(c.get("threshold", -1)) == threshold and c.get("centroid_mode") == mode]
            by_key = {(round(float(c.get("radius_m", -1)), 6), c.get("marker"), c.get("camera")): c for c in selected}
            valid_keys = {key for key, cell in by_key.items() if cell.get("valid") and int(cell.get("valid_frame_count", 0)) >= 19}
            per_radius = {str(radius): sum(key[0] == round(radius, 6) and key in valid_keys for key in required_keys) for radius in REQUIRED_RADII}
            details[f"{threshold:g}|{mode}"] = {"threshold": threshold, "centroid_mode": mode, "valid_cell_count": len(valid_keys), "required_cell_count": 30, "radius_valid_cell_counts": per_radius, "eligible": len(valid_keys) >= 24 and all(count >= 8 for count in per_radius.values())}
    eligible_count = sum(item["eligible"] for item in details.values())
    required_ok = all(details[f"{threshold:g}|{mode}"]["eligible"] for threshold, mode in REQUIRED_CONFIGS)
    return {"details": details, "eligible_configuration_count": eligible_count, "required_configurations_eligible": required_ok, "eligible": eligible_count >= 12 and required_ok}


def intrinsic_variant_pixel(normalized_xy: tuple[float, float], intrinsic: dict[str, float], offset: tuple[float, float] = (0.0, 0.0)) -> tuple[float, float]:
    return (intrinsic["fx"] * normalized_xy[0] + intrinsic["cx"] + offset[0], intrinsic["fy"] * normalized_xy[1] + intrinsic["cy"] + offset[1])


def _summary(values: list[float]) -> dict[str, Any]:
    return {"count": len(values), "median": float(np.median(values)) if values else None, "max": max(values) if values else None, "mean": float(np.mean(values)) if values else None, "std": float(np.std(values)) if values else None}


def _read_json(path: Path) -> dict[str, Any] | None:
    try:
        return json.loads(path.read_text(encoding="utf-8")) if path.exists() else None
    except (OSError, json.JSONDecodeError):
        return None


def _read_cells(directory: Path) -> list[dict[str, Any]]:
    path = directory / "absolute_offset_observations.jsonl"
    if not path.exists():
        return []
    cells = []
    for line in path.read_text(encoding="utf-8").splitlines():
        if line.strip():
            try:
                cells.append(json.loads(line))
            except json.JSONDecodeError:
                pass
    unique = {}
    for cell in cells:
        key = (round(float(cell.get("radius_m", -1)), 6), cell.get("marker"), cell.get("camera"), float(cell.get("threshold", -1)), cell.get("centroid_mode"))
        unique[key] = cell
    return list(unique.values())


def _run_prerequisite(directory: Path, radius: float) -> list[str]:
    failures = []
    run = _read_json(directory / "absolute_offset_run_summary.json")
    info = _read_json(directory / "camera_info.json")
    if run is None:
        failures.append(f"radius_{radius:g}:missing_run_summary")
    else:
        if run.get("evidence_acquisition_valid") is not True: failures.append(f"radius_{radius:g}:evidence_acquisition_invalid")
        if run.get("pose_coverage_rate") != 1.0: failures.append(f"radius_{radius:g}:pose_coverage_below_1")
        if run.get("camera_info_valid") is not True: failures.append(f"radius_{radius:g}:camera_info_invalid")
        coverage = run.get("reference_cell_coverage", {}).get("rate")
        if coverage != 1.0: failures.append(f"radius_{radius:g}:reference_cell_coverage_below_1")
        if _finite(run.get("reference_frame_success_rate", {}).get("min")) is None or run["reference_frame_success_rate"]["min"] < 0.95: failures.append(f"radius_{radius:g}:reference_frame_success_below_0.95")
    if info is None: failures.append(f"radius_{radius:g}:missing_camera_info")
    else:
        for topic in ("/left_camera/camera_info", "/right_camera/camera_info"):
            k = info.get(topic, {}).get("k")
            if not isinstance(k, list) or len(k) < 6 or any(_finite(value) is None for value in k[:6]): failures.append(f"radius_{radius:g}:camera_info_K_missing_{topic}")
    if not (directory / "absolute_offset_observations.jsonl").exists(): failures.append(f"radius_{radius:g}:missing_observations")
    return failures


def _variant_cells(cells: Iterable[dict[str, Any]], candidate: tuple[float, float]) -> tuple[list[dict[str, Any]], list[str]]:
    output, failures = [], []
    for source in cells:
        cell = dict(source)
        required = ["rendered_u_px", "rendered_v_px", "analytic_u_px", "analytic_v_px", "analytic_fx_px", "analytic_fy_px", "analytic_cx_px", "analytic_cy_px", "camera_info_fx_px", "camera_info_fy_px", "camera_info_cx_px", "camera_info_cy_px"]
        values = {key: _finite(cell.get(key)) for key in required}
        if any(value is None for value in values.values()):
            failures.append(f"missing_intrinsic_metadata:{cell.get('radius_m')}:{cell.get('marker')}:{cell.get('camera')}")
            continue
        normalized = ((values["analytic_u_px"] - values["analytic_cx_px"]) / values["analytic_fx_px"], (values["analytic_v_px"] - values["analytic_cy_px"]) / values["analytic_fy_px"])
        cell["normalized_x"], cell["normalized_y"] = normalized
        a = (values["analytic_u_px"], values["analytic_v_px"])
        b = intrinsic_variant_pixel(normalized, {"fx": values["camera_info_fx_px"], "fy": values["camera_info_fy_px"], "cx": values["camera_info_cx_px"], "cy": values["camera_info_cy_px"]})
        rendered = (values["rendered_u_px"], values["rendered_v_px"])
        for name, pixel in (("A", a), ("B", b), ("C", (a[0] + candidate[0], a[1] + candidate[1])), ("D", (b[0] + candidate[0], b[1] + candidate[1]))):
            cell[f"variant_{name}_du_px"], cell[f"variant_{name}_dv_px"] = rendered[0] - pixel[0], rendered[1] - pixel[1]
            cell[f"variant_{name}_residual_norm_px"] = math.hypot(cell[f"variant_{name}_du_px"], cell[f"variant_{name}_dv_px"])
        output.append(cell)
    return output, failures


def _baseline_summary(directory: Path | None) -> dict[str, Any]:
    if directory is None or not directory.exists(): return {"available": False}
    cells = _read_cells(directory)
    if not cells:
        case_path = directory / "case_results.jsonl"
        if case_path.exists():
            for line in case_path.read_text(encoding="utf-8").splitlines():
                try:
                    result = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if result.get("case") != "zero" or not result.get("valid"): continue
                analytic = result.get("analytic_projection", {})
                for camera, topic in (("left", "/left_camera/image"), ("right", "/right_camera/image")):
                    sample = result.get("sample", {}).get("cameras", {}).get(topic, {})
                    if sample.get("valid") and analytic.get(camera, {}).get("u_px") is not None:
                        cells.append({"delta_u_px": float(sample["u_px"]) - float(analytic[camera]["u_px"]), "delta_v_px": float(sample["v_px"]) - float(analytic[camera]["v_px"]), "camera": camera, "marker": result.get("marker", "center"), "valid": True})
    if not cells: return {"available": True, "read": False, "reason": "no_zero_observations"}
    median = continuous_median_offset(cells)
    fit = half_pixel_candidate_scores(cells)
    candidate = tuple(fit["best_candidate_px"]) if fit.get("best_candidate_px") else (0.0, 0.0)
    corrected = apply_offset_correction(cells, candidate)
    return {"available": True, "read": True, "source": "case_results.jsonl" if not (directory / "absolute_offset_observations.jsonl").exists() else "absolute_offset_observations.jsonl", "continuous_median_offset_px": list(median), "fit": fit, "corrected_residual_summary_px": _summary([c["corrected_residual_norm_px"] for c in corrected if c.get("corrected_residual_norm_px") is not None])}


def _directory_radius(path: Path) -> float | None:
    try:
        return float(path.name.split("radius-")[-1].replace("p", "."))
    except (ValueError, IndexError):
        return None


def classify_absolute_offset_cause(evidence: dict[str, Any]) -> dict[str, Any]:
    if not evidence.get("evidence_valid"): return {"cause_verdict": "inconclusive", "leading_hypothesis": "inconclusive", "confidence": "low"}
    half, radius, segmentation = bool(evidence.get("half_pixel_component_present")), bool(evidence.get("radius_dependence_detected")), bool(evidence.get("segmentation_dependence_detected"))
    verdict = "half_pixel_convention" if half and not radius and not segmentation else "marker_fixture_bias" if radius and not segmentation else "segmentation_bias" if segmentation and not radius else "mixed_bias" if radius and segmentation else "inconclusive"
    return {"cause_verdict": verdict, "leading_hypothesis": verdict, "confidence": "high" if verdict != "inconclusive" else "low"}


def analyze(baseline_dir: Path | None, sweep_dirs: list[Path], output_dir: Path) -> dict[str, Any]:
    output_dir.mkdir(parents=True, exist_ok=True)
    baseline = _baseline_summary(baseline_dir)
    if not sweep_dirs:
        summary = {"schema_version": 2, "base_commit": BASE_COMMIT, "analysis_stage": "baseline_only", "baseline_source": "b2_standard" if baseline.get("available") else "unavailable", "baseline_evidence_available": bool(baseline.get("available")), "baseline_analysis": baseline, "evidence_valid": False, "prerequisite_failures": ["sweep_evidence_missing"], "cause_verdict": "inconclusive", "leading_hypothesis": "inconclusive", "confidence": "low", "sweep_complete": False, "attitude_mapping_verified": True, "absolute_pixel_convention_understood": False, "full_rendered_pixel_equivalence_verified": False, "gazebo_sdf_equivalence_verified": False}
        (output_dir / "summary.json").write_text(json.dumps(summary, allow_nan=False, indent=2) + "\n", encoding="utf-8")
        return summary
    failures = []
    for radius in REQUIRED_RADII:
        directory = next((path for path in sweep_dirs if _directory_radius(path) is not None and math.isclose(_directory_radius(path), radius)), None)
        if directory is None: failures.append(f"radius_{radius:g}:missing_directory")
        else: failures.extend(_run_prerequisite(directory, radius))
    cells = [cell for directory in sweep_dirs for cell in _read_cells(directory) if not math.isclose(float(cell.get("radius_m", -1)), 0.25)]
    reference = [cell for cell in cells if cell.get("valid") and int(cell.get("valid_frame_count", 0)) >= 19 and float(cell.get("threshold", -1)) == 80.0 and cell.get("centroid_mode") == "weighted_rgb_dominance"]
    required_keys = {(round(radius, 6), marker, camera) for radius in REQUIRED_RADII for marker in REQUIRED_MARKERS for camera in REQUIRED_CAMERAS}
    valid_keys = {(round(float(cell.get("radius_m", -1)), 6), cell.get("marker"), cell.get("camera")) for cell in reference}
    if len(valid_keys) != 30: failures.append("required_reference_cell_coverage_not_30_of_30")
    if any(sum(key[0] == round(radius, 6) for key in valid_keys) < 8 for radius in REQUIRED_RADII): failures.append("required_radius_reference_coverage_below_8_of_10")
    fit = half_pixel_candidate_scores(reference)
    candidate = tuple(fit["best_candidate_px"]) if fit.get("best_candidate_px") else (0.0, 0.0)
    corrected = apply_offset_correction(reference, candidate)
    eligibility = evaluate_configuration_eligibility(cells)
    if eligibility["eligible_configuration_count"] < 12: failures.append("eligible_configuration_count_below_12_of_15")
    if not eligibility["required_configurations_eligible"]: failures.append("required_configuration_not_eligible")
    variant_cells, variant_failures = _variant_cells(reference, candidate)
    failures.extend(sorted(set(variant_failures)))
    radius_result = evaluate_radius_dependence(corrected)
    segmentation_result = evaluate_segmentation_dependence(apply_offset_correction([cell for cell in cells if cell.get("valid") and evaluate_configuration_eligibility(cells)["details"].get(f"{float(cell.get('threshold')):g}|{cell.get('centroid_mode')}", {}).get("eligible")], candidate))
    raw_norms = [math.hypot(*vector) for vector in _vectors(reference)]
    corrected_norms = [cell["corrected_residual_norm_px"] for cell in corrected if cell.get("corrected_residual_norm_px") is not None]
    median = continuous_median_offset(reference)
    left, right = continuous_median_offset([c for c in reference if c.get("camera") == "left"]), continuous_median_offset([c for c in reference if c.get("camera") == "right"])
    candidate_delta = [median[0] - candidate[0], median[1] - candidate[1]]
    baseline_vs_radius_1 = None
    if baseline.get("read") and baseline.get("continuous_median_offset_px") is not None:
        radius_one = continuous_median_offset([cell for cell in reference if math.isclose(float(cell.get("radius_m", -1)), 1.0)])
        baseline_vs_radius_1 = {"baseline_continuous_median_offset_px": baseline["continuous_median_offset_px"], "b3_radius_1p00_continuous_median_offset_px": list(radius_one), "difference_px": [baseline["continuous_median_offset_px"][axis] - radius_one[axis] for axis in (0, 1)]}
    half_present = candidate != (0.0, 0.0) and max(abs(v) for v in candidate_delta) <= 0.1 and max(abs(left[i] - right[i]) for i in (0, 1)) <= 0.1 and corrected_norms and np.median(corrected_norms) <= 0.15 and max(corrected_norms) <= 0.3 and all(np.std([v[i] for v in _vectors(reference)]) <= 0.1 for i in (0, 1))
    evidence = {"evidence_valid": not failures and all(_finite(value) is not None for cell in variant_cells for value in (cell.get("variant_A_du_px"), cell.get("variant_B_du_px"), cell.get("variant_C_du_px"), cell.get("variant_D_du_px"))), "half_pixel_component_present": bool(half_present), "radius_dependence_detected": radius_result["detected"], "segmentation_dependence_detected": segmentation_result["detected"]}
    verdict = classify_absolute_offset_cause(evidence)
    if not evidence["evidence_valid"]: verdict = classify_absolute_offset_cause({"evidence_valid": False})
    variant_summary = {name: {"residual_summary_px": _summary([cell[f"variant_{name}_residual_norm_px"] for cell in variant_cells]), "median_du_px": float(np.median([cell[f"variant_{name}_du_px"] for cell in variant_cells])) if variant_cells else None, "median_dv_px": float(np.median([cell[f"variant_{name}_dv_px"] for cell in variant_cells])) if variant_cells else None} for name in ("A", "B", "C", "D")}
    summary = {"schema_version": 2, "base_commit": BASE_COMMIT, "analysis_stage": "combined", "baseline_source": "b2_standard" if baseline.get("available") else "b3_radius_1p00_reference", "baseline_evidence_available": bool(baseline.get("available")), "baseline_analysis": baseline, "baseline_vs_b3_radius_1p00": baseline_vs_radius_1, **evidence, **verdict, "prerequisite_failures": sorted(set(failures)), "continuous_median_offset_px": list(median), "best_half_pixel_candidate_px": list(candidate), "candidate_to_median_delta_px": candidate_delta, "raw_residual_summary_px": _summary(raw_norms), "corrected_residual_summary_px": _summary(corrected_norms), "radius_dependence": radius_result, "segmentation_dependence": segmentation_result, "radius_dependence_detected": radius_result["detected"], "segmentation_dependence_detected": segmentation_result["detected"], "eligible_configuration_count": eligibility["eligible_configuration_count"], "eligible_configuration_details": eligibility["details"], "required_configurations_eligible": eligibility["required_configurations_eligible"], "required_cell_coverage": {"valid": len(valid_keys), "required": len(required_keys)}, "diagnostic_cell_coverage": {"total": sum(math.isclose(float(cell.get("radius_m", -1)), 0.25) for directory in sweep_dirs for cell in _read_cells(directory)), "valid": sum(math.isclose(float(cell.get("radius_m", -1)), 0.25) and cell.get("valid", False) for directory in sweep_dirs for cell in _read_cells(directory))}, "camera_offset_summary": grouped_offset_summary(reference, ("camera",)), "marker_offset_summary": grouped_offset_summary(reference, ("marker",)), "radius_offset_summary": grouped_offset_summary(corrected, ("radius_m",), corrected=True), "configuration_offset_summary": grouped_offset_summary(corrected, ("threshold", "centroid_mode"), corrected=True), "intrinsic_variant_summary": {"variants": variant_summary}, "attitude_mapping_verified": True, "absolute_pixel_convention_understood": verdict["cause_verdict"] != "inconclusive", "full_rendered_pixel_equivalence_verified": False, "gazebo_sdf_equivalence_verified": False}
    (output_dir / "half_pixel_fit.json").write_text(json.dumps({"continuous_median_offset_px": list(median), **fit, "candidate_to_median_delta_px": candidate_delta, "raw_residual_summary_px": summary["raw_residual_summary_px"], "corrected_residual_summary_px": summary["corrected_residual_summary_px"]}, allow_nan=False, indent=2) + "\n", encoding="utf-8")
    (output_dir / "radius_dependence.json").write_text(json.dumps(radius_result, allow_nan=False, indent=2) + "\n", encoding="utf-8")
    (output_dir / "segmentation_dependence.json").write_text(json.dumps(segmentation_result, allow_nan=False, indent=2) + "\n", encoding="utf-8")
    (output_dir / "intrinsic_variant_summary.json").write_text(json.dumps({"variants": variant_summary}, allow_nan=False, indent=2) + "\n", encoding="utf-8")
    (output_dir / "residual_vectors.jsonl").write_text("".join(json.dumps(cell, allow_nan=False) + "\n" for cell in variant_cells), encoding="utf-8")
    (output_dir / "summary.json").write_text(json.dumps(summary, allow_nan=False, indent=2) + "\n", encoding="utf-8")
    return summary


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--baseline-evidence-dir")
    parser.add_argument("--sweep-evidence-dir", action="append", default=[])
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args(argv)
    summary = analyze(Path(args.baseline_evidence_dir) if args.baseline_evidence_dir else None, [Path(path) for path in args.sweep_evidence_dir], Path(args.output_dir))
    print(json.dumps(summary, allow_nan=False, sort_keys=True))
    return 0 if summary["evidence_valid"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
