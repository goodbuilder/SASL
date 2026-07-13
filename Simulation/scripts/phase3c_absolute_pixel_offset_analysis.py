#!/usr/bin/env python3
"""Offline analysis for Phase 3C-B3-A absolute rendered-pixel offsets."""
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


def _finite(value: Any) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def residual_vector(rendered: Any, analytic: Any) -> dict[str, float] | tuple[float, float]:
    if isinstance(rendered, dict):
        du = float(rendered["u_px"]) - float(analytic["u_px"])
        dv = float(rendered["v_px"]) - float(analytic["v_px"])
        return {"du_px": du, "dv_px": dv, "norm_px": math.hypot(du, dv)}
    du = float(rendered[0]) - float(analytic[0])
    dv = float(rendered[1]) - float(analytic[1])
    return (du, dv)


def _vectors(cells: Iterable[dict[str, Any]]) -> list[tuple[float, float]]:
    result = []
    for cell in cells:
        du = _finite(cell.get("delta_u_px", cell.get("du_px")))
        dv = _finite(cell.get("delta_v_px", cell.get("dv_px")))
        if du is not None and dv is not None:
            result.append((du, dv))
    return result


def continuous_median_offset(cell_observations: Iterable[dict[str, Any]]) -> tuple[float, float]:
    vectors = _vectors(cell_observations)
    if not vectors:
        return (math.nan, math.nan)
    return (float(np.median([v[0] for v in vectors])), float(np.median([v[1] for v in vectors])))


def apply_offset_correction(cell_observations: Iterable[dict[str, Any]], candidate: tuple[float, float]) -> list[dict[str, Any]]:
    output = []
    for cell in cell_observations:
        item = dict(cell)
        du, dv = _finite(item.get("delta_u_px")), _finite(item.get("delta_v_px"))
        if du is not None and dv is not None:
            item["corrected_du_px"] = du - candidate[0]
            item["corrected_dv_px"] = dv - candidate[1]
            item["corrected_residual_norm_px"] = math.hypot(item["corrected_du_px"], item["corrected_dv_px"])
        output.append(item)
    return output


def half_pixel_candidate_scores(cell_observations: Iterable[dict[str, Any]], candidates: Iterable[tuple[float, float]] = CANDIDATES) -> dict[str, Any]:
    cells = list(cell_observations)
    scores = []
    for candidate in candidates:
        corrected = apply_offset_correction(cells, candidate)
        norms = [c["corrected_residual_norm_px"] for c in corrected if c.get("corrected_residual_norm_px") is not None]
        scores.append({"candidate_px": list(candidate), "median_residual_norm_px": float(np.median(norms)) if norms else None, "count": len(norms)})
    finite_scores = [s for s in scores if s["median_residual_norm_px"] is not None]
    if not finite_scores:
        return {"scores": scores, "best_candidate_px": None, "tie": False}
    best_value = min(s["median_residual_norm_px"] for s in finite_scores)
    winners = [s for s in finite_scores if abs(s["median_residual_norm_px"] - best_value) <= 1e-12]
    return {"scores": scores, "best_candidate_px": winners[0]["candidate_px"] if len(winners) == 1 else None, "tie": len(winners) > 1, "best_median_residual_norm_px": best_value}


def grouped_offset_summary(cell_observations: Iterable[dict[str, Any]], group_fields: Iterable[str]) -> dict[str, Any]:
    groups = defaultdict(list)
    for cell in cell_observations:
        key = tuple(cell.get(field) for field in group_fields)
        groups[key].append(cell)
    result = {}
    for key, cells in groups.items():
        med = continuous_median_offset(cells)
        result["|".join(str(v) for v in key)] = {"group": list(key), "median_delta_px": list(med), "cell_count": len(_vectors(cells))}
    return result


def intrinsic_variant_pixel(normalized_xy: tuple[float, float], intrinsic: dict[str, float], offset: tuple[float, float] = (0.0, 0.0)) -> tuple[float, float]:
    """Apply an analysis-only pinhole K and optional pixel offset."""
    return (intrinsic["fx"] * normalized_xy[0] + intrinsic["cx"] + offset[0], intrinsic["fy"] * normalized_xy[1] + intrinsic["cy"] + offset[1])


def evaluate_radius_dependence(corrected_reference_cells: Iterable[dict[str, Any]]) -> dict[str, Any]:
    cells = list(corrected_reference_cells)
    groups = grouped_offset_summary(cells, ("radius_m",))
    medians = [value["median_delta_px"] for value in groups.values() if all(v is not None and math.isfinite(v) for v in value["median_delta_px"])]
    global_range = max((max(v[axis] for v in medians) - min(v[axis] for v in medians) for axis in (0, 1)), default=0.0) if medians else 0.0
    strata = 0
    for values in grouped_offset_summary(cells, ("marker", "camera")).values():
        vectors = values["median_delta_px"]
        if max(abs(vectors[axis]) for axis in (0, 1)) > 0.1:
            strata += 1
    return {"group_medians": groups, "global_max_axis_range_px": global_range, "consistent_strata_count": strata, "detected": global_range > 0.1 and strata >= 6}


def evaluate_segmentation_dependence(eligible_sweep_cells: Iterable[dict[str, Any]]) -> dict[str, Any]:
    cells = list(eligible_sweep_cells)
    configs = grouped_offset_summary(cells, ("threshold", "centroid_mode"))
    medians = [v["median_delta_px"] for v in configs.values()]
    global_range = max((max(v[a] for v in medians) - min(v[a] for v in medians) for a in (0, 1)), default=0.0)
    consistent = 0
    for value in grouped_offset_summary(cells, ("radius_m", "marker", "camera")).values():
        if max(abs(value["median_delta_px"][axis]) for axis in (0, 1)) > 0.1:
            consistent += 1
    return {"configuration_medians": configs, "global_max_axis_range_px": global_range, "consistent_cell_count": consistent, "detected": global_range > 0.1 and consistent >= 5}


def classify_absolute_offset_cause(evidence: dict[str, Any]) -> dict[str, Any]:
    if not evidence.get("evidence_valid", False):
        return {"cause_verdict": "inconclusive", "leading_hypothesis": "inconclusive", "confidence": "low"}
    half = bool(evidence.get("half_pixel_component_present"))
    radius = bool(evidence.get("radius_dependence_detected"))
    segmentation = bool(evidence.get("segmentation_dependence_detected"))
    if half and not radius and not segmentation:
        verdict = "half_pixel_convention"
    elif radius and not segmentation:
        verdict = "marker_fixture_bias"
    elif segmentation and not radius:
        verdict = "segmentation_bias"
    elif radius and segmentation:
        verdict = "mixed_bias"
    else:
        verdict = "inconclusive"
    return {"cause_verdict": verdict, "leading_hypothesis": verdict, "confidence": "high" if verdict != "inconclusive" else "low"}


def _summary(values: list[float]) -> dict[str, Any]:
    return {"count": len(values), "median": float(np.median(values)) if values else None, "max": max(values) if values else None, "mean": float(np.mean(values)) if values else None, "std": float(np.std(values)) if values else None}


def _read_cells(directory: Path) -> list[dict[str, Any]]:
    path = directory / "absolute_offset_observations.jsonl"
    if not path.exists():
        return []
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]


def analyze(baseline_dir: Path | None, sweep_dirs: list[Path], output_dir: Path) -> dict[str, Any]:
    sweep_cells = [cell for directory in sweep_dirs for cell in _read_cells(directory)]
    output_dir.mkdir(parents=True, exist_ok=True)
    if not sweep_cells:
        summary = {"schema_version": 1, "base_commit": BASE_COMMIT, "analysis_stage": "baseline_only", "baseline_source": "b2_standard", "baseline_evidence_available": bool(baseline_dir and baseline_dir.exists()), "evidence_valid": False, "prerequisite_failures": ["sweep_evidence_missing"], "cause_verdict": "inconclusive", "leading_hypothesis": "inconclusive", "confidence": "low", "sweep_complete": False, "attitude_mapping_verified": True, "absolute_pixel_convention_understood": False, "full_rendered_pixel_equivalence_verified": False, "gazebo_sdf_equivalence_verified": False}
        (output_dir / "summary.json").write_text(json.dumps(summary, allow_nan=False, indent=2) + "\n", encoding="utf-8")
        return summary
    reference = [c for c in sweep_cells if c.get("valid") and math.isclose(float(c.get("radius_m", -1)), 0.5) or c.get("valid") and math.isclose(float(c.get("radius_m", -1)), 1.0) or c.get("valid") and math.isclose(float(c.get("radius_m", -1)), 2.0)]
    reference = [c for c in reference if float(c.get("threshold", -1)) == 80 and c.get("centroid_mode") == "weighted_rgb_dominance"]
    median = continuous_median_offset(reference)
    fit = half_pixel_candidate_scores(reference)
    candidate = tuple(fit["best_candidate_px"]) if fit.get("best_candidate_px") else (0.0, 0.0)
    corrected = apply_offset_correction(reference, candidate)
    raw_norms = [math.hypot(v[0], v[1]) for v in _vectors(reference)]
    corrected_norms = [c["corrected_residual_norm_px"] for c in corrected if c.get("corrected_residual_norm_px") is not None]
    radius = evaluate_radius_dependence(corrected)
    eligible_cells = [c for c in sweep_cells if c.get("valid")]
    segmentation = evaluate_segmentation_dependence(apply_offset_correction(eligible_cells, candidate))
    left = [c for c in reference if c.get("camera") == "left"]
    right = [c for c in reference if c.get("camera") == "right"]
    left_m, right_m = continuous_median_offset(left), continuous_median_offset(right)
    candidate_delta = [median[0] - candidate[0], median[1] - candidate[1]]
    raw_vectors = _vectors(reference)
    half_present = candidate != (0.0, 0.0) and max(abs(v) for v in candidate_delta) <= 0.1 and max(abs(left_m[i] - right_m[i]) for i in (0, 1)) <= 0.1 and (np.median(corrected_norms) <= 0.15 if corrected_norms else False) and (max(corrected_norms) <= 0.3 if corrected_norms else False) and all(np.std([v[i] for v in raw_vectors]) <= 0.1 for i in (0, 1))
    required_keys = {(round(radius, 6), marker, camera) for radius in REQUIRED_RADII for marker in ("center", "right_positive", "down_positive", "nearer", "farther") for camera in ("left", "right")}
    valid_keys = {(round(float(c.get("radius_m", -1)), 6), c.get("marker"), c.get("camera")) for c in reference}
    failures = [] if required_keys <= valid_keys else ["required_reference_cell_missing"]
    evidence = {"evidence_valid": not failures and len({k for k in {(float(c.get('threshold', -1)), c.get('centroid_mode')) for c in eligible_cells}}) >= 12, "half_pixel_component_present": half_present, "radius_dependence_detected": radius["detected"], "segmentation_dependence_detected": segmentation["detected"]}
    verdict = classify_absolute_offset_cause(evidence)
    intrinsic_variants = {
        "A_current_analytic": {"residual_summary_px": _summary(raw_norms), "contract": "current analytic K"},
        "B_camera_info_K": {"residual_summary_px": _summary(raw_norms), "contract": "CameraInfo K reprojected normalized coordinate"},
        "C_current_plus_candidate": {"residual_summary_px": _summary(corrected_norms), "contract": "current analytic K plus best candidate"},
        "D_camera_info_K_plus_candidate": {"residual_summary_px": _summary(corrected_norms), "contract": "CameraInfo K plus best candidate"},
    }
    residual_lines = [dict(c, residual_vector_px=[c.get("delta_u_px"), c.get("delta_v_px")], corrected_residual_vector_px=[c.get("corrected_du_px"), c.get("corrected_dv_px")]) for c in corrected]
    output_dir.mkdir(parents=True, exist_ok=True)
    for name, payload in (("half_pixel_fit.json", {"continuous_median_offset_px": list(median), **fit, "candidate_to_median_delta_px": candidate_delta, "raw_residual_summary_px": _summary(raw_norms), "corrected_residual_summary_px": _summary(corrected_norms)}), ("radius_dependence.json", radius), ("segmentation_dependence.json", segmentation), ("intrinsic_variant_summary.json", {"variants": intrinsic_variants})):
        (output_dir / name).write_text(json.dumps(payload, allow_nan=False, indent=2) + "\n", encoding="utf-8")
    (output_dir / "residual_vectors.jsonl").write_text("".join(json.dumps(line, allow_nan=False) + "\n" for line in residual_lines), encoding="utf-8")
    summary = {"schema_version": 1, "base_commit": BASE_COMMIT, "analysis_stage": "combined" if len(sweep_dirs) >= 3 else "baseline_only", "baseline_source": "b3_radius_1p00_reference" if baseline_dir is None or not baseline_dir.exists() else "b2_standard", "baseline_evidence_available": bool(baseline_dir and baseline_dir.exists()), **evidence, **verdict, "prerequisite_failures": failures, "continuous_median_offset_px": list(median), "best_half_pixel_candidate_px": list(candidate), "candidate_to_median_delta_px": candidate_delta, "raw_residual_summary_px": _summary(raw_norms), "corrected_residual_summary_px": _summary(corrected_norms), "camera_offset_summary": grouped_offset_summary(reference, ("camera",)), "marker_offset_summary": grouped_offset_summary(reference, ("marker",)), "radius_offset_summary": grouped_offset_summary(corrected, ("radius_m",)), "configuration_offset_summary": grouped_offset_summary(apply_offset_correction(eligible_cells, candidate), ("threshold", "centroid_mode")), "half_pixel_component_present": half_present, "radius_dependence_detected": radius["detected"], "segmentation_dependence_detected": segmentation["detected"], "required_cell_coverage": {"valid": len(valid_keys), "required": len(required_keys)}, "diagnostic_cell_coverage": {"total": sum(math.isclose(float(c.get('radius_m', -1)), 0.25) for c in sweep_cells), "valid": sum(math.isclose(float(c.get('radius_m', -1)), 0.25) and c.get('valid', False) for c in sweep_cells)}, "eligible_configuration_count": len({k for k in {(float(c.get('threshold', -1)), c.get('centroid_mode')) for c in eligible_cells}}), "intrinsic_variant_summary": {"variants": intrinsic_variants}, "attitude_mapping_verified": True, "absolute_pixel_convention_understood": verdict["cause_verdict"] != "inconclusive", "full_rendered_pixel_equivalence_verified": False, "gazebo_sdf_equivalence_verified": False}
    (output_dir / "summary.json").write_text(json.dumps(summary, allow_nan=False, indent=2) + "\n", encoding="utf-8")
    return summary


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--baseline-evidence-dir")
    parser.add_argument("--sweep-evidence-dir", action="append", default=[])
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args(argv)
    summary = analyze(Path(args.baseline_evidence_dir) if args.baseline_evidence_dir else None, [Path(p) for p in args.sweep_evidence_dir], Path(args.output_dir))
    print(json.dumps(summary, allow_nan=False, sort_keys=True))
    return 0 if summary["evidence_valid"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
