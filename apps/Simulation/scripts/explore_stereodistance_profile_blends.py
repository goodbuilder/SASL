#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path
from typing import Mapping, Sequence

import numpy as np
from scipy.spatial.transform import Rotation, Slerp


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_DIR = ROOT_DIR / "scripts"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from select_stereodistance_profile_from_catalog import (  # noqa: E402
    PUBLIC_TOPIC_CONTRACT,
    RDF_AXIS_CONTRACT,
    catalog_root,
    find_profile_row,
    load_catalog,
    load_minimal_set,
)
from select_stereodistance_profile_by_calibration_replay import (  # noqa: E402
    candidate_source_scenarios,
    finite_float,
    parse_split_list,
    read_csv,
    replay_selection_score,
    write_csv,
    write_json,
)
from stereodistance_profile_replay import load_profile, replay_rows  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Explore calibration-time blends between validated StereoDistance catalog profiles. "
            "The selected output is still a normal runtime profile that consumes raw RDF/disparity only."
        )
    )
    parser.add_argument("--catalog-json", type=Path, required=True)
    parser.add_argument("--input-csv", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--output-profile", type=Path, required=True)
    parser.add_argument("--minimal-set-json", type=Path)
    parser.add_argument("--split", default="validation")
    parser.add_argument("--stability-splits", default="train,validation")
    parser.add_argument("--target-range-m", type=float, default=1852.0)
    parser.add_argument("--target-window-m", type=float, default=30.0)
    parser.add_argument("--target-error-limit-m", type=float, default=10.0)
    parser.add_argument("--alphas", default="0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1")
    parser.add_argument(
        "--selection-policy",
        choices=("balanced", "validation_best", "target_guard", "simple_stable", "replay_best"),
        default="balanced",
        help=(
            "Policy used for the exported output profile. replay_best uses the top sorted candidate; "
            "other values use profile_blend_policy_recommendations.csv policy rows."
        ),
    )
    parser.add_argument(
        "--candidate-source-mode",
        choices=("minimal_set", "all_catalog"),
        default="minimal_set",
        help=(
            "minimal_set blends profiles from --minimal-set-json when provided; all_catalog explores "
            "all validated catalog profile pairs as an offline calibration-time search."
        ),
    )
    return parser.parse_args()


def parse_float_list(value: str) -> list[float]:
    parsed: list[float] = []
    for item in value.split(","):
        text = item.strip()
        if not text:
            continue
        number = float(text)
        if not math.isfinite(number):
            raise ValueError(f"non-finite alpha: {text!r}")
        parsed.append(min(1.0, max(0.0, number)))
    return list(dict.fromkeys(parsed))


def profile_json_path(catalog_json: Path, catalog: Mapping[str, object], scenario: str) -> Path:
    row = find_profile_row(catalog, scenario)
    if not bool(row.get("profile_exists")) or not bool(row.get("pass_cross_seed_replay")):
        raise ValueError(f"catalog profile for {scenario!r} is not validated")
    relative = str(row.get("catalog_profile_path", ""))
    if not relative:
        raise ValueError(f"catalog profile for {scenario!r} has no path")
    return catalog_root(catalog_json) / relative


def interp_lut(x_query: np.ndarray, lut: Mapping[str, object]) -> np.ndarray:
    x_values = np.asarray(lut.get("x_values_m", lut.get("x_values_px", [])), dtype=float)
    residual_values = np.asarray(lut.get("residual_values_m", []), dtype=float)
    if x_values.size == 0 or residual_values.size == 0:
        return np.zeros_like(x_query, dtype=float)
    if x_values.size == 1:
        return np.full_like(x_query, float(residual_values[0]), dtype=float)
    order = np.argsort(x_values)
    return np.interp(x_query, x_values[order], residual_values[order], left=residual_values[order][0], right=residual_values[order][-1])


def blend_forward_lut(a_lut: Mapping[str, object], b_lut: Mapping[str, object], alpha: float) -> dict[str, object]:
    a_x = np.asarray(a_lut.get("x_values_m", a_lut.get("x_values_px", [])), dtype=float)
    b_x = np.asarray(b_lut.get("x_values_m", b_lut.get("x_values_px", [])), dtype=float)
    x_values = np.unique(np.concatenate([a_x, b_x]))
    if x_values.size == 0:
        return {"x_values_m": [], "residual_values_m": [], "fit_basis": "empty blended raw-forward LUT"}
    residual = (1.0 - alpha) * interp_lut(x_values, a_lut) + alpha * interp_lut(x_values, b_lut)
    return {
        "x_values_m": [float(value) for value in x_values],
        "residual_values_m": [float(value) for value in residual],
        "fit_basis": "calibration-replay blend of two validated raw-forward residual LUT profiles",
        "blend_alpha_second_profile": float(alpha),
        "source_lut_a_fit_basis": str(a_lut.get("fit_basis", "")),
        "source_lut_b_fit_basis": str(b_lut.get("fit_basis", "")),
    }


def blend_sim3_profiles(a: Mapping[str, object], b: Mapping[str, object], *, alpha: float, profile_id: str) -> dict[str, object]:
    if a.get("mode") != "estimated_to_truth_sim3" or b.get("mode") != "estimated_to_truth_sim3":
        raise ValueError("only estimated_to_truth_sim3 profiles are currently blendable")
    a_rotation = Rotation.from_matrix(np.asarray(a["rotation_matrix_rdf"], dtype=float))
    b_rotation = Rotation.from_matrix(np.asarray(b["rotation_matrix_rdf"], dtype=float))
    rotation = Slerp([0.0, 1.0], Rotation.concatenate([a_rotation, b_rotation]))([alpha])[0]
    a_translation = np.asarray(a["translation_rdf_m"], dtype=float)
    b_translation = np.asarray(b["translation_rdf_m"], dtype=float)
    blended = dict(a)
    blended["profile_id"] = profile_id
    blended["method"] = f"{a.get('method', 'profile')}_catalog_blend"
    blended["mode"] = "estimated_to_truth_sim3"
    blended["profile_family"] = "estimated_to_truth_sim3"
    blended["rdf_axis_contract"] = RDF_AXIS_CONTRACT
    blended["public_topic_contract"] = PUBLIC_TOPIC_CONTRACT
    blended["runtime_truth_used"] = False
    blended["scale"] = float((1.0 - alpha) * float(a["scale"]) + alpha * float(b["scale"]))
    blended["rotation_matrix_rdf"] = rotation.as_matrix().tolist()
    blended["rotation_euler_zyx_deg"] = rotation.as_euler("zyx", degrees=True).tolist()
    blended["translation_rdf_m"] = ((1.0 - alpha) * a_translation + alpha * b_translation).tolist()
    blended["blend_alpha_second_profile"] = float(alpha)
    blended["blend_source_profile_a_id"] = str(a.get("profile_id", ""))
    blended["blend_source_profile_b_id"] = str(b.get("profile_id", ""))
    blended["blend_source_profile_a_method"] = str(a.get("method", ""))
    blended["blend_source_profile_b_method"] = str(b.get("method", ""))
    if "forward_residual_raw_forward_lut" in a or "forward_residual_raw_forward_lut" in b:
        blended["forward_residual_raw_forward_lut"] = blend_forward_lut(
            a.get("forward_residual_raw_forward_lut", {}),
            b.get("forward_residual_raw_forward_lut", {}),
            alpha,
        )
    blended["training_contract"] = (
        "Calibration-time profile blend between validated catalog profiles. Runtime apply uses only "
        "raw stereo RDF, measured disparity when required, and calibration-fixed blended parameters."
    )
    blended["blend_contract"] = (
        "Blend is fitted offline with calibration truth; the exported runtime profile must not use truth."
    )
    blended["forbidden_runtime_inputs"] = [
        "calibration_truth",
        "px4_rtk_truth",
        "true_disparity",
        "true_image_feature_location",
    ]
    return blended


def blend_inverse_depth_profiles(
    a: Mapping[str, object],
    b: Mapping[str, object],
    *,
    alpha: float,
    profile_id: str,
) -> dict[str, object]:
    if a.get("mode") != "estimated_to_truth_disparity_poly" or b.get("mode") != "estimated_to_truth_disparity_poly":
        raise ValueError("only estimated_to_truth_disparity_poly profiles are blendable by inverse-depth coefficient interpolation")
    if a.get("correction_space") != b.get("correction_space"):
        raise ValueError("inverse-depth profile blend requires matching correction_space")
    a_coeff = np.asarray(a.get("inverse_depth_coefficients_high_to_low", []), dtype=float)
    b_coeff = np.asarray(b.get("inverse_depth_coefficients_high_to_low", []), dtype=float)
    if a_coeff.size == 0 or a_coeff.shape != b_coeff.shape:
        raise ValueError("inverse-depth profile blend requires matching inverse_depth_coefficients_high_to_low")
    blended = dict(a)
    blended["profile_id"] = profile_id
    blended["method"] = f"{a.get('method', 'inverse_depth')}_catalog_blend"
    blended["mode"] = "estimated_to_truth_disparity_poly"
    blended["profile_family"] = "estimated_to_truth_disparity_poly"
    blended["rdf_axis_contract"] = RDF_AXIS_CONTRACT
    blended["public_topic_contract"] = PUBLIC_TOPIC_CONTRACT
    blended["runtime_truth_used"] = False
    blended["inverse_depth_coefficients_high_to_low"] = ((1.0 - alpha) * a_coeff + alpha * b_coeff).tolist()
    blended["blend_alpha_second_profile"] = float(alpha)
    blended["blend_source_profile_a_id"] = str(a.get("profile_id", ""))
    blended["blend_source_profile_b_id"] = str(b.get("profile_id", ""))
    blended["blend_source_profile_a_method"] = str(a.get("method", ""))
    blended["blend_source_profile_b_method"] = str(b.get("method", ""))
    blended["training_contract"] = (
        "Calibration-time inverse-depth coefficient blend between validated catalog profiles. Runtime apply "
        "uses raw stereo RDF, measured disparity when required, and calibration-fixed blended coefficients."
    )
    blended["blend_contract"] = (
        "Blend is fitted offline with calibration truth; the exported runtime profile must not use truth."
    )
    blended["forbidden_runtime_inputs"] = [
        "calibration_truth",
        "px4_rtk_truth",
        "true_disparity",
        "true_image_feature_location",
    ]
    return blended


def blend_profiles(a: Mapping[str, object], b: Mapping[str, object], *, alpha: float, profile_id: str) -> dict[str, object]:
    if a.get("mode") == "estimated_to_truth_sim3" and b.get("mode") == "estimated_to_truth_sim3":
        return blend_sim3_profiles(a, b, alpha=alpha, profile_id=profile_id)
    if a.get("mode") == "estimated_to_truth_disparity_poly" and b.get("mode") == "estimated_to_truth_disparity_poly":
        return blend_inverse_depth_profiles(a, b, alpha=alpha, profile_id=profile_id)
    raise ValueError("profile blend requires matching supported profile modes")


def replay_profile_summary(
    *,
    rows: Sequence[Mapping[str, str]],
    profile_path: Path,
    split: str,
    stability_splits: Sequence[str],
    target_range_m: float,
    target_window_m: float,
    target_error_limit_m: float,
) -> dict[str, object]:
    profile = load_profile(profile_path)
    _, summary = replay_rows(
        rows,
        profile,
        split_filter=split,
        target_range_m=target_range_m,
        target_window_m=target_window_m,
    )
    raw_rmse = finite_float(summary.get("validation_range_rmse_raw_m"), float("inf"))
    corrected_rmse = finite_float(summary.get("validation_range_rmse_corrected_m"), float("inf"))
    target_abs = finite_float(summary.get("target_abs_mean_error_corrected_m"), float("inf"))
    target_max_abs = finite_float(summary.get("target_max_abs_error_corrected_m"), float("inf"))
    improvement = raw_rmse - corrected_rmse
    target_count = int(float(summary.get("target_window_count", 0) or 0))
    stability_improvements: list[float] = []
    stability_target_abs: list[float] = []
    fail_reasons: list[str] = []
    for stability_split in stability_splits:
        _, split_summary = replay_rows(
            rows,
            profile,
            split_filter=stability_split,
            target_range_m=target_range_m,
            target_window_m=target_window_m,
        )
        split_raw = finite_float(split_summary.get("validation_range_rmse_raw_m"), float("inf"))
        split_corrected = finite_float(split_summary.get("validation_range_rmse_corrected_m"), float("inf"))
        split_improvement = split_raw - split_corrected
        split_target_count = int(float(split_summary.get("target_window_count", 0) or 0))
        split_target_abs = finite_float(split_summary.get("target_abs_mean_error_corrected_m"), float("nan"))
        split_target_max_abs = finite_float(split_summary.get("target_max_abs_error_corrected_m"), split_target_abs)
        if math.isfinite(split_improvement):
            stability_improvements.append(split_improvement)
        if math.isfinite(split_target_abs):
            stability_target_abs.append(split_target_abs)
        if not math.isfinite(split_improvement) or split_improvement <= 0.0:
            fail_reasons.append(f"{stability_split}:no_positive_range_rmse_improvement")
        if split_target_count > 0 and (not math.isfinite(split_target_abs) or split_target_abs > target_error_limit_m):
            fail_reasons.append(f"{stability_split}:target_error_over_limit")
        if split_target_count > 0 and (
            not math.isfinite(split_target_max_abs) or split_target_max_abs > target_error_limit_m
        ):
            fail_reasons.append(f"{stability_split}:target_max_error_over_limit")
    stability_min_improvement = min(stability_improvements) if stability_improvements else float("nan")
    stability_max_target = max(stability_target_abs) if stability_target_abs else float("nan")
    pass_replay = (
        math.isfinite(raw_rmse)
        and math.isfinite(corrected_rmse)
        and improvement > 0.0
        and target_count > 0
        and math.isfinite(target_abs)
        and target_abs <= target_error_limit_m
        and math.isfinite(target_max_abs)
        and target_max_abs <= target_error_limit_m
        and not fail_reasons
    )
    result = {
        "pass_replay": pass_replay,
        "pass_reason": "pass" if pass_replay else ";".join(fail_reasons or ["primary_replay_failed"]),
        "validation_range_rmse_raw_m": raw_rmse,
        "validation_range_rmse_corrected_m": corrected_rmse,
        "validation_range_rmse_improvement_m": improvement,
        "target_window_count": target_count,
        "target_abs_mean_error_corrected_m": target_abs,
        "target_max_abs_error_corrected_m": target_max_abs,
        "stability_min_range_improvement_m": stability_min_improvement,
        "stability_max_target_abs_mean_error_m": stability_max_target,
        "model_complexity_estimate": 16,
    }
    result["replay_selection_score"] = replay_selection_score(result)
    return result


def pareto_frontier(rows: Sequence[Mapping[str, object]]) -> list[dict[str, object]]:
    passing = [row for row in rows if bool(row.get("pass_replay"))]
    frontier: list[dict[str, object]] = []
    for row in passing:
        row_rmse = finite_float(row.get("validation_range_rmse_corrected_m"), float("inf"))
        row_target = finite_float(row.get("target_abs_mean_error_corrected_m"), float("inf"))
        row_stability = finite_float(row.get("stability_min_range_improvement_m"), float("-inf"))
        dominated = False
        for other in passing:
            if other is row:
                continue
            other_rmse = finite_float(other.get("validation_range_rmse_corrected_m"), float("inf"))
            other_target = finite_float(other.get("target_abs_mean_error_corrected_m"), float("inf"))
            other_stability = finite_float(other.get("stability_min_range_improvement_m"), float("-inf"))
            no_worse = other_rmse <= row_rmse and other_target <= row_target and other_stability >= row_stability
            strictly_better = other_rmse < row_rmse or other_target < row_target or other_stability > row_stability
            if no_worse and strictly_better:
                dominated = True
                break
        if not dominated:
            frontier.append(dict(row))
    frontier.sort(
        key=lambda row: (
            finite_float(row.get("validation_range_rmse_corrected_m"), float("inf")),
            finite_float(row.get("target_abs_mean_error_corrected_m"), float("inf")),
            -finite_float(row.get("stability_min_range_improvement_m"), float("-inf")),
        )
    )
    for rank, row in enumerate(frontier, start=1):
        row["pareto_rank"] = rank
    return frontier


def choose_policy_rows(rows: Sequence[Mapping[str, object]]) -> dict[str, dict[str, object]]:
    passing = [row for row in rows if bool(row.get("pass_replay"))]
    if not passing:
        return {}

    def metric(row: Mapping[str, object], name: str, default: float) -> float:
        return finite_float(row.get(name), default)

    return {
        "balanced": dict(
            min(
                passing,
                key=lambda row: (
                    metric(row, "replay_selection_score", float("inf")),
                    metric(row, "validation_range_rmse_corrected_m", float("inf")),
                    metric(row, "target_abs_mean_error_corrected_m", float("inf")),
                ),
            )
        ),
        "validation_best": dict(
            min(
                passing,
                key=lambda row: (
                    metric(row, "validation_range_rmse_corrected_m", float("inf")),
                    metric(row, "target_abs_mean_error_corrected_m", float("inf")),
                    -metric(row, "stability_min_range_improvement_m", float("-inf")),
                ),
            )
        ),
        "target_guard": dict(
            min(
                passing,
                key=lambda row: (
                    metric(row, "target_abs_mean_error_corrected_m", float("inf")),
                    metric(row, "validation_range_rmse_corrected_m", float("inf")),
                    -metric(row, "stability_min_range_improvement_m", float("-inf")),
                ),
            )
        ),
        "simple_stable": dict(
            min(
                passing,
                key=lambda row: (
                    abs(metric(row, "alpha_second_profile", 0.0)),
                    metric(row, "validation_range_rmse_corrected_m", float("inf")),
                    metric(row, "target_abs_mean_error_corrected_m", float("inf")),
                ),
            )
        ),
    }


def explore_blends(
    *,
    catalog_json: Path,
    input_csv: Path,
    output_dir: Path,
    output_profile: Path,
    minimal_set_json: Path | None,
    split: str,
    stability_splits: Sequence[str],
    target_range_m: float,
    target_window_m: float,
    target_error_limit_m: float,
    alphas: Sequence[float],
    candidate_source_mode: str = "minimal_set",
    selection_policy: str = "balanced",
) -> dict[str, object]:
    catalog = load_catalog(catalog_json)
    rows = read_csv(input_csv)
    if candidate_source_mode == "all_catalog":
        sources = candidate_source_scenarios(catalog, None)
    elif candidate_source_mode == "minimal_set":
        sources = candidate_source_scenarios(catalog, minimal_set_json)
    else:
        raise ValueError("candidate_source_mode must be 'minimal_set' or 'all_catalog'")
    output_dir.mkdir(parents=True, exist_ok=True)
    candidate_profile_dir = output_dir / "blend_candidate_profiles"
    candidate_profile_dir.mkdir(parents=True, exist_ok=True)
    candidates: list[dict[str, object]] = []
    for left_index, left_source in enumerate(sources):
        for right_source in sources[left_index + 1 :]:
            left_path = profile_json_path(catalog_json, catalog, left_source)
            right_path = profile_json_path(catalog_json, catalog, right_source)
            left_payload = json.loads(left_path.read_text(encoding="utf-8"))
            right_payload = json.loads(right_path.read_text(encoding="utf-8"))
            if left_payload.get("mode") != right_payload.get("mode"):
                continue
            for alpha in alphas:
                profile_id = f"blend-{left_source}-to-{right_source}-alpha-{alpha:.3f}"
                profile_payload = blend_profiles(left_payload, right_payload, alpha=alpha, profile_id=profile_id)
                profile_payload["blend_sources"] = [left_source, right_source]
                profile_payload["blend_alpha_second_profile"] = float(alpha)
                profile_path = candidate_profile_dir / f"{profile_id}.json"
                write_json(profile_path, profile_payload)
                result = replay_profile_summary(
                    rows=rows,
                    profile_path=profile_path,
                    split=split,
                    stability_splits=stability_splits,
                    target_range_m=target_range_m,
                    target_window_m=target_window_m,
                    target_error_limit_m=target_error_limit_m,
                )
                candidates.append(
                    {
                        "source_a": left_source,
                        "source_b": right_source,
                        "alpha_second_profile": float(alpha),
                        "profile_path": str(profile_path),
                        "axis_contract": RDF_AXIS_CONTRACT,
                        "runtime_input_contract": "blended runtime profile consumes raw RDF/disparity only; no runtime truth",
                        **result,
                    }
                )
    candidates.sort(
        key=lambda row: (
            not bool(row.get("pass_replay")),
            finite_float(row.get("replay_selection_score"), float("inf")),
            finite_float(row.get("validation_range_rmse_corrected_m"), float("inf")),
            finite_float(row.get("target_abs_mean_error_corrected_m"), float("inf")),
        )
    )
    for rank, row in enumerate(candidates, start=1):
        row["blend_selection_rank"] = rank
    frontier = pareto_frontier(candidates)
    frontier_keys = {
        (row.get("source_a"), row.get("source_b"), row.get("alpha_second_profile"))
        for row in frontier
    }
    for row in candidates:
        row["is_pareto_frontier"] = (
            row.get("source_a"),
            row.get("source_b"),
            row.get("alpha_second_profile"),
        ) in frontier_keys
    write_csv(output_dir / "profile_blend_candidates.csv", candidates)
    write_csv(output_dir / "profile_blend_pareto_frontier.csv", frontier)
    policy_rows = choose_policy_rows(candidates)
    policy_output_rows: list[dict[str, object]] = []
    for policy, row in policy_rows.items():
        policy_output_rows.append(
            {
                "policy": policy,
                **row,
                "policy_contract": (
                    "Blend policy is selected offline from calibration truth. Exported runtime profiles consume "
                    "raw RDF/disparity only and must pass promotion checks."
                ),
            }
        )
    write_csv(output_dir / "profile_blend_policy_recommendations.csv", policy_output_rows)
    passing = [row for row in candidates if bool(row.get("pass_replay"))]
    if not passing:
        summary = {
            "selected": False,
            "reason": "no blended profile passed replay gates",
            "candidate_count": len(candidates),
            "candidate_source_mode": candidate_source_mode,
            "candidate_source_scenarios": ",".join(sources),
            "axis_contract": RDF_AXIS_CONTRACT,
            "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        }
        write_json(output_dir / "profile_blend_selection.json", summary)
        raise ValueError(summary["reason"])
    if selection_policy == "replay_best":
        best = passing[0]
    elif selection_policy in policy_rows:
        best = policy_rows[selection_policy]
    else:
        raise ValueError(f"selection_policy {selection_policy!r} has no passing blend candidate")
    output_profile.parent.mkdir(parents=True, exist_ok=True)
    output_profile.write_text(Path(str(best["profile_path"])).read_text(encoding="utf-8"), encoding="utf-8")
    summary = {
        "selected": True,
        "selection_mode": "calibration_replay_catalog_profile_blend",
        "selection_policy": selection_policy,
        "source_a": best["source_a"],
        "source_b": best["source_b"],
        "alpha_second_profile": best["alpha_second_profile"],
        "source_profile": best["profile_path"],
        "output_profile": str(output_profile),
        "candidate_count": len(candidates),
        "pass_count": len(passing),
        "candidate_source_mode": candidate_source_mode,
        "candidate_source_scenarios": ",".join(sources),
        "split": split,
        "stability_splits_requested": ",".join(stability_splits),
        "validation_range_rmse_corrected_m": best["validation_range_rmse_corrected_m"],
        "target_abs_mean_error_corrected_m": best["target_abs_mean_error_corrected_m"],
        "target_max_abs_error_corrected_m": best.get("target_max_abs_error_corrected_m", ""),
        "replay_selection_score": best["replay_selection_score"],
        "pareto_frontier_count": len(frontier),
        "policy_recommendations": {
            policy: {
                "source_a": row.get("source_a", ""),
                "source_b": row.get("source_b", ""),
                "alpha_second_profile": row.get("alpha_second_profile", ""),
                "validation_range_rmse_corrected_m": row.get("validation_range_rmse_corrected_m", ""),
                "target_abs_mean_error_corrected_m": row.get("target_abs_mean_error_corrected_m", ""),
                "replay_selection_score": row.get("replay_selection_score", ""),
            }
            for policy, row in policy_rows.items()
        },
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "selection_contract": "Calibration truth is used offline to choose blend alpha; runtime profile uses no truth.",
    }
    write_json(output_dir / "profile_blend_selection.json", summary)
    write_summary_txt(output_dir / "profile_blend_summary.txt", summary, candidates[:10], frontier, policy_output_rows)
    return summary


def write_summary_txt(
    path: Path,
    summary: Mapping[str, object],
    top_rows: Sequence[Mapping[str, object]],
    pareto_rows: Sequence[Mapping[str, object]],
    policy_rows: Sequence[Mapping[str, object]],
) -> None:
    lines = [
        "StereoDistance catalog profile blend exploration",
        "",
        f"axis_contract: {summary.get('axis_contract', RDF_AXIS_CONTRACT)}",
        f"public_topic_contract: {summary.get('public_topic_contract', PUBLIC_TOPIC_CONTRACT)}",
        f"selection_contract: {summary.get('selection_contract', '')}",
        "",
        "Selected:",
        (
            "- selected={selected}, selection_policy={policy}, source_a={source_a}, source_b={source_b}, alpha_second_profile={alpha}, "
            "validation_rmse={rmse:.6f}m, target_abs={target:.6f}m, score={score:.6f}"
        ).format(
            selected=summary.get("selected", False),
            policy=summary.get("selection_policy", ""),
            source_a=summary.get("source_a", ""),
            source_b=summary.get("source_b", ""),
            alpha=summary.get("alpha_second_profile", ""),
            rmse=finite_float(summary.get("validation_range_rmse_corrected_m"), float("nan")),
            target=finite_float(summary.get("target_abs_mean_error_corrected_m"), float("nan")),
            score=finite_float(summary.get("replay_selection_score"), float("nan")),
        ),
        "",
        "Top candidates:",
    ]
    for row in top_rows:
        lines.append(
            (
                "- rank={rank}, alpha={alpha}, pass={passed}, rmse={rmse:.6f}m, "
                "target_abs={target:.6f}m, stability_min_improvement={stability:.6f}m, score={score:.6f}"
            ).format(
                rank=row.get("blend_selection_rank", ""),
                alpha=row.get("alpha_second_profile", ""),
                passed=row.get("pass_replay", ""),
                rmse=finite_float(row.get("validation_range_rmse_corrected_m"), float("nan")),
                target=finite_float(row.get("target_abs_mean_error_corrected_m"), float("nan")),
                stability=finite_float(row.get("stability_min_range_improvement_m"), float("nan")),
                score=finite_float(row.get("replay_selection_score"), float("nan")),
            )
        )
    lines.extend(
        [
            "",
            "Policy recommendations:",
        ]
    )
    for row in policy_rows:
        lines.append(
            (
                "- {policy}: alpha={alpha}, rmse={rmse:.6f}m, target_abs={target:.6f}m, "
                "stability_min_improvement={stability:.6f}m, score={score:.6f}"
            ).format(
                policy=row.get("policy", ""),
                alpha=row.get("alpha_second_profile", ""),
                rmse=finite_float(row.get("validation_range_rmse_corrected_m"), float("nan")),
                target=finite_float(row.get("target_abs_mean_error_corrected_m"), float("nan")),
                stability=finite_float(row.get("stability_min_range_improvement_m"), float("nan")),
                score=finite_float(row.get("replay_selection_score"), float("nan")),
            )
        )
    lines.extend(["", "Pareto frontier:"])
    for row in pareto_rows:
        lines.append(
            (
                "- pareto_rank={rank}, alpha={alpha}, rmse={rmse:.6f}m, "
                "target_abs={target:.6f}m, stability_min_improvement={stability:.6f}m"
            ).format(
                rank=row.get("pareto_rank", ""),
                alpha=row.get("alpha_second_profile", ""),
                rmse=finite_float(row.get("validation_range_rmse_corrected_m"), float("nan")),
                target=finite_float(row.get("target_abs_mean_error_corrected_m"), float("nan")),
                stability=finite_float(row.get("stability_min_range_improvement_m"), float("nan")),
            )
        )
    lines.extend(
        [
            "",
            (
                "Interpretation: a blend can be useful as a target-guard calibration-time candidate "
                "when 1852m error improves enough to justify any validation RMSE increase. The exported "
                "profile is still a normal runtime profile and must pass promotion checks before use."
            ),
        ]
    )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    summary = explore_blends(
        catalog_json=args.catalog_json,
        input_csv=args.input_csv,
        output_dir=args.output_dir,
        output_profile=args.output_profile,
        minimal_set_json=args.minimal_set_json,
        split=args.split,
        stability_splits=parse_split_list(args.stability_splits),
        target_range_m=args.target_range_m,
        target_window_m=args.target_window_m,
        target_error_limit_m=args.target_error_limit_m,
        alphas=parse_float_list(args.alphas),
        candidate_source_mode=args.candidate_source_mode,
        selection_policy=args.selection_policy,
    )
    print(f"selected blended profile -> {args.output_profile}")
    print(
        (
            "selected {policy} {a}->{b} alpha={alpha:.3f}: corrected RMSE={rmse:.3f} m, "
            "target_abs={target:.3f} m, target_max_abs={target_max:.3f} m"
        ).format(
            policy=summary["selection_policy"],
            a=summary["source_a"],
            b=summary["source_b"],
            alpha=float(summary["alpha_second_profile"]),
            rmse=float(summary["validation_range_rmse_corrected_m"]),
            target=float(summary["target_abs_mean_error_corrected_m"]),
            target_max=float(summary["target_max_abs_error_corrected_m"]),
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
