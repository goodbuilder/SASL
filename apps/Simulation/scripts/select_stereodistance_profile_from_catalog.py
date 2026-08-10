#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import shutil
from pathlib import Path
from typing import Mapping, Sequence


RDF_AXIS_CONTRACT = "stereo_center_opencv_optical:RDF(right,down,forward)"
PUBLIC_TOPIC_CONTRACT = "stereodistance_position_topics:FRD_range(forward,right,down,range)"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Select one StereoDistance runtime profile from a scenario profile catalog. "
            "The selection must be made from deployment metadata/operator configuration, "
            "not runtime truth."
        )
    )
    parser.add_argument("--catalog-json", type=Path)
    parser.add_argument("--deployment-strategy-json", type=Path)
    parser.add_argument(
        "--minimal-set-json",
        type=Path,
        help=(
            "Optional scenario_profile_catalog_minimal_set.json. When provided with --scenario, "
            "the requested deployment scenario is mapped to the smallest validated catalog source profile "
            "that covers it."
        ),
    )
    parser.add_argument("--scenario")
    parser.add_argument("--output-profile", type=Path, required=True)
    parser.add_argument("--selection-summary-json", type=Path)
    parser.add_argument(
        "--allow-unvalidated",
        action="store_true",
        help="Allow selecting a catalog row that did not pass cross-seed replay.",
    )
    return parser.parse_args()


def load_deployment_strategy(path: Path) -> dict[str, object]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    strategy = payload.get("deployment_strategy")
    if strategy not in {
        "single_runtime_profile",
        "single_near_tie_runtime_profile",
        "single_pooled_runtime_profile",
        "scenario_profile_catalog",
        "no_validated_runtime_strategy",
    }:
        raise ValueError(f"unsupported deployment strategy: {strategy!r}")
    return payload


def load_catalog(path: Path) -> dict[str, object]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if payload.get("mode") != "scenario_profile_catalog" and "rows" not in payload:
        raise ValueError(f"unsupported catalog mode: {payload.get('mode')!r}")
    profiles = payload.get("profiles", payload.get("rows"))
    if not isinstance(profiles, list):
        raise ValueError("scenario profile catalog requires a profiles list")
    normalized = dict(payload)
    normalized["mode"] = "scenario_profile_catalog"
    normalized["profiles"] = profiles
    return normalized


def load_minimal_set(path: Path) -> dict[str, object]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not bool(payload.get("pass_minimal_catalog_profile_set")):
        raise ValueError("minimal catalog profile set did not pass coverage validation")
    rows = payload.get("rows")
    if not isinstance(rows, list):
        raise ValueError("minimal catalog profile set requires a rows list")
    return payload


def catalog_root(catalog_json: Path) -> Path:
    # Generated catalogs live at <sweep_output>/scenario_profile_catalog/manifest.json;
    # profile paths stored in the manifest are relative to <sweep_output>.
    if catalog_json.parent.name == "scenario_profile_catalog":
        return catalog_json.parent.parent
    return catalog_json.parent


def method_contract_fields(method: str) -> dict[str, str]:
    name = str(method or "")
    if not name:
        return {
            "registration_model": "none",
            "residual_model": "none",
            "regularization_model": "none",
            "runtime_profile_family": "none",
            "selection_note": "no method selected",
        }
    if name == "raw":
        registration = "none_raw_stereo_rdf"
    elif "affine" in name:
        registration = "affine_rdf_estimated_to_truth"
    elif "left_reprojection_geometry" in name:
        registration = "left_reprojection_triangulation_then_sim3_estimated_to_truth"
    elif "left_disparity_camera_geometry" in name:
        registration = "left_center_disparity_triangulation_then_sim3_estimated_to_truth"
    elif "joint_disparity_ray_rotation_nls" in name:
        registration = "joint_disparity_ray_rotation_estimated_to_truth"
    elif "disparity_product_ray_scale" in name and "sim3" in name:
        registration = "disparity_product_ray_scale_then_sim3_estimated_to_truth"
    elif "piecewise_raw_forward" in name and "sim3" in name:
        registration = "piecewise_raw_forward_sim3_estimated_to_truth"
    elif "inverse_depth_linear_then_sim3" in name:
        registration = "inverse_depth_ray_scale_then_sim3_estimated_to_truth"
    elif "sim3" in name:
        registration = "sim3_estimated_to_truth"
    elif "global_scale_rotation" in name:
        registration = "scaled_rotation_estimated_to_truth"
    elif "global_rotation" in name:
        registration = "rotation_estimated_to_truth"
    elif "global_inverse_depth" in name or "ridge_global_inverse_depth" in name:
        registration = "raw_rdf_inverse_depth_ray_scale"
    elif "disparity" in name:
        registration = "raw_rdf_ray_scale"
    else:
        registration = "other_estimated_to_truth"

    if "piecewise_raw_forward" in name and "sim3" in name:
        residual = "none"
    elif "quadratic_local" in name and "raw_forward" in name:
        residual = "forward_quadratic_local_raw_forward_lut"
    elif "inverse_depth" in name and "raw_forward" in name:
        residual = "forward_inverse_depth_raw_forward_lut"
    elif "quadratic" in name and "raw_forward" in name:
        residual = "forward_quadratic_raw_forward_lut"
    elif "range_normalized" in name and "raw_forward" in name:
        residual = "forward_range_normalized_raw_forward_lut"
    elif "agreement_gated" in name and "raw_forward" in name:
        residual = "forward_disparity_raw_forward_agreement_gated_lut"
    elif "blended" in name and "raw_forward" in name:
        residual = "forward_disparity_raw_forward_blend_lut"
    elif "raw_forward" in name:
        residual = "forward_raw_rdf_forward_lut"
    elif "rdf_residual" in name or "binned_rdf_residual" in name:
        residual = "per_rdf_axis_disparity_residual"
    elif "binned_residual" in name:
        residual = "forward_disparity_lut"
    elif "residual_disparity" in name:
        residual = "forward_disparity_polynomial"
    elif "global_disparity" in name:
        residual = "corrected_disparity_polynomial"
    elif "left_reprojection_geometry" in name:
        residual = "truth_to_observation_left_center_disparity_reprojection"
    elif "left_disparity_camera_geometry" in name:
        residual = "corrected_left_center_disparity_triangulation"
    elif "joint_disparity_ray_rotation_nls" in name:
        residual = "corrected_disparity_linear_plus_rdf_rotation"
    elif "disparity_product_ray_scale" in name:
        residual = "calibrated_disparity_forward_product_ray_scale"
    elif "global_inverse_depth" in name or "ridge_global_inverse_depth" in name:
        residual = "corrected_inverse_depth_linear"
    else:
        residual = "none"

    regularizers: list[str] = []
    if "smoothed_binned" in name:
        regularizers.append("lut_shrinkage_neighbor_smoothing")
    if "kernel_smoothed" in name:
        regularizers.append("kernel_smoothing")
    if "capped_binned" in name:
        regularizers.append("lut_robust_winsorized_cap")
    if "blended" in name and "piecewise_raw_forward" not in name:
        regularizers.append("dual_key_residual_blend")
    if "blended" in name and "piecewise_raw_forward" not in name and "_alpha" in name:
        regularizers.append("dual_key_blend_alpha_grid")
    if "blended_piecewise_raw_forward" in name:
        regularizers.append("global_piecewise_registration_blend")
    if "blended_piecewise_raw_forward" in name and "_alpha" in name:
        regularizers.append("piecewise_blend_alpha_grid")
    if "agreement_gated" in name:
        regularizers.append("dual_key_residual_agreement_gate")
    if "range_normalized" in name:
        regularizers.append("range_normalized_residual")
    if "inverse_depth" in name:
        regularizers.append("inverse_depth_residual")
    if "joint_disparity_ray_rotation_nls" in name:
        regularizers.append("nonlinear_least_squares")
    if "huber_joint_disparity_ray_rotation_nls" in name:
        regularizers.append("huber_m_estimator")
    if "left_reprojection_geometry" in name:
        regularizers.append("nonlinear_least_squares")
        regularizers.append("stereo_camera_geometry")
        regularizers.append("truth_to_observation_reprojection")
    if "left_disparity_camera_geometry" in name:
        regularizers.append("nonlinear_least_squares")
        regularizers.append("stereo_camera_geometry")
    if "huber_left_disparity_camera_geometry" in name:
        regularizers.append("huber_m_estimator")
    if "disparity_product_ray_scale" in name:
        regularizers.append("stereo_depth_product")
    if "quadratic_local" in name:
        regularizers.append("semi_parametric_local_residual")
    if "quadratic" in name and "raw_forward" in name:
        regularizers.append("stereo_depth_bias_quadratic")
    if "ridge" in name:
        regularizers.append("ridge")
    if "identity_shrunk" in name:
        regularizers.append("identity_gain_shrinkage")
    if "huber" in name:
        regularizers.append("huber_irls")
    if "theil_sen" in name:
        regularizers.append("theil_sen_median_slope")
    if "tukey" in name:
        regularizers.append("tukey_biweight_irls")
    if "trimmed" in name:
        regularizers.append("least_trimmed_consensus")
    if "truncated_ls" in name:
        regularizers.append("truncated_least_squares_consensus")
    if "ransac" in name:
        regularizers.append("ransac_consensus")
    if "range_weighted" in name or "target_weighted" in name or "target_shrunk" in name:
        regularizers.append("target_range_weighting")
    if "target_shrunk" in name:
        regularizers.append("target_weight_shrinkage")
    if "target_weighted_quadratic_residual_raw_forward_lut" in name and "_sigma" in name:
        regularizers.append("target_residual_sigma_grid")
    if "target_weighted_quadratic_residual_raw_forward_lut" in name and "_shrink" in name:
        regularizers.append("target_residual_shrinkage")
    if "piecewise_raw_forward" in name and "sim3" in name:
        regularizers.append("raw_forward_piecewise_registration")
    if "bins" in name or "_support" in name:
        regularizers.append("lut_bin_count_grid")
    regularization = "+".join(regularizers) if regularizers else "none"

    if registration == "affine_rdf_estimated_to_truth":
        profile_family = "estimated_to_truth_affine_rdf"
    elif registration in {
        "sim3_estimated_to_truth",
        "disparity_product_ray_scale_then_sim3_estimated_to_truth",
        "left_center_disparity_triangulation_then_sim3_estimated_to_truth",
        "left_reprojection_triangulation_then_sim3_estimated_to_truth",
        "joint_disparity_ray_rotation_estimated_to_truth",
        "piecewise_raw_forward_sim3_estimated_to_truth",
        "inverse_depth_ray_scale_then_sim3_estimated_to_truth",
        "scaled_rotation_estimated_to_truth",
        "rotation_estimated_to_truth",
    }:
        profile_family = "estimated_to_truth_sim3"
    elif residual in {"corrected_disparity_polynomial", "corrected_inverse_depth_linear"}:
        profile_family = "estimated_to_truth_disparity_poly"
    elif name == "raw":
        profile_family = "none_raw"
    else:
        profile_family = "offline_or_nonpreferred_runtime_candidate"

    if "truncated_ls" in name and registration == "sim3_estimated_to_truth":
        selection_note = "truncated least-squares robust Sim(3) point-set registration for outlier-resistant pose/origin calibration"
    elif registration == "piecewise_raw_forward_sim3_estimated_to_truth":
        selection_note = "range-dependent Sim(3) registration nodes blended by runtime raw RDF forward distance"
    elif registration == "disparity_product_ray_scale_then_sim3_estimated_to_truth":
        selection_note = "calibrated disparity-forward product ray-scale correction followed by Sim(3) pose/origin registration"
    elif registration == "inverse_depth_ray_scale_then_sim3_estimated_to_truth":
        selection_note = "inverse-depth ray-scale correction followed by Sim(3) pose/origin registration"
    elif registration == "left_reprojection_triangulation_then_sim3_estimated_to_truth":
        selection_note = "truth-to-observation reprojection fit for bbox center/disparity offsets followed by Sim(3) pose/origin registration"
    elif residual == "forward_quadratic_local_raw_forward_lut":
        selection_note = "quadratic raw-forward residual calibration with shrunk local residual-of-residual LUT"
    elif residual == "forward_inverse_depth_raw_forward_lut":
        selection_note = "inverse-depth raw-forward residual calibration for stereo disparity scale/bias error"
    elif residual == "forward_quadratic_raw_forward_lut":
        selection_note = "quadratic raw-forward residual calibration for disparity-bias-like range error"
    elif residual == "forward_range_normalized_raw_forward_lut":
        selection_note = "range-proportional forward residual calibration keyed by runtime raw RDF forward distance"
    elif residual == "forward_disparity_raw_forward_agreement_gated_lut":
        selection_note = "dual-key residual calibration that suppresses residual correction when runtime LUT predictions disagree"
    elif residual == "forward_disparity_raw_forward_blend_lut":
        selection_note = "dual-key piecewise residual calibration split between measured disparity and raw RDF forward distance"
    elif residual == "forward_raw_rdf_forward_lut":
        selection_note = "piecewise forward residual calibration keyed by runtime raw RDF forward distance"
    elif residual == "forward_disparity_lut" and "capped_binned" in name:
        selection_note = "limits residual LUT gain to reduce transfer instability from sparse or stressed calibration data"
    elif residual == "forward_disparity_lut" and "smoothed_binned" in name:
        selection_note = "useful when outlier or sparse-bin stability beats raw bin medians"
    elif residual == "forward_disparity_lut":
        selection_note = "piecewise disparity residual calibration; validate bin count against held-out flight data"
    elif registration == "affine_rdf_estimated_to_truth":
        selection_note = "higher-flexibility estimated-to-truth mapping; require validation stability before promotion"
    elif "sim3" in name:
        selection_note = "similarity registration baseline for global pose/origin/scale error"
    elif residual == "corrected_disparity_polynomial":
        selection_note = "range-only ray-scale correction; limited pose/origin correction"
    elif residual == "corrected_inverse_depth_linear":
        selection_note = "range-only inverse-depth ray-scale correction; useful for stereo depth scale/bias diagnostics"
    else:
        selection_note = "diagnostic candidate"

    return {
        "registration_model": registration,
        "residual_model": residual,
        "regularization_model": regularization,
        "runtime_profile_family": profile_family,
        "selection_note": selection_note,
    }


def find_profile_row(catalog: Mapping[str, object], scenario: str) -> Mapping[str, object]:
    profiles = catalog.get("profiles", [])
    for profile in profiles if isinstance(profiles, list) else []:
        if isinstance(profile, Mapping) and str(profile.get("scenario", "")) == scenario:
            return profile
    known = sorted(
        str(profile.get("scenario", ""))
        for profile in profiles
        if isinstance(profile, Mapping) and profile.get("scenario")
    )
    raise ValueError(f"scenario {scenario!r} not found in catalog. Known scenarios: {', '.join(known)}")


def split_scenarios(value: object) -> set[str]:
    if isinstance(value, list):
        return {str(item).strip() for item in value if str(item).strip()}
    return {item.strip() for item in str(value or "").split(",") if item.strip()}


def find_minimal_set_source_scenario(minimal_set: Mapping[str, object], scenario: str) -> str:
    rows = minimal_set.get("rows", [])
    for row in rows if isinstance(rows, list) else []:
        if not isinstance(row, Mapping):
            continue
        newly_covered = split_scenarios(row.get("newly_covered_target_scenarios", ""))
        if scenario in newly_covered:
            return str(row.get("source_scenario", ""))
    for row in rows if isinstance(rows, list) else []:
        if not isinstance(row, Mapping):
            continue
        all_pass = split_scenarios(row.get("all_pass_target_scenarios", ""))
        if scenario in all_pass:
            return str(row.get("source_scenario", ""))
    covered = sorted(split_scenarios(minimal_set.get("covered_target_scenarios", [])))
    raise ValueError(
        f"scenario {scenario!r} is not covered by the minimal catalog profile set. "
        f"Covered scenarios: {', '.join(covered)}"
    )


def select_profile(
    *,
    catalog_json: Path,
    scenario: str,
    output_profile: Path,
    allow_unvalidated: bool = False,
) -> dict[str, object]:
    catalog = load_catalog(catalog_json)
    row = find_profile_row(catalog, scenario)
    if not bool(row.get("profile_exists")):
        raise ValueError(f"catalog profile for scenario {scenario!r} is marked missing")
    if not allow_unvalidated and not bool(row.get("pass_cross_seed_replay")):
        raise ValueError(f"catalog profile for scenario {scenario!r} did not pass cross-seed replay")
    relative_profile = row.get("catalog_profile_path")
    if not relative_profile:
        raise ValueError(f"catalog profile for scenario {scenario!r} has no catalog_profile_path")
    source_profile = catalog_root(catalog_json) / str(relative_profile)
    if not source_profile.exists():
        raise FileNotFoundError(source_profile)
    output_profile.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(source_profile, output_profile)
    summary = {
        "selected": True,
        "scenario": scenario,
        "method": row.get("method", ""),
        **method_contract_fields(str(row.get("method", ""))),
        "source_profile": str(source_profile),
        "output_profile": str(output_profile),
        "pass_cross_seed_replay": bool(row.get("pass_cross_seed_replay")),
        "validation_range_rmse_mean_m": row.get("validation_range_rmse_mean_m", ""),
        "target_abs_mean_error_max_m": row.get("target_abs_mean_error_max_m", ""),
        "selection_contract": catalog.get("selection_contract", ""),
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "runtime_input_contract": "selected profile is consumed as a normal single StereoDistance runtime profile; no runtime truth",
    }
    return summary


def select_profile_from_minimal_set(
    *,
    catalog_json: Path,
    minimal_set_json: Path,
    scenario: str,
    output_profile: Path,
    allow_unvalidated: bool = False,
) -> dict[str, object]:
    minimal_set = load_minimal_set(minimal_set_json)
    source_scenario = find_minimal_set_source_scenario(minimal_set, scenario)
    summary = select_profile(
        catalog_json=catalog_json,
        scenario=source_scenario,
        output_profile=output_profile,
        allow_unvalidated=allow_unvalidated,
    )
    summary["selection_mode"] = "scenario_profile_catalog_minimal_set"
    summary["requested_scenario"] = scenario
    summary["scenario"] = source_scenario
    summary["minimal_set_json"] = str(minimal_set_json)
    summary["minimal_set_profile_count"] = minimal_set.get("selected_profile_count", "")
    summary["minimal_set_selection_contract"] = minimal_set.get("selection_contract", "")
    summary["runtime_input_contract"] = (
        "minimal-set selected profile is consumed as a normal single StereoDistance runtime profile; "
        "selection uses deployment/calibration metadata and no runtime truth"
    )
    return summary


def select_profile_from_deployment_strategy(
    *,
    deployment_strategy_json: Path,
    output_profile: Path,
    scenario: str | None = None,
    catalog_json: Path | None = None,
    minimal_set_json: Path | None = None,
    allow_unvalidated: bool = False,
) -> dict[str, object]:
    strategy_payload = load_deployment_strategy(deployment_strategy_json)
    strategy = str(strategy_payload["deployment_strategy"])
    sweep_root = deployment_strategy_json.parent
    if strategy in {"single_near_tie_runtime_profile", "single_pooled_runtime_profile"} and scenario is None:
        if strategy == "single_near_tie_runtime_profile":
            relative_profile = str(strategy_payload.get("best_near_tie_profile_path", ""))
            method = str(strategy_payload.get("best_near_tie_method", ""))
            pass_count = strategy_payload.get("best_near_tie_transfer_pass_count", "")
            total_count = strategy_payload.get("best_near_tie_transfer_total_count", "")
            promotion_pass = strategy_payload.get("best_near_tie_promotion_pass", "")
            selection_mode = "deployment_strategy_single_near_tie_runtime_profile"
        else:
            relative_profile = str(strategy_payload.get("pooled_profile_path", ""))
            method = str(strategy_payload.get("pooled_profile_method", ""))
            pass_count = strategy_payload.get("pooled_profile_transfer_pass_count", "")
            total_count = strategy_payload.get("pooled_profile_transfer_total_count", "")
            promotion_pass = strategy_payload.get("pooled_profile_promotion_pass", "")
            selection_mode = "deployment_strategy_single_pooled_runtime_profile"
        if not relative_profile:
            raise ValueError(f"{strategy} strategy has no runtime profile path")
        source_profile = sweep_root / relative_profile
        if not source_profile.exists():
            raise FileNotFoundError(source_profile)
        output_profile.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source_profile, output_profile)
        return {
            "selected": True,
            "selection_mode": selection_mode,
            "deployment_strategy": strategy,
            "method": method,
            **method_contract_fields(method),
            "source_profile": str(source_profile),
            "output_profile": str(output_profile),
            "transfer_pass_count": pass_count,
            "transfer_total_count": total_count,
            "promotion_pass": promotion_pass,
            "selection_contract": strategy_payload.get("selection_contract", ""),
            "axis_contract": RDF_AXIS_CONTRACT,
            "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
            "runtime_input_contract": "selected profile is consumed as a normal single StereoDistance runtime profile; no runtime truth",
        }
    if strategy == "single_runtime_profile" and scenario is None:
        policy = str(strategy_payload.get("selected_policy") or strategy_payload.get("best_transfer_policy") or "")
        if not policy:
            raise ValueError("single_runtime_profile strategy has no selected_policy or best_transfer_policy")
        source_profile = sweep_root / "robust_policy_profiles" / f"{policy}_runtime_profile.json"
        if not source_profile.exists():
            raise FileNotFoundError(source_profile)
        output_profile.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source_profile, output_profile)
        return {
            "selected": True,
            "selection_mode": "deployment_strategy_single_runtime_profile",
            "deployment_strategy": strategy,
            "policy": policy,
            "method": strategy_payload.get("best_transfer_method", ""),
            **method_contract_fields(str(strategy_payload.get("best_transfer_method", ""))),
            "source_profile": str(source_profile),
            "output_profile": str(output_profile),
            "best_transfer_pass_count": strategy_payload.get("best_transfer_pass_count", ""),
            "best_transfer_total_count": strategy_payload.get("best_transfer_total_count", ""),
            "selection_contract": strategy_payload.get("selection_contract", ""),
            "axis_contract": RDF_AXIS_CONTRACT,
            "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
            "runtime_input_contract": "selected profile is consumed as a normal single StereoDistance runtime profile; no runtime truth",
        }
    if strategy == "scenario_profile_catalog":
        if scenario is None:
            raise ValueError("scenario_profile_catalog strategy requires --scenario from deployment metadata/operator configuration")
        catalog_path = catalog_json or (sweep_root / "scenario_profile_catalog" / "manifest.json")
        if minimal_set_json is not None:
            summary = select_profile_from_minimal_set(
                catalog_json=catalog_path,
                minimal_set_json=minimal_set_json,
                scenario=scenario,
                output_profile=output_profile,
                allow_unvalidated=allow_unvalidated,
            )
            summary["selection_mode"] = "deployment_strategy_scenario_profile_catalog_minimal_set"
        else:
            summary = select_profile(
                catalog_json=catalog_path,
                scenario=scenario,
                output_profile=output_profile,
                allow_unvalidated=allow_unvalidated,
            )
            summary["selection_mode"] = "deployment_strategy_scenario_profile_catalog"
        summary["deployment_strategy"] = strategy
        summary["deployment_strategy_json"] = str(deployment_strategy_json)
        return summary
    if scenario is not None:
        catalog_path = catalog_json or (sweep_root / "scenario_profile_catalog" / "manifest.json")
        summary = select_profile(
            catalog_json=catalog_path,
            scenario=scenario,
            output_profile=output_profile,
            allow_unvalidated=allow_unvalidated,
        )
        summary["selection_mode"] = "deployment_strategy_manual_scenario_override"
        summary["deployment_strategy"] = strategy
        summary["deployment_strategy_json"] = str(deployment_strategy_json)
        return summary
    raise ValueError("deployment strategy has no validated automatic runtime profile; provide --scenario only for an explicit override")


def write_summary(path: Path, summary: Mapping[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    if args.deployment_strategy_json is not None:
        summary = select_profile_from_deployment_strategy(
            deployment_strategy_json=args.deployment_strategy_json,
            catalog_json=args.catalog_json,
            minimal_set_json=args.minimal_set_json,
            scenario=args.scenario,
            output_profile=args.output_profile,
            allow_unvalidated=args.allow_unvalidated,
        )
    else:
        if args.catalog_json is None or args.scenario is None:
            raise ValueError("--catalog-json and --scenario are required unless --deployment-strategy-json is provided")
        if args.minimal_set_json is not None:
            summary = select_profile_from_minimal_set(
                catalog_json=args.catalog_json,
                minimal_set_json=args.minimal_set_json,
                scenario=args.scenario,
                output_profile=args.output_profile,
                allow_unvalidated=args.allow_unvalidated,
            )
        else:
            summary = select_profile(
                catalog_json=args.catalog_json,
                scenario=args.scenario,
                output_profile=args.output_profile,
                allow_unvalidated=args.allow_unvalidated,
            )
    summary_path = args.selection_summary_json
    if summary_path is None:
        summary_path = args.output_profile.with_suffix(args.output_profile.suffix + ".selection.json")
    write_summary(summary_path, summary)
    print(f"selected profile -> {args.output_profile}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
