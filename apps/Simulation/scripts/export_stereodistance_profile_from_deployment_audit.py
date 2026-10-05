#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Mapping, Sequence


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_DIR = ROOT_DIR / "scripts"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from explore_stereodistance_profile_blends import (  # noqa: E402
    explore_blends,
    parse_float_list,
)
from select_stereodistance_profile_by_calibration_replay import (  # noqa: E402
    parse_split_list,
    select_by_calibration_replay,
)
from select_stereodistance_profile_from_catalog import (  # noqa: E402
    PUBLIC_TOPIC_CONTRACT,
    RDF_AXIS_CONTRACT,
)
from stereodistance_profile_promotion_check import run_check  # noqa: E402


SINGLE_PROFILE_STRATEGY = "single_profile_calibration_replay_all_catalog"
SINGLE_PROFILE_MINIMAL_SET_STRATEGY = "single_profile_calibration_replay_minimal_set"
BLEND_POLICY_STRATEGY = "blend_policy_calibration_replay_all_catalog"
BLEND_POLICY_MINIMAL_SET_STRATEGY = "blend_policy_calibration_replay_minimal_set"
SUPPORTED_STRATEGIES = (
    SINGLE_PROFILE_STRATEGY,
    SINGLE_PROFILE_MINIMAL_SET_STRATEGY,
    BLEND_POLICY_STRATEGY,
    BLEND_POLICY_MINIMAL_SET_STRATEGY,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Export an active StereoDistance runtime profile from a post-sweep deployment audit "
            "recommendation. Calibration truth in --input-csv is used only for offline selection/replay; "
            "the exported runtime profile consumes raw RDF/disparity only."
        )
    )
    parser.add_argument("--deployment-audit-json", type=Path, required=True)
    parser.add_argument("--catalog-json", type=Path, required=True)
    parser.add_argument("--input-csv", type=Path, required=True)
    parser.add_argument("--output-profile", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--minimal-set-json", type=Path)
    parser.add_argument("--split", default="validation")
    parser.add_argument("--stability-splits", default="train,validation")
    parser.add_argument("--target-range-m", type=float, default=1852.0)
    parser.add_argument("--target-window-m", type=float, default=30.0)
    parser.add_argument("--target-error-limit-m", type=float, default=10.0)
    parser.add_argument("--alphas", default="0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1")
    parser.add_argument(
        "--candidate-source-mode",
        choices=("from_recommendation", "minimal_set", "all_catalog"),
        default="from_recommendation",
        help=(
            "from_recommendation derives minimal_set/all_catalog from the deployment-audit strategy name. "
            "minimal_set/all_catalog override the audit-derived scope."
        ),
    )
    parser.add_argument(
        "--selection-policy",
        choices=("from_recommendation", "balanced", "validation_best", "target_guard", "simple_stable", "replay_best"),
        default="from_recommendation",
        help="Blend policy to export. from_recommendation uses deployment_audit_strategy_recommendation.json.",
    )
    parser.add_argument("--allow-unvalidated", action="store_true")
    parser.add_argument("--skip-promotion-check", action="store_true")
    parser.add_argument("--promotion-max-samples", type=int, default=50)
    return parser.parse_args()


def read_json(path: Path) -> dict[str, object]:
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def write_text_summary(path: Path, summary: Mapping[str, object]) -> None:
    lines = [
        "StereoDistance active profile export from deployment audit",
        "",
        f"selected_strategy: {summary.get('selected_strategy', '')}",
        f"selected_policy: {summary.get('selected_policy', '')}",
        f"candidate_source_mode: {summary.get('candidate_source_mode', '')}",
        f"output_profile: {summary.get('output_profile', '')}",
        f"promotion_check_passed: {summary.get('promotion_check_passed', '')}",
        f"axis_contract: {summary.get('axis_contract', RDF_AXIS_CONTRACT)}",
        f"public_topic_contract: {summary.get('public_topic_contract', PUBLIC_TOPIC_CONTRACT)}",
        f"selection_contract: {summary.get('selection_contract', '')}",
        f"runtime_input_contract: {summary.get('runtime_input_contract', '')}",
        "",
        "Replay metrics:",
        f"validation_range_rmse_corrected_m: {summary.get('validation_range_rmse_corrected_m', '')}",
        f"target_abs_mean_error_corrected_m: {summary.get('target_abs_mean_error_corrected_m', '')}",
        f"target_max_abs_error_corrected_m: {summary.get('target_max_abs_error_corrected_m', '')}",
        f"replay_selection_score: {summary.get('replay_selection_score', '')}",
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def resolve_candidate_source_mode(strategy: str, requested: str) -> str:
    if requested != "from_recommendation":
        return requested
    if strategy.endswith("_minimal_set"):
        return "minimal_set"
    if strategy.endswith("_all_catalog"):
        return "all_catalog"
    raise ValueError(f"unsupported deployment audit strategy: {strategy!r}")


def is_blend_strategy(strategy: str) -> bool:
    return strategy.startswith("blend_policy_calibration_replay_")


def is_single_profile_strategy(strategy: str) -> bool:
    return strategy.startswith("single_profile_calibration_replay_")


def resolve_selection_policy(recommendation: Mapping[str, object], requested: str) -> str:
    if requested != "from_recommendation":
        return requested
    policy = str(recommendation.get("recommended_policy", "") or "")
    return policy if policy else "balanced"


def export_profile_from_deployment_audit(
    *,
    deployment_audit_json: Path,
    catalog_json: Path,
    input_csv: Path,
    output_profile: Path,
    output_dir: Path,
    minimal_set_json: Path | None = None,
    split: str = "validation",
    stability_splits: Sequence[str] = ("train", "validation"),
    target_range_m: float = 1852.0,
    target_window_m: float = 30.0,
    target_error_limit_m: float = 10.0,
    alphas: Sequence[float] = tuple(i / 10 for i in range(11)),
    candidate_source_mode: str = "from_recommendation",
    selection_policy: str = "from_recommendation",
    allow_unvalidated: bool = False,
    run_promotion: bool = True,
    promotion_max_samples: int = 50,
) -> dict[str, object]:
    recommendation = read_json(deployment_audit_json)
    strategy = str(recommendation.get("recommended_strategy", "") or "")
    if strategy not in SUPPORTED_STRATEGIES:
        raise ValueError(f"unsupported deployment audit strategy: {strategy!r}")
    resolved_candidate_source_mode = resolve_candidate_source_mode(strategy, candidate_source_mode)
    resolved_policy = resolve_selection_policy(recommendation, selection_policy)

    output_dir.mkdir(parents=True, exist_ok=True)
    if is_blend_strategy(strategy):
        selection_summary = explore_blends(
            catalog_json=catalog_json,
            input_csv=input_csv,
            output_dir=output_dir / "blend_profile_export",
            output_profile=output_profile,
            minimal_set_json=minimal_set_json,
            split=split,
            stability_splits=stability_splits,
            target_range_m=target_range_m,
            target_window_m=target_window_m,
            target_error_limit_m=target_error_limit_m,
            alphas=alphas,
            candidate_source_mode=resolved_candidate_source_mode,
            selection_policy=resolved_policy,
        )
    elif is_single_profile_strategy(strategy):
        selection_summary = select_by_calibration_replay(
            catalog_json=catalog_json,
            input_csv=input_csv,
            output_profile=output_profile,
            output_dir=output_dir / "single_profile_export",
            minimal_set_json=minimal_set_json,
            split=split,
            target_range_m=target_range_m,
            target_window_m=target_window_m,
            target_error_limit_m=target_error_limit_m,
            allow_unvalidated=allow_unvalidated,
            stability_splits=stability_splits,
            candidate_source_mode=resolved_candidate_source_mode,
        )
    else:
        raise ValueError(f"unsupported deployment audit strategy: {strategy!r}")

    promotion_summary: dict[str, object] | None = None
    if run_promotion:
        promotion_summary = run_check(
            output_profile,
            output_dir / "promotion_check",
            sample_csv=input_csv,
            split=split,
            max_samples=promotion_max_samples,
        )

    summary: dict[str, object] = {
        "selected_strategy": strategy,
        "selected_policy": resolved_policy if is_blend_strategy(strategy) else "single_best",
        "candidate_source_mode": resolved_candidate_source_mode,
        "deployment_audit_json": str(deployment_audit_json),
        "catalog_json": str(catalog_json),
        "input_csv": str(input_csv),
        "output_profile": str(output_profile),
        "selection_output_dir": str(output_dir / ("blend_profile_export" if is_blend_strategy(strategy) else "single_profile_export")),
        "promotion_check_dir": str(output_dir / "promotion_check") if run_promotion else "",
        "promotion_check_passed": bool(promotion_summary and promotion_summary.get("pass_promotion_check")) if run_promotion else "",
        "validation_range_rmse_corrected_m": selection_summary.get("validation_range_rmse_corrected_m", ""),
        "target_abs_mean_error_corrected_m": selection_summary.get("target_abs_mean_error_corrected_m", ""),
        "target_max_abs_error_corrected_m": selection_summary.get("target_max_abs_error_corrected_m", ""),
        "replay_selection_score": selection_summary.get("replay_selection_score", ""),
        "selection_summary": selection_summary,
        "promotion_summary": promotion_summary or {},
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
        "selection_contract": (
            "Calibration-time truth from the CSV may select the strategy/profile. Runtime StereoDistance "
            "must use only raw RDF/disparity and fixed exported profile parameters."
        ),
        "runtime_input_contract": "exported active profile consumes raw stereo-center RDF/disparity only; no runtime truth",
    }
    write_json(output_dir / "deployment_audit_profile_export_summary.json", summary)
    write_text_summary(output_dir / "deployment_audit_profile_export_summary.txt", summary)
    return summary


def main() -> int:
    args = parse_args()
    summary = export_profile_from_deployment_audit(
        deployment_audit_json=args.deployment_audit_json,
        catalog_json=args.catalog_json,
        input_csv=args.input_csv,
        output_profile=args.output_profile,
        output_dir=args.output_dir,
        minimal_set_json=args.minimal_set_json,
        split=args.split,
        stability_splits=parse_split_list(args.stability_splits),
        target_range_m=args.target_range_m,
        target_window_m=args.target_window_m,
        target_error_limit_m=args.target_error_limit_m,
        alphas=parse_float_list(args.alphas),
        candidate_source_mode=args.candidate_source_mode,
        selection_policy=args.selection_policy,
        allow_unvalidated=args.allow_unvalidated,
        run_promotion=not args.skip_promotion_check,
        promotion_max_samples=args.promotion_max_samples,
    )
    print(f"exported active runtime profile -> {summary['output_profile']}")
    print(
        "strategy={strategy}, policy={policy}, promotion_passed={promotion}".format(
            strategy=summary["selected_strategy"],
            policy=summary["selected_policy"],
            promotion=summary["promotion_check_passed"],
        )
    )
    return 0 if summary["promotion_check_passed"] in (True, "") else 1


if __name__ == "__main__":
    raise SystemExit(main())
