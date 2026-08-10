#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_DIR = ROOT_DIR / "scripts"
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from audit_stereodistance_profile_blend_strategies import audit_blend_strategies  # noqa: E402
from audit_stereodistance_profile_selection_strategies import audit_selection_strategies  # noqa: E402
from select_stereodistance_profile_by_calibration_replay import parse_split_list  # noqa: E402
from summarize_stereodistance_deployment_audits import (  # noqa: E402
    PUBLIC_TOPIC_CONTRACT,
    RDF_AXIS_CONTRACT,
    summarize_deployment_audits,
    write_json,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run the post-sweep StereoDistance deployment audit pipeline: all-catalog profile "
            "selection audit, all-catalog blend audit, then a final offline deployment strategy "
            "summary. Runtime profiles still consume raw RDF/disparity only."
        )
    )
    parser.add_argument("--sweep-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--catalog-json", type=Path)
    parser.add_argument("--minimal-set-json", type=Path)
    parser.add_argument("--split", default="validation")
    parser.add_argument("--stability-splits", default="train,validation")
    parser.add_argument("--target-range-m", type=float, default=1852.0)
    parser.add_argument("--target-window-m", type=float, default=30.0)
    parser.add_argument("--target-error-limit-m", type=float, default=10.0)
    parser.add_argument("--alphas", default="0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1")
    parser.add_argument(
        "--candidate-source-mode",
        choices=("all_catalog", "minimal_set"),
        default="all_catalog",
        help="Use all_catalog for broad offline search; minimal_set reproduces deployment-ready set-cover scope.",
    )
    parser.add_argument(
        "--skip-blend-audit",
        action="store_true",
        help="Run only profile selection audit and final summary. Useful for quick diagnostics.",
    )
    return parser.parse_args()


def parse_float_list(value: str) -> list[float]:
    parsed: list[float] = []
    for item in value.split(","):
        text = item.strip()
        if text:
            parsed.append(float(text))
    return list(dict.fromkeys(parsed))


def run_post_sweep_deployment_audit(
    *,
    sweep_dir: Path,
    output_dir: Path | None = None,
    catalog_json: Path | None = None,
    minimal_set_json: Path | None = None,
    split: str = "validation",
    stability_splits: list[str] | tuple[str, ...] = ("train", "validation"),
    target_range_m: float = 1852.0,
    target_window_m: float = 30.0,
    target_error_limit_m: float = 10.0,
    alphas: list[float] | tuple[float, ...] = tuple(i / 10.0 for i in range(11)),
    candidate_source_mode: str = "all_catalog",
    skip_blend_audit: bool = False,
) -> dict[str, object]:
    root = output_dir or (sweep_dir / "post_sweep_deployment_audit")
    selection_dir = root / "profile_selection_strategy_audit"
    blend_dir = root / "profile_blend_strategy_audit"
    summary_dir = root / "deployment_audit_strategy_summary"
    root.mkdir(parents=True, exist_ok=True)

    audit_selection_strategies(
        sweep_dir=sweep_dir,
        output_dir=selection_dir,
        catalog_json=catalog_json,
        minimal_set_json=minimal_set_json,
        split=split,
        target_range_m=target_range_m,
        target_window_m=target_window_m,
        target_error_limit_m=target_error_limit_m,
        stability_splits=stability_splits,
        candidate_source_mode=candidate_source_mode,
    )
    blend_summary: dict[str, object] | None = None
    if not skip_blend_audit:
        _, _, blend_summary = audit_blend_strategies(
            sweep_dir=sweep_dir,
            output_dir=blend_dir,
            catalog_json=catalog_json,
            minimal_set_json=minimal_set_json,
            split=split,
            stability_splits=stability_splits,
            target_range_m=target_range_m,
            target_window_m=target_window_m,
            target_error_limit_m=target_error_limit_m,
            alphas=alphas,
            candidate_source_mode=candidate_source_mode,
        )
    _, recommendation = summarize_deployment_audits(
        sweep_dir=sweep_dir,
        output_dir=summary_dir,
        selection_audit_dir=selection_dir,
        blend_audit_dir=None if skip_blend_audit else blend_dir,
    )
    manifest = {
        "post_sweep_deployment_audit": True,
        "sweep_dir": str(sweep_dir),
        "output_dir": str(root),
        "candidate_source_mode": candidate_source_mode,
        "selection_audit_dir": str(selection_dir),
        "blend_audit_dir": "" if skip_blend_audit else str(blend_dir),
        "deployment_audit_strategy_summary_dir": str(summary_dir),
        "recommended_strategy": recommendation.get("recommended_strategy", ""),
        "recommended_policy": recommendation.get("recommended_policy", ""),
        "blend_audit_policy_count": None if blend_summary is None else blend_summary.get("policy_count"),
        "selection_contract": (
            "Calibration-time truth is used only for offline audit and profile/blend policy selection. "
            "Runtime profile application must use only raw RDF/disparity and fixed calibration parameters."
        ),
        "axis_contract": RDF_AXIS_CONTRACT,
        "public_topic_contract": PUBLIC_TOPIC_CONTRACT,
    }
    write_json(root / "post_sweep_deployment_audit_manifest.json", manifest)
    return manifest


def main() -> int:
    args = parse_args()
    manifest = run_post_sweep_deployment_audit(
        sweep_dir=args.sweep_dir,
        output_dir=args.output_dir,
        catalog_json=args.catalog_json,
        minimal_set_json=args.minimal_set_json,
        split=args.split,
        stability_splits=parse_split_list(args.stability_splits),
        target_range_m=args.target_range_m,
        target_window_m=args.target_window_m,
        target_error_limit_m=args.target_error_limit_m,
        alphas=parse_float_list(args.alphas),
        candidate_source_mode=args.candidate_source_mode,
        skip_blend_audit=args.skip_blend_audit,
    )
    print(json.dumps(manifest, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
