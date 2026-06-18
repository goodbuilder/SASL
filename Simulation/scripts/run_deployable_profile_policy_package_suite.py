#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Mapping, Sequence


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import check_deployable_profile_candidate_package as package_checker  # noqa: E402
import check_deployable_profile_candidate_package_portability as portability_checker  # noqa: E402
import check_deployable_profile_candidate_readiness as readiness_checker  # noqa: E402
import check_deployable_profile_deliverables as deliverables_checker  # noqa: E402
import check_rtk_stereo_goal_acceptance as goal_acceptance_checker  # noqa: E402
import check_stereodistance_operator_view_launch as operator_checker  # noqa: E402
import export_deployable_profile_candidate_from_policy_selection as candidate_exporter  # noqa: E402
import export_deployable_profile_candidate_package as package_exporter  # noqa: E402
import install_stereodistance_active_profile_package as profile_installer  # noqa: E402
import select_deployable_profile_from_common_csv_matrix as policy_selector  # noqa: E402
import summarize_deployable_profile_candidate_packages as package_summarizer  # noqa: E402


POLICIES = ("range_priority", "target_priority", "score_priority", "balanced")


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run the full deployable-profile policy package suite: policy selection, active candidate export, "
            "readiness, package export/check, operator-view dry-run, portability smoke check, package summary, "
            "and optional selected-package install/deliverables/goal-acceptance audit."
        )
    )
    parser.add_argument("--matrix-summary-json", type=Path, required=True)
    parser.add_argument("--runtime-profiles-dir", type=Path, required=True)
    parser.add_argument("--sample-csv", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--policies", default=",".join(POLICIES))
    parser.add_argument("--split", default="validation")
    parser.add_argument("--max-samples", type=int, default=80)
    parser.add_argument("--target-error-limit-m", type=float, default=10.0)
    parser.add_argument(
        "--pipeline-dir",
        type=Path,
        help=(
            "Optional original calibration pipeline directory. When provided, the selected package is installed "
            "to an install_rehearsal directory and audited against required CSV/PNG/TXT deliverables and goal requirements."
        ),
    )
    parser.add_argument("--readme", type=Path, default=Path("Simulation/README.md"))
    return parser.parse_args(argv)


def write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def write_text(path: Path, payload: Mapping[str, object]) -> None:
    lines = [
        "Deployable profile policy package suite",
        "",
        f"status: {payload.get('status', '')}",
        f"selected_policy: {payload.get('selected_policy', '')}",
        f"selected_method: {payload.get('selected_method', '')}",
        f"selected_package_dir: {payload.get('selected_package_dir', '')}",
        f"policy_count: {payload.get('policy_count', '')}",
        f"pass_count: {payload.get('pass_count', '')}",
        f"runtime_truth_used: {payload.get('runtime_truth_used', '')}",
        f"selected_install_status: {payload.get('selected_install_status', '')}",
        f"selected_deliverables_status: {payload.get('selected_deliverables_status', '')}",
        f"selected_goal_acceptance_status: {payload.get('selected_goal_acceptance_status', '')}",
        f"axis_contract: {payload.get('axis_contract', '')}",
        f"public_topic_contract: {payload.get('public_topic_contract', '')}",
        "",
        "Policy runs:",
    ]
    for row in payload.get("policy_runs", []):
        if not isinstance(row, Mapping):
            continue
        lines.append(
            (
                "- {policy}: {status} method={method} package={package_status} "
                "operator={operator_status} portability={portability_status} "
                "rmse_mean={rmse_mean} target_max={target_max}"
            ).format(
                policy=row.get("policy", ""),
                status=row.get("status", ""),
                method=row.get("method", ""),
                package_status=row.get("package_check_status", ""),
                operator_status=row.get("operator_view_launch_status", ""),
                portability_status=row.get("portability_status", ""),
                rmse_mean=row.get("validation_range_rmse_corrected_mean_m", ""),
                target_max=row.get("target_max_abs_error_corrected_max_m", ""),
            )
        )
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_policy_list(text: str) -> list[str]:
    policies = [item.strip() for item in text.split(",") if item.strip()]
    unknown = [policy for policy in policies if policy not in POLICIES]
    if unknown:
        raise ValueError(f"unknown policy: {','.join(unknown)}")
    return policies


def run_suite(
    *,
    matrix_summary_json: Path,
    runtime_profiles_dir: Path,
    sample_csv: Path,
    output_dir: Path,
    policies: Sequence[str],
    split: str = "validation",
    max_samples: int = 80,
    target_error_limit_m: float = 10.0,
    pipeline_dir: Path | None = None,
    readme: Path = Path("Simulation/README.md"),
) -> dict[str, object]:
    output_dir.mkdir(parents=True, exist_ok=True)
    package_dirs: list[Path] = []
    policy_runs: list[dict[str, object]] = []
    for policy in policies:
        policy_dir = output_dir / policy
        selection_dir = policy_dir / "selection"
        selection = policy_selector.select_profiles(
            matrix_summary_json=matrix_summary_json,
            runtime_profiles_dir=runtime_profiles_dir,
            target_error_limit_m=target_error_limit_m,
            recommended_policy=policy,
        )
        policy_selector.write_outputs(selection_dir, selection)

        active_dir = policy_dir / "active_candidate"
        candidate = candidate_exporter.export_candidate(
            policy_selection_json=selection_dir / "deployable_profile_policy_selection.json",
            output_active_profile=active_dir / "active_runtime_profile.json",
            output_dir=active_dir,
            sample_csv=sample_csv,
            split=split,
            max_samples=max_samples,
        )

        readiness = readiness_checker.check_candidate_readiness(active_dir / "deployable_profile_candidate_export.json")
        readiness_checker.write_json(active_dir / "deployable_profile_candidate_readiness.json", readiness)
        readiness_checker.write_text(active_dir / "deployable_profile_candidate_readiness.txt", readiness)
        readiness_checker.write_csv(active_dir / "deployable_profile_candidate_readiness.csv", [readiness])

        package_dir = policy_dir / "candidate_package"
        package_exporter.export_package(
            candidate_readiness_json=active_dir / "deployable_profile_candidate_readiness.json",
            output_dir=package_dir,
        )
        package_check = package_checker.check_package(package_dir)
        package_checker.write_json(package_dir / "deployable_profile_candidate_package_check.json", package_check)
        package_checker.write_text(package_dir / "deployable_profile_candidate_package_check.txt", package_check)

        operator_check = operator_checker.check_operator_view_launch(package_dir=package_dir)
        operator_checker.write_json(package_dir / "operator_view_launch_check.json", operator_check)
        operator_checker.write_text(package_dir / "operator_view_launch_check.txt", operator_check)

        portability_check = portability_checker.check_portability(package_dir=package_dir)
        portability_checker.write_json(
            package_dir / "deployable_profile_candidate_package_portability.json",
            portability_check,
        )
        portability_checker.write_text(
            package_dir / "deployable_profile_candidate_package_portability.txt",
            portability_check,
        )

        package_dirs.append(package_dir)
        policy_runs.append(
            {
                "policy": policy,
                "status": "PASS"
                if selection.get("status") == "PASS"
                and candidate.get("status") == "PASS"
                and readiness.get("status") == "PASS"
                and package_check.get("status") == "PASS"
                and operator_check.get("status") == "PASS"
                and portability_check.get("status") == "PASS"
                else "FAIL",
                "method": candidate.get("method", ""),
                "selection_status": selection.get("status", ""),
                "candidate_status": candidate.get("status", ""),
                "readiness_status": readiness.get("status", ""),
                "package_check_status": package_check.get("status", ""),
                "operator_view_launch_status": operator_check.get("status", ""),
                "portability_status": portability_check.get("status", ""),
                "package_dir": str(package_dir),
                "active_profile_sha256": candidate.get("active_profile_sha256", ""),
                "validation_range_rmse_corrected_mean_m": readiness.get("validation_range_rmse_corrected_mean_m", ""),
                "validation_range_rmse_corrected_max_m": readiness.get("validation_range_rmse_corrected_max_m", ""),
                "target_max_abs_error_corrected_max_m": readiness.get("target_max_abs_error_corrected_max_m", ""),
                "runtime_truth_used": False,
            }
        )

    package_summary_dir = output_dir / "package_summary"
    package_summary = package_summarizer.summarize(package_dirs)
    package_summarizer.write_outputs(package_summary_dir, package_summary)

    selected_install: dict[str, object] = {}
    selected_deliverables: dict[str, object] = {}
    selected_goal_acceptance: dict[str, object] = {}
    selected_package_dir_text = str(package_summary.get("selected_package_dir", ""))
    selected_policy = str(package_summary.get("selected_policy", ""))
    selected_package_dir = Path(selected_package_dir_text) if selected_package_dir_text else None
    if pipeline_dir is not None and selected_package_dir is not None and package_summary.get("status") == "PASS":
        selected_policy_dir = output_dir / selected_policy if selected_policy else selected_package_dir.parent
        install_dir = selected_policy_dir / "install_rehearsal"
        selected_install = profile_installer.install_package(
            package_dir=selected_package_dir,
            output_profile=install_dir / "active_runtime_profile.json",
            force=True,
        )
        profile_installer.write_json(install_dir / "install_manifest.json", selected_install)
        profile_installer.write_text(install_dir / "install_manifest.txt", selected_install)

        deliverables_dir = selected_policy_dir / "deliverables_check"
        selected_deliverables = deliverables_checker.check_deliverables(
            pipeline_dir=pipeline_dir,
            package_summary_json=package_summary_dir / "deployable_profile_candidate_package_summary.json",
            candidate_package_dir=selected_package_dir,
            install_manifest_json=install_dir / "install_manifest.json",
            target_error_limit_m=target_error_limit_m,
        )
        deliverables_checker.write_json(
            deliverables_dir / "deployable_profile_deliverables_check.json",
            selected_deliverables,
        )
        deliverables_checker.write_text(
            deliverables_dir / "deployable_profile_deliverables_check.txt",
            selected_deliverables,
        )

    pass_count = sum(1 for row in policy_runs if row.get("status") == "PASS")
    status = (
        "PASS"
        if pass_count == len(policy_runs)
        and package_summary.get("status") == "PASS"
        and (not selected_install or selected_install.get("status") == "PASS")
        and (pipeline_dir is None or selected_deliverables.get("status") == "PASS")
        else "FAIL"
    )
    summary = {
        "mode": "deployable_profile_policy_package_suite",
        "status": status,
        "matrix_summary_json": str(matrix_summary_json),
        "runtime_profiles_dir": str(runtime_profiles_dir),
        "sample_csv": str(sample_csv),
        "pipeline_dir": str(pipeline_dir) if pipeline_dir is not None else "",
        "readme": str(readme),
        "output_dir": str(output_dir),
        "policies": list(policies),
        "policy_count": len(policy_runs),
        "pass_count": pass_count,
        "selected_policy": package_summary.get("selected_policy", ""),
        "selected_method": package_summary.get("selected_method", ""),
        "selected_package_dir": package_summary.get("selected_package_dir", ""),
        "selected_active_profile_sha256": package_summary.get("selected_active_profile_sha256", ""),
        "selected_install_status": selected_install.get("status", ""),
        "selected_install_manifest_json": str(output_dir / str(package_summary.get("selected_policy", "")) / "install_rehearsal" / "install_manifest.json")
        if selected_install
        else "",
        "selected_deliverables_status": selected_deliverables.get("status", ""),
        "selected_deliverables_check_json": str(
            output_dir / str(package_summary.get("selected_policy", "")) / "deliverables_check" / "deployable_profile_deliverables_check.json"
        )
        if selected_deliverables
        else "",
        "selected_goal_acceptance_status": selected_goal_acceptance.get("status", ""),
        "selected_goal_acceptance_json": str(
            output_dir / str(package_summary.get("selected_policy", "")) / "goal_acceptance" / "rtk_stereo_goal_acceptance.json"
        )
        if selected_goal_acceptance
        else "",
        "package_summary_dir": str(package_summary_dir),
        "runtime_truth_used": False,
        "axis_contract": "stereo_center_opencv_optical:RDF(right,down,forward)",
        "public_topic_contract": "stereodistance_position_topics:FRD_range(forward,right,down,range)",
        "policy_runs": policy_runs,
    }
    write_json(output_dir / "deployable_profile_policy_package_suite_summary.json", summary)
    write_text(output_dir / "deployable_profile_policy_package_suite_summary.txt", summary)
    if pipeline_dir is not None and selected_package_dir is not None and selected_deliverables:
        selected_policy_dir = output_dir / selected_policy if selected_policy else selected_package_dir.parent
        deliverables_dir = selected_policy_dir / "deliverables_check"
        goal_acceptance_dir = selected_policy_dir / "goal_acceptance"
        selected_goal_acceptance = goal_acceptance_checker.check_acceptance(
            suite_summary_json=output_dir / "deployable_profile_policy_package_suite_summary.json",
            deliverables_json=deliverables_dir / "deployable_profile_deliverables_check.json",
            candidate_package_dir=selected_package_dir,
            readme=readme,
            target_error_limit_m=target_error_limit_m,
        )
        goal_acceptance_checker.write_json(
            goal_acceptance_dir / "rtk_stereo_goal_acceptance.json",
            selected_goal_acceptance,
        )
        goal_acceptance_checker.write_csv(
            goal_acceptance_dir / "rtk_stereo_goal_acceptance.csv",
            selected_goal_acceptance.get("requirements", [])
            if isinstance(selected_goal_acceptance.get("requirements", []), list)
            else [],
        )
        goal_acceptance_checker.write_text(
            goal_acceptance_dir / "rtk_stereo_goal_acceptance.txt",
            selected_goal_acceptance,
        )
        summary["selected_goal_acceptance_status"] = selected_goal_acceptance.get("status", "")
        summary["selected_goal_acceptance_json"] = str(goal_acceptance_dir / "rtk_stereo_goal_acceptance.json")
        if selected_goal_acceptance.get("status") != "PASS":
            summary["status"] = "FAIL"
        write_json(output_dir / "deployable_profile_policy_package_suite_summary.json", summary)
        write_text(output_dir / "deployable_profile_policy_package_suite_summary.txt", summary)
    return summary


def run(argv: Sequence[str] | None = None) -> dict[str, object]:
    args = parse_args(argv)
    return run_suite(
        matrix_summary_json=args.matrix_summary_json,
        runtime_profiles_dir=args.runtime_profiles_dir,
        sample_csv=args.sample_csv,
        output_dir=args.output_dir,
        policies=parse_policy_list(args.policies),
        split=args.split,
        max_samples=args.max_samples,
        target_error_limit_m=args.target_error_limit_m,
        pipeline_dir=args.pipeline_dir,
        readme=args.readme,
    )


def main() -> int:
    result = run()
    compact = {
        key: result.get(key)
        for key in (
            "status",
            "selected_policy",
            "selected_method",
            "selected_package_dir",
            "policy_count",
            "pass_count",
            "selected_install_status",
            "selected_deliverables_status",
            "selected_goal_acceptance_status",
            "runtime_truth_used",
        )
    }
    print(json.dumps(compact, ensure_ascii=False, indent=2, allow_nan=False))
    return 0 if result.get("status") == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
