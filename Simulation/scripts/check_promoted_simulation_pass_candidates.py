#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path
from typing import Mapping


ROOT_DIR = Path(__file__).resolve().parents[1]
REPO_DIR = ROOT_DIR.parent
PACKAGE_SCRIPT = ROOT_DIR / "scripts" / "run_observation_fusion_deployment_package.py"
DEFAULT_TIER_SPEC = ROOT_DIR / "config" / "observation_fusion_fresh_deployment_tiers.json"
DEFAULT_RESIDUAL_PROFILE = ROOT_DIR / "config" / "disparity_residual_profile_disparity_noise_stress.json"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Re-run repo-local gates for promoted Simulation PASS candidates.",
    )
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--tier-spec-json", type=Path, default=DEFAULT_TIER_SPEC)
    parser.add_argument("--disparity-residual-profile-json", type=Path, default=DEFAULT_RESIDUAL_PROFILE)
    parser.add_argument("--skip-residual", action="store_true")
    return parser.parse_args()


def load_json(path: Path) -> dict[str, object]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"expected JSON object: {path}")
    return payload


def require(condition: bool, failures: list[str], reason: str) -> None:
    if not condition:
        failures.append(reason)


def run_package_check(
    *,
    output_dir: Path,
    tier_spec_json: Path,
    residual_profile_json: Path | None,
) -> dict[str, object]:
    command = [
        sys.executable,
        str(PACKAGE_SCRIPT),
        "--output-dir",
        str(output_dir),
        "--tier-spec-json",
        str(tier_spec_json),
    ]
    if residual_profile_json is not None:
        command.extend(["--disparity-residual-profile-json", str(residual_profile_json)])
    result = subprocess.run(command, cwd=REPO_DIR, check=False, text=True, capture_output=True)
    summary_path = output_dir / "observation_fusion_deployment_package_summary.json"
    summary = load_json(summary_path) if summary_path.is_file() else {}
    return {
        "command": " ".join(command),
        "returncode": result.returncode,
        "stdout_tail": result.stdout[-4000:],
        "stderr_tail": result.stderr[-4000:],
        "summary_path": str(summary_path),
        "summary": summary,
    }


def evaluate_package_result(
    result: Mapping[str, object],
    *,
    expected_residual_status_prefix: str | None,
) -> dict[str, object]:
    failures: list[str] = []
    summary = result.get("summary", {})
    if not isinstance(summary, Mapping):
        summary = {}
    require(result.get("returncode") == 0, failures, "command:returncode")
    require(summary.get("acceptance_status") == "PASS", failures, "package:acceptance_status")
    require(summary.get("default_tier") == "fresh_common_mode_b7x3_aggregate_best", failures, "package:default_tier")
    require(
        summary.get("stereodistance_node_smoke_fusion_status") == "fused:MultiBurstObservationFusion:21",
        failures,
        "node:fusion_status",
    )
    require(summary.get("operator_view_package_launch_status") == "PASS", failures, "operator:package_launch")
    require(summary.get("runtime_truth_used") is False, failures, "contract:runtime_truth_used")
    require(summary.get("coordinate_contracts_explicit") is True, failures, "contract:coordinate_contracts_explicit")
    residual_status = str(summary.get("stereodistance_node_smoke_disparity_residual_status", ""))
    if expected_residual_status_prefix is None:
        require(residual_status == "disabled", failures, "residual:expected_disabled")
    else:
        require(
            residual_status.startswith(expected_residual_status_prefix),
            failures,
            "residual:expected_applied",
        )
    return {
        "status": "PASS" if not failures else "FAIL",
        "failure_count": len(failures),
        "failures": failures,
        "acceptance_status": summary.get("acceptance_status", ""),
        "default_tier": summary.get("default_tier", ""),
        "fusion_status": summary.get("stereodistance_node_smoke_fusion_status", ""),
        "disparity_residual_status": residual_status,
        "runtime_truth_used": summary.get("runtime_truth_used", True),
        "coordinate_contracts_explicit": summary.get("coordinate_contracts_explicit", False),
    }


def write_outputs(output_dir: Path, payload: Mapping[str, object]) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "promoted_simulation_pass_candidates_check.json").write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    lines = [
        "Promoted Simulation PASS candidates check",
        f"status: {payload.get('status', '')}",
        f"candidate_count: {payload.get('candidate_count', 0)}",
        f"pass_count: {payload.get('pass_count', 0)}",
        f"failure_count: {payload.get('failure_count', 0)}",
    ]
    for row in payload.get("candidates", []):
        if not isinstance(row, Mapping):
            continue
        evaluation = row.get("evaluation", {})
        if not isinstance(evaluation, Mapping):
            evaluation = {}
        lines.append(
            "- {name}: status={status}, fusion={fusion}, residual={residual}".format(
                name=row.get("name", ""),
                status=evaluation.get("status", ""),
                fusion=evaluation.get("fusion_status", ""),
                residual=evaluation.get("disparity_residual_status", ""),
            )
        )
    (output_dir / "promoted_simulation_pass_candidates_check.txt").write_text(
        "\n".join(lines) + "\n",
        encoding="utf-8",
    )


def main() -> int:
    args = parse_args()
    output_dir = Path(args.output_dir)
    tier_spec_json = Path(args.tier_spec_json)
    residual_profile_json = None if args.skip_residual else Path(args.disparity_residual_profile_json)
    candidates: list[dict[str, object]] = []

    base_dir = output_dir / "observation_fusion"
    base_result = run_package_check(
        output_dir=base_dir,
        tier_spec_json=tier_spec_json,
        residual_profile_json=None,
    )
    candidates.append(
        {
            "name": "fresh_common_mode_b7x3_aggregate_best",
            "result": base_result,
            "evaluation": evaluate_package_result(base_result, expected_residual_status_prefix=None),
        }
    )

    if residual_profile_json is not None:
        residual_dir = output_dir / "observation_fusion_with_disparity_residual"
        residual_result = run_package_check(
            output_dir=residual_dir,
            tier_spec_json=tier_spec_json,
            residual_profile_json=residual_profile_json,
        )
        candidates.append(
            {
                "name": "fresh_common_mode_b7x3_aggregate_best+disparity_noise_stress_residual",
                "result": residual_result,
                "evaluation": evaluate_package_result(
                    residual_result,
                    expected_residual_status_prefix="applied:",
                ),
            }
        )

    failure_count = sum(
        int(candidate["evaluation"]["failure_count"])
        for candidate in candidates
        if isinstance(candidate.get("evaluation"), Mapping)
    )
    pass_count = sum(
        1
        for candidate in candidates
        if isinstance(candidate.get("evaluation"), Mapping)
        and candidate["evaluation"].get("status") == "PASS"
    )
    payload = {
        "mode": "promoted_simulation_pass_candidates_check",
        "status": "PASS" if failure_count == 0 and pass_count == len(candidates) else "FAIL",
        "candidate_count": len(candidates),
        "pass_count": pass_count,
        "failure_count": failure_count,
        "tier_spec_json": str(tier_spec_json),
        "disparity_residual_profile_json": "" if residual_profile_json is None else str(residual_profile_json),
        "runtime_truth_used": False,
        "coordinate_contracts_explicit": True,
        "candidates": candidates,
    }
    write_outputs(output_dir, payload)
    print(json.dumps(payload, ensure_ascii=False, indent=2))
    return 0 if payload["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
