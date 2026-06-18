#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
from pathlib import Path
from typing import Mapping, Sequence


SCRIPT_DIR = Path(__file__).resolve().parent
ROOT_DIR = SCRIPT_DIR.parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import check_stereodistance_active_profile_deployment_package as package_checker  # noqa: E402
import check_deployable_profile_candidate_package as candidate_package_checker  # noqa: E402
import check_stereodistance_launch_profile_contract as launch_contract_checker  # noqa: E402
import select_stereodistance_runtime_profile_by_label as label_selector  # noqa: E402
import select_disparity_residual_profile_by_label as residual_label_selector  # noqa: E402
import select_disparity_residual_profile_by_features as residual_feature_selector  # noqa: E402


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Check that a StereoDistance runtime profile/package resolves to an operator-view launch command."
    )
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--package-dir", type=Path)
    source.add_argument("--profile-json", type=Path)
    source.add_argument("--selector-json", type=Path)
    parser.add_argument("--deployment-label", default="")
    parser.add_argument("--scenario-label", default="")
    parser.add_argument("--scenario-name", default="")
    parser.add_argument("--observation-fusion-manifest", type=Path)
    parser.add_argument("--observation-fusion-tier", default="")
    parser.add_argument("--disparity-residual-profile-json", type=Path)
    parser.add_argument("--disparity-residual-decision-json", type=Path)
    parser.add_argument("--disparity-residual-selector-json", type=Path)
    parser.add_argument("--disparity-residual-feature-selector-json", type=Path)
    parser.add_argument("--disparity-residual-feature-catalog-json", type=Path)
    parser.add_argument("--node-disparity-residual-feature-selector-json", type=Path)
    parser.add_argument("--node-disparity-residual-feature-catalog-json", type=Path)
    parser.add_argument("--node-disparity-residual-feature-window-size", type=int, default=0)
    parser.add_argument("--node-disparity-residual-feature-min-samples", type=int, default=0)
    feature = parser.add_mutually_exclusive_group()
    feature.add_argument("--disparity-residual-feature-json", type=Path)
    feature.add_argument("--disparity-residual-feature-csv", type=Path)
    parser.add_argument("--disparity-residual-feature-row-index", type=int, default=0)
    parser.add_argument("--output-json", type=Path)
    parser.add_argument("--output-txt", type=Path)
    return parser.parse_args(argv)


def write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def load_json(path: Path) -> dict[str, object]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"expected JSON object: {path}")
    return payload


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def require(condition: bool, failures: list[str], reason: str) -> None:
    if not condition:
        failures.append(reason)


def has_runtime_truth(values: object) -> bool:
    if not isinstance(values, list):
        return True
    joined = " ".join(str(value).lower() for value in values)
    return any(token in joined for token in ("truth", "gt_", "px4", "rtk", "ned"))


def resolve_profile_from_package(package_dir: Path) -> Path:
    return package_dir / "active_runtime_profile.json"


def check_supported_package(package_dir: Path) -> tuple[str, dict[str, object]]:
    if (package_dir / "deployment_package_manifest.json").is_file():
        return "deployment_package", package_checker.check_package(package_dir)
    if (package_dir / "deployable_profile_candidate_package_manifest.json").is_file():
        return "candidate_package", candidate_package_checker.check_package(package_dir)
    if (package_dir / "active_runtime_profile.json").is_file():
        return "lightweight_profile_package", {
            "status": "PASS",
            "package_dir": str(package_dir),
            "active_profile": str(package_dir / "active_runtime_profile.json"),
            "runtime_truth_used": False,
            "axis_contract": "stereo_center_opencv_optical:RDF(right,down,forward)",
            "public_topic_contract": "stereodistance_position_topics:FRD_range(forward,right,down,range)",
        }
    return "unknown_package", {
        "status": "FAIL",
        "package_dir": str(package_dir),
        "failures": ["package_manifest:missing"],
        "runtime_truth_used": False,
    }


def run_print_only(
    *,
    package_dir: Path | None,
    profile_json: Path | None,
    selector_json: Path | None = None,
    deployment_label: str = "",
    scenario_label: str = "",
    scenario_name: str = "",
    observation_fusion_manifest: Path | None = None,
    observation_fusion_tier: str = "",
    disparity_residual_profile_json: Path | None = None,
    disparity_residual_decision_json: Path | None = None,
    disparity_residual_selector_json: Path | None = None,
    disparity_residual_feature_selector_json: Path | None = None,
    disparity_residual_feature_catalog_json: Path | None = None,
    disparity_residual_feature_json: Path | None = None,
    disparity_residual_feature_csv: Path | None = None,
    disparity_residual_feature_row_index: int = 0,
    node_disparity_residual_feature_selector_json: Path | None = None,
    node_disparity_residual_feature_catalog_json: Path | None = None,
    node_disparity_residual_feature_window_size: int = 0,
    node_disparity_residual_feature_min_samples: int = 0,
) -> subprocess.CompletedProcess[str]:
    command = [str(ROOT_DIR / "run_operator_view_with_calibration_profile.sh")]
    if package_dir is not None:
        command.extend(["--package-dir", str(package_dir)])
    if profile_json is not None:
        command.extend(["--profile-json", str(profile_json)])
    if selector_json is not None:
        command.extend(["--selector-json", str(selector_json)])
    if deployment_label:
        command.extend(["--deployment-label", deployment_label])
    if scenario_label:
        command.extend(["--scenario-label", scenario_label])
    if scenario_name:
        command.extend(["--scenario-name", scenario_name])
    if observation_fusion_manifest is not None:
        command.extend(["--observation-fusion-manifest", str(observation_fusion_manifest)])
    if observation_fusion_tier:
        command.extend(["--observation-fusion-tier", observation_fusion_tier])
    if disparity_residual_profile_json is not None:
        command.extend(["--disparity-residual-profile-json", str(disparity_residual_profile_json)])
    if disparity_residual_decision_json is not None:
        command.extend(["--disparity-residual-decision-json", str(disparity_residual_decision_json)])
    if disparity_residual_selector_json is not None:
        command.extend(["--disparity-residual-selector-json", str(disparity_residual_selector_json)])
    if disparity_residual_feature_selector_json is not None:
        command.extend(["--disparity-residual-feature-selector-json", str(disparity_residual_feature_selector_json)])
    if disparity_residual_feature_catalog_json is not None:
        command.extend(["--disparity-residual-feature-catalog-json", str(disparity_residual_feature_catalog_json)])
    if disparity_residual_feature_json is not None:
        command.extend(["--disparity-residual-feature-json", str(disparity_residual_feature_json)])
    if disparity_residual_feature_csv is not None:
        command.extend(
            [
                "--disparity-residual-feature-csv",
                str(disparity_residual_feature_csv),
                "--disparity-residual-feature-row-index",
                str(disparity_residual_feature_row_index),
            ]
        )
    if node_disparity_residual_feature_selector_json is not None:
        command.extend(
            [
                "--node-disparity-residual-feature-selector-json",
                str(node_disparity_residual_feature_selector_json),
            ]
        )
    if node_disparity_residual_feature_catalog_json is not None:
        command.extend(
            [
                "--node-disparity-residual-feature-catalog-json",
                str(node_disparity_residual_feature_catalog_json),
            ]
        )
    if node_disparity_residual_feature_window_size > 0:
        command.extend(
            [
                "--node-disparity-residual-feature-window-size",
                str(node_disparity_residual_feature_window_size),
            ]
        )
    if node_disparity_residual_feature_min_samples > 0:
        command.extend(
            [
                "--node-disparity-residual-feature-min-samples",
                str(node_disparity_residual_feature_min_samples),
            ]
        )
    command.extend(["--print-only", "--", "--no-px4", "yolo_target_hz:=0.0"])
    return subprocess.run(
        command,
        cwd=ROOT_DIR,
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )


def extract_runtime_profile(stdout: str) -> str:
    for line in stdout.splitlines():
        if line.startswith("Runtime profile:"):
            return line.split(":", 1)[1].strip()
    return ""


def extract_observation_fusion_manifest(stdout: str) -> str:
    for line in stdout.splitlines():
        if line.startswith("Observation fusion manifest:"):
            return line.split(":", 1)[1].strip()
    return ""


def extract_disparity_residual_profile(stdout: str) -> str:
    for line in stdout.splitlines():
        if line.startswith("Disparity residual profile:"):
            return line.split(":", 1)[1].strip()
    return ""


def extract_disparity_residual_decision(stdout: str) -> str:
    for line in stdout.splitlines():
        if line.startswith("Residual decision:"):
            return line.split(":", 1)[1].strip()
    return ""


def extract_disparity_residual_decision_label(stdout: str) -> str:
    for line in stdout.splitlines():
        if line.startswith("Residual decision selected label:"):
            return line.split(":", 1)[1].strip()
    return ""


def extract_node_disparity_residual_feature_selector(stdout: str) -> str:
    for line in stdout.splitlines():
        if line.startswith("Node disparity residual feature selector:"):
            return line.split(":", 1)[1].strip()
    return ""


def extract_node_disparity_residual_feature_catalog(stdout: str) -> str:
    for line in stdout.splitlines():
        if line.startswith("Node disparity residual feature catalog:"):
            return line.split(":", 1)[1].strip()
    return ""


def check_observation_fusion_manifest(path: Path) -> dict[str, object]:
    if not path.is_file():
        return {
            "status": "FAIL",
            "manifest": str(path),
            "failure_count": 1,
            "failures": ["manifest:missing"],
            "runtime_truth_used": False,
        }
    failures: list[str] = []
    payload = load_json(path)
    tiers = payload.get("tiers")
    require(isinstance(tiers, list) and bool(tiers), failures, "manifest:tiers")
    runtime_truth_used = bool(payload.get("runtime_truth_used", False))
    for tier in tiers if isinstance(tiers, list) else []:
        if not isinstance(tier, Mapping):
            failures.append("manifest:tier_object")
            continue
        adapter = tier.get("adapter")
        if not isinstance(adapter, Mapping):
            failures.append("manifest:tier_adapter")
            continue
        adapter_text = json.dumps(adapter, ensure_ascii=False).lower()
        if any(token in adapter_text for token in ("truth", "px4", "rtk", "ned")):
            runtime_truth_used = True
    require(not runtime_truth_used, failures, "manifest:runtime_truth")
    return {
        "status": "PASS" if not failures else "FAIL",
        "manifest": str(path),
        "default_recommendation": payload.get("default_recommendation", ""),
        "tier_count": len(tiers) if isinstance(tiers, list) else 0,
        "runtime_truth_used": runtime_truth_used,
        "failure_count": len(failures),
        "failures": failures,
    }


def check_operator_view_launch(
    *,
    package_dir: Path | None = None,
    profile_json: Path | None = None,
    selector_json: Path | None = None,
    deployment_label: str = "",
    scenario_label: str = "",
    scenario_name: str = "",
    observation_fusion_manifest: Path | None = None,
    observation_fusion_tier: str = "",
    disparity_residual_profile_json: Path | None = None,
    disparity_residual_decision_json: Path | None = None,
    disparity_residual_selector_json: Path | None = None,
    disparity_residual_feature_selector_json: Path | None = None,
    disparity_residual_feature_catalog_json: Path | None = None,
    disparity_residual_feature_json: Path | None = None,
    disparity_residual_feature_csv: Path | None = None,
    disparity_residual_feature_row_index: int = 0,
    node_disparity_residual_feature_selector_json: Path | None = None,
    node_disparity_residual_feature_catalog_json: Path | None = None,
    node_disparity_residual_feature_window_size: int = 0,
    node_disparity_residual_feature_min_samples: int = 0,
) -> dict[str, object]:
    failures: list[str] = []
    package_check: dict[str, object] = {}
    observation_fusion_check: dict[str, object] = {}
    label_selection: dict[str, object] = {}
    residual_label_selection: dict[str, object] = {}
    residual_feature_selection: dict[str, object] = {}
    residual_decision_payload: dict[str, object] = {}
    residual_profile_payload: dict[str, object] = {}
    node_residual_feature_selector_payload: dict[str, object] = {}
    node_residual_feature_catalog_payload: dict[str, object] = {}
    selected_label = ""
    residual_selected_label = ""
    residual_decision_selected_label = ""
    residual_feature_selected_label = ""
    source_mode = "package" if package_dir is not None else "profile"

    if selector_json is not None:
        source_mode = "selector"
        label_selection = label_selector.select_runtime_profile_by_label(
            selector_json=selector_json,
            deployment_label=deployment_label,
            scenario_label=scenario_label,
            scenario_name=scenario_name,
        )
        require(label_selection.get("status") == "PASS", failures, "selector:status")
        selected_label = str(label_selection.get("selected_label", ""))
        selected_package = str(label_selection.get("selected_package_dir", ""))
        selected_profile = str(label_selection.get("selected_profile", ""))
        if selected_package:
            package_dir = Path(selected_package)
        elif selected_profile:
            profile_json = Path(selected_profile)

    if package_dir is not None:
        package_dir = package_dir.resolve()
        require(package_dir.is_dir(), failures, "package_dir:missing")
        if package_dir.is_dir():
            package_type, package_check = check_supported_package(package_dir)
            require(package_check.get("status") == "PASS", failures, "package:check")
        else:
            package_type = "missing"
        profile_path = resolve_profile_from_package(package_dir)
        if observation_fusion_manifest is None:
            package_manifest = package_dir / "observation_fusion_runtime_manifest.json"
            if package_manifest.is_file():
                observation_fusion_manifest = package_manifest
    else:
        package_type = ""
        assert profile_json is not None
        profile_path = profile_json.resolve()

    if disparity_residual_selector_json is not None:
        residual_label_selection = residual_label_selector.select_residual_profile_by_label(
            selector_json=disparity_residual_selector_json,
            deployment_label=deployment_label,
            scenario_label=scenario_label,
            scenario_name=scenario_name,
        )
        require(residual_label_selection.get("status") == "PASS", failures, "residual_selector:status")
        residual_selected_label = str(residual_label_selection.get("selected_label", ""))
        selected_residual_profile = str(residual_label_selection.get("selected_profile", ""))
        if selected_residual_profile:
            disparity_residual_profile_json = Path(selected_residual_profile)

    if disparity_residual_decision_json is not None:
        require(
            disparity_residual_profile_json is None
            and disparity_residual_selector_json is None
            and disparity_residual_feature_selector_json is None,
            failures,
            "residual_decision:exclusive",
        )
        if disparity_residual_decision_json.is_file():
            residual_decision_payload = load_json(disparity_residual_decision_json)
            require(residual_decision_payload.get("status") == "PASS", failures, "residual_decision:status")
            require(
                residual_decision_payload.get("apply_residual_allowed") is True,
                failures,
                "residual_decision:apply_not_allowed",
            )
            require(
                residual_decision_payload.get("runtime_truth_used") is False,
                failures,
                "residual_decision:runtime_truth_used",
            )
            residual_decision_selected_label = str(residual_decision_payload.get("recommended_residual_label", ""))
            selected_residual_profile = str(residual_decision_payload.get("recommended_residual_profile", ""))
            require(bool(residual_decision_selected_label), failures, "residual_decision:label_missing")
            require(bool(selected_residual_profile), failures, "residual_decision:profile_missing")
            if selected_residual_profile:
                disparity_residual_profile_json = Path(selected_residual_profile)
        else:
            failures.append("residual_decision:missing")

    if disparity_residual_feature_selector_json is not None:
        require(disparity_residual_feature_catalog_json is not None, failures, "residual_feature_selector:catalog_missing")
        require(
            (disparity_residual_feature_json is None) != (disparity_residual_feature_csv is None),
            failures,
            "residual_feature_selector:feature_source",
        )
        feature_row: dict[str, object] = {}
        if disparity_residual_feature_json is not None and disparity_residual_feature_json.is_file():
            feature_row = residual_feature_selector.load_json(disparity_residual_feature_json)
        elif disparity_residual_feature_csv is not None and disparity_residual_feature_csv.is_file():
            feature_row = residual_feature_selector.feature_row_from_csv(
                disparity_residual_feature_csv,
                int(disparity_residual_feature_row_index),
            )
        else:
            failures.append("residual_feature_selector:feature_missing")
        if disparity_residual_feature_catalog_json is not None and feature_row:
            residual_feature_selection = residual_feature_selector.select_residual_profile_by_features(
                selector_model_json=disparity_residual_feature_selector_json,
                catalog_json=disparity_residual_feature_catalog_json,
                feature_row=feature_row,
            )
            require(residual_feature_selection.get("status") == "PASS", failures, "residual_feature_selector:status")
            residual_feature_selected_label = str(residual_feature_selection.get("selected_label", ""))
            selected_residual_profile = str(residual_feature_selection.get("selected_profile", ""))
            if selected_residual_profile:
                disparity_residual_profile_json = Path(selected_residual_profile)

    if node_disparity_residual_feature_selector_json is not None:
        require(
            disparity_residual_profile_json is None
            and disparity_residual_selector_json is None
            and disparity_residual_feature_selector_json is None,
            failures,
            "node_residual_feature_selector:exclusive",
        )
        require(node_disparity_residual_feature_catalog_json is not None, failures, "node_residual_feature_selector:catalog_missing")
        if node_disparity_residual_feature_selector_json.is_file():
            node_residual_feature_selector_payload = load_json(node_disparity_residual_feature_selector_json)
            require(
                not has_runtime_truth(node_residual_feature_selector_payload.get("feature_names", [])),
                failures,
                "node_residual_feature_selector:runtime_truth_features",
            )
        else:
            failures.append("node_residual_feature_selector:model_missing")
        if node_disparity_residual_feature_catalog_json is not None and node_disparity_residual_feature_catalog_json.is_file():
            node_residual_feature_catalog_payload = load_json(node_disparity_residual_feature_catalog_json)
            require(
                isinstance(node_residual_feature_catalog_payload.get("profile_paths", {}), Mapping),
                failures,
                "node_residual_feature_selector:catalog_profile_paths",
            )
        elif node_disparity_residual_feature_catalog_json is not None:
            failures.append("node_residual_feature_selector:catalog_missing")

    require(profile_path.is_file(), failures, "profile:missing")
    profile_payload: dict[str, object] = {}
    profile_sha256 = ""
    if profile_path.is_file():
        profile_sha256 = sha256_file(profile_path)
        profile_payload = load_json(profile_path)
        require("RDF(right,down,forward)" in str(profile_payload.get("axis_contract", "")), failures, "profile:rdf_contract")
        require(not has_runtime_truth(profile_payload.get("runtime_inputs", [])), failures, "profile:runtime_inputs_truth")

    if observation_fusion_manifest is not None:
        observation_fusion_manifest = observation_fusion_manifest.resolve()
        observation_fusion_check = check_observation_fusion_manifest(observation_fusion_manifest)
        require(observation_fusion_check.get("status") == "PASS", failures, "observation_fusion:manifest")

    if disparity_residual_profile_json is not None:
        disparity_residual_profile_json = disparity_residual_profile_json.resolve()
        require(disparity_residual_profile_json.is_file(), failures, "disparity_residual_profile:missing")
        if disparity_residual_profile_json.is_file():
            residual_profile_payload = load_json(disparity_residual_profile_json)
            require(
                str(residual_profile_payload.get("profile_type", "")) in {
                    "DisparityResidualRuntimeProfile",
                    "disparity_residual_runtime_profile",
                },
                failures,
                "disparity_residual_profile:type",
            )
            require(
                residual_profile_payload.get("runtime_truth_used") is False,
                failures,
                "disparity_residual_profile:runtime_truth_used",
            )
            require(
                not has_runtime_truth(residual_profile_payload.get("runtime_inputs", [])),
                failures,
                "disparity_residual_profile:runtime_inputs_truth",
            )

    result = run_print_only(
        package_dir=None if selector_json is not None else package_dir,
        profile_json=None if (package_dir is not None or selector_json is not None) else profile_path,
        selector_json=selector_json.resolve() if selector_json is not None else None,
        deployment_label=deployment_label,
        scenario_label=scenario_label,
        scenario_name=scenario_name,
        observation_fusion_manifest=observation_fusion_manifest,
        observation_fusion_tier=observation_fusion_tier,
        disparity_residual_profile_json=None
        if (
            disparity_residual_decision_json is not None
            or disparity_residual_selector_json is not None
            or disparity_residual_feature_selector_json is not None
        )
        else disparity_residual_profile_json,
        disparity_residual_decision_json=None
        if disparity_residual_decision_json is None
        else disparity_residual_decision_json.resolve(),
        disparity_residual_selector_json=None
        if disparity_residual_selector_json is None
        else disparity_residual_selector_json.resolve(),
        disparity_residual_feature_selector_json=None
        if disparity_residual_feature_selector_json is None
        else disparity_residual_feature_selector_json.resolve(),
        disparity_residual_feature_catalog_json=None
        if disparity_residual_feature_catalog_json is None
        else disparity_residual_feature_catalog_json.resolve(),
        disparity_residual_feature_json=None
        if disparity_residual_feature_json is None
        else disparity_residual_feature_json.resolve(),
        disparity_residual_feature_csv=None
        if disparity_residual_feature_csv is None
        else disparity_residual_feature_csv.resolve(),
        disparity_residual_feature_row_index=disparity_residual_feature_row_index,
        node_disparity_residual_feature_selector_json=None
        if node_disparity_residual_feature_selector_json is None
        else node_disparity_residual_feature_selector_json.resolve(),
        node_disparity_residual_feature_catalog_json=None
        if node_disparity_residual_feature_catalog_json is None
        else node_disparity_residual_feature_catalog_json.resolve(),
        node_disparity_residual_feature_window_size=node_disparity_residual_feature_window_size,
        node_disparity_residual_feature_min_samples=node_disparity_residual_feature_min_samples,
    )
    launch_contract = launch_contract_checker.check_launch_contract()
    require(launch_contract.get("status") == "PASS", failures, "launch_contract:status")
    require(result.returncode == 0, failures, "operator_wrapper:returncode")
    resolved_profile = extract_runtime_profile(result.stdout)
    require(resolved_profile == str(profile_path), failures, "operator_wrapper:resolved_profile")
    require("--bbox-adjustment" in result.stdout, failures, "operator_wrapper:bbox_adjustment")
    require(str(profile_path) in result.stdout, failures, "operator_wrapper:profile_arg")
    resolved_observation_fusion_manifest = extract_observation_fusion_manifest(result.stdout)
    if observation_fusion_manifest is not None:
        require(
            resolved_observation_fusion_manifest == str(observation_fusion_manifest),
            failures,
            "operator_wrapper:observation_fusion_manifest",
        )
        require("--observation-fusion-manifest" in result.stdout, failures, "operator_wrapper:observation_fusion_arg")
        if observation_fusion_tier:
            require("--observation-fusion-tier" in result.stdout, failures, "operator_wrapper:observation_fusion_tier")
    resolved_disparity_residual_profile = extract_disparity_residual_profile(result.stdout)
    resolved_disparity_residual_decision = extract_disparity_residual_decision(result.stdout)
    resolved_disparity_residual_decision_label = extract_disparity_residual_decision_label(result.stdout)
    if disparity_residual_profile_json is not None:
        require(
            resolved_disparity_residual_profile == str(disparity_residual_profile_json),
            failures,
            "operator_wrapper:disparity_residual_profile",
        )
        require("--disparity-residual-profile" in result.stdout, failures, "operator_wrapper:disparity_residual_arg")
    if disparity_residual_decision_json is not None:
        require(
            resolved_disparity_residual_decision == str(disparity_residual_decision_json.resolve()),
            failures,
            "operator_wrapper:disparity_residual_decision",
        )
        require(
            resolved_disparity_residual_decision_label == residual_decision_selected_label,
            failures,
            "operator_wrapper:disparity_residual_decision_label",
        )
    resolved_node_residual_feature_selector = extract_node_disparity_residual_feature_selector(result.stdout)
    resolved_node_residual_feature_catalog = extract_node_disparity_residual_feature_catalog(result.stdout)
    if node_disparity_residual_feature_selector_json is not None:
        require(
            resolved_node_residual_feature_selector == str(node_disparity_residual_feature_selector_json.resolve()),
            failures,
            "operator_wrapper:node_residual_feature_selector",
        )
        require(
            resolved_node_residual_feature_catalog == str(node_disparity_residual_feature_catalog_json.resolve())
            if node_disparity_residual_feature_catalog_json is not None
            else False,
            failures,
            "operator_wrapper:node_residual_feature_catalog",
        )
        require("--disparity-residual-feature-selector" in result.stdout, failures, "operator_wrapper:node_residual_feature_selector_arg")
        require("--disparity-residual-feature-catalog" in result.stdout, failures, "operator_wrapper:node_residual_feature_catalog_arg")
        if node_disparity_residual_feature_window_size > 0:
            require(
                f"--disparity-residual-feature-window-size  {node_disparity_residual_feature_window_size}" in result.stdout,
                failures,
                "operator_wrapper:node_residual_feature_window_size_arg",
            )
        if node_disparity_residual_feature_min_samples > 0:
            require(
                f"--disparity-residual-feature-min-samples  {node_disparity_residual_feature_min_samples}" in result.stdout,
                failures,
                "operator_wrapper:node_residual_feature_min_samples_arg",
            )
    require("--no-px4" in result.stdout, failures, "operator_wrapper:dry_run_arg")
    require("yolo_target_hz:=0.0" in result.stdout, failures, "operator_wrapper:yolo_disabled_arg")

    passed = not failures
    return {
        "mode": "stereodistance_operator_view_launch_check",
        "status": "PASS" if passed else "FAIL",
        "source_mode": source_mode,
        "selector_json": "" if selector_json is None else str(selector_json.resolve()),
        "deployment_label": deployment_label,
        "scenario_label": scenario_label,
        "scenario_name": scenario_name,
        "selected_label": selected_label,
        "residual_selected_label": residual_selected_label,
        "residual_decision_selected_label": residual_decision_selected_label,
        "residual_feature_selected_label": residual_feature_selected_label,
        "package_dir": "" if package_dir is None else str(package_dir),
        "package_type": package_type,
        "profile_json": str(profile_path),
        "resolved_runtime_profile_sha256": profile_sha256,
        "profile_id": profile_payload.get("profile_id", ""),
        "method": profile_payload.get("method", ""),
        "profile_mode": profile_payload.get("mode", ""),
        "operator_wrapper": str(ROOT_DIR / "run_operator_view_with_calibration_profile.sh"),
        "operator_command_stdout": result.stdout,
        "operator_command_stderr": result.stderr,
        "operator_command_returncode": result.returncode,
        "observation_fusion_manifest": "" if observation_fusion_manifest is None else str(observation_fusion_manifest),
        "observation_fusion_manifest_status": observation_fusion_check.get("status", "") if observation_fusion_check else "",
        "observation_fusion_tier": observation_fusion_tier,
        "resolved_observation_fusion_manifest": resolved_observation_fusion_manifest,
        "disparity_residual_profile_json": "" if disparity_residual_profile_json is None else str(disparity_residual_profile_json),
        "disparity_residual_decision_json": "" if disparity_residual_decision_json is None else str(disparity_residual_decision_json.resolve()),
        "disparity_residual_decision_status": residual_decision_payload.get("status", "") if residual_decision_payload else "",
        "disparity_residual_decision_apply_allowed": residual_decision_payload.get("apply_residual_allowed", False)
        if residual_decision_payload
        else False,
        "resolved_disparity_residual_decision": resolved_disparity_residual_decision,
        "resolved_disparity_residual_decision_label": resolved_disparity_residual_decision_label,
        "resolved_disparity_residual_profile": resolved_disparity_residual_profile,
        "node_disparity_residual_feature_selector_json": ""
        if node_disparity_residual_feature_selector_json is None
        else str(node_disparity_residual_feature_selector_json.resolve()),
        "node_disparity_residual_feature_catalog_json": ""
        if node_disparity_residual_feature_catalog_json is None
        else str(node_disparity_residual_feature_catalog_json.resolve()),
        "resolved_node_disparity_residual_feature_selector": resolved_node_residual_feature_selector,
        "resolved_node_disparity_residual_feature_catalog": resolved_node_residual_feature_catalog,
        "node_disparity_residual_feature_count": len(node_residual_feature_selector_payload.get("feature_names", []))
        if node_residual_feature_selector_payload
        else 0,
        "node_disparity_residual_catalog_profile_count": len(node_residual_feature_catalog_payload.get("profile_paths", {}))
        if node_residual_feature_catalog_payload
        else 0,
        "node_disparity_residual_feature_window_size": node_disparity_residual_feature_window_size,
        "node_disparity_residual_feature_min_samples": node_disparity_residual_feature_min_samples,
        "disparity_residual_profile_id": residual_profile_payload.get("profile_id", ""),
        "disparity_residual_method": residual_profile_payload.get("method", ""),
        "launch_contract_status": launch_contract.get("status", ""),
        "launch_file": launch_contract.get("launch_file", ""),
        "resolved_runtime_profile": resolved_profile,
        "bbox_adjustment_arg_present": "--bbox-adjustment" in result.stdout,
        "observation_fusion_arg_present": "--observation-fusion-manifest" in result.stdout,
        "disparity_residual_arg_present": "--disparity-residual-profile" in result.stdout,
        "node_disparity_residual_feature_selector_arg_present": "--disparity-residual-feature-selector" in result.stdout,
        "node_disparity_residual_feature_catalog_arg_present": "--disparity-residual-feature-catalog" in result.stdout,
        "node_disparity_residual_feature_window_size_arg_present": "--disparity-residual-feature-window-size"
        in result.stdout,
        "node_disparity_residual_feature_min_samples_arg_present": "--disparity-residual-feature-min-samples"
        in result.stdout,
        "package_check_status": package_check.get("status", "") if package_check else "",
        "label_selection_status": label_selection.get("status", "") if label_selection else "",
        "residual_label_selection_status": residual_label_selection.get("status", "") if residual_label_selection else "",
        "residual_decision_status": residual_decision_payload.get("status", "") if residual_decision_payload else "",
        "residual_feature_selection_status": residual_feature_selection.get("status", "")
        if residual_feature_selection
        else "",
        "runtime_truth_used": False,
        "coordinate_contract": (
            "operator wrapper passes the validated active profile to StereoDistance; runtime consumes "
            "stereo-center RDF(right,down,forward) plus measured disparity; optional observation fusion "
            "consumes image left_u/left_v/disparity only; optional residual profile consumes measured "
            "disparity plus corrected RDF; optional residual feature selector consumes only image/disparity/RDF "
            "summary features and resolves the residual profile path; optional residual decision artifact gates "
            "a preselected residual profile without runtime truth; public output remains FRD+range"
        ),
        "failure_count": len(failures),
        "failures": failures,
    }


def write_text(path: Path, payload: Mapping[str, object]) -> None:
    lines = [
        "StereoDistance operator-view launch check",
        f"status: {payload.get('status', '')}",
        f"source_mode: {payload.get('source_mode', '')}",
        f"selector_json: {payload.get('selector_json', '')}",
        f"deployment_label: {payload.get('deployment_label', '')}",
        f"scenario_label: {payload.get('scenario_label', '')}",
        f"scenario_name: {payload.get('scenario_name', '')}",
        f"selected_label: {payload.get('selected_label', '')}",
        f"residual_selected_label: {payload.get('residual_selected_label', '')}",
        f"residual_decision_selected_label: {payload.get('residual_decision_selected_label', '')}",
        f"residual_feature_selected_label: {payload.get('residual_feature_selected_label', '')}",
        f"package_dir: {payload.get('package_dir', '')}",
        f"package_type: {payload.get('package_type', '')}",
        f"profile_json: {payload.get('profile_json', '')}",
        f"resolved_runtime_profile_sha256: {payload.get('resolved_runtime_profile_sha256', '')}",
        f"profile_id: {payload.get('profile_id', '')}",
        f"method: {payload.get('method', '')}",
        f"profile_mode: {payload.get('profile_mode', '')}",
        f"package_check_status: {payload.get('package_check_status', '')}",
        f"operator_command_returncode: {payload.get('operator_command_returncode', '')}",
        f"observation_fusion_manifest: {payload.get('observation_fusion_manifest', '')}",
        f"observation_fusion_manifest_status: {payload.get('observation_fusion_manifest_status', '')}",
        f"observation_fusion_tier: {payload.get('observation_fusion_tier', '')}",
        f"resolved_observation_fusion_manifest: {payload.get('resolved_observation_fusion_manifest', '')}",
        f"disparity_residual_profile_json: {payload.get('disparity_residual_profile_json', '')}",
        f"disparity_residual_decision_json: {payload.get('disparity_residual_decision_json', '')}",
        f"disparity_residual_decision_status: {payload.get('disparity_residual_decision_status', '')}",
        f"disparity_residual_decision_apply_allowed: {payload.get('disparity_residual_decision_apply_allowed', False)}",
        f"resolved_disparity_residual_decision: {payload.get('resolved_disparity_residual_decision', '')}",
        f"resolved_disparity_residual_decision_label: {payload.get('resolved_disparity_residual_decision_label', '')}",
        f"resolved_disparity_residual_profile: {payload.get('resolved_disparity_residual_profile', '')}",
        f"node_disparity_residual_feature_selector_json: {payload.get('node_disparity_residual_feature_selector_json', '')}",
        f"node_disparity_residual_feature_catalog_json: {payload.get('node_disparity_residual_feature_catalog_json', '')}",
        f"resolved_node_disparity_residual_feature_selector: {payload.get('resolved_node_disparity_residual_feature_selector', '')}",
        f"resolved_node_disparity_residual_feature_catalog: {payload.get('resolved_node_disparity_residual_feature_catalog', '')}",
        f"node_disparity_residual_feature_count: {payload.get('node_disparity_residual_feature_count', 0)}",
        f"node_disparity_residual_catalog_profile_count: {payload.get('node_disparity_residual_catalog_profile_count', 0)}",
        f"node_disparity_residual_feature_window_size: {payload.get('node_disparity_residual_feature_window_size', 0)}",
        f"node_disparity_residual_feature_min_samples: {payload.get('node_disparity_residual_feature_min_samples', 0)}",
        f"disparity_residual_profile_id: {payload.get('disparity_residual_profile_id', '')}",
        f"disparity_residual_method: {payload.get('disparity_residual_method', '')}",
        f"launch_contract_status: {payload.get('launch_contract_status', '')}",
        f"label_selection_status: {payload.get('label_selection_status', '')}",
        f"residual_label_selection_status: {payload.get('residual_label_selection_status', '')}",
        f"residual_decision_status: {payload.get('residual_decision_status', '')}",
        f"residual_feature_selection_status: {payload.get('residual_feature_selection_status', '')}",
        f"launch_file: {payload.get('launch_file', '')}",
        f"resolved_runtime_profile: {payload.get('resolved_runtime_profile', '')}",
        f"bbox_adjustment_arg_present: {payload.get('bbox_adjustment_arg_present', False)}",
        f"observation_fusion_arg_present: {payload.get('observation_fusion_arg_present', False)}",
        f"disparity_residual_arg_present: {payload.get('disparity_residual_arg_present', False)}",
        f"node_disparity_residual_feature_selector_arg_present: {payload.get('node_disparity_residual_feature_selector_arg_present', False)}",
        f"node_disparity_residual_feature_catalog_arg_present: {payload.get('node_disparity_residual_feature_catalog_arg_present', False)}",
        f"node_disparity_residual_feature_window_size_arg_present: {payload.get('node_disparity_residual_feature_window_size_arg_present', False)}",
        f"node_disparity_residual_feature_min_samples_arg_present: {payload.get('node_disparity_residual_feature_min_samples_arg_present', False)}",
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
    payload = check_operator_view_launch(
        package_dir=args.package_dir,
        profile_json=args.profile_json,
        selector_json=args.selector_json,
        deployment_label=args.deployment_label,
        scenario_label=args.scenario_label,
        scenario_name=args.scenario_name,
        observation_fusion_manifest=args.observation_fusion_manifest,
        observation_fusion_tier=args.observation_fusion_tier,
        disparity_residual_profile_json=args.disparity_residual_profile_json,
        disparity_residual_decision_json=args.disparity_residual_decision_json,
        disparity_residual_selector_json=args.disparity_residual_selector_json,
        disparity_residual_feature_selector_json=args.disparity_residual_feature_selector_json,
        disparity_residual_feature_catalog_json=args.disparity_residual_feature_catalog_json,
        disparity_residual_feature_json=args.disparity_residual_feature_json,
        disparity_residual_feature_csv=args.disparity_residual_feature_csv,
        disparity_residual_feature_row_index=args.disparity_residual_feature_row_index,
        node_disparity_residual_feature_selector_json=args.node_disparity_residual_feature_selector_json,
        node_disparity_residual_feature_catalog_json=args.node_disparity_residual_feature_catalog_json,
        node_disparity_residual_feature_window_size=args.node_disparity_residual_feature_window_size,
        node_disparity_residual_feature_min_samples=args.node_disparity_residual_feature_min_samples,
    )
    if args.output_json is not None:
        write_json(args.output_json, payload)
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
