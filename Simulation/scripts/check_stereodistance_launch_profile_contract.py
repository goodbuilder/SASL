#!/usr/bin/env python3
from __future__ import annotations

import argparse
import ast
import json
from pathlib import Path
from typing import Mapping, Sequence


ROOT_DIR = Path(__file__).resolve().parents[1]
DEFAULT_LAUNCH_FILE = ROOT_DIR / "stereo_ground_sim" / "launch" / "stereo_ground_sim.launch.py"


REQUIRED_PROFILE_ARGS = (
    "enable_bbox_center_adjustment",
    "bbox_center_adjustment_path",
    "enable_observation_fusion",
    "observation_fusion_manifest_path",
    "observation_fusion_tier",
    "enable_disparity_residual_profile",
    "disparity_residual_profile_path",
    "enable_disparity_residual_feature_selector",
    "disparity_residual_feature_selector_model_path",
    "disparity_residual_feature_selector_catalog_path",
    "disparity_residual_feature_selector_window_size",
    "disparity_residual_feature_selector_min_samples",
)


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Check StereoDistance launch profile parameter contract.")
    parser.add_argument("--launch-file", type=Path, default=DEFAULT_LAUNCH_FILE)
    parser.add_argument("--output-json", type=Path)
    parser.add_argument("--output-txt", type=Path)
    return parser.parse_args(argv)


def write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def constant_strings(node: ast.AST) -> set[str]:
    return {child.value for child in ast.walk(node) if isinstance(child, ast.Constant) and isinstance(child.value, str)}


def declared_launch_arguments(tree: ast.AST) -> set[str]:
    declared: set[str] = set()
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        func_name = getattr(node.func, "id", "") or getattr(node.func, "attr", "")
        if func_name != "DeclareLaunchArgument" or not node.args:
            continue
        arg = node.args[0]
        if isinstance(arg, ast.Constant) and isinstance(arg.value, str):
            declared.add(arg.value)
    return declared


def assigned_string_set(tree: ast.AST, variable_name: str) -> set[str]:
    values: set[str] = set()
    for node in ast.walk(tree):
        if not isinstance(node, ast.Assign):
            continue
        if not any(isinstance(target, ast.Name) and target.id == variable_name for target in node.targets):
            continue
        if isinstance(node.value, (ast.Set, ast.List, ast.Tuple)):
            for element in node.value.elts:
                if isinstance(element, ast.Constant) and isinstance(element.value, str):
                    values.add(element.value)
    return values


def check_launch_contract(launch_file: Path = DEFAULT_LAUNCH_FILE) -> dict[str, object]:
    failures: list[str] = []
    launch_path = launch_file.resolve()
    if not launch_path.is_file():
        return {
            "mode": "stereodistance_launch_profile_contract",
            "status": "FAIL",
            "launch_file": str(launch_path),
            "failure_count": 1,
            "failures": ["launch_file:missing"],
            "runtime_truth_used": False,
        }

    source = launch_path.read_text(encoding="utf-8")
    tree = ast.parse(source)
    declared = declared_launch_arguments(tree)
    bool_params = assigned_string_set(tree, "bool_params")
    launch_param_names = assigned_string_set(tree, "names")
    strings = constant_strings(tree)

    for name in REQUIRED_PROFILE_ARGS:
        if name not in declared:
            failures.append(f"launch_argument:missing:{name}")
        if name not in launch_param_names:
            failures.append(f"distance_node_param:missing:{name}")
    if "enable_bbox_center_adjustment" not in bool_params:
        failures.append("bool_param:missing:enable_bbox_center_adjustment")
    if "enable_observation_fusion" not in bool_params:
        failures.append("bool_param:missing:enable_observation_fusion")
    if "enable_disparity_residual_profile" not in bool_params:
        failures.append("bool_param:missing:enable_disparity_residual_profile")
    if "enable_disparity_residual_feature_selector" not in bool_params:
        failures.append("bool_param:missing:enable_disparity_residual_feature_selector")
    has_distance_node_namespace = any(value.startswith("/stereo_distance_node:") for value in strings)
    if not has_distance_node_namespace:
        failures.append("params_yaml:node_namespace")
    if "--params-file" not in strings:
        failures.append("distance_node:params_file_arg")
    if "stereo_ground_sim.stereo_distance_node" not in strings:
        failures.append("distance_node:module")

    return {
        "mode": "stereodistance_launch_profile_contract",
        "status": "PASS" if not failures else "FAIL",
        "launch_file": str(launch_path),
        "declared_profile_arguments": [name for name in REQUIRED_PROFILE_ARGS if name in declared],
        "distance_node_profile_parameters": [name for name in REQUIRED_PROFILE_ARGS if name in launch_param_names],
        "enable_bbox_center_adjustment_is_bool": "enable_bbox_center_adjustment" in bool_params,
        "enable_observation_fusion_is_bool": "enable_observation_fusion" in bool_params,
        "enable_disparity_residual_profile_is_bool": "enable_disparity_residual_profile" in bool_params,
        "enable_disparity_residual_feature_selector_is_bool": "enable_disparity_residual_feature_selector"
        in bool_params,
        "distance_node_params_yaml_namespace_present": has_distance_node_namespace,
        "distance_node_params_file_arg_present": "--params-file" in strings,
        "distance_node_module": "stereo_ground_sim.stereo_distance_node",
        "runtime_truth_used": False,
        "coordinate_contract": (
            "launch contract passes a validated runtime profile path into StereoDistance; "
            "optional observation-fusion manifest configures runtime image-observation fusion; "
            "optional residual disparity profile applies after corrected stereo-center RDF; "
            "no PX4/NED, RDF, or FRD coordinate values are transformed here"
        ),
        "failure_count": len(failures),
        "failures": failures,
    }


def write_text(path: Path, payload: Mapping[str, object]) -> None:
    lines = [
        "StereoDistance launch profile contract",
        f"status: {payload.get('status', '')}",
        f"launch_file: {payload.get('launch_file', '')}",
        f"declared_profile_arguments: {','.join(str(v) for v in payload.get('declared_profile_arguments', []))}",
        f"distance_node_profile_parameters: {','.join(str(v) for v in payload.get('distance_node_profile_parameters', []))}",
        f"enable_bbox_center_adjustment_is_bool: {payload.get('enable_bbox_center_adjustment_is_bool', False)}",
        f"enable_observation_fusion_is_bool: {payload.get('enable_observation_fusion_is_bool', False)}",
        f"enable_disparity_residual_profile_is_bool: {payload.get('enable_disparity_residual_profile_is_bool', False)}",
        "enable_disparity_residual_feature_selector_is_bool: "
        f"{payload.get('enable_disparity_residual_feature_selector_is_bool', False)}",
        f"distance_node_params_yaml_namespace_present: {payload.get('distance_node_params_yaml_namespace_present', False)}",
        f"distance_node_params_file_arg_present: {payload.get('distance_node_params_file_arg_present', False)}",
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
    payload = check_launch_contract(args.launch_file)
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
