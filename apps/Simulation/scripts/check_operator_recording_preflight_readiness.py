#!/usr/bin/env python3
from __future__ import annotations

import argparse
import importlib.util
import json
from pathlib import Path
from typing import Mapping, Sequence


PUBLIC_AXIS_CONTRACT = "stereodistance_public_frd:forward,right,down,range"
REQUIRED_ESTIMATED_FIELDS = (
    "nominal_pose_forward_m",
    "nominal_pose_right_m",
    "nominal_pose_down_m",
    "nominal_pose_range_m",
)
REQUIRED_CORRECTED_FIELDS = (
    "corrected_forward_m",
    "corrected_right_m",
    "corrected_down_m",
    "corrected_range_m",
)
REQUIRED_DETECTION_FIELDS = ("left_cx", "left_cy", "raw_disparity_px")
REQUIRED_TOPICS = (
    "/stereo/nominal_pose_plot",
    "/stereo/corrected_position",
    "/stereo/detections",
    "/stereo/disparity",
    "/stereo/profile_status",
)


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Check that run_operator_view.sh --record-lite will use a recorder schema compatible with "
            "truth-free residual preflight."
        )
    )
    parser.add_argument("--root-dir", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--output-json", type=Path)
    parser.add_argument("--output-txt", type=Path)
    return parser.parse_args(argv)


def write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def require(condition: bool, failures: list[str], reason: str) -> None:
    if not condition:
        failures.append(reason)


def load_operator_recorder(path: Path):
    spec = importlib.util.spec_from_file_location("operator_recorder_readiness_under_test", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def check_readiness(root_dir: Path) -> dict[str, object]:
    root_dir = root_dir.resolve()
    run_script = root_dir / "run_operator_view.sh"
    recorder_py = root_dir / "stereo_ground_sim" / "stereo_ground_sim" / "operator_recorder.py"
    setup_py = root_dir / "stereo_ground_sim" / "setup.py"
    failures: list[str] = []

    require(run_script.is_file(), failures, "run_operator_view:missing")
    require(recorder_py.is_file(), failures, "operator_recorder:missing")
    require(setup_py.is_file(), failures, "setup_py:missing")

    run_script_text = run_script.read_text(encoding="utf-8") if run_script.is_file() else ""
    setup_text = setup_py.read_text(encoding="utf-8") if setup_py.is_file() else ""

    require("--record-lite" in run_script_text, failures, "run_operator_view:record_lite_option_missing")
    require("--record-full" in run_script_text, failures, "run_operator_view:record_full_option_missing")
    require("--record-debug" in run_script_text, failures, "run_operator_view:record_debug_option_missing")
    require("--headless" in run_script_text, failures, "run_operator_view:headless_option_missing")
    require("stereo_ground_sim.operator_recorder" in run_script_text, failures, "run_operator_view:recorder_module_missing")
    require("OPERATOR_RECORDER_PYTHON" in run_script_text, failures, "run_operator_view:recorder_python_missing")
    require("operator_recorder = stereo_ground_sim.operator_recorder:main" in setup_text, failures, "setup:entrypoint_missing")

    recorder_payload: dict[str, object] = {}
    if recorder_py.is_file():
        try:
            recorder = load_operator_recorder(recorder_py)
            estimated_fields = tuple(getattr(recorder, "ESTIMATED_POSITION_FIELDS", ()))
            corrected_fields = tuple(getattr(recorder, "CORRECTED_POSITION_FIELDS", ()))
            detection_fields = tuple(getattr(recorder, "DETECTION_FIELDS", ()))
            numeric_topics = dict(getattr(recorder, "NUMERIC_TOPICS", {}))
            axis_contract = str(getattr(recorder, "AXIS_CONTRACT", ""))
            require(axis_contract == PUBLIC_AXIS_CONTRACT, failures, "operator_recorder:axis_contract")
            for field in REQUIRED_ESTIMATED_FIELDS:
                require(field in estimated_fields, failures, f"operator_recorder:estimated_field:{field}")
            for field in REQUIRED_CORRECTED_FIELDS:
                require(field in corrected_fields, failures, f"operator_recorder:corrected_field:{field}")
            for field in REQUIRED_DETECTION_FIELDS:
                require(field in detection_fields, failures, f"operator_recorder:detection_field:{field}")
            for topic in REQUIRED_TOPICS:
                if topic == "/stereo/nominal_pose_plot":
                    require("_estimated_position_cb" in run_script_text or hasattr(recorder.OperatorRecorder, "_estimated_position_cb"), failures, "operator_recorder:estimated_topic_cb")
                elif topic == "/stereo/corrected_position":
                    require(hasattr(recorder.OperatorRecorder, "_corrected_position_cb"), failures, "operator_recorder:corrected_topic_cb")
                elif topic == "/stereo/detections":
                    require(hasattr(recorder.OperatorRecorder, "_detections_cb"), failures, "operator_recorder:detections_topic_cb")
                elif topic == "/stereo/profile_status":
                    require(hasattr(recorder.OperatorRecorder, "_profile_status_cb"), failures, "operator_recorder:profile_status_topic_cb")
                elif topic == "/stereo/disparity":
                    require(topic in numeric_topics, failures, "operator_recorder:disparity_topic")
            require(hasattr(recorder, "json"), failures, "operator_recorder:json_import")
            ambiguous_fields = [
                field
                for field in estimated_fields + corrected_fields
                if field in {"x", "y", "z"} or field.endswith("_x_m") or field.endswith("_y_m") or field.endswith("_z_m")
            ]
            require(not ambiguous_fields, failures, "operator_recorder:ambiguous_xyz_fields")
            recorder_payload = {
                "axis_contract": axis_contract,
                "estimated_position_fields": list(estimated_fields),
                "corrected_position_fields": list(corrected_fields),
                "detection_fields": list(detection_fields),
                "numeric_topics": numeric_topics,
                "ambiguous_fields": ambiguous_fields,
            }
        except Exception as exc:  # pragma: no cover - defensive artifact detail
            failures.append(f"operator_recorder:load_failed:{type(exc).__name__}")
            recorder_payload = {"load_error": str(exc)}

    command_template = (
        "cd Simulation && ./run_operator_view.sh --record-lite bags/operator_$(date +%Y%m%d_%H%M%S)_lite "
        "--bbox-adjustment <active_runtime_profile.json>"
    )
    return {
        "mode": "operator_recording_preflight_readiness",
        "status": "PASS" if not failures else "FAIL",
        "root_dir": str(root_dir),
        "run_operator_view": str(run_script),
        "operator_recorder": str(recorder_py),
        "setup_py": str(setup_py),
        "record_lite_supported": "--record-lite" in run_script_text,
        "headless_supported": "--headless" in run_script_text,
        "operator_recorder_entrypoint_declared": "operator_recorder = stereo_ground_sim.operator_recorder:main" in setup_text,
        "runtime_truth_used": False,
        "recommended_record_command": command_template,
        "coordinate_contract": (
            "operator recorder writes public FRD+range telemetry fields; residual preflight later converts "
            "raw/corrected public FRD fields plus image/disparity measurements into explicit stereo-center "
            "RDF selector observations; PX4/RTK/NED truth is not a runtime selector input"
        ),
        "failure_count": len(failures),
        "failures": failures,
        "recorder_schema": recorder_payload,
    }


def write_text(path: Path, payload: Mapping[str, object]) -> None:
    lines = [
        "Operator recording preflight readiness",
        f"status: {payload.get('status', '')}",
        f"root_dir: {payload.get('root_dir', '')}",
        f"run_operator_view: {payload.get('run_operator_view', '')}",
        f"operator_recorder: {payload.get('operator_recorder', '')}",
        f"record_lite_supported: {payload.get('record_lite_supported', False)}",
        f"headless_supported: {payload.get('headless_supported', False)}",
        f"operator_recorder_entrypoint_declared: {payload.get('operator_recorder_entrypoint_declared', False)}",
        f"recommended_record_command: {payload.get('recommended_record_command', '')}",
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
    payload = check_readiness(args.root_dir)
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
