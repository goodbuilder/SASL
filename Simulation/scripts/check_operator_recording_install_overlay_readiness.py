#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import shutil
import subprocess
from pathlib import Path
from typing import Mapping, Sequence


PUBLIC_AXIS_CONTRACT = "stereodistance_public_frd:forward,right,down,range"
REQUIRED_FIELDS = (
    "nominal_pose_forward_m",
    "nominal_pose_right_m",
    "nominal_pose_down_m",
    "corrected_forward_m",
    "corrected_right_m",
    "corrected_down_m",
    "left_cx",
    "left_cy",
    "raw_disparity_px",
)


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Check that the installed ROS overlay used by run_operator_view.sh exposes the current "
            "operator_recorder public FRD+range schema."
        )
    )
    parser.add_argument("--root-dir", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--output-json", type=Path)
    parser.add_argument("--output-txt", type=Path)
    return parser.parse_args(argv)


def write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def write_text(path: Path, payload: Mapping[str, object]) -> None:
    lines = [
        "Operator recording install overlay readiness",
        f"status: {payload.get('status', '')}",
        f"root_dir: {payload.get('root_dir', '')}",
        f"ros_setup: {payload.get('ros_setup', '')}",
        f"install_setup: {payload.get('install_setup', '')}",
        f"ros2_path: {payload.get('ros2_path', '')}",
        f"package_prefix: {payload.get('package_prefix', '')}",
        f"installed_axis_contract: {payload.get('installed_axis_contract', '')}",
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


def run_overlay_probe(*, root_dir: Path) -> subprocess.CompletedProcess[str]:
    script = r"""
set -euo pipefail
set +u
source /opt/ros/humble/setup.bash
source "$ROOT_DIR/install/setup.bash"
set -u
python3 - <<'PY'
import json
from pathlib import Path
import stereo_ground_sim.operator_recorder as recorder
payload = {
    "axis_contract": getattr(recorder, "AXIS_CONTRACT", ""),
    "estimated_position_fields": list(getattr(recorder, "ESTIMATED_POSITION_FIELDS", ())),
    "corrected_position_fields": list(getattr(recorder, "CORRECTED_POSITION_FIELDS", ())),
    "detection_fields": list(getattr(recorder, "DETECTION_FIELDS", ())),
    "module_file": str(Path(getattr(recorder, "__file__", "")).resolve()),
}
print(json.dumps(payload, ensure_ascii=False, sort_keys=True))
PY
ros2 pkg prefix stereo_ground_sim
"""
    return subprocess.run(
        ["bash", "-lc", script],
        cwd=root_dir,
        env={"ROOT_DIR": str(root_dir), **dict(__import__("os").environ)},
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )


def check_readiness(root_dir: Path) -> dict[str, object]:
    root_dir = root_dir.resolve()
    ros_setup = Path("/opt/ros/humble/setup.bash")
    install_setup = root_dir / "install" / "setup.bash"
    failures: list[str] = []
    ros2_path = shutil.which("ros2") or ""

    if not ros_setup.is_file():
        failures.append("ros_setup:missing")
    if not install_setup.is_file():
        failures.append("install_setup:missing")
    if not ros2_path:
        failures.append("ros2:not_found")

    installed_payload: dict[str, object] = {}
    package_prefix = ""
    probe_returncode: int | None = None
    probe_stderr = ""
    if not failures:
        probe = run_overlay_probe(root_dir=root_dir)
        probe_returncode = probe.returncode
        probe_stderr = probe.stderr[-4000:]
        if probe.returncode != 0:
            failures.append(f"overlay_probe:returncode:{probe.returncode}")
        else:
            lines = [line for line in probe.stdout.splitlines() if line.strip()]
            if len(lines) < 2:
                failures.append("overlay_probe:unexpected_stdout")
            else:
                try:
                    installed_payload = json.loads(lines[0])
                except json.JSONDecodeError:
                    failures.append("overlay_probe:invalid_json")
                package_prefix = lines[-1]

    installed_axis_contract = str(installed_payload.get("axis_contract", ""))
    if installed_payload:
        if installed_axis_contract != PUBLIC_AXIS_CONTRACT:
            failures.append("installed_operator_recorder:axis_contract_stale")
        installed_fields = set(installed_payload.get("estimated_position_fields", []))
        installed_fields.update(installed_payload.get("corrected_position_fields", []))
        installed_fields.update(installed_payload.get("detection_fields", []))
        for field in REQUIRED_FIELDS:
            if field not in installed_fields:
                failures.append(f"installed_operator_recorder:missing_field:{field}")

    return {
        "mode": "operator_recording_install_overlay_readiness",
        "status": "PASS" if not failures else "FAIL",
        "root_dir": str(root_dir),
        "ros_setup": str(ros_setup),
        "install_setup": str(install_setup),
        "ros2_path": ros2_path,
        "package_prefix": package_prefix,
        "probe_returncode": probe_returncode,
        "probe_stderr": probe_stderr,
        "installed_axis_contract": installed_axis_contract,
        "installed_operator_recorder": installed_payload,
        "runtime_truth_used": False,
        "coordinate_contract": (
            "installed operator_recorder must emit public FRD+range telemetry fields before live residual "
            "preflight converts them to stereo-center RDF selector observations; this check uses no PX4/RTK/NED truth"
        ),
        "failure_count": len(failures),
        "failures": failures,
    }


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
