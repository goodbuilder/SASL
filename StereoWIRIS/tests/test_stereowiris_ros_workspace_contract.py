from __future__ import annotations

import os
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
RUNNER = ROOT / "run.sh"


def test_help_is_dependency_free() -> None:
    env = os.environ.copy()
    env["SASL_ROS_WORKSPACE"] = "/definitely/missing/sasl-workspace"
    result = subprocess.run(
        [str(RUNNER), "--help"],
        cwd=ROOT,
        env=env,
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )

    assert result.returncode == 0, result.stderr
    assert "StereoWIRIS GUI application" in result.stdout


def test_runner_uses_only_canonical_stamped_overlay() -> None:
    source = RUNNER.read_text(encoding="utf-8")

    assert 'SASL_ROS_WORKSPACE="${SASL_ROS_WORKSPACE:-$SASL_ROOT/workspace/main}"' in source
    assert "86d8239e962f6939e05c3737784f60c02fa884db" in source
    assert "PX4_MSGS_OVERLAY_MANIFEST" in source
    assert 'source "$PX4_MSGS_OVERLAY_SETUP"' in source
    assert "PX4_WORKSPACE_SETUP" not in source
    assert "px4_msgs_ws" not in source


def test_direct_python_entrypoint_uses_only_canonical_ros_workspace() -> None:
    source = (ROOT / "main.py").read_text(encoding="utf-8")

    assert 'SASL_ROOT / "workspace" / "main"' in source
    assert "px4_msgs_ws" not in source
    assert "Path.home()" not in source


def test_missing_source_fails_before_network_bootstrap() -> None:
    env = os.environ.copy()
    env["SASL_ROS_WORKSPACE"] = "/definitely/missing/sasl-workspace"
    result = subprocess.run(
        [str(RUNNER), "--check"],
        cwd=ROOT,
        env=env,
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )

    assert result.returncode == 2
    assert "canonical px4_msgs source is missing" in result.stderr
    source = RUNNER.read_text(encoding="utf-8")
    assert source.index("require_canonical_px4_msgs_overlay") < source.index("bootstrap_network_tools.sh")


def test_workstation_check_skips_ros_overlay_bootstrap() -> None:
    env = os.environ.copy()
    env["SASL_ROS_WORKSPACE"] = "/definitely/missing/sasl-workspace"
    result = subprocess.run(
        [str(RUNNER), "--check", "--ui", "workstation"],
        cwd=ROOT,
        env=env,
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )

    assert result.returncode == 0, result.stderr
    assert "Workstation runtime check ok" in result.stdout
    assert "canonical px4_msgs source is missing" not in result.stderr

    result_reordered = subprocess.run(
        [str(RUNNER), "--ui", "workstation", "--check"],
        cwd=ROOT,
        env=env,
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    assert result_reordered.returncode == 0, result_reordered.stderr


def test_runner_keeps_ros_bootstrap_conditional_to_legacy() -> None:
    source = RUNNER.read_text(encoding="utf-8")

    assert 'WORKSTATION_UI_REQUESTED=0' in source
    assert 'if [[ "$WORKSTATION_UI_REQUESTED" != "1" ]]; then' in source
    assert 'source /opt/ros/humble/setup.bash' in source
