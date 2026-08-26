from __future__ import annotations

import os
import shutil
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
RUNNER = ROOT / "run.sh"


def _fake_launcher_tree(tmp_path: Path) -> tuple[Path, Path]:
    repository = tmp_path / "repository"
    app = repository / "apps" / "StereoWIRIS"
    app.mkdir(parents=True)
    runner = app / "run.sh"
    shutil.copy2(RUNNER, runner)
    python = repository / ".venv" / "bin" / "python"
    python.parent.mkdir(parents=True)
    python.write_text(
        "#!/usr/bin/env bash\n"
        "if [[ \"$*\" == *\"sasl_common.operator_runtime\"* ]]; then echo 'common runtime ok'; exit 0; fi\n"
        "if [[ \"$*\" == *\"-m Workstation\"* ]]; then echo 'Workstation runtime check ok'; exit 0; fi\n"
        "if [[ \"$1\" == */scripts/check_runtime.py ]]; then echo 'StereoWIRIS runtime check ok'; exit 0; fi\n"
        "exit 0\n",
        encoding="utf-8",
    )
    python.chmod(0o755)
    return repository, runner


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


def test_runner_uses_only_repository_gui_environment() -> None:
    source = RUNNER.read_text(encoding="utf-8")

    project_python = '"$SASL_ROOT/.venv/bin/python"'
    assert project_python in source
    assert '"$APP_DIR/.venv/bin/python"' not in source
    assert "Simulation/.venv" not in source
    assert "python3 python" not in source
    assert "bootstrap_network_tools.sh" not in source
    assert "bootstrap_calibration_deps.sh" not in source


def test_direct_python_entrypoint_uses_only_canonical_ros_workspace() -> None:
    source = (ROOT / "main.py").read_text(encoding="utf-8")

    assert 'SASL_ROOT / "workspace" / "main"' in source
    assert "px4_msgs_ws" not in source
    assert "Path.home()" not in source


def test_missing_ros_source_fails_after_common_runtime_gate(tmp_path: Path) -> None:
    _repository, runner = _fake_launcher_tree(tmp_path)
    env = os.environ.copy()
    env["SASL_ROS_WORKSPACE"] = "/definitely/missing/sasl-workspace"
    env["SASL_MAVLINK_ONLY"] = "0"
    result = subprocess.run(
        [str(runner), "--check"],
        cwd=runner.parent,
        env=env,
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )

    assert result.returncode == 2
    assert "canonical px4_msgs source is missing" in result.stderr
    assert "common runtime ok" in result.stdout
    source = runner.read_text(encoding="utf-8")
    assert source.index('"$PYTHON_BIN" -m sasl_common.operator_runtime') < source.index(
        'source /opt/ros/humble/setup.bash'
    )


def test_workstation_check_skips_ros_overlay_bootstrap(tmp_path: Path) -> None:
    _repository, runner = _fake_launcher_tree(tmp_path)
    env = os.environ.copy()
    env["SASL_ROS_WORKSPACE"] = "/definitely/missing/sasl-workspace"
    result = subprocess.run(
        [str(runner), "--check", "--ui", "workstation"],
        cwd=runner.parent,
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
        [str(runner), "--ui", "workstation", "--check"],
        cwd=runner.parent,
        env=env,
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    assert result_reordered.returncode == 0, result_reordered.stderr


def test_mavlink_only_check_skips_ros_overlay_bootstrap(tmp_path: Path) -> None:
    _repository, runner = _fake_launcher_tree(tmp_path)
    env = os.environ.copy()
    env["SASL_ROS_WORKSPACE"] = "/definitely/missing/sasl-workspace"
    env["SASL_MAVLINK_ONLY"] = "1"
    result = subprocess.run(
        [str(runner), "--check"],
        cwd=runner.parent,
        env=env,
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )

    assert result.returncode == 0, result.stderr
    assert "StereoWIRIS runtime check ok" in result.stdout
    assert "canonical px4_msgs source is missing" not in result.stderr


def test_runner_keeps_ros_bootstrap_conditional_to_legacy() -> None:
    source = RUNNER.read_text(encoding="utf-8")

    assert 'WORKSTATION_UI_REQUESTED=0' in source
    assert 'MAVLINK_ONLY_REQUESTED=0' in source
    assert 'if [[ "$WORKSTATION_UI_REQUESTED" != "1" && "$MAVLINK_ONLY_REQUESTED" != "1" ]]; then' in source
    assert 'source /opt/ros/humble/setup.bash' in source
