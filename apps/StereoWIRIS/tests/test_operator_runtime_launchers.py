from __future__ import annotations

import os
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

REPOSITORY_ROOT = Path(__file__).resolve().parents[3]


def _copy_launcher(tmp_path: Path, app_name: str) -> tuple[Path, Path]:
    fake_root = tmp_path / app_name
    destination = fake_root / "apps" / app_name / "run.sh"
    destination.parent.mkdir(parents=True)
    shutil.copy2(REPOSITORY_ROOT / "apps" / app_name / "run.sh", destination)
    return fake_root, destination


@pytest.mark.parametrize("app_name", ("StereoWIRIS", "Workstation"))
def test_launcher_help_needs_no_venv_and_ignores_wrong_override(
    tmp_path: Path,
    app_name: str,
) -> None:
    _root, launcher = _copy_launcher(tmp_path, app_name)
    env = os.environ.copy()
    env["PYTHON_BIN"] = "/definitely/wrong/python"

    result = subprocess.run(
        [str(launcher), "--help"],
        cwd=tmp_path,
        env=env,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0
    assert "--check" in result.stdout
    assert "canonical venv is missing" not in result.stderr


@pytest.mark.parametrize(
    ("app_name", "arguments"),
    (("StereoWIRIS", ("--check", "--mavlink-only")), ("Workstation", ("--check",))),
)
def test_launcher_does_not_fall_back_to_app_or_simulation_venv(
    tmp_path: Path,
    app_name: str,
    arguments: tuple[str, ...],
) -> None:
    fake_root, launcher = _copy_launcher(tmp_path, app_name)
    marker = fake_root / "fallback-used"
    for relative in (
        "apps/StereoWIRIS/.venv/bin/python",
        "apps/Workstation/.venv/bin/python",
        "apps/Simulation/.venv/bin/python",
    ):
        candidate = fake_root / relative
        candidate.parent.mkdir(parents=True, exist_ok=True)
        candidate.write_text(f"#!/bin/sh\ntouch '{marker}'\n", encoding="utf-8")
        candidate.chmod(0o755)

    result = subprocess.run(
        [str(launcher), *arguments],
        cwd=tmp_path,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 1
    assert "canonical venv is missing" in result.stderr
    assert not marker.exists()


@pytest.mark.parametrize("app_name", ("StereoWIRIS", "Workstation"))
def test_wrong_python_bin_is_exit_two(tmp_path: Path, app_name: str) -> None:
    fake_root, launcher = _copy_launcher(tmp_path, app_name)
    canonical = fake_root / ".venv" / "bin" / "python"
    canonical.parent.mkdir(parents=True)
    canonical.write_text("#!/bin/sh\nexit 99\n", encoding="utf-8")
    canonical.chmod(0o755)
    env = os.environ.copy()
    env["PYTHON_BIN"] = "/usr/bin/python3"

    result = subprocess.run(
        [str(launcher), "--check"],
        cwd=tmp_path,
        env=env,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 2
    assert "only supports" in result.stderr


def test_direct_workstation_app_venv_is_rejected_before_qt() -> None:
    python = REPOSITORY_ROOT / "apps" / "Workstation" / ".venv" / "bin" / "python"
    if not python.is_file():
        pytest.skip("Workstation local venv is not present on this checkout")
    env = os.environ.copy()
    env["PYTHONPATH"] = os.pathsep.join(
        (
            str(REPOSITORY_ROOT / "apps"),
            str(REPOSITORY_ROOT / "common" / "python"),
        )
    )

    result = subprocess.run(
        [str(python), "-m", "Workstation", "--check"],
        cwd=REPOSITORY_ROOT,
        env=env,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 2
    assert "wrong interpreter" in result.stderr
    assert "QApplication" not in result.stderr


def test_direct_stereowiris_simulation_venv_is_rejected_before_qt() -> None:
    python = REPOSITORY_ROOT / "apps" / "Simulation" / ".venv" / "bin" / "python"
    if not python.is_file():
        pytest.skip("Simulation venv is not present on this checkout")
    env = os.environ.copy()
    env["SASL_MAVLINK_ONLY"] = "1"

    result = subprocess.run(
        [str(python), str(REPOSITORY_ROOT / "apps" / "StereoWIRIS" / "main.py")],
        cwd=REPOSITORY_ROOT,
        env=env,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 2
    assert "wrong interpreter" in result.stderr
    assert "PyQt" not in result.stderr


@pytest.mark.parametrize(
    "module_name",
    ("wiris_stereo_distance.yolo_stream", "Workstation.workstation_core.yolo_stream"),
)
def test_camera_worker_pins_spawn_to_parent_sys_executable(monkeypatch, module_name: str) -> None:
    module = __import__(module_name, fromlist=["YoloStreamWorker"])

    class FakeContext:
        executable: str | None = None

        def set_executable(self, executable: str) -> None:
            self.executable = executable

    context = FakeContext()
    if module_name.startswith("wiris_"):
        monkeypatch.setattr(module.multiprocessing, "get_context", lambda _method: context)
        worker = module.YoloStreamWorker(
            url="rtsp://camera.invalid/visible",
            weights_path="model.pt",
            device="cpu",
            input_size=416,
            conf_threshold=0.5,
            iou_threshold=0.45,
            max_rate_hz=1.0,
        )
    else:
        worker = module.YoloStreamWorker(
            url="rtsp://camera.invalid/visible",
            weights_path="model.pt",
            device="cpu",
            input_size=416,
            conf_threshold=0.5,
            iou_threshold=0.45,
            max_rate_hz=1.0,
            mp_context=context,
        )

    assert worker._mp_context is context
    assert context.executable == sys.executable
    assert module.DEFAULT_YOLO_PYTHON == sys.executable
