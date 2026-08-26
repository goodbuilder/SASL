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


def _copy_runtime_doctor(tmp_path: Path) -> tuple[Path, Path]:
    fake_root = tmp_path / "runtime-doctor"
    destination = fake_root / "tools" / "check_operator_runtime.sh"
    destination.parent.mkdir(parents=True)
    shutil.copy2(REPOSITORY_ROOT / "tools" / "check_operator_runtime.sh", destination)
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


@pytest.mark.parametrize("app_name", ("StereoWIRIS", "Workstation"))
def test_launcher_enforces_isolated_user_site_and_pythonpath(
    tmp_path: Path,
    app_name: str,
) -> None:
    fake_root, launcher = _copy_launcher(tmp_path, app_name)
    canonical = fake_root / ".venv" / "bin" / "python"
    observed = fake_root / "observed-env"
    canonical.parent.mkdir(parents=True)
    canonical.write_text(
        "#!/bin/sh\n"
        f"printf '%s\\n' \"$PYTHONNOUSERSITE\" \"$PYTHONPATH\" \"$PATH\" "
        "\"${PYTHONHOME-unset}\" \"${LD_LIBRARY_PATH-unset}\" "
        "\"${GI_TYPELIB_PATH-unset}\" \"${GST_PLUGIN_PATH-unset}\" "
        "\"${GST_PLUGIN_PATH_1_0-unset}\" \"${GST_PLUGIN_SYSTEM_PATH-unset}\" "
        "\"${GST_PLUGIN_SYSTEM_PATH_1_0-unset}\" \"${GST_PLUGIN_SCANNER-unset}\" "
        f"\"${{GST_PLUGIN_SCANNER_1_0-unset}}\" > '{observed}'\n"
        "exit 0\n",
        encoding="utf-8",
    )
    canonical.chmod(0o755)
    env = os.environ.copy()
    env["PYTHONPATH"] = "/tmp/ambient-pythonpath"
    env["PATH"] = f"{Path.home() / '.local/bin'}:{env.get('PATH', '')}"
    env["PYTHONHOME"] = "/tmp/ambient-pythonhome"
    env["LD_LIBRARY_PATH"] = "/tmp/ambient-library"
    env["GI_TYPELIB_PATH"] = "/tmp/ambient-typelib"
    env["GST_PLUGIN_PATH"] = "/tmp/ambient-gst-plugin"
    env["GST_PLUGIN_PATH_1_0"] = "/tmp/ambient-gst-plugin-1.0"
    env["GST_PLUGIN_SYSTEM_PATH"] = "/tmp/ambient-gst-system-plugin"
    env["GST_PLUGIN_SYSTEM_PATH_1_0"] = "/tmp/ambient-gst-system-plugin-1.0"
    env["GST_PLUGIN_SCANNER"] = "/tmp/ambient-gst-scanner"
    env["GST_PLUGIN_SCANNER_1_0"] = "/tmp/ambient-gst-scanner-1.0"

    arguments = ["--check"]
    if app_name == "StereoWIRIS":
        arguments.append("--mavlink-only")
    result = subprocess.run(
        [str(launcher), *arguments],
        cwd=tmp_path,
        env=env,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0
    (
        no_user_site,
        python_path,
        executable_path,
        python_home,
        library_path,
        typelib_path,
        plugin_path,
        plugin_path_1_0,
        system_plugin_path,
        system_plugin_path_1_0,
        plugin_scanner,
        plugin_scanner_1_0,
    ) = observed.read_text(encoding="utf-8").splitlines()
    assert no_user_site == "1"
    assert "/tmp/ambient-pythonpath" not in python_path
    assert ".local/bin" not in executable_path
    assert python_home == "unset"
    assert library_path == "unset"
    assert typelib_path == "unset"
    assert plugin_path == "unset"
    assert plugin_path_1_0 == "unset"
    assert system_plugin_path == "unset"
    assert system_plugin_path_1_0 == "unset"
    assert plugin_scanner == "unset"
    assert plugin_scanner_1_0 == "unset"


def test_runtime_doctor_enforces_the_same_isolated_environment(tmp_path: Path) -> None:
    fake_root, doctor = _copy_runtime_doctor(tmp_path)
    canonical = fake_root / ".venv" / "bin" / "python"
    observed = fake_root / "observed-env"
    canonical.parent.mkdir(parents=True)
    canonical.write_text(
        "#!/bin/sh\n"
        f"printf '%s\\n' \"$PYTHONNOUSERSITE\" \"$PYTHONPATH\" \"$PATH\" "
        "\"${PYTHONHOME-unset}\" \"${LD_LIBRARY_PATH-unset}\" "
        "\"${GI_TYPELIB_PATH-unset}\" \"${GST_PLUGIN_PATH-unset}\" "
        "\"${GST_PLUGIN_PATH_1_0-unset}\" \"${GST_PLUGIN_SYSTEM_PATH-unset}\" "
        "\"${GST_PLUGIN_SYSTEM_PATH_1_0-unset}\" \"${GST_PLUGIN_SCANNER-unset}\" "
        f"\"${{GST_PLUGIN_SCANNER_1_0-unset}}\" > '{observed}'\n"
        "exit 0\n",
        encoding="utf-8",
    )
    canonical.chmod(0o755)
    env = os.environ.copy()
    env.update(
        {
            "PYTHONPATH": "/tmp/ambient-pythonpath",
            "PYTHONHOME": "/tmp/ambient-pythonhome",
            "LD_LIBRARY_PATH": "/tmp/ambient-library",
            "GI_TYPELIB_PATH": "/tmp/ambient-typelib",
            "GST_PLUGIN_PATH": "/tmp/ambient-gst-plugin",
            "GST_PLUGIN_PATH_1_0": "/tmp/ambient-gst-plugin-1.0",
            "GST_PLUGIN_SYSTEM_PATH": "/tmp/ambient-gst-system-plugin",
            "GST_PLUGIN_SYSTEM_PATH_1_0": "/tmp/ambient-gst-system-plugin-1.0",
            "GST_PLUGIN_SCANNER": "/tmp/ambient-gst-scanner",
            "GST_PLUGIN_SCANNER_1_0": "/tmp/ambient-gst-scanner-1.0",
            "PATH": f"{Path.home() / '.local/bin'}:{env.get('PATH', '')}",
        }
    )

    result = subprocess.run(
        [str(doctor), "--execution"],
        cwd=tmp_path,
        env=env,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0
    values = observed.read_text(encoding="utf-8").splitlines()
    assert values[0] == "1"
    assert values[1] == str(fake_root / "common" / "python")
    assert ".local/bin" not in values[2]
    assert values[3:] == ["unset"] * 9


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

    class FakeSharedValue:
        def __init__(self, value: int) -> None:
            self.value = value

        def get_lock(self):
            return self

        def __enter__(self):
            return self

        def __exit__(self, _exc_type, _exc_value, _traceback) -> bool:
            return False

    class FakeContext:
        executable: str | None = None

        def set_executable(self, executable: str) -> None:
            self.executable = executable

        def Value(self, _typecode: str, value: int) -> FakeSharedValue:
            return FakeSharedValue(value)

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
