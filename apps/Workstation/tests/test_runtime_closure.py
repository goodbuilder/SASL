from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest


REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
APPS_ROOT = REPOSITORY_ROOT / "apps"
COMMON_PYTHON_ROOT = REPOSITORY_ROOT / "common" / "python"


def _subprocess_environment(data_root: Path) -> dict[str, str]:
    environment = os.environ.copy()
    environment["SASL_WORKSTATION_DATA_ROOT"] = str(data_root)
    environment["PYTHONPATH"] = os.pathsep.join(
        (
            str(APPS_ROOT),
            str(REPOSITORY_ROOT),
            str(COMMON_PYTHON_ROOT),
            environment.get("PYTHONPATH", ""),
        )
    )
    return environment


def test_runtime_paths_and_settings_follow_data_root_override_from_any_cwd(tmp_path: Path) -> None:
    data_root = tmp_path / "operator-data"
    script = """
import json
from Workstation.runtime_paths import (
    WORKSTATION_CALIBRATION_ROOT,
    WORKSTATION_DATA_ROOT,
    WORKSTATION_MODELS_ROOT,
    WORKSTATION_RECORDINGS_ROOT,
    WORKSTATION_SETTINGS_PATH,
    resolve_workstation_data_path,
)
from Workstation.workstation_core.settings import AppSettings, DEFAULT_SETTINGS_PATH

settings = AppSettings()
print(json.dumps({
    "data": str(WORKSTATION_DATA_ROOT),
    "settings": str(WORKSTATION_SETTINGS_PATH),
    "recordings": str(WORKSTATION_RECORDINGS_ROOT),
    "calibration": str(WORKSTATION_CALIBRATION_ROOT),
    "models": str(WORKSTATION_MODELS_ROOT),
    "default_settings": str(DEFAULT_SETTINGS_PATH),
    "intrinsics": settings.intrinsics_path,
    "calibration_setting": settings.calibration_root,
    "validation_root": settings.calibration_validation_root,
    "validation_json": settings.calibration_validation_json,
    "weights": settings.yolo_weights_path,
    "resolved_intrinsics": str(resolve_workstation_data_path(settings.intrinsics_path)),
    "resolved_weights": str(resolve_workstation_data_path(settings.yolo_weights_path)),
}))
"""
    observed = []
    for cwd in (tmp_path, tmp_path / "different-cwd"):
        cwd.mkdir(exist_ok=True)
        result = subprocess.run(
            [sys.executable, "-c", script],
            cwd=cwd,
            env=_subprocess_environment(data_root),
            check=True,
            capture_output=True,
            text=True,
        )
        observed.append(json.loads(result.stdout))

    assert observed[0] == observed[1]
    assert observed[0] == {
        "data": str(data_root),
        "settings": str(data_root / "stereo_distance_settings.json"),
        "recordings": str(data_root / "recordings"),
        "calibration": str(data_root / "calibration"),
        "models": str(data_root / "models"),
        "default_settings": str(data_root / "stereo_distance_settings.json"),
        "intrinsics": "calibration/intrinsics.json",
        "calibration_setting": "calibration",
        "validation_root": "calibration",
        "validation_json": "calibration/intrinsics.json",
        "weights": "models/yolov8m.pt",
        "resolved_intrinsics": str(data_root / "calibration" / "intrinsics.json"),
        "resolved_weights": str(data_root / "models" / "yolov8m.pt"),
    }


def test_relative_data_root_override_is_repository_anchored(tmp_path: Path) -> None:
    environment = _subprocess_environment(tmp_path / "unused")
    environment["SASL_WORKSTATION_DATA_ROOT"] = ".local/test-workstation-root"
    script = "from Workstation.runtime_paths import WORKSTATION_DATA_ROOT; print(WORKSTATION_DATA_ROOT)"
    results = []
    for cwd in (tmp_path, tmp_path / "other"):
        cwd.mkdir(exist_ok=True)
        result = subprocess.run(
            [sys.executable, "-c", script],
            cwd=cwd,
            env=environment,
            check=True,
            capture_output=True,
            text=True,
        )
        results.append(result.stdout.strip())

    expected = str((REPOSITORY_ROOT / ".local" / "test-workstation-root").resolve())
    assert results == [expected, expected]


def test_model_defaults_and_package_lookup_share_the_data_root(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    data_root = tmp_path / "runtime-data"
    script = """
import json
from Workstation.runtime_paths import resolve_workstation_data_path
from Workstation.workstation_core.yolo_stream import DEFAULT_YOLO_WEIGHTS
print(json.dumps({
    "default": DEFAULT_YOLO_WEIGHTS,
    "relative": str(resolve_workstation_data_path("models/custom.pt")),
}))
"""
    result = subprocess.run(
        [sys.executable, "-c", script],
        cwd=tmp_path,
        env=_subprocess_environment(data_root),
        check=True,
        capture_output=True,
        text=True,
    )
    observed = json.loads(result.stdout)
    assert observed == {
        "default": str(data_root / "models" / "yolov8m.pt"),
        "relative": str(data_root / "models" / "custom.pt"),
    }

    from Workstation import runtime_paths
    from Workstation.measurement_package_controller import MeasurementPackageController

    monkeypatch.setattr(runtime_paths, "WORKSTATION_DATA_ROOT", data_root)
    model = data_root / "models" / "custom.pt"
    model.parent.mkdir(parents=True)
    model.write_bytes(b"model-sentinel")
    assert MeasurementPackageController._first_existing_path("models/custom.pt") == model


def test_settings_preserve_custom_paths_and_read_known_legacy_defaults(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from Workstation.workstation_core import settings as settings_module

    legacy_calibration = tmp_path / "legacy-calibration"
    legacy_models = tmp_path / "legacy-models"
    legacy_calibration.mkdir()
    legacy_models.mkdir()
    (legacy_calibration / "intrinsics.json").write_text("{}", encoding="utf-8")
    (legacy_models / "yolov8m.pt").write_bytes(b"legacy-model")
    monkeypatch.setattr(settings_module, "LEGACY_CALIBRATION_CAPTURES_ROOT", legacy_calibration)
    monkeypatch.setattr(settings_module, "LEGACY_REPOSITORY_DATA_ROOT", tmp_path / "unused-data")
    monkeypatch.setattr(settings_module, "LEGACY_MODELS_ROOT", legacy_models)

    legacy_path = tmp_path / "legacy-settings.json"
    legacy_path.write_text(
        json.dumps(
            {
                "intrinsics_path": "calibration_captures/intrinsics.json",
                "calibration_root": "calibration_captures",
                "calibration_validation_root": "calibration_captures",
                "calibration_validation_json": "calibration_captures/intrinsics.json",
                "yolo_weights_path": ".local/models/yolov8m.pt",
            }
        ),
        encoding="utf-8",
    )
    legacy = settings_module.load_settings(legacy_path)
    assert legacy.intrinsics_path == str(legacy_calibration / "intrinsics.json")
    assert legacy.calibration_root == str(legacy_calibration)
    assert legacy.calibration_validation_root == str(legacy_calibration)
    assert legacy.calibration_validation_json == str(legacy_calibration / "intrinsics.json")
    assert legacy.yolo_weights_path == str(legacy_models / "yolov8m.pt")

    legacy_nano = tmp_path / "legacy-nano-settings.json"
    legacy_nano.write_text(json.dumps({"yolo_weights_path": "yolov8n.pt"}), encoding="utf-8")
    monkeypatch.setattr(settings_module, "APP_ROOT", tmp_path)
    (tmp_path / "yolov8n.pt").write_bytes(b"legacy-nano")
    assert settings_module.load_settings(legacy_nano).yolo_weights_path == str(
        tmp_path / "yolov8n.pt"
    )

    custom_payload = {
        "intrinsics_path": " custom/intrinsics.json ",
        "calibration_root": "custom/captures",
        "calibration_validation_root": "/not-yet-mounted/validation",
        "calibration_validation_json": "custom/validation.json",
        "yolo_weights_path": "models/custom.pt",
    }
    custom_path = tmp_path / "custom-settings.json"
    custom_path.write_text(json.dumps(custom_payload), encoding="utf-8")
    custom = settings_module.load_settings(custom_path)
    for field_name, expected in custom_payload.items():
        assert getattr(custom, field_name) == expected


def test_default_settings_read_legacy_file_without_mutating_or_migrating(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from Workstation.workstation_core import settings as settings_module

    canonical_path = tmp_path / "canonical" / "settings.json"
    legacy_path = tmp_path / "legacy" / "settings.json"
    legacy_path.parent.mkdir()
    legacy_path.write_text(json.dumps({"preview_width": 777}), encoding="utf-8")
    monkeypatch.setattr(settings_module, "WORKSTATION_SETTINGS_PATH", canonical_path)
    monkeypatch.setattr(settings_module, "LEGACY_DEFAULT_SETTINGS_PATH", legacy_path)

    loaded = settings_module.load_settings(canonical_path, create_if_missing=False)

    assert loaded.preview_width == 777
    assert not canonical_path.exists()
    assert json.loads(legacy_path.read_text(encoding="utf-8")) == {"preview_width": 777}


def _valid_runtime_versions(*, include_av: bool = False) -> dict[str, str]:
    versions = {
        "PyQt5": "5.15.6",
        "numpy": "1.24.4",
        "scipy": "1.10.1",
        "opencv-python": "4.9.0.80",
        "PyYAML": "6.0.2",
        "pyarrow": "23.0.0",
        "XlsxWriter": "3.2.0",
        "pyserial": "3.5",
        "pymavlink": "2.4.49",
        "pyqtgraph": "0.13.7",
        "keyring": "23.5.0",
        "setuptools": "79.0.1",
        "packaging": "25.0",
        "ultralytics": "8.4.0",
    }
    if include_av:
        versions["av"] = "14.0.0"
    return versions


def _passing_common_runtime_report():
    from sasl_common.operator_runtime import Diagnostic, RuntimeReport

    return RuntimeReport((Diagnostic("PASS", "test.common", "fake common runtime ready"),))


def test_runtime_requirements_make_av_conditional_and_numpy_strict() -> None:
    from Workstation.__main__ import _runtime_requirements, _runtime_version_errors

    gstreamer_requirements = _runtime_requirements(SimpleNamespace(stream_backend="gstreamer"))
    pyav_requirements = _runtime_requirements(SimpleNamespace(stream_backend="pyav"))
    assert "av" not in {item.distribution for item in gstreamer_requirements}
    assert "av" in {item.distribution for item in pyav_requirements}
    assert _runtime_version_errors(_valid_runtime_versions(), gstreamer_requirements) == []

    incompatible = _valid_runtime_versions()
    incompatible["numpy"] = "1.26.4"
    errors = _runtime_version_errors(incompatible, gstreamer_requirements)
    assert errors == ["numpy==1.26.4 does not satisfy >=1.23.5,<1.25"]

    incompatible = _valid_runtime_versions()
    incompatible["setuptools"] = "80.0.0"
    errors = _runtime_version_errors(incompatible, gstreamer_requirements)
    assert errors == ["setuptools==80.0.0 does not satisfy >=77,<80"]

    errors = _runtime_version_errors(_valid_runtime_versions(), pyav_requirements)
    assert errors == ["missing distribution av (required >=11,<16)"]


def test_runtime_check_imports_app_capabilities_after_common_gate(
    capsys: pytest.CaptureFixture[str],
) -> None:
    from Workstation.__main__ import _runtime_check, _runtime_requirements

    settings = SimpleNamespace(stream_backend="gstreamer")
    requirements = _runtime_requirements(settings)

    imports = {requirement.import_name: object() for requirement in requirements}
    imports.update(
        {
            "sasl_common.lifecycle": object(),
            "Workstation.main_window": object(),
            "Workstation.standalone_gui": object(),
        }
    )

    assert _runtime_check(
        settings=settings,
        versions=_valid_runtime_versions(),
        importer=imports,
        common_report=_passing_common_runtime_report(),
    ) == 0
    output = capsys.readouterr()
    assert "runtime check ok" in output.out
    assert output.err == ""


def test_runtime_check_fails_for_selected_pyav_without_av(
    capsys: pytest.CaptureFixture[str],
) -> None:
    from Workstation.__main__ import _runtime_check

    assert _runtime_check(
        settings=SimpleNamespace(stream_backend="pyav"),
        versions=_valid_runtime_versions(),
        importer={},
        common_report=_passing_common_runtime_report(),
    ) == 1
    output = capsys.readouterr()
    assert "missing distribution av" in output.err


def test_runtime_model_check_requires_bytes_and_validates_default_hash(tmp_path: Path) -> None:
    from Workstation.__main__ import _runtime_model_errors

    model = tmp_path / "yolov8m.pt"
    manifest = tmp_path / "model-manifest.json"
    settings = SimpleNamespace(yolo_weights_path=str(model))
    assert "missing" in _runtime_model_errors(settings, manifest_path=manifest)[0]

    payload = b"approved-model-sentinel"
    model.write_bytes(payload)
    manifest.write_text(
        json.dumps(
            {
                "size_bytes": len(payload),
                "sha256": __import__("hashlib").sha256(payload).hexdigest(),
            }
        ),
        encoding="utf-8",
    )
    assert _runtime_model_errors(settings, manifest_path=manifest) == []
    model.write_bytes(b"tampered-model-sentinel")
    assert _runtime_model_errors(settings, manifest_path=manifest)


def test_launcher_help_is_dependency_free_and_does_not_create_runtime_state(tmp_path: Path) -> None:
    data_root = tmp_path / "runtime"
    environment = _subprocess_environment(data_root)
    environment["PYTHON_BIN"] = sys.executable
    result = subprocess.run(
        [str(Path(__file__).resolve().parents[1] / "run.sh"), "--help"],
        cwd=tmp_path,
        env=environment,
        check=True,
        capture_output=True,
        text=True,
    )

    assert "--check" in result.stdout
    assert "runtime check failed" not in result.stderr
    assert not data_root.exists()
