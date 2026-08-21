"""Direct launcher for the standalone Workstation application."""

from __future__ import annotations

import argparse
import faulthandler
import hashlib
import json
import os
import signal
import sys
import tempfile
from importlib import import_module, metadata
from pathlib import Path
from typing import Callable, Mapping, NamedTuple


class RuntimeRequirement(NamedTuple):
    distribution: str
    import_name: str
    specifier: str


# Keep these constraints aligned with requirements/runtime.txt.  They live
# here as well so ``--check`` validates the environment instead of merely
# proving that an unsupported installed version happens to import.  PyAV is
# evaluated separately because only the PyAV stream profile imports it.
CORE_RUNTIME_REQUIREMENTS = (
    RuntimeRequirement("PyQt5", "PyQt5.QtCore", ">=5.15,<6"),
    RuntimeRequirement("numpy", "numpy", ">=1.23.5,<1.25"),
    RuntimeRequirement("scipy", "scipy", ">=1.8,<1.11"),
    RuntimeRequirement("opencv-python", "cv2", ">=4.8,<5"),
    RuntimeRequirement("PyYAML", "yaml", ">=6,<7"),
    RuntimeRequirement("pyarrow", "pyarrow", ">=14,<24"),
    RuntimeRequirement("XlsxWriter", "xlsxwriter", ">=3.1,<4"),
    RuntimeRequirement("pyserial", "serial", ">=3.5,<4"),
    RuntimeRequirement("pymavlink", "pymavlink", ">=2.4.41,<3"),
    RuntimeRequirement("pyqtgraph", "pyqtgraph", ">=0.13,<0.14"),
    RuntimeRequirement("keyring", "keyring", ">=23,<26"),
    RuntimeRequirement("packaging", "packaging", ">=23,<27"),
    RuntimeRequirement("ultralytics", "ultralytics", ">=8.3,<9"),
)
PYAV_RUNTIME_REQUIREMENT = RuntimeRequirement("av", "av", ">=11,<16")


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Start the SASL Workstation UI.")
    parser.add_argument(
        "--check",
        action="store_true",
        help="Verify runtime versions and selected capabilities without opening Qt.",
    )
    parser.add_argument("--maximized", action="store_true", help="Open the Workstation maximized.")
    parser.add_argument(
        "--start",
        dest="start",
        action="store_true",
        default=False,
        help="Enable WIRIS/RTSP and start the camera preview after launch.",
    )
    parser.add_argument(
        "--no-start",
        dest="start",
        action="store_false",
        help="Open the Workstation without starting camera discovery or preview (default).",
    )
    parser.add_argument(
        "--start-pixhawk",
        "--connect-pixhawk",
        dest="start_pixhawk",
        action="store_true",
        help="Connect the configured Pixhawk MAVLink endpoints after launch.",
    )
    return parser


def _selected_stream_backend(settings: object) -> str:
    value = str(getattr(settings, "stream_backend", "gstreamer") or "gstreamer").strip().lower()
    if value in {"pyav", "av"}:
        return "pyav"
    if value in {"opencv", "ffmpeg", "opencv_ffmpeg", "cv2"}:
        return "opencv"
    if value in {"opencv_auto", "opencv-auto", "opencv_any", "opencv-any", "cap_any"}:
        return "opencv_auto"
    return "gstreamer"


def _runtime_requirements(settings: object) -> tuple[RuntimeRequirement, ...]:
    requirements = list(CORE_RUNTIME_REQUIREMENTS)
    if _selected_stream_backend(settings) == "pyav":
        requirements.append(PYAV_RUNTIME_REQUIREMENT)
    return tuple(requirements)


def _runtime_version_errors(
    versions: Mapping[str, str | None],
    requirements: tuple[RuntimeRequirement, ...],
) -> list[str]:
    try:
        from packaging.specifiers import InvalidSpecifier, SpecifierSet
        from packaging.version import InvalidVersion, Version
    except Exception as exc:
        return [f"version validator unavailable (packaging): {exc}"]

    errors: list[str] = []
    for requirement in requirements:
        installed = versions.get(requirement.distribution)
        if not installed:
            errors.append(
                f"missing distribution {requirement.distribution} "
                f"(required {requirement.specifier})"
            )
            continue
        try:
            accepted = Version(str(installed)) in SpecifierSet(requirement.specifier)
        except (InvalidSpecifier, InvalidVersion) as exc:
            errors.append(f"invalid {requirement.distribution} version metadata {installed!r}: {exc}")
            continue
        if not accepted:
            errors.append(
                f"{requirement.distribution}=={installed} does not satisfy {requirement.specifier}"
            )
    return errors


def _installed_runtime_versions(
    requirements: tuple[RuntimeRequirement, ...],
    version_lookup: Callable[[str], str] | None = None,
) -> dict[str, str | None]:
    lookup = metadata.version if version_lookup is None else version_lookup
    versions: dict[str, str | None] = {}
    for requirement in requirements:
        try:
            versions[requirement.distribution] = str(lookup(requirement.distribution))
        except metadata.PackageNotFoundError:
            versions[requirement.distribution] = None
        except Exception:
            versions[requirement.distribution] = None
    return versions


def _module_loader(
    importer: Callable[[str], object] | Mapping[str, object] | None,
) -> Callable[[str], object]:
    if importer is None:
        return import_module
    if callable(importer):
        return importer

    def load_from_mapping(name: str) -> object:
        if name not in importer:
            raise ModuleNotFoundError(name)
        result = importer[name]
        if isinstance(result, BaseException):
            raise result
        return result

    return load_from_mapping


def _runtime_import_errors(
    settings: object,
    requirements: tuple[RuntimeRequirement, ...],
    *,
    importer: Callable[[str], object] | Mapping[str, object] | None = None,
) -> list[str]:
    load_module = _module_loader(importer)
    module_names = [requirement.import_name for requirement in requirements]
    module_names.extend(
        (
            "sasl_common.lifecycle",
            "Workstation.main_window",
            "Workstation.standalone_gui",
        )
    )
    backend = _selected_stream_backend(settings)
    if backend == "gstreamer":
        module_names.append("gi")

    errors: list[str] = []
    loaded: dict[str, object] = {}
    for module_name in dict.fromkeys(module_names):
        try:
            loaded[module_name] = load_module(module_name)
        except Exception as exc:
            errors.append(f"cannot import {module_name}: {exc}")

    gi_module = loaded.get("gi")
    if backend == "gstreamer" and gi_module is not None:
        try:
            require_version = getattr(gi_module, "require_version")
            require_version("Gst", "1.0")
            require_version("GstRtp", "1.0")
            load_module("gi.repository.Gst")
            load_module("gi.repository.GstRtp")
        except Exception as exc:
            errors.append(f"GStreamer introspection unavailable: {exc}")
    return errors


def _runtime_model_errors(
    settings: object,
    *,
    manifest_path: Path | None = None,
) -> list[str]:
    if not hasattr(settings, "yolo_weights_path"):
        # Narrow unit/integration adapters may exercise only dependency
        # selection. Production AppSettings always declares this field.
        return []
    configured = str(getattr(settings, "yolo_weights_path", "") or "").strip()
    if not configured:
        return ["YOLO model path is not configured"]
    from Workstation.runtime_paths import resolve_workstation_data_path

    model_path = resolve_workstation_data_path(configured)
    if not model_path.is_file():
        return [f"YOLO model is missing: {model_path}"]
    selected_manifest = manifest_path or Path(__file__).with_name("model-manifest.json")
    if model_path.name != "yolov8m.pt" or not selected_manifest.is_file():
        return []
    try:
        manifest = json.loads(selected_manifest.read_text(encoding="utf-8"))
        expected_size = int(manifest["size_bytes"])
        expected_sha256 = str(manifest["sha256"]).lower()
    except (KeyError, OSError, TypeError, ValueError) as exc:
        return [f"YOLO model manifest is invalid: {exc}"]
    if model_path.stat().st_size != expected_size:
        return [
            f"YOLO model size mismatch: expected={expected_size}, "
            f"actual={model_path.stat().st_size}, path={model_path}"
        ]
    digest = hashlib.sha256()
    with model_path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    if digest.hexdigest() != expected_sha256:
        return [f"YOLO model SHA-256 does not match model-manifest.json: {model_path}"]
    return []


def _runtime_check(
    *,
    settings: object | None = None,
    versions: Mapping[str, str | None] | None = None,
    importer: Callable[[str], object] | Mapping[str, object] | None = None,
    version_lookup: Callable[[str], str] | None = None,
) -> int:
    try:
        if settings is None:
            from Workstation.runtime_paths import WORKSTATION_SETTINGS_PATH
            from Workstation.workstation_core.settings import load_settings

            settings = load_settings(WORKSTATION_SETTINGS_PATH, create_if_missing=False)
    except Exception as exc:
        print(f"SASL Workstation runtime check failed:\n- cannot load settings: {exc}", file=sys.stderr)
        return 1

    requirements = _runtime_requirements(settings)
    installed_versions = (
        dict(versions)
        if versions is not None
        else _installed_runtime_versions(requirements, version_lookup=version_lookup)
    )
    errors = _runtime_version_errors(installed_versions, requirements)
    errors.extend(_runtime_model_errors(settings))
    if not errors:
        errors.extend(_runtime_import_errors(settings, requirements, importer=importer))
    if errors:
        print("SASL Workstation runtime check failed:", file=sys.stderr)
        for error in errors:
            print(f"- {error}", file=sys.stderr)
        return 1

    version_summary = ", ".join(
        f"{requirement.distribution}={installed_versions[requirement.distribution]}"
        for requirement in requirements
    )
    print("SASL Workstation runtime check ok")
    print(f"python={sys.executable}")
    print(f"repository={Path(__file__).resolve().parents[2]}")
    print(f"stream_backend={_selected_stream_backend(settings)}")
    print(f"yolo_model={getattr(settings, 'yolo_weights_path', '')}")
    print(version_summary)
    return 0


def _workstation_lock_path() -> Path:
    uid = os.getuid() if hasattr(os, "getuid") else "user"
    runtime_dir = os.environ.get("XDG_RUNTIME_DIR", "").strip()
    base = Path(runtime_dir) if runtime_dir else Path(tempfile.gettempdir())
    return base / f"sasl-workstation-{uid}.lock"


def _acquire_single_instance_lock(lock_path: Path | None = None):
    """Keep one Workstation process per OS user to protect camera SDK ports."""

    from PyQt5 import QtCore

    lock = QtCore.QLockFile(str(lock_path or _workstation_lock_path()))
    lock.setStaleLockTime(30_000)
    return lock if lock.tryLock(0) else None


def main(argv: list[str] | None = None) -> int:
    os.environ.setdefault("QT_AUTO_SCREEN_SCALE_FACTOR", "1")
    try:
        faulthandler.register(signal.SIGUSR1)
    except (AttributeError, OSError, RuntimeError):
        pass
    args, qt_args = _parser().parse_known_args(sys.argv[1:] if argv is None else argv)
    if args.check:
        return _runtime_check()

    from PyQt5 import QtWidgets

    from .main_window import WorkstationMainWindow
    from .standalone_gui import _configure_gl_surface_format

    _configure_gl_surface_format()
    app = QtWidgets.QApplication([sys.argv[0], *qt_args])
    instance_lock = _acquire_single_instance_lock()
    if instance_lock is None:
        print("SASL Workstation is already running; duplicate launch was blocked.", file=sys.stderr)
        return 0
    window = WorkstationMainWindow(
        auto_start=bool(args.start),
        auto_start_pixhawk=bool(args.start_pixhawk),
    )
    if args.maximized:
        window.showMaximized()
    else:
        window.show()
    return int(app.exec_())


if __name__ == "__main__":
    raise SystemExit(main())
