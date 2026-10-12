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
import traceback
from collections.abc import Callable, Mapping
from importlib import import_module, metadata
from pathlib import Path
from typing import NamedTuple

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
COMMON_PYTHON = REPOSITORY_ROOT / "common" / "python"
if str(COMMON_PYTHON) not in sys.path:
    sys.path.insert(0, str(COMMON_PYTHON))

from sasl_common.operator_runtime import (
    DECODER_AUTO,
    RuntimeReport,
    check_execution,
    check_official_interpreter,
    render_report,
)


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
    RuntimeRequirement("setuptools", "setuptools", ">=77,<80"),
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
    errors: list[str] = []
    for module_name in dict.fromkeys(module_names):
        try:
            load_module(module_name)
        except Exception as exc:
            errors.append(f"cannot import {module_name}: {exc}")
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
    common_report: RuntimeReport | None = None,
) -> int:
    try:
        if settings is None:
            from Workstation.runtime_paths import WORKSTATION_SETTINGS_PATH
            from Workstation.workstation_core.settings import load_settings

            settings = load_settings(WORKSTATION_SETTINGS_PATH, create_if_missing=False)
    except Exception as exc:
        print(f"SASL Workstation runtime check failed:\n- cannot load settings: {exc}", file=sys.stderr)
        return 1

    runtime_report = common_report or check_execution(
        str(getattr(settings, "rtsp_decoder", DECODER_AUTO)),
        root=REPOSITORY_ROOT,
    )
    report_text = render_report(runtime_report, title="SASL common operator runtime")
    print(report_text, file=sys.stderr if runtime_report.exit_code else sys.stdout)

    requirements = _runtime_requirements(settings)
    installed_versions = (
        dict(versions)
        if versions is not None
        else _installed_runtime_versions(requirements, version_lookup=version_lookup)
    )
    errors = _runtime_version_errors(installed_versions, requirements)
    from Workstation.runtime_paths import workstation_settings_portability_errors

    errors.extend(workstation_settings_portability_errors(settings))
    errors.extend(_runtime_model_errors(settings))
    if not errors:
        errors.extend(_runtime_import_errors(settings, requirements, importer=importer))
    if errors or runtime_report.exit_code:
        print("SASL Workstation runtime check failed:", file=sys.stderr)
        for error in errors:
            print(f"- {error}", file=sys.stderr)
        return runtime_report.exit_code or 1

    version_summary = ", ".join(
        f"{requirement.distribution}={installed_versions[requirement.distribution]}"
        for requirement in requirements
    )
    print("SASL Workstation runtime check ok")
    print(f"python={sys.executable}")
    print(f"repository={Path(__file__).resolve().parents[2]}")
    print(f"stream_backend={_selected_stream_backend(settings)}")
    from Workstation.runtime_paths import resolve_workstation_data_path

    print(f"yolo_model={resolve_workstation_data_path(getattr(settings, 'yolo_weights_path', ''))}")
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


def _flush_window_session_state(window: object | None) -> None:
    """Best-effort durability barrier used after an unhandled Qt callback error."""

    if window is None:
        return
    backend = getattr(window, "backend", None)
    owner = getattr(backend, "owner_widget", None) or getattr(backend, "panel", None)
    if owner is None:
        owner = window

    flusher = getattr(owner, "flush_workstation_run_evidence", None)
    if callable(flusher):
        try:
            if flusher() is False:
                print("Emergency run-evidence flush failed", file=sys.stderr)
        except Exception as exc:  # noqa: BLE001 - emergency path must keep going.
            print(f"Emergency run-evidence flush failed: {exc}", file=sys.stderr)

    spool_groups = (
        getattr(owner, "distance_recording_rows", None),
        getattr(owner, "gnss_base_recording_rows", None),
        getattr(owner, "extrinsics_comparison_recording_rows", None),
        getattr(owner, "method_evaluation_bundle_rows", None),
    )
    nested_groups = (
        getattr(owner, "camera_recording_rows", None),
        getattr(owner, "px4_recording_rows", None),
    )
    spools = list(spool_groups)
    for group in nested_groups:
        if isinstance(group, Mapping):
            spools.extend(group.values())
    for spool in spools:
        flush = getattr(spool, "flush", None)
        if not callable(flush):
            continue
        try:
            if flush() is False:
                print("Emergency recording-spool flush failed", file=sys.stderr)
        except Exception as exc:  # noqa: BLE001 - report every independent spool.
            print(f"Emergency recording-spool flush failed: {exc}", file=sys.stderr)


def _install_qt_exception_hook(
    emergency_flush: Callable[[], object] | None = None,
) -> Callable[[type[BaseException], BaseException, object], None]:
    """Install a non-aborting hook for exceptions escaping Qt slots/timers.

    PyQt routes callback exceptions through ``sys.excepthook``.  Its default
    hook may terminate the process; this hook records the traceback, attempts
    a bounded session flush, and returns so unrelated device links stay alive.
    """

    handling = False

    def handle_exception(exc_type, exc_value, exc_traceback) -> None:
        nonlocal handling
        if handling:
            traceback.print_exception(exc_type, exc_value, exc_traceback, file=sys.stderr)
            return
        handling = True
        try:
            print("Unhandled Workstation Qt callback exception:", file=sys.stderr)
            traceback.print_exception(exc_type, exc_value, exc_traceback, file=sys.stderr)
            if emergency_flush is not None:
                try:
                    emergency_flush()
                except Exception as flush_exc:  # noqa: BLE001 - never recurse through the hook.
                    print(f"Emergency session flush failed: {flush_exc}", file=sys.stderr)
        finally:
            handling = False

    sys.excepthook = handle_exception
    return handle_exception


def main(argv: list[str] | None = None) -> int:
    os.environ.setdefault("QT_AUTO_SCREEN_SCALE_FACTOR", "1")
    try:
        faulthandler.register(signal.SIGUSR1)
    except (AttributeError, OSError, RuntimeError):
        pass
    args, qt_args = _parser().parse_known_args(sys.argv[1:] if argv is None else argv)
    interpreter_report = check_official_interpreter(root=REPOSITORY_ROOT)
    if interpreter_report.exit_code:
        print(
            render_report(interpreter_report, title="SASL Workstation startup blocked"),
            file=sys.stderr,
        )
        return interpreter_report.exit_code
    if args.check:
        return _runtime_check()

    # Validate the common mandatory media runtime before importing Qt. An
    # unavailable optional NVIDIA path is an allowed Auto/software warning;
    # a saved explicit NVIDIA choice is enforced when the camera is started.
    startup_report = check_execution(DECODER_AUTO, root=REPOSITORY_ROOT)
    if startup_report.exit_code:
        print(
            render_report(startup_report, title="SASL Workstation startup blocked"),
            file=sys.stderr,
        )
        return startup_report.exit_code
    if any(item.level == "WARN" for item in startup_report.diagnostics):
        print(render_report(startup_report, title="SASL Workstation startup warning"))

    from .workstation_core.native_frame_ring import (
        reclaim_orphaned_native_frame_segments,
    )

    reclaimed_native_segments = reclaim_orphaned_native_frame_segments()
    if reclaimed_native_segments:
        print(
            "[startup] reclaimed "
            f"{len(reclaimed_native_segments)} orphan native-frame shared-memory segment(s)"
        )

    from PyQt5 import QtWidgets

    from Workstation.application.main_window import WorkstationMainWindow
    from Workstation.application.standalone_gui import _configure_gl_surface_format

    _configure_gl_surface_format()
    app = QtWidgets.QApplication([sys.argv[0], *qt_args])
    window_holder: dict[str, object] = {}
    _install_qt_exception_hook(
        lambda: _flush_window_session_state(window_holder.get("window"))
    )
    instance_lock = _acquire_single_instance_lock()
    if instance_lock is None:
        print("SASL Workstation is already running; duplicate launch was blocked.", file=sys.stderr)
        return 0
    window = WorkstationMainWindow(
        auto_start=bool(args.start),
        auto_start_pixhawk=bool(args.start_pixhawk),
    )
    window_holder["window"] = window
    if args.maximized:
        window.showMaximized()
    else:
        window.show()
    return int(app.exec_())


if __name__ == "__main__":
    raise SystemExit(main())
