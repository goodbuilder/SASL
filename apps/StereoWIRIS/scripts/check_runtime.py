#!/usr/bin/env python3
"""Read-only application-specific runtime check for StereoWIRIS."""

from __future__ import annotations

import argparse
import hashlib
import importlib
import json
import sys
from pathlib import Path

APP_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = APP_ROOT.parents[1]
COMMON_PYTHON = REPOSITORY_ROOT / "common" / "python"
for path in (COMMON_PYTHON, APP_ROOT, REPOSITORY_ROOT / "apps", REPOSITORY_ROOT):
    path_text = str(path)
    if path_text not in sys.path:
        sys.path.insert(0, path_text)

from sasl_common.operator_runtime import (
    DECODER_AUTO,
    Diagnostic,
    RuntimeReport,
    check_execution,
    combine_reports,
    normalize_decoder_mode,
    probe_gstreamer,
    render_report,
)

SETTINGS_PATH = APP_ROOT / "stereo_distance_settings.json"
MODEL_MANIFEST_PATH = APP_ROOT / "model-manifest.json"
DEFAULT_MODEL_FILENAME = "yolov8n_uav.pt"
REQUIRED_APP_IMPORTS = (
    "cv2",
    "numpy",
    "pyarrow",
    "xlsxwriter",
    "yaml",
    "PyQt5.QtCore",
    "sasl_common.lifecycle",
    "sasl_common.stereo_geometry",
    "wiris_stereo_distance.gui",
    "wiris_stereo_distance.legacy_gui",
)


def _settings() -> tuple[dict[str, object], list[Diagnostic]]:
    if not SETTINGS_PATH.exists():
        return {}, [
            Diagnostic(
                "WARN",
                "stereowiris.settings",
                f"settings file is absent; defaults will be used: {SETTINGS_PATH}",
            )
        ]
    try:
        payload = json.loads(SETTINGS_PATH.read_text(encoding="utf-8"))
    except (OSError, TypeError, ValueError) as exc:
        return {}, [Diagnostic("FAIL", "stereowiris.settings", f"cannot read settings: {exc}")]
    if not isinstance(payload, dict):
        return {}, [Diagnostic("FAIL", "stereowiris.settings", "settings root must be a JSON object")]
    return payload, [Diagnostic("PASS", "stereowiris.settings", f"settings readable: {SETTINGS_PATH}")]


def _model_path(configured: object) -> Path:
    value = str(configured or ".local/models/yolov8m.pt").strip()
    path = Path(value).expanduser()
    if path.is_absolute():
        return path
    if path.parts and path.parts[0] == ".local":
        return REPOSITORY_ROOT / path
    return APP_ROOT / path


def _model_diagnostic(
    model: Path,
    *,
    manifest_path: Path = MODEL_MANIFEST_PATH,
) -> Diagnostic:
    recovery = (
        "Copy the approved yolov8n_uav.pt bytes matching "
        "apps/StereoWIRIS/model-manifest.json into apps/StereoWIRIS/",
    )
    if not model.is_file():
        return Diagnostic(
            "FAIL",
            "stereowiris.model",
            f"YOLO model is missing: {model}",
            recovery=recovery,
        )
    if model.name != DEFAULT_MODEL_FILENAME:
        return Diagnostic("PASS", "stereowiris.model", f"custom YOLO model: {model}")
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        expected_filename = str(manifest["filename"])
        expected_size = int(manifest["size_bytes"])
        expected_sha256 = str(manifest["sha256"]).lower()
        if expected_filename != DEFAULT_MODEL_FILENAME:
            raise ValueError(
                f"filename must be {DEFAULT_MODEL_FILENAME}, got {expected_filename}"
            )
        if expected_size <= 0:
            raise ValueError("size_bytes must be positive")
        if len(expected_sha256) != 64:
            raise ValueError("sha256 must contain 64 hexadecimal characters")
        int(expected_sha256, 16)
    except (KeyError, OSError, TypeError, ValueError) as exc:
        return Diagnostic(
            "FAIL",
            "stereowiris.model",
            f"YOLO model manifest is invalid: {exc}",
            recovery=recovery,
        )
    actual_size = model.stat().st_size
    if actual_size != expected_size:
        return Diagnostic(
            "FAIL",
            "stereowiris.model",
            f"YOLO model size mismatch: expected={expected_size}, actual={actual_size}, path={model}",
            recovery=recovery,
        )
    digest = hashlib.sha256()
    with model.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    if digest.hexdigest() != expected_sha256:
        return Diagnostic(
            "FAIL",
            "stereowiris.model",
            f"YOLO model SHA-256 does not match model-manifest.json: {model}",
            recovery=recovery,
        )
    return Diagnostic(
        "PASS",
        "stereowiris.model",
        f"approved YOLO model: {model} ({actual_size} bytes, sha256={expected_sha256})",
    )


def _app_report(payload: dict[str, object], diagnostics: list[Diagnostic]) -> RuntimeReport:
    model = _model_path(payload.get("yolo_weights_path"))
    diagnostics.append(_model_diagnostic(model))
    for module_name in REQUIRED_APP_IMPORTS:
        try:
            importlib.import_module(module_name)
        except Exception as exc:  # noqa: BLE001 - report arbitrary application import failures.
            diagnostics.append(
                Diagnostic("FAIL", f"stereowiris.import.{module_name}", f"cannot import {module_name}: {exc}")
            )
        else:
            diagnostics.append(Diagnostic("PASS", f"stereowiris.import.{module_name}", "import ok"))
    return RuntimeReport(tuple(diagnostics))


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--app-only",
        action="store_true",
        help="Skip the interpreter/pip gate already performed by run.sh; still check the saved decoder.",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    payload, settings_diagnostics = _settings()
    decoder = normalize_decoder_mode(str(payload.get("rtsp_decoder", DECODER_AUTO)))
    reports = []
    if args.app_only:
        reports.append(probe_gstreamer(decoder))
    else:
        reports.append(check_execution(decoder, root=REPOSITORY_ROOT))
    reports.append(_app_report(payload, settings_diagnostics))
    report = combine_reports(*reports)
    print(render_report(report, title="StereoWIRIS application runtime"))
    if report.exit_code == 0:
        print("StereoWIRIS runtime check ok")
    else:
        print("StereoWIRIS runtime check failed", file=sys.stderr)
    return report.exit_code


if __name__ == "__main__":
    raise SystemExit(main())
