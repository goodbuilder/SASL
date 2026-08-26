#!/usr/bin/env python3
"""Read-only application-specific runtime check for StereoWIRIS."""

from __future__ import annotations

import argparse
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


def _app_report(payload: dict[str, object], diagnostics: list[Diagnostic]) -> RuntimeReport:
    model = _model_path(payload.get("yolo_weights_path"))
    if model.is_file():
        diagnostics.append(Diagnostic("PASS", "stereowiris.model", f"YOLO model: {model}"))
    else:
        diagnostics.append(
            Diagnostic(
                "FAIL",
                "stereowiris.model",
                f"YOLO model is missing: {model}",
                recovery=("Set yolo_weights_path to an existing model in stereo_distance_settings.json",),
            )
        )
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
