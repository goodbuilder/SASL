"""Two immutable XLSX views of already-frozen campaign analysis results.

This exporter never fits a pose, reads GT, selects a path, or computes a yaw.
The analyzer owns the authoritative JSON results and input evidence.
"""
from __future__ import annotations

import hashlib
import json
import math
import os
import tempfile
from datetime import datetime
from pathlib import Path

from .gps_yaw90 import GPS_YAW90_CASE


WORKBOOK_NAMES = {"existing_yaw": "existing_yaw_results.xlsx",
                  "gps_yaw90": "gps_yaw90_results.xlsx"}
IDENTITY = ("yaw_source", "calibration_run_id", "evaluation_run_id", "trial_index",
            "yaw_case", "data_scope", "method", "attitude_variant")


def _flatten(value, prefix=""):
    result = {}
    for key, item in value.items():
        name = f"{prefix}.{key}" if prefix else str(key)
        if isinstance(item, dict):
            result.update(_flatten(item, name))
        elif isinstance(item, (tuple, list)):
            result[name] = json.dumps(item, ensure_ascii=False, allow_nan=False)
        else:
            result[name] = item
    return result


def _digest(path):
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _workbook_records(root, payloads):
    for payload in payloads:
        family = "gps_yaw90" if payload["yaw_case"] == GPS_YAW90_CASE else "existing_yaw"
        identity = {key: payload.get(key) for key in IDENTITY[1:6]}
        identity["yaw_source"] = family
        for source, target in (("results", "Evaluation"), ("calibration_results", "Calibration")):
            rows = payload.get(source, [])
            for row in rows:
                yield family, target, {**identity, **_flatten(row)}
            if not rows:
                yield family, target, {**identity, "status": payload.get("evaluation_status", "NO_RESULTS")}
        case_directory = Path(payload["case_directory"])
        config_path = (root / case_directory / "trial_config.json").resolve()
        if (case_directory.is_absolute() or ".." in case_directory.parts
                or not config_path.is_relative_to(root.resolve()) or not config_path.is_file()):
            raise ValueError("workbook trial input evidence missing or outside analysis")
        config = json.loads(config_path.read_text(encoding="utf-8"))
        yield family, "Inputs", {**identity, **_flatten(config),
            "trial_config_path": config_path.relative_to(root.resolve()).as_posix(),
            "trial_config_sha256": _digest(config_path)}


def export_analysis_workbooks(analysis_root: Path, evaluation_payloads):
    """Two streaming passes over a list or a repeatable iterator factory.

    The first retains only column sets/counts/hashes; the second writes rows.
    One-shot iterators are rejected rather than exhausted or silently buffered.
    """
    from contextlib import ExitStack
    import xlsxwriter
    root = Path(analysis_root)
    if not (root / "analysis_freeze.json").is_file():
        raise RuntimeError("analysis must be frozen before exporting V0 workbooks")
    if callable(evaluation_payloads):
        factory = evaluation_payloads
    else:
        if iter(evaluation_payloads) is evaluation_payloads:
            raise ValueError("workbook export requires a repeatable collection or iterator factory")
        factory = lambda: iter(evaluation_payloads)
    names = ("Evaluation", "Calibration", "Inputs")
    stats = {(family, name): {"fields": set(), "count": 0, "digest": hashlib.sha256()}
             for family in WORKBOOK_NAMES for name in names}
    def encoded(row):
        return (json.dumps(row, sort_keys=True, ensure_ascii=False, allow_nan=False) + "\n").encode()
    for family, name, row in _workbook_records(root, factory()):
        stat = stats[family, name]
        stat["fields"].update(row)
        stat["count"] += 1
        stat["digest"].update(encoded(row))
    temporary_files = {}
    tables = {}
    artifacts = {}
    try:
        with ExitStack() as stack:
            books = {}
            for family in WORKBOOK_NAMES:
                fd, temporary_name = tempfile.mkstemp(prefix=f".{family}-", suffix=".xlsx", dir=root)
                os.close(fd)
                temporary_files[family] = Path(temporary_name)
                book = stack.enter_context(xlsxwriter.Workbook(temporary_name, {"constant_memory": True,
                    "strings_to_formulas": False, "strings_to_urls": False}))
                books[family] = book
                book.set_properties({"title": f"Stereo calibration: {family}", "created": datetime(1980, 1, 1)})
                header = book.add_format({"bold": True, "bg_color": "#DCE6F1"})
                for name in names:
                    stat = stats[family, name]
                    fields = [key for key in IDENTITY if key in stat["fields"]]
                    fields += sorted(stat["fields"] - set(fields))
                    fields = fields or ["status"]
                    sheets = []
                    for part, start in enumerate(range(0, max(1, stat["count"]), 1_048_575), 1):
                        sheet = book.add_worksheet(name if part == 1 else f"{name}_{part}")
                        sheet.freeze_panes(1, 0)
                        sheet.set_column(0, len(fields)-1, 22)
                        sheet.write_row(0, 0, fields, header)
                        count = min(1_048_575, max(1, stat["count"])-start)
                        sheet.autofilter(0, 0, count, len(fields)-1)
                        sheets.append(sheet)
                    if not stat["count"]:
                        sheets[0].write_row(1, 0, ["NOT_RUN"])
                    tables[family, name] = {"fields": fields, "sheets": sheets,
                                           "count": 0, "digest": hashlib.sha256()}
            for family, name, row in _workbook_records(root, factory()):
                table = tables[family, name]
                if table["count"] >= stats[family, name]["count"] or not set(row) <= set(table["fields"]):
                    raise ValueError("workbook source changed between passes")
                values = [row.get(key) for key in table["fields"]]
                for value in values:
                    if isinstance(value, float) and not math.isfinite(value):
                        raise ValueError("non-finite analysis value cannot be exported")
                    if isinstance(value, str) and len(value) > 32767:
                        raise ValueError("analysis evidence exceeds Excel cell limit")
                part, index = divmod(table["count"], 1_048_575)
                table["sheets"][part].write_row(index+1, 0, values)
                table["count"] += 1
                table["digest"].update(encoded(row))
            for key, table in tables.items():
                if (table["count"] != stats[key]["count"]
                        or table["digest"].digest() != stats[key]["digest"].digest()):
                    raise ValueError("workbook source changed between passes")
            for family, book in books.items():
                config = book.add_worksheet("Contract")
                notes = {
                    "yaw_source": family,
                    "position_axis_contract": "world ENU [east,north,up] metres; estimated minus truth",
                    "ray_axis_contract": "optical RDF [right,down,forward]; R_WC maps optical to ENU",
                    "angle_contract": "phi optical roll; theta correction to nominal 20deg; psi compass yaw; rad/rad2",
                    "calibration": "A/B/C refitted for this yaw source; RPY_IMU estimates R/P/Y; YAW_ONLY fixes measured R/P",
                    "truth_policy": "GT only in synthetic producers, explicitly ideal diagnostics and post-freeze evaluation",
                    "gps_yaw90_branch": "noisy camera-RTK baseline perpendicular; nominal north selects hemisphere only; not live Pixhawk yaw",
                    "path_selection": "existing-yaw G0 calibration selection shared; no V0-driven path selection",
                    "sensitivity_scope": "existing-yaw sensitivity study remains separate; not duplicated in these primary-result workbooks",
                    "freeze_sha256": _digest(root / "analysis_freeze.json"),
                    }
                config.set_column(0, 0, 25)
                config.set_column(1, 1, 100)
                for index, item in enumerate(notes.items()):
                    config.write_row(index, 0, item)
        for family, filename in WORKBOOK_NAMES.items():
            destination = root / filename
            temporary = temporary_files[family]
            if destination.exists():
                if _digest(destination) != _digest(temporary):
                    raise FileExistsError(f"workbook exists with different content: {destination}")
            else:
                os.link(temporary, destination)
            artifacts[family] = {"path": filename, "sha256": _digest(destination),
                "evaluation_rows": stats[family, "Evaluation"]["count"],
                "calibration_rows": stats[family, "Calibration"]["count"]}
        return artifacts
    finally:
        for temporary in temporary_files.values():
            temporary.unlink(missing_ok=True)
