"""Rebuild a Measurement from durable deltas and its type-owned raw streams.

No current settings, sensors or geometry solvers participate in a rebuild.
Corrupt/incomplete history stays untouched and is reported in the workbook.
"""

from __future__ import annotations

import csv
import json
from datetime import datetime
from pathlib import Path
from typing import Any, Iterator

from Workstation.workstation_core.measurement_data_store import STREAM_PATHS
from Workstation.workstation_core.measurement_record import (
    DetectionRow, EventRow, MeasurementRecord, MethodResultRow,
    RECORD_STATE_STOPPED, telemetry_sheet_for,
)


def _rows(path: Path, errors: list[str]) -> Iterator[dict[str, Any]]:
    if not path.exists():
        path = path.with_suffix(".csv")
    if not path.exists():
        return
    if path.is_symlink():
        errors.append(f"refused symlink: {path.name}")
        return
    if path.suffix == ".csv":
        csv.field_size_limit(max(csv.field_size_limit(), 64 * 1024 * 1024))
        with path.open(encoding="utf-8", newline="") as handle:
            try:
                for index, row in enumerate(csv.DictReader(handle), 2):
                    try:
                        value = json.loads(row["record_json"])
                        if isinstance(value, dict):
                            yield value
                    except (KeyError, TypeError, ValueError) as exc:
                        errors.append(f"{path.name}:{index}: {exc}")
            except csv.Error as exc:
                errors.append(f"{path.name}: {exc}")
        return
    with path.open(encoding="utf-8", errors="replace") as handle:
        for index, line in enumerate(handle, 1):
            try:
                if not line.endswith("\n"):
                    raise ValueError("incomplete final record")
                value = json.loads(line)
                if isinstance(value, dict):
                    yield value
                else:
                    raise ValueError("expected a JSON object")
            except (ValueError, TypeError) as exc:
                errors.append(f"{path.name}:{index}: {exc}")


def load_measurement_record(journal_path: Path) -> MeasurementRecord:
    path = Path(journal_path)
    root = path.parent.parent
    errors: list[str] = []
    record: MeasurementRecord | None = None
    last_sequence = 0
    gaps = 0
    for item in _rows(path, errors):
        sequence = int(item.get("journal_sequence") or 0)
        if sequence <= last_sequence:
            continue  # idempotent replay of an append retried after a partial failure
        gaps += max(0, sequence - last_sequence - 1)
        last_sequence = sequence
        kind = item.get("kind")
        payload = item.get("payload") or {}
        if kind == "start":
            if record is not None:
                raise ValueError("journal contains more than one Measurement start")
            prefix = str(payload["file_prefix"])
            if prefix != root.name:
                raise ValueError("journal prefix does not match its run directory")
            record = MeasurementRecord(
                measurement_id=str(item["measurement_id"]), file_prefix=prefix,
                workbook_path=root / f"{prefix}_measurement.xlsx",
                started_local=datetime.fromisoformat(payload["started_local"]),
                started_utc_ns=int(payload["started_utc_ns"]),
                started_monotonic_ns=int(payload["started_monotonic_ns"]),
                metadata=payload.get("metadata"), calibration=payload.get("calibration"),
            )
            continue
        if record is None:
            raise ValueError("Measurement start record is missing; reconstruction cannot invent inputs")
        if item.get("measurement_id") != record.measurement_id:
            errors.append("foreign Measurement in journal")
            continue
        if kind == "pair":
            record.register_pair(
                payload["snapshot_payload"],
                submitted_method_ids=tuple(payload.get("submitted_method_ids", ())),
                not_attempted=payload.get("not_attempted", {}),
                method_evidence=payload.get("method_evidence"),
                submitted_monotonic_ns=payload.get("submitted_monotonic_ns"),
            )
        elif kind == "outcome":
            record.note_outcome(payload)
        elif kind == "detections":
            for raw in payload.get("rows", ()):
                row = DetectionRow(**raw)
                if row.detection_id not in record._detection_index:
                    record._detections[row.camera].append(row)
                    record._detection_index[row.detection_id] = row
        elif kind == "calibration":
            record.set_calibration(payload)
        elif kind == "alignment":
            record.add_alignment_row(payload)
        elif kind == "event":
            record.add_event(**payload)
        elif kind == "stop":
            record.stop(
                stopped_local=datetime.fromisoformat(payload["stopped_local"]),
                stopped_utc_ns=int(payload["stopped_utc_ns"]),
                stopped_monotonic_ns=int(payload["stopped_monotonic_ns"]),
            )
        elif kind == "fail_pending":
            for raw in payload.get("rows", ()):
                row = MethodResultRow(**raw)
                record._method_results[(row.pair_id, row.method_id)] = row
            if payload.get("event"):
                record._events.append(EventRow(**payload["event"]))
            record.refresh_state()

    if record is None:
        raise ValueError("Measurement start record is missing")
    # The main raw streams already preserve all consumed telemetry/artifacts.
    # Reading them here avoids recording another duplicate telemetry journal.
    for stream, relative in STREAM_PATHS.items():
        if not (stream.startswith("px4.") or stream.startswith("gnss.") or stream in {"alignment", "artifacts"}):
            continue
        source = root / "raw" / f"{record.file_prefix}_{relative.stem}.jsonl"
        for row in _rows(source, errors):
            if str(row.get("measurement_id") or row.get("runtime_generation") or "") != record.measurement_id:
                continue
            received = row.get("recorded_monotonic_ns")
            if telemetry_sheet_for(stream, row) and received is not None and record.stopped_monotonic_ns is not None:
                if int(received) > record.stopped_monotonic_ns:
                    continue
            record.observe_store_record(stream, row)
    # Pair association also works when a detection delta arrived after its pair.
    for pair in record._pairs.values():
        for identifier in (pair.left_detection_id, pair.right_detection_id):
            detection = record._detection_index.get(identifier)
            if detection is not None:
                detection.pair_id, detection.pair_sequence = pair.pair_id, pair.pair_sequence
    if record.running or record.pending_method_count():
        record.state = "INTERRUPTED"
        errors.append("Measurement has no complete stop or still has pending method results")
    else:
        record.state = RECORD_STATE_STOPPED
    record._storage.update({
        "raw_read_errors": len(errors), "journal_missing_records": gaps,
        "state": "DEGRADED" if errors or gaps else "READY",
        "last_error": "; ".join(errors[:10]),
    })
    if errors or gaps:
        record.add_event("RAW_REBUILD_INCOMPLETE", detail=f"missing journal records={gaps}; " + "; ".join(errors[:10]))
    return record
