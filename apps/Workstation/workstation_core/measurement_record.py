"""Per-Measurement in-memory record and the registry that routes evidence.

One ``MeasurementRecord`` covers exactly the interval between the operator's
Measurement Start and Stop: every detection candidate, every stereo pair,
every method result (attempted or not), the applied calibration/alignment,
every consumed telemetry sample, and lifecycle/storage events. The record
is Qt-free and thread-safe; a workbook writer serialises ``snapshot()``.

The ``MeasurementRegistry`` owns the current record plus any stopped records
that still have method jobs in flight, and routes late outcomes by
``measurement_id`` so a result can never land in the wrong Measurement.
Neither object ever gates Measurement Start/Stop: allocation failures are
recorded as events and the Measurement continues.
"""

from __future__ import annotations

import re
import threading
import time
from collections.abc import Callable, Mapping
from dataclasses import asdict, dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any

from Workstation.workstation_core.measurement_methods import MEASUREMENT_METHOD_IDS
from Workstation.workstation_core.method_result_evidence import derive_method_result
from Workstation.workstation_core.measurement_artifacts import (
    KST, MeasurementStorageContext, reserve_measurement_context,
)

STATUS_NOT_ATTEMPTED = "N/A"
STATUS_PENDING = "PENDING"
RECORD_STATE_RUNNING = "RUNNING"
RECORD_STATE_STOPPED_DRAINING = "STOPPED_DRAINING"
RECORD_STATE_STOPPED = "STOPPED"

METHOD_SHEET_NAMES: Mapping[str, str] = {
    "pixhawk_rtk_live": "10_Pixhawk_RTK",
    "checkerboard_stereo": "11_Checkerboard",
    "horizon_hybrid": "12_Horizon",
    "mountain_gnss": "13_Mountain_GNSS",
    "drone_rtk_rotation_hybrid": "14_Drone_RTK_Rot",
    "drone_rtk_multiview": "15_Drone_RTK_MV",
    "terrain_pnp": "16_Terrain_PnP",
    "combined_hybrid": "17_Combined",
    "pixhawk_rp_gps_baseline_yaw": "18_Pixhawk_RP_GPS_Yaw",
    "cloud_gnss": "19_Cloud_GNSS",
}

_PX4_GROUP_BY_SAMPLE_TYPE: Mapping[str, str] = {
    "pixhawk_attitude": "Attitude",
    "gps_rtk": "GNSS_RTK",
    "sensor_gps": "GNSS_RTK",
    "gnss_relative": "GNSS_RTK",
    "magnetometer": "IMU_Mag",
    "mavlink_raw_imu": "IMU_Mag",
    "mag_slot_status": "IMU_Mag",
    "mavlink_local_position_ned": "Position",
    "mavlink_global_position_int": "Position",
    "mavlink_vfr_hud": "Position",
    "mavlink_sys_status": "System",
    "mavlink_battery_status": "System",
    "mavlink_radio_status": "System",
}
_PX4_SHEET_PREFIX: Mapping[str, Mapping[str, str]] = {
    "left": {
        "Attitude": "20_PX4L_Attitude",
        "GNSS_RTK": "21_PX4L_GNSS_RTK",
        "IMU_Mag": "22_PX4L_IMU_Mag",
        "Position": "23_PX4L_Position",
        "System": "24_PX4L_System",
        "Other": "25_PX4L_Other",
    },
    "right": {
        "Attitude": "30_PX4R_Attitude",
        "GNSS_RTK": "31_PX4R_GNSS_RTK",
        "IMU_Mag": "32_PX4R_IMU_Mag",
        "Position": "33_PX4R_Position",
        "System": "34_PX4R_System",
        "Other": "35_PX4R_Other",
    },
    "drone": {
        "Attitude": "40_Drone_Attitude",
        "GNSS_RTK": "41_Drone_GNSS_RTK",
        "Position": "42_Drone_Position",
        "IMU_Mag": "44_Drone_IMU_Mag",
        "System": "45_Drone_System",
        "Other": "48_Drone_Other",
    },
}
BASE_GNSS_SHEET = "43_Base_GNSS"
GNSS_BASELINE_CENTER_SHEET = "46_GNSS_Baseline_Center"
ALWAYS_PRESENT_TELEMETRY_SHEETS: tuple[str, ...] = (
    "20_PX4L_Attitude",
    "21_PX4L_GNSS_RTK",
    "22_PX4L_IMU_Mag",
    "23_PX4L_Position",
    "24_PX4L_System",
    "30_PX4R_Attitude",
    "31_PX4R_GNSS_RTK",
    "32_PX4R_IMU_Mag",
    "33_PX4R_Position",
    "34_PX4R_System",
    "40_Drone_Attitude",
    "41_Drone_GNSS_RTK",
    "42_Drone_Position",
    BASE_GNSS_SHEET,
)

_SAFE_PREFIX = re.compile(r"[^0-9_-]+")


def telemetry_sheet_for(stream: str, payload: Mapping[str, Any]) -> str | None:
    """Map one storage stream record to its workbook telemetry sheet."""

    if stream.startswith("px4."):
        side = stream.split(".", 1)[1]
        prefixes = _PX4_SHEET_PREFIX.get(side)
        if prefixes is None:
            return None
        group = _PX4_GROUP_BY_SAMPLE_TYPE.get(str(payload.get("sample_type") or ""), "Other")
        return prefixes[group]
    if stream == "gnss.base":
        return BASE_GNSS_SHEET
    if stream == "gnss.baseline.center":
        return GNSS_BASELINE_CENTER_SHEET
    return None


def detection_id_for(camera: str, metadata: Mapping[str, Any], index: int) -> str:
    frame = metadata.get("frame_key") or metadata.get("app_receive_monotonic_ns")
    return f"{str(camera)[:1].upper()}:{frame}:{int(index)}"


def measurement_file_prefix(started_local: datetime) -> str:
    return started_local.strftime("%Y%m%d_%H%M%S")


def allocate_measurement_workbook_path(root: Path, prefix: str, *, reserved: set[str]) -> Path:
    """First free ``<prefix>[-NN]_measurement.xlsx`` under ``root``; never overwrites."""

    base = Path(root)
    base.mkdir(parents=True, exist_ok=True)
    safe_prefix = _SAFE_PREFIX.sub("", prefix) or "measurement"
    candidate_prefix = safe_prefix
    sequence = 0
    while True:
        candidate = base / f"{candidate_prefix}_measurement.xlsx"
        if candidate_prefix not in reserved and not candidate.exists() and not candidate.is_symlink():
            reserved.add(candidate_prefix)
            return candidate
        sequence += 1
        candidate_prefix = f"{safe_prefix}-{sequence:02d}"


@dataclass
class DetectionRow:
    camera: str
    detection_id: str
    index: int
    selected: bool
    frame_id: object
    frame_key: object
    timestamp_s: float
    app_receive_monotonic_ns: int | None
    x1: float
    y1: float
    x2: float
    y2: float
    confidence: float
    class_id: int
    class_name: str | None
    image_width: int
    image_height: int
    stream_generation: object
    rtcp_frame_ntp_unix_s: object
    pair_id: str = ""
    pair_sequence: int | None = None


@dataclass
class PairRow:
    pair_id: str
    pair_sequence: int
    created_unix_ns: int
    created_monotonic_ns: int
    timestamp_basis: str
    time_delta_s: float | None
    left_detection_id: str
    right_detection_id: str
    left_detection: dict[str, Any]
    right_detection: dict[str, Any]
    capture_sync: dict[str, Any]


@dataclass
class MethodResultRow:
    pair_id: str
    pair_sequence: int
    method_id: str
    attempted: bool
    status: str
    reason: str
    evidence: dict[str, Any] = field(default_factory=dict)
    outcome: dict[str, Any] | None = None
    derived: dict[str, Any] | None = None
    submitted_monotonic_ns: int | None = None


@dataclass
class EventRow:
    event_type: str
    utc_ns: int
    monotonic_ns: int
    detail: str = ""
    payload: dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class MeasurementSnapshot:
    """Immutable copy handed to the workbook writer."""

    measurement_id: str
    file_prefix: str
    workbook_path: Path | None
    state: str
    revision: int
    started_local_iso: str
    started_utc_ns: int
    started_monotonic_ns: int
    stopped_local_iso: str
    stopped_utc_ns: int | None
    stopped_monotonic_ns: int | None
    duration_s: float | None
    metadata: dict[str, Any]
    calibration: dict[str, Any]
    alignment_rows: tuple[dict[str, Any], ...]
    detections: dict[str, tuple[DetectionRow, ...]]
    pairs: tuple[PairRow, ...]
    method_results: tuple[MethodResultRow, ...]
    telemetry: dict[str, tuple[dict[str, Any], ...]]
    events: tuple[EventRow, ...]
    artifacts: tuple[dict[str, Any], ...]
    storage: dict[str, Any]

    def method_results_for(self, method_id: str) -> tuple[MethodResultRow, ...]:
        return tuple(row for row in self.method_results if row.method_id == method_id)


class MeasurementRecord:
    def __init__(
        self,
        *,
        measurement_id: str,
        file_prefix: str,
        workbook_path: Path | None,
        started_local: datetime,
        started_utc_ns: int,
        started_monotonic_ns: int,
        metadata: Mapping[str, Any] | None = None,
        calibration: Mapping[str, Any] | None = None,
        journal_sink: Callable[[Mapping[str, Any]], bool] | None = None,
    ) -> None:
        self.measurement_id = str(measurement_id)
        self.file_prefix = str(file_prefix)
        self.workbook_path = workbook_path
        self.started_local = started_local
        self.started_utc_ns = int(started_utc_ns)
        self.started_monotonic_ns = int(started_monotonic_ns)
        self.stopped_local: datetime | None = None
        self.stopped_utc_ns: int | None = None
        self.stopped_monotonic_ns: int | None = None
        self.state = RECORD_STATE_RUNNING
        self.selected_method_id = ""
        self.metadata: dict[str, Any] = dict(metadata or {})
        self.calibration: dict[str, Any] = dict(calibration or {})
        self._journal_sink = journal_sink
        self._journal_sequence = 0
        self.counters: dict[str, dict[str, int]] = {
            method_id: {
                "VALID": 0,
                "ESTIMATED": 0,
                "BLOCKED": 0,
                "FAILED": 0,
                "BACKPRESSURE_BLOCKED": 0,
                STATUS_NOT_ATTEMPTED: 0,
            }
            for method_id in MEASUREMENT_METHOD_IDS
        }
        self.submitted_pairs: set[tuple[str, int]] = set()
        self._lock = threading.Lock()
        self._revision = 0
        self._written_revision = -1
        self._alignment_rows: list[dict[str, Any]] = []
        self._detections: dict[str, list[DetectionRow]] = {"left": [], "right": []}
        self._detection_index: dict[str, DetectionRow] = {}
        self._pairs: dict[str, PairRow] = {}
        self._method_results: dict[tuple[str, str], MethodResultRow] = {}
        self._comparison_participants: set[str] = set()
        self._comparison_latest_pair_id = ""
        self._telemetry: dict[str, list[dict[str, Any]]] = {}
        self._events: list[EventRow] = []
        self._artifacts: list[dict[str, Any]] = []
        self._storage: dict[str, Any] = {
            "state": "NOT_WRITTEN",
            "last_write_utc_ns": None,
            "last_error": "",
            "write_count": 0,
            "failure_count": 0,
            "observer_errors": 0,
        }

    # -- identity -------------------------------------------------------

    @property
    def revision(self) -> int:
        with self._lock:
            return self._revision

    @property
    def running(self) -> bool:
        return self.state == RECORD_STATE_RUNNING

    @property
    def stopped(self) -> bool:
        return self.state != RECORD_STATE_RUNNING

    def _bump(self) -> None:
        self._revision += 1

    def _journal(self, kind: str, payload: Mapping[str, Any]) -> None:
        if self._journal_sink is None:
            return
        self._journal_sequence += 1
        try:
            accepted = self._journal_sink({
                "schema": "workstation-measurement-journal-v1",
                "measurement_id": self.measurement_id,
                "journal_sequence": self._journal_sequence,
                "kind": kind,
                "payload": dict(payload),
            })
        except Exception:
            accepted = False
        if not accepted:
            self._storage["journal_dropped_records"] = int(self._storage.get("journal_dropped_records", 0)) + 1

    # -- lifecycle ------------------------------------------------------

    def stop(self, *, stopped_local: datetime, stopped_utc_ns: int, stopped_monotonic_ns: int) -> None:
        with self._lock:
            if self.state != RECORD_STATE_RUNNING:
                return
            self.stopped_local = stopped_local
            self.stopped_utc_ns = int(stopped_utc_ns)
            self.stopped_monotonic_ns = int(stopped_monotonic_ns)
            self.state = (
                RECORD_STATE_STOPPED_DRAINING if self._pending_locked() else RECORD_STATE_STOPPED
            )
            self._events.append(
                EventRow(
                    event_type="MEASUREMENT_STOPPED",
                    utc_ns=self.stopped_utc_ns,
                    monotonic_ns=self.stopped_monotonic_ns,
                    detail=f"pending method jobs: {self._pending_locked()}",
                )
            )
            self._bump()

            self._journal("stop", {"stopped_local": stopped_local.isoformat(), "stopped_utc_ns": stopped_utc_ns, "stopped_monotonic_ns": stopped_monotonic_ns})

    def _pending_locked(self) -> int:
        return sum(
            1 for row in self._method_results.values() if row.attempted and row.status == STATUS_PENDING
        )

    def pending_method_count(self) -> int:
        with self._lock:
            return self._pending_locked()

    def pending_method_keys(self) -> tuple[tuple[str, str], ...]:
        with self._lock:
            return tuple(
                key
                for key, row in self._method_results.items()
                if row.attempted and row.status == STATUS_PENDING
            )

    def refresh_state(self) -> str:
        with self._lock:
            if self.state != RECORD_STATE_RUNNING:
                self.state = (
                    RECORD_STATE_STOPPED_DRAINING if self._pending_locked() else RECORD_STATE_STOPPED
                )
            return self.state

    def add_event(
        self,
        event_type: str,
        *,
        detail: str = "",
        payload: Mapping[str, Any] | None = None,
        utc_ns: int | None = None,
        monotonic_ns: int | None = None,
    ) -> None:
        with self._lock:
            self._events.append(
                EventRow(
                    event_type=str(event_type),
                    utc_ns=int(time.time_ns() if utc_ns is None else utc_ns),
                    monotonic_ns=int(time.monotonic_ns() if monotonic_ns is None else monotonic_ns),
                    detail=str(detail),
                    payload=dict(payload or {}),
                )
            )
            self._bump()
            self._journal("event", asdict(self._events[-1]))

    # -- calibration / alignment ---------------------------------------

    def set_calibration(self, calibration: Mapping[str, Any]) -> None:
        with self._lock:
            self.calibration = dict(calibration)
            self._bump()
            self._journal("calibration", self.calibration)

    def add_alignment_row(self, row: Mapping[str, Any]) -> None:
        with self._lock:
            self._alignment_rows.append(dict(row))
            self._bump()
            self._journal("alignment", row)

    # -- detections / pairs --------------------------------------------

    def observe_detection(self, camera: str, observation: Any) -> None:
        camera = str(camera)
        if camera not in self._detections or observation is None:
            return
        metadata = dict(getattr(observation, "timestamp_metadata", {}) or {})
        candidates = tuple(getattr(observation, "candidates", ()) or ())
        selected_index = int(getattr(observation, "selected_candidate_index", 0) or 0)
        app_ns = metadata.get("app_receive_monotonic_ns")
        rows: list[DetectionRow] = []
        if candidates:
            for candidate in candidates:
                rows.append(
                    DetectionRow(
                        camera=camera,
                        detection_id=detection_id_for(camera, metadata, candidate.index),
                        index=int(candidate.index),
                        selected=int(candidate.index) == selected_index,
                        frame_id=metadata.get("frame_id"),
                        frame_key=metadata.get("frame_key"),
                        timestamp_s=float(observation.timestamp_s),
                        app_receive_monotonic_ns=None if app_ns is None else int(app_ns),
                        x1=float(candidate.x1),
                        y1=float(candidate.y1),
                        x2=float(candidate.x2),
                        y2=float(candidate.y2),
                        confidence=float(candidate.confidence),
                        class_id=int(candidate.class_id),
                        class_name=candidate.class_name,
                        image_width=int(observation.width),
                        image_height=int(observation.height),
                        stream_generation=metadata.get("stream_generation"),
                        rtcp_frame_ntp_unix_s=metadata.get("rtcp_frame_ntp_unix_s"),
                    )
                )
        else:
            rows.append(
                DetectionRow(
                    camera=camera,
                    detection_id=detection_id_for(camera, metadata, selected_index),
                    index=selected_index,
                    selected=True,
                    frame_id=metadata.get("frame_id"),
                    frame_key=metadata.get("frame_key"),
                    timestamp_s=float(observation.timestamp_s),
                    app_receive_monotonic_ns=None if app_ns is None else int(app_ns),
                    x1=float(observation.x1),
                    y1=float(observation.y1),
                    x2=float(observation.x2),
                    y2=float(observation.y2),
                    confidence=float(observation.confidence),
                    class_id=int(observation.class_id),
                    class_name=getattr(observation, "class_name", None),
                    image_width=int(observation.width),
                    image_height=int(observation.height),
                    stream_generation=metadata.get("stream_generation"),
                    rtcp_frame_ntp_unix_s=metadata.get("rtcp_frame_ntp_unix_s"),
                )
            )
        with self._lock:
            added = []
            for row in rows:
                if row.detection_id in self._detection_index:
                    continue
                self._detections[camera].append(row)
                self._detection_index[row.detection_id] = row
                added.append(asdict(row))
            self._bump()
            if added:
                self._journal("detections", {"camera": camera, "rows": added})

    def register_pair(
        self,
        snapshot_payload: Mapping[str, Any],
        *,
        submitted_method_ids: tuple[str, ...],
        not_attempted: Mapping[str, str],
        method_evidence: Mapping[str, Mapping[str, Any]] | None = None,
        submitted_monotonic_ns: int | None = None,
    ) -> PairRow:
        pair_id = str(snapshot_payload.get("pair_id") or "")
        pair_sequence = int(snapshot_payload.get("pair_sequence") or 0)
        left = dict(snapshot_payload.get("left_detection") or {})
        right = dict(snapshot_payload.get("right_detection") or {})
        capture_sync = dict(snapshot_payload.get("capture_sync") or {})
        time_delta_s = capture_sync.get("pair_delta_s")
        try:
            time_delta_s = None if time_delta_s is None else abs(float(time_delta_s))
        except (TypeError, ValueError):
            time_delta_s = None
        left_id = _snapshot_detection_id("left", left)
        right_id = _snapshot_detection_id("right", right)
        row = PairRow(
            pair_id=pair_id,
            pair_sequence=pair_sequence,
            created_unix_ns=int(snapshot_payload.get("created_unix_ns") or 0),
            created_monotonic_ns=int(snapshot_payload.get("created_monotonic_ns") or 0),
            timestamp_basis=str(snapshot_payload.get("timestamp_basis") or ""),
            time_delta_s=time_delta_s,
            left_detection_id=left_id,
            right_detection_id=right_id,
            left_detection=left,
            right_detection=right,
            capture_sync=capture_sync,
        )
        evidence = dict(method_evidence or {})
        now_ns = int(time.monotonic_ns() if submitted_monotonic_ns is None else submitted_monotonic_ns)
        with self._lock:
            self._pairs[pair_id] = row
            previous = self._pairs.get(self._comparison_latest_pair_id)
            if previous is None or pair_sequence >= previous.pair_sequence:
                self._comparison_latest_pair_id = pair_id
            self._comparison_participants.update(submitted_method_ids)
            self._comparison_participants.update(k for k, v in evidence.items() if v.get("ready"))
            for detection_id in (left_id, right_id):
                detection = self._detection_index.get(detection_id)
                if detection is not None:
                    detection.pair_id = pair_id
                    detection.pair_sequence = pair_sequence
            for method_id in MEASUREMENT_METHOD_IDS:
                key = (pair_id, method_id)
                if key in self._method_results:
                    continue
                if method_id in submitted_method_ids:
                    self._method_results[key] = MethodResultRow(
                        pair_id=pair_id,
                        pair_sequence=pair_sequence,
                        method_id=method_id,
                        attempted=True,
                        status=STATUS_PENDING,
                        reason="",
                        evidence=dict(evidence.get(method_id) or {}),
                        submitted_monotonic_ns=now_ns,
                    )
                else:
                    self._method_results[key] = MethodResultRow(
                        pair_id=pair_id,
                        pair_sequence=pair_sequence,
                        method_id=method_id,
                        attempted=False,
                        status=STATUS_NOT_ATTEMPTED,
                        reason=str(not_attempted.get(method_id) or "not ready"),
                        evidence=dict(evidence.get(method_id) or {}),
                    )
                    self.counters[method_id][STATUS_NOT_ATTEMPTED] += 1
            self._bump()
            self._journal("pair", {
                "snapshot_payload": dict(snapshot_payload),
                "submitted_method_ids": list(submitted_method_ids),
                "not_attempted": dict(not_attempted),
                "method_evidence": evidence,
                "submitted_monotonic_ns": now_ns,
            })
        return row

    def note_outcome(self, outcome_record: Mapping[str, Any]) -> MethodResultRow | None:
        pair_id = str(outcome_record.get("pair_id") or "")
        method_id = str(outcome_record.get("method_id") or "")
        status = str(outcome_record.get("status") or "FAILED")
        with self._lock:
            key = (pair_id, method_id)
            row = self._method_results.get(key)
            if row is None:
                row = MethodResultRow(
                    pair_id=pair_id,
                    pair_sequence=int(outcome_record.get("pair_sequence") or 0),
                    method_id=method_id,
                    attempted=True,
                    status=STATUS_PENDING,
                    reason="",
                )
                self._method_results[key] = row
            if row.outcome is not None:
                return row
            row.attempted = True
            row.outcome = dict(outcome_record)
            try:
                frozen = outcome_record.get("recorded_geometry")
                row.derived = dict(frozen) if isinstance(frozen, Mapping) else derive_method_result(outcome_record)
                # Capture once from this immutable attempt, before persistence.
                # Both workbook views and future rebuilds use these same values.
                row.outcome["recorded_geometry"] = row.derived
            except Exception as exc:  # noqa: BLE001 - evidence derivation must never lose the outcome.
                row.derived = None
                row.reason = f"evidence derivation failed: {exc}"
            row.status = status
            reasons = outcome_record.get("reasons") or ()
            if reasons:
                row.reason = "; ".join(str(item) for item in reasons)
            counters = self.counters.setdefault(method_id, {})
            counters[status] = counters.get(status, 0) + 1
            if self.state != RECORD_STATE_RUNNING and not self._pending_locked():
                self.state = RECORD_STATE_STOPPED
            self._bump()
            self._journal("outcome", row.outcome)
            return row

    def fail_pending(self, reason: str) -> int:
        failed = 0
        now_ns = time.monotonic_ns()
        with self._lock:
            for row in self._method_results.values():
                if row.attempted and row.status == STATUS_PENDING:
                    row.status = "FAILED"
                    row.reason = str(reason)
                    row.outcome = {
                        "status": "FAILED",
                        "reasons": [str(reason)],
                        "completed_monotonic_ns": now_ns,
                    }
                    self.counters[row.method_id]["FAILED"] += 1
                    failed += 1
            if failed and self.state != RECORD_STATE_RUNNING:
                self.state = RECORD_STATE_STOPPED
            if failed:
                self._events.append(
                    EventRow(
                        event_type="METHOD_FAILED",
                        utc_ns=time.time_ns(),
                        monotonic_ns=now_ns,
                        detail=f"{failed} pending method job(s): {reason}",
                    )
                )
                self._bump()
                self._journal("fail_pending", {
                    "rows": [asdict(row) for row in self._method_results.values() if row.outcome and row.outcome.get("completed_monotonic_ns") == now_ns],
                    "event": asdict(self._events[-1]),
                })
        return failed

    # -- telemetry / store observation ----------------------------------

    def observe_store_record(self, stream: str, payload: Mapping[str, Any]) -> bool:
        sheet = telemetry_sheet_for(str(stream), payload)
        if sheet is not None:
            with self._lock:
                self._telemetry.setdefault(sheet, []).append(dict(payload))
                self._bump()
            return True
        if stream == "alignment":
            with self._lock:
                self._alignment_rows.append(dict(payload))
                self._bump()
            return True
        if stream == "artifacts":
            with self._lock:
                self._artifacts.append(dict(payload))
                self._bump()
            return True
        return False

    def note_storage_result(self, *, ok: bool, error: str = "", utc_ns: int | None = None) -> None:
        with self._lock:
            previous_state = self._storage["state"]
            if ok:
                self._storage["state"] = "READY"
                self._storage["last_write_utc_ns"] = int(time.time_ns() if utc_ns is None else utc_ns)
                self._storage["last_error"] = ""
                self._storage["write_count"] += 1
                if previous_state == "FAILED":
                    self._events.append(
                        EventRow(
                            event_type="STORAGE_RECOVERED",
                            utc_ns=time.time_ns(),
                            monotonic_ns=time.monotonic_ns(),
                            detail=str(self.workbook_path or ""),
                        )
                    )
                    self._bump()
            else:
                self._storage["state"] = "FAILED"
                self._storage["last_error"] = str(error)
                self._storage["failure_count"] += 1
                self._events.append(
                    EventRow(
                        event_type="STORAGE_FAILURE",
                        utc_ns=time.time_ns(),
                        monotonic_ns=time.monotonic_ns(),
                        detail=str(error),
                    )
                )
                self._bump()

    def note_written_revision(self, revision: int) -> None:
        with self._lock:
            self._written_revision = max(self._written_revision, int(revision))

    @property
    def written_revision(self) -> int:
        with self._lock:
            return self._written_revision

    def storage_snapshot(self) -> dict[str, Any]:
        with self._lock:
            return dict(self._storage)

    # -- snapshot -------------------------------------------------------

    def results_snapshot(self, pair_id: str | None = None) -> dict[str, Any]:
        """Bounded read-only copy for the live Method comparison (no geometry or I/O)."""
        import copy
        with self._lock:
            latest = self._comparison_latest_pair_id
            pair = self._pairs.get(pair_id or latest)
            rows = []
            for method_id in MEASUREMENT_METHOD_IDS:
                if method_id not in self._comparison_participants:
                    continue
                row = None if pair is None else self._method_results.get((pair.pair_id, method_id))
                if row is None:
                    rows.append({"method_id": method_id, "status": "N/A", "reason": "당시 미참여"})
                    continue
                rows.append({"method_id": method_id, "status": row.status if row.attempted else "BLOCKED",
                             "reason": row.reason, "attempted": row.attempted,
                             "derived": copy.deepcopy(row.derived or {}),
                             "outcome": copy.deepcopy(row.outcome or {}),
                             "evidence": copy.deepcopy(row.evidence)})
            return {"schema": "measurement-results-view-v1", "measurement_id": self.measurement_id,
                    "run_state": self.state, "revision": self._revision,
                    "latest_pair_id": latest, "pair_id": "" if pair is None else pair.pair_id,
                    "pair_sequence": None if pair is None else pair.pair_sequence,
                    "created_unix_ns": None if pair is None else pair.created_unix_ns,
                    "created_monotonic_ns": None if pair is None else pair.created_monotonic_ns,
                    "rows": rows, "all_terminal": bool(rows) and all(r["status"] != STATUS_PENDING for r in rows)}

    def snapshot(self) -> MeasurementSnapshot:
        with self._lock:
            stopped_local_iso = "" if self.stopped_local is None else self.stopped_local.isoformat()
            duration_s = (
                None
                if self.stopped_monotonic_ns is None
                else (self.stopped_monotonic_ns - self.started_monotonic_ns) * 1e-9
            )
            return MeasurementSnapshot(
                measurement_id=self.measurement_id,
                file_prefix=self.file_prefix,
                workbook_path=self.workbook_path,
                state=self.state,
                revision=self._revision,
                started_local_iso=self.started_local.isoformat(),
                started_utc_ns=self.started_utc_ns,
                started_monotonic_ns=self.started_monotonic_ns,
                stopped_local_iso=stopped_local_iso,
                stopped_utc_ns=self.stopped_utc_ns,
                stopped_monotonic_ns=self.stopped_monotonic_ns,
                duration_s=duration_s,
                metadata=dict(self.metadata),
                calibration=dict(self.calibration),
                alignment_rows=tuple(dict(row) for row in self._alignment_rows),
                detections={
                    camera: tuple(DetectionRow(**row.__dict__) for row in rows)
                    for camera, rows in self._detections.items()
                },
                pairs=tuple(self._pairs[pair_id] for pair_id in sorted(self._pairs, key=lambda key: self._pairs[key].pair_sequence)),
                method_results=tuple(
                    MethodResultRow(**row.__dict__)
                    for row in sorted(
                        self._method_results.values(),
                        key=lambda item: (item.pair_sequence, MEASUREMENT_METHOD_IDS.index(item.method_id) if item.method_id in MEASUREMENT_METHOD_IDS else 99),
                    )
                ),
                telemetry={sheet: tuple(rows) for sheet, rows in self._telemetry.items()},
                events=tuple(self._events),
                artifacts=tuple(dict(row) for row in self._artifacts),
                storage=dict(self._storage),
            )


def _snapshot_detection_id(camera: str, detection: Mapping[str, Any]) -> str:
    items = detection.get("timestamp_metadata_items") or ()
    metadata: dict[str, Any] = {}
    for item in items:
        if isinstance(item, (list, tuple)) and len(item) == 2:
            metadata[str(item[0])] = item[1]
    index = int(detection.get("selected_candidate_index") or 0)
    return detection_id_for(camera, metadata, index)


class MeasurementRegistry:
    """Owns the running record and stopped records that still drain."""

    def __init__(
        self,
        *,
        data_root_provider: Callable[[], Path],
        writer: Any | None = None,
        clock_local: Callable[[], datetime] | None = None,
        per_run_storage: bool = False,
        context_started: Callable[[MeasurementStorageContext], None] | None = None,
        context_stopped: Callable[[str], None] | None = None,
        journal_sink: Callable[[Mapping[str, Any]], bool] | None = None,
    ) -> None:
        self._data_root_provider = data_root_provider
        self._writer = writer
        self._clock_local = clock_local or (lambda: datetime.now().astimezone())
        self._lock = threading.Lock()
        self._records: dict[str, MeasurementRecord] = {}
        self._current_id = ""
        self._reserved_prefixes: set[str] = set()
        self._per_run_storage = per_run_storage
        self._context_started = context_started
        self._context_stopped = context_stopped
        self._journal_sink = journal_sink
        self._contexts: dict[str, MeasurementStorageContext] = {}
        self.observer_errors = 0

    @property
    def writer(self) -> Any | None:
        return self._writer

    def set_writer(self, writer: Any | None) -> None:
        self._writer = writer

    # -- lifecycle ------------------------------------------------------

    def begin(
        self,
        measurement_id: str,
        *,
        started_utc_ns: int,
        started_monotonic_ns: int,
        started_local: datetime | None = None,
        metadata: Mapping[str, Any] | None = None,
        calibration: Mapping[str, Any] | None = None,
    ) -> MeasurementRecord:
        local = (
            datetime.fromtimestamp(started_utc_ns // 1_000_000_000, tz=KST).replace(microsecond=(started_utc_ns % 1_000_000_000) // 1000)
            if self._per_run_storage else started_local or self._clock_local()
        )
        prefix = measurement_file_prefix(local)
        path: Path | None = None
        allocation_error = ""
        try:
            with self._lock:
                if self._per_run_storage:
                    context = reserve_measurement_context(Path(self._data_root_provider()), measurement_id, started_utc_ns, started_monotonic_ns)
                    self._contexts[measurement_id] = context
                    path = context.workbook_path
                else:
                    path = allocate_measurement_workbook_path(
                        Path(self._data_root_provider()), prefix, reserved=self._reserved_prefixes,
                    )
                prefix = path.name.removesuffix("_measurement.xlsx")
            if self._per_run_storage and self._context_started is not None:
                self._context_started(context)
        except Exception as exc:  # noqa: BLE001 - storage never gates Measurement Start.
            allocation_error = f"{type(exc).__name__}: {exc}"
        record = MeasurementRecord(
            measurement_id=measurement_id,
            file_prefix=prefix,
            workbook_path=path,
            started_local=local,
            started_utc_ns=started_utc_ns,
            started_monotonic_ns=started_monotonic_ns,
            metadata=metadata,
            calibration=calibration,
            journal_sink=self._journal_sink,
        )
        record._journal("start", {
            "file_prefix": prefix, "started_local": local.isoformat(),
            "started_utc_ns": started_utc_ns, "started_monotonic_ns": started_monotonic_ns,
            "metadata": record.metadata, "calibration": record.calibration,
        })
        record.add_event(
            "MEASUREMENT_STARTED",
            detail=str(path or "workbook path unavailable"),
            utc_ns=started_utc_ns,
            monotonic_ns=started_monotonic_ns,
        )
        if allocation_error:
            record.note_storage_result(ok=False, error=f"workbook path allocation failed: {allocation_error}")
        with self._lock:
            self._records[record.measurement_id] = record
            self._current_id = record.measurement_id
        self._request_write(record, "start")
        return record

    def stop(
        self,
        measurement_id: str,
        *,
        stopped_utc_ns: int,
        stopped_monotonic_ns: int,
        stopped_local: datetime | None = None,
    ) -> MeasurementRecord | None:
        record = self.record_for(measurement_id)
        if record is None:
            return None
        record.stop(
            stopped_local=(
                datetime.fromtimestamp(stopped_utc_ns // 1_000_000_000, tz=KST).replace(microsecond=(stopped_utc_ns % 1_000_000_000) // 1000)
                if self._per_run_storage else stopped_local or self._clock_local()
            ),
            stopped_utc_ns=stopped_utc_ns,
            stopped_monotonic_ns=stopped_monotonic_ns,
        )
        with self._lock:
            if self._current_id == record.measurement_id:
                self._current_id = ""
        self._request_write(record, "stop")
        if self._context_stopped is not None:
            self._context_stopped(measurement_id)
        return record

    def context_for(self, measurement_id: str) -> MeasurementStorageContext | None:
        with self._lock:
            return self._contexts.get(str(measurement_id))

    def current(self) -> MeasurementRecord | None:
        with self._lock:
            return self._records.get(self._current_id) if self._current_id else None

    def results_snapshot(self, measurement_id: str | None = None, pair_id: str | None = None) -> dict[str, Any]:
        with self._lock:
            record = self._records.get(self._current_id) if self._current_id else next(reversed(self._records.values()), None)
        if record is None:
            return {"measurement_id": "", "pair_id": "", "latest_pair_id": "", "rows": [], "all_terminal": False}
        return record.results_snapshot(pair_id if measurement_id == record.measurement_id else None)

    def record_for(self, measurement_id: str) -> MeasurementRecord | None:
        with self._lock:
            return self._records.get(str(measurement_id or ""))

    def records(self) -> tuple[MeasurementRecord, ...]:
        with self._lock:
            return tuple(self._records.values())

    def is_current(self, measurement_id: str) -> bool | None:
        """True/False for a known record; None when the registry never saw it."""

        with self._lock:
            record = self._records.get(str(measurement_id or ""))
            if record is None:
                return None
            return record.measurement_id == self._current_id and record.running

    # -- evidence routing ----------------------------------------------

    def observe_detection(self, camera: str, observation: Any) -> None:
        record = self.current()
        if record is None or not record.running:
            return
        try:
            record.observe_detection(camera, observation)
        except Exception:  # noqa: BLE001 - evidence capture never disturbs Detection.
            self.observer_errors += 1

    def observe_store_record(self, stream: str, payload: Mapping[str, Any]) -> None:
        try:
            self._observe_store_record(stream, payload)
        except Exception:  # noqa: BLE001 - never disturb the storage funnel.
            self.observer_errors += 1

    def _observe_store_record(self, stream: str, payload: Mapping[str, Any]) -> None:
        stream = str(stream)
        if stream == "measurement.journal":
            return
        if stream == "events":
            event_type = str(payload.get("event_type") or "")
            generation = str(payload.get("runtime_generation") or payload.get("measurement_id") or "")
            if event_type in {"measurement_start_requested", "measurement_stopped"}:
                return
            record = self.record_for(generation) if generation else self.current()
            if record is not None and event_type:
                record.add_event(event_type, detail=str(payload.get("detail") or payload.get("error") or ""), payload=payload, utc_ns=_int_or_none(payload.get("recorded_utc_ns")))
            return
        target: MeasurementRecord | None = None
        tagged = str(payload.get("measurement_id") or payload.get("runtime_generation") or "")
        if tagged:
            target = self.record_for(tagged)
        if target is None and not tagged:
            target = self.current()
        if target is None:
            return
        # Telemetry belongs to the Start->Stop interval only; a stopped record
        # still accepts artifact lifecycle rows (e.g. its own video close).
        if not target.running and stream != "artifacts":
            return
        target.observe_store_record(stream, payload)

    def route_outcome(self, outcome: Any) -> MeasurementRecord | None:
        record_payload = outcome.to_record() if hasattr(outcome, "to_record") else dict(outcome)
        record = self.record_for(str(record_payload.get("measurement_id") or ""))
        if record is None:
            return None
        record.note_outcome(record_payload)
        self._request_write(record, "outcome")
        return record

    def pending_job_count(self, measurement_id: str | None = None) -> int:
        if measurement_id is None:
            current = self.current()
            return 0 if current is None else current.pending_method_count()
        record = self.record_for(measurement_id)
        return 0 if record is None else record.pending_method_count()

    # -- retirement / shutdown -----------------------------------------

    def tick(self) -> None:
        """Drop stopped records once fully written; never drops the current one."""

        with self._lock:
            retired = [
                measurement_id
                for measurement_id, record in self._records.items()
                if measurement_id != self._current_id
                and record.stopped
                and record.pending_method_count() == 0
                and record.written_revision >= record.revision
            ]
            for measurement_id in retired:
                self._records.pop(measurement_id, None)

    def finalize_all(self, reason: str, *, timeout_s: float = 5.0) -> None:
        deadline = time.monotonic() + max(0.0, float(timeout_s))
        for record in self.records():
            record.fail_pending(reason)
            if record.running:
                record.add_event("APPLICATION_SHUTDOWN", detail=str(reason))
                self.stop(record.measurement_id, stopped_utc_ns=time.time_ns(), stopped_monotonic_ns=time.monotonic_ns())
        writer = self._writer
        if writer is None:
            return
        for record in self.records():
            remaining = max(0.0, deadline - time.monotonic())
            try:
                writer.write_now(record, reason=reason, timeout_s=remaining)
            except Exception:  # noqa: BLE001 - shutdown must finish regardless.
                self.observer_errors += 1

    def _request_write(self, record: MeasurementRecord, reason: str) -> None:
        writer = self._writer
        if writer is None:
            return
        try:
            writer.request(record, reason=reason)
        except Exception:  # noqa: BLE001 - the writer is best effort.
            self.observer_errors += 1

    def status_snapshot(self) -> dict[str, Any]:
        records = self.records()
        current = self.current()
        return {
            "current_measurement_id": "" if current is None else current.measurement_id,
            "records": [
                {
                    "measurement_id": record.measurement_id,
                    "state": record.refresh_state(),
                    "workbook_path": "" if record.workbook_path is None else str(record.workbook_path),
                    "pending_method_jobs": record.pending_method_count(),
                    "storage": record.storage_snapshot(),
                }
                for record in records
            ],
            "observer_errors": self.observer_errors,
        }


def _int_or_none(value: object) -> int | None:
    try:
        return None if value in (None, "") else int(value)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return None


__all__ = [
    "ALWAYS_PRESENT_TELEMETRY_SHEETS",
    "BASE_GNSS_SHEET",
    "GNSS_BASELINE_CENTER_SHEET",
    "METHOD_SHEET_NAMES",
    "RECORD_STATE_RUNNING",
    "RECORD_STATE_STOPPED",
    "RECORD_STATE_STOPPED_DRAINING",
    "STATUS_NOT_ATTEMPTED",
    "STATUS_PENDING",
    "DetectionRow",
    "EventRow",
    "MeasurementRecord",
    "MeasurementRegistry",
    "MeasurementSnapshot",
    "MethodResultRow",
    "PairRow",
    "allocate_measurement_workbook_path",
    "detection_id_for",
    "measurement_file_prefix",
    "telemetry_sheet_for",
]
