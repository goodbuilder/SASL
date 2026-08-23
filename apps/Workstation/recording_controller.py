"""Workstation recording session controller.

Recording rows retain their declared camera/PX4/GNSS coordinate and timestamp
contracts. This controller owns session state transitions and presentation;
persistence I/O runs in Workstation.session_workers.
"""

from __future__ import annotations

import hashlib
import json
import math
import os
import re
import time
import uuid
from collections import deque
from collections.abc import Mapping, Sequence
from datetime import datetime, timezone
from decimal import Decimal, InvalidOperation, ROUND_HALF_UP
from math import isfinite
from pathlib import Path
from statistics import median

from PyQt5 import QtCore, QtGui, QtWidgets

from Workstation.workstation_core.attitude_geometry import (quaternion_conjugate, quaternion_multiply, relative_attitude, relative_rotation_angle_deg)
from Workstation.workstation_core.measurement_attempt import (
    AttitudeSnapshot,
    BlockedMeasurementPair,
    GpsSnapshot,
    MeasurementAttempt,
    StereoIntrinsicsSnapshot,
)
from Workstation.workstation_core.measurement_methods import (
    CHECKERBOARD_STEREO,
    MEASUREMENT_METHOD_IDS,
)
from Workstation.workstation_core.dual_geometry import evaluate_nominal_active
from Workstation.workstation_core.coordinate_contract import (
    BODY_FRAME,
    CAMERA_FRAME,
    CAMERA_MOUNT_PROFILE,
    LEGACY_XYZ_ALIAS_CONTRACT,
    MEASUREMENT_AXIS_CONTRACT,
    MEASUREMENT_FRAME,
    PUBLIC_MEASUREMENT_VECTOR_CONTRACT,
    WORLD_FRAME,
)
from Workstation.workstation_core.measurement_widgets import DistanceHistoryPoint
from Workstation.workstation_core.hr_evidence import (
    CapturePurpose,
    create_hr_capture_event,
    import_hr_image_file,
)
from Workstation.workstation_core.native_frame_store import NativeFrameStore
from Workstation.ulog_controller import (
    ManualOwnershipClaim,
    ULogController,
)
from Workstation.workstation_core.mavlink_sources import MavlinkSourceId
from Workstation.workstation_core.recording_session import (
    SavedSessionFiles,
    SpoolRows,
    compact_distance_measurement_rows,
    discover_retryable_sessions,
    load_retryable_session,
    session_id_now,
    session_sheet_previews,
    write_recovery_metadata,
)
from Workstation.workstation_core.rtk_geometry import ned_delta_m, project_ned_baseline_to_p1_body_frd
from Workstation.workstation_core.runtime_camera_status import infer_runtime_camera_timestamp_reference
from Workstation.workstation_core.sync import (
    ATTITUDE_STALE_AGE_S,
    ATTITUDE_STALE_PAIR_DELTA_S,
    ATTITUDE_TIMING_POLICY,
    ATTITUDE_VERY_STALE_AGE_S,
    ATTITUDE_VERY_STALE_PAIR_DELTA_S,
    classify_attitude_timing,
)
from Workstation.workstation_core.strict_jsonl import (
    append_jsonl_record,
    prepare_jsonl_append,
)
from Workstation.device_runtime_workers import PixhawkParameterWorker
from Workstation.evidence import mavlink_link_provenance
from Workstation.session_workers import SessionSaveWorker
from Workstation.wiris_runtime_workers import (
    WirisOZPhotoCaptureWorker,
    WirisPnPPhotoCaptureWorker,
)

RECORDING_PREVIEW_LIMIT = 256
RECORDING_KEY_CACHE_LIMIT = 1024


def new_measurement_attempt_generation_id(session_id: str = "") -> str:
    return f"{session_id or 'runtime'}-{uuid.uuid4().hex}"


RECORDING_CONTROLLER_METHODS = [
    "_refresh_recording_status_label",
    "_recording_table_value",
    "_recording_preview_rows",
    "_recording_sheet_names",
    "_recording_sheet_kind",
    "_active_recording_sheet_name",
    "_append_recording_table_row",
    "_refresh_active_recording_sheet",
    "_fill_recording_sheet",
    "_refresh_recording_table",
    "_refresh_recording_sample_detail",
    "toggle_session_recording",
    "start_automatic_session_recording",
    "stop_automatic_session_recording",
    "_start_session_recording",
    "_recover_retryable_recording_session",
    "_stop_session_recording",
    "capture_pnp_hr_photos",
    "capture_pnp_oz_photos",
    "import_hr_files",
    "_camera_photo_target_infos",
    "_wiris_ftp_worker_kwargs",
    "_append_pnp_photo_capture_record",
    "_append_oz_photo_capture_record",
    "_handle_pnp_photo_capture_completed",
    "_handle_pnp_photo_capture_failed",
    "_clear_pnp_photo_capture_worker",
    "_handle_oz_photo_capture_completed",
    "_handle_oz_photo_capture_failed",
    "_clear_oz_photo_capture_worker",
    "toggle_distance_recording",
    "toggle_video_recording",
    "_pair_recording_base_row",
    "_upsert_pair_recording_row",
    "_record_unresolved_pair",
    "_record_blocked_measurement_pair",
    "_append_distance_history",
    "_append_extrinsics_comparison_snapshot",
    "_distance_history_class_name",
    "_fmt_record_float",
    "_fmt_record_bool",
    "_recording_elapsed_s",
    "_session_utc_ns_for_monotonic_ns",
    "_recording_side",
    "_raw_timeline_row",
    "_record_gui_camera_frame",
    "_record_external_timeline_row",
    "_record_bbox_parameters",
    "_detection_utc_unix_s",
    "_unix_seconds_to_ns",
    "_record_measurement_sync_timestamp_parameters",
    "_record_frame_timestamp_parameters",
    "_record_live_telemetry_snapshot_parameters",
    "_record_reference_delta_parameters",
    "_record_current_intrinsic_parameters",
    "_record_intrinsic_parameters",
    "_record_current_checkerboard_extrinsics_parameters",
    "_record_checkerboard_extrinsics_parameters",
    "_record_external_parameters",
    "_record_extrinsic_state",
    "_record_gps_state",
    "_record_gnss_relative_state",
    "save_distance_history_csv",
    "_handle_session_saved",
    "_handle_session_save_failed",
    "_handle_session_save_thread_finished",
    "_wait_for_session_save",
    "_record_px4_mag_status_rows",
    "_record_px4_mag_status_snapshot",
    "_recording_camera_timing_diagnostics",
    "_recording_session_metadata",
    "workstation_ulog_snapshot",
    "import_workstation_ulog",
    "_remove_distance_history_point",
]


class RecordingController:
    def __init__(self, panel):
        self.panel = panel
        # Completion/failure signals do not carry the operator purpose or the
        # Workstation request timestamp.  Keep that immutable request context
        # in the GUI owner until the worker returns; it is never camera truth.
        self._pending_hr_requests: dict[str, dict[str, object]] = {}
        self._pending_oz_requests: dict[str, dict[str, object]] = {}

    def _refresh_recording_status_label(self) -> None:
        if self.panel._session_save_thread is not None:
            self.panel.recording_status_label.setText(
                f"세션 기록: 저장 중, session={self.panel.recording_session_id or 'none'}"
            )
            return
        state = "진행 중" if self.panel._session_recording_active() else "중지"
        session = f", session={self.panel.recording_session_id}" if self.panel.recording_session_id else ""
        self.panel.recording_status_label.setText(
            "세션 기록: "
            f"{state}, 매칭 프레임쌍={len(self.panel.distance_recording_rows)}{session}"
        )

    def _recording_table_value(self, row: dict[str, object], field: str) -> str:
        value = row.get(field, "")
        if value is None:
            return ""
        return str(value)

    def _recording_preview_rows(self) -> list[dict[str, object]]:
        rows = self.panel.distance_recording_rows
        if isinstance(rows, SpoolRows):
            return rows.preview_rows
        return list(rows[-RECORDING_PREVIEW_LIMIT:])

    def _recording_sheet_names(self) -> tuple[str, ...]:
        """Workbook sheet names, in write order, without needing session rows."""

        return tuple(sheet.name for sheet in session_sheet_previews({}, []))

    def _recording_sheet_kind(self, sheet_name: str) -> str | None:
        return next(
            (sheet.kind for sheet in session_sheet_previews({}, []) if sheet.name == sheet_name),
            None,
        )

    def _active_recording_sheet_name(self) -> str | None:
        if not hasattr(self.panel, "recording_sheet_tabs"):
            return None
        index = self.panel.recording_sheet_tabs.currentIndex()
        if index < 0:
            return None
        return self.panel.recording_sheet_tabs.tabText(index)

    def _append_recording_table_row(self, _row: dict[str, object]) -> None:
        """Refresh the visible sheet after one accepted pair was recorded.

        Sheet columns are projected from every recorded row, so a new sample
        can widen them; rebuilding the visible sheet is simpler than patching
        one row into each of the seven sheets.
        """

        self.panel._refresh_active_recording_sheet(scroll_to_bottom=True)

    def _refresh_active_recording_sheet(self, *, scroll_to_bottom: bool = False) -> None:
        sheet_name = self.panel._active_recording_sheet_name()
        if sheet_name is None:
            return
        table = self.panel.recording_sheet_tables.get(sheet_name)
        if table is None:
            return
        rows = self.panel._recording_preview_rows()
        # Only the key/value sheets read metadata, and building it runs the
        # clock-sync and camera-timing probes — skip it on the sample sheets,
        # which are refreshed once per recorded pair.
        needs_metadata = sheet_name in {"Summary", "CalibrationStatic"}
        metadata = self.panel._recording_session_metadata() if needs_metadata else {}
        preview = next(
            (sheet for sheet in session_sheet_previews(metadata, rows) if sheet.name == sheet_name),
            None,
        )
        if preview is None:
            return
        table.blockSignals(True)
        try:
            if preview.kind == "key_value":
                self.panel._fill_recording_sheet(table, ("key", "value"), [
                    (str(key), "" if value is None else str(value)) for key, value in preview.entries
                ])
            else:
                display_rows = (
                    compact_distance_measurement_rows(
                        rows,
                        {
                            "session_started_unix_ns": self.panel.recording_session_started_unix_ns,
                            "session_started_monotonic_ns": self.panel.recording_session_started_monotonic_ns,
                        },
                    )
                    if sheet_name == "DistanceMeasurements"
                    else []
                )
                self.panel._fill_recording_sheet(
                    table,
                    preview.columns,
                    [
                        tuple(self.panel._recording_table_value(row, field) for field in preview.columns)
                        for row in display_rows
                    ],
                )
        finally:
            table.blockSignals(False)
        if scroll_to_bottom and table.rowCount():
            table.scrollToBottom()
            table.selectRow(table.rowCount() - 1)
        self.panel._refresh_recording_sample_detail()

    def _fill_recording_sheet(
        self,
        table: QtWidgets.QTableWidget,
        headers: Sequence[str],
        rows: Sequence[Sequence[str]],
    ) -> None:
        normalized_headers = tuple(str(header) for header in headers)
        current_headers = tuple(
            table.horizontalHeaderItem(index).text()
            if table.horizontalHeaderItem(index) is not None
            else ""
            for index in range(table.columnCount())
        )
        if table.columnCount() != len(normalized_headers):
            table.setColumnCount(len(normalized_headers))
        if current_headers != normalized_headers:
            table.setHorizontalHeaderLabels(list(normalized_headers))
        if table.rowCount() != len(rows):
            table.setRowCount(len(rows))
        for row_index, values in enumerate(rows):
            for column_index, value in enumerate(values):
                text = str(value)
                item = table.item(row_index, column_index)
                if item is None:
                    item = QtWidgets.QTableWidgetItem(text)
                    item.setFlags(QtCore.Qt.ItemIsEnabled | QtCore.Qt.ItemIsSelectable)
                    table.setItem(row_index, column_index, item)
                elif item.text() != text:
                    item.setText(text)

    def _refresh_recording_table(self) -> None:
        if not hasattr(self.panel, "recording_sheet_tabs"):
            return
        self.panel._refresh_active_recording_sheet()
        if not self.panel.distance_recording_rows:
            self.panel.recording_sample_detail.clear()

    def _refresh_recording_sample_detail(self) -> None:
        if not hasattr(self.panel, "recording_sheet_tabs"):
            return
        sheet_name = self.panel._active_recording_sheet_name()
        table = self.panel.recording_sheet_tables.get(sheet_name) if sheet_name else None
        if table is None:
            return
        if sheet_name is not None and self.panel._recording_sheet_kind(sheet_name) == "key_value":
            self.panel.recording_sample_detail.clear()
            return
        selected_rows = table.selectionModel().selectedRows()
        preview_rows = self.panel._recording_preview_rows()
        if not selected_rows:
            self.panel.recording_sample_detail.clear()
            return
        row_index = selected_rows[0].row()
        if row_index < 0 or row_index >= len(preview_rows):
            self.panel.recording_sample_detail.clear()
            return
        row = preview_rows[row_index]
        lines = [f"{key}: {value}" for key, value in row.items()]
        self.panel.recording_sample_detail.setPlainText("\n".join(lines))

    def toggle_session_recording(self) -> bool | None:
        if self.panel._session_recording_active():
            self.panel._stop_session_recording()
            if not self.panel._session_recording_active():
                self.panel._automatic_session_recording = False
            return None
        return self.panel._start_session_recording()

    def start_automatic_session_recording(self) -> bool:
        """Start the session owned by the Measurement/Detection lifecycle.

        Session IDs, spools, and video paths remain owned by the recording
        controller.  The camera lifecycle only asks for this boundary before
        it starts a detection stream, so the normal operator path never needs
        a separate Record or Save action.
        """

        if self.panel._session_recording_active():
            return True
        if bool(getattr(self.panel, "_measurement_package_explicit_lifecycle", False)):
            snapshotter = getattr(self.panel, "measurement_package_snapshot", None)
            snapshot = snapshotter() if callable(snapshotter) else {}
            if not isinstance(snapshot, Mapping) or str(snapshot.get("state") or "") != "COLLECTING":
                self.panel.recording_status_label.setText(
                    "세션 기록: 측정 시작이 준비한 COLLECTING 상태가 필요합니다"
                )
                return False
        if bool(getattr(self.panel, "_session_save_retry_required", False)):
            self.panel.recording_status_label.setText(
                "세션 기록: 이전 자동 저장 실패를 먼저 확인해야 합니다"
            )
            return False
        previous_mode = str(getattr(self.panel, "_recording_mode", "manual") or "manual")
        self.panel._automatic_session_recording = True
        self.panel._recording_mode = "automatic_detection"
        try:
            result = self._start_session_recording(start_camera_streams=False)
        except Exception:
            self.panel._automatic_session_recording = False
            self.panel._recording_mode = previous_mode
            raise
        if result is False:
            self.panel._automatic_session_recording = False
            self.panel._recording_mode = previous_mode
            return False
        self.panel.recording_status_label.setText(
            f"자동 세션 기록: 진행 중, session={self.panel.recording_session_id}"
        )
        return True

    def stop_automatic_session_recording(
        self,
        *,
        final_clock_sync_snapshot: dict[str, object] | None = None,
        restart_streams: bool = False,
    ) -> bool:
        """Stop producers and start legacy session persistence after Detection.

        The Measurement controller requests sealing when Detection ends; the
        asynchronous save callback resumes that fail-closed finalization.
        """

        if not bool(getattr(self.panel, "_automatic_session_recording", False)):
            return True
        if not self.panel._session_recording_active():
            self.panel._automatic_session_recording = False
            return True
        stop_kwargs: dict[str, object] = {"restart_streams": bool(restart_streams)}
        if final_clock_sync_snapshot is not None:
            stop_kwargs["final_clock_sync_snapshot"] = final_clock_sync_snapshot
        self._stop_session_recording(**stop_kwargs)
        if self.panel._session_recording_active():
            return False
        self.panel._automatic_session_recording = False
        return True

    def _start_session_recording(self, *, start_camera_streams: bool = True) -> bool | None:
        if not bool(getattr(self.panel, "_automatic_session_recording", False)):
            self.panel._recording_mode = "manual"
        if self._camera_photo_capture_active():
            self.panel.recording_status_label.setText("세션 기록: HR/OZ still 캡처 완료 대기 중")
            return False
        if self.panel._session_save_thread is not None:
            self.panel.recording_status_label.setText("세션 기록: 이전 세션 저장 완료 대기 중")
            return False
        if self.panel.wiris_time_sync_thread is not None:
            status_text = "세션 기록: WIRIS 시간 동기화 및 스트림 재연결 완료 대기 중"
            self.panel.recording_status_label.setText(status_text)
            self.panel._log(f"[저장] {status_text}")
            return False
        if (
            self.panel.pixhawk_connection_thread is not None
            or self.panel.pixhawk_disconnection_thread is not None
            or self.panel.pixhawk_usb_bootstrap_thread is not None
        ):
            status_text = "세션 기록: Pixhawk 연결/부트스트랩 완료 대기 중"
            self.panel.recording_status_label.setText(status_text)
            self.panel._log(f"[저장] {status_text}")
            return False
        if not self.panel._pnp_session_prepared:
            self.panel.recording_session_id = session_id_now()
            self.panel.pnp_photo_capture_records = []
            self.panel.oz_photo_capture_records = []
        self.panel._pnp_session_prepared = False
        self.panel._session_save_finalized = False
        self._ensure_ulog_controller()
        evidence_start = getattr(self.panel, "ensure_workstation_run_evidence", None)
        if callable(evidence_start):
            try:
                evidence_start(
                    "recording_started",
                    {
                        "session_id": self.panel.recording_session_id,
                        "measurement_id": str(getattr(self.panel, "_measurement_id", "") or ""),
                        "recording_mode": str(
                            getattr(self.panel, "_recording_mode", "manual") or "manual"
                        ),
                    },
                    source="RecordingController",
                    coordinate_contract="not_applicable",
                )
            except Exception as exc:
                self.panel._log(f"[저장] run evidence linkage failed: {exc}")
        self.panel._measurement_attempt_generation_id = new_measurement_attempt_generation_id(
            self.panel.recording_session_id
        )
        monotonic_before_ns = time.monotonic_ns()
        self.panel.recording_session_started_unix_ns = time.time_ns()
        monotonic_after_ns = time.monotonic_ns()
        self.panel.recording_session_started_monotonic_ns = (
            monotonic_before_ns + monotonic_after_ns
        ) // 2
        self.panel.recording_session_anchor_uncertainty_ns = (
            monotonic_after_ns - monotonic_before_ns
        ) // 2
        self.panel.recording_session_started_wall_s = self.panel.recording_session_started_unix_ns * 1e-9
        self.panel.recording_session_started_monotonic_s = self.panel.recording_session_started_monotonic_ns * 1e-9
        self.panel.distance_recording_started_monotonic_s = self.panel.recording_session_started_monotonic_s
        self.panel.distance_recording_rows = SpoolRows(
            preview_limit=RECORDING_PREVIEW_LIMIT,
            supports_updates=True,
        )
        self.panel.distance_recording_keys.clear()
        self.panel.external_parameter_recording_rows.clear()
        spool_root = self.panel._recording_session_dir(self.panel.recording_session_id) / ".spool"
        self.panel.distance_recording_rows.bind(spool_root / "pair_rows.jsonl")
        self.panel.camera_recording_rows = {
            side: SpoolRows() for side in ("left", "right")
        }
        self.panel.px4_recording_rows = {
            side: SpoolRows() for side in ("left", "right")
        }
        self.panel.gnss_base_recording_rows = SpoolRows()
        self.panel.extrinsics_comparison_recording_rows = SpoolRows()
        self.panel.method_evaluation_bundle_rows = SpoolRows()
        for side, rows in self.panel.camera_recording_rows.items():
            rows.bind(spool_root / f"camera_{side}.jsonl")
        for side, rows in self.panel.px4_recording_rows.items():
            rows.bind(spool_root / f"px4_{side}.jsonl")
        self.panel.gnss_base_recording_rows.bind(spool_root / "gnss_base.jsonl")
        self.panel.extrinsics_comparison_recording_rows.bind(spool_root / "extrinsics_comparison.jsonl")
        self.panel.method_evaluation_bundle_rows.bind(
            spool_root / "method_evaluation_bundles.jsonl"
        )
        self.panel._recording_pair_row_index.clear()
        self.panel._recovered_session_metadata = None
        self.panel._recovered_session_output_dir = None
        self.panel._recording_clock_sync_start_observation = None
        self.panel._recording_clock_sync_end_observation = None
        self.panel.recording_reference_attitudes = {
            camera: AttitudeSnapshot.from_state(state)
            for camera, state in self.panel.latest_extrinsics.items()
            if camera in {"left", "right"}
        }
        self.panel.recording_reference_gps = {
            camera: GpsSnapshot.from_state(state)
            for camera, state in self.panel.latest_gps.items()
            if camera in {"left", "right"}
        }
        self.panel.recording_origin_gps = self.panel.recording_reference_gps.get("left")
        self.panel.recording_nominal_intrinsics_snapshot = self._snapshot_current_intrinsics()
        self.panel.recording_nominal_optical_pose_profile_hash = str(
            getattr(self.panel, "_active_optical_pose_profile_hash", "") or ""
        )
        left_reference_gps = self.panel.recording_reference_gps.get("left")
        right_reference_gps = self.panel.recording_reference_gps.get("right")
        self.panel.recording_reference_baseline_ned_m = (
            None
            if left_reference_gps is None or right_reference_gps is None
            else ned_delta_m(left_reference_gps, right_reference_gps)
        )
        self.panel.distance_recording_enabled = True
        self.panel.video_recording_enabled = True
        self.panel._record_px4_mag_status_snapshot()
        self.panel._last_distance_history_key = None
        self.panel._measurement_attempt_counter = 0
        self.panel._last_measurement_attempt_key = None
        self.panel._last_measurement_attempt = None
        self.panel._last_frozen_measurement_pair = None
        self.panel._method_evaluation_submitted = set()
        self.panel._method_evaluation_committed = set()
        self.panel._method_evaluation_accepting = True
        self.panel._method_evaluation_stop_retry_scheduled = False
        self.panel._measurement_method_result_counters = {
            method_id: {"VALID": 0, "BLOCKED": 0, "FAILED": 0}
            for method_id in MEASUREMENT_METHOD_IDS
        }
        self.panel._prepare_video_recording_paths()
        write_recovery_metadata(
            self.panel._recording_session_dir(self.panel.recording_session_id),
            self.panel._recording_session_metadata(),
        )
        self.panel._refresh_recording_table()
        self.panel.record_button.setText("세션 기록 중지")
        self.panel._log(
            "[저장] 세션 기록을 시작했습니다: "
            f"session={self.panel.recording_session_id}, {self.panel._format_video_recording_paths()}"
        )
        if start_camera_streams and self.panel.camera_connected:
            self.panel._start_camera_streams(enable_detection=self.panel.yolo_enabled, switch_to_depth=self.panel.yolo_enabled)
        elif start_camera_streams:
            self.panel._log("[저장] 카메라 스트림을 시작하면 좌/우 영상 녹화가 함께 시작됩니다.")
        self.panel._refresh_recording_status_label()
        self.panel._refresh_operation_header()
        return None

    @staticmethod
    def _load_jsonl_records(path: Path) -> list[dict[str, object]]:
        if not path.is_file():
            return []
        prepare_jsonl_append(path)
        records: list[dict[str, object]] = []
        for line_number, raw_line in enumerate(
            path.read_text(encoding="utf-8").splitlines(),
            start=1,
        ):
            line = raw_line.strip()
            if not line:
                continue
            try:
                record = json.loads(line)
            except json.JSONDecodeError as exc:
                records.append(
                    {
                        "capture_id": f"corrupt-line-{line_number:06d}",
                        "state": "corrupt",
                        "error": f"malformed capture JSONL row at line {line_number}: {exc.msg}",
                        "line_number": line_number,
                        "raw_line_sha256": hashlib.sha256(
                            raw_line.encode("utf-8")
                        ).hexdigest(),
                    }
                )
                continue
            if isinstance(record, dict):
                records.append(record)
            else:
                records.append(
                    {
                        "capture_id": f"corrupt-line-{line_number:06d}",
                        "state": "corrupt",
                        "error": f"capture JSONL row at line {line_number} is not an object",
                        "line_number": line_number,
                        "raw_line_sha256": hashlib.sha256(
                            raw_line.encode("utf-8")
                        ).hexdigest(),
                    }
                )
        return records

    @staticmethod
    def _next_capture_id(records: Sequence[Mapping[str, object]], prefix: str) -> str:
        normalized = re.sub(r"[^a-z0-9]+", "", str(prefix).lower()) or "capture"
        pattern = re.compile(rf"^{re.escape(normalized)}-(\d+)$")
        high_water = max(
            (
                int(match.group(1))
                for record in records
                for match in (pattern.fullmatch(str(record.get("capture_id") or "").lower()),)
                if match is not None
            ),
            default=0,
        )
        return f"{normalized}-{high_water + 1:03d}"

    def _recover_retryable_recording_session(self) -> bool:
        try:
            candidates = discover_retryable_sessions(
                self.panel.app_root / "recordings"
            )
        except (FileExistsError, OSError, ValueError) as exc:
            self.panel._log(f"[저장] 복구 세션 경로 충돌: {exc}")
            self.panel.recording_status_label.setText(
                "세션 기록: direct/legacy 복구 경로 충돌 확인 필요"
            )
            return False
        if not candidates:
            return False
        recovered = None
        for candidate in candidates:
            try:
                recovered = load_retryable_session(candidate)
                break
            except Exception as exc:
                self.panel._log(
                    f"[저장] 미완료 세션 복구 정보를 건너뜁니다: "
                    f"path={candidate}, error={exc}"
                )
        if recovered is None:
            return False
        metadata = recovered.metadata

        def optional_int(value: object) -> int | None:
            return None if value in (None, "") else int(value)

        self.panel.recording_session_id = str(metadata.get("session_id") or recovered.output_dir.name)
        recovered_measurement_id = str(metadata.get("measurement_id") or "")
        if recovered_measurement_id and bool(
            getattr(self.panel, "_measurement_session_hierarchy_enabled", False)
        ):
            self.panel._measurement_session_id = self.panel.recording_session_id
            self.panel._measurement_id = recovered_measurement_id
            try:
                from Workstation.workstation_core.measurement_session import MeasurementSession

                session_root = recovered.output_dir.parent.parent
                session = MeasurementSession.open(session_root)
                self.panel._measurement_session = session
                self.panel._measurement_session_state = session.state
                self.panel._measurement_package = session.measurement_package(
                    recovered_measurement_id
                )
                self.panel._measurement_package_state = "RECOVERABLE"
            except Exception as exc:
                self.panel._log(f"[저장] 측정 세션 인덱스 복구 실패: {exc}")
        self.panel.recording_session_started_unix_ns = optional_int(
            metadata.get("session_started_unix_ns")
        )
        self.panel.recording_session_started_monotonic_ns = optional_int(
            metadata.get("session_started_monotonic_ns")
        )
        self.panel.recording_session_anchor_uncertainty_ns = optional_int(
            metadata.get("session_time_anchor_uncertainty_ns")
        )
        self.panel.recording_session_started_wall_s = (
            None
            if self.panel.recording_session_started_unix_ns is None
            else self.panel.recording_session_started_unix_ns * 1e-9
        )
        self.panel.recording_session_started_monotonic_s = (
            None
            if self.panel.recording_session_started_monotonic_ns is None
            else self.panel.recording_session_started_monotonic_ns * 1e-9
        )
        self.panel.distance_recording_rows = recovered.pair_rows
        self.panel.camera_recording_rows = recovered.camera_rows
        self.panel.px4_recording_rows = recovered.px4_rows
        self.panel.gnss_base_recording_rows = recovered.gnss_base_rows
        self.panel.extrinsics_comparison_recording_rows = recovered.extrinsics_comparison_rows
        self.panel.method_evaluation_bundle_rows = recovered.method_evaluation_bundle_rows
        # HR/OZ capture event_index/capture_id counters are derived from
        # len(pnp_photo_capture_records)/len(oz_photo_capture_records); without
        # restoring them here they would silently restart at 1 on top of a
        # recovered session's existing hr_captures.jsonl/oz_captures.jsonl,
        # producing duplicate event_index/capture_id values in that file.
        self.panel.pnp_photo_capture_records = self._load_jsonl_records(
            recovered.output_dir / "hr_captures.jsonl"
        )
        self.panel.oz_photo_capture_records = self._load_jsonl_records(
            recovered.output_dir / "oz_captures.jsonl"
        )
        self.panel._recovered_session_metadata = metadata
        # Pin retries to the exact discovered tree.  This prevents a legacy
        # ``recordings/sessions/<id>`` spool from being read there and then
        # finalized into the new direct ``recordings/<id>`` tree.
        self.panel._recovered_session_output_dir = recovered.output_dir
        self.panel._session_save_retry_required = True
        self.panel._session_save_finalized = False
        self.panel._automatic_session_recording = False
        self.panel._recording_mode = str(metadata.get("recording_mode") or "manual")
        self.panel.distance_recording_enabled = False
        self.panel.video_recording_enabled = False
        self.panel.record_button.setEnabled(False)
        self.panel.save_history_button.setEnabled(True)
        self.panel._refresh_recording_table()
        self.panel.recording_status_label.setText(
            f"미완료 세션 복구됨: session={self.panel.recording_session_id}, "
            f"프레임쌍={len(self.panel.distance_recording_rows)}개 — 세션 파일 저장으로 재시도"
        )
        self.panel._log(
            f"[저장] 미완료 세션 spool을 복구했습니다: {recovered.output_dir} "
            f"(pair={len(self.panel.distance_recording_rows)})"
        )
        return True

    def _stop_session_recording(
        self,
        *,
        restart_streams: bool = True,
        final_clock_sync_snapshot: dict[str, object] | None = None,
    ) -> None:
        if self._camera_photo_capture_active():
            self.panel.recording_status_label.setText("세션 기록: HR/OZ still 캡처 완료 대기 중")
            return
        measurement_owner = getattr(
            self.panel, "_measurement_view_controller_owner", None
        )
        if measurement_owner is not None:
            self.panel._method_evaluation_accepting = False
            pending_method_pairs = measurement_owner.method_evaluation_pending_count()
            if pending_method_pairs:
                self.panel.recording_status_label.setText(
                    f"세션 기록: method 결과 barrier 대기 ({pending_method_pairs} pairs)"
                )
                if not bool(
                    getattr(
                        self.panel,
                        "_method_evaluation_stop_retry_scheduled",
                        False,
                    )
                ):
                    self.panel._method_evaluation_stop_retry_scheduled = True

                    def retry_stop() -> None:
                        self.panel._method_evaluation_stop_retry_scheduled = False
                        self.panel._stop_session_recording(
                            restart_streams=restart_streams,
                            final_clock_sync_snapshot=final_clock_sync_snapshot,
                        )

                    QtCore.QTimer.singleShot(100, retry_stop)
                return
        if final_clock_sync_snapshot is None:
            final_clock_sync_snapshot = (
                self.panel._camera_time_sync_snapshot("last_observed_before_session_stop")
                if self.panel._session_recording_active()
                else None
            )
        if final_clock_sync_snapshot is not None:
            self.panel._recording_clock_sync_end_observation = final_clock_sync_snapshot
        try:
            self._finalize_workstation_ulogs()
        except Exception as exc:
            self.panel.recording_status_label.setText(
                f"세션 기록: ULog finalization failed ({exc})"
            )
            self.panel._log(f"[저장] ULog finalization 실패: {exc}")
            return
        target_owner = getattr(self.panel, "_target_drone_controller_owner", None)
        if target_owner is not None:
            try:
                target_snapshot = target_owner.target_drone_stream_snapshot()
                if target_snapshot.get("worker_alive") or target_snapshot.get("phase") in {
                    "RUNNING",
                    "STARTING",
                }:
                    target_snapshot = target_owner.stop_target_drone_stream(timeout_s=3.0)
                artifact = target_snapshot.get("artifact")
                if isinstance(artifact, dict):
                    self._register_session_owned_artifact(
                        artifact["absolute_path"],
                        role="target_drone_raw_mavlink_jsonl",
                        media_type="application/x-ndjson",
                        metadata={
                            key: value
                            for key, value in artifact.items()
                            if key != "absolute_path"
                        },
                        expected_size_bytes=artifact.get("size_bytes"),
                        expected_sha256=artifact.get("sha256"),
                    )
            except Exception as exc:
                self.panel.recording_status_label.setText(
                    f"세션 기록: Target Drone drain/artifact 실패 ({exc})"
                )
                self.panel._log(f"[저장] Target Drone finalization 실패: {exc}")
                return
        self.panel.distance_recording_enabled = False
        self.panel.video_recording_enabled = False
        self.panel.distance_recording_started_monotonic_s = None
        self.panel.record_button.setText("세션 기록 시작")
        self.panel._log(
            "[저장] 세션 기록을 중지했습니다: "
            f"session={self.panel.recording_session_id or 'none'}, {self.panel._format_video_recording_paths()}"
        )
        if restart_streams and self.panel.camera_connected:
            self.panel._start_camera_streams(enable_detection=self.panel.yolo_enabled, switch_to_depth=self.panel.yolo_enabled)
        self.panel.save_distance_history_csv(final_clock_sync_snapshot=final_clock_sync_snapshot)
        self.panel._refresh_recording_status_label()
        self.panel._refresh_operation_header()

    def _finalize_workstation_ulogs(self) -> None:
        """Stop/download only ULogs whose injected adapter proves ownership."""

        controller = getattr(self.panel, "_workstation_ulog_controller", None)
        if not isinstance(controller, ULogController):
            return
        snapshot = controller.ulog_snapshot()
        for source in MavlinkSourceId:
            row = snapshot.get("sources", {}).get(source.value, {})
            if not isinstance(row, dict) or row.get("owner") != "WORKSTATION":
                continue
            controller.stop_automatic_ulog_and_collect(source)
        refreshed = controller.ulog_snapshot()
        for artifact in refreshed.get("artifacts", ()) or ():
            if not isinstance(artifact, dict):
                continue
            relative = str(artifact.get("relative_path") or "")
            if not relative:
                continue
            self._register_session_owned_artifact(
                controller.session_root / relative,
                role=f"ulog/{artifact.get('source_id', 'unknown')}",
                media_type="application/vnd.px4.ulog",
                metadata=dict(artifact),
                expected_size_bytes=artifact.get("size_bytes"),
                expected_sha256=artifact.get("sha256"),
            )

    def _register_session_owned_artifact(
        self,
        path: Path | str,
        *,
        role: str,
        media_type: str,
        metadata: Mapping[str, object],
        expected_size_bytes: object = None,
        expected_sha256: object = None,
    ) -> object | None:
        """Register session-owned sealed bytes, with a legacy copy fallback."""

        registrar = getattr(self.panel, "register_workstation_session_artifact", None)
        if callable(registrar):
            return registrar(
                path,
                role=role,
                media_type=media_type,
                metadata=dict(metadata),
                expected_size_bytes=expected_size_bytes,
                expected_sha256=expected_sha256,
            )
        importer = getattr(self.panel, "import_workstation_session_artifact", None)
        if callable(importer):
            return importer(
                path,
                role=role,
                media_type=media_type,
                metadata=dict(metadata),
            )
        return None

    def _camera_photo_capture_active(self) -> bool:
        return bool(
            getattr(self.panel, "pnp_photo_capture_thread", None) is not None
            or getattr(self.panel, "oz_photo_capture_thread", None) is not None
        )

    @staticmethod
    def _hr_capture_purpose(value: CapturePurpose | str) -> CapturePurpose:
        try:
            return CapturePurpose(str(getattr(value, "value", value)).strip().lower())
        except ValueError as exc:
            allowed = ", ".join(item.value for item in CapturePurpose)
            raise ValueError(f"HR capture purpose must be one of: {allowed}") from exc

    def _prepare_camera_evidence_session(self, capture_channel: str) -> None:
        """Prepare camera evidence without pretending Detection started.

        The public capture method may lazily create a child only for ``Pre``.
        Post/checkpoint capture never allocates a new child after sealing, so
        evidence cannot silently cross a Measurement boundary.
        """

        if not self._measurement_package_capture_allowed():
            raise RuntimeError(
                "Create Session Folder is required before pre/post evidence capture"
            )
        if not self.panel.recording_session_id:
            self.panel.recording_session_id = session_id_now()
            self.panel.pnp_photo_capture_records = []
            self.panel.oz_photo_capture_records = []
            self.panel._session_save_finalized = False
            self.panel._pnp_session_prepared = True
        self._ensure_ulog_controller()
        evidence_start = getattr(self.panel, "ensure_workstation_run_evidence", None)
        if callable(evidence_start):
            evidence_start(
                f"{capture_channel}_capture_requested",
                {
                    "session_id": self.panel.recording_session_id,
                    "measurement_id": str(getattr(self.panel, "_measurement_id", "") or ""),
                    "capture_channel": capture_channel,
                },
                source="RecordingController",
                coordinate_contract="native raster u-right/v-down; OpenCV optical RDF",
            )

    def _measurement_package_capture_allowed(self, *, prepare_if_needed: bool = False) -> bool:
        """Keep legacy callers compatible while enforcing explicit UI sessions."""

        if not bool(getattr(self.panel, "_measurement_package_explicit_lifecycle", False)):
            return True
        snapshotter = getattr(self.panel, "measurement_package_snapshot", None)
        if not callable(snapshotter):
            return False
        try:
            snapshot = snapshotter()
        except Exception:
            return False
        if not isinstance(snapshot, Mapping):
            return False
        state = str(snapshot.get("state") or "").upper()
        if state in {"COLLECTING", "POST_CAPTURE"}:
            return True
        if not prepare_if_needed or state not in {
            "IDLE",
            "SEALED_COMPLETE",
            "SEALED_INCOMPLETE",
        }:
            return False
        starter = getattr(self.panel, "begin_measurement", None)
        if not callable(starter):
            starter = getattr(self.panel, "begin_next_measurement", None)
        if not callable(starter):
            return False
        try:
            if starter() is False:
                return False
            prepared = snapshotter()
        except Exception:
            return False
        return isinstance(prepared, Mapping) and str(
            prepared.get("state") or ""
        ).upper() == "COLLECTING"

    def _prepare_pnp_evidence_session(self) -> None:
        self._prepare_camera_evidence_session("hr")

    def _prepare_oz_evidence_session(self) -> None:
        self._prepare_camera_evidence_session("oz")

    def _ensure_ulog_controller(self) -> ULogController:
        session_id = str(getattr(self.panel, "recording_session_id", "") or "")
        if not session_id:
            raise RuntimeError("recording session ID is not prepared")
        root_resolver = getattr(self.panel, "_recording_session_dir", None)
        if not callable(root_resolver):
            raise RuntimeError("recording session directory owner is unavailable")
        root = Path(root_resolver(session_id)).resolve()
        current = getattr(self.panel, "_workstation_ulog_controller", None)
        if (
            not isinstance(current, ULogController)
            or current.session_id != session_id
            or current.session_root != root
        ):
            automatic_sources = getattr(self.panel, "ulog_automatic_sources", None)
            current = ULogController(
                root,
                session_id,
                automatic_sources=(
                    automatic_sources if isinstance(automatic_sources, dict) else None
                ),
            )
            setattr(self.panel, "_workstation_ulog_controller", current)
        return current

    def workstation_ulog_snapshot(self) -> dict[str, object]:
        controller = getattr(self.panel, "_workstation_ulog_controller", None)
        if not isinstance(controller, ULogController):
            return {
                "schema": "workstation-ulog-controller-v1",
                "state": "NOT_PREPARED",
                "artifacts": [],
                "sources": {},
            }
        return controller.ulog_snapshot()

    def import_workstation_ulog(
        self,
        source: str,
        path: str,
        *,
        ownership: str = "UNKNOWN",
        expected_sha256: str | None = None,
    ) -> dict[str, object]:
        """Import one exact P1/P2/target ULog into this session."""

        self._prepare_pnp_evidence_session()
        controller = self._ensure_ulog_controller()
        source_value = str(source).strip().lower()
        if source_value in {"target", "drone", "target_drone"}:
            source_value = MavlinkSourceId.TARGET_DRONE.value
        resolved_source = MavlinkSourceId(source_value)
        artifact = controller.import_manual_ulog(
            resolved_source,
            path,
            ownership=ManualOwnershipClaim(str(ownership).upper()),
            expected_sha256=expected_sha256,
        )
        absolute = controller.session_root / str(artifact["relative_path"])
        self._register_session_owned_artifact(
            absolute,
            role=f"ulog/{resolved_source.value}",
            media_type="application/vnd.px4.ulog",
            metadata=dict(artifact),
            expected_size_bytes=artifact.get("size_bytes"),
            expected_sha256=artifact.get("sha256"),
        )
        append_event = getattr(self.panel, "append_workstation_run_event", None)
        if callable(append_event):
            append_event(
                "ulog_imported",
                dict(artifact),
                source="RecordingController",
                coordinate_contract="opaque PX4 ULog; no optical RDF conversion",
            )
        return controller.ulog_snapshot()

    def _new_camera_request_context(
        self,
        capture_id: str,
        purpose: CapturePurpose,
        capture_channel: str,
    ) -> dict[str, object]:
        """Anchor an HR request in host UTC and the session monotonic clock.

        ``request_session_delta_ns`` is a signed host-monotonic delta from the
        distance-session anchor.  It is ``None`` for HR captures made before a
        distance session starts.  Neither value is a camera exposure time.
        """

        monotonic_before_ns = time.monotonic_ns()
        request_utc_ns = time.time_ns()
        monotonic_after_ns = time.monotonic_ns()
        request_monotonic_ns = (monotonic_before_ns + monotonic_after_ns) // 2
        session_started_ns = getattr(
            self.panel, "recording_session_started_monotonic_ns", None
        )
        request_session_delta_ns = (
            None
            if session_started_ns is None
            else request_monotonic_ns - int(session_started_ns)
        )
        return {
            "capture_id": str(capture_id),
            "purpose": purpose.value,
            "capture_channel": str(capture_channel),
            "request_utc_ns": request_utc_ns,
            "request_monotonic_ns": request_monotonic_ns,
            "request_clock_anchor_uncertainty_ns": (
                monotonic_after_ns - monotonic_before_ns
            )
            // 2,
            "request_utc_method": "host_time_ns_at_monotonic_midpoint",
            "request_session_delta_ns": request_session_delta_ns,
            "request_session_delta_s": (
                None
                if request_session_delta_ns is None
                else request_session_delta_ns * 1e-9
            ),
            "request_time_semantics": (
                "Workstation command request; not camera exposure or camera-file UTC"
            ),
        }

    def _new_hr_request_context(
        self,
        capture_id: str,
        purpose: CapturePurpose,
    ) -> dict[str, object]:
        return self._new_camera_request_context(capture_id, purpose, "hr")

    def _new_oz_request_context(
        self,
        capture_id: str,
        purpose: CapturePurpose,
    ) -> dict[str, object]:
        return self._new_camera_request_context(capture_id, purpose, "oz")

    @staticmethod
    def _camera_hr_request_delta(record: dict[str, object]) -> int | None:
        cameras = record.get("cameras")
        if not isinstance(cameras, dict):
            return None
        left = cameras.get("left")
        right = cameras.get("right")
        if not isinstance(left, dict) or not isinstance(right, dict):
            return None
        try:
            left_ns = int(left["capture_requested_unix_ns"])
            right_ns = int(right["capture_requested_unix_ns"])
        except (KeyError, TypeError, ValueError):
            return None
        return right_ns - left_ns

    def _hr_capture_event_record(
        self,
        record: dict[str, object],
        *,
        capture_id: str,
        request: dict[str, object] | None = None,
        note: str = "camera-internal HR capture",
    ) -> dict[str, object]:
        """Return one hash-bound, append-only HR event record."""

        context = dict(request or {})
        purpose = self._hr_capture_purpose(
            context.get("purpose", record.get("purpose", CapturePurpose.UNSPECIFIED))
        )
        request_utc_ns = int(context.get("request_utc_ns") or time.time_ns())
        event = create_hr_capture_event(
            capture_id,
            purpose=purpose,
            captured_utc_ns=request_utc_ns,
            note=note,
        ).to_mapping()
        event.update(context)
        event.update(record)
        event["capture_id"] = str(capture_id)
        event["purpose"] = purpose.value
        event["capture_time_semantics"] = (
            "Workstation request UTC; HR exposure UTC remains unavailable"
        )
        request_delta = self._camera_hr_request_delta(event)
        event["camera_request_delta_right_minus_left_ns"] = request_delta
        event["camera_request_delta_sign_contract"] = (
            "right capture_requested_unix_ns minus left capture_requested_unix_ns"
        )
        event["pixel_coordinate_contract"] = (
            "stored image raster u-right/v-down; OpenCV optical frame RDF "
            "(right/down/forward); no pixel or pose conversion performed"
        )
        return event

    def _oz_capture_event_record(
        self,
        record: dict[str, object],
        *,
        capture_id: str,
        request: dict[str, object] | None = None,
        note: str = "camera-internal OZ still capture",
    ) -> dict[str, object]:
        """Return one hash-bound, append-only OZ still event record."""

        context = dict(request or {})
        purpose = self._hr_capture_purpose(
            context.get("purpose", record.get("purpose", CapturePurpose.UNSPECIFIED))
        )
        request_utc_ns = int(context.get("request_utc_ns") or time.time_ns())
        event = {
            "schema": "workstation-oz-capture-event-v1",
            "event_id": str(uuid.uuid4()),
            "capture_id": str(capture_id),
            "purpose": purpose.value,
            "captured_utc_ns": request_utc_ns,
            "note": note,
        }
        event.update(context)
        event.update(record)
        event["capture_id"] = str(capture_id)
        event["capture_channel"] = "oz"
        event["purpose"] = purpose.value
        event["capture_time_semantics"] = (
            "Workstation request UTC; OZ exposure UTC remains unavailable"
        )
        request_delta = self._camera_hr_request_delta(event)
        event["camera_request_delta_right_minus_left_ns"] = request_delta
        event["camera_request_delta_sign_contract"] = (
            "right capture_requested_unix_ns minus left capture_requested_unix_ns"
        )
        event["pixel_coordinate_contract"] = (
            "camera-native visible/OZ still raster u-right/v-down; OpenCV optical "
            "frame RDF (right/down/forward); no pixel or pose conversion performed"
        )
        return event

    def capture_pnp_hr_photos(
        self,
        purpose: CapturePurpose | str = CapturePurpose.UNSPECIFIED,
    ) -> bool:
        """Capture one left/right HR pair and append its evidence to the session JSONL.

        ``purpose`` is an experiment label (pre/post/checkpoint/unspecified),
        not a change to WIRIS capture behavior.  The legacy zero-argument API
        remains equivalent to ``purpose="unspecified"``.
        """

        normalized_purpose = self._hr_capture_purpose(purpose)
        if not self._measurement_package_capture_allowed(
            prepare_if_needed=normalized_purpose is CapturePurpose.PRE
        ):
            self.panel.recording_status_label.setText(
                "Capture HR Still: 먼저 Session 폴더를 생성하세요"
            )
            return False
        if self._camera_photo_capture_active():
            self.panel.recording_status_label.setText("HR/OZ still 캡처가 이미 진행 중입니다")
            return False
        self._prepare_pnp_evidence_session()
        capture_id = self._next_capture_id(self.panel.pnp_photo_capture_records, "hr")
        request = self._new_hr_request_context(capture_id, normalized_purpose)
        self._pending_hr_requests[capture_id] = request
        target_infos = self._camera_photo_target_infos()
        if len(target_infos) != 2:
            self.panel._append_pnp_photo_capture_record(
                self._hr_capture_event_record(
                    {
                        "state": "not_started",
                        "error": "both WIRIS control hosts are required",
                        "stored_on_camera": False,
                        "ftp_downloaded": False,
                    },
                    capture_id=capture_id,
                    request=self._pending_hr_requests.pop(capture_id, None),
                )
            )
            return False
        self.panel.pnp_photo_capture_thread = QtCore.QThread(self.panel)
        self.panel.pnp_photo_capture_worker = WirisPnPPhotoCaptureWorker(
            target_infos,
            capture_id=capture_id,
            capture_purpose=normalized_purpose.value,
            completion_timeout_s=10.0,
            **self._wiris_ftp_worker_kwargs(),
        )
        self.panel.pnp_photo_capture_worker.moveToThread(self.panel.pnp_photo_capture_thread)
        self.panel.pnp_photo_capture_thread.started.connect(self.panel.pnp_photo_capture_worker.run)
        self.panel.pnp_photo_capture_worker.completed.connect(
            self.panel._handle_pnp_photo_capture_completed
        )
        self.panel.pnp_photo_capture_worker.failed.connect(
            lambda error, capture_id=capture_id: self.panel._handle_pnp_photo_capture_failed(
                capture_id, error
            )
        )
        self.panel.pnp_photo_capture_worker.finished.connect(self.panel.pnp_photo_capture_thread.quit)
        self.panel.pnp_photo_capture_worker.finished.connect(self.panel.pnp_photo_capture_worker.deleteLater)
        self.panel.pnp_photo_capture_thread.finished.connect(self.panel.pnp_photo_capture_thread.deleteLater)
        self.panel.pnp_photo_capture_thread.finished.connect(self.panel._clear_pnp_photo_capture_worker)
        self.panel.pnp_photo_capture_thread.start()
        self.panel.recording_status_label.setText(
            f"Capture HR Still {capture_id}: 좌/우 카메라 내부 저장 중"
        )
        return True

    def capture_pnp_oz_photos(
        self,
        purpose: CapturePurpose | str = CapturePurpose.UNSPECIFIED,
    ) -> bool:
        """Capture one visible operating-zoom still pair.

        OZ stills use the camera's normal visible-image storage command
        (``SIVI TRUE``) and are kept as a separate evidence stream from both
        the RTSP/MP4 session recording and the visible-HR capture stream.
        """

        normalized_purpose = self._hr_capture_purpose(purpose)
        if not self._measurement_package_capture_allowed(
            prepare_if_needed=normalized_purpose is CapturePurpose.PRE
        ):
            self.panel.recording_status_label.setText(
                "Capture OZ Still: 먼저 Session 폴더를 생성하세요"
            )
            return False
        if self._camera_photo_capture_active():
            self.panel.recording_status_label.setText("HR/OZ still 캡처가 이미 진행 중입니다")
            return False
        self._prepare_oz_evidence_session()
        capture_id = self._next_capture_id(
            getattr(self.panel, "oz_photo_capture_records", ()),
            "oz",
        )
        request = self._new_oz_request_context(capture_id, normalized_purpose)
        self._pending_oz_requests[capture_id] = request
        target_infos = self._camera_photo_target_infos()
        if len(target_infos) != 2:
            self.panel._append_oz_photo_capture_record(
                self._oz_capture_event_record(
                    {
                        "state": "not_started",
                        "error": "both WIRIS control hosts are required",
                        "stored_on_camera": False,
                        "ftp_downloaded": False,
                    },
                    capture_id=capture_id,
                    request=self._pending_oz_requests.pop(capture_id, None),
                )
            )
            return False
        self.panel.oz_photo_capture_thread = QtCore.QThread(self.panel)
        self.panel.oz_photo_capture_worker = WirisOZPhotoCaptureWorker(
            target_infos,
            capture_id=capture_id,
            capture_purpose=normalized_purpose.value,
            completion_timeout_s=10.0,
            **self._wiris_ftp_worker_kwargs(),
        )
        self.panel.oz_photo_capture_worker.moveToThread(self.panel.oz_photo_capture_thread)
        self.panel.oz_photo_capture_thread.started.connect(self.panel.oz_photo_capture_worker.run)
        self.panel.oz_photo_capture_worker.completed.connect(
            self.panel._handle_oz_photo_capture_completed
        )
        self.panel.oz_photo_capture_worker.failed.connect(
            lambda error, capture_id=capture_id: self.panel._handle_oz_photo_capture_failed(
                capture_id, error
            )
        )
        self.panel.oz_photo_capture_worker.finished.connect(self.panel.oz_photo_capture_thread.quit)
        self.panel.oz_photo_capture_worker.finished.connect(self.panel.oz_photo_capture_worker.deleteLater)
        self.panel.oz_photo_capture_thread.finished.connect(self.panel.oz_photo_capture_thread.deleteLater)
        self.panel.oz_photo_capture_thread.finished.connect(self.panel._clear_oz_photo_capture_worker)
        self.panel.oz_photo_capture_thread.start()
        self.panel.recording_status_label.setText(
            f"Capture OZ Still {capture_id}: 좌/우 카메라 내부 저장 중"
        )
        return True

    def _wiris_ftp_worker_kwargs(self, *, output_dir: Path | None = None) -> dict[str, object]:
        """FTP config shared by every WIRIS still-capture worker (HR/OZ/etc).

        The FTP server lives on the same WIRIS Enterprise host as the
        CPTR/ICPT command channel (only the port differs), so no separate
        host is needed here -- the worker reuses the host from target_infos.
        Each camera has its own local FTP account, so the port/user/password/
        remote dir/timeout are looked up per ``left``/``right`` label.

        ``output_dir`` lets a caller outside the Recording session (e.g.
        Calibration, whose save root is independent of any recording
        session) stage FTP downloads under its own directory instead of the
        default ``<recording_session_dir>/frames``.
        """

        settings = getattr(self.panel, "settings", None)
        if output_dir is None and self.panel.recording_session_id:
            output_dir = self.panel._recording_session_dir(self.panel.recording_session_id) / "frames"
        return {
            "ftp_enabled": bool(getattr(settings, "wiris_ftp_enabled", False)),
            "ftp_settings_by_label": {
                "left": {
                    "port": int(getattr(settings, "left_ftp_port", 21)),
                    "user": getattr(settings, "left_ftp_user", "anonymous"),
                    "password": getattr(settings, "left_ftp_password", ""),
                    "remote_dir": getattr(settings, "left_ftp_remote_dir", "/"),
                    "timeout_s": float(getattr(settings, "left_ftp_timeout_s", 10.0)),
                },
                "right": {
                    "port": int(getattr(settings, "right_ftp_port", 21)),
                    "user": getattr(settings, "right_ftp_user", "anonymous"),
                    "password": getattr(settings, "right_ftp_password", ""),
                    "remote_dir": getattr(settings, "right_ftp_remote_dir", "/"),
                    "timeout_s": float(getattr(settings, "right_ftp_timeout_s", 10.0)),
                },
            },
            "ftp_output_dir": output_dir,
        }

    def _wiris_ftp_status_text(self, ftp_downloaded: bool) -> str:
        settings = getattr(self.panel, "settings", None)
        if not bool(getattr(settings, "wiris_ftp_enabled", False)):
            return "FTP=disabled"
        return "FTP=ok" if ftp_downloaded else "FTP=failed"

    def _camera_photo_target_infos(self) -> tuple[tuple[str, str, int, float], ...]:
        return tuple(
            (
                label,
                host,
                self.panel._control_port_for_camera(label),
                self.panel.zoom_timeout_spin.value(),
            )
            for label, host in self.panel.zoom_hosts("both")
            if str(host or "").strip()
        )

    def _append_pnp_photo_capture_record(
        self, record: dict[str, object]
    ) -> dict[str, object]:
        # JSON round-tripping detaches nested mappings from signal payloads.
        # The append-only JSONL record and in-memory row therefore cannot be
        # changed by a worker mutating its original result after this call.
        normalized = json.loads(
            json.dumps(record, ensure_ascii=False, allow_nan=False, sort_keys=True)
        )
        normalized["session_id"] = self.panel.recording_session_id
        normalized["measurement_id"] = str(getattr(self.panel, "_measurement_id", "") or "")
        normalized["event_index"] = len(self.panel.pnp_photo_capture_records) + 1
        normalized.setdefault("recorded_utc_ns", time.time_ns())
        # Preserve the legacy spelling for downstream readers while making
        # the UTC basis explicit in new records.
        normalized.setdefault("recorded_unix_ns", normalized["recorded_utc_ns"])
        normalized.setdefault("schema", "workstation-hr-capture-event-v1")
        normalized.setdefault("capture_channel", "hr")
        normalized.setdefault("event_id", str(uuid.uuid4()))
        normalized.setdefault("purpose", CapturePurpose.UNSPECIFIED.value)
        normalized.setdefault("active_zoom", {
            side: dict(self.panel.actual_visible_zoom.get(side, {}))
            for side in ("left", "right")
        })
        normalized.setdefault("requested_zoom_level", self.panel.zoom_level_spin.value())
        normalized.setdefault("storage", "camera_internal")
        normalized.setdefault("ftp_downloaded", False)
        normalized["record_sha256"] = hashlib.sha256(
            json.dumps(
                normalized,
                ensure_ascii=False,
                allow_nan=False,
                sort_keys=True,
                separators=(",", ":"),
            ).encode("utf-8")
        ).hexdigest()
        output_dir = self.panel._recording_session_dir(self.panel.recording_session_id)
        output_dir.mkdir(parents=True, exist_ok=True)
        event_path = output_dir / "hr_captures.jsonl"
        append_jsonl_record(event_path, normalized)
        self.panel.pnp_photo_capture_records.append(normalized)
        run_event_writer = getattr(self.panel, "append_workstation_run_event", None)
        if callable(run_event_writer):
            try:
                run_event_writer(
                    "hr_capture_event",
                    normalized,
                    source="RecordingController",
                    coordinate_contract=str(
                        normalized.get("pixel_coordinate_contract", "not_applicable")
                    ),
                    source_timestamp_ns=normalized.get("request_utc_ns"),
                )
            except Exception as exc:
                logger = getattr(self.panel, "_log", None)
                if callable(logger):
                    logger(f"[Capture HR Still] run evidence linkage failed: {exc}")
        return json.loads(
            json.dumps(normalized, ensure_ascii=False, allow_nan=False, sort_keys=True)
        )

    def _append_oz_photo_capture_record(
        self, record: dict[str, object]
    ) -> dict[str, object]:
        """Append one camera-internal OZ still event to session evidence."""

        normalized = json.loads(
            json.dumps(record, ensure_ascii=False, allow_nan=False, sort_keys=True)
        )
        records = getattr(self.panel, "oz_photo_capture_records", None)
        if not isinstance(records, list):
            records = []
            self.panel.oz_photo_capture_records = records
        normalized["session_id"] = self.panel.recording_session_id
        normalized["measurement_id"] = str(getattr(self.panel, "_measurement_id", "") or "")
        normalized["event_index"] = len(records) + 1
        normalized.setdefault("recorded_utc_ns", time.time_ns())
        normalized.setdefault("recorded_unix_ns", normalized["recorded_utc_ns"])
        normalized.setdefault("schema", "workstation-oz-capture-event-v1")
        normalized.setdefault("capture_channel", "oz")
        normalized.setdefault("event_id", str(uuid.uuid4()))
        normalized.setdefault("purpose", CapturePurpose.UNSPECIFIED.value)
        normalized.setdefault(
            "active_zoom",
            {
                side: dict(self.panel.actual_visible_zoom.get(side, {}))
                for side in ("left", "right")
            },
        )
        normalized.setdefault("requested_zoom_level", self.panel.zoom_level_spin.value())
        normalized.setdefault("storage", "camera_internal")
        normalized.setdefault("ftp_downloaded", False)
        normalized.setdefault("image_enable_command", "SIVI TRUE")
        normalized["record_sha256"] = hashlib.sha256(
            json.dumps(
                normalized,
                ensure_ascii=False,
                allow_nan=False,
                sort_keys=True,
                separators=(",", ":"),
            ).encode("utf-8")
        ).hexdigest()
        output_dir = self.panel._recording_session_dir(self.panel.recording_session_id)
        output_dir.mkdir(parents=True, exist_ok=True)
        event_path = output_dir / "oz_captures.jsonl"
        append_jsonl_record(event_path, normalized)
        records.append(normalized)
        run_event_writer = getattr(self.panel, "append_workstation_run_event", None)
        if callable(run_event_writer):
            try:
                run_event_writer(
                    "oz_capture_event",
                    normalized,
                    source="RecordingController",
                    coordinate_contract=str(
                        normalized.get("pixel_coordinate_contract", "not_applicable")
                    ),
                    source_timestamp_ns=normalized.get("request_utc_ns"),
                )
            except Exception as exc:
                logger = getattr(self.panel, "_log", None)
                if callable(logger):
                    logger(f"[Capture OZ Still] run evidence linkage failed: {exc}")
        return json.loads(
            json.dumps(normalized, ensure_ascii=False, sort_keys=True)
        )

    def _handle_pnp_photo_capture_completed(self, result: object) -> None:
        record = dict(result or {}) if isinstance(result, dict) else {"state": "failed"}
        cameras = record.get("cameras")
        camera_states = (
            [str(item.get("state", "failed")) for item in cameras.values()]
            if isinstance(cameras, dict)
            else ["failed"]
        )
        record["state"] = "completed" if camera_states and all(
            state == "completed" for state in camera_states
        ) else "partial_or_failed"
        record["stored_on_camera"] = record["state"] == "completed"
        ftp_states = (
            [bool(item.get("ftp_downloaded")) for item in cameras.values()]
            if isinstance(cameras, dict)
            else []
        )
        record["ftp_downloaded"] = bool(ftp_states) and all(ftp_states)
        ftp_text = self._wiris_ftp_status_text(record["ftp_downloaded"])
        capture_id = str(record.get("capture_id") or "")
        record = self._hr_capture_event_record(
            record,
            capture_id=capture_id,
            request=self._pending_hr_requests.pop(capture_id, None),
        )
        self.panel._append_pnp_photo_capture_record(record)
        self.panel._log(
            f"[Capture HR Still] {capture_id} 카메라 내부 저장: state={record['state']}, {ftp_text}"
        )
        self.panel.recording_status_label.setText(
            f"Capture HR Still {capture_id}: {record['state']} · camera internal · {ftp_text}"
        )

    def _handle_pnp_photo_capture_failed(self, capture_id: str, error: str) -> None:
        self.panel._append_pnp_photo_capture_record(
            self._hr_capture_event_record(
                {
                    "state": "failed",
                    "error": str(error),
                    "stored_on_camera": False,
                    "ftp_downloaded": False,
                },
                capture_id=str(capture_id),
                request=self._pending_hr_requests.pop(str(capture_id), None),
            )
        )
        self.panel._log(f"[Capture HR Still] {capture_id} 촬영 실패: {error}")
        self.panel.recording_status_label.setText(f"Capture HR Still {capture_id}: failed ({error})")

    def _handle_oz_photo_capture_completed(self, result: object) -> None:
        record = dict(result or {}) if isinstance(result, dict) else {"state": "failed"}
        cameras = record.get("cameras")
        camera_states = (
            [str(item.get("state", "failed")) for item in cameras.values()]
            if isinstance(cameras, dict)
            else ["failed"]
        )
        record["state"] = "completed" if camera_states and all(
            state == "completed" for state in camera_states
        ) else "partial_or_failed"
        record["stored_on_camera"] = record["state"] == "completed"
        ftp_states = (
            [bool(item.get("ftp_downloaded")) for item in cameras.values()]
            if isinstance(cameras, dict)
            else []
        )
        record["ftp_downloaded"] = bool(ftp_states) and all(ftp_states)
        ftp_text = self._wiris_ftp_status_text(record["ftp_downloaded"])
        capture_id = str(record.get("capture_id") or "")
        record = self._oz_capture_event_record(
            record,
            capture_id=capture_id,
            request=self._pending_oz_requests.pop(capture_id, None),
        )
        self.panel._append_oz_photo_capture_record(record)
        self.panel._log(
            f"[Capture OZ Still] {capture_id} 카메라 내부 저장: state={record['state']}, {ftp_text}"
        )
        self.panel.recording_status_label.setText(
            f"Capture OZ Still {capture_id}: {record['state']} · camera internal · {ftp_text}"
        )

    def _handle_oz_photo_capture_failed(self, capture_id: str, error: str) -> None:
        self.panel._append_oz_photo_capture_record(
            self._oz_capture_event_record(
                {
                    "state": "failed",
                    "error": str(error),
                    "stored_on_camera": False,
                    "ftp_downloaded": False,
                },
                capture_id=str(capture_id),
                request=self._pending_oz_requests.pop(str(capture_id), None),
            )
        )
        self.panel._log(f"[Capture OZ Still] {capture_id} 촬영 실패: {error}")
        self.panel.recording_status_label.setText(f"Capture OZ Still {capture_id}: failed ({error})")

    def _clear_pnp_photo_capture_worker(self) -> None:
        self.panel.pnp_photo_capture_thread = None
        self.panel.pnp_photo_capture_worker = None
        continuation = getattr(
            self.panel, "_continue_measurement_package_finalization", None
        )
        if callable(continuation):
            continuation()

    def _clear_oz_photo_capture_worker(self) -> None:
        self.panel.oz_photo_capture_thread = None
        self.panel.oz_photo_capture_worker = None
        continuation = getattr(
            self.panel, "_continue_measurement_package_finalization", None
        )
        if callable(continuation):
            continuation()

    @staticmethod
    def _hr_capture_path_component(capture_id: str) -> str:
        raw = str(capture_id or "").strip()
        if not raw:
            raise ValueError("capture_id is required")
        safe = re.sub(r"[^A-Za-z0-9._-]+", "-", raw).strip(".-")
        if not safe:
            safe = "capture"
        if safe != raw:
            # Prevent different unsafe IDs from collapsing onto one directory.
            safe = f"{safe}-{uuid.uuid5(uuid.NAMESPACE_URL, raw).hex[:10]}"
        return safe[:128]

    def _known_hr_purpose(self, capture_id: str) -> CapturePurpose:
        for record in reversed(self.panel.pnp_photo_capture_records):
            if str(record.get("capture_id") or "") == str(capture_id):
                try:
                    return self._hr_capture_purpose(
                        record.get("purpose", CapturePurpose.UNSPECIFIED)
                    )
                except ValueError:
                    break
        return CapturePurpose.UNSPECIFIED

    def import_hr_files(
        self,
        capture_id: str,
        *,
        side: str | None = None,
        path: str | Path | None = None,
        left_path: str | Path | None = None,
        right_path: str | Path | None = None,
        camera_serial: str | None = None,
        left_camera_serial: str | None = None,
        right_camera_serial: str | None = None,
        camera_serials: dict[str, str] | None = None,
        purpose: CapturePurpose | str | None = None,
        captured_utc_ns: int | None = None,
        expected_sha256_by_side: dict[str, str] | None = None,
        expected_resolution_by_side: dict[str, tuple[int, int]] | None = None,
    ) -> dict[str, object]:
        """Validate and atomically retain exact HR files under the session.

        Callers may supply one ``side``/``path`` pair or ``left_path`` and/or
        ``right_path``.  File decoding, byte hashing, camera-serial validation,
        atomic writes, and event persistence all remain controller-owned; UI
        code should pass paths only.

        Pixel coordinates, if later annotated, are native raster ``u`` right,
        ``v`` down and map to an OpenCV optical RDF frame.  This import performs
        no geometry or frame conversion.
        """

        capture_text = str(capture_id or "").strip()
        capture_component = self._hr_capture_path_component(capture_text)
        single_supplied = side is not None or path is not None
        pair_supplied = left_path is not None or right_path is not None
        if single_supplied and pair_supplied:
            raise ValueError("use side/path or left_path/right_path, not both")
        sources: dict[str, Path] = {}
        if single_supplied:
            if side is None or path is None:
                raise ValueError("side and path must be supplied together")
            normalized_side = str(side).strip().lower()
            if normalized_side not in {"left", "right"}:
                raise ValueError("side must be left or right")
            sources[normalized_side] = Path(path)
        else:
            if left_path is not None:
                sources["left"] = Path(left_path)
            if right_path is not None:
                sources["right"] = Path(right_path)
        if not sources:
            raise ValueError("at least one HR file path is required")

        serials = {
            str(key).strip().lower(): str(value or "").strip()
            for key, value in dict(camera_serials or {}).items()
        }
        if left_camera_serial is not None:
            serials["left"] = str(left_camera_serial).strip()
        if right_camera_serial is not None:
            serials["right"] = str(right_camera_serial).strip()
        if camera_serial is not None:
            if len(sources) != 1:
                raise ValueError(
                    "camera_serial is only valid for one side; use per-side serials for a pair"
                )
            serials[next(iter(sources))] = str(camera_serial).strip()
        missing_serials = [side_name for side_name in sources if not serials.get(side_name)]
        if missing_serials:
            raise ValueError(
                "explicit camera serial required for HR import: " + ", ".join(missing_serials)
            )

        expected_hashes = dict(expected_sha256_by_side or {})
        expected_resolutions = dict(expected_resolution_by_side or {})
        imported_utc_ns = time.time_ns()
        imported = {}
        # Validate every source before materializing any destination.  This
        # prevents a bad second side from producing a misleading pair event.
        for side_name, source_path in sources.items():
            expected_resolution = expected_resolutions.get(side_name)
            if expected_resolution is not None and len(expected_resolution) != 2:
                raise ValueError("expected resolution must be a (width_px, height_px) pair")
            imported[side_name] = import_hr_image_file(
                source_path,
                side=side_name,
                camera_serial=serials[side_name],
                expected_sha256=expected_hashes.get(side_name),
                expected_width_px=(
                    None if expected_resolution is None else int(expected_resolution[0])
                ),
                expected_height_px=(
                    None if expected_resolution is None else int(expected_resolution[1])
                ),
                imported_utc_ns=imported_utc_ns,
            )

        self._prepare_pnp_evidence_session()
        session_root = Path(
            self.panel._recording_session_dir(self.panel.recording_session_id)
        ).resolve()
        destination_root = session_root / "hr" / "imported" / capture_component
        stored_files: dict[str, dict[str, object]] = {}
        for side_name, image in imported.items():
            suffix = Path(image.source_name).suffix.lower()
            if not re.fullmatch(r"\.[a-z0-9]{1,10}", suffix):
                suffix = ".bin"
            destination = destination_root / f"{side_name}-{image.sha256}{suffix}"
            stored = image.write_exact(destination)
            resolved = stored.resolve(strict=True)
            try:
                relative_path = resolved.relative_to(session_root)
            except ValueError as exc:  # defensive: destination must be session-owned
                raise RuntimeError("HR destination escaped the session directory") from exc
            stored_files[side_name] = {
                **image.to_mapping(),
                "session_relative_path": relative_path.as_posix(),
            }

        normalized_purpose = (
            self._known_hr_purpose(capture_text)
            if purpose is None
            else self._hr_capture_purpose(purpose)
        )
        known_request = next(
            (
                dict(record)
                for record in reversed(self.panel.pnp_photo_capture_records)
                if str(record.get("capture_id") or "") == capture_text
            ),
            {},
        )
        source_capture_utc_ns = (
            int(captured_utc_ns)
            if captured_utc_ns is not None
            else int(known_request.get("request_utc_ns") or imported_utc_ns)
        )
        time_semantics = (
            "caller_supplied_source_capture_utc"
            if captured_utc_ns is not None
            else (
                "Workstation_capture_request_utc"
                if known_request.get("request_utc_ns") is not None
                else "import_utc_fallback_not_source_capture_truth"
            )
        )
        capture_event = create_hr_capture_event(
            capture_text,
            purpose=normalized_purpose,
            left=imported.get("left"),
            right=imported.get("right"),
            captured_utc_ns=source_capture_utc_ns,
            note="exact HR file import into Workstation session",
        ).to_mapping()
        capture_event.update(
            {
                "event_type": "hr_file_imported",
                "state": "imported",
                "storage": "session_owned_exact_file",
                "ftp_downloaded": False,
                "stored_files": stored_files,
                "capture_time_semantics": time_semantics,
                "pixel_coordinate_contract": (
                    "native image raster u-right/v-down; OpenCV optical RDF "
                    "(right/down/forward); exact bytes retained; no conversion"
                ),
            }
        )
        for key in (
            "request_utc_ns",
            "request_monotonic_ns",
            "request_clock_anchor_uncertainty_ns",
            "request_utc_method",
            "request_session_delta_ns",
            "request_session_delta_s",
            "request_time_semantics",
        ):
            if key in known_request:
                capture_event[key] = known_request[key]
        return self._append_pnp_photo_capture_record(capture_event)

    def toggle_distance_recording(self) -> None:
        self.panel.toggle_session_recording()

    def toggle_video_recording(self) -> None:
        self.panel.toggle_session_recording()

    def save_distance_history_csv(
        self,
        *,
        final_clock_sync_snapshot: dict[str, object] | None = None,
    ) -> None:
        if self.panel.distance_recording_enabled:
            self.panel.recording_status_label.setText("세션 기록: 기록을 중지한 뒤 저장할 수 있습니다")
            self.panel._refresh_operation_header()
            return
        if self.panel._session_save_thread is not None:
            self.panel.recording_status_label.setText("세션 기록: 저장 작업이 이미 진행 중입니다")
            self.panel._refresh_operation_header()
            return
        if self.panel._session_save_finalized:
            self.panel.recording_status_label.setText("세션 기록: 현재 세션 파일은 이미 확정되었습니다")
            return
        if not self.panel.recording_session_id and not self.panel.distance_recording_rows:
            self.panel.recording_status_label.setText("세션 기록: 저장할 샘플 없음")
            self.panel._refresh_operation_header()
            return
        stamp = self.panel.recording_session_id or session_id_now()
        recovered_output = getattr(
            self.panel, "_recovered_session_output_dir", None
        )
        output_dir = (
            Path(recovered_output)
            if recovered_output is not None
            else self.panel._recording_session_dir(stamp)
        )
        method_evaluation_bundle_rows = getattr(
            self.panel, "method_evaluation_bundle_rows", None
        )
        for rows in (
            self.panel.distance_recording_rows,
            *self.panel.camera_recording_rows.values(),
            *self.panel.px4_recording_rows.values(),
            self.panel.gnss_base_recording_rows,
            self.panel.extrinsics_comparison_recording_rows,
            method_evaluation_bundle_rows,
        ):
            if isinstance(rows, SpoolRows):
                rows.close()
        if self.panel._recovered_session_metadata is None:
            metadata = self.panel._recording_session_metadata(clock_sync_snapshot=final_clock_sync_snapshot)
            write_recovery_metadata(output_dir, metadata)
            self.panel._recovered_session_metadata = json.loads(json.dumps(metadata))
        else:
            metadata = json.loads(json.dumps(self.panel._recovered_session_metadata))
            metadata.update(
                {
                    "matched_frame_pair_count": len(self.panel.distance_recording_rows),
                    "camera_left_gui_frame_count": len(self.panel.camera_recording_rows["left"]),
                    "camera_right_gui_frame_count": len(self.panel.camera_recording_rows["right"]),
                    "px4_left_sample_count": len(self.panel.px4_recording_rows["left"]),
                    "px4_right_sample_count": len(self.panel.px4_recording_rows["right"]),
                    "px4_drone_sample_count": len(
                        self.panel.px4_recording_rows.get("drone", ())
                    ),
                    "gnss_base_sample_count": len(self.panel.gnss_base_recording_rows),
                    "extrinsics_comparison_sample_count": len(self.panel.extrinsics_comparison_recording_rows),
                    "method_evaluation_bundle_count": len(
                        self.panel.method_evaluation_bundle_rows
                    ),
                    "finalization": {
                        "requested_unix_ns": time.time_ns(),
                        "recovered_retry": True,
                        "provenance_source": ".spool/session_metadata.json",
                    },
                }
            )
            # Automatic selection and annotation review may have happened
            # after the initial recovery metadata was written.  Refresh this
            # projection for the final static JSON while preserving the
            # previous recovery copy if the owner is unavailable after a
            # restart.
            optical_pose = self._recording_optical_pose_session_data()
            if optical_pose.get("state") not in {"UNAVAILABLE", "ERROR"}:
                previous_optical_pose = metadata.get("optical_pose")
                current_snapshot = optical_pose.get("snapshot")
                previous_snapshot = (
                    previous_optical_pose.get("snapshot")
                    if isinstance(previous_optical_pose, Mapping)
                    else None
                )
                evidence_keys = (
                    "frozen_frame_artifacts",
                    "auto_suggestions",
                    "accepted_auto_suggestion",
                    "annotation_review",
                    "candidates",
                    "active_profile_id",
                )
                current_has_evidence = isinstance(current_snapshot, Mapping) and any(
                    current_snapshot.get(key) for key in evidence_keys
                )
                previous_has_evidence = isinstance(previous_snapshot, Mapping) and any(
                    previous_snapshot.get(key) for key in evidence_keys
                )
                # A retry after an application restart has a fresh owner in
                # NO DATA state. Do not erase a previously persisted pose
                # review merely because that owner has not rehydrated it yet.
                if (
                    optical_pose.get("state") != "NO DATA"
                    or current_has_evidence
                    or not previous_has_evidence
                ):
                    metadata["optical_pose"] = optical_pose
        # Keep retry metadata aligned with the exact payload sent to the
        # worker.  This matters when finalization fails after a suggestion or
        # manual annotation was recorded.
        write_recovery_metadata(output_dir, metadata)
        self.panel._recovered_session_metadata = json.loads(json.dumps(metadata))
        if isinstance(self.panel.distance_recording_rows, SpoolRows):
            pair_rows = self.panel.distance_recording_rows
        else:
            pair_rows = [dict(row) for row in self.panel.distance_recording_rows]
        # RecordingController is a plain Python delegate, not a QObject.  The
        # hidden Workstation panel is the actual Qt lifecycle owner and must
        # parent the save thread so construction and shutdown are valid.
        thread = QtCore.QThread(self.panel)
        worker = SessionSaveWorker(
            output_dir=output_dir,
            metadata=metadata,
            pair_rows=pair_rows,
            camera_rows=dict(self.panel.camera_recording_rows),
            px4_rows=dict(self.panel.px4_recording_rows),
            gnss_base_rows=self.panel.gnss_base_recording_rows,
            extrinsics_comparison_rows=self.panel.extrinsics_comparison_recording_rows,
            method_evaluation_bundle_rows=method_evaluation_bundle_rows,
        )
        worker.moveToThread(thread)
        thread.started.connect(worker.run)
        worker.saved.connect(self.panel._handle_session_saved)
        worker.failed.connect(self.panel._handle_session_save_failed)
        worker.finished.connect(thread.quit, QtCore.Qt.DirectConnection)
        worker.finished.connect(worker.deleteLater)
        thread.finished.connect(self.panel._handle_session_save_thread_finished)
        self.panel._session_save_thread = thread
        self.panel._session_save_worker = worker
        self.panel._session_save_retry_required = False
        self.panel._session_save_last_error = ""
        self.panel.record_button.setEnabled(False)
        self.panel.save_history_button.setEnabled(False)
        self.panel.recording_status_label.setText(
            f"세션 기록: 저장 중, 매칭 프레임쌍={len(pair_rows)}개"
        )
        self.panel._log(f"[저장] 세션 파일을 백그라운드에서 확정 중입니다: {output_dir}")
        thread.start()
        self.panel._refresh_operation_header()

    @QtCore.pyqtSlot(object)
    def _handle_session_saved(self, saved_files: SavedSessionFiles) -> None:
        self.panel._log(
            "[저장] canonical JSONL 원본을 저장했습니다: "
            + ", ".join(str(path) for path in saved_files.authoritative_jsonl_files)
        )
        self.panel._log(
            "[저장] derived typed Parquet을 저장했습니다: "
            f"pairs={saved_files.measurement_pairs_parquet}, "
            f"camera={saved_files.camera_timeline_parquet}, "
            f"pixhawk={saved_files.pixhawk_timeline_parquet}, "
            f"gnss={saved_files.gnss_base_timeline_parquet}, "
            f"geometry={saved_files.geometry_comparison_parquet}"
        )
        self.panel._log(f"[저장] 측정 context를 저장했습니다: {saved_files.context_json}")
        self.panel._log(f"[저장] 측정 manifest를 저장했습니다: {saved_files.manifest}")
        if saved_files.review_pending:
            self.panel._log(
                "[저장] review.xlsx는 모든 측정을 통합해 Close Session 시 생성됩니다."
            )
        elif saved_files.review_error:
            self.panel._log(
                "[저장] JSONL 원본은 확정되었지만 review.xlsx 재생성은 실패했습니다: "
                + saved_files.review_error
            )
        else:
            self.panel._log(f"[저장] 세션 통합 XLSX를 저장했습니다: {saved_files.review_xlsx}")
        output_text = ", ".join(
            str(output) for output in saved_files.outputs if output.exists()
        )
        self.panel.recording_status_label.setText(
            "세션 기록 저장 완료: "
            f"매칭 프레임쌍={len(self.panel.distance_recording_rows)}개 - {output_text}"
        )
        self.panel._session_save_retry_required = False
        self.panel._session_save_last_error = ""
        self.panel._session_save_finalized = True
        self.panel._recovered_session_output_dir = None
        self.panel._refresh_operation_header()
        continuation = getattr(
            self.panel, "_continue_measurement_package_finalization", None
        )
        if callable(continuation):
            continuation()

    @QtCore.pyqtSlot(str)
    def _handle_session_save_failed(self, error: str) -> None:
        self.panel._session_save_retry_required = True
        self.panel._session_save_finalized = False
        self.panel._session_save_last_error = str(error)
        self.panel._log(f"[저장] 세션 파일 확정 실패; spool을 보존했습니다: {error}")
        self.panel.recording_status_label.setText(f"세션 저장 실패(재시도 가능): {error}")
        self.panel._refresh_operation_header()
        continuation = getattr(
            self.panel, "_continue_measurement_package_finalization", None
        )
        if callable(continuation):
            continuation()

    @QtCore.pyqtSlot()
    def _handle_session_save_thread_finished(self) -> None:
        thread = self.panel._session_save_thread
        self.panel._session_save_thread = None
        self.panel._session_save_worker = None
        self.panel._session_save_shutdown_wait_started_s = None
        self.panel._session_save_shutdown_timeout_logged = False
        if thread is not None:
            thread.deleteLater()
        self.panel.save_history_button.setEnabled(not self.panel._session_save_finalized)
        self.panel.record_button.setEnabled(not self.panel._session_save_retry_required)
        self.panel._refresh_operation_header()
        continuation = getattr(
            self.panel, "_continue_measurement_package_finalization", None
        )
        if callable(continuation):
            continuation()

    def _wait_for_session_save(self, timeout_ms: int = 15000) -> bool:
        thread = self.panel._session_save_thread
        if thread is None:
            self.panel._session_save_shutdown_wait_started_s = None
            self.panel._session_save_shutdown_timeout_logged = False
            return True
        try:
            running = thread.isRunning()
        except RuntimeError:
            self.panel._session_save_thread = None
            self.panel._session_save_worker = None
            return True
        now_s = time.monotonic()
        started_s = getattr(self.panel, "_session_save_shutdown_wait_started_s", None)
        if started_s is None:
            started_s = now_s
            self.panel._session_save_shutdown_wait_started_s = started_s
        timeout_s = max(0.0, float(timeout_ms) / 1000.0)
        if (
            running
            and now_s - float(started_s) >= timeout_s
            and not bool(getattr(self.panel, "_session_save_shutdown_timeout_logged", False))
        ):
            self.panel._log(
                "[저장] 종료 대기 시간이 초과되었습니다. 원본 spool과 manifest를 보존해 재시도할 수 있습니다."
            )
            self.panel._session_save_shutdown_timeout_logged = True
        # Keep the Qt event loop alive so saved/failed/finished signals can be
        # delivered.  The top-level closeEvent retries this lifecycle step.
        return False

    def _record_px4_mag_status_rows(self) -> list[dict[str, object]]:
        if not hasattr(self.panel, "px4_mag_managers"):
            return []
        now_s = time.monotonic()
        rows: list[dict[str, object]] = []
        for camera, label in (("left", "Pixhawk 1"), ("right", "Pixhawk 2")):
            manager = self.panel.px4_mag_managers.get(camera)
            if manager is None:
                continue
            if not self.panel._px4_mag_manager_has_data_source(camera):
                continue
            if not self.panel._sync_px4_mag_manager_store(camera):
                # A raw MAVLink sample does not expose CAL_MAG priority
                # readback; retain live telemetry but do not persist guessed
                # slot state from the local fallback defaults.
                continue
            status = self.panel._px4_mag_status_cache.get(camera)
            if status is None:
                # PARAM readback is a blocking MAVLink operation and is owned
                # by PixhawkParameterWorker.  Recording must never trigger a
                # hidden readback from the GUI thread.
                continue
            for slot in status.slots:
                rows.append(
                    {
                        "session_id": self.panel.recording_session_id,
                        "measurement_id": str(getattr(self.panel, "_measurement_id", "") or ""),
                        "session_started_unix_s": self.panel._fmt_record_float(self.panel.recording_session_started_wall_s, 6),
                        "recording_elapsed_s": self.panel._fmt_record_float(self.panel._recording_elapsed_s(now_s), 6),
                        "recorded_app_timestamp_s": self.panel._fmt_record_float(now_s, 6),
                        "pixhawk_side": camera,
                        "pixhawk_label": label,
                        "cal_mag_slot": f"CAL_MAG{slot.slot}",
                        "mag_slot": slot.slot,
                        "mag_device_id": "" if slot.device_id is None else str(slot.device_id),
                        "mag_device_type": slot.device_type,
                        "mag_interface": slot.interface,
                        "mag_is_rm3100": self.panel._fmt_record_bool(slot.is_rm3100),
                        "mag_priority": "" if slot.priority is None else str(slot.priority),
                        "mag_rotation": "" if slot.rotation is None else str(slot.rotation),
                        "mag_enabled": self.panel._fmt_record_bool(slot.enabled),
                        "mag_expected_selected": self.panel._fmt_record_bool(slot.expected_selected),
                        "mag_actual_selected": self.panel._fmt_record_bool(slot.actual_selected),
                        "mag_expected_selected_device_id": (
                            "" if status.expected_selected_device_id is None else str(status.expected_selected_device_id)
                        ),
                        "mag_actual_selected_device_id": (
                            "" if status.actual_selected_device_id is None else str(status.actual_selected_device_id)
                        ),
                        "mag_is_actual_rm3100": self.panel._fmt_record_bool(status.is_actual_rm3100),
                        "mag_armed": self.panel._fmt_record_bool(status.armed),
                        "mag_status": slot.status,
                        "mag_warning_text": status.warning_text,
                        "mag_rm3100_primary_id": status.rm3100_primary_id,
                        "mag_rm3100_backup_id": status.rm3100_backup_id,
                        "mag_param_uavcan_enable": status.params.get("UAVCAN_ENABLE"),
                        "mag_param_uavcan_sub_mag": status.params.get("UAVCAN_SUB_MAG"),
                        "mag_param_ekf2_mag_type": status.params.get("EKF2_MAG_TYPE"),
                        "mag_param_snapshot_json": json.dumps(status.params, sort_keys=True),
                    }
                )
        return rows

    def _record_px4_mag_status_snapshot(self, camera: str | None = None) -> None:
        """Persist the current MAG slot/priority selection state in the PX4 timeline."""
        for values in self.panel._record_px4_mag_status_rows():
            if camera is not None and values.get("pixhawk_side") != camera:
                continue
            self.panel._record_external_timeline_row(
                str(values["pixhawk_side"]),
                "mag_slot_status",
                {
                    "namespace": "",
                    "topic": "px4_mag_manager",
                    "sample_app_timestamp_s": values["recorded_app_timestamp_s"],
                    "sensor_slot": values["cal_mag_slot"],
                    **values,
                },
            )

    def _recording_camera_timing_diagnostics(self) -> dict[str, object]:
        """Summarize the raw camera timing evidence retained in the session spool."""

        def numeric_values(rows: list[dict[str, object]], field: str) -> list[float]:
            values: list[float] = []
            for row in rows:
                value = row.get(field)
                try:
                    parsed = float(value)
                except (TypeError, ValueError):
                    continue
                if isfinite(parsed):
                    values.append(parsed)
            return values

        def summary(values: list[float], *, scale: float = 1.0) -> dict[str, float | None]:
            if not values:
                return {"min": None, "median": None, "max": None}
            scaled = [value * scale for value in values]
            return {
                "min": min(scaled),
                "median": median(scaled),
                "max": max(scaled),
            }

        diagnostics: dict[str, object] = {}
        for camera in ("left", "right"):
            rows = [dict(row) for row in self.panel.camera_recording_rows[camera]]
            valid_rtcp_count = sum(
                1 for row in rows
                if row.get("rtcp_frame_ntp_unix_s") not in (None, "")
            )
            invalid_rtcp_count = sum(
                1 for row in rows
                if row.get("rtcp_timing_invalid_reason") not in (None, "")
            )
            clock_rate_values = numeric_values(rows, "rtcp_clock_rate_hz")
            media_rate_values = numeric_values(rows, "rtcp_camera_media_rate")
            estimated_media_clock_values: list[float] = []
            for row in rows:
                try:
                    clock_rate = float(row.get("rtcp_clock_rate_hz"))
                    media_rate = float(row.get("rtcp_camera_media_rate"))
                except (TypeError, ValueError):
                    continue
                if isfinite(clock_rate) and isfinite(media_rate):
                    estimated_media_clock_values.append(clock_rate * media_rate)
            rtp_timestamps = numeric_values(rows, "rtp_timestamp")
            diagnostics[camera] = {
                "frame_count": len(rows),
                "rtcp_valid_frame_count": valid_rtcp_count,
                "rtcp_missing_frame_count": max(0, len(rows) - valid_rtcp_count),
                "rtcp_invalid_frame_count": invalid_rtcp_count,
                "rtcp_valid_ratio": (
                    None if not rows else valid_rtcp_count / float(len(rows))
                ),
                "rtcp_receive_delay_ms": summary(
                    numeric_values(rows, "rtcp_frame_receive_delay_s"),
                    scale=1000.0,
                ),
                "rtcp_transport_delay_ms": summary(
                    numeric_values(rows, "rtcp_frame_transport_delay_s"),
                    scale=1000.0,
                ),
                "rtcp_clock_rate_hz": summary(clock_rate_values),
                "rtcp_camera_ntp_rate": summary(numeric_values(rows, "rtcp_camera_ntp_rate")),
                "rtcp_camera_media_rate": summary(media_rate_values),
                "estimated_rtp_clock_rate_hz": summary(estimated_media_clock_values),
                "rtp_timestamp_first": None if not rtp_timestamps else rtp_timestamps[0],
                "rtp_timestamp_last": None if not rtp_timestamps else rtp_timestamps[-1],
            }
        return diagnostics

    def _recording_optical_pose_session_data(self) -> dict[str, object]:
        """Return JSON-safe optical-pose state for session_static.json.

        The live snapshot can contain QImages for the annotation UI.  Session
        metadata stores only frame identities, artifact paths, candidates,
        automatic proposals, and annotation provenance; the lossless pixels
        remain in the session-owned native-frame artifacts.
        """

        evidence_files = {
            "run_events": "run_events.jsonl",
            "native_frames": "native_frames.jsonl",
            "native_frame_root": "frames/optical-pose-burst",
        }
        reader = getattr(self.panel, "workstation_optical_pose_snapshot", None)
        if not callable(reader):
            return {
                "schema": "workstation-optical-pose-session-v1",
                "state": "UNAVAILABLE",
                "snapshot": {},
                "evidence_files": evidence_files,
                "reason": "optical pose owner snapshot is unavailable",
            }
        try:
            try:
                snapshot = reader(include_frames=False)
            except TypeError:
                # Keep compatibility with small test/legacy owners that do
                # not yet expose the include_frames keyword.
                snapshot = reader()
        except Exception as exc:
            return {
                "schema": "workstation-optical-pose-session-v1",
                "state": "ERROR",
                "snapshot": {},
                "evidence_files": evidence_files,
                "reason": str(exc),
            }
        if not isinstance(snapshot, Mapping):
            return {
                "schema": "workstation-optical-pose-session-v1",
                "state": "ERROR",
                "snapshot": {},
                "evidence_files": evidence_files,
                "reason": "optical pose owner snapshot is not a mapping",
            }
        projection = dict(snapshot)
        # Never allow a UI QImage to cross into recovery/static JSON, even if
        # a legacy owner ignores include_frames=False.
        projection.pop("frozen_frames", None)
        projection.pop("frozen_native_frames", None)
        try:
            safe_projection = json.loads(
                json.dumps(projection, ensure_ascii=False)
            )
        except (TypeError, ValueError) as exc:
            return {
                "schema": "workstation-optical-pose-session-v1",
                "state": "ERROR",
                "snapshot": {},
                "evidence_files": evidence_files,
                "reason": f"snapshot is not JSON-safe: {exc}",
            }
        return {
            "schema": "workstation-optical-pose-session-v1",
            "state": str(snapshot.get("state") or "NO DATA"),
            "snapshot": safe_projection,
            "evidence_files": evidence_files,
        }

    def _recording_session_metadata(
        self,
        *,
        clock_sync_snapshot: dict[str, object] | None = None,
    ) -> dict[str, object]:
        stamp = self.panel.recording_session_id or time.strftime("%Y%m%d_%H%M%S")
        clock_sync_snapshot = clock_sync_snapshot or self.panel._camera_time_sync_snapshot("metadata_read")
        camera_time_metrics = clock_sync_snapshot["metrics"]
        if not isinstance(camera_time_metrics, dict):
            camera_time_metrics = {}
        triangulation_source = self.panel._triangulation_extrinsics_source()
        try:
            measurement_methods = self.panel.workstation_measurement_methods_snapshot()
        except (AttributeError, RuntimeError, TypeError, ValueError):
            measurement_methods = {}
        selected_method_row: dict[str, object] = {}
        if isinstance(measurement_methods, dict):
            selected_id = str(measurement_methods.get("selected_method_id") or "")
            selected_instance = str(measurement_methods.get("selected_instance_id") or "")
            method_rows = measurement_methods.get("all_methods", ())
            if not isinstance(method_rows, (tuple, list)) or not method_rows:
                method_rows = measurement_methods.get("ready_methods", ())
            selected_method_row = next(
                (
                    dict(row)
                    for row in method_rows
                    if isinstance(row, Mapping)
                    and str(row.get("method_id") or "") == selected_id
                    and (
                        not selected_instance
                        or str(row.get("instance_id") or "") == selected_instance
                    )
                ),
                {},
            )
        selected_method_id = str(selected_method_row.get("method_id") or "")
        selected_uses_pixhawk = bool(selected_method_row.get("requires_live_pixhawk"))
        selected_uses_gnss = bool(selected_method_row.get("requires_live_gnss"))
        selected_relative_pose = bool(selected_method_id and selected_method_id != "pixhawk_rtk_live")
        if selected_method_id:
            baseline_source = (
                "live_gnss"
                if selected_uses_gnss
                else "checkerboard"
                if selected_method_id == CHECKERBOARD_STEREO
                else "resolved_relative_pose"
            )
        else:
            _baseline_m, baseline_source, _baseline_warnings = self.panel._active_baseline_m()
        session_owner = getattr(self.panel, "_measurement_session", None)
        session_snapshotter = getattr(session_owner, "snapshot", None)
        try:
            session_snapshot = session_snapshotter() if callable(session_snapshotter) else {}
        except Exception:
            session_snapshot = {}
        session_snapshot = session_snapshot if isinstance(session_snapshot, Mapping) else {}
        return {
            "session_id": self.panel.recording_session_id,
            "session_created_utc_ns": session_snapshot.get("created_utc_ns"),
            "measurement_id": str(getattr(self.panel, "_measurement_id", "") or ""),
            "measurement_package_id": str(
                getattr(self.panel, "_measurement_id", "") or ""
            ),
            "recording_mode": str(
                getattr(self.panel, "_recording_mode", "manual") or "manual"
            ),
            "session_dir": str(self.panel._recording_session_dir(stamp)),
            "measurement_dir": str(self.panel._recording_session_dir(stamp)),
            "session_started_unix_s": self.panel.recording_session_started_wall_s,
            "session_started_unix_ns": self.panel.recording_session_started_unix_ns,
            "session_started_monotonic_ns": self.panel.recording_session_started_monotonic_ns,
            "session_time_anchor_uncertainty_ns": self.panel.recording_session_anchor_uncertainty_ns,
            "measurement_started_unix_s": self.panel.recording_session_started_wall_s,
            "measurement_started_unix_ns": self.panel.recording_session_started_unix_ns,
            "measurement_started_monotonic_ns": self.panel.recording_session_started_monotonic_ns,
            "measurement_time_anchor_uncertainty_ns": self.panel.recording_session_anchor_uncertainty_ns,
            "schema_version": "stereowiris-paired-frame-v3",
            "row_contract": "one row per terminal left/right matched detection outcome",
            "terminal_record_id_contract": (
                "attempt:<measurement_attempt_sha256> or "
                "blocked:<blocked_pair_sha256>"
            ),
            "left_video_path": str(self.panel.video_recording_paths.get("left", "")),
            "right_video_path": str(self.panel.video_recording_paths.get("right", "")),
            "matched_frame_pair_count": len(self.panel.distance_recording_rows),
            "camera_left_gui_frame_count": len(self.panel.camera_recording_rows["left"]),
            "camera_right_gui_frame_count": len(self.panel.camera_recording_rows["right"]),
            "px4_left_sample_count": len(self.panel.px4_recording_rows["left"]),
            "px4_right_sample_count": len(self.panel.px4_recording_rows["right"]),
            "px4_drone_sample_count": len(self.panel.px4_recording_rows.get("drone", ())),
            "gnss_base_sample_count": len(self.panel.gnss_base_recording_rows),
            "extrinsics_comparison_sample_count": len(self.panel.extrinsics_comparison_recording_rows),
            "method_evaluation_bundle_count": len(
                self.panel.method_evaluation_bundle_rows
            ),
            "device_timeline_contract": {
                "camera_row": "one row per frame delivered to the GUI frame_changed callback",
                "camera_not_guaranteed": "not a one-to-one index of encoded MP4 frames",
                "mp4_frame_index_contract": "not_available",
                "px4_row": "one row per PX4 telemetry callback received by the GUI",
                "sample_monotonic": "producer/app receive CLOCK_MONOTONIC timestamp",
                "session_utc": "derived from session host Unix/monotonic anchor",
                "source_utc": "RTCP NTP or GNSS UTC only when explicitly valid",
                "mavlink_provenance": (
                    "endpoint, link-open/heartbeat, link target system/component, and received packet "
                    "source system/component/sequence/message id are recorded when available"
                ),
            },
            "camera_time_sync_status": camera_time_metrics["status"],
            "camera_time_sync_basis": camera_time_metrics["basis"],
            "camera_time_sync_warning": camera_time_metrics["warning"],
            "stream_sync_state": self.panel.camera_sync_status,
            "stream_sync_reason": self.panel.camera_sync_reason,
            "camera_time_sync_observation": clock_sync_snapshot,
            "camera_time_sync_observations": {
                "start": self.panel._recording_clock_sync_start_observation,
                "end": self.panel._recording_clock_sync_end_observation,
            },
            "camera_time_sync_drift": self.panel._camera_time_sync_drift_metadata(),
            "pnp_hr_photo_capture": {
                "storage": "camera_internal",
                "ftp_downloaded": False,
                "event_file": "hr_captures.jsonl",
                "capture_command": "SIHR TRUE + CPTR + ICPT",
                "captures": list(getattr(self.panel, "pnp_photo_capture_records", ())),
            },
            "oz_photo_capture": {
                "storage": "camera_internal",
                "ftp_downloaded": False,
                "event_file": "oz_captures.jsonl",
                "capture_command": "SIVI TRUE + CPTR + ICPT",
                "captures": list(getattr(self.panel, "oz_photo_capture_records", ())),
            },
            "camera_timing_diagnostics": self.panel._recording_camera_timing_diagnostics(),
            "pc_utc_sync": self.panel._last_wiris_pc_utc_sync,
            "gnss_operation": self.panel.gnss_operation_manager.metadata(),
            "gnss_operation_config": self.panel.gnss_operation_manager.config.to_mapping(),
            "gnss_fixed_base_profile": (
                None
                if self.panel.gnss_fixed_base_profile is None
                else self.panel.gnss_fixed_base_profile.to_mapping()
            ),
            "rtcp_pair_delta_s": camera_time_metrics["pair_delta_s"],
            "rtcp_pc_offset_max_ms": camera_time_metrics["max_pc_offset_ms"],
            "rtcp_frame_transit_max_ms": camera_time_metrics["max_frame_transit_delay_ms"],
            "left_rtcp_frame_transit_ms": camera_time_metrics["left_frame_transit_delay_ms"],
            "right_rtcp_frame_transit_ms": camera_time_metrics["right_frame_transit_delay_ms"],
            "calculation": {
                "triangulation_extrinsics_source": triangulation_source,
                "measurement_method_id": str(
                    measurement_methods.get("selected_method_id", "")
                    if isinstance(measurement_methods, dict)
                    else ""
                ),
                "measurement_method_instance_id": str(
                    measurement_methods.get("selected_instance_id", "")
                    if isinstance(measurement_methods, dict)
                    else ""
                ),
                "measurement_method_selection_reason": str(
                    measurement_methods.get("selection_reason", "")
                    if isinstance(measurement_methods, dict)
                    else ""
                ),
                "measurement_method_fallback_order": (
                    list(measurement_methods.get("fallback_order", ()))
                    if isinstance(measurement_methods, dict)
                    else []
                ),
                "measurement_method_fill_policy": (
                    dict(measurement_methods.get("fill_policy", {}))
                    if isinstance(measurement_methods, dict)
                    else {}
                ),
                "distance_mode": self.panel._distance_mode_label(),
                "baseline_source": baseline_source,
                "measurement_path": self.panel._measurement_path_label(),
                "method": "two_ray_closest_midpoint",
                "measurement_frame": MEASUREMENT_FRAME,
                "measurement_axis_contract": MEASUREMENT_AXIS_CONTRACT,
                "public_vector_contract": PUBLIC_MEASUREMENT_VECTOR_CONTRACT,
                "legacy_xyz_alias_contract": LEGACY_XYZ_ALIAS_CONTRACT,
                "baseline_frame_scope": "measurement_attempt",
                "baseline_frame_origin_contract": (
                    "midpoint_of_left_optical_origin_and_selected_right_origin_proxy"
                ),
                "baseline_frame_origin_model": (
                    "resolved_relative_pose_optical_centers"
                    if selected_relative_pose
                    else "gps_rtk_antenna_baseline_proxy_no_lever_arm"
                ),
                "baseline_frame_axis_contract": (
                    "right=left_to_right_center; forward=projected_average_optical_forward; "
                    "down=forward_cross_right"
                ),
                "attitude_timing_policy": ATTITUDE_TIMING_POLICY,
                "attitude_timing_blocks_measurement": selected_uses_pixhawk,
                "attitude_stale_age_threshold_s": ATTITUDE_STALE_AGE_S,
                "attitude_very_stale_age_threshold_s": ATTITUDE_VERY_STALE_AGE_S,
                "attitude_stale_pair_delta_threshold_s": ATTITUDE_STALE_PAIR_DELTA_S,
                "attitude_very_stale_pair_delta_threshold_s": ATTITUDE_VERY_STALE_PAIR_DELTA_S,
                "mount_assumption_model": (
                    "checkerboard-stereo-extrinsics-v1"
                    if selected_method_id == CHECKERBOARD_STEREO
                    else "hybrid-candidate-live-components-v1"
                    if selected_relative_pose and (selected_uses_pixhawk or selected_uses_gnss)
                    else "optical-pose-relative-r-t-v1"
                    if selected_relative_pose
                    else "gps-rtk-baseline-only-v1"
                ),
            },
            "checkerboard_stereo_extrinsics": self.panel._record_current_checkerboard_extrinsics_parameters(),
            "timestamp_contract": {
                "manufacturer_confirmed": False,
                "camera_captured_timestamp_clock": "WIRIS RTC checked by STUS/GDTI",
                "rtp_rtcp_timestamp_clock": "RTP relative media counter; RTCP Sender Report NTP-derived stream time",
                "rtp_timestamp_note": "RTP timestamp is a 90 kHz media clock counter; mapping RTCP NTP to WIRIS GDTI/UTC is unverified.",
                "preferred_frame_time_field": "rtcp_frame_ntp_unix_s",
                "legacy_equivalent_field": "camera_rtc_frame_unix_s",
                "clock_offset_check": "Use the 시간 tab RTCP/NTP check plus direct WIRIS GDTI validation before absolute UTC claims.",
            },
            "intrinsics": self.panel._record_current_intrinsic_parameters(),
            "dual_geometry": {
                "state": "nominal_active_enabled",
                "nominal_profile_sha256": (
                    ""
                    if not isinstance(
                        getattr(self.panel, "recording_nominal_intrinsics_snapshot", None),
                        StereoIntrinsicsSnapshot,
                    )
                    else self.panel.recording_nominal_intrinsics_snapshot.content_sha256
                ),
                "nominal_intrinsics_profile_sha256": (
                    ""
                    if not isinstance(
                        getattr(self.panel, "recording_nominal_intrinsics_snapshot", None),
                        StereoIntrinsicsSnapshot,
                    )
                    else self.panel.recording_nominal_intrinsics_snapshot.intrinsics_content_sha256
                ),
                "nominal_optical_pose_profile_hash": str(
                    getattr(self.panel, "recording_nominal_optical_pose_profile_hash", "") or ""
                ),
                "active_selection_id": str(
                    getattr(self.panel, "_active_optical_pose_selection_id", "") or ""
                ),
                "selection_effective_from_pair_sequence": getattr(
                    self.panel, "_active_optical_pose_effective_from_pair_sequence", None
                ),
                "same_detection_evaluated_twice": True,
                "coordinate_contract": "both evaluations use OpenCV optical RDF and baseline-center FRD output",
            },
            "optical_pose": self._recording_optical_pose_session_data(),
            "stream": {
                "left_rtsp_url": self.panel.left_url_edit.text().strip(),
                "right_rtsp_url": self.panel.right_url_edit.text().strip(),
                "backend": self.panel.stream_backend_combo.currentData(),
                "transport": self.panel.rtsp_transport_combo.currentData(),
                "decoder": self.panel.decoder_combo.currentData(),
                "rtsp_latency_ms": self.panel.rtsp_latency_spin.value(),
            },
        }

    def _remove_distance_history_point(self, point: DistanceHistoryPoint) -> None:
        point_key = self.panel._distance_history_key(point)
        filtered_history: deque[DistanceHistoryPoint] = deque(maxlen=self.panel.distance_history.maxlen)
        removed_history = False
        for candidate in self.panel.distance_history:
            if self.panel._distance_history_key(candidate) == point_key:
                removed_history = True
                continue
            filtered_history.append(candidate)
        if not removed_history:
            self.panel._log("[Depth] 거리 그래프 숨김 요청이 현재 뷰와 일치하지 않았습니다.")
            return

        self.panel.distance_history = filtered_history
        append_event = getattr(self.panel, "append_workstation_run_event", None)
        if callable(append_event):
            try:
                append_event(
                    "measurement_point_hidden",
                    {
                        "sample_key": list(point_key),
                        "timestamp_s": point.timestamp_s,
                        "distance_m": point.distance_m,
                        "quality": point.quality,
                        "method": point.method,
                        "view_only": True,
                    },
                    source="RecordingController",
                    coordinate_contract="baseline_center_frd view annotation; canonical pair remains append-only",
                )
            except Exception as exc:
                self.panel._log(f"[Depth] 그래프 숨김 감사 이벤트 기록 실패: {exc}")
        self.panel._refresh_recording_status_label()
        self.panel._refresh_operation_header()
        self.panel._log(
            f"[Depth] 거리 그래프 점을 뷰에서만 숨겼습니다: {point.distance_m:.2f} m "
            f"({point.method}, quality={point.quality}); canonical 원본 pair/spool은 보존됩니다."
        )




    def _pair_recording_base_row(self, measurement_attempt: MeasurementAttempt) -> dict[str, object]:
        pair_mid_monotonic_s = measurement_attempt.detection_midpoint_monotonic_s
        pair_mid_monotonic_ns = int(round(pair_mid_monotonic_s * 1_000_000_000.0))
        session_elapsed_ns = (
            None
            if self.panel.recording_session_started_monotonic_ns is None
            else max(0, pair_mid_monotonic_ns - self.panel.recording_session_started_monotonic_ns)
        )
        row: dict[str, object] = {
            "session_id": self.panel.recording_session_id,
            "measurement_id": str(getattr(self.panel, "_measurement_id", "") or ""),
            "measurement_package_id": str(
                getattr(self.panel, "_measurement_id", "") or ""
            ),
            "terminal_record_id": measurement_attempt.terminal_record_id,
            "measurement_attempt_id": str(measurement_attempt.attempt_id),
            "measurement_attempt_session_id": measurement_attempt.attempt_session_id,
            "measurement_attempt_sha256": measurement_attempt.canonical_sha256(),
            "measurement_attempt_canonical_json": measurement_attempt.canonical_json(),
            "measurement_detection_pair_key": json.dumps(
                measurement_attempt.detection_pair_key, separators=(",", ":")
            ),
            "schema_version": "stereowiris-paired-frame-v3",
            "measurement_attempt_content_sha256": measurement_attempt.content_sha256(),
            "measurement_attempt_instance_sha256": measurement_attempt.instance_sha256(),
            "measurement_attempt_timestamp_basis": measurement_attempt.timestamp_basis,
            "measurement_attempt_source_quality": measurement_attempt.source_quality,
            "measurement_attempt_warnings": "; ".join(measurement_attempt.warnings),
            "measurement_frame": MEASUREMENT_FRAME,
            "measurement_axis_contract": MEASUREMENT_AXIS_CONTRACT,
            "public_vector_contract": PUBLIC_MEASUREMENT_VECTOR_CONTRACT,
            "legacy_xyz_alias_contract": LEGACY_XYZ_ALIAS_CONTRACT,
            "baseline_frame_scope": "measurement_attempt",
            "baseline_frame_instance_id": measurement_attempt.instance_sha256(),
            "baseline_frame_origin_contract": (
                "midpoint_of_left_optical_origin_and_selected_right_origin_proxy"
            ),
                "baseline_frame_origin_model": (
                    "resolved_relative_pose_optical_centers"
                    if measurement_attempt.measurement_geometry_kind == "relative_pose"
                    else "gps_rtk_antenna_baseline_proxy_no_lever_arm"
                ),
            "baseline_frame_axis_construction": (
                "right=left_to_right_center;forward=projected_average_optical_forward;"
                "down=forward_cross_right"
            ),
            "measurement_valid": "false",
            "pair_status": "matched_geometry_pending",
            "session_started_unix_ns": self.panel.recording_session_started_unix_ns,
            "session_started_monotonic_ns": self.panel.recording_session_started_monotonic_ns,
            "recorded_unix_ns": int(round(measurement_attempt.created_wall_s * 1_000_000_000.0)),
            "pair_app_mid_monotonic_ns": pair_mid_monotonic_ns,
            "session_elapsed_ns": session_elapsed_ns,
            "session_elapsed_s": (
                "" if session_elapsed_ns is None else session_elapsed_ns * 1e-9
            ),
            "left_video_path": str(self.panel.video_recording_paths.get("left", "")),
            "right_video_path": str(self.panel.video_recording_paths.get("right", "")),
            "selected_left_pairing_timestamp_s": self.panel._fmt_record_float(
                measurement_attempt.left_detection.pairing_timestamp_s, 9
            ),
            "selected_right_pairing_timestamp_s": self.panel._fmt_record_float(
                measurement_attempt.right_detection.pairing_timestamp_s, 9
            ),
        }
        row.update(self._persist_native_measurement_pair(measurement_attempt))
        row.update(self.panel._record_bbox_parameters(measurement_attempt))
        row.update(self.panel._record_measurement_sync_timestamp_parameters(measurement_attempt))
        row.update(self.panel._record_intrinsic_parameters(measurement_attempt))
        row.update(self.panel._record_frame_timestamp_parameters(measurement_attempt))
        row.update(self.panel._record_live_telemetry_snapshot_parameters(measurement_attempt))
        row.update(self.panel._measurement_gnss_record_metadata())
        row.update(self.panel._record_external_parameters(estimate=None, measurement_attempt=measurement_attempt))
        row.update(self.panel._record_reference_delta_parameters(measurement_attempt))
        row.update(self._record_dual_geometry_parameters(measurement_attempt))
        return row

    def _snapshot_current_intrinsics(self) -> StereoIntrinsicsSnapshot | None:
        """Freeze the run-start K,D and checkerboard R,t for nominal comparison."""

        result = getattr(self.panel, "active_intrinsics", None)
        if result is None:
            return None
        try:
            zoom = int(self.panel.zoom_level_spin.value())
            return StereoIntrinsicsSnapshot.from_result(
                result,
                active_zoom_level=zoom,
                selected_profile_path=str(
                    getattr(self.panel, "_active_intrinsics_profile_path", "")
                    or f"memory:nominal-calibration:zoom-{zoom:02d}"
                ),
                selected_extrinsics_profile_path=str(
                    getattr(self.panel, "_active_extrinsics_profile_path", "") or ""
                ),
                intrinsics_source_zoom_level=getattr(
                    self.panel, "_active_intrinsics_source_zoom_level", None
                ),
                extrinsics_source_zoom_level=getattr(
                    self.panel, "_active_extrinsics_source_zoom_level", None
                ),
                selection_mode=str(
                    getattr(self.panel, "_active_calibration_selection_mode", "")
                    or "legacy_combined"
                ),
            )
        except (AttributeError, TypeError, ValueError) as exc:
            logger = getattr(self.panel, "_log", None)
            if callable(logger):
                logger(f"[저장] nominal K,D snapshot unavailable: {exc}")
            return None

    def _record_dual_geometry_parameters(
        self, measurement_attempt: MeasurementAttempt
    ) -> dict[str, object]:
        nominal = getattr(self.panel, "recording_nominal_intrinsics_snapshot", None)
        active = measurement_attempt.intrinsics
        if (
            not isinstance(nominal, StereoIntrinsicsSnapshot)
            or not isinstance(active, StereoIntrinsicsSnapshot)
            or measurement_attempt.measurement_geometry_kind != "relative_pose"
        ):
            return {
                "dual_geometry_state": (
                    "not_applicable_live_pixhawk_path"
                    if measurement_attempt.measurement_geometry_kind != "relative_pose"
                    else "nominal_or_active_profile_unavailable"
                ),
                "nominal_profile_sha256": "" if not isinstance(nominal, StereoIntrinsicsSnapshot) else nominal.content_sha256,
                "active_profile_sha256": "" if not isinstance(active, StereoIntrinsicsSnapshot) else active.content_sha256,
            }
        try:
            nominal_eval, active_eval = evaluate_nominal_active(
                measurement_attempt,
                nominal_intrinsics=nominal,
                active_intrinsics=active,
                active_selection_id=str(
                    getattr(self.panel, "_active_optical_pose_selection_id", "") or ""
                ),
                active_effective_from_pair_sequence=getattr(
                    self.panel, "_active_optical_pose_effective_from_pair_sequence", None
                ),
            )
            fields = {}
            fields.update(nominal_eval.to_record_fields())
            fields.update(active_eval.to_record_fields())
            fields["dual_geometry_state"] = "evaluated"
            fields["nominal_optical_pose_profile_hash"] = str(
                getattr(self.panel, "recording_nominal_optical_pose_profile_hash", "") or ""
            )
            return fields
        except (TypeError, ValueError, RuntimeError) as exc:
            return {
                "dual_geometry_state": "evaluation_error",
                "dual_geometry_error": str(exc),
                "nominal_profile_sha256": nominal.content_sha256,
                "active_profile_sha256": active.content_sha256,
            }

    def _persist_native_detection_pair(
        self,
        *,
        left_detection: object,
        right_detection: object,
        pair_id: str,
        purpose: str,
    ) -> dict[str, object]:
        """Persist evaluated-pair source pixels without resizing or overlays.

        The producer's OpenCV/native image coordinates are retained.  Missing
        cached frames do not block raw attempt recording; the explicit status
        lets offline analysis distinguish absence from a hash/path mismatch.
        """

        if not self.panel.recording_session_id:
            return {"native_pair_status": "session_id_missing"}
        resolver = getattr(self.panel, "_recording_session_dir", None)
        if not callable(resolver):
            return {"native_pair_status": "session_directory_owner_missing"}
        store = NativeFrameStore(resolver(self.panel.recording_session_id))
        output: dict[str, object] = {"native_pair_status": "complete"}
        missing: list[str] = []
        for side, detection in (
            ("left", left_detection),
            ("right", right_detection),
        ):
            pane = getattr(self.panel, f"{side}_pane", None)
            getter = getattr(pane, "native_measurement_frame", None)
            # CameraPane's native cache is keyed by the producer's
            # app-receive monotonic timestamp.  pairing_timestamp_s may instead
            # be RTCP/NTP Unix time and is only valid for stereo pairing.
            cached = (
                getter(detection.app_receive_monotonic_s)
                if callable(getter)
                else None
            )
            if cached is None:
                missing.append(side)
                continue
            _timestamp_s, image, metadata = cached
            try:
                artifact = store.persist(
                    side=side,
                    purpose=purpose,
                    frame_id=pair_id,
                    image=image,
                    metadata=metadata,
                )
            except Exception as exc:
                output[f"native_{side}_error"] = str(exc)
                missing.append(side)
                continue
            output.update(
                {
                    f"native_{side}_frame_id": artifact.frame_id,
                    f"native_{side}_png_path": artifact.relative_path,
                    f"native_{side}_png_sha256": artifact.sha256,
                    f"native_{side}_width_px": artifact.width,
                    f"native_{side}_height_px": artifact.height,
                    f"native_{side}_timestamp_reference": artifact.timestamp_reference,
                    f"native_{side}_source_frame_id": artifact.source_native_frame_id,
                }
            )
        if missing:
            output["native_pair_status"] = "missing:" + ",".join(missing)
        return output

    def _persist_native_measurement_pair(
        self,
        measurement_attempt: MeasurementAttempt,
    ) -> dict[str, object]:
        return self._persist_native_detection_pair(
            left_detection=measurement_attempt.left_detection,
            right_detection=measurement_attempt.right_detection,
            pair_id=f"pair-{measurement_attempt.attempt_id}",
            purpose="accepted-measurement",
        )

    @staticmethod
    def _measurement_result_evidence(
        *,
        attempt_sha256: str,
        status: str,
        valid: bool,
        method: str,
        quality: str,
        warnings: tuple[str, ...] | list[str] | str,
        vector_frd_range_m: tuple[float | None, float | None, float | None, float | None],
        policy_id: str = "field_measurement_v1",
        validity: str | None = None,
        hard_block_reasons: tuple[str, ...] = (),
        advisory_reasons: tuple[str, ...] = (),
    ) -> dict[str, str]:
        warning_values = (
            [str(value) for value in warnings]
            if isinstance(warnings, (tuple, list))
            else [str(warnings)] if warnings else []
        )
        resolved_validity = validity or (
            "INVALID" if not valid else "ESTIMATED" if warning_values else "VALID"
        )
        payload = {
            "schema": "workstation-measurement-result-v2",
            "measurement_attempt_sha256": str(attempt_sha256),
            "status": str(status),
            "valid": bool(valid),
            "method": str(method),
            "quality": str(quality),
            "warnings": warning_values,
            "policy_id": str(policy_id),
            "validity": resolved_validity,
            "hard_block_reasons": [str(value) for value in hard_block_reasons],
            "advisory_reasons": [str(value) for value in advisory_reasons],
            "measurement_frame": MEASUREMENT_FRAME,
            "measurement_axis_contract": MEASUREMENT_AXIS_CONTRACT,
            "unit": "m",
            "vector_frd_range_m": list(vector_frd_range_m),
        }
        encoded = json.dumps(
            payload,
            ensure_ascii=False,
            allow_nan=False,
            sort_keys=True,
            separators=(",", ":"),
        )
        return {
            "measurement_result_canonical_json": encoded,
            "measurement_result_sha256": hashlib.sha256(encoded.encode("utf-8")).hexdigest(),
        }

    def _upsert_pair_recording_row(
        self,
        measurement_attempt: MeasurementAttempt,
        values: dict[str, object],
    ) -> bool:
        if not self.panel.distance_recording_enabled:
            return False
        key = tuple(measurement_attempt.detection_pair_key)
        row_index = self.panel._recording_pair_row_index.get(key)
        if row_index is None:
            row_index = len(self.panel.distance_recording_rows)
            values["sample_index"] = row_index
            values["pair_sequence"] = row_index
            content_sha256 = values.pop("measurement_attempt_content_sha256", None)
            instance_sha256 = values.pop("measurement_attempt_instance_sha256", None)
            values["measurement_attempt_content_sha256"] = content_sha256
            values["measurement_attempt_instance_sha256"] = instance_sha256
            self.panel._recording_pair_row_index[key] = row_index
            self.panel.distance_recording_rows.append(values)
            self.panel.distance_recording_keys.append(key)
            if len(self.panel.distance_recording_keys) > RECORDING_KEY_CACHE_LIMIT:
                self.panel.distance_recording_keys.pop(0)
            minimum_cached_index = max(
                0, len(self.panel.distance_recording_rows) - RECORDING_KEY_CACHE_LIMIT
            )
            self.panel._recording_pair_row_index = {
                cached_key: cached_index
                for cached_key, cached_index in self.panel._recording_pair_row_index.items()
                if cached_index >= minimum_cached_index
            }
            self.panel._append_recording_table_row(values)
            inserted = True
        else:
            current = self.panel.distance_recording_rows[row_index]
            sample_index = current.get("sample_index", row_index)
            pair_sequence = current.get("pair_sequence", row_index)
            current.update(values)
            current["sample_index"] = sample_index
            current["pair_sequence"] = pair_sequence
            if isinstance(self.panel.distance_recording_rows, SpoolRows):
                self.panel.distance_recording_rows.replace(row_index, current)
            self.panel._refresh_recording_table()
            inserted = False
        self.panel._refresh_recording_status_label()
        self.panel._refresh_operation_header()
        return inserted

    def _record_unresolved_pair(
        self,
        measurement_attempt: MeasurementAttempt,
        *,
        status: str,
        warnings: tuple[str, ...] = (),
    ) -> None:
        if not self.panel.distance_recording_enabled:
            return
        row = self.panel._pair_recording_base_row(measurement_attempt)
        row.update(
            {
                "pair_status": status,
                "measurement_valid": "false",
                "measurement_policy_id": measurement_attempt.measurement_policy_id,
                "measurement_validity": "INVALID",
                "measurement_hard_block_reasons": json.dumps(
                    list(measurement_attempt.hard_block_reasons or (status,)),
                    separators=(",", ":"),
                    ensure_ascii=False,
                ),
                "measurement_advisory_reasons": json.dumps(
                    list(warnings or measurement_attempt.advisory_reasons),
                    separators=(",", ":"),
                    ensure_ascii=False,
                ),
                "warnings": "; ".join(warnings or measurement_attempt.warnings),
            }
        )
        row.update(
            self._measurement_result_evidence(
                attempt_sha256=measurement_attempt.canonical_sha256(),
                status=status,
                valid=False,
                method=measurement_attempt.measurement_method_id,
                quality=measurement_attempt.source_quality,
                warnings=warnings or measurement_attempt.warnings,
                policy_id=measurement_attempt.measurement_policy_id,
                validity="INVALID",
                hard_block_reasons=measurement_attempt.hard_block_reasons or (status,),
                advisory_reasons=warnings or measurement_attempt.advisory_reasons,
                vector_frd_range_m=(None, None, None, None),
            )
        )
        self.panel._upsert_pair_recording_row(measurement_attempt, row)

    def _record_blocked_measurement_pair(self, blocked: BlockedMeasurementPair) -> None:
        if not self.panel.distance_recording_enabled:
            return
        key = tuple(blocked.detection_pair_key)
        blocked_sha256 = blocked.canonical_sha256()
        row: dict[str, object] = {
            "session_id": self.panel.recording_session_id,
            "measurement_id": str(getattr(self.panel, "_measurement_id", "") or ""),
            "measurement_package_id": str(
                getattr(self.panel, "_measurement_id", "") or ""
            ),
            "terminal_record_id": blocked.terminal_record_id,
            "schema_version": "stereowiris-paired-frame-v3",
            "measurement_valid": "false",
            "measurement_policy_id": "field_measurement_v1",
            "measurement_validity": "INVALID",
            "measurement_hard_block_reasons": json.dumps(
                list(blocked.reasons), separators=(",", ":"), ensure_ascii=False
            ),
            "measurement_advisory_reasons": "[]",
            "pair_status": blocked.status,
            "measurement_geometry_kind": "unresolved",
            "measurement_method_id": "",
            "measurement_method_instance_id": "",
            "measurement_attempt_sha256": "",
            "blocked_pair_sha256": blocked_sha256,
            "blocked_pair_canonical_json": blocked.canonical_json(),
            "measurement_readiness_sha256": blocked.readiness_sha256,
            "measurement_readiness_canonical_json": blocked.readiness_payload_json,
            "measurement_attempt_timestamp_basis": blocked.timestamp_basis,
            "measurement_detection_pair_key": json.dumps(key, separators=(",", ":")),
            "warnings": "; ".join(blocked.reasons),
            "recorded_unix_ns": int(round(blocked.created_wall_s * 1_000_000_000.0)),
            "pair_app_mid_monotonic_ns": int(
                round(
                    (
                        blocked.left_detection.timestamp_s
                        + blocked.right_detection.timestamp_s
                    )
                    * 0.5
                    * 1_000_000_000.0
                )
            ),
            "selected_left_pairing_timestamp_s": blocked.left_detection.pairing_timestamp_s,
            "selected_right_pairing_timestamp_s": blocked.right_detection.pairing_timestamp_s,
            "left_bbox_xyxy_px": json.dumps(blocked.left_detection.bbox_xyxy_px),
            "right_bbox_xyxy_px": json.dumps(blocked.right_detection.bbox_xyxy_px),
            "left_class_id": blocked.left_detection.class_id,
            "right_class_id": blocked.right_detection.class_id,
        }
        row.update(
            self._persist_native_detection_pair(
                left_detection=blocked.left_detection,
                right_detection=blocked.right_detection,
                pair_id=f"blocked-{blocked_sha256[:20]}",
                purpose="blocked-measurement",
            )
        )
        row.update(
            self._measurement_result_evidence(
                attempt_sha256=blocked_sha256,
                status=blocked.status,
                valid=False,
                method="",
                quality="BLOCKED",
                warnings=blocked.reasons,
                vector_frd_range_m=(None, None, None, None),
            )
        )
        row_index = self.panel._recording_pair_row_index.get(key)
        if row_index is None:
            row_index = len(self.panel.distance_recording_rows)
            row["sample_index"] = row_index
            row["pair_sequence"] = row_index
            self.panel._recording_pair_row_index[key] = row_index
            self.panel.distance_recording_rows.append(row)
            self.panel.distance_recording_keys.append(key)
            if len(self.panel.distance_recording_keys) > RECORDING_KEY_CACHE_LIMIT:
                self.panel.distance_recording_keys.pop(0)
            minimum_cached_index = max(
                0,
                len(self.panel.distance_recording_rows) - RECORDING_KEY_CACHE_LIMIT,
            )
            self.panel._recording_pair_row_index = {
                cached_key: cached_index
                for cached_key, cached_index in self.panel._recording_pair_row_index.items()
                if cached_index >= minimum_cached_index
            }
            self.panel._append_recording_table_row(row)
        else:
            current = self.panel.distance_recording_rows[row_index]
            sample_index = current.get("sample_index", row_index)
            pair_sequence = current.get("pair_sequence", row_index)
            current.update(row)
            current["sample_index"] = sample_index
            current["pair_sequence"] = pair_sequence
            if isinstance(self.panel.distance_recording_rows, SpoolRows):
                self.panel.distance_recording_rows.replace(row_index, current)
            self.panel._refresh_recording_table()
        self.panel._refresh_recording_status_label()
        self.panel._refresh_operation_header()

    def _append_distance_history(
        self,
        *,
        estimate,
        distance_m: float,
        method: str,
        baseline_source: str,
        baseline_m: float,
        point_m: tuple[float, float, float] | None = None,
        raw_point_m: tuple[float, float, float] | None = None,
        raw_point_left_m: tuple[float, float, float] | None = None,
        point_left_m: tuple[float, float, float] | None = None,
        measurement_attempt: MeasurementAttempt,
    ) -> bool:
        baseline_source = measurement_attempt.baseline_source
        baseline_m = measurement_attempt.baseline_m
        key = tuple(measurement_attempt.detection_pair_key)
        if key == self.panel._last_distance_history_key:
            return False
        self.panel._last_distance_history_key = key
        if point_m is None:
            point_m = point_left_m
        if raw_point_m is None:
            raw_point_m = raw_point_left_m
        timestamp_s = max(estimate.left.timestamp_s, estimate.right.timestamp_s)
        class_name = self.panel._distance_history_class_name(estimate)
        recording_elapsed_s = None
        if self.panel.distance_recording_enabled:
            if self.panel.distance_recording_started_monotonic_s is None:
                self.panel.distance_recording_started_monotonic_s = time.monotonic()
            recording_elapsed_s = max(0.0, time.monotonic() - self.panel.distance_recording_started_monotonic_s)
        point = DistanceHistoryPoint(
            timestamp_s=timestamp_s,
            distance_m=distance_m,
            quality=estimate.sync_quality,
            method=method,
            forward_m=None if point_m is None else float(point_m[0]),
            right_m=None if point_m is None else float(point_m[1]),
            down_m=None if point_m is None else float(point_m[2]),
            class_name=class_name,
            sample_key=tuple(measurement_attempt.detection_pair_key),
            recording_elapsed_s=recording_elapsed_s,
        )
        self.panel.distance_history.append(point)
        self.panel.distance_plot.add_point(point)
        if not self.panel.distance_recording_enabled:
            return True
        raw_distance_m = None
        if raw_point_m is not None:
            raw_distance_m = math.sqrt(
                raw_point_m[0] * raw_point_m[0]
                + raw_point_m[1] * raw_point_m[1]
                + raw_point_m[2] * raw_point_m[2]
            )
        measurement_attempt_content_sha256 = measurement_attempt.content_sha256()
        measurement_attempt_instance_sha256 = measurement_attempt.instance_sha256()
        row = self.panel._pair_recording_base_row(measurement_attempt)
        row.update({
            "session_id": self.panel.recording_session_id,
            "measurement_id": str(getattr(self.panel, "_measurement_id", "") or ""),
            "measurement_attempt_id": str(measurement_attempt.attempt_id),
            "measurement_attempt_session_id": measurement_attempt.attempt_session_id,
            "measurement_attempt_sha256": measurement_attempt.canonical_sha256(),
            "measurement_detection_pair_key": json.dumps(
                measurement_attempt.detection_pair_key, separators=(",", ":")
            ),
            "measurement_attempt_timestamp_basis": measurement_attempt.timestamp_basis,
            "selected_left_pairing_timestamp_s": self.panel._fmt_record_float(
                measurement_attempt.left_detection.pairing_timestamp_s, 6
            ),
            "selected_right_pairing_timestamp_s": self.panel._fmt_record_float(
                measurement_attempt.right_detection.pairing_timestamp_s, 6
            ),
            "measurement_attempt_source_quality": measurement_attempt.source_quality,
            "measurement_attempt_warnings": "; ".join(measurement_attempt.warnings),
            "measurement_policy_id": measurement_attempt.measurement_policy_id,
            "measurement_validity": (
                "ESTIMATED" if estimate.warnings else "VALID"
            ),
            "measurement_hard_block_reasons": json.dumps([], separators=(",", ":")),
            "measurement_advisory_reasons": json.dumps(
                list(estimate.warnings), separators=(",", ":"), ensure_ascii=False
            ),
            "session_started_unix_s": self.panel._fmt_record_float(self.panel.recording_session_started_wall_s, 6),
            "left_video_path": str(self.panel.video_recording_paths.get("left", "")),
            "right_video_path": str(self.panel.video_recording_paths.get("right", "")),
            "timestamp_s": f"{timestamp_s:.6f}",
            "distance_m": f"{distance_m:.6f}",
            "method": method,
            "quality": estimate.sync_quality,
            "disparity_px": f"{estimate.disparity_px:.6f}",
            "raw_disparity_px": f"{estimate.disparity_px:.6f}",
            "camera_delta_s": f"{estimate.time_delta_s:.6f}",
            "vertical_delta_px": f"{estimate.vertical_delta_px:.6f}",
            "baseline_source": baseline_source,
            "baseline_m": f"{baseline_m:.6f}",
            "measurement_frame": MEASUREMENT_FRAME,
            "measurement_axis_contract": MEASUREMENT_AXIS_CONTRACT,
            "legacy_xyz_alias_contract": LEGACY_XYZ_ALIAS_CONTRACT,
            "point_frame": MEASUREMENT_FRAME,
            "output_frame": MEASUREMENT_FRAME,
            "public_vector_contract": PUBLIC_MEASUREMENT_VECTOR_CONTRACT,
            "baseline_frame_scope": "measurement_attempt",
            "baseline_frame_instance_id": measurement_attempt.instance_sha256(),
            "baseline_frame_origin_contract": (
                "midpoint_of_left_optical_origin_and_selected_right_origin_proxy"
            ),
            "baseline_frame_origin_model": (
                "resolved_relative_pose_optical_centers"
                if measurement_attempt.measurement_geometry_kind == "relative_pose"
                else "gps_rtk_antenna_baseline_proxy_no_lever_arm"
            ),
            "measurement_method": "two_ray_closest_midpoint",
            "range_m": self.panel._fmt_record_float(distance_m, 6),
            "baseline_center_forward_m": self.panel._fmt_record_float(point.forward_m, 6),
            "baseline_center_right_m": self.panel._fmt_record_float(point.right_m, 6),
            "baseline_center_down_m": self.panel._fmt_record_float(point.down_m, 6),
            "baseline_center_range_m": self.panel._fmt_record_float(distance_m, 6),
            "raw_baseline_center_forward_m": self.panel._fmt_record_float(
                None if raw_point_m is None else raw_point_m[0], 6
            ),
            "raw_baseline_center_right_m": self.panel._fmt_record_float(
                None if raw_point_m is None else raw_point_m[1], 6
            ),
            "raw_baseline_center_down_m": self.panel._fmt_record_float(
                None if raw_point_m is None else raw_point_m[2], 6
            ),
            "raw_baseline_center_range_m": self.panel._fmt_record_float(raw_distance_m, 6),
            "source_left_camera_rdf_right_m": self.panel._fmt_record_float(
                None if raw_point_left_m is None else raw_point_left_m[0], 6
            ),
            "source_left_camera_rdf_down_m": self.panel._fmt_record_float(
                None if raw_point_left_m is None else raw_point_left_m[1], 6
            ),
            "source_left_camera_rdf_forward_m": self.panel._fmt_record_float(
                None if raw_point_left_m is None else raw_point_left_m[2], 6
            ),
            "recording_elapsed_s": self.panel._fmt_record_float(recording_elapsed_s, 6),
            "warnings": "; ".join(estimate.warnings),
            "pair_status": "measured",
            "measurement_valid": "true",
        })
        row.update(self.panel._record_bbox_parameters(estimate))
        row.update(self.panel._record_measurement_sync_timestamp_parameters(estimate))
        row.update(self.panel._record_intrinsic_parameters(measurement_attempt))
        row.update(self.panel._record_frame_timestamp_parameters(measurement_attempt))
        row.update(self.panel._record_live_telemetry_snapshot_parameters(measurement_attempt))
        row.update(self.panel._record_external_parameters(estimate=estimate, measurement_attempt=measurement_attempt))
        row.update(
            self._measurement_result_evidence(
                attempt_sha256=measurement_attempt.canonical_sha256(),
                status="measured",
                valid=True,
                method=method,
                quality=estimate.sync_quality,
                warnings=tuple(estimate.warnings),
                policy_id=measurement_attempt.measurement_policy_id,
                validity="ESTIMATED" if estimate.warnings else "VALID",
                advisory_reasons=tuple(estimate.warnings),
                vector_frd_range_m=(
                    point.forward_m,
                    point.right_m,
                    point.down_m,
                    distance_m,
                ),
            )
        )
        row.pop("measurement_attempt_content_sha256", None)
        row.pop("measurement_attempt_instance_sha256", None)
        row["measurement_attempt_content_sha256"] = measurement_attempt_content_sha256
        row["measurement_attempt_instance_sha256"] = measurement_attempt_instance_sha256
        self.panel._upsert_pair_recording_row(measurement_attempt, row)
        pair_sequence = self.panel._recording_pair_row_index.get(
            tuple(measurement_attempt.detection_pair_key)
        )
        if pair_sequence is not None:
            self.panel._append_extrinsics_comparison_snapshot(pair_sequence, timestamp_s)
        return True

    def _append_extrinsics_comparison_snapshot(self, pair_sequence: int, timestamp_s: float) -> None:
        """Snapshot analysis-only per-method previews for one accepted pair.

        ``geometry_comparison_rows`` is recomputed every ``_refresh_distance``
        tick; only the value at the exact moment a pair is accepted is worth
        persisting, so this is called once from ``_append_distance_history``
        rather than on every tick. These rows are a separate analysis namespace,
        never an official Measurement result or MeasurementAttempt.
        """

        if not self.panel.distance_recording_enabled:
            return
        for comparison_row in getattr(self.panel, "geometry_comparison_rows", ()) or ():
            record = {
                "pair_sequence": pair_sequence,
                "timestamp_s": self.panel._fmt_record_float(timestamp_s, 6),
                "method": str(comparison_row.get("method", "")),
                "forward_m": self.panel._fmt_record_float(
                    comparison_row.get("forward_m"), 6
                ),
                "right_m": self.panel._fmt_record_float(
                    comparison_row.get("right_m"), 6
                ),
                "down_m": self.panel._fmt_record_float(
                    comparison_row.get("down_m"), 6
                ),
                "range_m": self.panel._fmt_record_float(
                    comparison_row.get("range_m"), 6
                ),
                "status": str(comparison_row.get("status", "")),
            }
            for name in (
                "schema_version",
                "evidence_namespace",
                "result_class",
                "not_measurement_result",
                "source_state",
                "source_profile_id",
                "source_profile_hash",
                "candidate_profile_id",
                "candidate_profile_hash",
                "candidate_intrinsics_hash",
                "active_intrinsics_hash",
                "candidate_applied",
                "candidate_applicability",
                "candidate_quality",
            ):
                value = comparison_row.get(name)
                if value is not None:
                    record[name] = value
            self.panel.extrinsics_comparison_recording_rows.append(record)

    def _distance_history_class_name(self, estimate) -> str | None:
        left_class_name = str(getattr(estimate.left, "class_name", "") or "").strip()
        right_class_name = str(getattr(estimate.right, "class_name", "") or "").strip()
        if left_class_name and right_class_name:
            if left_class_name == right_class_name:
                return left_class_name
            return f"{left_class_name}/{right_class_name}"
        return left_class_name or right_class_name or None

    def _fmt_record_float(self, value: float | None, precision: int = 6) -> str:
        return "" if value is None else f"{float(value):.{precision}f}"

    def _fmt_record_bool(self, value: bool | None) -> str:
        return "" if value is None else ("true" if value else "false")

    def _recording_elapsed_s(self, timestamp_s: float | None = None) -> float | None:
        started_s = self.panel.distance_recording_started_monotonic_s or self.panel.recording_session_started_monotonic_s
        if started_s is None:
            return None
        if timestamp_s is None:
            timestamp_s = time.monotonic()
        return max(0.0, float(timestamp_s) - started_s)

    def _session_utc_ns_for_monotonic_ns(self, monotonic_ns: int) -> int | None:
        if (
            self.panel.recording_session_started_unix_ns is None
            or self.panel.recording_session_started_monotonic_ns is None
        ):
            return None
        return (
            self.panel.recording_session_started_unix_ns
            + monotonic_ns
            - self.panel.recording_session_started_monotonic_ns
        )

    def _recording_side(self, camera: str) -> str | None:
        normalized = str(camera).lower()
        if normalized.startswith("left"):
            return "left"
        if normalized.startswith("right"):
            return "right"
        if normalized.startswith("drone"):
            return "drone"
        return None

    def _raw_timeline_row(
        self,
        *,
        side: str,
        sample_type: str,
        row_contract: str,
        sample_monotonic_s: float,
        host_receive_monotonic_ns: int,
        host_receive_utc_ns: int,
        source_utc_ns: int | None,
        source_utc_method: str,
        sequence: int,
    ) -> dict[str, object] | None:
        sample_monotonic_ns = int(round(float(sample_monotonic_s) * 1_000_000_000.0))
        started_monotonic_ns = self.panel.recording_session_started_monotonic_ns
        if started_monotonic_ns is not None and sample_monotonic_ns < started_monotonic_ns:
            return None
        session_elapsed_s = (
            None
            if started_monotonic_ns is None
            else (sample_monotonic_ns - started_monotonic_ns) * 1e-9
        )
        session_utc_ns = self.panel._session_utc_ns_for_monotonic_ns(sample_monotonic_ns)
        return {
            "sequence": sequence,
            "session_id": self.panel.recording_session_id,
            "measurement_id": str(getattr(self.panel, "_measurement_id", "") or ""),
            "side": side,
            "sample_type": sample_type,
            "row_contract": row_contract,
            "sample_monotonic_ns": sample_monotonic_ns,
            "sample_monotonic_s": self.panel._fmt_record_float(sample_monotonic_s, 9),
            "session_elapsed_s": self.panel._fmt_record_float(session_elapsed_s, 9),
            "session_utc_ns": "" if session_utc_ns is None else session_utc_ns,
            "session_utc_s": self.panel._fmt_record_float(
                None if session_utc_ns is None else session_utc_ns * 1e-9,
                9,
            ),
            "session_utc_method": (
                "unavailable" if session_utc_ns is None else "host_unix_monotonic_session_anchor"
            ),
            "session_utc_anchor_uncertainty_ns": (
                ""
                if self.panel.recording_session_anchor_uncertainty_ns is None
                else self.panel.recording_session_anchor_uncertainty_ns
            ),
            "source_utc_ns": "" if source_utc_ns is None else source_utc_ns,
            "source_utc_s": self.panel._fmt_record_float(
                None if source_utc_ns is None else source_utc_ns * 1e-9,
                9,
            ),
            "source_utc_valid": self.panel._fmt_record_bool(source_utc_ns is not None),
            "source_utc_method": source_utc_method if source_utc_ns is not None else "unavailable",
            "host_receive_monotonic_ns": host_receive_monotonic_ns,
            "host_receive_utc_ns": host_receive_utc_ns,
        }

    def _record_gui_camera_frame(
        self,
        camera: str,
        image: QtGui.QImage,
        metadata: dict[str, object],
    ) -> None:
        if not self.panel.distance_recording_enabled or camera not in self.panel.camera_recording_rows:
            return
        host_receive_monotonic_ns = time.monotonic_ns()
        host_receive_utc_ns = time.time_ns()
        sample_monotonic_s = float(
            metadata.get("timestamp_s") or host_receive_monotonic_ns * 1e-9
        )
        rtcp_ntp_s = metadata.get("rtcp_frame_ntp_unix_s")
        source_utc_ns = (
            None
            if rtcp_ntp_s is None
            else self.panel._unix_seconds_to_ns(float(rtcp_ntp_s))
        )
        rows = self.panel.camera_recording_rows[camera]
        row = self.panel._raw_timeline_row(
            side=camera,
            sample_type="gui_camera_frame",
            row_contract="one row per frame delivered to the GUI frame_changed callback",
            sample_monotonic_s=sample_monotonic_s,
            host_receive_monotonic_ns=host_receive_monotonic_ns,
            host_receive_utc_ns=host_receive_utc_ns,
            source_utc_ns=source_utc_ns,
            source_utc_method="rtcp_ntp",
            sequence=len(rows),
        )
        if row is None:
            return
        row.update(
            {
                "frame_stage": "gui_delivered_preview",
                "mp4_frame_index_contract": "not_available",
                "image_width_px": image.width(),
                "image_height_px": image.height(),
                "image_axis_contract": "opencv_pixels_origin_top_left_u_right_v_down",
                "timestamp_source": str(metadata.get("timestamp_source") or ""),
                "timestamp_reference": str(metadata.get("timestamp_reference") or ""),
                "source_timestamp_s": self.panel._fmt_record_float(
                    None
                    if metadata.get("source_timestamp_s") is None
                    else float(metadata["source_timestamp_s"]),
                    9,
                ),
                "rtp_timestamp": (
                    "" if metadata.get("rtp_timestamp") is None else str(int(metadata["rtp_timestamp"]))
                ),
                "rtp_ssrc": (
                    "" if metadata.get("rtp_ssrc") is None else str(int(metadata["rtp_ssrc"]))
                ),
                "rtp_marker_seq": (
                    "" if metadata.get("rtp_marker_seq") is None else str(int(metadata["rtp_marker_seq"]))
                ),
                "stream_generation": (
                    ""
                    if metadata.get("stream_generation") is None
                    else int(metadata["stream_generation"])
                ),
                "rtcp_sr_ntp_raw": (
                    ""
                    if metadata.get("rtcp_sr_ntp_raw") is None
                    else int(metadata["rtcp_sr_ntp_raw"])
                ),
                "rtcp_sr_ntp_unix_s": self.panel._fmt_record_float(
                    None
                    if metadata.get("rtcp_sr_ntp_unix_s") is None
                    else float(metadata["rtcp_sr_ntp_unix_s"]),
                    9,
                ),
                "rtcp_sr_received_count": (
                    ""
                    if metadata.get("rtcp_sr_received_count") is None
                    else str(int(metadata["rtcp_sr_received_count"]))
                ),
                "rtcp_sr_rtp_timestamp": (
                    ""
                    if metadata.get("rtcp_sr_rtp_timestamp") is None
                    else str(int(metadata["rtcp_sr_rtp_timestamp"]))
                ),
                "rtcp_clock_rate_hz": self.panel._fmt_record_float(
                    None
                    if metadata.get("rtcp_clock_rate_hz") is None
                    else float(metadata["rtcp_clock_rate_hz"]),
                    3,
                ),
                "rtcp_sr_age_s": self.panel._fmt_record_float(
                    None if metadata.get("rtcp_sr_age_s") is None else float(metadata["rtcp_sr_age_s"]),
                    9,
                ),
                "rtcp_sr_pc_offset_s": self.panel._fmt_record_float(
                    None
                    if metadata.get("rtcp_sr_pc_offset_s") is None
                    else float(metadata["rtcp_sr_pc_offset_s"]),
                    9,
                ),
                "rtcp_frame_ntp_unix_s": self.panel._fmt_record_float(
                    None if rtcp_ntp_s is None else float(rtcp_ntp_s),
                    9,
                ),
                "rtcp_frame_received_wall_s": self.panel._fmt_record_float(
                    None
                    if metadata.get("rtcp_frame_received_wall_s") is None
                    else float(metadata["rtcp_frame_received_wall_s"]),
                    9,
                ),
                "rtcp_frame_receive_delay_s": self.panel._fmt_record_float(
                    None
                    if metadata.get("rtcp_frame_receive_delay_s") is None
                    else float(metadata["rtcp_frame_receive_delay_s"]),
                    9,
                ),
                "rtcp_frame_transport_delay_s": self.panel._fmt_record_float(
                    None
                    if metadata.get("rtcp_frame_transport_delay_s") is None
                    else float(metadata["rtcp_frame_transport_delay_s"]),
                    9,
                ),
                "rtcp_camera_ntp_rate": self.panel._fmt_record_float(
                    None
                    if metadata.get("rtcp_camera_ntp_rate") is None
                    else float(metadata["rtcp_camera_ntp_rate"]),
                    9,
                ),
                "rtcp_camera_media_rate": self.panel._fmt_record_float(
                    None
                    if metadata.get("rtcp_camera_media_rate") is None
                    else float(metadata["rtcp_camera_media_rate"]),
                    9,
                ),
                "rtsp_transport": str(metadata.get("rtsp_transport") or ""),
                "rtcp_timing_invalid_reason": str(
                    metadata.get("rtcp_timing_invalid_reason") or ""
                ),
            }
        )
        rows.append(row)

    def _record_external_timeline_row(self, camera: str, sample_type: str, values: dict[str, object]) -> None:
        if not self.panel.distance_recording_enabled:
            return
        side = self.panel._recording_side(camera)
        if side is None:
            return
        if side not in self.panel.px4_recording_rows:
            rows = SpoolRows()
            rows.bind(
                self.panel._recording_session_dir(self.panel.recording_session_id)
                / ".spool"
                / f"px4_{side}.jsonl"
            )
            self.panel.px4_recording_rows[side] = rows
        host_receive_monotonic_ns = time.monotonic_ns()
        host_receive_utc_ns = time.time_ns()
        raw_sample_monotonic_s = values.get("sample_app_timestamp_s")
        sample_monotonic_s = (
            host_receive_monotonic_ns * 1e-9
            if raw_sample_monotonic_s in (None, "")
            else float(raw_sample_monotonic_s)
        )
        raw_gps_utc_us = values.get("gps_utc_us")
        source_utc_ns = (
            None
            if raw_gps_utc_us in (None, "", 0, "0")
            else int(raw_gps_utc_us) * 1000
        )
        rows = self.panel.px4_recording_rows[side]
        row = self.panel._raw_timeline_row(
            side=side,
            sample_type=sample_type,
            row_contract="one row per PX4 telemetry callback received by the GUI",
            sample_monotonic_s=sample_monotonic_s,
            host_receive_monotonic_ns=host_receive_monotonic_ns,
            host_receive_utc_ns=host_receive_utc_ns,
            source_utc_ns=source_utc_ns,
            source_utc_method=str(values.get("source_utc_method") or "gps_utc"),
            sequence=len(rows),
        )
        if row is None:
            return
        row.update(
            {
                "camera": camera,
                "triangulation_extrinsics_source": self.panel._triangulation_extrinsics_source(),
                "distance_mode": self.panel._distance_mode_label(),
            }
        )
        row.update(mavlink_link_provenance(self.panel, side))
        row.update(values)
        rows.append(row)
        self.panel.external_parameter_recording_rows.append(row)
        self.panel._refresh_recording_status_label()

    def _record_bbox_parameters(self, estimate) -> dict[str, str]:
        row: dict[str, str] = {}
        for camera in ("left", "right"):
            detection = getattr(estimate, camera, None)
            center = getattr(detection, "center", None)
            if center is None:
                center_x = None
                center_y = None
            else:
                center_x = float(center[0])
                center_y = float(center[1])
            x1 = getattr(detection, "x1", None)
            y1 = getattr(detection, "y1", None)
            x2 = getattr(detection, "x2", None)
            y2 = getattr(detection, "y2", None)
            bbox_width = None
            bbox_height = None
            if x1 is not None and x2 is not None:
                bbox_width = float(x2) - float(x1)
            if y1 is not None and y2 is not None:
                bbox_height = float(y2) - float(y1)
            row[f"{camera}_bbox_x1_px"] = self.panel._fmt_record_float(getattr(detection, "x1", None), 3)
            row[f"{camera}_bbox_y1_px"] = self.panel._fmt_record_float(getattr(detection, "y1", None), 3)
            row[f"{camera}_bbox_x2_px"] = self.panel._fmt_record_float(getattr(detection, "x2", None), 3)
            row[f"{camera}_bbox_y2_px"] = self.panel._fmt_record_float(getattr(detection, "y2", None), 3)
            row[f"{camera}_bbox_center_x_px"] = self.panel._fmt_record_float(center_x, 3)
            row[f"{camera}_bbox_center_y_px"] = self.panel._fmt_record_float(center_y, 3)
            row[f"{camera}_bbox_confidence"] = self.panel._fmt_record_float(getattr(detection, "confidence", None), 6)
            row[f"{camera}_bbox_class_id"] = "" if getattr(detection, "class_id", None) is None else str(detection.class_id)
            row[f"{camera}_bbox_class_name"] = str(getattr(detection, "class_name", "") or "")
            row[f"{camera}_x1"] = self.panel._fmt_record_float(x1, 3)
            row[f"{camera}_y1"] = self.panel._fmt_record_float(y1, 3)
            row[f"{camera}_x2"] = self.panel._fmt_record_float(x2, 3)
            row[f"{camera}_y2"] = self.panel._fmt_record_float(y2, 3)
            row[f"{camera}_cx"] = self.panel._fmt_record_float(center_x, 3)
            row[f"{camera}_cy"] = self.panel._fmt_record_float(center_y, 3)
            row[f"{camera}_width_px"] = self.panel._fmt_record_float(bbox_width, 3)
            row[f"{camera}_height_px"] = self.panel._fmt_record_float(bbox_height, 3)
            row[f"{camera}_confidence"] = self.panel._fmt_record_float(getattr(detection, "confidence", None), 6)
        return row

    def _detection_utc_unix_s(self, detection) -> float | None:
        metadata = dict(getattr(detection, "timestamp_metadata", None) or {})
        value = metadata.get("rtcp_frame_ntp_unix_s")
        if value is None:
            return None
        try:
            parsed = float(value)
        except (TypeError, ValueError):
            return None
        return parsed if isfinite(parsed) else None

    def _unix_seconds_to_ns(self, value: float | None) -> int | None:
        if value is None:
            return None
        try:
            return int(
                (Decimal(str(value)) * Decimal(1_000_000_000)).to_integral_value(
                    rounding=ROUND_HALF_UP
                )
            )
        except (InvalidOperation, ValueError):
            return None

    def _record_measurement_sync_timestamp_parameters(self, estimate) -> dict[str, object]:
        left_timestamp_s = float(getattr(estimate.left, "timestamp_s", 0.0))
        right_timestamp_s = float(getattr(estimate.right, "timestamp_s", 0.0))
        bbox_mid_timestamp_s = (left_timestamp_s + right_timestamp_s) / 2.0
        left_utc_unix_s = self.panel._detection_utc_unix_s(estimate.left)
        right_utc_unix_s = self.panel._detection_utc_unix_s(estimate.right)
        bbox_mid_utc_unix_s = (
            (left_utc_unix_s + right_utc_unix_s) / 2.0
            if left_utc_unix_s is not None and right_utc_unix_s is not None
            else None
        )
        left_utc_ns = self.panel._unix_seconds_to_ns(left_utc_unix_s)
        right_utc_ns = self.panel._unix_seconds_to_ns(right_utc_unix_s)
        pair_mid_utc_ns = (
            None if left_utc_ns is None or right_utc_ns is None else (left_utc_ns + right_utc_ns) // 2
        )
        rtcp_pair_delta_ns = (
            None if left_utc_ns is None or right_utc_ns is None else right_utc_ns - left_utc_ns
        )
        pair_limit_s = self.panel._yolo_pair_max_delta_s()
        pair_limit_ns = (
            None
            if not isfinite(pair_limit_s)
            else int(round(pair_limit_s * 1_000_000_000.0))
        )
        camera_time_metrics = self.panel._camera_time_sync_metrics(
            dict(getattr(estimate.left, "timestamp_metadata", None) or {}),
            dict(getattr(estimate.right, "timestamp_metadata", None) or {}),
            left_receive_mono_s=left_timestamp_s,
            right_receive_mono_s=right_timestamp_s,
        )
        row = {
            "left_bbox_timestamp_s": self.panel._fmt_record_float(left_timestamp_s, 6),
            "right_bbox_timestamp_s": self.panel._fmt_record_float(right_timestamp_s, 6),
            "bbox_mid_timestamp_s": self.panel._fmt_record_float(bbox_mid_timestamp_s, 6),
            "left_bbox_utc_unix_s": self.panel._fmt_record_float(left_utc_unix_s, 6),
            "right_bbox_utc_unix_s": self.panel._fmt_record_float(right_utc_unix_s, 6),
            "bbox_mid_utc_unix_s": self.panel._fmt_record_float(bbox_mid_utc_unix_s, 6),
            "left_frame_utc_ns": "" if left_utc_ns is None else left_utc_ns,
            "right_frame_utc_ns": "" if right_utc_ns is None else right_utc_ns,
            "pair_mid_utc_ns": "" if pair_mid_utc_ns is None else pair_mid_utc_ns,
            "rtcp_pair_delta_ns": "" if rtcp_pair_delta_ns is None else rtcp_pair_delta_ns,
            "pair_timing_limit_ns": "" if pair_limit_ns is None else pair_limit_ns,
            "pair_timing_valid": self.panel._fmt_record_bool(
                rtcp_pair_delta_ns is not None
                and (pair_limit_ns is None or abs(rtcp_pair_delta_ns) <= pair_limit_ns)
            ),
            "left_bbox_timestamp_reference": str(
                (getattr(estimate.left, "timestamp_metadata", None) or {}).get("timestamp_reference") or "app-receive"
            ),
            "right_bbox_timestamp_reference": str(
                (getattr(estimate.right, "timestamp_metadata", None) or {}).get("timestamp_reference") or "app-receive"
            ),
            "camera_time_sync_status": str(camera_time_metrics["status"]),
            "camera_time_sync_basis": str(camera_time_metrics["basis"]),
            "camera_time_sync_warning": str(camera_time_metrics["warning"]),
            "stream_sync_state": self.panel.camera_sync_status,
            "stream_sync_reason": self.panel.camera_sync_reason,
            "rtcp_pair_delta_s": self.panel._fmt_record_float(camera_time_metrics["pair_delta_s"], 6),
            "rtcp_pc_offset_max_ms": self.panel._fmt_record_float(camera_time_metrics["max_pc_offset_ms"], 3),
            "rtcp_frame_transit_max_ms": self.panel._fmt_record_float(camera_time_metrics["max_frame_transit_delay_ms"], 3),
            "left_rtcp_frame_transit_ms": self.panel._fmt_record_float(camera_time_metrics["left_frame_transit_delay_ms"], 3),
            "right_rtcp_frame_transit_ms": self.panel._fmt_record_float(camera_time_metrics["right_frame_transit_delay_ms"], 3),
            "left_rtcp_pc_offset_ms": self.panel._fmt_record_float(camera_time_metrics["left_pc_offset_ms"], 3),
            "right_rtcp_pc_offset_ms": self.panel._fmt_record_float(camera_time_metrics["right_pc_offset_ms"], 3),
        }
        return row

    def _record_frame_timestamp_parameters(
        self, measurement_attempt: MeasurementAttempt
    ) -> dict[str, object]:
        row: dict[str, object] = {}
        detections = {
            "left": measurement_attempt.left_detection,
            "right": measurement_attempt.right_detection,
        }
        for camera, detection in detections.items():
            metadata = detection.timestamp_metadata
            timestamp_s = detection.app_receive_monotonic_s
            source_timestamp = metadata.get("source_timestamp_s")
            timestamp_reference = infer_runtime_camera_timestamp_reference(metadata)
            row[f"{camera}_display_frame_timestamp_s"] = self.panel._fmt_record_float(timestamp_s, 6)
            row[f"{camera}_source_frame_timestamp_s"] = self.panel._fmt_record_float(
                None if source_timestamp is None else float(source_timestamp),
                6,
            )
            row[f"{camera}_frame_timestamp_source"] = str(metadata.get("timestamp_source") or "")
            row[f"{camera}_frame_timestamp_reference"] = timestamp_reference
            row[f"{camera}_rtp_timestamp"] = "" if metadata.get("rtp_timestamp") is None else str(int(metadata["rtp_timestamp"]))
            row[f"{camera}_rtp_ssrc"] = "" if metadata.get("rtp_ssrc") is None else str(int(metadata["rtp_ssrc"]))
            row[f"{camera}_stream_generation"] = (
                ""
                if metadata.get("stream_generation") is None
                else int(metadata["stream_generation"])
            )
            row[f"{camera}_rtcp_frame_ntp_unix_s"] = self.panel._fmt_record_float(
                None if metadata.get("rtcp_frame_ntp_unix_s") is None else float(metadata["rtcp_frame_ntp_unix_s"]),
                6,
            )
            rtcp_frame_ntp_unix_s = metadata.get("rtcp_frame_ntp_unix_s")
            row[f"{camera}_rtcp_frame_ntp_unix_ns"] = (
                ""
                if rtcp_frame_ntp_unix_s is None
                else self.panel._unix_seconds_to_ns(float(rtcp_frame_ntp_unix_s))
            )
            row[f"{camera}_app_receive_monotonic_ns"] = int(
                round(timestamp_s * 1_000_000_000.0)
            )
            row[f"{camera}_camera_rtc_frame_unix_s"] = row[f"{camera}_rtcp_frame_ntp_unix_s"]
            row[f"{camera}_rtcp_sr_ntp_unix_s"] = self.panel._fmt_record_float(
                None if metadata.get("rtcp_sr_ntp_unix_s") is None else float(metadata["rtcp_sr_ntp_unix_s"]),
                6,
            )
            row[f"{camera}_camera_rtc_sr_unix_s"] = row[f"{camera}_rtcp_sr_ntp_unix_s"]
            row[f"{camera}_rtcp_sr_rtp_timestamp"] = (
                "" if metadata.get("rtcp_sr_rtp_timestamp") is None else str(int(metadata["rtcp_sr_rtp_timestamp"]))
            )
            row[f"{camera}_rtsp_transport"] = str(metadata.get("rtsp_transport") or "")
            row[f"{camera}_camera_rtc_basis"] = (
                "rtcp-sender-report"
                if metadata.get("rtcp_frame_ntp_unix_s") is not None
                else timestamp_reference
            )
        display_delta_s = (
            measurement_attempt.right_detection.app_receive_monotonic_s
            - measurement_attempt.left_detection.app_receive_monotonic_s
        )
        left_metadata = measurement_attempt.left_detection.timestamp_metadata
        right_metadata = measurement_attempt.right_detection.timestamp_metadata
        left_source = left_metadata.get("source_timestamp_s")
        right_source = right_metadata.get("source_timestamp_s")
        source_delta_s = (
            float(right_source) - float(left_source)
            if left_source is not None and right_source is not None
            else None
        )
        left_rtcp_ntp = left_metadata.get("rtcp_frame_ntp_unix_s")
        right_rtcp_ntp = right_metadata.get("rtcp_frame_ntp_unix_s")
        rtcp_ntp_delta_s = (
            float(right_rtcp_ntp) - float(left_rtcp_ntp)
            if left_rtcp_ntp is not None and right_rtcp_ntp is not None
            else None
        )
        row["display_frame_delta_s"] = self.panel._fmt_record_float(display_delta_s, 6)
        row["source_frame_delta_s"] = self.panel._fmt_record_float(source_delta_s, 6)
        row["rtcp_frame_ntp_delta_s"] = self.panel._fmt_record_float(rtcp_ntp_delta_s, 6)
        row["camera_rtc_frame_delta_s"] = row["rtcp_frame_ntp_delta_s"]
        return row

    def _record_live_telemetry_snapshot_parameters(
        self, measurement_attempt: MeasurementAttempt
    ) -> dict[str, object]:
        row: dict[str, object] = {
            "attitude_frame": BODY_FRAME,
            "attitude_units": "deg",
            "gps_quality_frame": "gps_telemetry_metadata",
            "gps_quality_units": "satellites_count,eph_m,epv_m",
        }
        for camera in ("left", "right"):
            attitude = (
                measurement_attempt.left_attitude
                if camera == "left"
                else measurement_attempt.right_attitude
            )
            attitude_age_s = (
                measurement_attempt.left_attitude_age_s
                if camera == "left"
                else measurement_attempt.right_attitude_age_s
            )
            row[f"{camera}_roll_deg"] = self.panel._fmt_record_float(None if attitude is None else attitude.roll_deg, 6)
            row[f"{camera}_pitch_deg"] = self.panel._fmt_record_float(None if attitude is None else attitude.pitch_deg, 6)
            row[f"{camera}_yaw_deg"] = self.panel._fmt_record_float(None if attitude is None else attitude.yaw_deg, 6)
            row[f"{camera}_attitude_sample_age_s"] = self.panel._fmt_record_float(
                attitude_age_s,
                6,
            )
            row[f"{camera}_attitude_alignment"] = (
                "missing" if attitude is None else "nearest_app_receive_monotonic"
            )
            row[f"{camera}_attitude_topic"] = "" if attitude is None else str(attitude.topic)
            gps = measurement_attempt.left_gps if camera == "left" else measurement_attempt.right_gps
            gps_age_s = (
                None
                if gps is None
                else abs(
                    (
                        measurement_attempt.left_detection.app_receive_monotonic_s
                        if camera == "left"
                        else measurement_attempt.right_detection.app_receive_monotonic_s
                    )
                    - gps.app_receive_monotonic_s
                )
            )
            row[f"{camera}_gps_satellites_used"] = (
                "" if gps is None or getattr(gps, "satellites_used", None) is None else str(gps.satellites_used)
            )
            row[f"{camera}_gps_eph_m"] = self.panel._fmt_record_float(
                None if gps is None else getattr(gps, "eph_m", None),
                6,
            )
            row[f"{camera}_gps_epv_m"] = self.panel._fmt_record_float(
                None if gps is None else getattr(gps, "epv_m", None),
                6,
            )
            row[f"{camera}_gps_fix_type"] = "" if gps is None or getattr(gps, "fix_type", None) is None else str(gps.fix_type)
            row[f"{camera}_gps_sample_age_s"] = self.panel._fmt_record_float(
                gps_age_s
                if measurement_attempt.baseline is None
                else (
                    measurement_attempt.baseline.left_age_at_capture_s
                    if camera == "left"
                    else measurement_attempt.baseline.right_age_at_capture_s
                ),
                6,
            )
            row[f"{camera}_gps_alignment"] = (
                "missing" if gps is None else "latest_selected_app_receive_monotonic"
            )
            row[f"{camera}_gps_topic"] = "" if gps is None else str(gps.topic)
        return row

    def _record_reference_delta_parameters(
        self, measurement_attempt: MeasurementAttempt
    ) -> dict[str, object]:
        row: dict[str, object] = {
            "gps_position_frame": "session_left_start_gps_antenna_ned",
            "gps_position_axis_contract": "north,east,down; metres",
            "attitude_orientation_contract": "px4_frd_to_ned; quaternion_wxyz",
        }
        attitudes = {
            "left": measurement_attempt.left_attitude,
            "right": measurement_attempt.right_attitude,
        }
        gps_states = {
            "left": measurement_attempt.left_gps,
            "right": measurement_attempt.right_gps,
        }
        for camera, attitude in attitudes.items():
            if attitude is None:
                continue
            reference = self.panel.recording_reference_attitudes.setdefault(camera, attitude)
            delta = relative_attitude(reference.quaternion_wxyz, attitude.quaternion_wxyz)
            prefix = f"{camera}_pixhawk"
            row.update(
                {
                    f"{prefix}_reference_q_wxyz": json.dumps(
                        list(reference.quaternion_wxyz), separators=(",", ":")
                    ),
                    f"{prefix}_delta_roll_deg": self.panel._fmt_record_float(delta.roll_deg, 9),
                    f"{prefix}_delta_pitch_deg": self.panel._fmt_record_float(delta.pitch_deg, 9),
                    f"{prefix}_delta_yaw_deg": self.panel._fmt_record_float(delta.yaw_deg, 9),
                    f"{prefix}_delta_rotation_angle_deg": self.panel._fmt_record_float(
                        relative_rotation_angle_deg(reference.quaternion_wxyz, attitude.quaternion_wxyz), 9
                    ),
                }
            )

        if (
            attitudes["left"] is not None
            and attitudes["right"] is not None
            and "left" in self.panel.recording_reference_attitudes
            and "right" in self.panel.recording_reference_attitudes
        ):
            initial_relative_q = quaternion_multiply(
                self.panel.recording_reference_attitudes["right"].quaternion_wxyz,
                quaternion_conjugate(self.panel.recording_reference_attitudes["left"].quaternion_wxyz),
            )
            current_relative_q = quaternion_multiply(
                attitudes["right"].quaternion_wxyz,
                quaternion_conjugate(attitudes["left"].quaternion_wxyz),
            )
            row["left_right_relative_reference_q_wxyz"] = json.dumps(
                list(initial_relative_q), separators=(",", ":")
            )
            row["left_right_relative_q_wxyz"] = json.dumps(
                list(current_relative_q), separators=(",", ":")
            )
            row["left_right_relative_delta_rotation_angle_deg"] = self.panel._fmt_record_float(
                relative_rotation_angle_deg(initial_relative_q, current_relative_q), 9
            )

        if self.panel.recording_origin_gps is None and gps_states["left"] is not None:
            self.panel.recording_origin_gps = gps_states["left"]
        for camera, gps in gps_states.items():
            if gps is None:
                continue
            reference = self.panel.recording_reference_gps.setdefault(camera, gps)
            origin_delta = (
                None if self.panel.recording_origin_gps is None else ned_delta_m(self.panel.recording_origin_gps, gps)
            )
            reference_delta = ned_delta_m(reference, gps)
            prefix = f"{camera}_gps_antenna"
            if origin_delta is not None:
                row[f"{prefix}_north_m"] = self.panel._fmt_record_float(origin_delta[0], 9)
                row[f"{prefix}_east_m"] = self.panel._fmt_record_float(origin_delta[1], 9)
                row[f"{prefix}_down_m"] = self.panel._fmt_record_float(origin_delta[2], 9)
            if reference_delta is not None:
                row[f"{prefix}_delta_north_m"] = self.panel._fmt_record_float(reference_delta[0], 9)
                row[f"{prefix}_delta_east_m"] = self.panel._fmt_record_float(reference_delta[1], 9)
                row[f"{prefix}_delta_down_m"] = self.panel._fmt_record_float(reference_delta[2], 9)
                row[f"{prefix}_delta_position_m"] = self.panel._fmt_record_float(
                    math.sqrt(sum(value * value for value in reference_delta)), 9
                )

        if gps_states["left"] is not None and gps_states["right"] is not None:
            baseline_ned = ned_delta_m(gps_states["left"], gps_states["right"])
            if baseline_ned is not None:
                if self.panel.recording_reference_baseline_ned_m is None:
                    self.panel.recording_reference_baseline_ned_m = baseline_ned
                baseline_delta = tuple(
                    value - reference
                    for value, reference in zip(baseline_ned, self.panel.recording_reference_baseline_ned_m)
                )
                baseline_length_m = math.sqrt(sum(value * value for value in baseline_ned))
                reference_baseline_length_m = math.sqrt(
                    sum(value * value for value in self.panel.recording_reference_baseline_ned_m)
                )
                row.update(
                    {
                        "baseline_north_m": self.panel._fmt_record_float(baseline_ned[0], 9),
                        "baseline_east_m": self.panel._fmt_record_float(baseline_ned[1], 9),
                        "baseline_down_m": self.panel._fmt_record_float(baseline_ned[2], 9),
                        "baseline_length_m": self.panel._fmt_record_float(baseline_length_m, 9),
                        "baseline_reference_length_m": self.panel._fmt_record_float(
                            reference_baseline_length_m, 9
                        ),
                        "baseline_length_change_m": self.panel._fmt_record_float(
                            baseline_length_m - reference_baseline_length_m, 9
                        ),
                        "baseline_delta_north_m": self.panel._fmt_record_float(baseline_delta[0], 9),
                        "baseline_delta_east_m": self.panel._fmt_record_float(baseline_delta[1], 9),
                        "baseline_delta_down_m": self.panel._fmt_record_float(baseline_delta[2], 9),
                        "baseline_delta_length_m": self.panel._fmt_record_float(
                            math.sqrt(sum(value * value for value in baseline_delta)), 9
                        ),
                    }
                )
        return row

    def _record_current_intrinsic_parameters(self) -> dict[str, str]:
        intrinsics_source_zoom = getattr(
            self.panel, "_active_intrinsics_source_zoom_level", None
        )
        extrinsics_source_zoom = getattr(
            self.panel, "_active_extrinsics_source_zoom_level", None
        )
        row: dict[str, str] = {
            "intrinsics_active": "true" if self.panel.active_intrinsics is not None else "false",
            "intrinsics_path": str(
                self.panel._workstation_setting("intrinsics_path", self.panel.settings.intrinsics_path)
            ),
            "intrinsics_zoom_level": str(self.panel.zoom_level_spin.value()),
            "intrinsics_zoom_path": str(self.panel._intrinsics_storage_path()),
            "intrinsics_latest_path": str(self.panel._intrinsics_storage_base_path()),
            "intrinsics_profile_path": str(
                getattr(self.panel, "_active_intrinsics_profile_path", "") or ""
            ),
            "intrinsics_profile_sha256": str(
                getattr(self.panel, "_active_intrinsics_profile_hash", "") or ""
            ),
            "intrinsics_source_zoom_level": (
                "" if intrinsics_source_zoom is None else str(intrinsics_source_zoom)
            ),
            "extrinsics_profile_path": str(
                getattr(self.panel, "_active_extrinsics_profile_path", "") or ""
            ),
            "extrinsics_profile_sha256": str(
                getattr(self.panel, "_active_extrinsics_profile_hash", "") or ""
            ),
            "extrinsics_source_zoom_level": (
                "" if extrinsics_source_zoom is None else str(extrinsics_source_zoom)
            ),
            "calibration_selection_mode": str(
                getattr(self.panel, "_active_calibration_selection_mode", "") or ""
            ),
            "calibration_validation_gate": "disabled",
            "cross_zoom_selection": self.panel._fmt_record_bool(
                intrinsics_source_zoom is not None
                and extrinsics_source_zoom is not None
                and int(intrinsics_source_zoom) != int(extrinsics_source_zoom)
            ),
            "actual_visible_zoom": json.dumps(
                getattr(self.panel, "actual_visible_zoom", {}) or {},
                separators=(",", ":"),
            ),
            "manual_focal_override": self.panel._fmt_record_bool(self.panel.manual_focal_checkbox.isChecked()),
            "runtime_focal_px": self.panel._fmt_record_float(self.panel.focal_spin.value(), 3),
        }
        if self.panel.active_intrinsics is None:
            return row
        for prefix, intrinsics in (("left", self.panel.active_intrinsics.left), ("right", self.panel.active_intrinsics.right)):
            matrix = intrinsics.camera_matrix
            row.update(
                {
                    f"{prefix}_intr_width": str(intrinsics.width),
                    f"{prefix}_intr_height": str(intrinsics.height),
                    f"{prefix}_intr_fx": self.panel._fmt_record_float(matrix[0][0], 6),
                    f"{prefix}_intr_fy": self.panel._fmt_record_float(matrix[1][1], 6),
                    f"{prefix}_intr_cx": self.panel._fmt_record_float(matrix[0][2], 6),
                    f"{prefix}_intr_cy": self.panel._fmt_record_float(matrix[1][2], 6),
                    f"{prefix}_intr_distortion": json.dumps(intrinsics.distortion_coefficients, separators=(",", ":")),
                    f"{prefix}_intr_reprojection_error_px": self.panel._fmt_record_float(
                        intrinsics.reprojection_error_px, 6
                    ),
                    f"{prefix}_intr_image_count": str(intrinsics.image_count),
                }
            )
        return row

    def _record_intrinsic_parameters(
        self, measurement_attempt: MeasurementAttempt
    ) -> dict[str, str]:
        intrinsics_snapshot = measurement_attempt.intrinsics
        row: dict[str, str] = {
            "intrinsics_active": self.panel._fmt_record_bool(intrinsics_snapshot is not None),
            "intrinsics_path": "" if intrinsics_snapshot is None else intrinsics_snapshot.selected_profile_path,
            "intrinsics_zoom_level": "" if intrinsics_snapshot is None else str(intrinsics_snapshot.active_zoom_level),
            "intrinsics_zoom_path": "" if intrinsics_snapshot is None else intrinsics_snapshot.selected_profile_path,
            "intrinsics_latest_path": "" if intrinsics_snapshot is None else intrinsics_snapshot.selected_profile_path,
            "intrinsics_content_sha256": "" if intrinsics_snapshot is None else intrinsics_snapshot.content_sha256,
            "intrinsics_profile_sha256": (
                "" if intrinsics_snapshot is None else intrinsics_snapshot.intrinsics_content_sha256
            ),
            "intrinsics_source_zoom_level": (
                ""
                if intrinsics_snapshot is None
                else str(intrinsics_snapshot.intrinsics_source_zoom_level)
            ),
            "extrinsics_profile_path": (
                ""
                if intrinsics_snapshot is None
                else intrinsics_snapshot.selected_extrinsics_profile_path
            ),
            "extrinsics_profile_sha256": (
                ""
                if intrinsics_snapshot is None
                else intrinsics_snapshot.extrinsics_content_sha256
            ),
            "extrinsics_source_zoom_level": (
                ""
                if intrinsics_snapshot is None
                or intrinsics_snapshot.extrinsics_source_zoom_level is None
                else str(intrinsics_snapshot.extrinsics_source_zoom_level)
            ),
            "calibration_selection_mode": (
                "" if intrinsics_snapshot is None else intrinsics_snapshot.selection_mode
            ),
            "calibration_validation_gate": (
                "" if intrinsics_snapshot is None else intrinsics_snapshot.validation_gate
            ),
            "cross_zoom_selection": self.panel._fmt_record_bool(
                intrinsics_snapshot is not None
                and intrinsics_snapshot.extrinsics_source_zoom_level is not None
                and intrinsics_snapshot.intrinsics_source_zoom_level
                != intrinsics_snapshot.extrinsics_source_zoom_level
            ),
            "manual_focal_override": self.panel._fmt_record_bool(
                measurement_attempt.geometry_config.manual_focal_override
            ),
            "runtime_focal_px": self.panel._fmt_record_float(
                measurement_attempt.geometry_config.focal_length_px, 3
            ),
        }
        if intrinsics_snapshot is None:
            return row
        for prefix, intrinsics in (
            ("left", intrinsics_snapshot.left),
            ("right", intrinsics_snapshot.right),
        ):
            matrix = intrinsics.camera_matrix
            row.update(
                {
                    f"{prefix}_intr_width": str(intrinsics.width),
                    f"{prefix}_intr_height": str(intrinsics.height),
                    f"{prefix}_intr_fx": self.panel._fmt_record_float(matrix[0][0], 6),
                    f"{prefix}_intr_fy": self.panel._fmt_record_float(matrix[1][1], 6),
                    f"{prefix}_intr_cx": self.panel._fmt_record_float(matrix[0][2], 6),
                    f"{prefix}_intr_cy": self.panel._fmt_record_float(matrix[1][2], 6),
                    f"{prefix}_intr_distortion": json.dumps(
                        intrinsics.distortion_coefficients, separators=(",", ":")
                    ),
                    f"{prefix}_intr_reprojection_error_px": self.panel._fmt_record_float(
                        intrinsics.reprojection_error_px, 6
                    ),
                    f"{prefix}_intr_image_count": str(intrinsics.image_count),
                }
            )
        return row

    def _record_current_checkerboard_extrinsics_parameters(self) -> dict[str, object]:
        calibration = self.panel.active_intrinsics
        extrinsics = None if calibration is None else calibration.extrinsics
        row: dict[str, object] = {
            "active": extrinsics is not None,
            "source": "independent_profile_composed_with_intrinsics",
            "profile_path": str(
                getattr(self.panel, "_active_extrinsics_profile_path", "") or ""
            ),
            "profile_sha256": str(
                getattr(self.panel, "_active_extrinsics_profile_hash", "") or ""
            ),
            "source_zoom_level": getattr(
                self.panel, "_active_extrinsics_source_zoom_level", None
            ),
            "validation_gate": "disabled",
            "frame_contract": "opencv_optical:RDF(right,down,forward); X_right = R_left_to_right * X_left + T_left_to_right_m",
        }
        if extrinsics is None:
            return row
        right_origin = self.panel._checkerboard_translation_right_in_left_m()
        row.update(
            {
                "pair_count": int(extrinsics.pair_count),
                "reprojection_error_px": float(extrinsics.reprojection_error_px),
                "rotation_left_to_right": [list(map(float, values)) for values in extrinsics.rotation_left_to_right],
                "translation_left_to_right_m": [float(value) for value in extrinsics.translation_left_to_right_m],
                "right_camera_origin_in_left_rdf_m": (
                    None if right_origin is None else [float(value) for value in right_origin]
                ),
            }
        )
        return row

    def _record_checkerboard_extrinsics_parameters(
        self, measurement_attempt: MeasurementAttempt
    ) -> dict[str, object]:
        calibration = measurement_attempt.intrinsics
        extrinsics = None if calibration is None else calibration.checkerboard_extrinsics
        row: dict[str, object] = {
            "active": extrinsics is not None,
            "source": "measurement_attempt_independent_profile_bundle",
            "profile_path": (
                ""
                if calibration is None
                else calibration.selected_extrinsics_profile_path
            ),
            "profile_sha256": (
                "" if calibration is None else calibration.extrinsics_content_sha256
            ),
            "source_zoom_level": (
                None if calibration is None else calibration.extrinsics_source_zoom_level
            ),
            "validation_gate": (
                "disabled" if calibration is None else calibration.validation_gate
            ),
            "frame_contract": "opencv_optical:RDF(right,down,forward); X_right = R_left_to_right * X_left + T_left_to_right_m",
        }
        if extrinsics is None:
            return row
        right_origin = measurement_attempt.translation_right_in_left_m
        row.update(
            {
                "pair_count": extrinsics.pair_count,
                "reprojection_error_px": extrinsics.reprojection_error_px,
                "rotation_left_to_right": [list(values) for values in extrinsics.rotation_left_to_right],
                "translation_left_to_right_m": list(extrinsics.translation_left_to_right_m),
                "right_camera_origin_in_left_rdf_m": (
                    None if right_origin is None else list(right_origin)
                ),
            }
        )
        return row

    def _record_external_parameters(
        self, *, estimate=None, measurement_attempt: MeasurementAttempt
    ) -> dict[str, str]:
        triangulation_source = measurement_attempt.triangulation_source
        checkerboard_mode = measurement_attempt.measurement_method_id == CHECKERBOARD_STEREO
        relative_pose_mode = measurement_attempt.measurement_geometry_kind == "relative_pose"
        component_sources = dict(measurement_attempt.measurement_component_sources)
        uses_live_gnss = "live_gnss" in component_sources.values()
        uses_live_pixhawk = "live_pixhawk" in component_sources.values()
        attitude_timing_status = classify_attitude_timing(
            uses_pixhawk_attitude=uses_live_pixhawk,
            left_age_s=(
                measurement_attempt.left_attitude_age_s
                if estimate is None
                else estimate.left_attitude_age_s
            ),
            right_age_s=(
                measurement_attempt.right_attitude_age_s
                if estimate is None
                else estimate.right_attitude_age_s
            ),
            pair_delta_s=(
                measurement_attempt.attitude_pair_delta_s
                if estimate is None
                else estimate.attitude_pair_delta_s
            ),
        )
        checkerboard = self.panel._record_checkerboard_extrinsics_parameters(measurement_attempt)
        row: dict[str, str] = {
            "triangulation_extrinsics_source": triangulation_source,
            "measurement_method_id": measurement_attempt.measurement_method_id,
            "measurement_method_instance_id": measurement_attempt.measurement_method_instance_id,
            "measurement_geometry_kind": measurement_attempt.measurement_geometry_kind,
            "measurement_geometry_sha256": measurement_attempt.measurement_geometry_sha256,
            "measurement_selection_reason": measurement_attempt.measurement_selection_reason,
            "measurement_component_sources": json.dumps(
                component_sources, sort_keys=True, separators=(",", ":")
            ),
            "use_rtk_baseline": self.panel._fmt_record_bool(uses_live_gnss),
            "rtk_baseline_applied": self.panel._fmt_record_bool(uses_live_gnss),
            "require_rtk_fixed": self.panel._fmt_record_bool(
                measurement_attempt.geometry_config.require_rtk_fixed_setting
                if uses_live_gnss
                else False
            ),
            "use_triangulation": self.panel._fmt_record_bool(True),
            "distance_mode": measurement_attempt.measurement_method_id,
            "world_frame": WORLD_FRAME,
            "body_frame": BODY_FRAME,
            "camera_frame": CAMERA_FRAME,
            "camera_mount_profile": CAMERA_MOUNT_PROFILE,
            "mount_assumption_model": (
                "checkerboard-stereo-extrinsics-v1"
                if checkerboard_mode
                else "hybrid-candidate-live-components-v1"
                if relative_pose_mode and (uses_live_pixhawk or uses_live_gnss)
                else "optical-pose-relative-r-t-v1"
                if relative_pose_mode
                else "gps-rtk-baseline-only-v1"
            ),
            "mount_assumption_status": (
                "checkerboard stereo extrinsics"
                if checkerboard_mode
                else "resolved candidate/live component geometry"
                if relative_pose_mode
                else "GPS/RTK baseline only"
            ),
            "mount_assumption_detail": (
                "applied_checkerboard_stereo_extrinsics_used_as_fixed_optical_transform"
                if checkerboard_mode
                else "resolved_relative_pose_with_per_dof_sources"
                if relative_pose_mode
                else "live_gps_rtk_baseline_used_as_stereo_translation"
            ),
            "checkerboard_extrinsics_active": self.panel._fmt_record_bool(
                checkerboard_mode and bool(checkerboard["active"])
            ),
            "checkerboard_extrinsics_pair_count": (
                str(checkerboard.get("pair_count", "")) if checkerboard_mode else ""
            ),
            "checkerboard_extrinsics_reprojection_error_px": self.panel._fmt_record_float(
                checkerboard.get("reprojection_error_px") if checkerboard_mode else None, 6
            ),
            "checkerboard_extrinsics_frame_contract": str(checkerboard["frame_contract"]),
            "require_attitude": self.panel._fmt_record_bool(uses_live_pixhawk),
            "attitude_used_for_triangulation": self.panel._fmt_record_bool(uses_live_pixhawk),
            "attitude_timing_policy": ATTITUDE_TIMING_POLICY,
            "attitude_timing_status": attitude_timing_status,
            "attitude_timing_blocks_measurement": self.panel._fmt_record_bool(uses_live_pixhawk),
            "attitude_stale_age_threshold_s": self.panel._fmt_record_float(ATTITUDE_STALE_AGE_S, 3),
            "attitude_very_stale_age_threshold_s": self.panel._fmt_record_float(
                ATTITUDE_VERY_STALE_AGE_S, 3
            ),
            "attitude_stale_pair_delta_threshold_s": self.panel._fmt_record_float(
                ATTITUDE_STALE_PAIR_DELTA_S, 3
            ),
            "attitude_very_stale_pair_delta_threshold_s": self.panel._fmt_record_float(
                ATTITUDE_VERY_STALE_PAIR_DELTA_S, 3
            ),
            "left_attitude_age_s": self.panel._fmt_record_float(
                measurement_attempt.left_attitude_age_s if estimate is None else estimate.left_attitude_age_s, 6
            ),
            "right_attitude_age_s": self.panel._fmt_record_float(
                measurement_attempt.right_attitude_age_s if estimate is None else estimate.right_attitude_age_s, 6
            ),
            "attitude_pair_delta_s": self.panel._fmt_record_float(
                measurement_attempt.attitude_pair_delta_s if estimate is None else estimate.attitude_pair_delta_s, 6
            ),
            "relative_roll_deg": self.panel._fmt_record_float(
                measurement_attempt.relative_roll_deg if estimate is None else estimate.relative_roll_deg, 6
            ),
            "relative_pitch_deg": self.panel._fmt_record_float(
                measurement_attempt.relative_pitch_deg if estimate is None else estimate.relative_pitch_deg, 6
            ),
            "relative_yaw_deg": self.panel._fmt_record_float(
                measurement_attempt.relative_yaw_deg if estimate is None else estimate.relative_yaw_deg, 6
            ),
        }
        baseline = measurement_attempt.baseline
        if baseline is not None:
            camera_translation = baseline.translation_right_in_left_rdf_m
            baseline_forward_geometry = self.panel._baseline_forward_geometry_from_camera_translation(camera_translation)
            ned = baseline.ned_right_from_left_m
            row.update(
                {
                    "rtk_baseline_north_m": self.panel._fmt_record_float(None if ned is None else ned[0], 6),
                    "rtk_baseline_east_m": self.panel._fmt_record_float(None if ned is None else ned[1], 6),
                    "rtk_baseline_down_m": self.panel._fmt_record_float(None if ned is None else ned[2], 6),
                    "baseline_p1_body_frd_state": "WAITING",
                    "baseline_p1_body_frd_forward_m": "",
                    "baseline_p1_body_frd_right_m": "",
                    "baseline_p1_body_frd_down_m": "",
                    "baseline_p1_body_frd_attitude_age_s": "",
                    "baseline_p1_body_frd_frame": BODY_FRAME,
                    "baseline_p1_body_frd_axis_contract": (
                        "P1->P2 in P1 body FRD: [Forward, Right, Down]; metres"
                    ),
                    "rtk_left_age_s": self.panel._fmt_record_float(baseline.left_age_at_capture_s, 6),
                    "rtk_right_age_s": self.panel._fmt_record_float(baseline.right_age_at_capture_s, 6),
                    "rtk_fix_quality": baseline.fix_quality,
                    "rtk_warnings": "; ".join(baseline.warnings),
                    "selected_gps_pair_delta_s": self.panel._fmt_record_float(baseline.gps_pair_delta_s, 6),
                    "selected_left_gps_detection_age_s": self.panel._fmt_record_float(
                        baseline.left_age_from_detection_midpoint_s, 6
                    ),
                    "selected_right_gps_detection_age_s": self.panel._fmt_record_float(
                        baseline.right_age_from_detection_midpoint_s, 6
                    ),
                }
            )
            left_attitude = measurement_attempt.left_attitude
            if ned is not None and left_attitude is not None:
                projection = project_ned_baseline_to_p1_body_frd(
                    ned,
                    left_attitude.quaternion_wxyz,
                    attitude_timestamp_s=left_attitude.app_receive_monotonic_s,
                    attitude_age_s=measurement_attempt.left_attitude_age_s,
                )
                row.update(
                    {
                        "baseline_p1_body_frd_state": "READY",
                        "baseline_p1_body_frd_forward_m": self.panel._fmt_record_float(
                            projection.forward_m, 6
                        ),
                        "baseline_p1_body_frd_right_m": self.panel._fmt_record_float(
                            projection.right_m, 6
                        ),
                        "baseline_p1_body_frd_down_m": self.panel._fmt_record_float(
                            projection.down_m, 6
                        ),
                        "baseline_p1_body_frd_attitude_age_s": self.panel._fmt_record_float(
                            projection.attitude_age_s, 6
                        ),
                    }
                )
            if camera_translation is not None:
                row.update(
                    {
                        "baseline_left_camera_rdf_right_m": self.panel._fmt_record_float(camera_translation[0], 6),
                        "baseline_left_camera_rdf_down_m": self.panel._fmt_record_float(camera_translation[1], 6),
                        "baseline_left_camera_rdf_forward_m": self.panel._fmt_record_float(camera_translation[2], 6),
                    }
                )
            if baseline_forward_geometry is not None:
                angle_deg, effective_m, length_m, forward_m = baseline_forward_geometry
                row.update(
                    {
                        "baseline_rdf_forward_angle_deg": self.panel._fmt_record_float(angle_deg, 6),
                        "baseline_effective_perpendicular_m": self.panel._fmt_record_float(effective_m, 6),
                        "baseline_camera_frame_length_m": self.panel._fmt_record_float(length_m, 6),
                        "baseline_forward_component_m": self.panel._fmt_record_float(forward_m, 6),
                    }
                )
        for camera in ("left", "right"):
            attitude = (
                measurement_attempt.left_attitude
                if camera == "left"
                else measurement_attempt.right_attitude
            )
            attitude_age_s = (
                measurement_attempt.left_attitude_age_s
                if camera == "left"
                else measurement_attempt.right_attitude_age_s
            )
            gps = measurement_attempt.left_gps if camera == "left" else measurement_attempt.right_gps
            gps_age_s = None
            if measurement_attempt.baseline is not None:
                gps_age_s = (
                    measurement_attempt.baseline.left_age_at_capture_s
                    if camera == "left"
                    else measurement_attempt.baseline.right_age_at_capture_s
                )
            if gps_age_s is None and gps is not None:
                frame_timestamp_s = (
                    measurement_attempt.left_detection.app_receive_monotonic_s
                    if camera == "left"
                    else measurement_attempt.right_detection.app_receive_monotonic_s
                )
                gps_age_s = abs(frame_timestamp_s - gps.app_receive_monotonic_s)
            row.update(self.panel._record_extrinsic_state(camera, attitude, attitude_age_s))
            row.update(self.panel._record_gps_state(camera, gps, gps_age_s))
            row.update(self.panel._record_gnss_relative_state(camera))
        return row

    def _record_extrinsic_state(
        self,
        camera: str,
        state: AttitudeSnapshot | None,
        age_s: float | None,
    ) -> dict[str, str]:
        prefix = f"{camera}_pixhawk"
        if state is None:
            return {}
        return {
            f"{prefix}_namespace": state.namespace,
            f"{prefix}_topic": state.topic,
            f"{prefix}_roll_deg": self.panel._fmt_record_float(state.roll_deg, 6),
            f"{prefix}_pitch_deg": self.panel._fmt_record_float(state.pitch_deg, 6),
            f"{prefix}_yaw_deg": self.panel._fmt_record_float(state.yaw_deg, 6),
            f"{prefix}_q": json.dumps(list(state.quaternion_wxyz), separators=(",", ":")),
            f"{prefix}_q_w": self.panel._fmt_record_float(state.quaternion_wxyz[0], 9),
            f"{prefix}_q_x": self.panel._fmt_record_float(state.quaternion_wxyz[1], 9),
            f"{prefix}_q_y": self.panel._fmt_record_float(state.quaternion_wxyz[2], 9),
            f"{prefix}_q_z": self.panel._fmt_record_float(state.quaternion_wxyz[3], 9),
            f"{prefix}_age_s": self.panel._fmt_record_float(age_s, 6),
            f"{prefix}_app_receive_timestamp_s": self.panel._fmt_record_float(state.timestamp_s, 6),
            f"{prefix}_app_receive_monotonic_ns": int(round(state.timestamp_s * 1_000_000_000.0)),
            f"{prefix}_px4_timestamp_s": self.panel._fmt_record_float(state.px4_timestamp_s, 6),
            f"{prefix}_px4_timestamp_us": "" if state.px4_timestamp_us is None else state.px4_timestamp_us,
            f"{prefix}_px4_timestamp_sample_us": (
                "" if state.px4_timestamp_sample_us is None else state.px4_timestamp_sample_us
            ),
            f"{prefix}_delta_q_reset_wxyz": (
                "" if state.delta_q_reset is None else json.dumps(list(state.delta_q_reset), separators=(",", ":"))
            ),
            f"{prefix}_quat_reset_counter": (
                "" if state.quat_reset_counter is None else state.quat_reset_counter
            ),
            f"{prefix}_mavlink_source_system": (
                "" if state.mavlink_source_system is None else state.mavlink_source_system
            ),
            f"{prefix}_mavlink_source_component": (
                "" if state.mavlink_source_component is None else state.mavlink_source_component
            ),
            f"{prefix}_mavlink_sequence": "" if state.mavlink_sequence is None else state.mavlink_sequence,
            f"{prefix}_mavlink_message_id": (
                "" if state.mavlink_message_id is None else state.mavlink_message_id
            ),
        }

    def _record_gps_state(
        self,
        camera: str,
        state: GpsSnapshot | None,
        age_s: float | None,
    ) -> dict[str, str]:
        prefix = f"{camera}_gps"
        if state is None:
            return {}
        return {
            f"{prefix}_namespace": state.namespace,
            f"{prefix}_topic": state.topic,
            f"{prefix}_lat_deg": self.panel._fmt_record_float(state.latitude_deg, 9),
            f"{prefix}_lon_deg": self.panel._fmt_record_float(state.longitude_deg, 9),
            f"{prefix}_alt_m": self.panel._fmt_record_float(state.altitude_m, 6),
            f"{prefix}_altitude_ellipsoid_m": self.panel._fmt_record_float(state.altitude_ellipsoid_m, 6),
            f"{prefix}_fix_type": "" if state.fix_type is None else str(state.fix_type),
            f"{prefix}_satellites": "" if state.satellites_used is None else str(state.satellites_used),
            f"{prefix}_eph_m": self.panel._fmt_record_float(state.eph_m, 6),
            f"{prefix}_epv_m": self.panel._fmt_record_float(state.epv_m, 6),
            f"{prefix}_heading_deg": self.panel._fmt_record_float(state.heading_deg, 6),
            f"{prefix}_heading_accuracy_deg": self.panel._fmt_record_float(state.heading_accuracy_deg, 6),
            f"{prefix}_rtk_label": state.rtk_label,
            f"{prefix}_age_s": self.panel._fmt_record_float(age_s, 6),
            f"{prefix}_app_receive_timestamp_s": self.panel._fmt_record_float(state.timestamp_s, 6),
            f"{prefix}_app_receive_monotonic_ns": int(round(state.timestamp_s * 1_000_000_000.0)),
            f"{prefix}_px4_timestamp_s": self.panel._fmt_record_float(state.px4_timestamp_s, 6),
            f"{prefix}_px4_timestamp_us": "" if state.px4_timestamp_us is None else state.px4_timestamp_us,
            f"{prefix}_px4_timestamp_sample_us": (
                "" if state.px4_timestamp_sample_us is None else state.px4_timestamp_sample_us
            ),
            f"{prefix}_gps_utc_us": "" if state.gps_utc_us is None else state.gps_utc_us,
            f"{prefix}_gps_utc_valid": self.panel._fmt_record_bool(state.gps_utc_us is not None),
            f"{prefix}_timestamp_time_relative_us": (
                ""
                if state.gps_timestamp_time_relative_us is None
                else state.gps_timestamp_time_relative_us
            ),
            f"{prefix}_velocity_n_m_s": self.panel._fmt_record_float(state.velocity_n_m_s, 6),
            f"{prefix}_velocity_e_m_s": self.panel._fmt_record_float(state.velocity_e_m_s, 6),
            f"{prefix}_velocity_d_m_s": self.panel._fmt_record_float(state.velocity_d_m_s, 6),
            f"{prefix}_velocity_ned_valid": self.panel._fmt_record_bool(state.velocity_ned_valid),
            f"{prefix}_hdop": self.panel._fmt_record_float(state.hdop, 6),
            f"{prefix}_vdop": self.panel._fmt_record_float(state.vdop, 6),
            f"{prefix}_device_id": "" if state.device_id is None else state.device_id,
            f"{prefix}_time_source": state.time_source,
            f"{prefix}_mavlink_source_system": (
                "" if state.mavlink_source_system is None else state.mavlink_source_system
            ),
            f"{prefix}_mavlink_source_component": (
                "" if state.mavlink_source_component is None else state.mavlink_source_component
            ),
            f"{prefix}_mavlink_sequence": "" if state.mavlink_sequence is None else state.mavlink_sequence,
            f"{prefix}_mavlink_message_id": (
                "" if state.mavlink_message_id is None else state.mavlink_message_id
            ),
        }

    def _record_gnss_relative_state(self, camera: str) -> dict[str, str]:
        state = self.panel.latest_gnss_relative.get(camera)
        prefix = f"{camera}_gnss_relative"
        if state is None:
            return {}
        age_s = max(0.0, time.monotonic() - state.timestamp_s)
        heading_deg = None if state.heading_rad is None else math.degrees(state.heading_rad)
        heading_accuracy_deg = None if state.heading_accuracy_rad is None else math.degrees(state.heading_accuracy_rad)
        return {
            f"{prefix}_namespace": state.namespace,
            f"{prefix}_topic": state.topic,
            f"{prefix}_device_id": "" if state.device_id is None else str(state.device_id),
            f"{prefix}_heading_deg": self.panel._fmt_record_float(heading_deg, 6),
            f"{prefix}_heading_accuracy_deg": self.panel._fmt_record_float(heading_accuracy_deg, 6),
            f"{prefix}_position_length_m": self.panel._fmt_record_float(state.position_length_m, 6),
            f"{prefix}_accuracy_length_m": self.panel._fmt_record_float(state.accuracy_length_m, 6),
            f"{prefix}_gnss_fix_ok": self.panel._fmt_record_bool(state.gnss_fix_ok),
            f"{prefix}_differential_solution": self.panel._fmt_record_bool(state.differential_solution),
            f"{prefix}_relative_position_valid": self.panel._fmt_record_bool(state.relative_position_valid),
            f"{prefix}_carrier_solution_floating": self.panel._fmt_record_bool(state.carrier_solution_floating),
            f"{prefix}_carrier_solution_fixed": self.panel._fmt_record_bool(state.carrier_solution_fixed),
            f"{prefix}_moving_base_mode": self.panel._fmt_record_bool(state.moving_base_mode),
            f"{prefix}_heading_valid": self.panel._fmt_record_bool(state.heading_valid),
            f"{prefix}_age_s": self.panel._fmt_record_float(age_s, 6),
            f"{prefix}_px4_timestamp_s": self.panel._fmt_record_float(state.px4_timestamp_s, 6),
            f"{prefix}_px4_timestamp_us": "" if state.px4_timestamp_us is None else state.px4_timestamp_us,
            f"{prefix}_px4_timestamp_sample_us": (
                "" if state.px4_timestamp_sample_us is None else state.px4_timestamp_sample_us
            ),
            f"{prefix}_gps_utc_us": "" if state.gps_utc_us is None else state.gps_utc_us,
            f"{prefix}_gps_utc_valid": self.panel._fmt_record_bool(state.gps_utc_us is not None),
            f"{prefix}_mavlink_source_system": (
                "" if state.mavlink_source_system is None else state.mavlink_source_system
            ),
            f"{prefix}_mavlink_source_component": (
                "" if state.mavlink_source_component is None else state.mavlink_source_component
            ),
            f"{prefix}_mavlink_sequence": "" if state.mavlink_sequence is None else state.mavlink_sequence,
            f"{prefix}_mavlink_message_id": (
                "" if state.mavlink_message_id is None else state.mavlink_message_id
            ),
        }
