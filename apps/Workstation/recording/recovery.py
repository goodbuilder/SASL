"""Restore retryable recording rows and capture records from session evidence.

Malformed JSONL remains a hard error; immutable recovery provenance is retained."""
from __future__ import annotations
import hashlib
import json
from pathlib import Path
from Workstation.workstation_core.session_storage.recovery import discover_retryable_sessions, load_retryable_session
from Workstation.workstation_core.strict_jsonl import prepare_jsonl_append
from .state import (PhotoCaptureState, RecordingRowsState, SessionSaveState, RecordingSessionState)
from .ports import RecoveryPort
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .presentation import Presentation


class Recovery:
    def __init__(self, *,
        capture_state: PhotoCaptureState,
        rows: RecordingRowsState,
        save_state: SessionSaveState,
        session: RecordingSessionState,
        boundary: RecoveryPort,
    ) -> None:
        self.capture_state = capture_state
        self.rows = rows
        self.save_state = save_state
        self.session = session
        self.boundary = boundary
        self.discover_sessions = discover_retryable_sessions
        self.load_session = load_retryable_session

    def bind_collaborators(self, *,
        presentation: "Presentation",
    ) -> None:
        self.presentation = presentation

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

    def _recover_retryable_recording_session(self) -> bool:
        try:
            candidates = self.discover_sessions(
                self.boundary.app_root / "recordings"
            )
        except (FileExistsError, OSError, ValueError) as exc:
            self.boundary._log(f"[저장] 복구 세션 경로 충돌: {exc}")
            self.boundary.recording_status_label.setText(
                "세션 기록: direct/legacy 복구 경로 충돌 확인 필요"
            )
            return False
        if not candidates:
            return False
        recovered = None
        for candidate in candidates:
            try:
                recovered = self.load_session(candidate)
                break
            except Exception as exc:
                self.boundary._log(
                    f"[저장] 미완료 세션 복구 정보를 건너뜁니다: "
                    f"path={candidate}, error={exc}"
                )
        if recovered is None:
            return False
        metadata = recovered.metadata

        def optional_int(value: object) -> int | None:
            return None if value in (None, "") else int(value)

        self.session.recording_session_id = str(metadata.get("session_id") or recovered.output_dir.name)
        recovered_measurement_id = str(metadata.get("measurement_id") or "")
        if recovered_measurement_id and bool(
            getattr(self.boundary, "_measurement_session_hierarchy_enabled", False)
        ):
            self.boundary._measurement_session_id = self.session.recording_session_id
            self.boundary._measurement_id = recovered_measurement_id
            try:
                from Workstation.workstation_core.measurement_session import MeasurementSession

                session_root = recovered.output_dir.parent.parent
                session = MeasurementSession.open(session_root)
                self.boundary._measurement_session = session
                self.boundary._measurement_session_state = session.state
                self.boundary._measurement_package = session.measurement_package(
                    recovered_measurement_id
                )
                self.boundary._measurement_package_state = "RECOVERABLE"
            except Exception as exc:
                self.boundary._log(f"[저장] 측정 세션 인덱스 복구 실패: {exc}")
        self.session.recording_session_started_unix_ns = optional_int(
            metadata.get("session_started_unix_ns")
        )
        self.session.recording_session_started_monotonic_ns = optional_int(
            metadata.get("session_started_monotonic_ns")
        )
        self.session.recording_session_anchor_uncertainty_ns = optional_int(
            metadata.get("session_time_anchor_uncertainty_ns")
        )
        self.session.recording_session_started_wall_s = (
            None
            if self.session.recording_session_started_unix_ns is None
            else self.session.recording_session_started_unix_ns * 1e-9
        )
        self.session.recording_session_started_monotonic_s = (
            None
            if self.session.recording_session_started_monotonic_ns is None
            else self.session.recording_session_started_monotonic_ns * 1e-9
        )
        self.rows.distance_recording_rows = recovered.pair_rows
        self.rows.camera_recording_rows = recovered.camera_rows
        self.rows.px4_recording_rows = recovered.px4_rows
        self.rows.gnss_base_recording_rows = recovered.gnss_base_rows
        self.rows.extrinsics_comparison_recording_rows = recovered.extrinsics_comparison_rows
        self.rows.method_evaluation_bundle_rows = recovered.method_evaluation_bundle_rows
        # HR/OZ capture event_index/capture_id counters are derived from
        # len(pnp_photo_capture_records)/len(oz_photo_capture_records); without
        # restoring them here they would silently restart at 1 on top of a
        # recovered session's existing hr_captures.jsonl/oz_captures.jsonl,
        # producing duplicate event_index/capture_id values in that file.
        self.capture_state.pnp_photo_capture_records = self._load_jsonl_records(
            recovered.output_dir / "hr_captures.jsonl"
        )
        self.capture_state.oz_photo_capture_records = self._load_jsonl_records(
            recovered.output_dir / "oz_captures.jsonl"
        )
        self.session._recovered_session_metadata = metadata
        # Pin retries to the exact discovered tree.  This prevents a legacy
        # ``recordings/sessions/<id>`` spool from being read there and then
        # finalized into the new direct ``recordings/<id>`` tree.
        self.session._recovered_session_output_dir = recovered.output_dir
        self.save_state._session_save_retry_required = True
        self.save_state._session_save_finalized = False
        self.session._automatic_session_recording = False
        self.session._recording_mode = str(metadata.get("recording_mode") or "manual")
        self.session.distance_recording_enabled = False
        self.session.video_recording_enabled = False
        self.boundary.record_button.setEnabled(False)
        self.boundary.save_history_button.setEnabled(True)
        self.presentation._refresh_recording_table()
        self.boundary.recording_status_label.setText(
            f"미완료 세션 복구됨: session={self.session.recording_session_id}, "
            f"프레임쌍={len(self.rows.distance_recording_rows)}개 — 세션 파일 저장으로 재시도"
        )
        self.boundary._log(
            f"[저장] 미완료 세션 spool을 복구했습니다: {recovered.output_dir} "
            f"(pair={len(self.rows.distance_recording_rows)})"
        )
        return True
