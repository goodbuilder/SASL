"""Own save worker/thread completion, retry, and bounded shutdown waiting.

Workers connect to explicit QObject receiver capabilities, preserving GUI-thread
affinity before the compatibility slot delivers to this owner."""
from __future__ import annotations
import json
import time
from collections.abc import Mapping
from pathlib import Path
from PyQt5 import QtCore
from Workstation.workstation_core.session_storage.models import SavedSessionFiles
from Workstation.workstation_core.session_storage.spool import SpoolRows
from Workstation.workstation_core.session_storage.paths import session_id_now
from Workstation.workstation_core.session_storage.recovery import write_recovery_metadata
from Workstation.workers.session_workers import SessionSaveWorker
from .state import (RecordingRowsState, SessionSaveState, RecordingSessionState)
from .ports import SessionSavePort
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .metadata import Metadata
    from .session_start import SessionStart


class SessionSave:
    def __init__(self, *,
        rows: RecordingRowsState,
        save_state: SessionSaveState,
        session: RecordingSessionState,
        boundary: SessionSavePort,
    ) -> None:
        self.rows = rows
        self.save_state = save_state
        self.session = session
        self.boundary = boundary

    def bind_collaborators(self, *,
        metadata: "Metadata",
        session_start: "SessionStart",
    ) -> None:
        self.metadata = metadata
        self.session_start = session_start

    def toggle_distance_recording(self) -> None:
        self.session_start.toggle_session_recording()

    def toggle_video_recording(self) -> None:
        self.session_start.toggle_session_recording()

    def save_distance_history_csv(
        self,
        *,
        final_clock_sync_snapshot: dict[str, object] | None = None,
    ) -> None:
        if not bool(getattr(self.boundary, "_measurement_session_hierarchy_enabled", True)):
            flush = getattr(self.boundary, "flush_workstation_data", None)
            if callable(flush):
                flush()
            self.save_state._session_save_finalized = True
            self.save_state._session_save_retry_required = False
            self.boundary.recording_status_label.setText(
                "데이터 기록: append flush 요청 완료 · 다음 측정 가능"
            )
            self.boundary._refresh_operation_header()
            return
        if self.session.distance_recording_enabled:
            self.boundary.recording_status_label.setText("세션 기록: 기록을 중지한 뒤 저장할 수 있습니다")
            self.boundary._refresh_operation_header()
            return
        if self.save_state._session_save_thread is not None:
            self.boundary.recording_status_label.setText("세션 기록: 저장 작업이 이미 진행 중입니다")
            self.boundary._refresh_operation_header()
            return
        if self.save_state._session_save_finalized:
            self.boundary.recording_status_label.setText("세션 기록: 현재 세션 파일은 이미 확정되었습니다")
            return
        if not self.session.recording_session_id and not self.rows.distance_recording_rows:
            self.boundary.recording_status_label.setText("세션 기록: 저장할 샘플 없음")
            self.boundary._refresh_operation_header()
            return
        stamp = self.session.recording_session_id or session_id_now()
        recovered_output = getattr(
            self.session, "_recovered_session_output_dir", None
        )
        output_dir = (
            Path(recovered_output)
            if recovered_output is not None
            else self.boundary._recording_session_dir(stamp)
        )
        method_evaluation_bundle_rows = getattr(
            self.rows, "method_evaluation_bundle_rows", None
        )
        for rows in (
            self.rows.distance_recording_rows,
            *self.rows.camera_recording_rows.values(),
            *self.rows.px4_recording_rows.values(),
            self.rows.gnss_base_recording_rows,
            self.rows.extrinsics_comparison_recording_rows,
            method_evaluation_bundle_rows,
        ):
            if isinstance(rows, SpoolRows):
                rows.close()
        if self.session._recovered_session_metadata is None:
            metadata = self.metadata._recording_session_metadata(clock_sync_snapshot=final_clock_sync_snapshot)
            write_recovery_metadata(output_dir, metadata)
            self.session._recovered_session_metadata = json.loads(json.dumps(metadata))
        else:
            metadata = json.loads(json.dumps(self.session._recovered_session_metadata))
            metadata.update(
                {
                    "matched_frame_pair_count": len(self.rows.distance_recording_rows),
                    "camera_left_gui_frame_count": len(self.rows.camera_recording_rows["left"]),
                    "camera_right_gui_frame_count": len(self.rows.camera_recording_rows["right"]),
                    "px4_left_sample_count": len(self.rows.px4_recording_rows["left"]),
                    "px4_right_sample_count": len(self.rows.px4_recording_rows["right"]),
                    "px4_drone_sample_count": len(
                        self.rows.px4_recording_rows.get("drone", ())
                    ),
                    "gnss_base_sample_count": len(self.rows.gnss_base_recording_rows),
                    "extrinsics_comparison_sample_count": len(self.rows.extrinsics_comparison_recording_rows),
                    "method_evaluation_bundle_count": len(
                        self.rows.method_evaluation_bundle_rows
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
            optical_pose = self.metadata._recording_optical_pose_session_data()
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
                    "drone_rtk_capture_observations",
                    "drone_rtk_multiview_samples",
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
        self.session._recovered_session_metadata = json.loads(json.dumps(metadata))
        if isinstance(self.rows.distance_recording_rows, SpoolRows):
            pair_rows = self.rows.distance_recording_rows
        else:
            pair_rows = [dict(row) for row in self.rows.distance_recording_rows]
        # RecordingController is a plain Python delegate, not a QObject.  The
        # hidden Workstation panel is the actual Qt lifecycle owner and must
        # parent the save thread so construction and shutdown are valid.
        thread = QtCore.QThread(self.boundary.qobject_parent)
        worker = SessionSaveWorker(
            output_dir=output_dir,
            metadata=metadata,
            pair_rows=pair_rows,
            camera_rows=dict(self.rows.camera_recording_rows),
            px4_rows=dict(self.rows.px4_recording_rows),
            gnss_base_rows=self.rows.gnss_base_recording_rows,
            extrinsics_comparison_rows=self.rows.extrinsics_comparison_recording_rows,
            method_evaluation_bundle_rows=method_evaluation_bundle_rows,
        )
        worker.moveToThread(thread)
        thread.started.connect(worker.run)
        worker.saved.connect(self.boundary._handle_session_saved)
        worker.failed.connect(self.boundary._handle_session_save_failed)
        worker.finished.connect(thread.quit, QtCore.Qt.DirectConnection)
        worker.finished.connect(worker.deleteLater)
        thread.finished.connect(self.boundary._handle_session_save_thread_finished)
        self.save_state._session_save_thread = thread
        self.save_state._session_save_worker = worker
        self.save_state._session_save_retry_required = False
        self.save_state._session_save_last_error = ""
        self.boundary.record_button.setEnabled(False)
        self.boundary.save_history_button.setEnabled(False)
        self.boundary.recording_status_label.setText(
            f"세션 기록: 저장 중, 매칭 프레임쌍={len(pair_rows)}개"
        )
        self.boundary._log(f"[저장] 세션 파일을 백그라운드에서 확정 중입니다: {output_dir}")
        thread.start()
        self.boundary._refresh_operation_header()

    @QtCore.pyqtSlot(object)
    def _handle_session_saved(self, saved_files: SavedSessionFiles) -> None:
        self.boundary._log(
            "[저장] canonical JSONL 원본을 저장했습니다: "
            + ", ".join(str(path) for path in saved_files.authoritative_jsonl_files)
        )
        self.boundary._log(
            "[저장] derived typed Parquet을 저장했습니다: "
            f"pairs={saved_files.measurement_pairs_parquet}, "
            f"camera={saved_files.camera_timeline_parquet}, "
            f"pixhawk={saved_files.pixhawk_timeline_parquet}, "
            f"gnss={saved_files.gnss_base_timeline_parquet}, "
            f"geometry={saved_files.geometry_comparison_parquet}"
        )
        self.boundary._log(f"[저장] 측정 context를 저장했습니다: {saved_files.context_json}")
        self.boundary._log(f"[저장] 측정 manifest를 저장했습니다: {saved_files.manifest}")
        if saved_files.review_pending:
            self.boundary._log(
                "[저장] review.xlsx는 모든 측정을 통합해 Close Session 시 생성됩니다."
            )
        elif saved_files.review_error:
            self.boundary._log(
                "[저장] JSONL 원본은 확정되었지만 review.xlsx 재생성은 실패했습니다: "
                + saved_files.review_error
            )
        else:
            self.boundary._log(f"[저장] 세션 통합 XLSX를 저장했습니다: {saved_files.review_xlsx}")
        output_text = ", ".join(
            str(output) for output in saved_files.outputs if output.exists()
        )
        self.boundary.recording_status_label.setText(
            "세션 기록 저장 완료: "
            f"매칭 프레임쌍={len(self.rows.distance_recording_rows)}개 - {output_text}"
        )
        self.save_state._session_save_retry_required = False
        self.save_state._session_save_last_error = ""
        self.save_state._session_save_finalized = True
        self.session._recovered_session_output_dir = None
        self.boundary._refresh_operation_header()
        continuation = getattr(
            self.boundary, "_continue_measurement_package_finalization", None
        )
        if callable(continuation):
            continuation()

    @QtCore.pyqtSlot(str)
    def _handle_session_save_failed(self, error: str) -> None:
        self.save_state._session_save_retry_required = True
        self.save_state._session_save_finalized = False
        self.save_state._session_save_last_error = str(error)
        self.boundary._log(f"[저장] 세션 파일 확정 실패; spool을 보존했습니다: {error}")
        self.boundary.recording_status_label.setText(f"세션 저장 실패(재시도 가능): {error}")
        self.boundary._refresh_operation_header()
        continuation = getattr(
            self.boundary, "_continue_measurement_package_finalization", None
        )
        if callable(continuation):
            continuation()

    @QtCore.pyqtSlot()
    def _handle_session_save_thread_finished(self) -> None:
        thread = self.save_state._session_save_thread
        self.save_state._session_save_thread = None
        self.save_state._session_save_worker = None
        self.save_state._session_save_shutdown_wait_started_s = None
        self.save_state._session_save_shutdown_timeout_logged = False
        if thread is not None:
            thread.deleteLater()
        self.boundary.save_history_button.setEnabled(not self.save_state._session_save_finalized)
        self.boundary.record_button.setEnabled(not self.save_state._session_save_retry_required)
        self.boundary._refresh_operation_header()
        continuation = getattr(
            self.boundary, "_continue_measurement_package_finalization", None
        )
        if callable(continuation):
            continuation()

    def _wait_for_session_save(self, timeout_ms: int = 15000) -> bool:
        thread = self.save_state._session_save_thread
        if thread is None:
            self.save_state._session_save_shutdown_wait_started_s = None
            self.save_state._session_save_shutdown_timeout_logged = False
            return True
        try:
            running = thread.isRunning()
        except RuntimeError:
            self.save_state._session_save_thread = None
            self.save_state._session_save_worker = None
            return True
        now_s = time.monotonic()
        started_s = getattr(self.save_state, "_session_save_shutdown_wait_started_s", None)
        if started_s is None:
            started_s = now_s
            self.save_state._session_save_shutdown_wait_started_s = started_s
        timeout_s = max(0.0, float(timeout_ms) / 1000.0)
        if (
            running
            and now_s - float(started_s) >= timeout_s
            and not bool(getattr(self.save_state, "_session_save_shutdown_timeout_logged", False))
        ):
            self.boundary._log(
                "[저장] 종료 대기 시간이 초과되었습니다. 원본 spool과 manifest를 보존해 재시도할 수 있습니다."
            )
            self.save_state._session_save_shutdown_timeout_logged = True
        # Keep the Qt event loop alive so saved/failed/finished signals can be
        # delivered.  The top-level closeEvent retries this lifecycle step.
        return False
