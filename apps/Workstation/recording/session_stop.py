"""Close recording producers after the named evidence and method barriers.

Generation-guarded retries retain the original stop request. External controllers
are represented by pending-count, snapshot, stop, and flush operations only."""
from __future__ import annotations
from collections.abc import Mapping
from pathlib import Path
from PyQt5 import QtCore
from Workstation.controllers.run_evidence_controller import persistence_failure_error
from Workstation.workstation_core.mp4_integrity import H264_MP4_INTEGRITY_SCHEMA
from Workstation.workstation_core.media_stream.policy import RECORDING_ENCODING_DECODED_NVENC_LOSSLESS, RECORDING_ENCODING_H264_COPY_GATED
from Workstation.workstation_core.media_stream.configuration import recording_integrity_contract
from .state import (RecordingRowsState, RecordingSessionState, VideoSegmentState)
from .ports import SessionStopPort
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .native_submission import NativeSubmission
    from .photo_capture import PhotoCapture
    from .presentation import Presentation
    from .session_save import SessionSave
    from .ulog import Ulog
    from .video_segments import VideoSegments


class SessionStop:
    def __init__(self, *,
        rows: RecordingRowsState,
        session: RecordingSessionState,
        video_state: VideoSegmentState,
        boundary: SessionStopPort,
    ) -> None:
        self.rows = rows
        self.session = session
        self.video_state = video_state
        self.boundary = boundary

    def bind_collaborators(self, *,
        native_submission: "NativeSubmission",
        photo_capture: "PhotoCapture",
        presentation: "Presentation",
        session_save: "SessionSave",
        ulog: "Ulog",
        video_segments: "VideoSegments",
    ) -> None:
        self.native_submission = native_submission
        self.photo_capture = photo_capture
        self.presentation = presentation
        self.session_save = session_save
        self.ulog = ulog
        self.video_segments = video_segments

    def _schedule_legacy_recording_stop_retry(
        self,
        *,
        restart_streams: bool,
        final_clock_sync_snapshot: dict[str, object] | None,
        barrier: str,
    ) -> None:
        generation = int(
            getattr(self.session, "_recording_stop_retry_generation", 0) or 0
        ) + 1
        self.session._recording_stop_retry_generation = generation
        self.session._recording_stop_retry_request = {
            "generation": generation,
            "recording_session_id": str(
                getattr(self.session, "recording_session_id", "") or ""
            ),
            "restart_streams": bool(restart_streams),
            "final_clock_sync_snapshot": final_clock_sync_snapshot,
            "barrier": str(barrier),
        }
        timer = getattr(self.session, "_recording_stop_retry_timer", None)
        if timer is None:
            parent = self.boundary.qobject_parent if isinstance(self.boundary.qobject_parent, QtCore.QObject) else None
            timer = QtCore.QTimer(parent)
            timer.setSingleShot(True)
            timer.timeout.connect(self._retry_legacy_recording_stop)
            self.session._recording_stop_retry_timer = timer
        timer.start(100)

    def _cancel_legacy_recording_stop_retry(self) -> None:
        self.session._recording_stop_retry_generation = int(
            getattr(self.session, "_recording_stop_retry_generation", 0) or 0
        ) + 1
        self.session._recording_stop_retry_request = None
        self.boundary._method_evaluation_stop_retry_scheduled = False
        self.session._native_frame_stop_retry_scheduled = False
        timer = getattr(self.session, "_recording_stop_retry_timer", None)
        if timer is not None:
            try:
                timer.stop()
            except RuntimeError:
                pass

    def _retry_legacy_recording_stop(self) -> None:
        request = getattr(self.session, "_recording_stop_retry_request", None)
        if not isinstance(request, Mapping):
            return
        generation = int(request.get("generation") or -1)
        if generation != int(
            getattr(self.session, "_recording_stop_retry_generation", 0) or 0
        ):
            return
        expected_session = str(request.get("recording_session_id") or "")
        current_session = str(
            getattr(self.session, "recording_session_id", "") or ""
        )
        if expected_session != current_session:
            self._cancel_legacy_recording_stop_retry()
            return
        active = getattr(self.boundary, "_session_recording_active", None)
        if callable(active) and not bool(active()):
            self._cancel_legacy_recording_stop_retry()
            return
        self.session._recording_stop_retry_request = None
        self.boundary._method_evaluation_stop_retry_scheduled = False
        self.session._native_frame_stop_retry_scheduled = False
        self._stop_session_recording(
            restart_streams=bool(request.get("restart_streams", True)),
            final_clock_sync_snapshot=request.get("final_clock_sync_snapshot"),
        )

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

        if not bool(getattr(self.session, "_automatic_session_recording", False)):
            return True
        if not self.boundary._session_recording_active():
            self.session._automatic_session_recording = False
            return True
        stop_kwargs: dict[str, object] = {"restart_streams": bool(restart_streams)}
        if final_clock_sync_snapshot is not None:
            stop_kwargs["final_clock_sync_snapshot"] = final_clock_sync_snapshot
        self._stop_session_recording(**stop_kwargs)
        if self.boundary._session_recording_active():
            return False
        self.session._automatic_session_recording = False
        return True

    def _stop_session_recording(
        self,
        *,
        restart_streams: bool = True,
        final_clock_sync_snapshot: dict[str, object] | None = None,
    ) -> None:
        if not bool(getattr(self.boundary, "_measurement_session_hierarchy_enabled", True)):
            self._cancel_legacy_recording_stop_retry()
            # Sessionless stop is a resource boundary, not a persistence
            # transaction. Optional/late writers retain their own bounded
            # ownership and never delay the next Measurement.
            self.boundary._method_evaluation_accepting = False
            if final_clock_sync_snapshot is not None:
                self.session._recording_clock_sync_end_observation = final_clock_sync_snapshot
            self.session.distance_recording_enabled = False
            self.session.video_recording_enabled = False
            self.video_segments._set_video_recording_start_snapshot(
                {},
                detail="video recording stopped",
                refresh=False,
            )
            self.session.distance_recording_started_monotonic_s = None
            sync_rtcm_context = getattr(
                self.boundary, "_sync_rtcm_evidence_context", None
            )
            if callable(sync_rtcm_context):
                try:
                    sync_rtcm_context()
                except Exception as exc:
                    # Must never delay/block the stop path; only surface that
                    # RTCM evidence sync for this session may be incomplete.
                    self.boundary._log(f"[저장] RTCM evidence sync 실패(계속 진행): {exc}")
            was_manual = bool(getattr(self.session, "_manual_recording_active", False))
            self.session._manual_recording_active = False
            for rows in (
                getattr(self.rows, "distance_recording_rows", ()),
                *(getattr(self.rows, "camera_recording_rows", {}) or {}).values(),
                *(getattr(self.rows, "px4_recording_rows", {}) or {}).values(),
                getattr(self.rows, "gnss_base_recording_rows", ()),
                getattr(self.rows, "extrinsics_comparison_recording_rows", ()),
                getattr(self.rows, "method_evaluation_bundle_rows", ()),
            ):
                closer = getattr(rows, "close", None)
                if callable(closer):
                    try:
                        closer()
                    except Exception as exc:
                        # Never let one writer's close failure block the rest
                        # of the stop path; just record that data may be lost.
                        self.boundary._log(f"[저장] 기록 writer close 실패(계속 진행): {exc}")
            append = getattr(self.boundary, "append_workstation_data", None)
            if callable(append):
                try:
                    append(
                        "events",
                        {
                            "event_type": "recording_stopped",
                            "recording_mode": str(
                                getattr(self.session, "_recording_mode", "manual") or "manual"
                            ),
                            "runtime_generation": str(
                                getattr(self.session, "recording_session_id", "") or ""
                            ),
                            "was_manual": was_manual,
                        },
                        priority=True,
                    )
                except Exception as exc:
                    # Evidence append failures must never block the stop path.
                    self.boundary._log(
                        f"[저장] recording_stopped 이벤트 기록 실패(계속 진행): {exc}"
                    )
                for side, raw_path in (
                    getattr(self.video_state, "video_recording_paths", {}) or {}
                ).items():
                    path = Path(raw_path).expanduser()
                    backend = self.boundary.stream_backend_combo.currentData()
                    pane = getattr(self.boundary, f"{side}_pane", None)
                    finalizations = getattr(pane, "recording_finalizations", {})
                    finalization = (
                        dict(finalizations.get(str(path), {}))
                        if isinstance(finalizations, dict)
                        else {}
                    )
                    default_contract = recording_integrity_contract(backend)
                    encoding_mode = str(
                        finalization.get("recording_encoding_mode")
                        or default_contract["recording_encoding_mode"]
                    )
                    try:
                        inferred_present = path.is_file() and path.stat().st_size > 0
                    except OSError:
                        inferred_present = False
                    if (
                        not finalization
                        and inferred_present
                        and encoding_mode == RECORDING_ENCODING_DECODED_NVENC_LOSSLESS
                    ):
                        finalization = {
                            **default_contract,
                            "path": str(path),
                            "stream_backend": backend,
                            "success": True,
                            "container_finalization_success": True,
                            "container_finalization_inferred": True,
                            "detail": (
                                "camera child closed with non-empty decoded NVENC MP4; "
                                "output validation pending"
                            ),
                            "integrity_validation_status": "PENDING",
                            "recording_integrity_verified": False,
                            "recording_file_integrity_verified": False,
                            "source_media_integrity_verified": False,
                            "recording_usable": True,
                            "integrity_failure_reasons": [
                                "finalized_mp4_output_validation_pending"
                            ],
                        }
                    integrity_pass = bool(
                        finalization.get("integrity_schema")
                        == H264_MP4_INTEGRITY_SCHEMA
                        and finalization.get("integrity_validation_status") == "PASS"
                        and finalization.get("recording_usable") is True
                        and finalization.get("recording_integrity_verified") is True
                    )
                    h264_copy = encoding_mode == RECORDING_ENCODING_H264_COPY_GATED
                    integrity_status = str(
                        finalization.get("integrity_validation_status")
                        or ("PENDING" if not finalization else "FAIL")
                    )
                    integrity_failure_reasons = list(
                        finalization.get("integrity_failure_reasons", ()) or ()
                    )
                    if h264_copy and finalization and not integrity_pass:
                        integrity_status = "FAIL"
                        if (
                            finalization.get("integrity_schema")
                            != H264_MP4_INTEGRITY_SCHEMA
                        ):
                            integrity_failure_reasons.append(
                                "integrity_validation_schema_invalid"
                            )
                        if not integrity_failure_reasons:
                            integrity_failure_reasons.append(
                                "finalized_mp4_integrity_evidence_incomplete"
                            )
                        integrity_failure_reasons = list(
                            dict.fromkeys(integrity_failure_reasons)
                        )
                    finalization_state = (
                        "success"
                        if finalization.get("success") is True
                        and (integrity_pass if h264_copy else True)
                        else "failed"
                        if finalization.get("success") is False
                        or (h264_copy and finalization.get("success") is True)
                        else "pending"
                    )
                    presence_row = self.video_segments.record_measurement_video_segment_presence(
                        str(side),
                        path,
                        detail=str(finalization.get("detail") or "recording stopped"),
                        runtime_generation=str(
                            getattr(self.session, "recording_session_id", "") or ""
                        ),
                    )
                    present = bool(presence_row.get("file_present"))
                    size_bytes = max(0, int(presence_row.get("size_bytes") or 0))
                    validated_usable = integrity_pass
                    try:
                        append(
                            "artifacts",
                            {
                                "event_type": "video_closed",
                                "runtime_generation": str(
                                    getattr(self.session, "recording_session_id", "")
                                    or ""
                                ),
                                "side": str(side),
                                "path": str(path),
                                "media_type": "video/mp4",
                                "stream_backend": backend,
                                **default_contract,
                                "recording_encoding_mode": encoding_mode,
                                "recording_integrity_scope": str(
                                    finalization.get("recording_integrity_scope")
                                    or default_contract["recording_integrity_scope"]
                                ),
                                "container_finalization_success": bool(
                                    finalization.get(
                                        "container_finalization_success",
                                        finalization.get("success", False),
                                    )
                                ),
                                "container_finalization_inferred": bool(
                                    finalization.get(
                                        "container_finalization_inferred",
                                        False,
                                    )
                                ),
                                "integrity_validation_status": integrity_status,
                                "recording_integrity_verified": bool(integrity_pass),
                                "recording_usable": bool(
                                    finalization.get("recording_usable", False)
                                    if not h264_copy
                                    else integrity_pass
                                ),
                                "source_media_integrity_verified": bool(
                                    finalization.get(
                                        "source_media_integrity_verified",
                                        False,
                                    )
                                ),
                                "recording_file_integrity_verified": bool(
                                    finalization.get(
                                        "recording_file_integrity_verified",
                                        integrity_pass,
                                    )
                                ),
                                "integrity_failure_reasons": integrity_failure_reasons,
                                "finalization_state": finalization_state,
                                "finalization_detail": str(
                                    finalization.get("detail") or ""
                                ),
                                "present": present,
                                "presence_state": "SAVED" if present else "FAILED",
                                "size_bytes": size_bytes,
                            },
                            priority=True,
                        )
                    except Exception as exc:
                        # Evidence append failures must never block the stop
                        # path; the video file on disk is unaffected either way.
                        self.boundary._log(
                            f"[저장] video_closed 이벤트 기록 실패(계속 진행): {exc}"
                        )
            self.session.recording_session_id = ""
            self.session._recording_mode = "idle"
            flush = getattr(self.boundary, "flush_workstation_data", None)
            if callable(flush):
                try:
                    flush()
                except Exception as exc:
                    # Flush failure must never block the stop path; surface it
                    # so an operator can tell that this session's data may be
                    # incomplete on disk.
                    self.boundary._log(f"[저장] workstation data flush 실패(계속 진행): {exc}")
            record_button = getattr(self.boundary, "record_button", None)
            setter = getattr(record_button, "setText", None)
            if callable(setter):
                setter("수동 기록 시작")
            self.boundary._log(
                "[저장] 데이터 기록을 중지했습니다; background 저장은 다음 측정을 차단하지 않습니다."
            )
            if restart_streams and self.boundary.camera_connected:
                self.boundary._start_camera_streams(
                    enable_detection=self.boundary.yolo_enabled,
                    switch_to_depth=self.boundary.yolo_enabled,
                )
            resume = getattr(self.boundary, "_resume_queued_measurement_start", None)
            if callable(resume):
                resume()
            refresh = getattr(self.presentation, "_refresh_recording_status_label", None)
            if callable(refresh):
                refresh()
            refresh = getattr(self.boundary, "_refresh_operation_header", None)
            if callable(refresh):
                refresh()
            return
        if self.photo_capture._camera_photo_capture_active():
            self.boundary.recording_status_label.setText("세션 기록: HR/OZ still 캡처 완료 대기 중")
            return
        pending_counter = getattr(
            self.boundary, "method_evaluation_pending_count", None
        )
        if callable(pending_counter):
            self.boundary._method_evaluation_accepting = False
            pending_method_pairs = pending_counter()
            if pending_method_pairs:
                self.boundary.recording_status_label.setText(
                    f"세션 기록: method 결과 barrier 대기 ({pending_method_pairs} pairs)"
                )
                if not bool(
                    getattr(
                        self.boundary,
                        "_method_evaluation_stop_retry_scheduled",
                        False,
                    )
                ):
                    self.boundary._method_evaluation_stop_retry_scheduled = True
                    self._schedule_legacy_recording_stop_retry(
                        restart_streams=restart_streams,
                        final_clock_sync_snapshot=final_clock_sync_snapshot,
                        barrier="method_evaluation",
                    )
                return
        pending_native_frames = self.native_submission.native_frame_persistence_pending_count()
        if pending_native_frames:
            self.boundary.recording_status_label.setText(
                f"세션 기록: native frame 저장 barrier 대기 ({pending_native_frames} requests)"
            )
            if not bool(
                getattr(self.session, "_native_frame_stop_retry_scheduled", False)
            ):
                self.session._native_frame_stop_retry_scheduled = True
                self._schedule_legacy_recording_stop_retry(
                    restart_streams=restart_streams,
                    final_clock_sync_snapshot=final_clock_sync_snapshot,
                    barrier="native_frame_persistence",
                )
            return
        self._cancel_legacy_recording_stop_retry()
        self.native_submission._shutdown_native_frame_stores()
        if final_clock_sync_snapshot is None:
            final_clock_sync_snapshot = (
                self.boundary._camera_time_sync_snapshot("last_observed_before_session_stop")
                if self.boundary._session_recording_active()
                else None
            )
        if final_clock_sync_snapshot is not None:
            self.session._recording_clock_sync_end_observation = final_clock_sync_snapshot
        try:
            self.ulog._finalize_workstation_ulogs()
        except Exception as exc:
            self.boundary.recording_status_label.setText(
                f"세션 기록: ULog finalization failed ({exc})"
            )
            self.boundary._log(f"[저장] ULog finalization 실패: {exc}")
            return
        evidence_flusher = getattr(
            self.boundary,
            "flush_workstation_run_evidence",
            None,
        )
        if callable(evidence_flusher) and not bool(evidence_flusher()):
            self.boundary.recording_status_label.setText(
                "세션 기록: run evidence flush 실패 — 복구 가능한 spool을 보존합니다"
            )
            return
        target_snapshot_reader = getattr(self.boundary, "target_drone_stream_snapshot", None)
        if callable(target_snapshot_reader):
            try:
                target_snapshot = target_snapshot_reader()
                if target_snapshot.get("worker_alive") or target_snapshot.get("phase") in {
                    "RUNNING",
                    "STARTING",
                }:
                    target_snapshot = self.boundary.stop_target_drone_stream(timeout_s=3.0)
                artifact = target_snapshot.get("artifact")
                if isinstance(artifact, dict):
                    registration = self.ulog._register_session_owned_artifact(
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
                    error = persistence_failure_error(registration)
                    if error is not None:
                        raise OSError(error)
            except Exception as exc:
                self.boundary.recording_status_label.setText(
                    f"세션 기록: Target Drone drain/artifact 실패 ({exc})"
                )
                self.boundary._log(f"[저장] Target Drone finalization 실패: {exc}")
                return
        self.session.distance_recording_enabled = False
        self.session.video_recording_enabled = False
        self.session.distance_recording_started_monotonic_s = None
        self.boundary.record_button.setText("세션 기록 시작")
        self.boundary._log(
            "[저장] 세션 기록을 중지했습니다: "
            f"session={self.session.recording_session_id or 'none'}, {self.boundary._format_video_recording_paths()}"
        )
        if restart_streams and self.boundary.camera_connected:
            self.boundary._start_camera_streams(enable_detection=self.boundary.yolo_enabled, switch_to_depth=self.boundary.yolo_enabled)
        self.session_save.save_distance_history_csv(final_clock_sync_snapshot=final_clock_sync_snapshot)
        self.presentation._refresh_recording_status_label()
        self.boundary._refresh_operation_header()
