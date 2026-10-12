"""Workstation checkerboard calibration workflow controller.

Image observations and calibration matrices retain OpenCV optical RDF
(right, down, forward) conventions.  Stereo ``T`` follows
``X_right = R_left_to_right * X_left + T_left_to_right_m``; the right-camera
origin expressed in left RDF is the distinct value ``-R^T T``.  This module
performs no NED/ENU aliasing.
"""

from __future__ import annotations
from Workstation.calibration_workflow.jobs import CALIBRATION_JOB_FIELDS, CalibrationJobs
from Workstation.calibration_workflow.capture_feedback import CalibrationCaptureFeedback
from Workstation.calibration_workflow.calibration_geometry_state import apply_geometry_values
from Workstation.calibration_workflow.inputs import CalibrationInputs
from Workstation.calibration_workflow.capture import CalibrationCapture, LiveCaptureInputs, is_nal1_non_idr_auto_capture_metadata
from Workstation.calibration_workflow.capture_request import CaptureRequestBuilder, CaptureDirectories
from Workstation.calibration_workflow.oz_ledger import OzCaptureLedger
from Workstation.calibration_workflow.ftp import CalibrationFtpTransactions, FtpQueueActions, FTP_RESOURCE_FIELDS
from Workstation.calibration_workflow.timing import CaptureTiming
from Workstation.calibration_workflow.quality import CalibrationQuality
from Workstation.calibration_workflow.profiles import CalibrationProfiles
from Workstation.calibration_workflow.validation import CalibrationValidation, VALIDATION_FIELDS

import json
import hashlib
import math
import os
import time
import uuid
from dataclasses import replace
from copy import deepcopy
from pathlib import Path
from typing import Mapping

from PyQt5 import QtCore, QtGui, QtWidgets

from Workstation.runtime_paths import portable_workstation_data_path_text
from Workstation.workers.calibration_workers import CheckerboardCalibrationWorker, CheckerboardCaptureWorker, CheckerboardExtrinsicsWorker, CheckerboardRandomValidationWorker
from Workstation.calibration_workflow.calibration_sessions import scan_calibration_capture_inventory, write_calibration_session_manifest
from Workstation.workstation_core.calibration import (
    CheckerboardConfig,
    StereoExtrinsics,
    StereoExtrinsicsProfile,
    StereoIntrinsicsResult,
    compose_stereo_calibration,
    independent_zoom_selection_mode,
    intrinsics_content_sha256,
    load_intrinsics_json,
    load_stereo_extrinsics_json,
    opencv_available,
    save_intrinsics_json,
    save_stereo_extrinsics_json,
)
from Workstation.workstation_core.calibration_quality import (
    CalibrationViewGeometry,
    extract_view_geometry,
    pose_novelty,
)
from Workstation.workstation_core.calibration_input import (
    CalibrationInputSnapshot,
    build_calibration_input_snapshot,
)
from Workstation.workstation_core.calibration_timing import (
    CAMERA_TIMESTAMP_VERIFIED,
    EXPOSURE_TIMESTAMP_VERIFIED,
    HOST_DISPATCH_PROXY,
    HOST_RECEIVE_PROXY,
    UNKNOWN_TIMING_EVIDENCE,
    CalibrationTimingEvidence,
    load_calibration_pair_usage,
)
from Workstation.workstation_core.calibration_selection import (
    CALIBRATION_IMAGE_SUFFIXES,
    CALIBRATION_SELECTION_SCHEMA,
    load_calibration_selection_sidecar,
    resolve_calibration_selection_paths,
)
from Workstation.workstation_core.settings import (
    DEFAULT_CALIBRATION_ROOT,
    resolve_settings_path,
)
from Workstation.workstation_core.native_frame_ring import FrameKey
from Workstation.wiris_workers.ftp import WirisFtpBatchDeleteWorker, WirisOzFtpSyncWorker
from Workstation.wiris_workers.commands import WirisImageFormatWorker
from Workstation.wiris_workers.capture import WirisOZPhotoCaptureWorker, WirisPnPPhotoCaptureWorker

CHECKERBOARD_STEREO_PAIR_MAX_DELTA_S = 0.100
CALIBRATION_AUTO_DETECTION_MAX_AGE_S = 2.0
_USE_SELECTED_EXTRINSICS = object()




CALIBRATION_WORKFLOW_METHODS = [
    "_app_relative_path_text",
    "_checkerboard_validation_mode",
    "_checkerboard_validation_root",
    "_capture_zoom_snapshot",
    "_capture_zoom_level_for_paths",
    "_calibration_image_paths_for_root",
    "_calibration_stereo_pair_paths_for_root",
    "_checkerboard_validation_scan_counts",
    "scan_checkerboard_validation_files",
    "choose_checkerboard_validation_root",
    "choose_checkerboard_validation_json",
    "_checkerboard_validation_set_result",
    "_checkerboard_result_baseline_m",
    "_rotation_delta_deg_from_matrices",
    "_format_checkerboard_validation_result_summary",
    "load_checkerboard_validation_json",
    "_checkerboard_random_subset_count",
    "_calibration_calculation_pair_limit",
    "_select_calibration_pairs",
    "_checkerboard_intrinsic_delta_px",
    "_checkerboard_validation_stddev",
    "_calibration_quality_allows_action",
    "_publish_immutable_calibration_file",
    "_publish_immutable_intrinsics_profile",
    "_publish_immutable_extrinsics_profile",
    "workstation_calibration_quality_snapshot",
    "run_checkerboard_random_validation",
    "_handle_random_validation_failed",
    "_handle_random_validation_completed",
    "_checkerboard_validation_result_row",
    "_checkerboard_validation_result_data",
    "save_checkerboard_validation_result",
    "apply_checkerboard_validation_result",
    "apply_intrinsics_to_distance",
    "_checkerboard_config",
    "_archive_loaded_intrinsics",
    "_calibration_dirs",
    "_intrinsics_storage_base_path",
    "_intrinsics_storage_path",
    "_extrinsics_storage_path",
    "_calibration_image_paths",
    "_next_calibration_capture_stem",
    "_checkerboard_capture_pair_timing",
    "_checkerboard_capture_pair_timing_evidence",
    "workstation_calibration_capture_snapshot",
    "capture_calibration_images",
    "toggle_auto_calibration_capture",
    "start_auto_calibration_capture",
    "stop_auto_calibration_capture",
    "_handle_checkerboard_capture_completed",
    "_handle_checkerboard_capture_failed",
    "run_intrinsic_calibration",
    "run_stereo_extrinsics_calibration",
    "_handle_stereo_extrinsics_completed",
    "save_stereo_extrinsics_profile",
    "load_stereo_extrinsics_profile",
    "apply_calibration_bundle",
    "_handle_calibration_progress",
    "_handle_intrinsic_calibration_completed",
    "_handle_intrinsic_calibration_failed",
    "_clear_checkerboard_calibration_worker",
    "workstation_checkerboard_validation_snapshot",
    "configure_workstation_checkerboard_validation",
    "load_workstation_checkerboard_validation_json",
    "run_workstation_checkerboard_validation",
    "save_workstation_checkerboard_validation_result",
    "apply_workstation_checkerboard_validation_result",
    "capture_calibration_images_wiris_ftp",
    "_handle_calibration_wiris_ftp_capture_completed",
    "_handle_calibration_wiris_ftp_capture_failed",
    "_clear_calibration_wiris_ftp_capture_worker",
    "capture_calibration_images_wiris_ftp_oz",
    "_handle_calibration_wiris_ftp_oz_capture_completed",
    "_handle_calibration_wiris_ftp_oz_capture_failed",
    "_clear_calibration_wiris_ftp_oz_capture_worker",
    "capture_calibration_images_wiris_oz_capture_only",
    "_handle_calibration_wiris_oz_format_preset_completed",
    "_handle_calibration_wiris_oz_format_preset_failed",
    "_clear_calibration_wiris_oz_format_worker",
    "_start_calibration_wiris_oz_capture",
    "_handle_calibration_wiris_oz_capture_only_completed",
    "_handle_calibration_wiris_oz_capture_only_failed",
    "_clear_calibration_wiris_oz_capture_only_worker",
    "sync_calibration_images_wiris_oz_ftp",
    "_handle_calibration_wiris_oz_gilc_completed",
    "_handle_calibration_wiris_oz_gilc_failed",
    "_clear_calibration_wiris_oz_gilc_worker",
    "_handle_calibration_wiris_oz_sync_completed",
    "_handle_calibration_wiris_oz_sync_failed",
    "_clear_calibration_wiris_oz_sync_worker",
    "_process_next_ftp_oz_sync_pair",
    "_start_calibration_wiris_oz_delete",
    "_handle_calibration_wiris_oz_delete_completed",
    "_handle_calibration_wiris_oz_delete_failed",
    "_clear_calibration_wiris_oz_delete_worker",
    "_calib_oz_ledger_path",
    "_load_calib_oz_ledger",
    "_save_calib_oz_ledger",
    "_calibration_pair_selection_path",
    "_load_calibration_pair_selection",
    "_choose_calibration_pairs",
]


def _feedback_field(name: str) -> property:
    return property(lambda owner: getattr(owner._capture_feedback_owner(), name),
                    lambda owner, value: setattr(owner._capture_feedback_owner(), name, value))


def _capture_field(name):
    return property(lambda controller: getattr(controller._capture_runtime_owner(), name),
                    lambda controller, value: setattr(controller._capture_runtime_owner(), name, value))


def _ftp_field(name):
    return property(lambda controller: getattr(controller._ftp_owner(), name),
                    lambda controller, value: setattr(controller._ftp_owner(), name, value))


class CalibrationWorkflowController:
    _pending_oz_capture_target_infos = _ftp_field('_pending_oz_capture_target_infos')
    _pending_ftp_oz_sync_target_infos = _ftp_field('_pending_ftp_oz_sync_target_infos')
    _pending_ftp_oz_sync_rounds = _ftp_field('_pending_ftp_oz_sync_rounds')
    _pending_ftp_oz_sync_round_ids = _ftp_field('_pending_ftp_oz_sync_round_ids')
    _pending_ftp_oz_sync_frames = _ftp_field('_pending_ftp_oz_sync_frames')
    _ftp_oz_sync_saved_count = _ftp_field('_ftp_oz_sync_saved_count')

    _auto_capture_timer = _capture_field('_auto_capture_timer')
    _auto_capture_active = _capture_field('_auto_capture_active')
    _auto_capture_saved_pairs = _capture_field('_auto_capture_saved_pairs')
    _auto_capture_last_pair_key = _capture_field('_auto_capture_last_pair_key')
    _auto_capture_inflight_pair_key = _capture_field('_auto_capture_inflight_pair_key')
    _auto_capture_last_attempt_s = _capture_field('_auto_capture_last_attempt_s')
    _capture_origin = _capture_field('_capture_origin')
    _auto_capture_pose_views = _capture_field('_auto_capture_pose_views')
    _auto_capture_pending_pose_views = _capture_field('_auto_capture_pending_pose_views')
    _auto_capture_pending_pose_novelty = _capture_field('_auto_capture_pending_pose_novelty')

    _capture_feedback_root_text = _feedback_field("_capture_feedback_root_text")
    _capture_feedback_run = _feedback_field("_capture_feedback_run")
    _capture_feedback_last = _feedback_field("_capture_feedback_last")
    _capture_feedback_inventory_stamp = _feedback_field("_capture_feedback_inventory_stamp")
    _capture_feedback_inventory = _feedback_field("_capture_feedback_inventory")


    def __init__(self, panel):
        self.panel = panel
        self._capture_feedback = CalibrationCaptureFeedback()
        self._capture_runtime_owner()

    def _capture_runtime_owner(self):
        owner = getattr(self, "_capture_runtime", None)
        if owner is None:
            application = getattr(self.panel, "application", None)
            features = getattr(application, "features", {})
            feature = features.get("calibration") if isinstance(features, dict) else None
            owner = getattr(getattr(feature, "state", None), "capture_runtime", None)
            if owner is None:
                parent = self.panel if isinstance(self.panel, QtCore.QObject) else None
                owner = CalibrationCapture(parent=parent)
            owner.bind(
                read_live_inputs=self._capture_live_inputs,
                submit_capture=lambda **request: self.capture_calibration_images(**request),
                present_status=lambda text: self.panel.calibration_status_label.setText(text),
            )
            self._capture_runtime = owner
        return owner

    def _capture_live_inputs(self):
        detections = getattr(self.panel, "calibration_preview_detection_results", {})
        config_reader = getattr(self.panel, "_checkerboard_config", None)
        config = config_reader() if callable(config_reader) else None
        return LiveCaptureInputs(
            busy=self.panel.checkerboard_calibration_thread is not None,
            detections={side: dict(row) for side, row in detections.items() if isinstance(row, dict)} if isinstance(detections, dict) else {},
            allow_preview_fallback=bool(getattr(self.panel, "_allow_legacy_calibration_preview_fallback", False)),
            native_frame_getters={side: getattr(getattr(self.panel, f"{side}_pane", None), "calibration_native_frame_for_key", None) for side in ("left", "right")},
            pattern_size=None if config is None else config.pattern_size,
            pair_max_delta_s=self._checkerboard_pair_max_delta_s(),
        )

    def _ftp_owner(self):
        owner = getattr(self, "_calibration_ftp", None)
        if owner is not None:
            return owner
        application = getattr(self.panel, "application", None)
        feature = getattr(application, "features", {}).get("calibration") if application is not None else None
        owner = getattr(getattr(feature, "state", None), "ftp_transactions", None)
        if owner is None:
            def publish(value):
                for name in FTP_RESOURCE_FIELDS:
                    setattr(self.panel, name, getattr(value, name))
            parent = self.panel if isinstance(self.panel, QtCore.QObject) else None
            owner = CalibrationFtpTransactions(parent, publish_legacy=publish)
            for name in FTP_RESOURCE_FIELDS:
                setattr(owner, name, getattr(self.panel, name, None))
        owner.actions = FtpQueueActions(
            capture=lambda **request: self.panel.capture_calibration_images(**request),
            jobs=lambda: getattr(self, "_checkerboard_jobs", None),
            legacy_thread=lambda: self.panel.checkerboard_calibration_thread,
            shutting_down=lambda: bool(getattr(self.panel, "_shutdown_in_progress", False)),
            completed=self._present_ftp_queue_completed,
        )
        self._calibration_ftp = owner
        return owner

    def _present_ftp_queue_completed(self, saved):
        self.panel._log(f"[캘리브레이션] WIRIS FTP 동기화 완료: {saved}쌍 저장")
        self.panel.calibration_status_label.setText(f"WIRIS FTP 동기화 완료: {saved}쌍 저장")

    def _validation_owner(self):
        application = getattr(self.panel, "application", None)
        feature = getattr(application, "features", {}).get("calibration") if application is not None else None
        owner = getattr(getattr(feature, "state", None), "validation", None)
        if owner is not None:
            return owner
        # Legacy embedded fakes have no feature state; hydrate only the finite
        # validation result contract, never expose the panel to the owner.
        owner = getattr(self, "_legacy_validation", None)
        if owner is None:
            owner = CalibrationValidation()
            self._legacy_validation = owner
        for name in VALIDATION_FIELDS:
            if hasattr(self.panel, name):
                setattr(owner, name, getattr(self.panel, name))
        return owner

    def _publish_validation_compatibility(self, owner):
        application = getattr(self.panel, "application", None)
        features = getattr(application, "features", {})
        feature = features.get("calibration") if isinstance(features, dict) else None
        if getattr(getattr(feature, "state", None), "validation", None) is not owner:
            for name in VALIDATION_FIELDS:
                setattr(self.panel, name, getattr(owner, name))

    def _profiles_owner(self):
        owner = getattr(self, "_calibration_profiles", None)
        if owner is None:
            owner = CalibrationProfiles(lambda values: apply_geometry_values(self.panel, values))
            self._calibration_profiles = owner
        return owner

    def resource_inventory(self):
        resources = {}
        for name in ("_capture_runtime", "_calibration_ftp", "_checkerboard_jobs"):
            owner = getattr(self, name, None)
            if owner is not None:
                resources.update(owner.resource_inventory())
        return resources

    def request_shutdown(self):
        capture = getattr(self, "_capture_runtime", None)
        if capture is not None:
            capture.stop()
        ftp_done = self._ftp_owner().request_shutdown()
        jobs_done = self._calibration_job_owner().request_shutdown()
        return ftp_done and jobs_done

    def _input_owner(self):
        owner = getattr(self, "_calibration_inputs", None)
        if owner is None:
            owner = CalibrationInputs(self._report_calibration_selection_errors)
            self._calibration_inputs = owner
        return owner

    def _capture_feedback_owner(self) -> CalibrationCaptureFeedback:
        owner = getattr(self, "_capture_feedback", None)
        if owner is None:
            owner = CalibrationCaptureFeedback()
            self._capture_feedback = owner
        return owner

    def _calibration_job_owner(self) -> CalibrationJobs:
        owner = getattr(self, "_checkerboard_jobs", None)
        if owner is not None:
            return owner
        application = getattr(self.panel, "application", None)
        features = getattr(application, "features", {})
        feature = features.get("calibration") if isinstance(features, dict) else None
        owner = getattr(getattr(feature, "state", None), "calibration_jobs", None)
        if owner is None:
            def publish(value):
                for name in CALIBRATION_JOB_FIELDS:
                    setattr(self.panel, name, getattr(value, name))
            parent = self.panel if isinstance(self.panel, QtCore.QObject) else None
            owner = CalibrationJobs(parent, publish_legacy=publish)
            for name in CALIBRATION_JOB_FIELDS:
                setattr(owner, name, getattr(self.panel, name, None))
        self._checkerboard_jobs = owner
        return owner

    def _has_admitted_calibration_job(self) -> bool:
        owner = getattr(self, "_checkerboard_jobs", None)
        return bool(owner is not None and owner.has_admitted)

    def _prepare_calibration_job(self, kind, *, worker_factory, completed, failed, progress):
        try:
            return self._calibration_job_owner().prepare(
                kind, worker_factory=worker_factory, completed=completed,
                failed=failed, progress=progress, retired=self._finish_checkerboard_calibration_job,
            )
        except Exception as exc:
            failed(str(exc))
            return None

    def _start_calibration_job(self, job, prepare_ui) -> bool:
        owner = self._calibration_job_owner()
        try:
            prepare_ui()
            owner.start(job)
            return True
        except Exception as exc:
            return owner.fail_start(job, exc)

    def _apply_capture_job_result(self, result, origin, root=None):
        self._capture_origin = origin
        self._apply_owned_checkerboard_capture_completed(result, source_root=root)

    def _apply_capture_job_failure(self, error, origin):
        self._capture_origin = origin
        self._apply_owned_checkerboard_capture_failed(error)

    _empty_capture_feedback_run = staticmethod(CalibrationCaptureFeedback._empty_capture_feedback_run)

    def _ensure_capture_feedback_state(self) -> None:
        return self._capture_feedback_owner()._ensure_capture_feedback_state()

    _capture_feedback_root_id = staticmethod(CalibrationCaptureFeedback._capture_feedback_root_id)

    def _capture_feedback_prepare_root(self, root: str | Path) -> Path:
        return self._capture_feedback_owner()._capture_feedback_prepare_root(root)

    def _capture_feedback_reset_run(self, *, origin: str) -> None:
        return self._capture_feedback_owner()._capture_feedback_reset_run(origin=origin)

    def _capture_feedback_begin(self, *, root: str | Path, capture_id: str, origin: str) -> None:
        return self._capture_feedback_owner()._capture_feedback_begin(root=root, capture_id=capture_id, origin=origin)

    def _capture_feedback_complete(self, *, capture_id: str, origin: str, saved_sides: tuple[str, ...], stereo_pair_saved: bool) -> None:
        return self._capture_feedback_owner()._capture_feedback_complete(capture_id=capture_id, origin=origin, saved_sides=saved_sides, stereo_pair_saved=stereo_pair_saved)

    def _capture_feedback_failed(self, *, origin: str) -> None:
        return self._capture_feedback_owner()._capture_feedback_failed(origin=origin)

    _capture_inventory_stamp = staticmethod(CalibrationCaptureFeedback._capture_inventory_stamp)

    def _capture_feedback_inventory_for(self, root: Path) -> dict[str, object]:
        return self._capture_feedback_owner()._capture_feedback_inventory_for(root)

    def _capture_feedback_detection(self, side: str, now_s: float) -> dict[str, object]:
        detections = getattr(self.panel, "calibration_preview_detection_results", {})
        row = detections.get(side) if isinstance(detections, dict) else None
        if not isinstance(row, dict):
            preview_images = getattr(self.panel, "latest_calibration_preview_images", {})
            has_frame = isinstance(preview_images, dict) and preview_images.get(side) is not None
            return {
                "state": "DETECTING" if has_frame else "NO_FRAME",
                "found": None,
                "corner_count": None,
                "age_s": None,
                "request_id": None,
                "detail": (
                    "checkerboard detection pending"
                    if has_frame
                    else "calibration preview frame unavailable"
                ),
            }

        metadata = row.get("metadata")
        metadata = metadata if isinstance(metadata, dict) else {}
        age_s: float | None = None
        try:
            candidate_age_s = now_s - float(metadata.get("timestamp_s"))
        except (TypeError, ValueError, OverflowError):
            pass
        else:
            if math.isfinite(candidate_age_s):
                age_s = candidate_age_s
        status = str(row.get("status") or "checkerboard: not found")
        status_lower = status.lower()
        if "opencv unavailable" in status_lower or "invalid pattern" in status_lower:
            state = "UNAVAILABLE"
        elif "missing frame" in status_lower:
            state = "NO_FRAME"
        elif age_s is not None and (
            age_s < 0.0 or age_s > CALIBRATION_AUTO_DETECTION_MAX_AGE_S
        ):
            state = "STALE"
        elif bool(row.get("found")):
            state = "FOUND"
        else:
            state = "NOT_FOUND"
        corners = row.get("corners")
        corner_count = len(corners) if isinstance(corners, (list, tuple)) else None
        return {
            "state": state,
            "found": bool(row.get("found")),
            "corner_count": corner_count,
            "age_s": age_s,
            "request_id": row.get("request_id"),
            "detail": status,
        }

    def workstation_calibration_capture_snapshot(self) -> dict[str, object]:
        """Return source-backed live and persisted capture feedback."""

        self._ensure_capture_feedback_state()
        root_error = ""
        root: Path | None = None
        root_reader = getattr(self.panel, "_calibration_dirs", None)
        if callable(root_reader):
            try:
                directories = root_reader()
                root = Path(directories[0])
            except (OSError, TypeError, ValueError, IndexError) as exc:
                root_error = str(exc)
        if root is None:
            session_root = getattr(self.panel, "_calibration_session_root", None)
            if session_root is not None:
                root = Path(session_root)
        if root is not None:
            root = self._capture_feedback_prepare_root(root)
            capture_set = self._capture_feedback_inventory_for(root)
            capture_set_id = self._capture_feedback_root_id(root)
        else:
            capture_set = {
                "root": "",
                "state": "UNAVAILABLE",
                "left_image_count": None,
                "right_image_count": None,
                "stereo_candidate_count": None,
                "left_intrinsics_only_count": None,
                "right_intrinsics_only_count": None,
                "legacy_unverified_pair_count": None,
                "invalid_metadata_pair_count": None,
                "evidence_warning_pair_count": None,
                "error": root_error or "calibration capture root unavailable",
            }
            capture_set_id = ""

        minimum_widget = getattr(self.panel, "calib_min_images_spin", None)
        minimum_reader = getattr(minimum_widget, "value", None)
        try:
            minimum_images = int(minimum_reader()) if callable(minimum_reader) else int(
                getattr(getattr(self.panel, "settings", None), "calibration_min_images", 20)
            )
        except (TypeError, ValueError, OverflowError):
            minimum_images = 20
        status_widget = getattr(self.panel, "calibration_status_label", None)
        status_reader = getattr(status_widget, "text", None)
        last = dict(self._capture_feedback_last)
        last["message"] = (
            str(status_reader()).strip()
            if callable(status_reader)
            else ""
        )
        now_s = time.monotonic()
        return {
            "schema": "workstation-calibration-capture-status-v1",
            "capture_root": "" if root is None else str(root),
            "capture_set_id": capture_set_id,
            "auto_capture_active": bool(
                getattr(self, "_auto_capture_active", False)
            ),
            "detection": {
                side: self._capture_feedback_detection(side, now_s)
                for side in ("left", "right")
            },
            "run": dict(self._capture_feedback_run),
            "capture_set": capture_set,
            "minimum_images_per_side": max(3, minimum_images),
            "last": last,
            "source": (
                "checkerboard preview result + calibration controller lifecycle + "
                "persisted capture files/metadata"
            ),
            "frame_contract": (
                "checkerboard corners are OpenCV image pixels (u right, v down); "
                "no NED/FRD or calibration-matrix conversion"
            ),
        }

    def _publish_calibration_capture_event(
        self,
        payload: object,
        *,
        operation: str = "",
        capture_id: str = "",
    ) -> None:
        """Publish a left/right capture lifecycle event to the Workstation UI.

        The camera workers remain UI-independent.  This small adapter adds the
        calibration operation identity and forwards the raw command/response
        evidence through the owner's Qt signal when the owner exposes it.
        """

        event = dict(payload) if isinstance(payload, dict) else {"response": str(payload)}
        if operation:
            event.setdefault("operation", operation)
        if capture_id:
            event.setdefault("capture_id", capture_id)
        event.setdefault("timestamp_unix_ns", time.time_ns())
        signal = getattr(self.panel, "calibration_capture_event", None)
        emit = getattr(signal, "emit", None)
        if callable(emit):
            emit(event)

    def _connect_calibration_worker_progress(
        self,
        worker: object,
        *,
        operation: str,
        capture_id: str = "",
    ) -> None:
        signal = getattr(worker, "progress", None)
        connect = getattr(signal, "connect", None)
        if not callable(connect):
            return

        def relay(payload: object) -> None:
            self._publish_calibration_capture_event(
                payload,
                operation=operation,
                capture_id=capture_id,
            )

        connect(relay)

    def _publish_calibration_camera_summary(
        self,
        *,
        operation: str,
        capture_id: str,
        cameras: object,
    ) -> None:
        records = cameras if isinstance(cameras, dict) else {}
        for side in ("left", "right"):
            item = records.get(side)
            if not isinstance(item, dict):
                self._publish_calibration_capture_event(
                    {
                        "side": side,
                        "phase": "completed",
                        "state": "missing",
                        "response": "camera result missing",
                    },
                    operation=operation,
                    capture_id=capture_id,
                )
                continue
            response_parts = []
            for key in (
                "hr_enable_response",
                "oz_enable_response",
                "capture_response",
                "capture_status_response",
            ):
                value = str(item.get(key) or "").strip()
                if value:
                    response_parts.append(f"{key}={value}")
            for key in ("error", "ftp_error"):
                value = str(item.get(key) or "").strip()
                if value:
                    response_parts.append(f"{key}={value}")
            self._publish_calibration_capture_event(
                {
                    "side": side,
                    "phase": "completed",
                    "state": str(item.get("state") or "failed"),
                    "response": "; ".join(response_parts) or "capture completed",
                    "path": str(item.get("ftp_local_path") or ""),
                    "remote_path": str(item.get("ftp_remote_path") or ""),
                    "stored_on_camera": bool(item.get("stored_on_camera")),
                    "ftp_downloaded": bool(item.get("ftp_downloaded")),
                },
                operation=operation,
                capture_id=capture_id,
            )

    def _publish_calibration_ftp_pair_ready(
        self,
        *,
        operation: str,
        capture_id: str,
        cameras: Mapping[str, object],
        frames: Mapping[str, QtGui.QImage | None],
    ) -> None:
        """Publish one atomic UI event after both downloaded images decode."""

        paths = {
            side: str((cameras.get(side) or {}).get("ftp_local_path") or "")
            if isinstance(cameras.get(side), Mapping)
            else ""
            for side in ("left", "right")
        }
        dimensions = {
            side: [int(frames[side].width()), int(frames[side].height())]
            for side in ("left", "right")
            if isinstance(frames.get(side), QtGui.QImage) and not frames[side].isNull()
        }
        self._publish_calibration_capture_event(
            {
                "phase": "ftp_pair_ready",
                "state": "ready",
                "paths": paths,
                "dimensions": dimensions,
            },
            operation=operation,
            capture_id=capture_id,
        )

    def _app_relative_path_text(self, path: str | Path) -> str:
        return portable_workstation_data_path_text(
            path,
            data_root=self.panel.app_root,
            field_name="asset_path",
        )

    def _checkerboard_validation_mode(self) -> str:
        if not hasattr(self.panel, "checkerboard_validation_mode_combo"):
            return "pixhawk"
        mode = str(self.panel.checkerboard_validation_mode_combo.currentData() or "pixhawk")
        return mode if mode in {"pixhawk", "checkerboard"} else "pixhawk"

    def _checkerboard_validation_root(self) -> Path:
        text = self.panel.checkerboard_validation_root_edit.text().strip() if hasattr(self.panel, "checkerboard_validation_root_edit") else ""
        if text:
            return resolve_settings_path(text, base_dir=self.panel.app_root)
        root, _left_dir, _right_dir = self.panel._calibration_dirs()
        return root

    _zoom_index = staticmethod(CalibrationInputs._zoom_index)

    def _capture_zoom_snapshot(self) -> dict[str, object]:
        """Capture the latest camera readback alongside every calibration image."""

        actual = getattr(self.panel, "actual_visible_zoom", {}) or {}
        values: dict[str, int | None] = {}
        readbacks: dict[str, dict[str, object]] = {}
        for side in ("left", "right"):
            row = actual.get(side, {}) if isinstance(actual, dict) else {}
            readback = dict(row) if isinstance(row, dict) else {}
            readbacks[side] = readback
            values[side] = self._zoom_index(readback.get("index"))
        shared_level = (
            values["left"]
            if values["left"] is not None and values["left"] == values["right"]
            else None
        )
        requested = getattr(self.panel, "zoom_level_spin", None)
        try:
            requested_level = int(requested.value()) if requested is not None else None
        except (AttributeError, TypeError, ValueError):
            requested_level = None
        return {
            "capture_zoom_level": shared_level,
            "capture_left_zoom_level": values["left"],
            "capture_right_zoom_level": values["right"],
            "capture_zoom_source": "GZVV" if shared_level is not None else "unresolved",
            "capture_zoom_readback": readbacks,
            "requested_zoom_level": requested_level,
        }

    def _capture_zoom_level_for_paths(self, root, paths):
        return self._input_owner()._capture_zoom_level_for_paths(root, paths)

    def _calibration_image_paths_for_root(self, root: Path, side: str):
        return self._input_owner()._calibration_image_paths_for_root(root, side)

    def _calibration_stereo_pair_paths_for_root(self, root: Path):
        return self._input_owner()._calibration_stereo_pair_paths_for_root(root)

    _calibration_pair_stereo_eligible = staticmethod(CalibrationInputs._calibration_pair_stereo_eligible)

    def _checkerboard_validation_scan_counts(self) -> tuple[Path, list[Path], list[Path], list[tuple[str, Path, Path]]]:
        root = self.panel._checkerboard_validation_root()
        left_paths = self.panel._calibration_image_paths_for_root(root, "left")
        right_paths = self.panel._calibration_image_paths_for_root(root, "right")
        pairs = self.panel._calibration_stereo_pair_paths_for_root(root)
        return root, left_paths, right_paths, pairs

    def scan_checkerboard_validation_files(self) -> None:
        root, left_paths, right_paths, pairs = self.panel._checkerboard_validation_scan_counts()
        mode = self.panel._checkerboard_validation_mode()
        requirement = (
            "필수: left/right 이미지"
            if mode == "pixhawk"
            else "필수: matching stereo pair"
        )
        level = "good" if (pairs if mode == "checkerboard" else left_paths and right_paths) else "warn"
        text = (
            f"root={root} | left={len(left_paths)} right={len(right_paths)} "
            f"stereo_pair={len(pairs)} | {requirement}"
        )
        self.panel._set_health_label(self.panel.checkerboard_validation_scan_label, text, level)

    def choose_checkerboard_validation_root(self) -> None:
        directory = QtWidgets.QFileDialog.getExistingDirectory(
            self.panel,
            "체커보드 사진셋 root 선택",
            str(self.panel._checkerboard_validation_root()),
        )
        if not directory:
            return
        self.panel.checkerboard_validation_root_edit.setText(directory)
        self.panel.scan_checkerboard_validation_files()

    def choose_checkerboard_validation_json(self) -> None:
        path, _filter = QtWidgets.QFileDialog.getOpenFileName(
            self.panel,
            "기존 intrinsics JSON 선택",
            str(self.panel._intrinsics_storage_path()),
            "Intrinsics JSON (*.json);;All Files (*)",
        )
        if path:
            self.panel.checkerboard_validation_json_edit.setText(path)

    def _checkerboard_validation_set_result(self, result, *, mode, summary, text, zoom_level=None, selection_snapshot=None):
        owner = self._validation_owner()
        def publish_diagnostics(rows):
            self._publish_validation_compatibility(owner)
            signal = getattr(self.panel, "calibration_diagnostics_changed", None)
            emit = getattr(signal, "emit", None)
            if callable(emit):
                emit(rows)
        result = owner.accept_full_result(
            result, mode=mode, summary=summary, root=self.panel._calibration_dirs()[0],
            zoom_level=zoom_level, selection_snapshot=selection_snapshot,
            publish_diagnostics=publish_diagnostics,
        )
        self._publish_validation_compatibility(owner)
        self.panel.calibration_result_text.setPlainText(text)
        extrinsics_state = "with checkerboard stereo R/T" if result.extrinsics is not None else "intrinsics only"
        quality_state = str(owner.calibration_quality.get("state") or "UNVERIFIED").upper()
        self.panel._set_health_label(
            self.panel.checkerboard_validation_status_label,
            f"계산 결과 있음: {mode} | {extrinsics_state} | quality={quality_state} | 저장 전",
            {"PASS": "good", "FAIL": "bad"}.get(quality_state, "warn"),
        )
        self.panel._refresh_camera_parameters_tab()
        self.panel._refresh_zoom_intrinsics_tab()

    def _checkerboard_result_baseline_m(self, result: StereoIntrinsicsResult) -> float | None:
        return CalibrationValidation()._checkerboard_result_baseline_m(result)

    def _rotation_delta_deg_from_matrices(
        self,
        reference: list[list[float]] | None,
        current: list[list[float]] | None,
    ) -> float | None:
        return CalibrationValidation()._rotation_delta_deg_from_matrices(reference, current)

    def _format_checkerboard_validation_result_summary(
        self,
        result: StereoIntrinsicsResult,
        *,
        mode: str,
        title: str,
        root: Path | None = None,
        pair_count: int | None = None,
    ) -> str:
        return CalibrationValidation()._format_checkerboard_validation_result_summary(result, mode=mode, title=title, root=root, pair_count=pair_count)

    def load_checkerboard_validation_json(self) -> None:
        path_text = self.panel.checkerboard_validation_json_edit.text().strip()
        if not path_text:
            self.panel._set_health_label(self.panel.checkerboard_validation_status_label, "JSON 경로가 비어 있습니다", "bad")
            return
        path = resolve_settings_path(path_text, base_dir=self.panel.app_root)
        try:
            result = self._profiles_owner().load_intrinsics(path)
        except Exception as exc:
            self.panel._set_health_label(self.panel.checkerboard_validation_status_label, f"JSON 불러오기 실패: {exc}", "bad")
            return
        source_metadata = self._profiles_owner().read_source_metadata(path)
        try:
            archived_path = self._archive_loaded_intrinsics(path)
        except Exception as exc:
            self.panel._set_health_label(
                self.panel.checkerboard_validation_status_label,
                f"JSON Session 보관 실패: {exc}",
                "bad",
            )
            return
        source_zoom_level = self._zoom_index(
            source_metadata.get(
                "capture_zoom_level",
                source_metadata.get("zoom_level"),
            )
        )
        mode = "checkerboard" if result.extrinsics is not None else "pixhawk"
        if hasattr(self.panel, "checkerboard_validation_mode_combo"):
            self.panel._set_combo_data(self.panel.checkerboard_validation_mode_combo, mode)
        text = self.panel._format_checkerboard_validation_result_summary(
            result,
            mode=mode,
            title=f"Loaded intrinsics JSON: {path}",
        )
        self.panel._checkerboard_validation_set_result(
            result,
            mode=mode,
            summary={
                "source": "json",
                "path": str(path),
                "capture_zoom_level": source_zoom_level,
            },
            text=text,
            zoom_level=source_zoom_level,
        )
        # Loading creates a candidate.  Preserve its exact bytes without
        # changing the identity of any calibration already active at runtime.
        self.panel._candidate_intrinsics_profile_path = str(archived_path or path)
        self.panel.checkerboard_validation_result_saved = True
        self.panel._set_health_label(
            self.panel.checkerboard_validation_status_label,
            f"기존 JSON 불러옴: {mode} | {path}"
            + (f" | Session copy={archived_path}" if archived_path is not None else ""),
            "good",
        )

    def _checkerboard_random_subset_count(self, total_pairs: int) -> int:
        configured = self.panel.checkerboard_validation_random_pair_count_spin.value()
        if configured > 0:
            return min(total_pairs, max(3, configured))
        min_images = self.panel.calib_min_images_spin.value()
        return min(total_pairs, max(3, min_images, int(math.ceil(total_pairs * 0.7))))

    def _calibration_calculation_pair_limit(self) -> int:
        widget = getattr(self.panel, "calib_calculation_pairs_spin", None)
        try:
            return max(0, int(widget.value()))
        except (AttributeError, TypeError, ValueError):
            return 0

    _evenly_sampled_calibration_items = staticmethod(CalibrationInputs._evenly_sampled_calibration_items)

    def _select_calibration_pairs(self, pairs, *, pair_limit=None):
        limit = self._calibration_calculation_pair_limit() if pair_limit is None else pair_limit
        return self._input_owner()._select_calibration_pairs(pairs, pair_limit=limit)

    _select_calibration_image_paths = staticmethod(CalibrationInputs._select_calibration_image_paths)

    def _checkerboard_intrinsic_delta_px(self, result: StereoIntrinsicsResult, reference: StereoIntrinsicsResult) -> tuple[float, float]:
        return CalibrationValidation()._checkerboard_intrinsic_delta_px(result, reference)

    def _checkerboard_validation_stddev(self, values: list[float]) -> float:
        return CalibrationValidation()._checkerboard_validation_stddev(values)

    _calibration_quality_mapping = staticmethod(CalibrationQuality._calibration_quality_mapping)

    def _calibration_quality_state(self, result):
        fallbacks = []
        if not any(getattr(result, name, None) is not None for name in ("calibration_quality", "left_quality", "right_quality")):
            for name in ("checkerboard_calibration_quality", "calibration_quality_snapshot", "checkerboard_quality_snapshot"):
                value = getattr(self.panel, name, None)
                if callable(value):
                    try:
                        value = value()
                    except Exception:
                        value = None
                if value is not None:
                    fallbacks.append(value)
        return CalibrationQuality()._calibration_quality_state(result, fallbacks)

    def run_checkerboard_random_validation(self) -> None:
        owner = self._calibration_job_owner()
        if owner.closing or owner.busy:
            self.panel._set_health_label(
                self.panel.checkerboard_validation_status_label,
                "A calibration calculation is already running",
                "warn",
            )
            return
        available, error = opencv_available()
        if not available:
            self.panel._set_health_label(self.panel.checkerboard_validation_status_label, f"OpenCV unavailable: {error}", "bad")
            return
        reference = self.panel.checkerboard_validation_result
        if reference is None:
            self.panel._set_health_label(
                self.panel.checkerboard_validation_status_label,
                "랜덤 검증 전에 전체 사진으로 계산을 먼저 실행하세요.",
                "warn",
            )
            return
        mode = self.panel._checkerboard_validation_mode()
        _root, left_paths, right_paths, pairs = self._random_validation_inputs(
            reference,
            mode=mode,
        )
        candidate_count = len(pairs) if mode == "checkerboard" else min(
            len(left_paths),
            len(right_paths),
        )
        if candidate_count < 3:
            input_label = "stereo pair" if mode == "checkerboard" else "left/right image"
            self.panel._set_health_label(
                self.panel.checkerboard_validation_status_label,
                f"랜덤 검증 실패: {input_label}가 부족합니다 "
                f"(candidate={candidate_count}, need>=3)",
                "bad",
            )
            return
        subset_count = self.panel._checkerboard_random_subset_count(candidate_count)
        runs = self.panel.checkerboard_validation_random_runs_spin.value()
        seed = self.panel.checkerboard_validation_seed_spin.value()
        worker_factory = lambda: CheckerboardRandomValidationWorker(
            mode=mode,
            left_intrinsic_paths=left_paths,
            right_intrinsic_paths=right_paths,
            stereo_pairs=pairs,
            config=self.panel._checkerboard_config(),
            runs=runs,
            subset_count=subset_count,
            seed=seed,
        )
        job = self._prepare_calibration_job(
            "random_validation", worker_factory=worker_factory,
            completed=lambda results,
            reference=reference,
            mode=mode,
            pair_count=len(pairs),
            runs=runs,
            subset_count=subset_count,
            seed=seed: self._apply_owned_random_validation_completed(
                results,
                reference=reference,
                mode=mode,
                pair_count=pair_count,
                runs=runs,
                subset_count=subset_count,
                seed=seed,
            ),
            failed=self._apply_owned_random_validation_failed,
            progress=self._apply_owned_calibration_progress,
        )
        if job is None:
            return

        def prepare_ui():
            self.panel.capture_calibration_button.setEnabled(False)
            self.panel.run_calibration_button.setEnabled(False)
            self.panel.checkerboard_validation_random_button.setEnabled(False)
            self.panel._set_health_label(
                self.panel.checkerboard_validation_status_label,
                f"Random validation running: runs={runs}, subset_pairs={subset_count}, mode={mode}",
                "warn",
            )
            self.panel.calibration_progress = {
                "phase": "validation",
                "processed": 0,
                "total": subset_count,
                "percent": 0.0,
                "message": f"랜덤 검증 준비: {runs}회 × {subset_count} pair",
                "run": 0,
                "runs": runs,
            }
            if hasattr(self.panel, "calibration_progress_bar"):
                self.panel.calibration_progress_bar.setValue(0)
                self.panel.calibration_progress_bar.setFormat("랜덤 검증 준비 · %p%")
        self._start_calibration_job(job, prepare_ui)

    def _handle_random_validation_failed(self, error: str) -> None:
        if not self._has_admitted_calibration_job():
            return self._apply_owned_random_validation_failed(error)

    def _apply_owned_random_validation_failed(self, error: str) -> None:
        self.panel.calibration_progress = {
            "phase": "validation_failed",
            "processed": 0,
            "total": 0,
            "percent": 0.0,
            "message": str(error),
        }
        if hasattr(self.panel, "calibration_progress_bar"):
            self.panel.calibration_progress_bar.setValue(0)
            self.panel.calibration_progress_bar.setFormat("랜덤 검증 실패 · %p%")
        self.panel._set_health_label(
            self.panel.checkerboard_validation_status_label,
            f"Random validation failed: {error}",
            "bad",
        )
        self.panel._log(f"[캘리브레이션] 랜덤 검증 실패: {error}")

    def _handle_random_validation_completed(self, results: object, *, reference: StereoIntrinsicsResult, mode: str, pair_count: int, runs: int, subset_count: int, seed: int) -> None:
        if not self._has_admitted_calibration_job():
            return self._apply_owned_random_validation_completed(results, reference=reference, mode=mode, pair_count=pair_count, runs=runs, subset_count=subset_count, seed=seed)

    def _apply_owned_random_validation_completed(self, results, *, reference, mode, pair_count, runs, subset_count, seed):
        owner = self._validation_owner()
        rows_text, quality_state = owner.complete_random_validation(
            results, reference=reference, mode=mode, pair_count=pair_count,
            runs=runs, subset_count=subset_count, seed=seed,
            quality=self._calibration_quality_state(reference),
        )
        self._publish_validation_compatibility(owner)
        full_text = self.panel.calibration_result_text.toPlainText().rstrip()
        self.panel.calibration_result_text.setPlainText(full_text + "\n\nRandom stereo-pair validation:\n" + rows_text)
        self.panel._set_health_label(
            self.panel.checkerboard_validation_status_label,
            f"랜덤 검증 완료: runs={runs}, subset_pairs={subset_count}, mode={mode}, quality={quality_state}",
            {"PASS": "good", "FAIL": "bad"}.get(quality_state, "warn"),
        )
        if hasattr(self.panel, "calibration_progress_bar"):
            self.panel.calibration_progress_bar.setValue(100)
            self.panel.calibration_progress_bar.setFormat("랜덤 검증 완료 · %p%")

    def _checkerboard_validation_result_row(
        self,
        name: str,
        result: StereoIntrinsicsResult,
        reference: StereoIntrinsicsResult,
        pairs: int,
    ) -> str:
        return CalibrationValidation()._checkerboard_validation_result_row(name, result, reference, pairs)

    def _checkerboard_validation_result_data(
        self,
        name: str,
        result: StereoIntrinsicsResult,
        reference: StereoIntrinsicsResult,
        pairs: int,
    ) -> dict[str, object]:
        return CalibrationValidation()._checkerboard_validation_result_data(name, result, reference, pairs)

    def _calibration_quality_allows_action(
        self,
        action: str,
        *,
        warning_confirmed: bool = False,
    ) -> bool:
        result = self.panel.checkerboard_validation_result
        if not isinstance(result, StereoIntrinsicsResult):
            return False
        state, reasons = self._calibration_quality_state(result)
        decision = CalibrationQuality.decide(action, state, reasons, warning_confirmed=warning_confirmed)
        if not decision.allowed:
            self.panel._set_health_label(self.panel.checkerboard_validation_status_label, decision.message, decision.severity)
            return False
        if decision.acknowledge_warning:
            summary = dict(getattr(self.panel, "checkerboard_validation_summary", {}) or {})
            summary["quality_override"] = {
                "action": str(action),
                "confirmed": True,
                "confirmed_utc_ns": time.time_ns(),
                "reason_codes": list(reasons),
            }
            self.panel.checkerboard_validation_summary = summary
        return True

    def _publish_immutable_calibration_file(self, source, *, family, prefix):
        root = self.panel._calibration_dirs()[0]
        return self._profiles_owner()._publish_immutable_calibration_file(source, root=root, family=family, prefix=prefix)

    def _publish_immutable_intrinsics_profile(self, result, *, metadata):
        root = self.panel._calibration_dirs()[0]
        return self._profiles_owner()._publish_immutable_intrinsics_profile(result, root=root, metadata=metadata)

    def _publish_immutable_extrinsics_profile(self, profile):
        root = self.panel._calibration_dirs()[0]
        return self._profiles_owner()._publish_immutable_extrinsics_profile(profile, root=root)

    def save_checkerboard_validation_result(
        self,
        *,
        quality_warning_confirmed: bool = False,
    ) -> None:
        self._ensure_calibration_session_root("calculation")
        result = self.panel.checkerboard_validation_result
        if result is None:
            self.panel._set_health_label(self.panel.checkerboard_validation_status_label, "저장할 계산 결과가 없습니다", "warn")
            return
        root, _left_dir, _right_dir = self.panel._calibration_dirs()
        zoom_level = getattr(self.panel, "last_calibration_zoom_level", None)
        quality_state, quality_reasons = self._calibration_quality_state(result)
        approved_output = (
            self.panel._intrinsics_storage_path(zoom_level)
            if CalibrationQuality.approved_for_profile_save(quality_state, capture_zoom_known=zoom_level is not None)
            else None
        )
        metadata = {
            "note": "Generated by StereoWIRIS calibration calculation",
            "timestamp_s": time.time(),
            "zoom_level": zoom_level,
            "capture_zoom_level": zoom_level,
            "zoom_target": str(self.panel.zoom_target_combo.currentData() or "both"),
            "calculation_mode": self.panel.checkerboard_validation_result_mode,
            "validation": self.panel.checkerboard_validation_summary,
        }
        saved = self._profiles_owner().save_intrinsics_attempt(
            result, root=root, zoom_level=zoom_level, quality_state=quality_state,
            quality_reasons=quality_reasons, metadata=metadata, approved_output=approved_output,
        )
        output = saved.output
        self.panel.checkerboard_validation_result_saved = True
        self.panel._last_calibration_attempt_path = str(output)
        if saved.approved:
            immutable_output = self._profiles_owner().publish_saved_intrinsics(
                saved, root=root, latest_output=self.panel._intrinsics_storage_base_path(),
            )
            self.panel.last_calibration_zoom_level = zoom_level
            self.panel._candidate_intrinsics_profile_path = str(immutable_output)
            self.panel._add_zoom_intrinsics_comparison_path(immutable_output)
            self.panel._save_current_settings()
            message = (
                f"승인 profile 저장 완료: zoom {int(zoom_level):02d} | "
                f"immutable={immutable_output} | alias={output}"
            )
            level = "good"
        else:
            reason = ", ".join(quality_reasons) or (
                "capture zoom readback missing" if zoom_level is None else quality_state
            )
            message = (
                f"적용 불가 calibration attempt 저장 완료: {quality_state} | "
                f"{reason} | {output}"
            )
            level = "bad" if quality_state == "FAIL" else "warn"
        self.panel._set_health_label(
            self.panel.checkerboard_validation_status_label,
            message,
            level,
        )
        self.panel._refresh_operation_header()
        self.panel._refresh_zoom_intrinsics_tab()

    def apply_checkerboard_validation_result(
        self,
        *,
        quality_warning_confirmed: bool = False,
    ) -> None:
        result = self.panel.checkerboard_validation_result
        if result is None:
            self.panel._set_health_label(self.panel.checkerboard_validation_status_label, "적용할 계산 결과가 없습니다", "warn")
            return
        if not self._calibration_quality_allows_action(
            "적용",
            warning_confirmed=quality_warning_confirmed,
        ):
            return
        mode = self.panel.checkerboard_validation_result_mode
        if mode == "checkerboard" and result.extrinsics is None:
            self.panel._set_health_label(
                self.panel.checkerboard_validation_status_label,
                "체커보드 Stereo 모드는 stereo R/T가 있는 결과만 적용할 수 있습니다",
                "bad",
            )
            return
        zoom_level = getattr(self.panel, "last_calibration_zoom_level", None)
        if zoom_level is None:
            self.panel._set_health_label(
                self.panel.checkerboard_validation_status_label,
                "적용 실패: 계산 결과에 촬영 당시 줌 readback이 없습니다",
                "bad",
            )
            return
        self.panel.last_calibration_result = result
        self.panel._set_combo_data(self.panel.distance_mode_combo, mode)
        explicit_profile = (
            self._full_checkerboard_result_profile(result, zoom_level=zoom_level)
            if mode == "checkerboard"
            else None
        )
        applied = (
            self.panel.apply_intrinsics_to_distance(
                quality_warning_confirmed=True,
                extrinsics_profile_override=explicit_profile,
            )
            if quality_warning_confirmed
            else self.panel.apply_intrinsics_to_distance(
                extrinsics_profile_override=explicit_profile,
            )
        )
        if applied is False:
            return
        self.panel._save_current_settings()
        saved_text = "저장됨" if self.panel.checkerboard_validation_result_saved else "저장 전"
        self.panel._set_health_label(
            self.panel.checkerboard_validation_status_label,
            f"계산에 적용 완료: mode={mode} | {saved_text}",
            "good",
        )

    def workstation_calibration_quality_snapshot(self) -> dict[str, object]:
        current_result = getattr(self.panel, "last_calibration_result", None)
        source_result = current_result
        if source_result is None:
            validation_result = getattr(self.panel, "checkerboard_validation_result", None)
            source_result = validation_result
        snapshot = getattr(source_result, "calibration_quality", None)
        if source_result is None and not isinstance(snapshot, dict):
            snapshot = getattr(self.panel, "calibration_quality", None)
        if isinstance(snapshot, dict):
            return json.loads(json.dumps(snapshot, ensure_ascii=False, allow_nan=False))
        return {
            "schema": "workstation-camera-calibration-quality-v1",
            "state": "UNVERIFIED",
            "source": "legacy/no quality evidence",
            "reason_codes": ["quality_snapshot_unavailable"],
            "reasons": ["No source-backed calibration quality snapshot is available."],
        }

    def workstation_checkerboard_validation_snapshot(self) -> dict:
        """Read-only projection of the validation workflow's current state.

        The action methods above (``load_checkerboard_validation_json`` etc.)
        read/write panel widget attributes directly and return ``None``; this
        gives a payload-in/result-out caller (the navigation UI, not the
        legacy panel's own hidden widgets) something to read after any
        action, or to poll while a random-validation run is in flight.
        """

        panel = self.panel
        return {
            "root": panel.checkerboard_validation_root_edit.text(),
            "mode": str(panel.checkerboard_validation_mode_combo.currentData() or "pixhawk"),
            "json_path": panel.checkerboard_validation_json_edit.text(),
            "scan_status": panel.checkerboard_validation_scan_label.text(),
            "status": panel.checkerboard_validation_status_label.text(),
            "random_pair_count": panel.checkerboard_validation_random_pair_count_spin.value(),
            "runs": panel.checkerboard_validation_random_runs_spin.value(),
            "seed": panel.checkerboard_validation_seed_spin.value(),
            "running": panel.checkerboard_calibration_thread is not None,
            "result_available": panel.checkerboard_validation_result is not None,
            "result_mode": str(getattr(panel, "checkerboard_validation_result_mode", "") or ""),
            "result_saved": bool(getattr(panel, "checkerboard_validation_result_saved", False)),
            "summary": dict(getattr(panel, "checkerboard_validation_summary", {}) or {}),
            "run_results": list(getattr(panel, "checkerboard_validation_run_results", []) or []),
            "calibration_quality": self.workstation_calibration_quality_snapshot(),
        }

    def configure_workstation_checkerboard_validation(self, payload: dict) -> dict:
        """Apply operator-supplied settings, keeping the panel widgets as the
        single owner of this state (unchanged since before the navigation UI
        existed) rather than duplicating it. Only the keys present in
        ``payload`` are touched.
        """

        panel = self.panel
        if "root" in payload:
            panel.checkerboard_validation_root_edit.setText(str(payload["root"]))
        if "mode" in payload:
            panel._set_combo_data(panel.checkerboard_validation_mode_combo, str(payload["mode"]))
        if "json_path" in payload:
            panel.checkerboard_validation_json_edit.setText(str(payload["json_path"]))
        if "random_pair_count" in payload:
            panel.checkerboard_validation_random_pair_count_spin.setValue(int(payload["random_pair_count"]))
        if "runs" in payload:
            panel.checkerboard_validation_random_runs_spin.setValue(int(payload["runs"]))
        if "seed" in payload:
            panel.checkerboard_validation_seed_spin.setValue(int(payload["seed"]))
        if "root" in payload or "mode" in payload:
            panel.scan_checkerboard_validation_files()
        return self.panel.workstation_checkerboard_validation_snapshot()

    def load_workstation_checkerboard_validation_json(self, payload: dict) -> dict:
        self.panel.configure_workstation_checkerboard_validation(payload)
        self.panel.load_checkerboard_validation_json()
        return self.panel.workstation_checkerboard_validation_snapshot()

    def run_workstation_checkerboard_validation(self, payload: dict) -> dict:
        self.panel.configure_workstation_checkerboard_validation(payload)
        self.panel.run_checkerboard_random_validation()
        return self.panel.workstation_checkerboard_validation_snapshot()

    def save_workstation_checkerboard_validation_result(
        self,
        *,
        quality_warning_confirmed: bool = False,
    ) -> dict:
        self.panel.save_checkerboard_validation_result(
            quality_warning_confirmed=quality_warning_confirmed
        )
        return self.panel.workstation_checkerboard_validation_snapshot()

    def apply_workstation_checkerboard_validation_result(
        self,
        *,
        quality_warning_confirmed: bool = False,
    ) -> dict:
        self.panel.apply_checkerboard_validation_result(
            quality_warning_confirmed=quality_warning_confirmed
        )
        return self.panel.workstation_checkerboard_validation_snapshot()

    def apply_intrinsics_to_distance(
        self,
        *,
        quality_warning_confirmed: bool = False,
        extrinsics_profile_override: object = _USE_SELECTED_EXTRINSICS,
    ) -> bool:
        panel = self.panel
        if panel.last_calibration_result is None:
            panel.calibration_status_label.setText("Run or load calibration before applying intrinsics.")
            return False
        quality_state, quality_reasons = self._calibration_quality_state(
            panel.last_calibration_result
        )
        if quality_state in {"FAIL", "WARNING"} and not (
            quality_state == "WARNING" and quality_warning_confirmed
        ):
            qualifier = "FAIL" if quality_state == "FAIL" else "WARNING requires confirmation"
            panel.calibration_status_label.setText(
                f"Cannot apply intrinsics: calibration quality {qualifier} ("
                + ", ".join(quality_reasons)
                + ")"
            )
            panel._set_health_label(
                panel.checkerboard_validation_status_label,
                f"적용 차단: calibration quality {qualifier}",
                "bad" if quality_state == "FAIL" else "warn",
            )
            return False
        if quality_state == "WARNING":
            summary = dict(getattr(panel, "checkerboard_validation_summary", {}) or {})
            summary["quality_override"] = {
                "action": "apply_intrinsics_to_distance",
                "confirmed": True,
                "confirmed_utc_ns": time.time_ns(),
                "reason_codes": list(quality_reasons),
            }
            panel.checkerboard_validation_summary = summary
        for side, intrinsics in (
            ("left", panel.last_calibration_result.left),
            ("right", panel.last_calibration_result.right),
        ):
            pane = getattr(panel, f"{side}_pane", None)
            native_reader = getattr(pane, "latest_native_calibration_frame", None)
            native = native_reader(maximum_age_s=5.0) if callable(native_reader) else None
            if native is None:
                continue
            _frame_id, image, _metadata = native
            if (image.width(), image.height()) != (intrinsics.width, intrinsics.height):
                panel.calibration_status_label.setText(
                    f"Cannot apply {side} intrinsics: profile grid "
                    f"{intrinsics.width}x{intrinsics.height} differs from live native grid "
                    f"{image.width()}x{image.height()}"
                )
                panel._set_health_label(
                    panel.checkerboard_validation_status_label,
                    f"적용 차단: {side} native pixel grid mismatch",
                    "bad",
                )
                return False
        applied = self._profiles_owner().apply_bundle(
            root=panel._calibration_dirs()[0], result=panel.last_calibration_result,
            zoom_level=panel.last_calibration_zoom_level,
            candidate_path=getattr(panel, "_candidate_intrinsics_profile_path", ""),
            active_intrinsics=getattr(panel, "active_intrinsics", None),
            active_path=getattr(panel, "_active_intrinsics_profile_path", ""),
            selected_profile=panel.selected_extrinsics_profile,
            selected_path=getattr(panel, "_selected_extrinsics_profile_path", ""),
            selected_hash=getattr(panel, "_selected_extrinsics_profile_hash", ""),
            use_selected_extrinsics=extrinsics_profile_override is _USE_SELECTED_EXTRINSICS,
            extrinsics_profile_override=extrinsics_profile_override,
            quality_state=quality_state, quality_reasons=quality_reasons,
            quality_warning_confirmed=quality_warning_confirmed,
        )
        panel._candidate_intrinsics_profile_path = applied.candidate_path
        selected_profile = applied.selected_profile
        if applied.checkerboard_candidate is not None:
            panel.checkerboard_extrinsics_candidate_profile = applied.checkerboard_candidate
        if applied.optical_candidate is not None:
            panel.optical_pose_extrinsics_candidate_profile = applied.optical_candidate
        if selected_profile is not None:
            panel._set_combo_data(panel.distance_mode_combo, 'checkerboard')
        left_fx = panel.active_intrinsics.left.camera_matrix[0][0]
        right_fx = panel.active_intrinsics.right.camera_matrix[0][0]
        panel.focal_spin.setValue((left_fx + right_fx) / 2.0)
        panel.manual_focal_checkbox.setChecked(False)
        selection_label = {
            "manual_cross_zoom": "cross-zoom/manual",
            "manual_same_zoom": "same-zoom/manual",
            "manual_independent_zoom": "independent/manual; source zoom unknown",
        }.get(panel._active_calibration_selection_mode, "")
        detail = f"K,D zoom={panel.last_calibration_zoom_level}; " + (
            f"R,t zoom={selected_profile.source_zoom_level}; "
            f"{selection_label}, validation not gated"
            if selected_profile is not None
            else "R,t not selected"
        )
        panel.calibration_status_label.setText(f"Applied calibration bundle: {detail}")
        panel._refresh_camera_parameters_tab()
        panel._refresh_distance()
        panel._refresh_optical_alignment_view()
        panel._refresh_operation_header()
        return True

    def _full_checkerboard_result_profile(
        self,
        result: StereoIntrinsicsResult,
        *,
        zoom_level: int,
    ) -> StereoExtrinsicsProfile:
        """Bind one full-calibration Apply to that result's own R,t bytes."""

        if result.extrinsics is None:
            raise ValueError("full checkerboard result has no stereo R,t")
        profile = StereoExtrinsicsProfile(
            extrinsics=result.extrinsics,
            source_zoom_level=int(zoom_level),
            method="checkerboard_full_result",
            source_intrinsics_path=str(
                getattr(self.panel, "_candidate_intrinsics_profile_path", "") or ""
            ),
            source_intrinsics_sha256=intrinsics_content_sha256(result),
            profile_id="full-checkerboard-result",
            metadata={
                "application_contract": "exact_full_result_kd_rt_bundle",
                "calculation_mode": "checkerboard",
            },
        )
        return replace(
            profile,
            profile_id=f"full-checkerboard-{profile.content_sha256[:16]}",
        )

    def _checkerboard_config(self) -> CheckerboardConfig:
        return CheckerboardConfig(
            inner_cols=self.panel.calib_cols_spin.value(),
            inner_rows=self.panel.calib_rows_spin.value(),
            square_size_m=self.panel.calib_square_spin.value() / 1000.0,
        )

    def _default_calibration_root(self) -> Path:
        configured = getattr(self.panel, "default_calibration_root", None)
        if configured is None:
            configured = resolve_settings_path(
                DEFAULT_CALIBRATION_ROOT,
                base_dir=self.panel.app_root,
            )
        return Path(configured).expanduser().resolve(strict=False)

    def _configured_calibration_root(self) -> Path:
        text = self.panel.calib_dir_edit.text().strip()
        if not text or text == DEFAULT_CALIBRATION_ROOT:
            return self._default_calibration_root()
        return resolve_settings_path(
            text,
            base_dir=self.panel.app_root,
        ).expanduser().resolve(strict=False)

    def _calibration_dirs(self) -> tuple[Path, Path, Path]:
        """Resolve the current left/right calibration directories.

        Read-only: this is called from status/polling paths (for example
        ``workstation_calibration_capture_snapshot``) as well as from actual
        capture/compute paths, so it must not mutate session bookkeeping.
        When the configured root has diverged from the session-owned default
        (custom root, hierarchy disabled) this simply returns the configured
        root without touching ``_calibration_session_root``/
        ``_calibration_session_roots`` -- ``_ensure_calibration_session_root``
        independently re-validates and evicts stale entries from that dict
        the next time it actually needs to hand out a session root, so no
        proactive reset is needed here.
        """

        configured_root = self._configured_calibration_root()
        session_root = getattr(self.panel, "_calibration_session_root", None)
        default_root = self._default_calibration_root()
        root = CaptureDirectories.current_root(
            configured_root=configured_root, default_root=default_root,
            hierarchy_enabled=bool(getattr(self.panel, "_measurement_session_hierarchy_enabled", False)),
            session_root=session_root,
        )
        left_dir = root / "left"
        right_dir = root / "right"
        return root, left_dir, right_dir

    def _ensure_calibration_session_root(self, capture_origin: str = "manual") -> Path | None:
        """Resolve the calibration root for the current lifecycle state.

        Calibration is also a pre-session operation: its output is one of the
        inputs needed to make a measurement method ready.  When no parent
        Session is open, use the configured reusable calibration root instead
        of requiring the very Session that calibration is meant to prepare.
        Once a Session is open, retain the existing session-owned run layout.
        """

        if not hasattr(self.panel, "calib_dir_edit") or not hasattr(self.panel, "app_root"):
            return None
        configured_root = self._configured_calibration_root()
        hierarchy_enabled = bool(getattr(self.panel, "_measurement_session_hierarchy_enabled", False))
        session_state = str(getattr(self.panel, "_measurement_session_state", "IDLE") or "IDLE").upper()
        session_id = str(getattr(self.panel, "_measurement_session_id", "") or getattr(self.panel, "recording_session_id", "") or "") if hierarchy_enabled else ""
        resolver = getattr(self.panel, "_measurement_session_dir", None)
        session_directory = (
            Path(resolver(session_id))
            if hierarchy_enabled and session_state == "OPEN" and session_id and callable(resolver)
            else None
        )
        resolution = CaptureDirectories.resolve_session_root(
            configured_root=configured_root,
            default_root=None if hierarchy_enabled else self._default_calibration_root(),
            hierarchy_enabled=hierarchy_enabled, session_state=session_state,
            session_id=session_id, session_directory=session_directory,
            capture_origin=capture_origin,
            session_roots=getattr(self.panel, "_calibration_session_roots", None),
        )
        root = resolution.root
        if root is None:
            return None
        if resolution.cache_changed:
            self.panel._calibration_session_roots = resolution.session_roots
        if resolution.activate:
            self.panel._calibration_session_root = root
        if resolution.pre_session:
            CaptureDirectories.write_manifest(root, session_kind="calibration_pre_session", source=resolution.source)
            self.panel._log(f"[캘리브레이션] 세션 전 공용 캘리브레이션 폴더: {root}")
        elif resolution.created:
            CaptureDirectories.write_manifest(root, source=resolution.source)
            self.panel._log(f"[캘리브레이션] 이번 Workstation {resolution.session_key} 캘리브레이션 폴더: {root}")
        return root

    def _archive_loaded_intrinsics(self, source: Path) -> Path | None:
        """Copy one operator-loaded intrinsic JSON into the open Session."""

        if not bool(getattr(self.panel, "_measurement_session_hierarchy_enabled", False)):
            return None
        session_state = str(
            getattr(self.panel, "_measurement_session_state", "IDLE") or "IDLE"
        ).upper()
        session_id = str(
            getattr(self.panel, "_measurement_session_id", "")
            or getattr(self.panel, "recording_session_id", "")
            or ""
        )
        resolver = getattr(self.panel, "_measurement_session_dir", None)
        if session_state != "OPEN":
            # A pre-session load remains a valid reusable calibration input;
            # its bytes will be snapshotted into the measurement package when
            # that package is later created.
            return None
        if not session_id or not callable(resolver):
            raise RuntimeError("open Session calibration owner is unavailable")
        calibration_root = self._ensure_calibration_session_root("loaded_intrinsics")
        if calibration_root is None:
            return None
        return self._profiles_owner().archive_loaded_intrinsics(source, root=calibration_root)

    @staticmethod
    def _prepare_ftp_capture_dirs(root: Path, capture_family: str) -> None:
        CaptureDirectories.prepare_capture_family(root, capture_family)

    def _intrinsics_storage_base_path(self) -> Path:
        if hasattr(self.panel, "calib_dir_edit"):
            root, _left_dir, _right_dir = self.panel._calibration_dirs()
            path = root / "intrinsics.json"
        else:
            path = resolve_settings_path(self.panel.settings.intrinsics_path, base_dir=self.panel.app_root)
        stem = path.stem
        if "_zoom_" in stem:
            base_stem, zoom_suffix = stem.rsplit("_zoom_", 1)
            if zoom_suffix.isdigit():
                return path.with_name(f"{base_stem}{path.suffix}")
        return path

    def _intrinsics_storage_path(self, zoom_level: int | None = None) -> Path:
        base_path = self.panel._intrinsics_storage_base_path()
        level = self.panel.zoom_level_spin.value() if zoom_level is None else int(zoom_level)
        return base_path.with_name(f"{base_path.stem}_zoom_{level:02d}{base_path.suffix}")

    def _extrinsics_storage_path(self, zoom_level: int | None = None) -> Path:
        root, _left_dir, _right_dir = self.panel._calibration_dirs()
        selected_zoom = getattr(self.panel, "last_calibration_zoom_level", None)
        level = (
            self.panel.zoom_level_spin.value()
            if zoom_level is None and selected_zoom is None
            else selected_zoom
            if zoom_level is None
            else int(zoom_level)
        )
        return root / f"extrinsics_zoom_{int(level):02d}_large_board.json"

    def _calibration_image_paths(self, side: str) -> list[Path]:
        _root, left_dir, right_dir = self.panel._calibration_dirs()
        directory = left_dir if side == "left" else right_dir
        return sorted(directory.glob(f"{side}_*.png"))

    def _next_calibration_capture_stem(self) -> str:
        max_index = self.panel.calibration_capture_index
        for side in ("left", "right"):
            for path in self.panel._calibration_image_paths(side):
                suffix = path.stem.replace(f"{side}_", "", 1)
                if suffix.isdigit():
                    max_index = max(max_index, int(suffix))
        self.panel.calibration_capture_index = max_index + 1
        return f"{self.panel.calibration_capture_index:03d}"

    def _checkerboard_pair_max_delta_s(self) -> float:
        """Return the operator-selected Live pair timing gate in seconds."""

        widget = getattr(self.panel, "calibration_pair_max_delta_spin", None)
        reader = getattr(widget, "value", None)
        try:
            if callable(reader):
                value_s = float(reader()) / 1000.0
            else:
                settings = getattr(self.panel, "settings", None)
                value_s = float(
                    getattr(
                        settings,
                        "calibration_pair_max_delta_s",
                        CHECKERBOARD_STEREO_PAIR_MAX_DELTA_S,
                    )
                )
        except (TypeError, ValueError, OverflowError):
            return CHECKERBOARD_STEREO_PAIR_MAX_DELTA_S
        if not math.isfinite(value_s) or value_s < 0.0:
            return CHECKERBOARD_STEREO_PAIR_MAX_DELTA_S
        return min(1.0, value_s)

    def _checkerboard_capture_pair_timing(self, metadata_by_camera=None, *, maximum_delta_s=None):
        metadata = metadata_by_camera or self.panel.camera_frame_metadata
        limit = self._checkerboard_pair_max_delta_s() if maximum_delta_s is None else maximum_delta_s
        return CaptureTiming()._checkerboard_capture_pair_timing(metadata, maximum_delta_s=limit)

    _app_receive_monotonic_ns_from_metadata = staticmethod(CaptureTiming._app_receive_monotonic_ns_from_metadata)

    def _checkerboard_capture_pair_timing_evidence(self, metadata_by_camera=None, *, maximum_delta_s=None):
        metadata = metadata_by_camera or self.panel.camera_frame_metadata
        limit = self._checkerboard_pair_max_delta_s() if maximum_delta_s is None else maximum_delta_s
        return CaptureTiming()._checkerboard_capture_pair_timing_evidence(metadata, maximum_delta_s=limit)

    def capture_calibration_images(
        self,
        *,
        frames: dict[str, QtGui.QImage | None] | None = None,
        metadata_by_camera: dict[str, dict[str, object]] | None = None,
        capture_origin: str = "manual",
        capture_root: Path | None = None,
    ) -> bool:
        owner = self._calibration_job_owner()
        if owner.closing or owner.busy:
            self.panel.calibration_status_label.setText("A calibration operation is already running.")
            return False
        if frames is None:
            frames = {}
            native_metadata: dict[str, dict[str, object]] = {}
            allow_preview_fallback = bool(
                getattr(self.panel, "_allow_legacy_calibration_preview_fallback", False)
            )
            for side in ("left", "right"):
                pane = getattr(self.panel, f"{side}_pane", None)
                getter = getattr(pane, "latest_native_calibration_frame", None)
                if callable(getter):
                    native = getter(maximum_age_s=CALIBRATION_AUTO_DETECTION_MAX_AGE_S)
                    if native is not None:
                        frame_id, image, metadata = native
                        frames[side] = image
                        native_metadata[side] = {
                            **dict(metadata),
                            "frame_id": str(frame_id),
                            "calibration_source": "native_frame_cache",
                        }
                    else:
                        frames[side] = None
                else:
                    if allow_preview_fallback:
                        current = getattr(pane, "current_image", None)
                        frames[side] = current() if callable(current) else None
                    else:
                        frames[side] = None
            if native_metadata:
                metadata_by_camera = native_metadata
        if frames["left"] is None and frames["right"] is None:
            self.panel.calibration_status_label.setText(
                "Calibration capture requires a fresh native lossless frame; "
                "preview-only frames are not accepted."
            )
            self.panel._log(
                "[캘리브레이션] 이미지 캡처 실패: native lossless frame API/프레임이 없습니다."
            )
            return False
        # Freeze native images before session/root I/O, as at the original
        # admission boundary. The builder consumes these owned image copies.
        frames = {
            side: None if image is None else image.convertToFormat(QtGui.QImage.Format_RGB888).copy()
            for side, image in frames.items()
        }
        metadata_by_camera = deepcopy(metadata_by_camera or self.panel.camera_frame_metadata)
        if capture_root is None:
            self._ensure_calibration_session_root(capture_origin)
            root, _left_dir, _right_dir = self.panel._calibration_dirs()
            stem = self.panel._next_calibration_capture_stem()
        else:
            root = Path(capture_root)
            max_index = self.panel.calibration_capture_index
            for side in ("left", "right"):
                for path in self._input_owner()._calibration_image_paths_for_root(root, side):
                    suffix = path.stem.replace(f"{side}_", "", 1)
                    if suffix.isdigit():
                        max_index = max(max_index, int(suffix))
            self.panel.calibration_capture_index = max_index + 1
            stem = f"{self.panel.calibration_capture_index:03d}"
        available, error = opencv_available()
        config = self.panel._checkerboard_config() if available else None
        zoom_snapshot = self._capture_zoom_snapshot()
        self.panel.last_capture_zoom_level = zoom_snapshot.get("capture_zoom_level")
        request = CaptureRequestBuilder.build(
            root=root, stem=stem, frames=frames, metadata_by_camera=metadata_by_camera,
            copy_frames=False,
            preview_metadata=getattr(self.panel, "camera_frame_metadata", {}),
            capture_origin=capture_origin, pair_max_delta_s=self._checkerboard_pair_max_delta_s(),
            zoom_snapshot=zoom_snapshot, config=config, opencv_error=error,
            pose_views=getattr(self, "_auto_capture_pending_pose_views", None),
            pose_novelty=getattr(self, "_auto_capture_pending_pose_novelty", None),
        )
        frames = request.frames
        worker_factory = lambda: CheckerboardCaptureWorker(**request.worker_arguments())
        for side in ("left", "right"):
            self._publish_calibration_capture_event(
                {
                    "side": side,
                    "phase": "started",
                    "state": "frame_ready" if frames.get(side) is not None else "no_frame",
                    "command": "RTSP current frame",
                    "response": "frame queued for PNG/checkerboard validation"
                    if frames.get(side) is not None
                    else "no current frame",
                },
                operation="Capture Live Pair",
                capture_id=stem,
            )
        job = self._prepare_calibration_job(
            "capture", worker_factory=worker_factory,
            completed=lambda result, origin=str(capture_origin), root=root: self._apply_capture_job_result(result, origin, root),
            failed=lambda error, origin=str(capture_origin): self._apply_capture_job_failure(error, origin),
            progress=lambda payload: self._publish_calibration_capture_event(payload, operation="Capture Live Pair", capture_id=stem),
        )
        if job is None:
            return False

        def prepare_ui():
            self._capture_origin = str(capture_origin)
            self.panel.capture_calibration_button.setEnabled(False)
            self.panel.run_calibration_button.setEnabled(False)
            self.panel.checkerboard_validation_random_button.setEnabled(False)
            self.panel.calibration_status_label.setText(f"Capture Live Pair {stem}: encoding and checkerboard validation running")
            self._capture_feedback_begin(
                root=root,
                capture_id=stem,
                origin=str(capture_origin),
            )
        return self._start_calibration_job(job, prepare_ui)

    def capture_calibration_images_wiris_ftp(self) -> bool:
        """Capture a left/right pair via a WIRIS HR still + FTP download.

        A second source alongside ``capture_calibration_images``'s live-RTSP
        path, not a replacement: once the still is downloaded, it is handed
        to that exact same method (``frames=..., capture_origin="ftp"``), so
        save/naming/checkerboard-validation logic is entirely unchanged.
        """
        if self._ftp_owner().closing:
            return False

        if self.panel.calibration_wiris_ftp_thread is not None:
            self._set_ftp_status(
                "A WIRIS FTP calibration capture is already in progress."
            )
            return False
        if self.panel.calibration_wiris_ftp_oz_thread is not None:
            self._set_ftp_status(
                "A WIRIS FTP (OZ) calibration capture is already in progress."
            )
            return False
        if self.panel.checkerboard_calibration_thread is not None:
            self._set_ftp_status("A calibration operation is already running.")
            return False
        settings = getattr(self.panel, "settings", None)
        if not bool(getattr(settings, "wiris_ftp_enabled", False)):
            self._set_ftp_status(
                "WIRIS FTP capture requires \"Enable FTP download\" in Settings."
            )
            return False
        target_infos = self.panel._camera_photo_target_infos()
        if len(target_infos) != 2:
            self._set_ftp_status(
                "WIRIS FTP capture failed: both WIRIS control hosts are required."
            )
            return False
        self._ensure_calibration_session_root("ftp")
        root, _left_dir, _right_dir = self.panel._calibration_dirs()
        self._prepare_ftp_capture_dirs(root, "HR")
        capture_id = f"calib-{time.time_ns()}-{uuid.uuid4().hex[:8]}"
        kwargs = self.panel._wiris_ftp_worker_kwargs(output_dir=root)
        worker = WirisPnPPhotoCaptureWorker(
            target_infos,
            capture_id=capture_id,
            capture_channel="calib",
            completion_timeout_s=10.0,
            **kwargs,
        )
        thread, worker = self._ftp_owner().prepare(
            'ftp', worker, root=root,
            completed=self.panel._handle_calibration_wiris_ftp_capture_completed, failed=self.panel._handle_calibration_wiris_ftp_capture_failed, retired=self.panel._clear_calibration_wiris_ftp_capture_worker,
        )
        self._connect_calibration_worker_progress(
            worker,
            operation="Capture HR Still (FTP)",
            capture_id=capture_id,
        )
        self.panel.capture_calibration_button.setEnabled(False)
        self.panel.run_calibration_button.setEnabled(False)
        status = f"WIRIS FTP Capture {capture_id}: 좌/우 카메라 내부 저장 중"
        self.panel.calibration_status_label.setText(status)
        self.panel._log(f"[캘리브레이션] {status}")
        return self._ftp_owner().start('ftp', expected_thread=thread)

    def _handle_calibration_wiris_ftp_capture_completed(self, result: object) -> None:
        record = result if isinstance(result, dict) else {}
        cameras = record.get("cameras")
        cameras = cameras if isinstance(cameras, dict) else {}
        capture_id = str(record.get("capture_id") or "")
        self._log_ftp_camera_results("WIRIS FTP Capture", capture_id, cameras)
        self._publish_calibration_camera_summary(
            operation="Capture HR Still (FTP)",
            capture_id=capture_id,
            cameras=cameras,
        )
        failures = self._ftp_capture_failures(cameras)
        if set(cameras) != {"left", "right"} or failures:
            self._handle_calibration_wiris_ftp_capture_failed(
                "; ".join(failures) or "camera result is missing left/right entries"
            )
            return
        frames: dict[str, QtGui.QImage | None] = {}
        for side, item in cameras.items():
            path = str(item.get("ftp_local_path") or "")
            image = QtGui.QImage(path) if path else QtGui.QImage()
            if image.isNull():
                self._handle_calibration_wiris_ftp_capture_failed(
                    f"{side}: downloaded file could not be decoded as an image ({path or 'no path'})"
                )
                return
            frames[side] = image
        self._publish_calibration_ftp_pair_ready(
            operation="Capture HR Still (FTP)",
            capture_id=capture_id,
            cameras=cameras,
            frames=frames,
        )
        self.panel._log(f"[캘리브레이션] WIRIS FTP Capture {capture_id}: 다운로드 완료, 캘리브레이션 저장 시작")
        self.panel.capture_calibration_images(
            frames=frames,
            metadata_by_camera=self._ftp_calibration_capture_metadata(
                capture_id=capture_id,
                cameras=cameras,
                frames=frames,
            ),
            capture_origin="ftp",
            capture_root=self._ftp_owner().root_for("ftp"),
        )

    def _handle_calibration_wiris_ftp_capture_failed(self, error: str) -> None:
        for side in ("left", "right"):
            self._publish_calibration_capture_event(
                {
                    "side": side,
                    "phase": "failed",
                    "state": "failed",
                    "response": str(error),
                },
                operation="Capture HR Still (FTP)",
            )
        self.panel._log(f"[캘리브레이션] WIRIS FTP Capture 실패: {error}")
        self.panel.calibration_status_label.setText(f"WIRIS FTP Capture failed: {error}")
        self.panel.capture_calibration_button.setEnabled(True)
        self.panel.run_calibration_button.setEnabled(True)

    def _clear_calibration_wiris_ftp_capture_worker(self) -> None:
        self._ftp_owner().clear('ftp')

    def _set_ftp_status(self, message: str) -> None:
        self.panel.calibration_status_label.setText(message)
        self.panel._log(f"[캘리브레이션] FTP 상태: {message}")

    def _log_ftp_camera_results(
        self,
        source_label: str,
        capture_id: str,
        cameras: dict[object, object],
    ) -> None:
        for side in ("left", "right"):
            item = cameras.get(side)
            if not isinstance(item, dict):
                continue
            details = [
                f"state={item.get('state', 'failed')}",
                f"camera_saved={bool(item.get('stored_on_camera'))}",
                f"ftp_downloaded={bool(item.get('ftp_downloaded'))}",
            ]
            path = str(item.get("ftp_local_path") or "")
            if path:
                details.append(f"local={path}")
            remote_path = str(item.get("ftp_remote_path") or "")
            if remote_path:
                details.append(f"remote={remote_path}")
            for key in ("error", "ftp_error"):
                error = str(item.get(key) or "")
                if error:
                    details.append(f"{key}={error}")
            self.panel._log(
                f"[캘리브레이션] {source_label} {capture_id} {side}: "
                + "; ".join(details)
            )

    _ftp_capture_failures = staticmethod(OzCaptureLedger._ftp_capture_failures)

    _ftp_calibration_capture_metadata = staticmethod(OzCaptureLedger._ftp_calibration_capture_metadata)

    def capture_calibration_images_wiris_ftp_oz(self) -> bool:
        """Same as ``capture_calibration_images_wiris_ftp``, on the OZ channel.

        The WIRIS normal visible operating-zoom still (``SIVI TRUE``) instead
        of the HR still (``SIHR TRUE``) -- lower resolution than HR, but a
        faster capture with no camera reboot risk (unlike changing HR
        resolution via ``SHRR``). Downloads land in a separate
        ``calib_capture/OZ`` subfolder so they never mix with HR downloads,
        and feed the exact same ``capture_calibration_images`` save path.
        """
        if self._ftp_owner().closing:
            return False

        if self.panel.calibration_wiris_ftp_oz_thread is not None:
            self._set_ftp_status(
                "A WIRIS FTP (OZ) calibration capture is already in progress."
            )
            return False
        if self.panel.calibration_wiris_ftp_thread is not None:
            self._set_ftp_status(
                "A WIRIS FTP calibration capture is already in progress."
            )
            return False
        if self.panel.checkerboard_calibration_thread is not None:
            self._set_ftp_status("A calibration operation is already running.")
            return False
        settings = getattr(self.panel, "settings", None)
        if not bool(getattr(settings, "wiris_ftp_enabled", False)):
            self._set_ftp_status(
                "WIRIS FTP capture requires \"Enable FTP download\" in Settings."
            )
            return False
        target_infos = self.panel._camera_photo_target_infos()
        if len(target_infos) != 2:
            self._set_ftp_status(
                "WIRIS FTP capture failed: both WIRIS control hosts are required."
            )
            return False
        self._ensure_calibration_session_root("ftp_oz")
        root, _left_dir, _right_dir = self.panel._calibration_dirs()
        self._prepare_ftp_capture_dirs(root, "OZ")
        capture_id = f"calib-oz-{time.time_ns()}-{uuid.uuid4().hex[:8]}"
        kwargs = self.panel._wiris_ftp_worker_kwargs(output_dir=root)
        worker = WirisPnPPhotoCaptureWorker(
            target_infos,
            capture_id=capture_id,
            image_enable_command="SIVI TRUE",
            capture_channel="calib_oz",
            completion_timeout_s=10.0,
            **kwargs,
        )
        thread, worker = self._ftp_owner().prepare(
            'ftp_oz', worker, root=root,
            completed=self.panel._handle_calibration_wiris_ftp_oz_capture_completed, failed=self.panel._handle_calibration_wiris_ftp_oz_capture_failed, retired=self.panel._clear_calibration_wiris_ftp_oz_capture_worker,
        )
        self._connect_calibration_worker_progress(
            worker,
            operation="Capture OZ Still (FTP)",
            capture_id=capture_id,
        )
        self.panel.capture_calibration_button.setEnabled(False)
        self.panel.run_calibration_button.setEnabled(False)
        status = f"WIRIS FTP (OZ) Capture {capture_id}: 좌/우 카메라 내부 저장 중"
        self.panel.calibration_status_label.setText(status)
        self.panel._log(f"[캘리브레이션] {status}")
        return self._ftp_owner().start('ftp_oz', expected_thread=thread)

    def _handle_calibration_wiris_ftp_oz_capture_completed(self, result: object) -> None:
        record = result if isinstance(result, dict) else {}
        cameras = record.get("cameras")
        cameras = cameras if isinstance(cameras, dict) else {}
        capture_id = str(record.get("capture_id") or "")
        self._log_ftp_camera_results("WIRIS FTP (OZ) Capture", capture_id, cameras)
        self._publish_calibration_camera_summary(
            operation="Capture OZ Still (FTP)",
            capture_id=capture_id,
            cameras=cameras,
        )
        failures = self._ftp_capture_failures(cameras)
        if set(cameras) != {"left", "right"} or failures:
            self._handle_calibration_wiris_ftp_oz_capture_failed(
                "; ".join(failures) or "camera result is missing left/right entries"
            )
            return
        frames: dict[str, QtGui.QImage | None] = {}
        for side, item in cameras.items():
            path = str(item.get("ftp_local_path") or "")
            image = QtGui.QImage(path) if path else QtGui.QImage()
            if image.isNull():
                self._handle_calibration_wiris_ftp_oz_capture_failed(
                    f"{side}: downloaded file could not be decoded as an image ({path or 'no path'})"
                )
                return
            frames[side] = image
        self._publish_calibration_ftp_pair_ready(
            operation="Capture OZ Still (FTP)",
            capture_id=capture_id,
            cameras=cameras,
            frames=frames,
        )
        self.panel._log(f"[캘리브레이션] WIRIS FTP (OZ) Capture {capture_id}: 다운로드 완료, 캘리브레이션 저장 시작")
        self.panel.capture_calibration_images(
            frames=frames,
            metadata_by_camera=self._ftp_calibration_capture_metadata(
                capture_id=capture_id,
                cameras=cameras,
                frames=frames,
            ),
            capture_origin="ftp_oz",
            capture_root=self._ftp_owner().root_for("ftp_oz"),
        )

    def _handle_calibration_wiris_ftp_oz_capture_failed(self, error: str) -> None:
        for side in ("left", "right"):
            self._publish_calibration_capture_event(
                {
                    "side": side,
                    "phase": "failed",
                    "state": "failed",
                    "response": str(error),
                },
                operation="Capture OZ Still (FTP)",
            )
        self.panel._log(f"[캘리브레이션] WIRIS FTP (OZ) Capture 실패: {error}")
        self.panel.calibration_status_label.setText(f"WIRIS FTP (OZ) Capture failed: {error}")
        self.panel.capture_calibration_button.setEnabled(True)
        self.panel.run_calibration_button.setEnabled(True)

    def _clear_calibration_wiris_ftp_oz_capture_worker(self) -> None:
        self._ftp_owner().clear('ftp_oz')

    # -- OZ capture-only + FTP sync -------------------------------------
    #
    # "Capture OZ Still (FTP)" above bundles a still trigger with an immediate FTP
    # download; every capture pays the full FTP round-trip before the next
    # one can start. These three pieces decouple that: capture repeatedly
    # (fast, camera-local only), then sync everything new in one pass, then
    # let the operator pick which synced pairs feed the calculation.
    #
    # Extrinsics need proof that a left/right pair came from the *same*
    # trigger. The two camera clocks can differ by hours, so remote
    # timestamps can't establish that once several stills have piled up
    # un-downloaded. Instead a local ledger (``calib_capture/OZ/ledger.json``)
    # records each trigger round and whether *both* sides reported
    # ``stored_on_camera`` -- only such "synchronized" rounds are ever
    # eligible for sync/pairing, and syncing matches each camera's new
    # remote files to ledger rounds strictly in that camera's own upload
    # order (MDTM), never by cross-camera timestamp.

    def _calib_oz_ledger_path(self) -> Path:
        root = self._ftp_owner().ledger_root
        if root is None:
            root = self.panel._calibration_dirs()[0]
        return Path(root) / "calib_capture" / "OZ" / "ledger.json"

    def _load_calib_oz_ledger(self):
        return OzCaptureLedger().read(self.panel._calib_oz_ledger_path())

    def _save_calib_oz_ledger(self, ledger):
        OzCaptureLedger().write(self.panel._calib_oz_ledger_path(), ledger)

    def capture_calibration_images_wiris_oz_capture_only(self) -> bool:
        """Trigger an OZ still on both cameras and leave it on camera storage.

        No FTP download here -- see ``sync_calibration_images_wiris_oz_ftp``.
        Meant to be pressed repeatedly while watching the live view for board
        framing, accumulating stills quickly without paying an FTP round-trip
        per shot.
        """
        if self._ftp_owner().closing:
            return False

        busy_threads = (
            self.panel.calibration_wiris_oz_format_thread,
            self.panel.calibration_wiris_oz_gilc_thread,
            self.panel.calibration_wiris_oz_capture_thread,
            self.panel.calibration_wiris_oz_sync_thread,
            self.panel.calibration_wiris_ftp_thread,
            self.panel.calibration_wiris_ftp_oz_thread,
            self.panel.checkerboard_calibration_thread,
        )
        if any(thread is not None for thread in busy_threads):
            self.panel.calibration_status_label.setText(
                "A WIRIS OZ capture/sync/calibration operation is already in progress."
            )
            return False
        target_infos = self.panel._camera_photo_target_infos()
        if len(target_infos) != 2:
            self.panel.calibration_status_label.setText(
                "OZ capture failed: both WIRIS control hosts are required."
            )
            return False
        self._ensure_calibration_session_root("oz_capture")
        self._ftp_owner().ledger_root = self.panel._calibration_dirs()[0]
        # Force a known, single-file-per-round format state before every
        # trigger -- CPTR emits one file per enabled SIxx toggle, and the
        # two cameras were found with different toggles on (left had
        # radiometric on, right had HR on). Best-effort: proceed to capture
        # either way, since a failed format command shouldn't block the shot,
        # it'll just be caught later by the sync's per-name ``-visible.jpg``
        # filter and count check.
        self.panel.calibration_status_label.setText("Capture OZ Still: 이미지 포맷 강제 설정 중")
        worker = WirisImageFormatWorker(
            target_infos,
            commands=("SIRJ FALSE", "SISR FALSE", "SIHR FALSE", "SIVI TRUE"),
        )
        thread, worker = self._ftp_owner().prepare(
            'oz_format', worker, root=self._ftp_owner().ledger_root,
            completed=self.panel._handle_calibration_wiris_oz_format_preset_completed, failed=self.panel._handle_calibration_wiris_oz_format_preset_failed, retired=self.panel._clear_calibration_wiris_oz_format_worker,
        )
        self._connect_calibration_worker_progress(
            worker,
            operation="Capture OZ Still (format)",
        )
        self._pending_oz_capture_target_infos = target_infos
        return self._ftp_owner().start('oz_format', expected_thread=thread)

    def _handle_calibration_wiris_oz_format_preset_completed(self, result: object) -> None:
        record = result if isinstance(result, dict) else {}
        for label, responses in record.items():
            self.panel._log(f"[캘리브레이션] OZ 포맷 강제 설정 {label}: {responses}")
        self._start_calibration_wiris_oz_capture()

    def _handle_calibration_wiris_oz_format_preset_failed(self, error: str) -> None:
        self.panel._log(f"[캘리브레이션] OZ 포맷 강제 설정 실패 (촬영은 계속 진행): {error}")
        self._start_calibration_wiris_oz_capture()

    def _clear_calibration_wiris_oz_format_worker(self) -> None:
        self._ftp_owner().clear('oz_format')

    def _start_calibration_wiris_oz_capture(self) -> None:
        if self._ftp_owner().closing:
            return None
        target_infos = getattr(self, "_pending_oz_capture_target_infos", ())
        self._pending_oz_capture_target_infos = ()
        if len(target_infos) != 2:
            return
        self._ensure_calibration_session_root("oz_capture")
        capture_id = f"calib-ozcap-{time.time_ns()}-{uuid.uuid4().hex[:8]}"
        worker = WirisOZPhotoCaptureWorker(target_infos, capture_id=capture_id, completion_timeout_s=10.0)
        thread, worker = self._ftp_owner().prepare(
            'oz_capture', worker, root=self._ftp_owner().root_for('oz_format', self._ftp_owner().ledger_root),
            completed=self.panel._handle_calibration_wiris_oz_capture_only_completed, failed=self.panel._handle_calibration_wiris_oz_capture_only_failed, retired=self.panel._clear_calibration_wiris_oz_capture_only_worker,
        )
        self._connect_calibration_worker_progress(
            worker,
            operation="Capture OZ Still",
            capture_id=capture_id,
        )
        self.panel.calibration_status_label.setText(f"Capture OZ Still {capture_id}: 좌/우 카메라 내부 저장 중")
        self._ftp_owner().start('oz_capture', expected_thread=thread)

    def _handle_calibration_wiris_oz_capture_only_completed(self, result: object) -> None:
        record = result if isinstance(result, dict) else {}
        cameras = record.get("cameras")
        cameras = cameras if isinstance(cameras, dict) else {}
        capture_id = str(record.get("capture_id") or "")
        self._publish_calibration_camera_summary(
            operation="Capture OZ Still",
            capture_id=capture_id,
            cameras=cameras,
        )
        ledger = self.panel._load_calib_oz_ledger()
        left_ok, right_ok, synchronized, pending = OzCaptureLedger().record_capture(
            ledger, capture_id=capture_id, cameras=cameras,
        )
        self.panel._save_calib_oz_ledger(ledger)
        if not synchronized:
            self.panel._log(
                f"[캘리브레이션] Capture OZ Still {capture_id}: 좌우 중 한쪽만 성공 -- 페어링 대상에서 제외 "
                f"(left={left_ok}, right={right_ok})"
            )
        self.panel.calibration_status_label.setText(
            f"Capture OZ Still {capture_id}: 완료 (동기화 대기 {pending}쌍)"
        )

    def _handle_calibration_wiris_oz_capture_only_failed(self, error: str) -> None:
        for side in ("left", "right"):
            self._publish_calibration_capture_event(
                {
                    "side": side,
                    "phase": "failed",
                    "state": "failed",
                    "response": str(error),
                },
                operation="Capture OZ Still",
            )
        self.panel._log(f"[캘리브레이션] Capture OZ Still 실패: {error}")
        self.panel.calibration_status_label.setText(f"Capture OZ Still failed: {error}")

    def _clear_calibration_wiris_oz_capture_only_worker(self) -> None:
        self._ftp_owner().clear('oz_capture')

    #: GILC's storage-location names, mapped to their FTP root folder name.
    IMAGE_LOCATION_FTP_ROOTS = {"SSD": "ssd", "SD_CARD": "sdcard", "FLASH_DRIVE": "flashd"}

    def sync_calibration_images_wiris_oz_ftp(self) -> bool:
        """Download every un-synced, both-sides-successful OZ round via FTP.

        WIRIS Enterprise has no fixed image directory -- stills land under
        /<GILC storage root>/<session folder>/img/, and a new session folder
        appears every time Ethernet Mode (re)starts or the camera reboots.
        The storage root is queried fresh via GILC before every sync (a
        camera-side ``SILC`` change should be picked up immediately, not
        cached); the sync worker itself walks every session folder under it.
        """
        if self._ftp_owner().closing:
            return False

        busy_threads = (
            self.panel.calibration_wiris_oz_format_thread,
            self.panel.calibration_wiris_oz_gilc_thread,
            self.panel.calibration_wiris_oz_capture_thread,
            self.panel.calibration_wiris_oz_sync_thread,
            self.panel.calibration_wiris_ftp_thread,
            self.panel.calibration_wiris_ftp_oz_thread,
            self.panel.checkerboard_calibration_thread,
        )
        if any(thread is not None for thread in busy_threads):
            self.panel.calibration_status_label.setText(
                "A WIRIS OZ capture/sync/calibration operation is already in progress."
            )
            return False
        settings = getattr(self.panel, "settings", None)
        if not bool(getattr(settings, "wiris_ftp_enabled", False)):
            self.panel.calibration_status_label.setText(
                "WIRIS FTP sync requires \"Enable FTP download\" in Settings."
            )
            return False
        target_infos = self.panel._camera_photo_target_infos()
        if len(target_infos) != 2:
            self.panel.calibration_status_label.setText(
                "WIRIS FTP sync failed: both WIRIS control hosts are required."
            )
            return False
        self._ensure_calibration_session_root("ftp_oz_sync")
        self._ftp_owner().ledger_root = self.panel._calibration_dirs()[0]
        ledger = self.panel._load_calib_oz_ledger()
        rounds = ledger["rounds"]
        pending_rounds = [row for row in rounds if row.get("synchronized") and not row.get("downloaded")]
        if not pending_rounds:
            self.panel.calibration_status_label.setText("동기화할 새 OZ 회차가 없습니다.")
            return False
        self.panel.calibration_status_label.setText("WIRIS FTP 동기화: 저장소 위치(GILC) 확인 중")
        worker = WirisImageFormatWorker(target_infos, commands=("GILC",))
        thread, worker = self._ftp_owner().prepare(
            'oz_gilc', worker, root=self._ftp_owner().ledger_root,
            completed=self.panel._handle_calibration_wiris_oz_gilc_completed, failed=self.panel._handle_calibration_wiris_oz_gilc_failed, retired=self.panel._clear_calibration_wiris_oz_gilc_worker,
        )
        self._pending_ftp_oz_sync_target_infos = target_infos
        self._pending_ftp_oz_sync_rounds = pending_rounds
        return self._ftp_owner().start('oz_gilc', expected_thread=thread)

    def _handle_calibration_wiris_oz_gilc_completed(self, result: object) -> None:
        if self._ftp_owner().closing:
            return None
        record = result if isinstance(result, dict) else {}
        target_infos = getattr(self, "_pending_ftp_oz_sync_target_infos", ())
        pending_rounds = getattr(self, "_pending_ftp_oz_sync_rounds", [])
        self._pending_ftp_oz_sync_target_infos = ()
        self._pending_ftp_oz_sync_rounds = []
        storage_root_by_label: dict[str, str] = {}
        for label, responses in record.items():
            location = str((responses or {}).get("GILC", "")).strip().upper()
            root_name = self.IMAGE_LOCATION_FTP_ROOTS.get(location)
            if root_name is None:
                self.panel._log(
                    f"[캘리브레이션] WIRIS FTP 동기화 실패: {label} GILC 응답을 알 수 없습니다 ({location!r})."
                )
                self.panel.calibration_status_label.setText(
                    "WIRIS FTP sync failed: 저장소 위치(GILC)를 확인하지 못했습니다."
                )
                return
            storage_root_by_label[label] = root_name
        if len(storage_root_by_label) != 2:
            self.panel.calibration_status_label.setText(
                "WIRIS FTP sync failed: 좌우 저장소 위치를 모두 확인하지 못했습니다."
            )
            return
        root = self._ftp_owner().root_for("oz_gilc", self.panel._calibration_dirs()[0])
        output_dir = CaptureDirectories.prepare_capture_family(root, "OZ")
        kwargs = self.panel._wiris_ftp_worker_kwargs(output_dir=output_dir)
        ledger = self.panel._load_calib_oz_ledger()
        fetch_counts = {"left": len(pending_rounds), "right": len(pending_rounds)}
        worker = WirisOzFtpSyncWorker(
            target_infos,
            ftp_settings_by_label=kwargs["ftp_settings_by_label"],
            storage_root_by_label=storage_root_by_label,
            ftp_output_dir=output_dir,
            fetch_counts=fetch_counts,
            downloaded_remote_names=ledger.get("consumed_remote_names") or {},
        )
        thread, worker = self._ftp_owner().prepare(
            'oz_sync', worker, root=root,
            completed=self.panel._handle_calibration_wiris_oz_sync_completed, failed=self.panel._handle_calibration_wiris_oz_sync_failed, retired=self.panel._clear_calibration_wiris_oz_sync_worker,
        )
        self._pending_ftp_oz_sync_round_ids = [str(row.get("capture_id")) for row in pending_rounds]
        self.panel.calibration_status_label.setText(
            f"WIRIS FTP 동기화: {len(pending_rounds)}쌍 다운로드 중"
        )
        self._ftp_owner().start('oz_sync', expected_thread=thread)

    def _handle_calibration_wiris_oz_gilc_failed(self, error: str) -> None:
        self.panel._log(f"[캘리브레이션] WIRIS FTP 동기화: GILC 조회 실패 - {error}")
        self.panel.calibration_status_label.setText(f"WIRIS FTP sync failed: GILC query failed ({error})")
        self._pending_ftp_oz_sync_target_infos = ()
        self._pending_ftp_oz_sync_rounds = []

    def _clear_calibration_wiris_oz_gilc_worker(self) -> None:
        self._ftp_owner().clear('oz_gilc')

    def _handle_calibration_wiris_oz_sync_completed(self, result: object) -> None:
        record = result if isinstance(result, dict) else {}
        left = record.get("left") if isinstance(record.get("left"), dict) else {}
        right = record.get("right") if isinstance(record.get("right"), dict) else {}
        for side, item in (("left", left), ("right", right)):
            if item.get("error"):
                self.panel._log(f"[캘리브레이션] WIRIS FTP 동기화: {side} 오류 - {item['error']}")
        left_items = list(left.get("items") or ())
        right_items = list(right.get("items") or ())
        round_ids = list(getattr(self, "_pending_ftp_oz_sync_round_ids", ()))
        self._pending_ftp_oz_sync_round_ids = []

        ledger = self.panel._load_calib_oz_ledger()
        batch, messages, status = OzCaptureLedger().assemble_sync_batch(ledger, record, round_ids)
        for message in messages:
            self.panel._log(message)
        if batch is None:
            self.panel.calibration_status_label.setText(status)
            return
        queue, delete_names = batch
        self.panel._save_calib_oz_ledger(ledger)
        self._ftp_owner().queue_root = self._ftp_owner().root_for("oz_sync")
        self._pending_ftp_oz_sync_frames = queue
        self._ftp_oz_sync_saved_count = 0
        self.panel._process_next_ftp_oz_sync_pair()
        # Only ever asked for once every file in the batch downloaded clean
        # and every round paired 1:1 -- see the hard-fail check above.
        self.panel._start_calibration_wiris_oz_delete(delete_names)

    def _handle_calibration_wiris_oz_sync_failed(self, error: str) -> None:
        self.panel._log(f"[캘리브레이션] WIRIS FTP 동기화 실패: {error}")
        self.panel.calibration_status_label.setText(f"WIRIS FTP sync failed: {error}")
        self._pending_ftp_oz_sync_round_ids = []

    def _clear_calibration_wiris_oz_sync_worker(self) -> None:
        self._ftp_owner().clear('oz_sync')

    def _start_calibration_wiris_oz_delete(self, names_by_label: dict[str, list[str]]) -> None:
        if self._ftp_owner().closing:
            return None
        if not any(names_by_label.values()):
            return
        if self.panel.calibration_wiris_oz_delete_thread is not None:
            self.panel._log(
                "[캘리브레이션] WIRIS FTP 원본 삭제: 이전 삭제 작업이 아직 진행 중이라 이번 정리는 건너뜁니다 "
                "(로컬 사본은 이미 확보됨, 카메라에 파일이 남아있습니다)."
            )
            return
        target_infos = self.panel._camera_photo_target_infos()
        if len(target_infos) != 2:
            return
        kwargs = self.panel._wiris_ftp_worker_kwargs()
        worker = WirisFtpBatchDeleteWorker(
            target_infos,
            ftp_settings_by_label=kwargs["ftp_settings_by_label"],
            names_by_label=names_by_label,
        )
        thread, worker = self._ftp_owner().prepare(
            'oz_delete', worker,
            completed=self.panel._handle_calibration_wiris_oz_delete_completed, failed=self.panel._handle_calibration_wiris_oz_delete_failed, retired=self.panel._clear_calibration_wiris_oz_delete_worker,
        )
        self._ftp_owner().start('oz_delete', expected_thread=thread)

    def _handle_calibration_wiris_oz_delete_completed(self, result: object) -> None:
        record = result if isinstance(result, dict) else {}
        for side in ("left", "right"):
            item = record.get(side) if isinstance(record.get(side), dict) else {}
            if item.get("error"):
                self.panel._log(f"[캘리브레이션] WIRIS FTP 원본 삭제: {side} 오류 - {item['error']}")
            for name, error in (item.get("delete_errors") or {}).items():
                self.panel._log(
                    f"[캘리브레이션] WIRIS FTP 원본 삭제 실패 {side}/{name}: {error} "
                    "(로컬 사본은 이미 확보됨, 카메라에 파일이 남아있습니다)"
                )
            deleted = item.get("deleted_names") or []
            if deleted:
                self.panel._log(f"[캘리브레이션] WIRIS FTP 원본 삭제: {side} {len(deleted)}개 삭제됨")

    def _handle_calibration_wiris_oz_delete_failed(self, error: str) -> None:
        self.panel._log(f"[캘리브레이션] WIRIS FTP 원본 삭제 실패: {error} (로컬 사본은 이미 확보됨)")

    def _clear_calibration_wiris_oz_delete_worker(self) -> None:
        self._ftp_owner().clear('oz_delete')

    def _process_next_ftp_oz_sync_pair(self):
        self._ftp_owner()._process_next_ftp_oz_sync_pair()

    def _resume_ftp_oz_sync_queue(self, queue):
        self._ftp_owner()._resume_ftp_oz_sync_queue(queue)

    # -- Manual calculation-pair selection --------------------------------

    def _calibration_pair_selection_path(self) -> Path:
        root, _left_dir, _right_dir = self.panel._calibration_dirs()
        return root / "calibration_pair_selection.json"

    def _load_calibration_pair_selection(self) -> dict[str, object]:
        path = self.panel._calibration_pair_selection_path()
        return load_calibration_selection_sidecar(path).to_mapping()

    _resolve_calibration_selection_paths = staticmethod(CalibrationInputs._resolve_calibration_selection_paths)

    def _report_calibration_selection_errors(self, errors: tuple[str, ...]) -> None:
        if not errors:
            return
        text = (
            "Calibration selection is invalid; all-photo fallback blocked: "
            + ", ".join(errors)
        )
        if hasattr(self.panel, "calibration_status_label"):
            self.panel.calibration_status_label.setText(text)
        if hasattr(self.panel, "checkerboard_validation_status_label") and hasattr(
            self.panel, "_set_health_label"
        ):
            self.panel._set_health_label(
                self.panel.checkerboard_validation_status_label,
                text,
                "bad",
            )
        logger = getattr(self.panel, "_log", None)
        if callable(logger):
            logger(f"[캘리브레이션] {text}")

    def _selected_calibration_file_lists(self, root, fallback_left, fallback_right, *, selection=None):
        selected = self.panel._load_calibration_pair_selection() if selection is None else selection
        return self._input_owner()._selected_calibration_file_lists(root, fallback_left, fallback_right, selection=selected)

    _selected_calibration_pairs = staticmethod(CalibrationInputs._selected_calibration_pairs)

    def _effective_calibration_inputs(self, root, fallback_left, fallback_right, scanned_pairs, *, mode, selection=None, pair_limit=None):
        selected = self.panel._load_calibration_pair_selection() if selection is None else selection
        limit = self._calibration_calculation_pair_limit() if pair_limit is None else pair_limit
        return self._input_owner()._effective_calibration_inputs(root, fallback_left, fallback_right, scanned_pairs, mode=mode, selection=selected, pair_limit=limit)

    def _random_validation_inputs(
        self,
        reference: StereoIntrinsicsResult,
        *,
        mode: str,
    ) -> tuple[Path, list[Path], list[Path], list[tuple[str, Path, Path]]]:
        """Return the exact effective pool used by ``reference`` when available."""

        snapshot = getattr(self.panel, "checkerboard_calibration_selection_snapshot", None)
        if (
            isinstance(snapshot, dict)
            and snapshot.get("reference_result") is reference
            and str(snapshot.get("mode") or "") == str(mode)
        ):
            return (
                Path(snapshot["root"]),
                [Path(path) for path in snapshot.get("left_intrinsic_paths", ())],
                [Path(path) for path in snapshot.get("right_intrinsic_paths", ())],
                [
                    (str(key), Path(left), Path(right))
                    for key, left, right in snapshot.get("stereo_pairs", ())
                ],
            )

        root, fallback_left, fallback_right, scanned_pairs = (
            self.panel._checkerboard_validation_scan_counts()
        )
        left_paths, right_paths, stereo_pairs, _explicit = self._effective_calibration_inputs(
            root,
            fallback_left,
            fallback_right,
            scanned_pairs,
            mode=mode,
        )
        return root, left_paths, right_paths, stereo_pairs

    _solver_effective_selection = staticmethod(CalibrationInputs._solver_effective_selection)

    def _choose_calibration_pairs(self, pairs, *, selection=None, root=None, pair_limit=None):
        selected = self.panel._load_calibration_pair_selection() if selection is None else selection
        selected_root = self.panel._calibration_dirs()[0] if root is None else root
        limit = self._calibration_calculation_pair_limit() if pair_limit is None else pair_limit
        return self._input_owner()._choose_calibration_pairs(pairs, selection=selected, root=selected_root, pair_limit=limit)

    def toggle_auto_calibration_capture(self) -> None:
        if self._auto_capture_active:
            self.stop_auto_calibration_capture()
        else:
            self.start_auto_calibration_capture()

    def start_auto_calibration_capture(self) -> bool:
        if self._auto_capture_active:
            return True
        if self.panel.checkerboard_calibration_thread is not None:
            self.panel.calibration_status_label.setText("A calibration operation is already running.")
            return False
        interval_s = float(self.panel.calibration_auto_interval_spin.value())
        self._capture_feedback_reset_run(origin="auto")
        self._capture_runtime_owner().start(interval_s)
        self.panel.capture_calibration_auto_button.setText("자동 촬영 중지")
        self.panel.calibration_status_label.setText(
            f"자동 촬영 대기: 유효 stereo pair 0개 | "
            f"간격={interval_s:.1f}s | Stop할 때까지 계속 촬영합니다."
        )
        self._auto_calibration_capture_tick()
        return True

    def stop_auto_calibration_capture(self, message: str | None = None) -> bool:
        was_active = self._capture_runtime_owner().stop()
        if hasattr(self.panel, "capture_calibration_auto_button"):
            self.panel.capture_calibration_auto_button.setText("자동 촬영 시작")
        if message is not None:
            self.panel.calibration_status_label.setText(message)
        elif was_active:
            self.panel.calibration_status_label.setText(
                f"자동 촬영 중지: 유효 stereo pair {self._auto_capture_saved_pairs}개 저장"
            )
        return True

    def _auto_calibration_capture_tick(self):
        if not self._auto_capture_active:
            return
        self._capture_runtime_owner().tick(self._capture_live_inputs())

    def _handle_checkerboard_capture_completed(self, result: object) -> None:
        if not self._has_admitted_calibration_job():
            return self._apply_owned_checkerboard_capture_completed(result)

    def _apply_owned_checkerboard_capture_completed(self, result: object, *, source_root=None) -> None:
        capture_origin = self._capture_origin
        source_label = {
            "ftp": "FTP HR",
            "ftp_oz": "FTP OZ",
            "ftp_oz_sync": "FTP OZ sync",
        }.get(capture_origin, "")
        source_prefix = f"{source_label} " if source_label else ""
        if not isinstance(result, dict):
            self._apply_owned_checkerboard_capture_failed("worker returned an invalid capture result")
            return
        stem = str(result.get("stem") or "--")
        saved_sides = tuple(str(side) for side in result.get("saved_sides", ()) or ())
        status_parts = tuple(str(part) for part in result.get("status_parts", ()) or ())
        side_results = result.get("side_results")
        if isinstance(side_results, dict):
            for side in ("left", "right"):
                detail = side_results.get(side)
                if isinstance(detail, dict):
                    self._publish_calibration_capture_event(
                        {
                            "side": side,
                            "phase": "completed",
                            **detail,
                        },
                        operation={
                            "ftp": "Capture HR Still (FTP)",
                            "ftp_oz": "Capture OZ Still (FTP)",
                            "auto": "Auto capture",
                        }.get(capture_origin, "Capture Live Pair"),
                        capture_id=stem,
                    )
        if not saved_sides:
            self._capture_feedback_complete(
                capture_id=stem,
                origin=capture_origin,
                saved_sides=(),
                stereo_pair_saved=False,
            )
            status = f"Capture {stem}: no calibration images saved; " + ", ".join(status_parts)
            if capture_origin == "auto" and self._auto_capture_active:
                status = f"자동 촬영 대기: Capture {stem} 실패; 다음 새 검출 pair를 기다리는 중"
            self.panel.calibration_status_label.setText(status)
            self.panel._log(
                f"[캘리브레이션] {source_prefix}이미지 캡처 실패 {stem}: "
                f"{', '.join(status_parts)}"
            )
            if capture_origin == "auto":
                self._capture_runtime_owner().fail_auto()
            return
        status = f"Captured {stem}: " + ", ".join(status_parts)
        pair_delta_s = result.get("pair_delta_s")
        pair_saved = bool(
            result.get(
                "stereo_pair_saved",
                bool(result.get("pair_timing_ok"))
                and set(saved_sides) == {"left", "right"},
            )
        )
        intrinsics_only_sides = tuple(
            str(side)
            for side in result.get(
                "intrinsics_only_sides",
                () if pair_saved else saved_sides,
            )
            or ()
        )
        self._capture_feedback_complete(
            capture_id=stem,
            origin=capture_origin,
            saved_sides=saved_sides,
            stereo_pair_saved=pair_saved,
        )
        if (
            pair_saved
            and isinstance(pair_delta_s, (int, float))
        ):
            status += (
                f"; pair_time_basis={result.get('pair_time_basis', 'unknown')}, "
                f"pair_delta_s={float(pair_delta_s):.6f}"
            )
        elif intrinsics_only_sides:
            status += (
                f"; intrinsics-only={','.join(intrinsics_only_sides)}; "
                "excluded from stereo R,t"
            )
        self.panel.calibration_status_label.setText(status)
        session_root = source_root if source_root is not None else getattr(self.panel, "_calibration_session_root", None)
        if session_root is not None:
            try:
                CaptureDirectories.write_manifest(session_root, source=capture_origin)
            except OSError:
                pass
        self.panel._log(
            f"[캘리브레이션] {source_prefix}이미지 캡처 {stem}: "
            f"저장={','.join(saved_sides)}; {', '.join(status_parts)}"
        )
        if capture_origin == "auto":
            self._capture_runtime_owner().complete_auto(saved_sides, pair_saved)
            if self._auto_capture_active:
                if pair_saved:
                    self.panel.calibration_status_label.setText(
                        f"자동 촬영 대기: 유효 stereo pair "
                        f"{self._auto_capture_saved_pairs}개 저장됨 | Stop할 때까지 계속"
                    )
                else:
                    self.panel.calibration_status_label.setText(
                        f"자동 촬영 대기: Capture {stem} "
                        f"{','.join(intrinsics_only_sides)} intrinsics-only 저장 | "
                        f"유효 stereo pair {self._auto_capture_saved_pairs}개 | "
                        "Stop할 때까지 계속"
                    )
            elif saved_sides:
                usage = "stereo pair" if pair_saved else "intrinsics-only 이미지"
                self.panel.calibration_status_label.setText(
                    f"자동 촬영은 중지됨: 진행 중이던 {usage}는 파일로 저장됐지만 "
                    "중지된 자동 촬영 횟수에는 포함하지 않았습니다."
                )

    def _handle_checkerboard_capture_failed(self, error: str) -> None:
        if not self._has_admitted_calibration_job():
            return self._apply_owned_checkerboard_capture_failed(error)

    def _apply_owned_checkerboard_capture_failed(self, error: str) -> None:
        capture_origin = self._capture_origin
        self._capture_feedback_failed(origin=capture_origin)
        source_label = {
            "ftp": "FTP HR",
            "ftp_oz": "FTP OZ",
            "ftp_oz_sync": "FTP OZ sync",
        }.get(capture_origin, "")
        source_prefix = f"{source_label} " if source_label else ""
        operation = {
            "ftp": "Capture HR Still (FTP)",
            "ftp_oz": "Capture OZ Still (FTP)",
            "auto": "Auto capture",
        }.get(capture_origin, "Capture Live Pair")
        for side in ("left", "right"):
            self._publish_calibration_capture_event(
                {
                    "side": side,
                    "phase": "failed",
                    "state": "failed",
                    "response": str(error),
                },
                operation=operation,
            )
        if self._capture_origin == "auto" and self._auto_capture_active:
            self.panel.calibration_status_label.setText(f"자동 촬영 대기: Capture worker 실패 ({error})")
        else:
            self.panel.calibration_status_label.setText(f"Calibration capture failed: {error}")
        self.panel._log(f"[캘리브레이션] {source_prefix}이미지 캡처 worker 실패: {error}")
        if self._capture_origin == "auto":
            self._capture_runtime_owner().fail_auto()

    def _calibration_input_snapshot(
        self,
        input_request: Mapping[str, object] | None,
        *,
        mode_override: str | None = None,
    ) -> tuple[CalibrationInputSnapshot, int, bool]:
        """Resolve one UI-requested root into the sole worker input identity."""

        request = dict(input_request or {})
        raw_root = str(request.get("root") or "").strip()
        if raw_root:
            root = resolve_settings_path(raw_root, base_dir=self.panel.app_root)
        else:
            root_reader = getattr(self.panel, "_checkerboard_validation_root", None)
            if callable(root_reader):
                root = Path(root_reader())
            else:
                root = Path(self.panel._checkerboard_validation_scan_counts()[0])
        root = root.resolve(strict=True)
        mode = str(
            mode_override
            or request.get("mode")
            or self.panel._checkerboard_validation_mode()
        )
        if mode not in {"pixhawk", "checkerboard"}:
            raise ValueError(f"unsupported calibration calculation mode: {mode}")
        pair_limit = max(
            0,
            int(
                request.get("pair_limit")
                if request.get("pair_limit") is not None
                else self._calibration_calculation_pair_limit()
            ),
        )
        return self._input_owner().snapshot(root, mode=mode, pair_limit=pair_limit)

    def run_intrinsic_calibration(
        self,
        input_request: Mapping[str, object] | None = None,
    ) -> None:
        self._ensure_calibration_session_root("calculation")
        if self._auto_capture_active:
            self.stop_auto_calibration_capture()
        owner = self._calibration_job_owner()
        if owner.closing or owner.busy:
            self.panel._set_health_label(
                self.panel.checkerboard_validation_status_label,
                "Calibration is already running",
                "warn",
            )
            return
        available, error = opencv_available()
        if not available:
            self.panel._set_health_label(self.panel.checkerboard_validation_status_label, f"OpenCV unavailable: {error}", "bad")
            self.panel.calibration_result_text.setPlainText(f"OpenCV unavailable:\n{error}")
            return
        try:
            input_snapshot, source_pair_count, explicit_selection = (
                self._calibration_input_snapshot(input_request)
            )
        except (OSError, TypeError, ValueError) as exc:
            self.panel._set_health_label(
                self.panel.checkerboard_validation_status_label,
                f"계산 입력 차단: {exc}",
                "bad",
            )
            return
        mode = input_snapshot.mode
        root = input_snapshot.root_path
        left_paths = list(input_snapshot.left_paths)
        right_paths = list(input_snapshot.right_paths)
        selected_pairs = list(input_snapshot.pair_paths)
        calculation_pair_limit = input_snapshot.pair_limit
        pair_count = len(selected_pairs)
        min_images = self.panel.calib_min_images_spin.value()
        stereo_paths = [
            path
            for _key, left_path, right_path in selected_pairs
            for path in (left_path, right_path)
        ]
        zoom_input_paths = list(dict.fromkeys([*left_paths, *right_paths, *stereo_paths]))
        capture_zoom_level, zoom_detail = self._capture_zoom_level_for_paths(
            root,
            zoom_input_paths,
        )
        if capture_zoom_level is None:
            self.panel._set_health_label(
                self.panel.checkerboard_validation_status_label,
                f"계산 실패: {zoom_detail}. 촬영 당시 양쪽 GZVV readback이 같은지 확인하고 다시 촬영하세요.",
                "bad",
            )
            self.panel._log(f"[캘리브레이션] 계산 중단: {zoom_detail}")
            return
        self.panel.scan_checkerboard_validation_files()
        selection_snapshot = {
            "root": root,
            "mode": mode,
            "left_intrinsic_paths": tuple(left_paths),
            "right_intrinsic_paths": tuple(right_paths),
            "stereo_pairs": tuple(selected_pairs),
            "explicit_selection": bool(explicit_selection),
            "calibration_input_snapshot": input_snapshot.to_mapping(),
            "calibration_input_snapshot_sha256": input_snapshot.snapshot_sha256,
        }
        worker_factory = lambda: CheckerboardCalibrationWorker(
            mode=mode,
            config=self.panel._checkerboard_config(),
            input_snapshot=input_snapshot,
        )
        job = self._prepare_calibration_job(
            "intrinsics", worker_factory=worker_factory,
            completed=lambda result,
            mode=mode,
            root=root,
            pair_count=pair_count,
            min_images=min_images,
            capture_zoom_level=capture_zoom_level,
            left_count=len(left_paths),
            right_count=len(right_paths),
            selection_snapshot=selection_snapshot: self._apply_owned_intrinsic_calibration_completed(
                result,
                mode=mode,
                root=root,
                pair_count=pair_count,
                min_images=min_images,
                capture_zoom_level=capture_zoom_level,
                left_count=left_count,
                right_count=right_count,
                selection_snapshot=selection_snapshot,
            ),
            failed=self._apply_owned_intrinsic_calibration_failed,
            progress=self._apply_owned_calibration_progress,
        )
        if job is None:
            return

        def prepare_ui():
            self.panel.capture_calibration_button.setEnabled(False)
            self.panel.run_calibration_button.setEnabled(False)
            self.panel.checkerboard_validation_random_button.setEnabled(False)
            self.panel._set_health_label(
                self.panel.checkerboard_validation_status_label,
                f"Calibration running: mode={mode}, calculation pairs={pair_count}/{source_pair_count}, "
                f"left={len(left_paths)}, right={len(right_paths)}",
                "warn",
            )
            self.panel.calibration_progress = {
                "phase": "queued",
                "processed": 0,
                "total": max(len(left_paths), len(right_paths), pair_count),
                "percent": 0.0,
                "message": (
                    f"계산 준비: {pair_count}/{source_pair_count} pair"
                    if calculation_pair_limit > 0 or mode == "checkerboard"
                    else f"계산 준비: left={len(left_paths)}, right={len(right_paths)}"
                ),
            }
        self._start_calibration_job(job, prepare_ui)

    def run_stereo_extrinsics_calibration(
        self,
        input_request: Mapping[str, object] | None = None,
    ) -> None:
        """Run only OpenCV stereo R/T with the selected K/D held fixed."""

        self._ensure_calibration_session_root("calculation")

        if self._auto_capture_active:
            self.stop_auto_calibration_capture()
        owner = self._calibration_job_owner()
        if owner.closing or owner.busy:
            self.panel._set_health_label(
                self.panel.checkerboard_validation_status_label,
                "Calibration is already running",
                "warn",
            )
            return
        intrinsics = self.panel.last_calibration_result
        if intrinsics is None:
            self.panel._set_health_label(
                self.panel.checkerboard_validation_status_label,
                "R,t only 계산 전에 source K,D profile을 계산하거나 불러오세요",
                "bad",
            )
            return
        available, error = opencv_available()
        if not available:
            self.panel._set_health_label(
                self.panel.checkerboard_validation_status_label,
                f"OpenCV unavailable: {error}",
                "bad",
            )
            return
        try:
            input_snapshot, _source_pair_count, _explicit_selection = (
                self._calibration_input_snapshot(
                    input_request,
                    mode_override="checkerboard",
                )
            )
        except (OSError, TypeError, ValueError) as exc:
            self.panel._set_health_label(
                self.panel.checkerboard_validation_status_label,
                f"R,t only 계산 입력 차단: {exc}",
                "bad",
            )
            return
        root = input_snapshot.root_path
        selected_pairs = list(input_snapshot.pair_paths)
        left_paths = [left for _key, left, _right in selected_pairs]
        right_paths = [right for _key, _left, right in selected_pairs]
        source_zoom, zoom_detail = self._capture_zoom_level_for_paths(
            root,
            [*left_paths, *right_paths],
        )
        if source_zoom is None:
            self.panel._set_health_label(
                self.panel.checkerboard_validation_status_label,
                f"R,t only 계산 실패: {zoom_detail}. 촬영 당시 양쪽 GZVV readback이 같은지 확인하고 다시 촬영하세요.",
                "bad",
            )
            self.panel._log(f"[캘리브레이션] R,t 계산 중단: {zoom_detail}")
            return
        source_path = str(
            getattr(self.panel, "_candidate_intrinsics_profile_path", "")
            or getattr(self.panel, "_active_intrinsics_profile_path", "")
            or ""
        )
        source_hash = intrinsics_content_sha256(intrinsics)
        worker_factory = lambda: CheckerboardExtrinsicsWorker(
            config=self.panel._checkerboard_config(),
            left_intrinsics=intrinsics.left,
            right_intrinsics=intrinsics.right,
            input_snapshot=input_snapshot,
        )
        job = self._prepare_calibration_job(
            "extrinsics", worker_factory=worker_factory,
            completed=lambda result,
            source_zoom=source_zoom,
            source_path=source_path,
            source_hash=source_hash,
            root=root,
            input_snapshot=input_snapshot: self._apply_owned_stereo_extrinsics_completed(
                result,
                source_zoom=source_zoom,
                source_path=source_path,
                source_hash=source_hash,
                root=root,
                input_snapshot=input_snapshot,
            ),
            failed=self._apply_owned_intrinsic_calibration_failed,
            progress=self._apply_owned_calibration_progress,
        )
        if job is None:
            return

        def prepare_ui():
            self.panel._set_health_label(
                self.panel.checkerboard_validation_status_label,
                f"R,t only 계산 중: fixed K,D zoom={source_zoom}, pairs={len(selected_pairs)}",
                "warn",
            )
        self._start_calibration_job(job, prepare_ui)

    def _handle_stereo_extrinsics_completed(self, result: object, *, source_zoom: int | None, source_path: str, source_hash: str, root: Path, input_snapshot: CalibrationInputSnapshot | None=None) -> None:
        if not self._has_admitted_calibration_job():
            return self._apply_owned_stereo_extrinsics_completed(result, source_zoom=source_zoom, source_path=source_path, source_hash=source_hash, root=root, input_snapshot=input_snapshot)

    def _apply_owned_stereo_extrinsics_completed(
        self,
        result: object,
        *,
        source_zoom: int | None,
        source_path: str,
        source_hash: str,
        root: Path,
        input_snapshot: CalibrationInputSnapshot | None = None,
    ) -> None:
        if not isinstance(result, StereoExtrinsics):
            self._apply_owned_intrinsic_calibration_failed(
                "worker returned invalid stereo extrinsics"
            )
            return
        profile = StereoExtrinsicsProfile(
            extrinsics=result,
            source_zoom_level=source_zoom,
            method="checkerboard_external_only",
            source_intrinsics_path=source_path,
            source_intrinsics_sha256=source_hash,
            profile_id=(
                f"extrinsics_zoom_{int(source_zoom):02d}_large_board"
                if source_zoom is not None
                else "extrinsics_large_board"
            ),
            metadata={
                "timestamp_s": time.time(),
                "calibration_root": str(root),
                "validation_managed_by_application": False,
                "calibration_input_snapshot": (
                    None if input_snapshot is None else input_snapshot.to_mapping()
                ),
                "calibration_input_snapshot_sha256": (
                    "" if input_snapshot is None else input_snapshot.snapshot_sha256
                ),
            },
        )
        self.panel.selected_extrinsics_profile = profile
        # Raw stereoCalibrate output remains preview-only until Apply promotes
        # it through the quality and live-grid gates.
        self.panel.checkerboard_extrinsics_candidate_profile = profile
        self.panel._selected_extrinsics_profile_path = (
            f"memory:{profile.profile_id}"
        )
        self.panel._selected_extrinsics_profile_hash = profile.content_sha256
        self.panel._set_health_label(
            self.panel.checkerboard_validation_status_label,
            f"R,t only candidate 준비됨: source zoom={source_zoom}, "
            f"pairs={result.pair_count}, baseline={profile.baseline_m:.6f} m",
            "good",
        )
        self.panel._log(
            "[캘리브레이션] 고정 K,D 기반 R,t 계산 완료: "
            f"source_zoom={source_zoom}, frame={result.frame_contract}"
        )
        self.panel._refresh_camera_parameters_tab()
        self.panel._refresh_operation_header()

    def save_stereo_extrinsics_profile(
        self,
        *,
        quality_warning_confirmed: bool = False,
    ) -> bool:
        self._ensure_calibration_session_root("calculation")
        profile = getattr(self.panel, "selected_extrinsics_profile", None)
        if not isinstance(profile, StereoExtrinsicsProfile):
            self.panel.calibration_status_label.setText(
                "Calculate or load an R,t profile before saving."
            )
            return False
        intrinsics_result = getattr(self.panel, "last_calibration_result", None)
        quality_state = "UNASSESSED"
        quality_reasons: tuple[str, ...] = ()
        if isinstance(intrinsics_result, StereoIntrinsicsResult):
            quality_state, quality_reasons = self._calibration_quality_state(intrinsics_result)
        output = self.panel._extrinsics_storage_path(profile.source_zoom_level)
        saved = self._profiles_owner().save_extrinsics_attempt(profile, output=output, quality_state=quality_state)
        output = saved.output
        self.panel._last_calibration_attempt_path = str(output)
        if saved.approved:
            immutable_output = self._publish_immutable_extrinsics_profile(profile)
            self.panel._selected_extrinsics_profile_path = str(immutable_output)
            self.panel._selected_extrinsics_profile_hash = profile.content_sha256
            message = (
                f"Saved approved stereo R,t profile: immutable={immutable_output} | "
                f"alias={output}"
            )
        else:
            message = (
                f"Saved non-applicable stereo R,t attempt: {quality_state} "
                f"({', '.join(quality_reasons)}) | {output}"
            )
        self.panel.calibration_status_label.setText(message)
        self.panel._log(f"[캘리브레이션] {message}")
        self.panel._refresh_operation_header()
        return True

    def load_stereo_extrinsics_profile(self) -> None:
        initial_path = self.panel._extrinsics_storage_path()
        selected, _selected_filter = QtWidgets.QFileDialog.getOpenFileName(
            self.panel,
            "Load stereo R,t profile",
            str(initial_path if initial_path.exists() else initial_path.parent),
            "Stereo extrinsics JSON (*.json);;All files (*)",
        )
        if not selected:
            return
        path = Path(selected)
        try:
            profile = self._profiles_owner().load_extrinsics(path)
        except Exception as exc:
            self.panel.calibration_status_label.setText(f"Failed to load stereo R,t: {exc}")
            self.panel._log(f"[캘리브레이션] R,t profile 로드 실패: {path}: {exc}")
            return
        self.panel.selected_extrinsics_profile = profile
        # A loaded profile is still a candidate. Route it only into a preview
        # slot; Apply is the sole promotion into Measurement-ready state.
        if str(profile.method or "").startswith("optical_pose/"):
            self.panel.optical_pose_extrinsics_candidate_profile = profile
        else:
            self.panel.checkerboard_extrinsics_candidate_profile = profile
        self.panel._selected_extrinsics_profile_path = str(path)
        self.panel._selected_extrinsics_profile_hash = profile.content_sha256
        self.panel.calibration_status_label.setText(
            f"Loaded stereo R,t candidate: {path.name} (source zoom={profile.source_zoom_level})"
        )
        self.panel._log(
            f"[캘리브레이션] 독립 R,t profile 로드: {path}; "
            "cross-zoom 조합은 검증 gate 없이 허용"
        )
        self.panel._refresh_camera_parameters_tab()
        self.panel._refresh_operation_header()

    def apply_calibration_bundle(
        self,
        *,
        quality_warning_confirmed: bool = False,
    ) -> bool:
        """Apply independent K/D and R/T after the camera-quality gate.

        Cross-zoom validation remains informational; this confirmation covers
        only the source-backed camera calibration WARNING state.
        """

        applied = (
            self.panel.apply_intrinsics_to_distance(quality_warning_confirmed=True)
            if quality_warning_confirmed
            else self.panel.apply_intrinsics_to_distance()
        )
        return False if applied is False else True

    def _handle_calibration_progress(self, progress: object) -> None:
        if not self._has_admitted_calibration_job():
            return self._apply_owned_calibration_progress(progress)

    def _apply_owned_calibration_progress(self, progress: object) -> None:
        if not isinstance(progress, dict):
            return
        payload = dict(progress)
        self.panel.calibration_progress = payload
        message = str(payload.get("message") or payload.get("phase") or "계산 중")
        try:
            percent = max(0.0, min(100.0, float(payload.get("percent", 0.0))))
        except (TypeError, ValueError):
            percent = 0.0
        text = f"계산 중: {message} ({percent:.0f}%)"
        if hasattr(self.panel, "checkerboard_validation_status_label"):
            self.panel._set_health_label(
                self.panel.checkerboard_validation_status_label,
                text,
                "warn",
            )
        if hasattr(self.panel, "calibration_status_label"):
            self.panel.calibration_status_label.setText(text)
        if hasattr(self.panel, "calibration_progress_bar"):
            self.panel.calibration_progress_bar.setValue(round(percent))
            self.panel.calibration_progress_bar.setFormat(f"{message} · %p%")

    def _handle_intrinsic_calibration_completed(self, result: object, *, mode: str, root: Path, pair_count: int, min_images: int, capture_zoom_level: int, left_count: int, right_count: int, selection_snapshot: dict[str, object] | None=None) -> None:
        if not self._has_admitted_calibration_job():
            return self._apply_owned_intrinsic_calibration_completed(result, mode=mode, root=root, pair_count=pair_count, min_images=min_images, capture_zoom_level=capture_zoom_level, left_count=left_count, right_count=right_count, selection_snapshot=selection_snapshot)

    def _apply_owned_intrinsic_calibration_completed(
        self,
        result: object,
        *,
        mode: str,
        root: Path,
        pair_count: int,
        min_images: int,
        capture_zoom_level: int,
        left_count: int,
        right_count: int,
        selection_snapshot: dict[str, object] | None = None,
    ) -> None:
        if not isinstance(result, StereoIntrinsicsResult):
            self._apply_owned_intrinsic_calibration_failed("worker returned an invalid calibration result")
            return
        assessment = CalibrationQuality.assess_minimum_effective_poses(result, min_images)
        result = assessment.result
        warning = (
            f"\nWARNING: valid images below requested minimum {min_images}."
            if assessment.below_minimum else ""
        )
        effective_pair_count = pair_count
        solver_effective = self._solver_effective_selection(result)
        if solver_effective is not None:
            effective_left, effective_right, effective_pairs = solver_effective
            effective_pair_count = len(effective_pairs)
            if selection_snapshot is not None:
                selection_snapshot = {
                    **dict(selection_snapshot),
                    "left_intrinsic_paths": effective_left,
                    "right_intrinsic_paths": effective_right,
                    "stereo_pairs": effective_pairs,
                    "input_left_intrinsic_paths": tuple(
                        selection_snapshot.get("left_intrinsic_paths", ())
                    ),
                    "input_right_intrinsic_paths": tuple(
                        selection_snapshot.get("right_intrinsic_paths", ())
                    ),
                    "input_stereo_pairs": tuple(selection_snapshot.get("stereo_pairs", ())),
                }
        text = self.panel._format_checkerboard_validation_result_summary(
            result,
            mode=mode,
            title="Full checkerboard calibration result",
            root=root,
            pair_count=effective_pair_count if mode == "checkerboard" else None,
        ) + warning
        self.panel._checkerboard_validation_set_result(
            result,
            mode=mode,
            summary={
                "source": "full",
                "root": str(root),
                "mode": mode,
                "capture_zoom_level": capture_zoom_level,
                "left_image_count": left_count,
                "right_image_count": right_count,
                "left_effective_image_count": result.left.image_count,
                "right_effective_image_count": result.right.image_count,
                "stereo_input_pair_count": pair_count,
                "stereo_pair_count": effective_pair_count,
                "min_images": min_images,
            },
            text=text,
            zoom_level=capture_zoom_level,
            selection_snapshot=selection_snapshot,
        )
        if mode == "checkerboard":
            self.panel._log("[캘리브레이션] 전체 사진 기반 intrinsic 및 체커보드 stereo 외부 파라미터 계산을 완료했습니다.")
        else:
            self.panel._log("[캘리브레이션] 전체 사진 기반 Pixhawk 모드용 intrinsic 계산을 완료했습니다.")
        self.panel._refresh_operation_header()
        self.panel.calibration_progress = {
            "phase": "complete",
            "processed": 1,
            "total": 1,
            "percent": 100.0,
            "message": "계산 완료",
        }
        if hasattr(self.panel, "calibration_progress_bar"):
            self.panel.calibration_progress_bar.setValue(100)
            self.panel.calibration_progress_bar.setFormat("계산 완료 · %p%")

    def _handle_intrinsic_calibration_failed(self, error: str) -> None:
        if not self._has_admitted_calibration_job():
            return self._apply_owned_intrinsic_calibration_failed(error)

    def _apply_owned_intrinsic_calibration_failed(self, error: str) -> None:
        self.panel.calibration_progress = {
            "phase": "failed",
            "processed": 0,
            "total": 0,
            "percent": 0.0,
            "message": str(error),
        }
        if hasattr(self.panel, "calibration_progress_bar"):
            self.panel.calibration_progress_bar.setValue(0)
            self.panel.calibration_progress_bar.setFormat("계산 실패 · %p%")
        self.panel._set_health_label(
            self.panel.checkerboard_validation_status_label,
            f"Calibration failed: {error}",
            "bad",
        )
        self.panel.calibration_result_text.setPlainText(str(error))
        self.panel._log(f"[캘리브레이션] 검증/계산 실패: {error}")

    def _clear_checkerboard_calibration_worker(self, thread: QtCore.QThread) -> None:
        if self._has_admitted_calibration_job():
            # Only the ordered GUI finished relay can release an admitted
            # slot; physical thread exit alone cannot discard queued results.
            return
        if self.panel.checkerboard_calibration_thread is thread:
            self.panel.checkerboard_calibration_thread = None
            self.panel.checkerboard_calibration_worker = None
            self._finish_checkerboard_calibration_job()
        thread.deleteLater()

    def _finish_checkerboard_calibration_job(self) -> None:
        if hasattr(self.panel, "capture_calibration_button"):
            self.panel.capture_calibration_button.setEnabled(True)
        if hasattr(self.panel, "run_calibration_button"):
            self.panel.run_calibration_button.setEnabled(True)
        if hasattr(self.panel, "checkerboard_validation_random_button"):
            self.panel.checkerboard_validation_random_button.setEnabled(
                self.panel.checkerboard_validation_result is not None
            )
