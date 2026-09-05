"""Workstation checkerboard calibration workflow controller.

Image observations and calibration matrices retain OpenCV optical RDF
(right, down, forward) conventions.  Stereo ``T`` follows
``X_right = R_left_to_right * X_left + T_left_to_right_m``; the right-camera
origin expressed in left RDF is the distinct value ``-R^T T``.  This module
performs no NED/ENU aliasing.
"""

from __future__ import annotations

import json
import hashlib
import math
import os
import time
import uuid
from dataclasses import replace
from pathlib import Path
from typing import Mapping

from PyQt5 import QtCore, QtGui, QtWidgets

from Workstation.runtime_paths import portable_workstation_data_path_text
from Workstation.calibration_workers import (
    CheckerboardCalibrationWorker,
    CheckerboardCaptureWorker,
    CheckerboardExtrinsicsWorker,
    CheckerboardRandomValidationWorker,
)
from Workstation.calibration_sessions import (
    scan_calibration_capture_inventory,
    write_calibration_session_manifest,
)
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
from Workstation.wiris_runtime_workers import (
    WirisFtpBatchDeleteWorker,
    WirisImageFormatWorker,
    WirisOzFtpSyncWorker,
    WirisOZPhotoCaptureWorker,
    WirisPnPPhotoCaptureWorker,
)

CHECKERBOARD_STEREO_PAIR_MAX_DELTA_S = 0.100
CALIBRATION_AUTO_DETECTION_MAX_AGE_S = 2.0
_USE_SELECTED_EXTRINSICS = object()


def is_nal1_non_idr_auto_capture_metadata(metadata: object) -> bool:
    if not isinstance(metadata, Mapping):
        return False
    nal_types = metadata.get("h264_access_unit_nal_types")
    if not isinstance(nal_types, (list, tuple)):
        return False
    try:
        normalized_nal_types = {int(value) for value in nal_types}
        pts_ns = int(metadata["h264_access_unit_pts_ns"])
        decoded_pts_ns = int(metadata["decoded_buffer_pts_ns"])
    except (KeyError, TypeError, ValueError, OverflowError):
        return False
    return bool(
        metadata.get("h264_access_unit_kind") == "nal1_non_idr_intra"
        and metadata.get("h264_nal1_non_idr_intra") is True
        and metadata.get("h264_access_unit_is_delta") is False
        and metadata.get("h264_random_access_guaranteed") is False
        and pts_ns >= 0
        and decoded_pts_ns == pts_ns
        and 1 in normalized_nal_types
        and 5 not in normalized_nal_types
    )


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


class CalibrationWorkflowController:
    def __init__(self, panel):
        self.panel = panel
        timer_parent = panel if isinstance(panel, QtCore.QObject) else None
        self._auto_capture_timer = QtCore.QTimer(timer_parent)
        self._auto_capture_timer.timeout.connect(self._auto_calibration_capture_tick)
        self._auto_capture_active = False
        self._auto_capture_saved_pairs = 0
        self._auto_capture_last_pair_key: tuple[int, int] | None = None
        self._auto_capture_inflight_pair_key: tuple[int, int] | None = None
        self._auto_capture_last_attempt_s = 0.0
        self._capture_origin = "manual"
        self._capture_feedback_root_text = ""
        self._capture_feedback_run = self._empty_capture_feedback_run()
        self._capture_feedback_last: dict[str, object] = {
            "state": "IDLE",
            "capture_id": "",
            "origin": "",
        }
        self._capture_feedback_inventory_stamp: tuple[object, ...] | None = None
        self._capture_feedback_inventory: dict[str, object] | None = None

    @staticmethod
    def _empty_capture_feedback_run() -> dict[str, int]:
        return {
            "inspection_request_count": 0,
            "stereo_saved_count": 0,
            "intrinsics_only_saved_count": 0,
            "failed_count": 0,
            "inflight_count": 0,
        }

    def _ensure_capture_feedback_state(self) -> None:
        """Keep legacy/lightweight controller construction compatible."""

        if not isinstance(getattr(self, "_capture_feedback_run", None), dict):
            self._capture_feedback_run = self._empty_capture_feedback_run()
        if not isinstance(getattr(self, "_capture_feedback_last", None), dict):
            self._capture_feedback_last = {
                "state": "IDLE",
                "capture_id": "",
                "origin": "",
            }
        if not hasattr(self, "_capture_feedback_root_text"):
            self._capture_feedback_root_text = ""
        if not hasattr(self, "_capture_feedback_inventory_stamp"):
            self._capture_feedback_inventory_stamp = None
        if not hasattr(self, "_capture_feedback_inventory"):
            self._capture_feedback_inventory = None

    @staticmethod
    def _capture_feedback_root_id(root: Path) -> str:
        if root.name == "AutoCapture" and root.parent.name == "calib_capture":
            return f"{root.parent.parent.name}/AutoCapture"
        return root.name or str(root)

    def _capture_feedback_prepare_root(self, root: str | Path) -> Path:
        self._ensure_capture_feedback_state()
        resolved = Path(root).expanduser().resolve(strict=False)
        root_text = str(resolved)
        if root_text != self._capture_feedback_root_text:
            self._capture_feedback_root_text = root_text
            self._capture_feedback_run = self._empty_capture_feedback_run()
            self._capture_feedback_inventory_stamp = None
            self._capture_feedback_inventory = None
            self._capture_feedback_last = {
                "state": "IDLE",
                "capture_id": "",
                "origin": "",
            }
        return resolved

    def _capture_feedback_reset_run(self, *, origin: str) -> None:
        """Start a new visible run without changing persisted capture inventory."""

        self._ensure_capture_feedback_state()
        self._capture_feedback_run = self._empty_capture_feedback_run()
        self._capture_feedback_last = {
            "state": "WAITING",
            "capture_id": "",
            "origin": str(origin),
        }

    def _capture_feedback_begin(
        self,
        *,
        root: str | Path,
        capture_id: str,
        origin: str,
    ) -> None:
        self._capture_feedback_prepare_root(root)
        self._capture_feedback_run["inspection_request_count"] += 1
        self._capture_feedback_run["inflight_count"] += 1
        self._capture_feedback_last = {
            "state": "INFLIGHT",
            "capture_id": str(capture_id),
            "origin": str(origin),
        }

    def _capture_feedback_complete(
        self,
        *,
        capture_id: str,
        origin: str,
        saved_sides: tuple[str, ...],
        stereo_pair_saved: bool,
    ) -> None:
        self._ensure_capture_feedback_state()
        counted = self._capture_feedback_run["inflight_count"] > 0
        if counted:
            self._capture_feedback_run["inflight_count"] -= 1
            if stereo_pair_saved:
                self._capture_feedback_run["stereo_saved_count"] += 1
                state = "STEREO_SAVED"
            elif saved_sides:
                self._capture_feedback_run["intrinsics_only_saved_count"] += 1
                state = "INTRINSICS_ONLY_SAVED"
            else:
                self._capture_feedback_run["failed_count"] += 1
                state = "FAILED"
        else:
            state = "LATE_RESULT"
        self._capture_feedback_last = {
            "state": state,
            "capture_id": str(capture_id),
            "origin": str(origin),
        }
        self._capture_feedback_inventory_stamp = None

    def _capture_feedback_failed(self, *, origin: str) -> None:
        self._ensure_capture_feedback_state()
        counted = self._capture_feedback_run["inflight_count"] > 0
        if counted:
            self._capture_feedback_run["inflight_count"] -= 1
            self._capture_feedback_run["failed_count"] += 1
        self._capture_feedback_last = {
            "state": "FAILED" if counted else "LATE_FAILURE",
            "capture_id": str(self._capture_feedback_last.get("capture_id") or ""),
            "origin": str(origin),
        }
        self._capture_feedback_inventory_stamp = None

    @staticmethod
    def _capture_inventory_stamp(root: Path) -> tuple[object, ...]:
        stamp: list[object] = [str(root)]
        for path in (root, root / "left", root / "right", root / "capture_metadata"):
            try:
                stat = path.stat()
            except FileNotFoundError:
                stamp.append((str(path), "missing"))
            except OSError as exc:
                stamp.append((str(path), "error", type(exc).__name__, str(exc)))
            else:
                stamp.append(
                    (
                        str(path),
                        "dir" if path.is_dir() else "file",
                        int(stat.st_mtime_ns),
                        int(stat.st_size),
                    )
                )
        return tuple(stamp)

    def _capture_feedback_inventory_for(self, root: Path) -> dict[str, object]:
        stamp = self._capture_inventory_stamp(root)
        if (
            self._capture_feedback_inventory is None
            or self._capture_feedback_inventory_stamp != stamp
        ):
            self._capture_feedback_inventory = scan_calibration_capture_inventory(
                root
            ).to_mapping()
            self._capture_feedback_inventory_stamp = stamp
        return dict(self._capture_feedback_inventory)

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

    @staticmethod
    def _zoom_index(value: object) -> int | None:
        try:
            level = int(value)
        except (TypeError, ValueError):
            return None
        return level if level >= 0 else None

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

    def _capture_zoom_level_for_paths(
        self,
        root: Path,
        paths: list[Path] | tuple[Path, ...],
    ) -> tuple[int | None, str]:
        """Resolve one common zoom from the capture metadata selected for a run."""

        capture_ids = {
            path.stem.split("_", 1)[1]
            for path in paths
            if "_" in path.stem
        }
        if not capture_ids:
            return None, "capture IDs are missing from the calibration image names"
        levels: dict[str, int] = {}
        missing: list[str] = []
        metadata_root = root / "capture_metadata"
        for capture_id in sorted(capture_ids):
            metadata_path = metadata_root / f"capture_{capture_id}.json"
            try:
                payload = json.loads(metadata_path.read_text(encoding="utf-8"))
            except (OSError, ValueError, TypeError):
                missing.append(capture_id)
                continue
            level = self._zoom_index(payload.get("capture_zoom_level"))
            if level is None:
                left_level = self._zoom_index(payload.get("capture_left_zoom_level"))
                right_level = self._zoom_index(payload.get("capture_right_zoom_level"))
                if left_level is not None and left_level == right_level:
                    level = left_level
            if level is None:
                missing.append(capture_id)
            else:
                levels[capture_id] = level
        if missing:
            return (
                None,
                "capture-time zoom readback is missing for "
                + ", ".join(missing[:5])
                + (" …" if len(missing) > 5 else ""),
            )
        unique_levels = sorted(set(levels.values()))
        if len(unique_levels) != 1:
            return None, f"mixed capture zoom levels: {', '.join(str(level) for level in unique_levels)}"
        return unique_levels[0], f"capture-time zoom={unique_levels[0]}"

    def _calibration_image_paths_for_root(self, root: Path, side: str) -> list[Path]:
        directory = root / side
        if not directory.is_dir():
            return []
        return sorted(
            path
            for path in directory.iterdir()
            if path.is_file() and path.suffix.lower() in CALIBRATION_IMAGE_SUFFIXES
        )

    def _calibration_stereo_pair_paths_for_root(self, root: Path) -> list[tuple[str, Path, Path]]:
        def indexed(paths: list[Path], side: str) -> dict[str, Path]:
            result: dict[str, Path] = {}
            for path in paths:
                suffix = path.stem.replace(f"{side}_", "", 1)
                result[suffix] = path
            return result

        left_by_id = indexed(self._calibration_image_paths_for_root(root, "left"), "left")
        right_by_id = indexed(self._calibration_image_paths_for_root(root, "right"), "right")
        common_keys = sorted(set(left_by_id) & set(right_by_id))
        if common_keys:
            return [
                (key, left_by_id[key], right_by_id[key])
                for key in common_keys
                if self._calibration_pair_stereo_eligible(
                    root,
                    left_by_id[key],
                    right_by_id[key],
                )
            ]
        left_paths = self._calibration_image_paths_for_root(root, "left")
        right_paths = self._calibration_image_paths_for_root(root, "right")
        if len(left_paths) == len(right_paths):
            return [
                (f"ordered-{index:06d}", left, right)
                for index, (left, right) in enumerate(zip(left_paths, right_paths, strict=False))
            ]
        return []

    @staticmethod
    def _calibration_pair_stereo_eligible(
        root: Path,
        left_path: Path,
        right_path: Path,
    ) -> bool:
        left_key = left_path.stem.removeprefix("left_")
        right_key = right_path.stem.removeprefix("right_")
        left_usage = load_calibration_pair_usage(
            Path(root) / "capture_metadata" / f"capture_{left_key}.json"
        )
        if not left_usage.stereo_eligible:
            return False
        if left_key == right_key:
            return True
        right_usage = load_calibration_pair_usage(
            Path(root) / "capture_metadata" / f"capture_{right_key}.json"
        )
        return right_usage.stereo_eligible

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

    def _checkerboard_validation_set_result(
        self,
        result: StereoIntrinsicsResult,
        *,
        mode: str,
        summary: dict[str, object],
        text: str,
        zoom_level: int | None = None,
        selection_snapshot: dict[str, object] | None = None,
    ) -> None:
        quality_snapshot = (
            dict(result.calibration_quality)
            if isinstance(result.calibration_quality, dict)
            else {
                "schema": "workstation-camera-calibration-quality-v1",
                "state": "UNVERIFIED",
                "source": "legacy/no quality evidence",
                "reason_codes": ["quality_snapshot_unavailable"],
                "reasons": ["This result predates source-backed calibration quality checks."],
            }
        )
        if result.calibration_quality is None:
            result = replace(result, calibration_quality=quality_snapshot)
        self.panel.calibration_quality = quality_snapshot
        self.panel.last_calibration_quality = quality_snapshot
        self.panel.checkerboard_validation_result = result
        self.panel.checkerboard_validation_result_mode = mode
        self.panel.checkerboard_validation_summary = {
            **dict(summary),
            "calibration_quality": quality_snapshot,
        }
        extrinsics = getattr(result, "extrinsics", None)
        pair_count = int(
            summary.get("stereo_pair_count")
            or getattr(extrinsics, "pair_count", 0)
            or 0
        )
        self.panel.checkerboard_validation_run_results = [
            self.panel._checkerboard_validation_result_data(
                "Full",
                result,
                result,
                pair_count,
            )
        ]
        self.panel.checkerboard_validation_result_saved = False
        self.panel.last_calibration_result = result
        if selection_snapshot is None:
            self.panel.checkerboard_calibration_selection_snapshot = None
        else:
            effective_selection = dict(selection_snapshot)
            effective_selection["reference_result"] = result
            self.panel.checkerboard_calibration_selection_snapshot = effective_selection
        self.panel.calibration_image_diagnostics = tuple(result.image_diagnostics)
        self.panel.calibration_diagnostics_root = str(
            self.panel._calibration_dirs()[0]
        )
        diagnostics_signal = getattr(self.panel, "calibration_diagnostics_changed", None)
        diagnostics_emit = getattr(diagnostics_signal, "emit", None)
        if callable(diagnostics_emit):
            diagnostics_emit(tuple(result.image_diagnostics))
        if zoom_level is not None:
            self.panel.last_calibration_zoom_level = int(zoom_level)
        else:
            # A newly loaded/calculated result owns its zoom provenance.  Do
            # not inherit either the current spinner or a previous result's
            # capture zoom when this result has no readback evidence.
            self.panel.last_calibration_zoom_level = None
        self.panel._candidate_intrinsics_profile_path = (
            f"memory:checkerboard-validation:{mode}:zoom-"
            f"{self.panel.last_calibration_zoom_level:02d}"
            if self.panel.last_calibration_zoom_level is not None
            else f"memory:checkerboard-validation:{mode}:zoom-unknown"
        )
        self.panel.calibration_result_text.setPlainText(text)
        extrinsics_state = "with checkerboard stereo R/T" if result.extrinsics is not None else "intrinsics only"
        quality_state = str(quality_snapshot.get("state") or "UNVERIFIED").upper()
        self.panel._set_health_label(
            self.panel.checkerboard_validation_status_label,
            f"계산 결과 있음: {mode} | {extrinsics_state} | quality={quality_state} | 저장 전",
            {"PASS": "good", "FAIL": "bad"}.get(quality_state, "warn"),
        )
        self.panel._refresh_camera_parameters_tab()
        self.panel._refresh_zoom_intrinsics_tab()

    def _checkerboard_result_baseline_m(self, result: StereoIntrinsicsResult) -> float | None:
        extrinsics = result.extrinsics
        if extrinsics is None:
            return None
        return math.sqrt(sum(value * value for value in extrinsics.translation_left_to_right_m))

    def _rotation_delta_deg_from_matrices(
        self,
        reference: list[list[float]] | None,
        current: list[list[float]] | None,
    ) -> float | None:
        if reference is None or current is None:
            return None
        try:
            trace = sum(
                float(current[row][col]) * float(reference[row][col])
                for row in range(3)
                for col in range(3)
            )
        except (TypeError, ValueError, IndexError):
            return None
        ratio = max(-1.0, min(1.0, (trace - 1.0) / 2.0))
        return math.degrees(math.acos(ratio))

    def _format_checkerboard_validation_result_summary(
        self,
        result: StereoIntrinsicsResult,
        *,
        mode: str,
        title: str,
        root: Path | None = None,
        pair_count: int | None = None,
    ) -> str:
        baseline_m = self.panel._checkerboard_result_baseline_m(result)
        extrinsics = result.extrinsics
        lines = [
            title,
            f"mode: {mode}",
            f"root: {root}" if root is not None else "",
            f"Left: images={result.left.image_count}, error={result.left.reprojection_error_px:.3f}px, "
            f"fx={result.left.camera_matrix[0][0]:.3f}, fy={result.left.camera_matrix[1][1]:.3f}, "
            f"cx={result.left.camera_matrix[0][2]:.3f}, cy={result.left.camera_matrix[1][2]:.3f}",
            f"Right: images={result.right.image_count}, error={result.right.reprojection_error_px:.3f}px, "
            f"fx={result.right.camera_matrix[0][0]:.3f}, fy={result.right.camera_matrix[1][1]:.3f}, "
            f"cx={result.right.camera_matrix[0][2]:.3f}, cy={result.right.camera_matrix[1][2]:.3f}",
        ]
        if extrinsics is None:
            lines.append("Extrinsics: not computed (Pixhawk mode uses live attitude + GPS/RTK baseline)")
        else:
            lines.extend(
                [
                    f"Stereo: pairs={extrinsics.pair_count}, error={extrinsics.reprojection_error_px:.3f}px, "
                    f"|T|={baseline_m:.6f} m",
                    f"T_left_to_right_m={extrinsics.translation_left_to_right_m}",
                    f"R_left_to_right={extrinsics.rotation_left_to_right}",
                ]
            )
        if pair_count is not None:
            lines.append(f"selected stereo pairs: {pair_count}")
        quality = result.calibration_quality
        if isinstance(quality, dict):
            lines.extend(
                (
                    f"Calibration quality: {quality.get('state', 'UNVERIFIED')}",
                    "Quality reasons: "
                    + ", ".join(str(code) for code in quality.get("reason_codes", ()) or ())
                    if quality.get("reason_codes")
                    else "Quality reasons: none",
                    f"Effective poses: {quality.get('effective_pose_count', '--')}",
                    f"Principal offsets: {quality.get('principal_point_offsets_px', '--')}",
                    f"Subset stability: {quality.get('subset_stability', '--')}",
                )
            )
        return "\n".join(line for line in lines if line)

    def load_checkerboard_validation_json(self) -> None:
        path_text = self.panel.checkerboard_validation_json_edit.text().strip()
        if not path_text:
            self.panel._set_health_label(self.panel.checkerboard_validation_status_label, "JSON 경로가 비어 있습니다", "bad")
            return
        path = resolve_settings_path(path_text, base_dir=self.panel.app_root)
        try:
            result = load_intrinsics_json(path)
        except Exception as exc:
            self.panel._set_health_label(self.panel.checkerboard_validation_status_label, f"JSON 불러오기 실패: {exc}", "bad")
            return
        try:
            source_payload = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, ValueError, TypeError):
            source_payload = {}
        try:
            archived_path = self._archive_loaded_intrinsics(path)
        except Exception as exc:
            self.panel._set_health_label(
                self.panel.checkerboard_validation_status_label,
                f"JSON Session 보관 실패: {exc}",
                "bad",
            )
            return
        source_metadata = (
            source_payload.get("metadata")
            if isinstance(source_payload, dict)
            and isinstance(source_payload.get("metadata"), dict)
            else {}
        )
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

    def _select_calibration_pairs(
        self,
        pairs: list[tuple[str, Path, Path]],
        *,
        pair_limit: int | None = None,
    ) -> list[tuple[str, Path, Path]]:
        """Select evenly distributed capture pairs for calibration input.

        A deterministic spread keeps early/late board poses represented instead
        of silently taking the first N captures, while zero means all pairs.
        """

        candidates = list(pairs)
        limit = (
            self._calibration_calculation_pair_limit()
            if pair_limit is None
            else max(0, int(pair_limit))
        )
        if limit <= 0 or limit >= len(candidates):
            return candidates
        limit = max(3, limit)
        if limit >= len(candidates):
            return candidates
        indices = sorted(
            {
                round(index * (len(candidates) - 1) / (limit - 1))
                for index in range(limit)
            }
        )
        return [candidates[index] for index in indices]

    @staticmethod
    def _select_calibration_image_paths(
        paths: list[Path],
        limit: int,
    ) -> list[Path]:
        """Evenly sample one camera's files when no stereo IDs are available."""

        candidates = list(paths)
        if limit <= 0 or limit >= len(candidates):
            return candidates
        limit = max(3, limit)
        if limit >= len(candidates):
            return candidates
        indices = sorted(
            {
                round(index * (len(candidates) - 1) / (limit - 1))
                for index in range(limit)
            }
        )
        return [candidates[index] for index in indices]

    def _checkerboard_intrinsic_delta_px(self, result: StereoIntrinsicsResult, reference: StereoIntrinsicsResult) -> tuple[float, float]:
        left_delta = max(
            abs(float(result.left.camera_matrix[row][col]) - float(reference.left.camera_matrix[row][col]))
            for row, col in ((0, 0), (1, 1), (0, 2), (1, 2))
        )
        right_delta = max(
            abs(float(result.right.camera_matrix[row][col]) - float(reference.right.camera_matrix[row][col]))
            for row, col in ((0, 0), (1, 1), (0, 2), (1, 2))
        )
        return left_delta, right_delta

    def _checkerboard_validation_stddev(self, values: list[float]) -> float:
        if len(values) < 2:
            return 0.0
        mean = sum(values) / len(values)
        return math.sqrt(sum((value - mean) ** 2 for value in values) / (len(values) - 1))

    @staticmethod
    def _calibration_quality_mapping(value: object) -> dict[str, object] | None:
        if value is None:
            return None
        if isinstance(value, dict):
            return dict(value)
        to_mapping = getattr(value, "to_mapping", None)
        if callable(to_mapping):
            try:
                payload = to_mapping()
            except Exception:
                payload = None
            if isinstance(payload, dict):
                return dict(payload)
        state = str(getattr(value, "state", "") or "").strip().upper()
        if not state:
            return None
        return {
            "state": state,
            "reason_codes": tuple(getattr(value, "reason_codes", ()) or ()),
        }

    def _calibration_quality_state(
        self,
        result: StereoIntrinsicsResult,
    ) -> tuple[str, tuple[str, ...]]:
        """Read quality output without turning missing assessment into success."""

        containers: list[object] = [result, getattr(result, "calibration_quality", None)]
        result_has_quality = any(
            value is not None
            for value in (
                getattr(result, "calibration_quality", None),
                getattr(result, "left_quality", None),
                getattr(result, "right_quality", None),
            )
        )
        # A result-owned snapshot is authoritative. Panel-level attributes are
        # compatibility fallbacks for older integrations and may refer to a
        # previously displayed result.
        if not result_has_quality:
            for name in (
                "checkerboard_calibration_quality",
                "calibration_quality_snapshot",
                "checkerboard_quality_snapshot",
            ):
                value = getattr(self.panel, name, None)
                if callable(value):
                    try:
                        value = value()
                    except Exception:
                        value = None
                if value is not None:
                    containers.append(value)

        side_states: dict[str, str] = {}
        overall_states: list[str] = []
        reason_codes: list[str] = []
        for container in containers:
            container_mapping = self._calibration_quality_mapping(container)
            if container_mapping is not None:
                state = str(container_mapping.get("state") or "").strip().upper()
                if state in {"PASS", "WARNING", "FAIL"}:
                    overall_states.append(state)
                reason_codes.extend(
                    str(code)
                    for code in container_mapping.get("reason_codes", ()) or ()
                    if str(code)
                )
            for side in ("left", "right"):
                key = f"{side}_quality"
                value = container.get(key) if isinstance(container, dict) else getattr(container, key, None)
                mapping = self._calibration_quality_mapping(value)
                if mapping is None:
                    continue
                state = str(mapping.get("state") or "").strip().upper()
                if state in {"PASS", "WARNING", "FAIL"}:
                    side_states[side] = state
                reason_codes.extend(
                    str(code)
                    for code in mapping.get("reason_codes", ()) or ()
                    if str(code)
                )

        states = [*overall_states, *side_states.values()]
        if "FAIL" in states:
            state = "FAIL"
        elif "WARNING" in states:
            state = "WARNING"
        elif overall_states or set(side_states) == {"left", "right"}:
            state = "PASS"
        else:
            state = "UNASSESSED"
            reason_codes.append("quality_snapshot_unavailable")
        return state, tuple(dict.fromkeys(reason_codes))

    def run_checkerboard_random_validation(self) -> None:
        if self.panel.checkerboard_calibration_thread is not None:
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
        worker = CheckerboardRandomValidationWorker(
            mode=mode,
            left_intrinsic_paths=left_paths,
            right_intrinsic_paths=right_paths,
            stereo_pairs=pairs,
            config=self.panel._checkerboard_config(),
            runs=runs,
            subset_count=subset_count,
            seed=seed,
        )
        thread = QtCore.QThread(self.panel)
        worker.moveToThread(thread)
        thread.started.connect(worker.run)
        worker.completed.connect(
            lambda results,
            reference=reference,
            mode=mode,
            pair_count=len(pairs),
            runs=runs,
            subset_count=subset_count,
            seed=seed: self.panel._handle_random_validation_completed(
                results,
                reference=reference,
                mode=mode,
                pair_count=pair_count,
                runs=runs,
                subset_count=subset_count,
                seed=seed,
            )
        )
        worker.progress.connect(self.panel._handle_calibration_progress)
        worker.failed.connect(self.panel._handle_random_validation_failed)
        worker.finished.connect(thread.quit)
        worker.finished.connect(worker.deleteLater)
        thread.finished.connect(lambda: self.panel._clear_checkerboard_calibration_worker(thread))
        self.panel.checkerboard_calibration_thread = thread
        self.panel.checkerboard_calibration_worker = worker
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
        thread.start()

    def _handle_random_validation_failed(self, error: str) -> None:
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

    def _handle_random_validation_completed(
        self,
        results: object,
        *,
        reference: StereoIntrinsicsResult,
        mode: str,
        pair_count: int,
        runs: int,
        subset_count: int,
        seed: int,
    ) -> None:
        rows: list[str] = []
        baselines: list[float] = []
        rotations: list[float] = []
        left_deltas: list[float] = []
        right_deltas: list[float] = []
        reference_rotation = None if reference.extrinsics is None else reference.extrinsics.rotation_left_to_right
        run_results: list[dict[str, object]] = [
            self.panel._checkerboard_validation_result_data(
                "Full",
                reference,
                reference,
                pair_count,
            )
        ]
        rows.append(
            "Run       Pairs  Lerr   Rerr   Serr   Baseline(m)  dB(mm)   dR(deg)  max_dK_L(px)  max_dK_R(px)"
        )
        rows.append(self.panel._checkerboard_validation_result_row("Full", reference, reference, pair_count))
        result_rows = results if isinstance(results, (tuple, list)) else ()
        failed_runs = 0
        successful_runs = 0
        for row in result_rows:
            if not isinstance(row, dict):
                continue
            index = int(row.get("index") or 0)
            error = str(row.get("error") or "")
            if error:
                failed_runs += 1
                rows.append(f"Random {index:<2} FAILED: {error}")
                run_results.append(
                    {
                        "run": f"Random {index}",
                        "pairs": subset_count,
                        "state": "FAILED",
                        "error": error,
                    }
                )
                continue
            result = row.get("result")
            if not isinstance(result, StereoIntrinsicsResult):
                failed_runs += 1
                rows.append(f"Random {index:<2} FAILED: invalid worker result")
                run_results.append(
                    {
                        "run": f"Random {index}",
                        "pairs": subset_count,
                        "state": "FAILED",
                        "error": "invalid worker result",
                    }
                )
                continue
            successful_runs += 1
            run_results.append(
                self.panel._checkerboard_validation_result_data(
                    f"Random {index}",
                    result,
                    reference,
                    subset_count,
                )
            )
            rows.append(
                self.panel._checkerboard_validation_result_row(
                    f"Random {index}",
                    result,
                    reference,
                    subset_count,
                )
            )
            left_delta, right_delta = self.panel._checkerboard_intrinsic_delta_px(result, reference)
            left_deltas.append(left_delta)
            right_deltas.append(right_delta)
            baseline = self.panel._checkerboard_result_baseline_m(result)
            if baseline is not None:
                baselines.append(baseline)
            rotation = self.panel._rotation_delta_deg_from_matrices(
                reference_rotation,
                None if result.extrinsics is None else result.extrinsics.rotation_left_to_right,
            )
            if rotation is not None:
                rotations.append(rotation)
        baseline_std = self.panel._checkerboard_validation_stddev(baselines)
        rotation_std = self.panel._checkerboard_validation_stddev(rotations)
        quality_state, quality_reasons = self._calibration_quality_state(reference)
        if successful_runs == 0:
            quality_state = "FAIL"
            quality_reasons = (*quality_reasons, "all_random_validation_runs_failed")
        elif failed_runs:
            if quality_state != "FAIL":
                quality_state = "WARNING"
            quality_reasons = (*quality_reasons, "random_validation_run_failed")
        quality_reasons = tuple(dict.fromkeys(quality_reasons))
        rows.extend(
            [
                "",
                "Stability:",
                f"max_dK_L std={self.panel._checkerboard_validation_stddev(left_deltas):.3f} px",
                f"max_dK_R std={self.panel._checkerboard_validation_stddev(right_deltas):.3f} px",
                f"baseline std={baseline_std * 1000.0:.3f} mm" if baselines else "baseline std=n/a",
                f"rotation delta std={rotation_std:.6f} deg" if rotations else "rotation delta std=n/a",
                f"quality state={quality_state}",
            ]
        )
        summary = dict(self.panel.checkerboard_validation_summary)
        summary["random_validation"] = {
            "runs": runs,
            "subset_pair_count": subset_count,
            "seed": None if seed == 0 else seed,
            "baseline_std_m": baseline_std if baselines else None,
            "rotation_delta_std_deg": rotation_std if rotations else None,
            "left_intrinsics_delta_std_px": self.panel._checkerboard_validation_stddev(left_deltas),
            "right_intrinsics_delta_std_px": self.panel._checkerboard_validation_stddev(right_deltas),
            "successful_runs": successful_runs,
            "failed_runs": failed_runs,
            "quality_state": quality_state,
            "quality_reason_codes": list(quality_reasons),
        }
        self.panel.checkerboard_validation_summary = summary
        self.panel.checkerboard_validation_run_results = run_results
        full_text = self.panel.calibration_result_text.toPlainText().rstrip()
        self.panel.calibration_result_text.setPlainText(full_text + "\n\nRandom stereo-pair validation:\n" + "\n".join(rows))
        self.panel._set_health_label(
            self.panel.checkerboard_validation_status_label,
            f"랜덤 검증 완료: runs={runs}, subset_pairs={subset_count}, "
            f"mode={mode}, quality={quality_state}",
            {"PASS": "good", "FAIL": "bad"}.get(quality_state, "warn"),
        )
        self.panel.calibration_progress = {
            "phase": "validation_complete",
            "processed": runs,
            "total": runs,
            "percent": 100.0,
            "message": f"랜덤 검증 완료: {runs}회 · quality={quality_state}",
        }
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
        baseline = self.panel._checkerboard_result_baseline_m(result)
        reference_baseline = self.panel._checkerboard_result_baseline_m(reference)
        baseline_text = "n/a" if baseline is None else f"{baseline:.6f}"
        baseline_delta_text = (
            "n/a"
            if baseline is None or reference_baseline is None
            else f"{(baseline - reference_baseline) * 1000.0:+.3f}"
        )
        rotation_delta = self.panel._rotation_delta_deg_from_matrices(
            None if reference.extrinsics is None else reference.extrinsics.rotation_left_to_right,
            None if result.extrinsics is None else result.extrinsics.rotation_left_to_right,
        )
        rotation_text = "n/a" if rotation_delta is None else f"{rotation_delta:.6f}"
        stereo_error = "n/a" if result.extrinsics is None else f"{result.extrinsics.reprojection_error_px:.3f}"
        left_delta, right_delta = self.panel._checkerboard_intrinsic_delta_px(result, reference)
        return (
            f"{name:<10} {pairs:>5}  {result.left.reprojection_error_px:>5.3f}  "
            f"{result.right.reprojection_error_px:>5.3f}  {stereo_error:>5}  "
            f"{baseline_text:>11}  {baseline_delta_text:>7}  {rotation_text:>8}  "
            f"{left_delta:>11.3f}  {right_delta:>11.3f}"
        )

    def _checkerboard_validation_result_data(
        self,
        name: str,
        result: StereoIntrinsicsResult,
        reference: StereoIntrinsicsResult,
        pairs: int,
    ) -> dict[str, object]:
        baseline = self.panel._checkerboard_result_baseline_m(result)
        reference_baseline = self.panel._checkerboard_result_baseline_m(reference)
        rotation_delta = self.panel._rotation_delta_deg_from_matrices(
            None if reference.extrinsics is None else reference.extrinsics.rotation_left_to_right,
            None if result.extrinsics is None else result.extrinsics.rotation_left_to_right,
        )
        left_delta, right_delta = self.panel._checkerboard_intrinsic_delta_px(result, reference)
        return {
            "run": name,
            "pairs": pairs,
            "left_reprojection_error_px": result.left.reprojection_error_px,
            "right_reprojection_error_px": result.right.reprojection_error_px,
            "stereo_reprojection_error_px": (
                None if result.extrinsics is None else result.extrinsics.reprojection_error_px
            ),
            "baseline_m": baseline,
            "baseline_delta_mm": (
                None
                if baseline is None or reference_baseline is None
                else (baseline - reference_baseline) * 1000.0
            ),
            "rotation_delta_deg": rotation_delta,
            "left_intrinsics_delta_px": left_delta,
            "right_intrinsics_delta_px": right_delta,
            "state": "OK",
            "error": "",
        }

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
        reason_text = ", ".join(reasons) or "no reason supplied"
        if state == "FAIL":
            self.panel._set_health_label(
                self.panel.checkerboard_validation_status_label,
                f"{action} 차단: calibration quality FAIL ({reason_text})",
                "bad",
            )
            return False
        if state == "WARNING" and not warning_confirmed:
            self.panel._set_health_label(
                self.panel.checkerboard_validation_status_label,
                f"{action} 대기: WARNING 사유를 검토하고 명시적으로 승인하세요 ({reason_text})",
                "warn",
            )
            return False
        if state == "WARNING":
            summary = dict(getattr(self.panel, "checkerboard_validation_summary", {}) or {})
            summary["quality_override"] = {
                "action": str(action),
                "confirmed": True,
                "confirmed_utc_ns": time.time_ns(),
                "reason_codes": list(reasons),
            }
            self.panel.checkerboard_validation_summary = summary
        return True

    def _publish_immutable_calibration_file(
        self,
        source: Path,
        *,
        family: str,
        prefix: str,
    ) -> Path:
        """Publish exact profile bytes once under a content-addressed name."""

        payload = Path(source).read_bytes()
        digest = hashlib.sha256(payload).hexdigest()
        root, _left_dir, _right_dir = self.panel._calibration_dirs()
        destination_dir = root / "profiles" / family
        destination_dir.mkdir(parents=True, exist_ok=True)
        destination = destination_dir / f"{prefix}-{digest}.json"
        if destination.exists():
            if destination.read_bytes() != payload:
                raise RuntimeError(
                    f"immutable calibration hash collision at {destination}"
                )
            return destination
        temporary = destination.with_name(f".{destination.name}.{uuid.uuid4().hex}.tmp")
        try:
            with temporary.open("xb") as handle:
                handle.write(payload)
                handle.flush()
                os.fsync(handle.fileno())
            try:
                os.link(temporary, destination)
            except FileExistsError:
                if destination.read_bytes() != payload:
                    raise RuntimeError(
                        f"immutable calibration destination changed at {destination}"
                    )
            descriptor = os.open(destination_dir, os.O_RDONLY)
            try:
                os.fsync(descriptor)
            finally:
                os.close(descriptor)
        finally:
            temporary.unlink(missing_ok=True)
        return destination

    def _publish_immutable_intrinsics_profile(
        self,
        result: StereoIntrinsicsResult,
        *,
        metadata: Mapping[str, object],
    ) -> Path:
        root, _left_dir, _right_dir = self.panel._calibration_dirs()
        staging = root / "profiles" / ".staging"
        staging.mkdir(parents=True, exist_ok=True)
        temporary = staging / f"intrinsics-{uuid.uuid4().hex}.json"
        try:
            save_intrinsics_json(temporary, result, metadata=dict(metadata))
            return self._publish_immutable_calibration_file(
                temporary,
                family="intrinsics",
                prefix="intrinsics",
            )
        finally:
            temporary.unlink(missing_ok=True)

    def _publish_immutable_extrinsics_profile(
        self,
        profile: StereoExtrinsicsProfile,
    ) -> Path:
        root, _left_dir, _right_dir = self.panel._calibration_dirs()
        staging = root / "profiles" / ".staging"
        staging.mkdir(parents=True, exist_ok=True)
        temporary = staging / f"extrinsics-{uuid.uuid4().hex}.json"
        try:
            save_stereo_extrinsics_json(temporary, profile)
            return self._publish_immutable_calibration_file(
                temporary,
                family="extrinsics",
                prefix="extrinsics",
            )
        finally:
            temporary.unlink(missing_ok=True)

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
        root.mkdir(parents=True, exist_ok=True)
        zoom_level = getattr(self.panel, "last_calibration_zoom_level", None)
        quality_state, quality_reasons = self._calibration_quality_state(result)
        approved = quality_state in {"PASS", "UNASSESSED"} and zoom_level is not None
        if approved:
            output = self.panel._intrinsics_storage_path(zoom_level)
        else:
            attempts = root / "attempts"
            attempts.mkdir(parents=True, exist_ok=True)
            zoom_token = "unknown" if zoom_level is None else f"{int(zoom_level):02d}"
            state_token = str(quality_state or "unassessed").lower()
            output = attempts / (
                f"intrinsics_zoom_{zoom_token}_{state_token}_{time.time_ns()}.json"
            )
        metadata = {
            "note": "Generated by StereoWIRIS calibration calculation",
            "timestamp_s": time.time(),
            "zoom_level": zoom_level,
            "capture_zoom_level": zoom_level,
            "zoom_target": str(self.panel.zoom_target_combo.currentData() or "both"),
            "calculation_mode": self.panel.checkerboard_validation_result_mode,
            "validation": self.panel.checkerboard_validation_summary,
            "profile_status": "approved" if approved else "attempt_only",
            "eligible_for_application": bool(approved),
            "quality_state": quality_state,
            "quality_reasons": list(quality_reasons),
        }
        save_intrinsics_json(output, result, metadata=metadata)
        self.panel.checkerboard_validation_result_saved = True
        self.panel._last_calibration_attempt_path = str(output)
        if approved:
            latest_output = self.panel._intrinsics_storage_base_path()
            if output != latest_output:
                save_intrinsics_json(latest_output, result, metadata=metadata)
            immutable_output = self._publish_immutable_intrinsics_profile(
                result,
                metadata=metadata,
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
        candidate_path_text = str(
            getattr(panel, "_candidate_intrinsics_profile_path", "") or ""
        )
        # Lightweight legacy facades may not expose the candidate slot yet.
        # Reuse their pre-Apply marker only when no active matrix exists, so a
        # new candidate can never borrow an older active profile identity.
        if not candidate_path_text and getattr(panel, "active_intrinsics", None) is None:
            candidate_path_text = str(
                getattr(panel, "_active_intrinsics_profile_path", "") or ""
            )
        current_intrinsics_path = Path(candidate_path_text)
        if (
            current_intrinsics_path.is_file()
            and current_intrinsics_path.parent.name == "intrinsics"
            and current_intrinsics_path.parent.parent.name == "profiles"
        ):
            immutable_intrinsics = current_intrinsics_path
        else:
            immutable_intrinsics = self._publish_immutable_intrinsics_profile(
                panel.last_calibration_result,
                metadata={
                    "profile_status": "runtime_applied",
                    "eligible_for_application": True,
                    "zoom_level": panel.last_calibration_zoom_level,
                    "capture_zoom_level": panel.last_calibration_zoom_level,
                    "quality_state": quality_state,
                    "quality_reasons": list(quality_reasons),
                    "quality_warning_confirmed": bool(
                        quality_state == "WARNING" and quality_warning_confirmed
                    ),
                },
            )
        panel._candidate_intrinsics_profile_path = str(immutable_intrinsics)
        selected_profile = (
            panel.selected_extrinsics_profile
            if extrinsics_profile_override is _USE_SELECTED_EXTRINSICS
            else extrinsics_profile_override
        )
        if selected_profile is not None and not isinstance(
            selected_profile, StereoExtrinsicsProfile
        ):
            raise TypeError(
                "extrinsics_profile_override must be StereoExtrinsicsProfile or None"
            )
        if (
            selected_profile is None
            and extrinsics_profile_override is _USE_SELECTED_EXTRINSICS
            and panel.last_calibration_result.extrinsics is not None
        ):
            selected_profile = StereoExtrinsicsProfile(
                extrinsics=panel.last_calibration_result.extrinsics,
                source_zoom_level=panel.last_calibration_zoom_level,
                method="legacy_embedded_checkerboard",
                source_intrinsics_path=panel._candidate_intrinsics_profile_path,
                source_intrinsics_sha256=intrinsics_content_sha256(
                    panel.last_calibration_result
                ),
                profile_id="legacy_embedded_extrinsics",
            )
            panel.selected_extrinsics_profile = selected_profile
            panel._selected_extrinsics_profile_path = (
                f"{panel._candidate_intrinsics_profile_path}#embedded-extrinsics"
            )
            panel._selected_extrinsics_profile_hash = selected_profile.content_sha256
        if selected_profile is not None:
            # Always derive the immutable path from the profile being applied.
            # Reusing the currently selected path can pair a fresh profile hash
            # with stale R,t bytes from an earlier candidate.
            immutable_extrinsics = self._publish_immutable_extrinsics_profile(
                selected_profile
            )
            panel.selected_extrinsics_profile = selected_profile
            if str(selected_profile.method or "").startswith("optical_pose/"):
                panel.optical_pose_extrinsics_candidate_profile = selected_profile
            else:
                panel.checkerboard_extrinsics_candidate_profile = selected_profile
            panel._selected_extrinsics_profile_path = str(immutable_extrinsics)
            panel._selected_extrinsics_profile_hash = selected_profile.content_sha256
        panel.active_intrinsics = compose_stereo_calibration(
            panel.last_calibration_result,
            selected_profile,
        )
        panel.active_extrinsics_profile = selected_profile
        if selected_profile is None:
            panel.checkerboard_extrinsics_profile = None
            panel.optical_pose_extrinsics_profile = None
        else:
            if str(selected_profile.method or "").startswith("optical_pose/"):
                panel.optical_pose_extrinsics_profile = selected_profile
            else:
                panel.checkerboard_extrinsics_profile = selected_profile
        # Apply is the only transition that promotes candidate provenance to
        # the active K,D identity consumed by Measurement and package export.
        panel._active_intrinsics_profile_path = str(immutable_intrinsics)
        panel._active_intrinsics_source_zoom_level = panel.last_calibration_zoom_level
        panel._active_intrinsics_profile_hash = intrinsics_content_sha256(
            panel.last_calibration_result
        )
        if selected_profile is None:
            panel._active_extrinsics_profile_path = ""
            panel._active_extrinsics_profile_hash = ""
            panel._active_extrinsics_source_zoom_level = None
            panel._active_calibration_selection_mode = "intrinsics_only"
        else:
            panel._active_extrinsics_profile_path = (
                panel._selected_extrinsics_profile_path
                or f"memory:{selected_profile.profile_id or 'stereo_extrinsics'}"
            )
            panel._active_extrinsics_profile_hash = selected_profile.content_sha256
            panel._active_extrinsics_source_zoom_level = selected_profile.source_zoom_level
            panel._active_calibration_selection_mode = independent_zoom_selection_mode(
                panel.last_calibration_zoom_level,
                selected_profile.source_zoom_level,
            )
            panel._set_combo_data(panel.distance_mode_combo, "checkerboard")
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
        configured_root = self._configured_calibration_root()
        session_root = getattr(self.panel, "_calibration_session_root", None)
        default_root = self._default_calibration_root()
        if session_root is not None and (
            configured_root == default_root
            or bool(getattr(self.panel, "_measurement_session_hierarchy_enabled", False))
        ):
            root = Path(session_root)
        else:
            if session_root is not None:
                self.panel._calibration_session_root = None
                self.panel._calibration_session_roots = {}
            root = configured_root
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
        hierarchy_enabled = bool(
            getattr(self.panel, "_measurement_session_hierarchy_enabled", False)
        )
        session_id = ""
        if hierarchy_enabled:
            session_state = str(
                getattr(self.panel, "_measurement_session_state", "IDLE") or "IDLE"
            ).upper()
            session_id = str(
                getattr(self.panel, "_measurement_session_id", "")
                or getattr(self.panel, "recording_session_id", "")
                or ""
            )
            resolver = getattr(self.panel, "_measurement_session_dir", None)
            if session_state == "OPEN":
                if not session_id or not callable(resolver):
                    raise RuntimeError("open Session calibration owner is unavailable")
                session_base = Path(resolver(session_id)) / "calibration"
            else:
                # Pre-session calibration is deliberately reusable and is not
                # owned by a not-yet-created measurement Session.  Keep the
                # configured path stable so a later compute action can use the
                # same photo set selected by the UI.
                configured_root.mkdir(parents=True, exist_ok=True)
                write_calibration_session_manifest(
                    configured_root,
                    session_kind="calibration_pre_session",
                    source=f"pre_session:{capture_origin}",
                )
                self.panel._log(
                    f"[캘리브레이션] 세션 전 공용 캘리브레이션 폴더: {configured_root}"
                )
                return configured_root
        else:
            default_root = self._default_calibration_root()
            if configured_root != default_root:
                return None
            session_base = default_root
        origin_key = "auto" if str(capture_origin).strip().lower() == "auto" else "default"
        session_key = f"{session_id}:{origin_key}" if hierarchy_enabled else origin_key
        session_roots = getattr(self.panel, "_calibration_session_roots", None)
        if not isinstance(session_roots, dict):
            session_roots = {}
        existing = session_roots.get(session_key)
        if existing is not None:
            existing_path = Path(existing)
            try:
                existing_path.resolve(strict=False).relative_to(session_base.resolve(strict=False))
            except ValueError:
                session_roots.pop(session_key, None)
            else:
                self.panel._calibration_session_root = existing_path
                return existing_path
        session_base.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y-%m-%d_%H-%M-%S", time.localtime())
        candidate = session_base / stamp
        suffix = 1
        while candidate.exists():
            candidate = session_base / f"{stamp}_{suffix:02d}"
            suffix += 1
        if origin_key == "auto":
            candidate = candidate / "calib_capture" / "AutoCapture"
        candidate.mkdir(parents=True, exist_ok=False)
        session_roots[session_key] = candidate
        self.panel._calibration_session_roots = session_roots
        self.panel._calibration_session_root = candidate
        write_calibration_session_manifest(candidate, source=origin_key)
        self.panel._log(
            f"[캘리브레이션] 이번 Workstation {session_key} 캘리브레이션 폴더: {candidate}"
        )
        return candidate

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
        destination_dir = calibration_root / "loaded_intrinsics"
        destination_dir.mkdir(parents=True, exist_ok=True)
        destination = destination_dir / f"{time.time_ns()}-{source.name}"
        payload = source.read_bytes()
        temporary = destination.with_name(f".{destination.name}.{uuid.uuid4().hex}.tmp")
        try:
            with temporary.open("xb") as handle:
                handle.write(payload)
                handle.flush()
            temporary.replace(destination)
        finally:
            temporary.unlink(missing_ok=True)
        return destination

    @staticmethod
    def _prepare_ftp_capture_dirs(root: Path, capture_family: str) -> None:
        capture_root = root / "calib_capture" / capture_family
        for side in ("left", "right"):
            (capture_root / side).mkdir(parents=True, exist_ok=True)

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

    def _checkerboard_capture_pair_timing(
        self,
        metadata_by_camera: dict[str, dict[str, object]] | None = None,
        *,
        maximum_delta_s: float | None = None,
    ) -> tuple[bool, str, float | None, str]:
        evidence = self._checkerboard_capture_pair_timing_evidence(
            metadata_by_camera,
            maximum_delta_s=maximum_delta_s,
        )
        compatibility_detail = (
            "ok"
            if evidence.pair_persistence_allowed
            and evidence.evidence_class == CAMERA_TIMESTAMP_VERIFIED
            else evidence.detail
        )
        return (
            evidence.pair_persistence_allowed,
            evidence.timestamp_reference,
            evidence.pair_delta_s,
            compatibility_detail,
        )

    @staticmethod
    def _app_receive_monotonic_ns_from_metadata(
        metadata: Mapping[str, object],
    ) -> tuple[int | None, str]:
        """Return one decoded-frame host receipt stamp and its field source.

        Normal camera metadata carries the exact integer field.  Native-ring
        descriptors also guarantee that ``timestamp_s`` is the producer's
        app-receive monotonic clock, so it remains a bounded compatibility
        source when preview throttling omitted the redundant integer field.
        """

        exact = metadata.get("app_receive_monotonic_ns")
        if exact is not None:
            if isinstance(exact, bool):
                return None, ""
            try:
                if isinstance(exact, float) and (
                    not math.isfinite(exact) or not exact.is_integer()
                ):
                    raise ValueError
                exact_ns = int(exact)
            except (TypeError, ValueError, OverflowError):
                return None, ""
            if exact_ns <= 0:
                return None, ""
            return exact_ns, "app_receive_monotonic_ns"

        compatibility = metadata.get("timestamp_s")
        if compatibility is None or isinstance(compatibility, bool):
            return None, ""
        try:
            compatibility_s = float(compatibility)
        except (TypeError, ValueError, OverflowError):
            return None, ""
        if not math.isfinite(compatibility_s) or compatibility_s <= 0.0:
            return None, ""
        return int(round(compatibility_s * 1_000_000_000.0)), "timestamp_s"

    def _checkerboard_capture_pair_timing_evidence(
        self,
        metadata_by_camera: dict[str, dict[str, object]] | None = None,
        *,
        maximum_delta_s: float | None = None,
    ) -> CalibrationTimingEvidence:
        metadata_by_camera = metadata_by_camera or self.panel.camera_frame_metadata
        left_metadata = metadata_by_camera.get("left") or {}
        right_metadata = metadata_by_camera.get("right") or {}
        limit_s = (
            self._checkerboard_pair_max_delta_s()
            if maximum_delta_s is None
            else float(maximum_delta_s)
        )
        if not math.isfinite(limit_s) or limit_s < 0.0:
            limit_s = CHECKERBOARD_STEREO_PAIR_MAX_DELTA_S

        def exposure_timestamp(metadata: Mapping[str, object]) -> float | None:
            if metadata.get("exposure_timestamp_verified") is not True:
                return None
            try:
                if metadata.get("exposure_timestamp_unix_ns") is not None:
                    return int(metadata["exposure_timestamp_unix_ns"]) * 1.0e-9
                if metadata.get("exposure_timestamp_unix_s") is not None:
                    return float(metadata["exposure_timestamp_unix_s"])
            except (TypeError, ValueError, OverflowError):
                return None
            return None

        left_exposure = exposure_timestamp(left_metadata)
        right_exposure = exposure_timestamp(right_metadata)
        if left_exposure is not None or right_exposure is not None:
            if left_exposure is None or right_exposure is None:
                return CalibrationTimingEvidence(
                    UNKNOWN_TIMING_EVIDENCE,
                    False,
                    False,
                    False,
                    "camera-exposure",
                    None,
                    limit_s,
                    "verified exposure timestamp is missing for one camera",
                )
            delta_s = right_exposure - left_exposure
            allowed = abs(delta_s) <= limit_s
            return CalibrationTimingEvidence(
                EXPOSURE_TIMESTAMP_VERIFIED,
                allowed,
                allowed,
                True,
                "camera-exposure",
                delta_s,
                limit_s,
                "verified exposure timestamp pair within gate"
                if allowed
                else (
                    f"exposure timestamp delta {delta_s * 1000.0:.1f} ms exceeds "
                    f"{limit_s * 1000.0:.1f} ms"
                ),
            )

        left_ftp_id = str(left_metadata.get("ftp_capture_id") or "")
        right_ftp_id = str(right_metadata.get("ftp_capture_id") or "")
        if left_ftp_id or right_ftp_id:
            contract = "same_capture_id_and_synchronized_cptr_dispatch_v1"
            if not left_ftp_id or left_ftp_id != right_ftp_id:
                return CalibrationTimingEvidence(
                    HOST_DISPATCH_PROXY, False, False, False,
                    "ftp-cptr-dispatch", None, limit_s, "FTP capture_id mismatch",
                )
            if (
                left_metadata.get("capture_pairing_contract") != contract
                or right_metadata.get("capture_pairing_contract") != contract
            ):
                return CalibrationTimingEvidence(
                    HOST_DISPATCH_PROXY,
                    False,
                    False,
                    False,
                    "ftp-cptr-dispatch",
                    None,
                    limit_s,
                    "FTP still lacks the synchronized CPTR dispatch pairing contract",
                )
            timestamp_fields = (
                ("capture_requested_monotonic_ns", 1.0e-9, "ftp-cptr-dispatch-monotonic"),
                ("capture_requested_unix_ns", 1.0e-9, "ftp-cptr-dispatch-unix"),
            )
            for field, scale, basis in timestamp_fields:
                try:
                    left_value = int(left_metadata.get(field))
                    right_value = int(right_metadata.get(field))
                except (TypeError, ValueError, OverflowError):
                    continue
                if left_value <= 0 or right_value <= 0:
                    continue
                delta_s = (right_value - left_value) * scale
                if abs(delta_s) > limit_s:
                    return CalibrationTimingEvidence(
                        HOST_DISPATCH_PROXY,
                        False,
                        False,
                        False,
                        basis,
                        delta_s,
                        limit_s,
                        (
                            f"FTP CPTR dispatch delta {delta_s * 1000.0:.1f} ms exceeds "
                            f"{limit_s * 1000.0:.1f} ms; exposure timestamps remain unknown"
                        ),
                    )
                return CalibrationTimingEvidence(
                    HOST_DISPATCH_PROXY,
                    True,
                    False,
                    False,
                    basis,
                    delta_s,
                    limit_s,
                    (
                        "same FTP capture_id; synchronized CPTR dispatch within gate; "
                        "camera exposure timestamps unknown"
                    ),
                )
            return CalibrationTimingEvidence(
                HOST_DISPATCH_PROXY,
                False,
                False,
                False,
                "ftp-cptr-dispatch",
                None,
                limit_s,
                "FTP still lacks per-camera CPTR dispatch timestamps",
            )

        left_receive_ns, left_receive_source = (
            self._app_receive_monotonic_ns_from_metadata(left_metadata)
        )
        right_receive_ns, right_receive_source = (
            self._app_receive_monotonic_ns_from_metadata(right_metadata)
        )
        if left_receive_ns is None or right_receive_ns is None:
            return CalibrationTimingEvidence(
                UNKNOWN_TIMING_EVIDENCE,
                False,
                False,
                False,
                "app-receive-monotonic",
                None,
                limit_s,
                "missing decoded-frame app-receive monotonic timestamp for left or right frame",
            )
        delta_ns = right_receive_ns - left_receive_ns
        delta_s = delta_ns * 1.0e-9
        source_detail = (
            "exact integer metadata"
            if left_receive_source == right_receive_source == "app_receive_monotonic_ns"
            else (
                "native-frame timestamp_s compatibility"
                if left_receive_source == right_receive_source == "timestamp_s"
                else "mixed exact/compatibility app-receive metadata"
            )
        )
        if abs(delta_s) > limit_s:
            return CalibrationTimingEvidence(
                HOST_RECEIVE_PROXY,
                False,
                False,
                False,
                "app-receive-monotonic",
                delta_s,
                limit_s,
                f"decoded-frame app-receive pair delta {delta_s * 1000.0:.1f} ms exceeds "
                f"{limit_s * 1000.0:.1f} ms ({source_detail}); camera exposure remains unverified",
            )
        return CalibrationTimingEvidence(
            HOST_RECEIVE_PROXY,
            True,
            False,
            False,
            "app-receive-monotonic",
            delta_s,
            limit_s,
            f"decoded-frame app-receive delta within gate ({source_detail}); "
            "camera exposure remains unverified",
        )

    def capture_calibration_images(
        self,
        *,
        frames: dict[str, QtGui.QImage | None] | None = None,
        metadata_by_camera: dict[str, dict[str, object]] | None = None,
        capture_origin: str = "manual",
    ) -> bool:
        if self.panel.checkerboard_calibration_thread is not None:
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
        frames = {
            side: None
            if image is None
            else image.convertToFormat(QtGui.QImage.Format_RGB888).copy()
            for side, image in frames.items()
        }
        metadata_by_camera = metadata_by_camera or self.panel.camera_frame_metadata
        self._ensure_calibration_session_root(capture_origin)
        root, _left_dir, _right_dir = self.panel._calibration_dirs()
        stem = self.panel._next_calibration_capture_stem()
        available, error = opencv_available()
        config = self.panel._checkerboard_config() if available else None
        pair_candidate = frames["left"] is not None and frames["right"] is not None
        pair_max_delta_s = self._checkerboard_pair_max_delta_s()
        timing_evidence = (
            self._checkerboard_capture_pair_timing_evidence(
                metadata_by_camera,
                maximum_delta_s=pair_max_delta_s,
            )
            if pair_candidate
            else CalibrationTimingEvidence(
                UNKNOWN_TIMING_EVIDENCE,
                False,
                False,
                False,
                "none",
                None,
                pair_max_delta_s,
                "single-camera capture",
            )
        )
        pair_timing_ok = timing_evidence.pair_persistence_allowed
        pair_time_basis = timing_evidence.timestamp_reference
        pair_delta_s = timing_evidence.pair_delta_s
        pair_timing_detail = timing_evidence.detail
        app_receive_samples = {
            side: (
                self._app_receive_monotonic_ns_from_metadata(
                    metadata_by_camera.get(side) or {}
                )
                if frames.get(side) is not None
                else (None, "")
            )
            for side in ("left", "right")
        }
        left_app_receive_ns = app_receive_samples["left"][0]
        right_app_receive_ns = app_receive_samples["right"][0]
        app_receive_pair_delta_ns = (
            right_app_receive_ns - left_app_receive_ns
            if left_app_receive_ns is not None and right_app_receive_ns is not None
            else None
        )
        zoom_snapshot = self._capture_zoom_snapshot()
        self.panel.last_capture_zoom_level = zoom_snapshot.get("capture_zoom_level")
        metadata_payload = {
            "capture_origin": str(capture_origin),
            "capture_recorded_unix_ns": time.time_ns(),
            "pair_time_basis": pair_time_basis,
            "pair_delta_s": pair_delta_s,
            "pair_delta_ns": (
                app_receive_pair_delta_ns
                if pair_time_basis == "app-receive-monotonic"
                else None
            ),
            "pair_max_delta_s": pair_max_delta_s,
            "pair_timing_status": pair_timing_detail,
            "calibration_timing_evidence": timing_evidence.to_mapping(),
            "timing_evidence_class": timing_evidence.evidence_class,
            "pair_persistence_allowed": timing_evidence.pair_persistence_allowed,
            "precision_timing_qualified": timing_evidence.precision_qualified,
            "actual_exposure_verified": timing_evidence.actual_exposure_verified,
            "left_app_receive_monotonic_ns": left_app_receive_ns,
            "right_app_receive_monotonic_ns": right_app_receive_ns,
            "app_receive_pair_delta_ns": app_receive_pair_delta_ns,
            "left_rtcp_frame_ntp_unix_s": (metadata_by_camera.get("left") or {}).get("rtcp_frame_ntp_unix_s"),
            "right_rtcp_frame_ntp_unix_s": (metadata_by_camera.get("right") or {}).get("rtcp_frame_ntp_unix_s"),
            "frame_contract": (
                "OpenCV source image pixels (x right, y down); no application resize/crop/overlay; "
                "native/preview/saved grids are explicit per side; "
                f"stereo pairing gate uses {pair_time_basis}; decoded-frame app receipt and "
                "FTP CPTR dispatch are host-side pairing proxies and never exposure timestamps"
            ),
            "frame_grid_by_side": {
                side: {
                    "frame_id": str((metadata_by_camera.get(side) or {}).get("frame_id") or ""),
                    "frame_key": str(
                        (metadata_by_camera.get(side) or {}).get("frame_key") or ""
                    ),
                    "h264_access_unit_kind": (
                        metadata_by_camera.get(side) or {}
                    ).get("h264_access_unit_kind"),
                    "h264_access_unit_nal_types": list(
                        (metadata_by_camera.get(side) or {}).get(
                            "h264_access_unit_nal_types", ()
                        )
                        or ()
                    ),
                    "h264_access_unit_is_delta": (
                        metadata_by_camera.get(side) or {}
                    ).get("h264_access_unit_is_delta"),
                    "h264_access_unit_pts_ns": (
                        metadata_by_camera.get(side) or {}
                    ).get("h264_access_unit_pts_ns"),
                    "decoded_buffer_pts_ns": (
                        metadata_by_camera.get(side) or {}
                    ).get("decoded_buffer_pts_ns"),
                    "h264_nal1_non_idr_intra": (
                        metadata_by_camera.get(side) or {}
                    ).get("h264_nal1_non_idr_intra"),
                    "h264_random_access_guaranteed": (
                        metadata_by_camera.get(side) or {}
                    ).get("h264_random_access_guaranteed"),
                    "app_receive_monotonic_ns": app_receive_samples[side][0],
                    "app_receive_timestamp_source": app_receive_samples[side][1],
                    "source_native_width": int(
                        (metadata_by_camera.get(side) or {}).get("native_width")
                        or (frames.get(side).width() if frames.get(side) is not None else 0)
                    ),
                    "source_native_height": int(
                        (metadata_by_camera.get(side) or {}).get("native_height")
                        or (frames.get(side).height() if frames.get(side) is not None else 0)
                    ),
                    "saved_width": frames.get(side).width() if frames.get(side) is not None else 0,
                    "saved_height": frames.get(side).height() if frames.get(side) is not None else 0,
                    "preview_width": int(
                        (getattr(self.panel, "camera_frame_metadata", {}).get(side) or {}).get(
                            "preview_width", 0
                        )
                    ),
                    "preview_height": int(
                        (getattr(self.panel, "camera_frame_metadata", {}).get(side) or {}).get(
                            "preview_height", 0
                        )
                    ),
                    "resize_scale_xy": [1.0, 1.0],
                    "crop_xywh": None,
                    "overlay_applied": False,
                    "native_lossless_source": bool(
                        (metadata_by_camera.get(side) or {}).get("native_lossless_source", False)
                    ),
                }
                for side in ("left", "right")
            },
            "auto_pose_geometry_by_side": {
                side: view.to_mapping()
                for side, view in (
                    getattr(self, "_auto_capture_pending_pose_views", None) or {}
                ).items()
                if isinstance(view, CalibrationViewGeometry)
            },
            "auto_pose_novelty_by_side": dict(
                getattr(self, "_auto_capture_pending_pose_novelty", None) or {}
            ),
            **zoom_snapshot,
        }
        worker = CheckerboardCaptureWorker(
            root=root,
            frames=frames,
            stem=stem,
            config=config,
            opencv_error=error,
            pair_timing_ok=pair_timing_ok,
            pair_time_basis=pair_time_basis,
            pair_delta_s=pair_delta_s,
            pair_timing_detail=pair_timing_detail,
            metadata_payload=metadata_payload,
            timing_evidence=timing_evidence.to_mapping(),
        )
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
        thread = QtCore.QThread(self.panel)
        worker.moveToThread(thread)
        thread.started.connect(worker.run)
        worker.completed.connect(self.panel._handle_checkerboard_capture_completed)
        worker.failed.connect(self.panel._handle_checkerboard_capture_failed)
        worker.finished.connect(thread.quit)
        worker.finished.connect(worker.deleteLater)
        thread.finished.connect(lambda: self.panel._clear_checkerboard_calibration_worker(thread))
        self.panel.checkerboard_calibration_thread = thread
        self.panel.checkerboard_calibration_worker = worker
        self._connect_calibration_worker_progress(
            worker,
            operation="Capture Live Pair",
            capture_id=stem,
        )
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
        thread.start()
        return True

    def capture_calibration_images_wiris_ftp(self) -> bool:
        """Capture a left/right pair via a WIRIS HR still + FTP download.

        A second source alongside ``capture_calibration_images``'s live-RTSP
        path, not a replacement: once the still is downloaded, it is handed
        to that exact same method (``frames=..., capture_origin="ftp"``), so
        save/naming/checkerboard-validation logic is entirely unchanged.
        """

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
        thread = QtCore.QThread(self.panel)
        worker = WirisPnPPhotoCaptureWorker(
            target_infos,
            capture_id=capture_id,
            capture_channel="calib",
            completion_timeout_s=10.0,
            **kwargs,
        )
        worker.moveToThread(thread)
        thread.started.connect(worker.run)
        worker.completed.connect(self.panel._handle_calibration_wiris_ftp_capture_completed)
        worker.failed.connect(self.panel._handle_calibration_wiris_ftp_capture_failed)
        worker.finished.connect(thread.quit)
        worker.finished.connect(worker.deleteLater)
        thread.finished.connect(thread.deleteLater)
        thread.finished.connect(self.panel._clear_calibration_wiris_ftp_capture_worker)
        self.panel.calibration_wiris_ftp_thread = thread
        self.panel.calibration_wiris_ftp_worker = worker
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
        thread.start()
        return True

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
        self.panel.calibration_wiris_ftp_thread = None
        self.panel.calibration_wiris_ftp_worker = None

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

    @staticmethod
    def _ftp_capture_failures(cameras: dict[object, object]) -> list[str]:
        failures: list[str] = []
        for side, raw_item in cameras.items():
            item = raw_item if isinstance(raw_item, dict) else {}
            state = str(item.get("state") or "failed")
            if state != "completed":
                reason = str(item.get("error") or state)
            elif not bool(item.get("ftp_downloaded")):
                reason = str(item.get("ftp_error") or "FTP download failed")
            else:
                continue
            failures.append(f"{side}: {reason}")
        return failures

    @staticmethod
    def _ftp_calibration_capture_metadata(
        *,
        capture_id: str,
        cameras: Mapping[object, object],
        frames: Mapping[str, QtGui.QImage | None],
    ) -> dict[str, dict[str, object]]:
        """Describe FTP stills without borrowing the live RTSP clock.

        CPTR/ICPT and FTP timestamps are lifecycle evidence, not exposure
        timestamps.  Keeping the reference explicitly unknown prevents
        ``capture_calibration_images`` from falling back to the panel's
        unrelated current preview metadata and forming a false stereo pair.
        """

        result: dict[str, dict[str, object]] = {}
        for side in ("left", "right"):
            raw_item = cameras.get(side)
            item = raw_item if isinstance(raw_item, Mapping) else {}
            image = frames.get(side)
            result[side] = {
                "timestamp_reference": "unknown",
                "capture_timestamp_known": False,
                "ftp_capture_id": capture_id,
                "ftp_remote_path": str(item.get("ftp_remote_path") or ""),
                "capture_requested_unix_ns": item.get("capture_requested_unix_ns"),
                "capture_requested_monotonic_ns": item.get(
                    "capture_requested_monotonic_ns"
                ),
                "capture_pairing_contract": item.get("capture_pairing_contract"),
                "capture_completed_unix_ns": item.get("capture_completed_unix_ns"),
                "ftp_download_finished_unix_ns": item.get("finished_unix_ns"),
                "native_width": image.width() if isinstance(image, QtGui.QImage) else 0,
                "native_height": image.height() if isinstance(image, QtGui.QImage) else 0,
                "native_lossless_source": False,
            }
        return result

    def capture_calibration_images_wiris_ftp_oz(self) -> bool:
        """Same as ``capture_calibration_images_wiris_ftp``, on the OZ channel.

        The WIRIS normal visible operating-zoom still (``SIVI TRUE``) instead
        of the HR still (``SIHR TRUE``) -- lower resolution than HR, but a
        faster capture with no camera reboot risk (unlike changing HR
        resolution via ``SHRR``). Downloads land in a separate
        ``calib_capture/OZ`` subfolder so they never mix with HR downloads,
        and feed the exact same ``capture_calibration_images`` save path.
        """

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
        thread = QtCore.QThread(self.panel)
        worker = WirisPnPPhotoCaptureWorker(
            target_infos,
            capture_id=capture_id,
            image_enable_command="SIVI TRUE",
            capture_channel="calib_oz",
            completion_timeout_s=10.0,
            **kwargs,
        )
        worker.moveToThread(thread)
        thread.started.connect(worker.run)
        worker.completed.connect(self.panel._handle_calibration_wiris_ftp_oz_capture_completed)
        worker.failed.connect(self.panel._handle_calibration_wiris_ftp_oz_capture_failed)
        worker.finished.connect(thread.quit)
        worker.finished.connect(worker.deleteLater)
        thread.finished.connect(thread.deleteLater)
        thread.finished.connect(self.panel._clear_calibration_wiris_ftp_oz_capture_worker)
        self.panel.calibration_wiris_ftp_oz_thread = thread
        self.panel.calibration_wiris_ftp_oz_worker = worker
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
        thread.start()
        return True

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
        self.panel.calibration_wiris_ftp_oz_thread = None
        self.panel.calibration_wiris_ftp_oz_worker = None

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
        root, _left_dir, _right_dir = self.panel._calibration_dirs()
        return root / "calib_capture" / "OZ" / "ledger.json"

    def _load_calib_oz_ledger(self) -> dict[str, object]:
        empty: dict[str, object] = {"rounds": [], "consumed_remote_names": {"left": [], "right": []}}
        path = self.panel._calib_oz_ledger_path()
        if not path.exists():
            return empty
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return empty
        if not isinstance(data, dict):
            return empty
        data.setdefault("rounds", [])
        data.setdefault("consumed_remote_names", {"left": [], "right": []})
        return data

    def _save_calib_oz_ledger(self, ledger: dict[str, object]) -> None:
        path = self.panel._calib_oz_ledger_path()
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(ledger, indent=2), encoding="utf-8")

    def capture_calibration_images_wiris_oz_capture_only(self) -> bool:
        """Trigger an OZ still on both cameras and leave it on camera storage.

        No FTP download here -- see ``sync_calibration_images_wiris_oz_ftp``.
        Meant to be pressed repeatedly while watching the live view for board
        framing, accumulating stills quickly without paying an FTP round-trip
        per shot.
        """

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
        # Force a known, single-file-per-round format state before every
        # trigger -- CPTR emits one file per enabled SIxx toggle, and the
        # two cameras were found with different toggles on (left had
        # radiometric on, right had HR on). Best-effort: proceed to capture
        # either way, since a failed format command shouldn't block the shot,
        # it'll just be caught later by the sync's per-name ``-visible.jpg``
        # filter and count check.
        self.panel.calibration_status_label.setText("Capture OZ Still: 이미지 포맷 강제 설정 중")
        thread = QtCore.QThread(self.panel)
        worker = WirisImageFormatWorker(
            target_infos,
            commands=("SIRJ FALSE", "SISR FALSE", "SIHR FALSE", "SIVI TRUE"),
        )
        worker.moveToThread(thread)
        thread.started.connect(worker.run)
        worker.completed.connect(self.panel._handle_calibration_wiris_oz_format_preset_completed)
        worker.failed.connect(self.panel._handle_calibration_wiris_oz_format_preset_failed)
        worker.finished.connect(thread.quit)
        worker.finished.connect(worker.deleteLater)
        thread.finished.connect(thread.deleteLater)
        thread.finished.connect(self.panel._clear_calibration_wiris_oz_format_worker)
        self.panel.calibration_wiris_oz_format_thread = thread
        self.panel.calibration_wiris_oz_format_worker = worker
        self._connect_calibration_worker_progress(
            worker,
            operation="Capture OZ Still (format)",
        )
        self._pending_oz_capture_target_infos = target_infos
        thread.start()
        return True

    def _handle_calibration_wiris_oz_format_preset_completed(self, result: object) -> None:
        record = result if isinstance(result, dict) else {}
        for label, responses in record.items():
            self.panel._log(f"[캘리브레이션] OZ 포맷 강제 설정 {label}: {responses}")
        self._start_calibration_wiris_oz_capture()

    def _handle_calibration_wiris_oz_format_preset_failed(self, error: str) -> None:
        self.panel._log(f"[캘리브레이션] OZ 포맷 강제 설정 실패 (촬영은 계속 진행): {error}")
        self._start_calibration_wiris_oz_capture()

    def _clear_calibration_wiris_oz_format_worker(self) -> None:
        self.panel.calibration_wiris_oz_format_thread = None
        self.panel.calibration_wiris_oz_format_worker = None

    def _start_calibration_wiris_oz_capture(self) -> None:
        target_infos = getattr(self, "_pending_oz_capture_target_infos", ())
        self._pending_oz_capture_target_infos = ()
        if len(target_infos) != 2:
            return
        self._ensure_calibration_session_root("oz_capture")
        capture_id = f"calib-ozcap-{time.time_ns()}-{uuid.uuid4().hex[:8]}"
        thread = QtCore.QThread(self.panel)
        worker = WirisOZPhotoCaptureWorker(target_infos, capture_id=capture_id, completion_timeout_s=10.0)
        worker.moveToThread(thread)
        thread.started.connect(worker.run)
        worker.completed.connect(self.panel._handle_calibration_wiris_oz_capture_only_completed)
        worker.failed.connect(self.panel._handle_calibration_wiris_oz_capture_only_failed)
        worker.finished.connect(thread.quit)
        worker.finished.connect(worker.deleteLater)
        thread.finished.connect(thread.deleteLater)
        thread.finished.connect(self.panel._clear_calibration_wiris_oz_capture_only_worker)
        self.panel.calibration_wiris_oz_capture_thread = thread
        self.panel.calibration_wiris_oz_capture_worker = worker
        self._connect_calibration_worker_progress(
            worker,
            operation="Capture OZ Still",
            capture_id=capture_id,
        )
        self.panel.calibration_status_label.setText(f"Capture OZ Still {capture_id}: 좌/우 카메라 내부 저장 중")
        thread.start()

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
        left_ok = bool((cameras.get("left") or {}).get("stored_on_camera"))
        right_ok = bool((cameras.get("right") or {}).get("stored_on_camera"))
        synchronized = left_ok and right_ok
        ledger = self.panel._load_calib_oz_ledger()
        rounds = ledger["rounds"]
        rounds.append(
            {
                "capture_id": capture_id,
                "requested_at_unix_ns": min(
                    (
                        int((cameras.get(side) or {}).get("capture_requested_unix_ns"))
                        for side in ("left", "right")
                        if (cameras.get(side) or {}).get("capture_requested_unix_ns")
                    ),
                    default=time.time_ns(),
                ),
                "left": {
                    "stored_on_camera": left_ok,
                    "state": (cameras.get("left") or {}).get("state"),
                    "capture_requested_unix_ns": (cameras.get("left") or {}).get(
                        "capture_requested_unix_ns"
                    ),
                    "capture_requested_monotonic_ns": (cameras.get("left") or {}).get(
                        "capture_requested_monotonic_ns"
                    ),
                    "capture_pairing_contract": (cameras.get("left") or {}).get(
                        "capture_pairing_contract"
                    ),
                },
                "right": {
                    "stored_on_camera": right_ok,
                    "state": (cameras.get("right") or {}).get("state"),
                    "capture_requested_unix_ns": (cameras.get("right") or {}).get(
                        "capture_requested_unix_ns"
                    ),
                    "capture_requested_monotonic_ns": (cameras.get("right") or {}).get(
                        "capture_requested_monotonic_ns"
                    ),
                    "capture_pairing_contract": (cameras.get("right") or {}).get(
                        "capture_pairing_contract"
                    ),
                },
                "synchronized": synchronized,
                "downloaded": False,
                "left_remote_name": None,
                "right_remote_name": None,
            }
        )
        self.panel._save_calib_oz_ledger(ledger)
        pending = sum(1 for row in rounds if row.get("synchronized") and not row.get("downloaded"))
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
        self.panel.calibration_wiris_oz_capture_thread = None
        self.panel.calibration_wiris_oz_capture_worker = None

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
        ledger = self.panel._load_calib_oz_ledger()
        rounds = ledger["rounds"]
        pending_rounds = [row for row in rounds if row.get("synchronized") and not row.get("downloaded")]
        if not pending_rounds:
            self.panel.calibration_status_label.setText("동기화할 새 OZ 회차가 없습니다.")
            return False
        self.panel.calibration_status_label.setText("WIRIS FTP 동기화: 저장소 위치(GILC) 확인 중")
        thread = QtCore.QThread(self.panel)
        worker = WirisImageFormatWorker(target_infos, commands=("GILC",))
        worker.moveToThread(thread)
        thread.started.connect(worker.run)
        worker.completed.connect(self.panel._handle_calibration_wiris_oz_gilc_completed)
        worker.failed.connect(self.panel._handle_calibration_wiris_oz_gilc_failed)
        worker.finished.connect(thread.quit)
        worker.finished.connect(worker.deleteLater)
        thread.finished.connect(thread.deleteLater)
        thread.finished.connect(self.panel._clear_calibration_wiris_oz_gilc_worker)
        self.panel.calibration_wiris_oz_gilc_thread = thread
        self.panel.calibration_wiris_oz_gilc_worker = worker
        self._pending_ftp_oz_sync_target_infos = target_infos
        self._pending_ftp_oz_sync_rounds = pending_rounds
        thread.start()
        return True

    def _handle_calibration_wiris_oz_gilc_completed(self, result: object) -> None:
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
        root, _left_dir, _right_dir = self.panel._calibration_dirs()
        output_dir = root / "calib_capture" / "OZ"
        for side in ("left", "right"):
            (output_dir / side).mkdir(parents=True, exist_ok=True)
        kwargs = self.panel._wiris_ftp_worker_kwargs(output_dir=output_dir)
        ledger = self.panel._load_calib_oz_ledger()
        fetch_counts = {"left": len(pending_rounds), "right": len(pending_rounds)}
        thread = QtCore.QThread(self.panel)
        worker = WirisOzFtpSyncWorker(
            target_infos,
            ftp_settings_by_label=kwargs["ftp_settings_by_label"],
            storage_root_by_label=storage_root_by_label,
            ftp_output_dir=output_dir,
            fetch_counts=fetch_counts,
            downloaded_remote_names=ledger.get("consumed_remote_names") or {},
        )
        worker.moveToThread(thread)
        thread.started.connect(worker.run)
        worker.completed.connect(self.panel._handle_calibration_wiris_oz_sync_completed)
        worker.failed.connect(self.panel._handle_calibration_wiris_oz_sync_failed)
        worker.finished.connect(thread.quit)
        worker.finished.connect(worker.deleteLater)
        thread.finished.connect(thread.deleteLater)
        thread.finished.connect(self.panel._clear_calibration_wiris_oz_sync_worker)
        self.panel.calibration_wiris_oz_sync_thread = thread
        self.panel.calibration_wiris_oz_sync_worker = worker
        self._pending_ftp_oz_sync_round_ids = [str(row.get("capture_id")) for row in pending_rounds]
        self.panel.calibration_status_label.setText(
            f"WIRIS FTP 동기화: {len(pending_rounds)}쌍 다운로드 중"
        )
        thread.start()

    def _handle_calibration_wiris_oz_gilc_failed(self, error: str) -> None:
        self.panel._log(f"[캘리브레이션] WIRIS FTP 동기화: GILC 조회 실패 - {error}")
        self.panel.calibration_status_label.setText(f"WIRIS FTP sync failed: GILC query failed ({error})")
        self._pending_ftp_oz_sync_target_infos = ()
        self._pending_ftp_oz_sync_rounds = []

    def _clear_calibration_wiris_oz_gilc_worker(self) -> None:
        self.panel.calibration_wiris_oz_gilc_thread = None
        self.panel.calibration_wiris_oz_gilc_worker = None

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

        # Hard-fail the *whole* batch on any count mismatch or unreadable
        # file, rather than zipping via min(len(left), len(right)). A
        # mismatch anywhere except the tail (a round missing on one side
        # only, or a stray pre-existing remote file mixed in) would
        # otherwise get silently zipped against the wrong round -- exactly
        # the cross-camera mixup the ledger exists to prevent. Nothing here
        # is deleted from the camera; a failed batch can simply be retried
        # once the remote directory / ledger state is sorted out.
        left_invalid = [str(item.get("name")) for item in left_items if not item.get("valid")]
        right_invalid = [str(item.get("name")) for item in right_items if not item.get("valid")]
        if len(left_items) != len(round_ids) or len(right_items) != len(round_ids) or left_invalid or right_invalid:
            self.panel._log(
                "[캘리브레이션] WIRIS FTP 동기화 실패: 좌우 회차 수/유효성이 요청과 일치하지 않아 "
                f"배치 전체를 건너뜁니다 (요청={len(round_ids)}, left={len(left_items)}개"
                f"(무효 {left_invalid}), right={len(right_items)}개(무효 {right_invalid})). "
                "카메라 원본은 삭제하지 않았습니다 -- 다른 캡처 채널이 같은 원격 디렉토리에 "
                "파일을 남기지 않았는지 확인 후 다시 시도하세요."
            )
            self.panel.calibration_status_label.setText(
                "WIRIS FTP sync failed: 좌우 회차가 정확히 매칭되지 않아 중단했습니다 (로그 참고)."
            )
            return
        ledger = self.panel._load_calib_oz_ledger()
        rounds_by_id = {str(row.get("capture_id")): row for row in ledger["rounds"]}
        consumed = ledger.setdefault("consumed_remote_names", {"left": [], "right": []})
        queue: list[
            tuple[str, dict[str, QtGui.QImage], dict[str, dict[str, object]]]
        ] = []
        delete_names: dict[str, list[str]] = {"left": [], "right": []}
        for index, capture_id in enumerate(round_ids):
            left_name, right_name = str(left_items[index]["name"]), str(right_items[index]["name"])
            row = rounds_by_id.get(capture_id)
            if row is not None:
                row["downloaded"] = True
                row["left_remote_name"] = left_name
                row["right_remote_name"] = right_name
            consumed.setdefault("left", []).append(left_name)
            consumed.setdefault("right", []).append(right_name)
            delete_names["left"].append(left_name)
            delete_names["right"].append(right_name)
            frames = {
                "left": QtGui.QImage(str(left_items[index]["path"])),
                "right": QtGui.QImage(str(right_items[index]["path"])),
            }
            lifecycle = {
                "left": {
                    "ftp_remote_path": left_name,
                    "capture_requested_unix_ns": None if row is None else (
                        (row.get("left") or {}).get("capture_requested_unix_ns")
                        or row.get("requested_at_unix_ns")
                    ),
                    "capture_requested_monotonic_ns": None if row is None else (
                        (row.get("left") or {}).get("capture_requested_monotonic_ns")
                    ),
                    "capture_pairing_contract": None if row is None else (
                        (row.get("left") or {}).get("capture_pairing_contract")
                    ),
                },
                "right": {
                    "ftp_remote_path": right_name,
                    "capture_requested_unix_ns": None if row is None else (
                        (row.get("right") or {}).get("capture_requested_unix_ns")
                        or row.get("requested_at_unix_ns")
                    ),
                    "capture_requested_monotonic_ns": None if row is None else (
                        (row.get("right") or {}).get("capture_requested_monotonic_ns")
                    ),
                    "capture_pairing_contract": None if row is None else (
                        (row.get("right") or {}).get("capture_pairing_contract")
                    ),
                },
            }
            queue.append(
                (
                    capture_id,
                    frames,
                    self._ftp_calibration_capture_metadata(
                        capture_id=capture_id,
                        cameras=lifecycle,
                        frames=frames,
                    ),
                )
            )
        self.panel._save_calib_oz_ledger(ledger)
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
        self.panel.calibration_wiris_oz_sync_thread = None
        self.panel.calibration_wiris_oz_sync_worker = None

    def _start_calibration_wiris_oz_delete(self, names_by_label: dict[str, list[str]]) -> None:
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
        thread = QtCore.QThread(self.panel)
        worker = WirisFtpBatchDeleteWorker(
            target_infos,
            ftp_settings_by_label=kwargs["ftp_settings_by_label"],
            names_by_label=names_by_label,
        )
        worker.moveToThread(thread)
        thread.started.connect(worker.run)
        worker.completed.connect(self.panel._handle_calibration_wiris_oz_delete_completed)
        worker.failed.connect(self.panel._handle_calibration_wiris_oz_delete_failed)
        worker.finished.connect(thread.quit)
        worker.finished.connect(worker.deleteLater)
        thread.finished.connect(thread.deleteLater)
        thread.finished.connect(self.panel._clear_calibration_wiris_oz_delete_worker)
        self.panel.calibration_wiris_oz_delete_thread = thread
        self.panel.calibration_wiris_oz_delete_worker = worker
        thread.start()

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
        self.panel.calibration_wiris_oz_delete_thread = None
        self.panel.calibration_wiris_oz_delete_worker = None

    def _process_next_ftp_oz_sync_pair(self) -> None:
        queue = getattr(self, "_pending_ftp_oz_sync_frames", None)
        if not queue:
            saved = getattr(self, "_ftp_oz_sync_saved_count", 0)
            self.panel._log(f"[캘리브레이션] WIRIS FTP 동기화 완료: {saved}쌍 저장")
            self.panel.calibration_status_label.setText(f"WIRIS FTP 동기화 완료: {saved}쌍 저장")
            return
        capture_id, frames, metadata_by_camera = queue.pop(0)
        started = self.panel.capture_calibration_images(
            frames=frames,
            metadata_by_camera=metadata_by_camera,
            capture_origin="ftp_oz_sync",
        )
        if not started:
            QtCore.QTimer.singleShot(200, self.panel._process_next_ftp_oz_sync_pair)
            return
        self._ftp_oz_sync_saved_count = getattr(self, "_ftp_oz_sync_saved_count", 0) + 1
        thread = self.panel.checkerboard_calibration_thread
        if thread is not None:
            thread.finished.connect(self.panel._process_next_ftp_oz_sync_pair)
        else:
            QtCore.QTimer.singleShot(200, self.panel._process_next_ftp_oz_sync_pair)

    # -- Manual calculation-pair selection --------------------------------

    def _calibration_pair_selection_path(self) -> Path:
        root, _left_dir, _right_dir = self.panel._calibration_dirs()
        return root / "calibration_pair_selection.json"

    def _load_calibration_pair_selection(self) -> dict[str, object]:
        path = self.panel._calibration_pair_selection_path()
        return load_calibration_selection_sidecar(path).to_mapping()

    @staticmethod
    def _resolve_calibration_selection_paths(
        raw_paths: object,
        root: Path,
        *,
        stored_root: object = None,
    ) -> list[Path]:
        paths, _errors = resolve_calibration_selection_paths(
            raw_paths,
            root,
            stored_root=stored_root,
        )
        return paths

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

    def _selected_calibration_file_lists(
        self,
        root: Path,
        fallback_left: list[Path],
        fallback_right: list[Path],
        *,
        selection: Mapping[str, object] | None = None,
    ) -> tuple[list[Path], list[Path], bool]:
        selection = (
            self.panel._load_calibration_pair_selection()
            if selection is None
            else dict(selection)
        )
        if str(selection.get("_sidecar_state") or "") == "INVALID":
            errors = tuple(str(value) for value in selection.get("_sidecar_errors") or ())
            self._report_calibration_selection_errors(errors)
            return [], [], True
        stored_root = selection.get("root")
        excluded_left_paths, excluded_left_errors = resolve_calibration_selection_paths(
            selection.get("excluded_left_paths"), root, stored_root=stored_root
        )
        excluded_right_paths, excluded_right_errors = resolve_calibration_selection_paths(
            selection.get("excluded_right_paths"), root, stored_root=stored_root
        )
        left_paths, left_errors = resolve_calibration_selection_paths(
            selection.get("left_paths"), root, stored_root=stored_root
        )
        right_paths, right_errors = resolve_calibration_selection_paths(
            selection.get("right_paths"), root, stored_root=stored_root
        )
        errors = (
            *excluded_left_errors,
            *excluded_right_errors,
            *left_errors,
            *right_errors,
        )
        if errors:
            self._report_calibration_selection_errors(errors)
            return [], [], True
        excluded_left = set(excluded_left_paths)
        excluded_right = set(excluded_right_paths)
        if not selection.get("manual_mode"):
            return (
                [path for path in fallback_left if path not in excluded_left],
                [path for path in fallback_right if path not in excluded_right],
                False,
            )
        if not left_paths and not right_paths:
            included = {str(key) for key in selection.get("included_keys") or ()}
            if included:
                left_paths = [
                    path
                    for path in fallback_left
                    if path.stem.removeprefix("left_") in included
                ]
                right_paths = [
                    path
                    for path in fallback_right
                    if path.stem.removeprefix("right_") in included
                ]
            else:
                self._report_calibration_selection_errors(
                    ("manual selection contains no recoverable files or pair keys",)
                )
                return [], [], True
        left = [path for path in left_paths if path not in excluded_left]
        right = [path for path in right_paths if path not in excluded_right]
        return left, right, True

    @staticmethod
    def _selected_calibration_pairs(
        left_paths: list[Path],
        right_paths: list[Path],
    ) -> list[tuple[str, Path, Path]]:
        """Pair explicit selections by row, with key matching for unequal lists."""

        def key(path: Path, side: str) -> str:
            return path.stem.removeprefix(f"{side}_")

        right_by_key: dict[str, list[Path]] = {}
        for path in right_paths:
            right_by_key.setdefault(key(path, "right"), []).append(path)
        common_pairs: list[tuple[str, Path, Path]] = []
        for left in left_paths:
            capture_key = key(left, "left")
            candidates = right_by_key.get(capture_key) or []
            if candidates:
                common_pairs.append((capture_key, left, candidates.pop(0)))
        if common_pairs:
            return common_pairs
        if len(left_paths) == len(right_paths):
            return [
                (f"selected-{index:04d}", left, right)
                for index, (left, right) in enumerate(zip(left_paths, right_paths, strict=False))
            ]
        right_by_key: dict[str, list[Path]] = {}
        for path in right_paths:
            key = path.stem.removeprefix("right_")
            right_by_key.setdefault(key, []).append(path)
        pairs: list[tuple[str, Path, Path]] = []
        for left in left_paths:
            key = left.stem.removeprefix("left_")
            candidates = right_by_key.get(key) or []
            if candidates:
                pairs.append((key, left, candidates.pop(0)))
        return pairs

    def _effective_calibration_inputs(
        self,
        root: Path,
        fallback_left: list[Path],
        fallback_right: list[Path],
        scanned_pairs: list[tuple[str, Path, Path]],
        *,
        mode: str,
        selection: Mapping[str, object] | None = None,
        pair_limit: int | None = None,
    ) -> tuple[list[Path], list[Path], list[tuple[str, Path, Path]], bool]:
        """Resolve one immutable calculation input set from the selection sidecar.

        Intrinsic selection belongs to each camera independently. Stereo R,t
        receives only ordered pairs whose left and right files both survived
        those independent selections. This prevents excluding a left outlier
        from silently deleting its otherwise useful right-camera observation.
        """

        left_paths, right_paths, explicit_selection = self._selected_calibration_file_lists(
            root,
            fallback_left,
            fallback_right,
            selection=selection,
        )
        left_members = set(left_paths)
        right_members = set(right_paths)
        pair_candidates = [
            pair
            for pair in scanned_pairs
            if pair[1] in left_members and pair[2] in right_members
        ]
        selected_pairs = self._choose_calibration_pairs(
            pair_candidates,
            selection=selection,
            root=root,
            pair_limit=pair_limit,
        )
        selected_pairs = [
            pair
            for pair in selected_pairs
            if self._calibration_pair_stereo_eligible(root, pair[1], pair[2])
        ]
        return left_paths, right_paths, selected_pairs, explicit_selection

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

    @staticmethod
    def _solver_effective_selection(
        result: StereoIntrinsicsResult,
    ) -> tuple[tuple[Path, ...], tuple[Path, ...], tuple[tuple[str, Path, Path], ...]] | None:
        """Read the exact post-deduplication input set emitted by the solver."""

        diagnostics = result.solver_diagnostics
        if not isinstance(diagnostics, dict):
            return None
        payload = diagnostics.get("effective_selection")
        if not isinstance(payload, dict):
            return None
        raw_left = payload.get("left_intrinsic_paths")
        raw_right = payload.get("right_intrinsic_paths")
        raw_pairs = payload.get("stereo_pairs")
        if not isinstance(raw_left, (list, tuple)) or not isinstance(
            raw_right, (list, tuple)
        ) or not isinstance(raw_pairs, (list, tuple)):
            return None
        pairs: list[tuple[str, Path, Path]] = []
        for row in raw_pairs:
            if not isinstance(row, (list, tuple)) or len(row) != 3:
                return None
            pairs.append((str(row[0]), Path(str(row[1])), Path(str(row[2]))))
        return (
            tuple(Path(str(path)) for path in raw_left),
            tuple(Path(str(path)) for path in raw_right),
            tuple(pairs),
        )

    def _choose_calibration_pairs(
        self,
        pairs: list[tuple[str, Path, Path]],
        *,
        selection: Mapping[str, object] | None = None,
        root: Path | None = None,
        pair_limit: int | None = None,
    ) -> list[tuple[str, Path, Path]]:
        """Manual picker override of ``_select_calibration_pairs``.

        Falls back to the existing even-distribution auto-selection unless
        the operator has switched on manual selection in the picker (which
        writes ``calibration_pair_selection.json`` -- the picker lives on the
        visible workspace panel, calculation runs on this hidden one, so a
        JSON sidecar is the handoff instead of a cross-panel command).
        """

        selection = (
            self.panel._load_calibration_pair_selection()
            if selection is None
            else dict(selection)
        )
        if str(selection.get("_sidecar_state") or "") == "INVALID":
            errors = tuple(str(value) for value in selection.get("_sidecar_errors") or ())
            self._report_calibration_selection_errors(errors)
            return []
        root = self.panel._calibration_dirs()[0] if root is None else Path(root)
        stored_root = selection.get("root")
        explicit_left, left_errors = resolve_calibration_selection_paths(
            selection.get("left_paths"),
            root,
            stored_root=stored_root,
        )
        explicit_right, right_errors = resolve_calibration_selection_paths(
            selection.get("right_paths"),
            root,
            stored_root=stored_root,
        )
        excluded_left_paths, excluded_left_errors = resolve_calibration_selection_paths(
            selection.get("excluded_left_paths"), root, stored_root=stored_root
        )
        excluded_right_paths, excluded_right_errors = resolve_calibration_selection_paths(
            selection.get("excluded_right_paths"), root, stored_root=stored_root
        )
        errors = (
            *left_errors,
            *right_errors,
            *excluded_left_errors,
            *excluded_right_errors,
        )
        if errors:
            self._report_calibration_selection_errors(errors)
            return []
        manual_mode = bool(selection.get("manual_mode"))
        if manual_mode and (explicit_left or explicit_right):
            candidates = self._selected_calibration_pairs(explicit_left, explicit_right)
        elif not manual_mode:
            candidates = list(pairs)
        else:
            included = set(selection.get("included_keys") or ())
            candidates = [pair for pair in pairs if pair[0] in included]
        excluded_left = set(excluded_left_paths)
        excluded_right = set(excluded_right_paths)
        excluded_keys = {str(key) for key in selection.get("excluded_pair_keys") or ()}
        eligible = [
            pair
            for pair in candidates
            if pair[0] not in excluded_keys
            and pair[1] not in excluded_left
            and pair[2] not in excluded_right
        ]
        # ``calibration_calculation_pair_count`` is a stereo-pair limit. Apply
        # it once, after exclusions and side membership have established the
        # usable intersection. Sampling before that intersection can discard
        # many otherwise valid pairs when only one camera has an exclusion.
        return (
            self._select_calibration_pairs(eligible, pair_limit=pair_limit)
            if not manual_mode
            else eligible
        )

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
        self._auto_capture_active = True
        self._auto_capture_saved_pairs = 0
        self._auto_capture_last_pair_key = None
        self._auto_capture_inflight_pair_key = None
        self._auto_capture_pose_views = {"left": [], "right": []}
        self._auto_capture_pending_pose_views = None
        self._auto_capture_pending_pose_novelty = None
        self._auto_capture_last_attempt_s = 0.0
        self._auto_capture_timer.start(max(100, int(interval_s * 1000.0)))
        self.panel.capture_calibration_auto_button.setText("자동 촬영 중지")
        self.panel.calibration_status_label.setText(
            f"자동 촬영 대기: 유효 stereo pair 0개 | "
            f"간격={interval_s:.1f}s | Stop할 때까지 계속 촬영합니다."
        )
        self._auto_calibration_capture_tick()
        return True

    def stop_auto_calibration_capture(self, message: str | None = None) -> bool:
        was_active = self._auto_capture_active or self._auto_capture_timer.isActive()
        self._auto_capture_timer.stop()
        self._auto_capture_active = False
        self._auto_capture_inflight_pair_key = None
        self._auto_capture_pending_pose_views = None
        self._auto_capture_pending_pose_novelty = None
        if hasattr(self.panel, "capture_calibration_auto_button"):
            self.panel.capture_calibration_auto_button.setText("자동 촬영 시작")
        if message is not None:
            self.panel.calibration_status_label.setText(message)
        elif was_active:
            self.panel.calibration_status_label.setText(
                f"자동 촬영 중지: 유효 stereo pair {self._auto_capture_saved_pairs}개 저장"
            )
        return True

    def _auto_calibration_capture_tick(self) -> None:
        if not self._auto_capture_active:
            return
        if self.panel.checkerboard_calibration_thread is not None:
            return

        detections = getattr(self.panel, "calibration_preview_detection_results", {})
        left = detections.get("left") if isinstance(detections, dict) else None
        right = detections.get("right") if isinstance(detections, dict) else None
        if not isinstance(left, dict) or not isinstance(right, dict):
            self.panel.calibration_status_label.setText("자동 촬영 대기: 좌·우 체커보드 검출 결과를 기다리는 중")
            return
        if not bool(left.get("found")) or not bool(right.get("found")):
            self.panel.calibration_status_label.setText("자동 촬영 대기: 좌·우 화면에서 체커보드가 모두 검출되어야 합니다")
            return
        left_image = left.get("image")
        right_image = right.get("image")
        if not isinstance(left_image, QtGui.QImage) or not isinstance(right_image, QtGui.QImage):
            self.panel.calibration_status_label.setText("자동 촬영 대기: 좌·우 최신 프레임을 기다리는 중")
            return

        try:
            left_request_id = int(left.get("request_id") or 0)
            right_request_id = int(right.get("request_id") or 0)
        except (TypeError, ValueError):
            return
        if left_request_id <= 0 or right_request_id <= 0:
            return
        pair_key = (left_request_id, right_request_id)
        if pair_key == self._auto_capture_last_pair_key:
            return

        preview_metadata = {
            "left": dict(left.get("metadata") or {}) if isinstance(left.get("metadata"), dict) else {},
            "right": dict(right.get("metadata") or {}) if isinstance(right.get("metadata"), dict) else {},
        }
        allow_preview_fallback = bool(
            getattr(self.panel, "_allow_legacy_calibration_preview_fallback", False)
        )
        if not allow_preview_fallback:
            missing_nal1_sides = [
                side
                for side in ("left", "right")
                if not is_nal1_non_idr_auto_capture_metadata(
                    preview_metadata[side]
                )
            ]
            if missing_nal1_sides:
                self.panel.calibration_status_label.setText(
                    "자동 촬영 대기: "
                    + ", ".join(missing_nal1_sides)
                    + " NAL1 non-delta frame을 기다리는 중"
                )
                return
        native_boundary_available = all(
            callable(
                getattr(
                    getattr(self.panel, f"{side}_pane", None),
                    "calibration_native_frame_for_key",
                    None,
                )
            )
            for side in ("left", "right")
        )
        corners_available = all(bool(row.get("corners")) for row in (left, right))
        try:
            pose_views = {
                "left": extract_view_geometry(
                    left.get("corners") or (),
                    (left_image.width(), left_image.height()),
                    pattern_size=self.panel._checkerboard_config().pattern_size,
                    view_id=f"preview-left-{left_request_id}",
                ),
                "right": extract_view_geometry(
                    right.get("corners") or (),
                    (right_image.width(), right_image.height()),
                    pattern_size=self.panel._checkerboard_config().pattern_size,
                    view_id=f"preview-right-{right_request_id}",
                ),
            } if corners_available else {}
        except (TypeError, ValueError) as exc:
            self.panel.calibration_status_label.setText(
                f"자동 촬영 대기: checkerboard pose 분석 실패 ({exc})"
            )
            return
        if (native_boundary_available or not allow_preview_fallback) and not pose_views:
            self.panel.calibration_status_label.setText(
                "자동 촬영 대기: checkerboard corner geometry가 없습니다"
            )
            return
        reference_views = getattr(
            self,
            "_auto_capture_pose_views",
            {"left": [], "right": []},
        )
        novelty = {
            side: pose_novelty(pose_views[side], reference_views.get(side, ()))
            for side in ("left", "right")
        } if pose_views else {"left": 2.0, "right": 2.0}
        if any(value <= 1.0 for value in novelty.values()):
            duplicate_sides = ", ".join(
                side for side, value in novelty.items() if value <= 1.0
            )
            self.panel.calibration_status_label.setText(
                f"자동 촬영 대기: {duplicate_sides} checkerboard pose가 이전과 거의 같습니다. "
                "위치·거리·yaw/pitch/roll을 바꾸세요."
            )
            return

        native_frames: dict[str, QtGui.QImage] = {}
        metadata_by_camera: dict[str, dict[str, object]] = {}
        for side in ("left", "right"):
            pane = getattr(self.panel, f"{side}_pane", None)
            native_getter = getattr(pane, "calibration_native_frame_for_key", None)
            if not callable(native_getter):
                if not allow_preview_fallback:
                    self.panel.calibration_status_label.setText(
                        f"자동 촬영 대기: {side} exact FrameKey API가 없습니다"
                    )
                    return
                preview_image = left_image if side == "left" else right_image
                native_frames[side] = preview_image
                metadata_by_camera[side] = dict(preview_metadata[side])
                continue
            try:
                frame_key = FrameKey.from_mapping(preview_metadata[side])
            except (TypeError, ValueError):
                self.panel.calibration_status_label.setText(
                    f"자동 촬영 대기: {side} exact FrameKey가 없습니다"
                )
                return
            native = native_getter(
                frame_key,
                maximum_age_s=CALIBRATION_AUTO_DETECTION_MAX_AGE_S,
            )
            if native is None:
                self.panel.calibration_status_label.setText(
                    f"자동 촬영 대기: {side} exact native FrameKey가 만료되어 새 NAL1 frame을 기다리는 중"
                )
                return
            frame_id, native_image, native_metadata = native
            if (
                str(frame_id) != frame_key.text
                or str(native_metadata.get("frame_key") or "") != frame_key.text
                or not is_nal1_non_idr_auto_capture_metadata(native_metadata)
                or int(native_metadata["h264_access_unit_pts_ns"])
                != int(preview_metadata[side]["h264_access_unit_pts_ns"])
            ):
                self.panel.calibration_status_label.setText(
                    f"자동 촬영 대기: {side} NAL1/FrameKey provenance가 일치하지 않습니다"
                )
                return
            native_frames[side] = native_image
            metadata_by_camera[side] = {
                **dict(native_metadata),
                "frame_id": str(frame_id),
                "calibration_source": "native_frame_cache",
                "preview_width": left_image.width() if side == "left" else right_image.width(),
                "preview_height": left_image.height() if side == "left" else right_image.height(),
            }
        left_image = native_frames["left"]
        right_image = native_frames["right"]
        now_s = time.monotonic()
        for camera in ("left", "right"):
            timestamp_s = metadata_by_camera[camera].get("timestamp_s")
            try:
                age_s = now_s - float(timestamp_s)
            except (TypeError, ValueError):
                self.panel.calibration_status_label.setText(
                    "자동 촬영 대기: 좌·우 프레임의 수신 시각이 없습니다"
                )
                return
            if age_s < 0.0 or age_s > CALIBRATION_AUTO_DETECTION_MAX_AGE_S:
                self.panel.calibration_status_label.setText(
                    "자동 촬영 대기: 체커보드 검출 프레임이 오래되어 새 프레임을 기다리는 중"
                )
                return

        pair_timing_ok, _pair_time_basis, _pair_delta_s, pair_timing_detail = (
            self.panel._checkerboard_capture_pair_timing(metadata_by_camera)
        )

        self._auto_capture_last_pair_key = pair_key
        self._auto_capture_last_attempt_s = now_s
        self._auto_capture_inflight_pair_key = pair_key
        self._auto_capture_pending_pose_views = pose_views
        self._auto_capture_pending_pose_novelty = novelty
        started = self.capture_calibration_images(
            frames={"left": left_image, "right": right_image},
            metadata_by_camera=metadata_by_camera,
            capture_origin="auto",
        )
        if not started:
            self._auto_capture_inflight_pair_key = None
            self._auto_capture_pending_pose_views = None
            self._auto_capture_pending_pose_novelty = None
            return
        usage = (
            "stereo pair 후보"
            if pair_timing_ok
            else f"intrinsics-only 후보 ({pair_timing_detail})"
        )
        self.panel.calibration_status_label.setText(
            f"자동 촬영 중: {usage} | 유효 stereo pair "
            f"{self._auto_capture_saved_pairs}개 | Capture worker가 보드 검증 중"
        )

    def _handle_checkerboard_capture_completed(self, result: object) -> None:
        capture_origin = self._capture_origin
        source_label = {
            "ftp": "FTP HR",
            "ftp_oz": "FTP OZ",
            "ftp_oz_sync": "FTP OZ sync",
        }.get(capture_origin, "")
        source_prefix = f"{source_label} " if source_label else ""
        if not isinstance(result, dict):
            self.panel._handle_checkerboard_capture_failed("worker returned an invalid capture result")
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
                self._auto_capture_inflight_pair_key = None
                self._auto_capture_pending_pose_views = None
                self._auto_capture_pending_pose_novelty = None
                self._capture_origin = "manual"
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
        session_root = getattr(self.panel, "_calibration_session_root", None)
        if session_root is not None:
            try:
                write_calibration_session_manifest(session_root, source=capture_origin)
            except OSError:
                pass
        self.panel._log(
            f"[캘리브레이션] {source_prefix}이미지 캡처 {stem}: "
            f"저장={','.join(saved_sides)}; {', '.join(status_parts)}"
        )
        if capture_origin == "auto":
            self._auto_capture_inflight_pair_key = None
            count_toward_auto_run = pair_saved and self._auto_capture_active
            if count_toward_auto_run:
                self._auto_capture_saved_pairs += 1
            if self._auto_capture_active:
                pending_views = getattr(self, "_auto_capture_pending_pose_views", None)
                if isinstance(pending_views, dict):
                    pose_views = getattr(
                        self,
                        "_auto_capture_pose_views",
                        {"left": [], "right": []},
                    )
                    for side in saved_sides:
                        view = pending_views.get(side)
                        if isinstance(view, CalibrationViewGeometry):
                            pose_views.setdefault(side, []).append(view)
                    self._auto_capture_pose_views = pose_views
            self._auto_capture_pending_pose_views = None
            self._auto_capture_pending_pose_novelty = None
            self._capture_origin = "manual"
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
            self._auto_capture_inflight_pair_key = None
            self._auto_capture_pending_pose_views = None
            self._auto_capture_pending_pose_novelty = None
            self._capture_origin = "manual"

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
        fallback_left = self._calibration_image_paths_for_root(root, "left")
        fallback_right = self._calibration_image_paths_for_root(root, "right")
        scanned_pairs = self._calibration_stereo_pair_paths_for_root(root)
        sidecar_path = root / "calibration_pair_selection.json"
        document = load_calibration_selection_sidecar(sidecar_path)
        selection = document.to_mapping()
        if document.state == "INVALID":
            self._report_calibration_selection_errors(document.errors)
            raise ValueError("; ".join(document.errors))
        left_paths, right_paths, selected_pairs, explicit = (
            self._effective_calibration_inputs(
                root,
                fallback_left,
                fallback_right,
                scanned_pairs,
                mode=mode,
                selection=selection,
                pair_limit=pair_limit,
            )
        )
        if not left_paths or not right_paths:
            raise ValueError(
                f"left/right calibration images are required: left={len(left_paths)}, "
                f"right={len(right_paths)}"
            )
        if mode == "checkerboard" and len(selected_pairs) < 3:
            raise ValueError(
                f"checkerboard calibration requires at least 3 stereo pairs; "
                f"found {len(selected_pairs)}"
            )
        stored_root = selection.get("root")
        excluded_left, excluded_left_errors = resolve_calibration_selection_paths(
            selection.get("excluded_left_paths"), root, stored_root=stored_root
        )
        excluded_right, excluded_right_errors = resolve_calibration_selection_paths(
            selection.get("excluded_right_paths"), root, stored_root=stored_root
        )
        if excluded_left_errors or excluded_right_errors:
            errors = (*excluded_left_errors, *excluded_right_errors)
            self._report_calibration_selection_errors(errors)
            raise ValueError("; ".join(errors))
        snapshot = build_calibration_input_snapshot(
            root=root,
            mode=mode,
            left_paths=left_paths,
            right_paths=right_paths,
            stereo_pairs=selected_pairs,
            included_pair_keys=tuple(selection.get("included_keys") or ()),
            excluded_pair_keys=tuple(selection.get("excluded_pair_keys") or ()),
            excluded_left_paths=excluded_left,
            excluded_right_paths=excluded_right,
            selection_policy=("manual_explicit" if explicit else "automatic_even_spread"),
            pair_limit=pair_limit,
            sidecar_state=document.state,
            sidecar_schema=str(selection.get("schema") or ""),
            sidecar_path=sidecar_path,
        )
        return snapshot, len(scanned_pairs), explicit

    def run_intrinsic_calibration(
        self,
        input_request: Mapping[str, object] | None = None,
    ) -> None:
        self._ensure_calibration_session_root("calculation")
        if self._auto_capture_active:
            self.stop_auto_calibration_capture()
        if self.panel.checkerboard_calibration_thread is not None:
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
        worker = CheckerboardCalibrationWorker(
            mode=mode,
            config=self.panel._checkerboard_config(),
            input_snapshot=input_snapshot,
        )
        thread = QtCore.QThread(self.panel)
        worker.moveToThread(thread)
        thread.started.connect(worker.run)
        worker.completed.connect(
            lambda result,
            mode=mode,
            root=root,
            pair_count=pair_count,
            min_images=min_images,
            capture_zoom_level=capture_zoom_level,
            left_count=len(left_paths),
            right_count=len(right_paths),
            selection_snapshot=selection_snapshot: self.panel._handle_intrinsic_calibration_completed(
                result,
                mode=mode,
                root=root,
                pair_count=pair_count,
                min_images=min_images,
                capture_zoom_level=capture_zoom_level,
                left_count=left_count,
                right_count=right_count,
                selection_snapshot=selection_snapshot,
            )
        )
        worker.progress.connect(self.panel._handle_calibration_progress)
        worker.failed.connect(self.panel._handle_intrinsic_calibration_failed)
        worker.finished.connect(thread.quit)
        worker.finished.connect(worker.deleteLater)
        thread.finished.connect(lambda: self.panel._clear_checkerboard_calibration_worker(thread))
        self.panel.checkerboard_calibration_thread = thread
        self.panel.checkerboard_calibration_worker = worker
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
        thread.start()

    def run_stereo_extrinsics_calibration(
        self,
        input_request: Mapping[str, object] | None = None,
    ) -> None:
        """Run only OpenCV stereo R/T with the selected K/D held fixed."""

        self._ensure_calibration_session_root("calculation")

        if self._auto_capture_active:
            self.stop_auto_calibration_capture()
        if self.panel.checkerboard_calibration_thread is not None:
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
        worker = CheckerboardExtrinsicsWorker(
            config=self.panel._checkerboard_config(),
            left_intrinsics=intrinsics.left,
            right_intrinsics=intrinsics.right,
            input_snapshot=input_snapshot,
        )
        thread = QtCore.QThread(self.panel)
        worker.moveToThread(thread)
        thread.started.connect(worker.run)
        worker.completed.connect(
            lambda result,
            source_zoom=source_zoom,
            source_path=source_path,
            source_hash=source_hash,
            root=root,
            input_snapshot=input_snapshot: self.panel._handle_stereo_extrinsics_completed(
                result,
                source_zoom=source_zoom,
                source_path=source_path,
                source_hash=source_hash,
                root=root,
                input_snapshot=input_snapshot,
            )
        )
        worker.progress.connect(self.panel._handle_calibration_progress)
        worker.failed.connect(self.panel._handle_intrinsic_calibration_failed)
        worker.finished.connect(thread.quit)
        worker.finished.connect(worker.deleteLater)
        thread.finished.connect(lambda: self.panel._clear_checkerboard_calibration_worker(thread))
        self.panel.checkerboard_calibration_thread = thread
        self.panel.checkerboard_calibration_worker = worker
        self.panel._set_health_label(
            self.panel.checkerboard_validation_status_label,
            f"R,t only 계산 중: fixed K,D zoom={source_zoom}, pairs={len(selected_pairs)}",
            "warn",
        )
        thread.start()

    def _handle_stereo_extrinsics_completed(
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
            self.panel._handle_intrinsic_calibration_failed(
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
        approved = quality_state in {"PASS", "UNASSESSED"}
        if not approved:
            attempts = output.parent / "attempts"
            attempts.mkdir(parents=True, exist_ok=True)
            output = attempts / (
                f"{output.stem}_{quality_state.lower()}_{time.time_ns()}{output.suffix}"
            )
        output.parent.mkdir(parents=True, exist_ok=True)
        save_stereo_extrinsics_json(output, profile)
        self.panel._last_calibration_attempt_path = str(output)
        if approved:
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
            profile = load_stereo_extrinsics_json(path)
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

    def _handle_intrinsic_calibration_completed(
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
            self.panel._handle_intrinsic_calibration_failed("worker returned an invalid calibration result")
            return
        if result.left.image_count < min_images or result.right.image_count < min_images:
            quality = dict(result.calibration_quality or {})
            reason_codes = list(quality.get("reason_codes", ()) or ())
            reasons = list(quality.get("reasons", ()) or ())
            code = "configured_minimum_effective_poses_not_met"
            if code not in reason_codes:
                reason_codes.append(code)
                reasons.append(
                    {
                        "severity": "FAIL",
                        "code": code,
                        "message": (
                            f"effective poses left={result.left.image_count}, "
                            f"right={result.right.image_count}; configured minimum={min_images}"
                        ),
                    }
                )
            quality.update(state="FAIL", reason_codes=reason_codes, reasons=reasons)
            result = replace(result, calibration_quality=quality)
        warning = (
            f"\nWARNING: valid images below requested minimum {min_images}."
            if result.left.image_count < min_images or result.right.image_count < min_images
            else ""
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
        if self.panel.checkerboard_calibration_thread is thread:
            self.panel.checkerboard_calibration_thread = None
            self.panel.checkerboard_calibration_worker = None
            if hasattr(self.panel, "capture_calibration_button"):
                self.panel.capture_calibration_button.setEnabled(True)
            if hasattr(self.panel, "run_calibration_button"):
                self.panel.run_calibration_button.setEnabled(True)
            if hasattr(self.panel, "checkerboard_validation_random_button"):
                self.panel.checkerboard_validation_random_button.setEnabled(
                    self.panel.checkerboard_validation_result is not None
                )
        thread.deleteLater()
