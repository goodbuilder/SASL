"""Calibration feature: owned state/views and explicit dependency ports.

Coordinate values retain their producer contracts (NED, body FRD, optical RDF,
and baseline-center FRD). This ownership change performs no axis conversion.
"""
from __future__ import annotations

from Workstation.panel.panel_feature_runtime import PanelFeature
from Workstation.calibration_workflow.calibration_geometry_state import CalibrationGeometryState, restore_geometry_values
from Workstation.workstation_core.active_geometry import ActiveGeometryStore
import time
from PyQt5 import (
    QtCore,
    QtGui,
    QtWidgets,
)
from Workstation.controllers.calibration_controller import is_nal1_non_idr_auto_capture_metadata
from Workstation.workers.calibration_workers import CheckerboardPreviewWorker
from Workstation.runtime_paths import (
    import_workstation_asset_file,
    portable_workstation_data_path_text,
)
from Workstation.workstation_core.calibration import (
    StereoExtrinsicsProfile,
    StereoIntrinsicsResult,
    load_intrinsics_json,
    save_intrinsics_json,
    validate_intrinsics_result,
    without_stereo_extrinsics,
)
from Workstation.workstation_core.runtime_camera_status import (
    infer_runtime_camera_timestamp_reference,
)
from Workstation.workstation_core.settings import (
    DEFAULT_CALIBRATION_ROOT,
)
from Workstation.workstation_core.stereo import (
    DetectionObservation,
    correspondence_advisories,
    correspondence_rejection_reason,
)
from Workstation.workstation_core.sync import (
    match_nearest_pair,
)
from Workstation.workstation_core.theme import (
    ACCENT,
    ERROR,
)
from collections import (
    deque,
)
from math import (
    isfinite,
)
from pathlib import (
    Path,
)


from Workstation.calibration_workflow.jobs import CalibrationJobs
from Workstation.calibration_workflow.state import CalibrationRuntimeState
from Workstation.calibration_workflow.profiles import CalibrationProfiles
from Workstation.calibration_workflow.quality import CalibrationQuality


class CalibrationFeature(PanelFeature):
    def __init__(self, widget, *, geometry_store=None):
        store = ActiveGeometryStore() if geometry_store is None else geometry_store
        super().__init__(widget, state_factory=lambda owner: CalibrationRuntimeState(store, CalibrationJobs(owner)))

    def resource_inventory(self):
        return {**super().resource_inventory(), **self.state.calibration_jobs.resource_inventory(), **self.state.ftp_transactions.resource_inventory(), **self.state.capture_runtime.resource_inventory()}

    feature_name = 'calibration'
    dependency_names = (
        '_active_extrinsics_source_zoom_level',
        '_add_zoom_intrinsics_comparison_path',
        '_archive_loaded_intrinsics',
        '_auto_capture_active',
        '_auto_load_intrinsics_candidate',
        '_build_calibration_preview_label',
        '_build_parameter_text',
        '_calibration_dirs',
        '_calibration_workflow_controller_owner',
        '_capture_recording_clock_sync_start_observation',
        '_checkerboard_config',
        '_clear_selected_runtime_measurement_pair',
        '_coordinated_detection_pair',
        '_decorate_clock_epoch_metadata',
        '_detection_pair_coordinator_active',
        '_intrinsics_file_zoom_level',
        '_intrinsics_storage_base_path',
        '_intrinsics_storage_path',
        '_is_clock_sync_tab_visible',
        '_last_correspondence_rejection_reason',
        '_last_pair_advisories',
        '_load_current_zoom_intrinsics_for_apply',
        '_log',
        '_measurement_detection_started_monotonic_s',
        '_publish_immutable_intrinsics_profile',
        '_record_clock_offset_sample',
        '_record_gui_camera_frame',
        '_refresh_active_recording_sheet',
        '_refresh_camera_parameters_tab',
        '_refresh_operation_header',
        '_refresh_zoom_intrinsics_tab',
        '_runtime_pair_uses_app_fallback',
        '_save_current_settings',
        '_set_combo_data',
        '_stop_calibration_preview_worker',
        '_update_clock_sync_display',
        '_workstation_setting',
        '_yolo_match_timestamp_s',
        '_yolo_pair_max_delta_s',
        '_yolo_vertical_max_delta_px',
        'add_zoom_intrinsics_comparison_files',
        'app_root',
        'apply_checkerboard_validation_result',
        'apply_intrinsics_to_distance',
        'apply_selected_zoom_intrinsics_comparison_file',
        'calibration_preview_changed',
        'calibration_preview_requested',
        'camera_connected',
        'camera_frame_metadata',
        'camera_frame_metadata_history',
        'camera_frame_times',
        'capture_calibration_images',
        'choose_checkerboard_validation_json',
        'choose_checkerboard_validation_root',
        'default_calibration_root',
        'depth_tab_index',
        'detection_buffers',
        'left_pane',
        'load_checkerboard_validation_json',
        'measurement_tabs',
        'parameter_tab_index',
        'remove_selected_zoom_intrinsics_comparison_files',
        'results_tab_index',
        'right_pane',
        'run_checkerboard_random_validation',
        'run_intrinsic_calibration',
        'save_checkerboard_validation_result',
        'scan_checkerboard_validation_files',
        'settings',
        'tabs',
        'time_tab_index',
        'toggle_auto_calibration_capture',
        'triangulation_quality_plot',
        'triangulation_quality_tab_index',
        'workstation_calibration_preview_snapshot',
        'yolo_enabled',
        'zoom_level_spin',
        'zoom_target_combo',
    )
    writable_dependencies = frozenset(('_active_extrinsics_source_zoom_level', '_add_zoom_intrinsics_comparison_path', '_archive_loaded_intrinsics', '_auto_capture_active', '_auto_load_intrinsics_candidate', '_build_calibration_preview_label', '_build_parameter_text', '_calibration_dirs', '_calibration_workflow_controller_owner', '_capture_recording_clock_sync_start_observation', '_checkerboard_config', '_clear_selected_runtime_measurement_pair', '_coordinated_detection_pair', '_detection_pair_coordinator_active', '_intrinsics_file_zoom_level', '_intrinsics_storage_base_path', '_intrinsics_storage_path', '_is_clock_sync_tab_visible', '_last_correspondence_rejection_reason', '_last_pair_advisories', '_load_current_zoom_intrinsics_for_apply', '_log', '_measurement_detection_started_monotonic_s', '_publish_immutable_intrinsics_profile', '_record_clock_offset_sample', '_record_gui_camera_frame', '_refresh_active_recording_sheet', '_refresh_camera_parameters_tab', '_refresh_operation_header', '_refresh_zoom_intrinsics_tab', '_runtime_pair_uses_app_fallback', '_save_current_settings', '_set_combo_data', '_stop_calibration_preview_worker', '_update_clock_sync_display', '_workstation_setting', '_yolo_match_timestamp_s', '_yolo_pair_max_delta_s', '_yolo_vertical_max_delta_px', 'add_zoom_intrinsics_comparison_files', 'app_root', 'apply_checkerboard_validation_result', 'apply_intrinsics_to_distance', 'apply_selected_zoom_intrinsics_comparison_file', 'calibration_preview_changed', 'calibration_preview_requested', 'camera_connected', 'camera_frame_metadata', 'camera_frame_metadata_history', 'camera_frame_times', 'capture_calibration_images', 'choose_checkerboard_validation_json', 'choose_checkerboard_validation_root', 'default_calibration_root', 'depth_tab_index', 'detection_buffers', 'left_pane', 'load_checkerboard_validation_json', 'measurement_tabs', 'parameter_tab_index', 'remove_selected_zoom_intrinsics_comparison_files', 'results_tab_index', 'right_pane', 'run_checkerboard_random_validation', 'run_intrinsic_calibration', 'save_checkerboard_validation_result', 'scan_checkerboard_validation_files', 'settings', 'tabs', 'time_tab_index', 'toggle_auto_calibration_capture', 'triangulation_quality_plot', 'triangulation_quality_tab_index', 'workstation_calibration_preview_snapshot', 'yolo_enabled', 'zoom_level_spin', 'zoom_target_combo'))

    def initialize_state(self, *, auto_start=False, auto_start_pixhawk=False, app_root=None, settings_path=None):
        self.ftp_log_lines: deque[str] = deque(maxlen=600)
        self.latest_calibration_preview_images: dict[str, QtGui.QImage] = {}
        self.calibration_preview_overlay_images: dict[str, QtGui.QImage] = {}
        self.calibration_preview_detection_results: dict[str, dict[str, object]] = {}
        self.calibration_preview_times = {
            "left": 0.0,
            "right": 0.0,
        }
        self.calibration_preview_request_ids = {
            "left": 0,
            "right": 0,
        }
        self._calibration_preview_next_request_id = 0
        self._calibration_preview_worker: CheckerboardPreviewWorker | None = None
        self._calibration_preview_thread: QtCore.QThread | None = None
        self.checkerboard_calibration_worker: QtCore.QObject | None = None
        self.checkerboard_calibration_thread: QtCore.QThread | None = None
        self.calibration_wiris_ftp_worker: QtCore.QObject | None = None
        self.calibration_wiris_ftp_thread: QtCore.QThread | None = None
        self.calibration_wiris_ftp_oz_worker: QtCore.QObject | None = None
        self.calibration_wiris_ftp_oz_thread: QtCore.QThread | None = None
        self.calibration_wiris_oz_capture_worker: QtCore.QObject | None = None
        self.calibration_wiris_oz_capture_thread: QtCore.QThread | None = None
        self.calibration_wiris_oz_sync_worker: QtCore.QObject | None = None
        self.calibration_wiris_oz_sync_thread: QtCore.QThread | None = None
        self.calibration_wiris_oz_delete_worker: QtCore.QObject | None = None
        self.calibration_wiris_oz_delete_thread: QtCore.QThread | None = None
        self.calibration_wiris_oz_format_worker: QtCore.QObject | None = None
        self.calibration_wiris_oz_format_thread: QtCore.QThread | None = None
        self.calibration_wiris_oz_gilc_worker: QtCore.QObject | None = None
        self.calibration_wiris_oz_gilc_thread: QtCore.QThread | None = None
        self._workstation_calibration_preview_active = False
        self.calibration_capture_index = 0
        self.last_calibration_result: StereoIntrinsicsResult | None = None
        self.calibration_image_diagnostics: tuple[dict[str, object], ...] = ()
        self.calibration_diagnostics_root = ""
        self.checkerboard_validation_result: StereoIntrinsicsResult | None = None
        self.checkerboard_validation_result_mode = "pixhawk"
        self.checkerboard_validation_summary: dict[str, object] = {}
        self.checkerboard_validation_run_results: list[dict[str, object]] = []
        self.checkerboard_validation_result_saved = False
        self.calibration_progress: dict[str, object] = {
            "phase": "idle",
            "processed": 0,
            "total": 0,
            "percent": 0.0,
            "message": "대기",
        }
        self.last_calibration_zoom_level: int | None = None
        self.active_intrinsics: StereoIntrinsicsResult | None = None
        self._active_intrinsics_profile_path = ""
        self._candidate_intrinsics_profile_path = ""
        self.selected_extrinsics_profile: StereoExtrinsicsProfile | None = None
        self.active_extrinsics_profile: StereoExtrinsicsProfile | None = None
        self.checkerboard_extrinsics_candidate_profile: StereoExtrinsicsProfile | None = None
        self.optical_pose_extrinsics_candidate_profile: StereoExtrinsicsProfile | None = None
        self.checkerboard_extrinsics_profile: StereoExtrinsicsProfile | None = None
        self.optical_pose_extrinsics_profile: StereoExtrinsicsProfile | None = None
        self._selected_extrinsics_profile_path = ""
        self._selected_extrinsics_profile_hash = ""
        self._active_intrinsics_profile_hash = ""
        self._active_intrinsics_source_zoom_level: int | None = None
        self._active_extrinsics_profile_path = ""
        self._active_extrinsics_profile_hash = ""
        self._active_calibration_selection_mode = ""
        self.zoom_intrinsics_compare_paths: list[Path] = []
        self.pnp_photo_capture_thread: QtCore.QThread | None = None
        self.pnp_photo_capture_worker: QtCore.QObject | None = None
        self.pnp_photo_capture_records: list[dict[str, object]] = []
        self.oz_photo_capture_thread: QtCore.QThread | None = None
        self.oz_photo_capture_worker: QtCore.QObject | None = None
        self.oz_photo_capture_records: list[dict[str, object]] = []
        self.optical_pose_wiris_ftp_thread: QtCore.QThread | None = None
        self.optical_pose_wiris_ftp_worker: QtCore.QObject | None = None
        self.optical_pose_wiris_ftp_oz_thread: QtCore.QThread | None = None
        self.optical_pose_wiris_ftp_oz_worker: QtCore.QObject | None = None
        self._pnp_session_prepared = False
        self._measurement_package_calibration_collected = False
        self._px4_calibration_parameter_error: dict[str, str] = {}
        self._px4_calibration_reset_error: dict[str, str] = {}
        self._px4_calibration_reset_pending: dict[str, str] = {}

    def _update_calibration_preview(self, camera: str, image: QtGui.QImage, metadata: object | None = None) -> None:
        now_s = time.monotonic()
        if isinstance(metadata, dict):
            frame_metadata = dict(metadata)
        elif metadata is None:
            frame_metadata = {}
        else:
            frame_metadata = {"timestamp_s": float(metadata)}
        raw_frame_timestamp_s = frame_metadata.get("timestamp_s", now_s)
        if raw_frame_timestamp_s is None:
            raw_frame_timestamp_s = now_s
        frame_timestamp_s = float(raw_frame_timestamp_s)
        frame_metadata.setdefault("timestamp_s", frame_timestamp_s)
        frame_metadata.setdefault("source_timestamp_s", None)
        frame_metadata.setdefault("timestamp_source", "app-receive")
        frame_metadata.setdefault("timestamp_reference", infer_runtime_camera_timestamp_reference(frame_metadata))
        decorate_clock_epoch = getattr(self, "_decorate_clock_epoch_metadata", None)
        if callable(decorate_clock_epoch):
            frame_metadata = decorate_clock_epoch(camera, frame_metadata)
        else:
            # Narrow preview/calibration adapters do not own the Workstation
            # clock controller. Preserve their existing metadata contract.
            frame_metadata.setdefault("clock_epoch_id", None)
            frame_metadata.setdefault("clock_epoch_state", "unverified")
        self._record_gui_camera_frame(camera, image, frame_metadata)
        if not self.yolo_enabled:
            self.left_pane.clear_measurement_pair_display()
            self.right_pane.clear_measurement_pair_display()
            self._clear_selected_runtime_measurement_pair(update_labels=False)
        self.camera_frame_metadata[camera] = frame_metadata
        self.camera_frame_metadata_history[camera].append(dict(frame_metadata))
        self.camera_frame_times[camera].append(frame_timestamp_s)
        self._capture_recording_clock_sync_start_observation()
        # Stream stats already refresh the camera status/header once per
        # second. Rebuilding those UI labels on every preview frame made two
        # live cameras continuously consume the GUI event loop.
        self._record_clock_offset_sample(now_s)
        if self._is_clock_sync_tab_visible():
            self._update_clock_sync_display()
        # QImage is implicitly shared. Keep a cheap, value-semantics handle for
        # explicit snapshots/checkerboard work instead of two deep copies and a
        # duplicate raw-preview signal on every frame. CalibrationWorkspace
        # already receives the same raw frame from CameraPane.frame_changed;
        # calibration_preview_changed remains reserved for throttled overlays.
        self.latest_calibration_preview_images[camera] = QtGui.QImage(image)
        if not (
            self._is_checkerboard_calibration_visible()
            or self._workstation_calibration_preview_active
        ):
            return
        if now_s - self.calibration_preview_times[camera] < 0.5:
            return
        self.calibration_preview_times[camera] = now_s
        self._refresh_calibration_preview(camera)

    def _recent_detection_observations(self, camera: str, *, now_s: float | None = None) -> list[DetectionObservation]:
        if not self.yolo_enabled and not self.camera_connected:
            return list(self.detection_buffers[camera])
        _ = time.monotonic() if now_s is None else now_s
        interval_start = getattr(self, "_measurement_detection_started_monotonic_s", None)
        return [
            detection
            for detection in self.detection_buffers[camera]
            if interval_start is None or float(detection.timestamp_s) >= float(interval_start)
        ]

    def _matched_detection_pair(self):
        if bool(getattr(self, "_detection_pair_coordinator_active", False)):
            return getattr(self, "_coordinated_detection_pair", None)
        now_s = time.monotonic()
        advisories: list[str] = []
        left_samples = self._recent_detection_observations("left", now_s=now_s)
        right_samples = self._recent_detection_observations("right", now_s=now_s)
        self._runtime_pair_uses_app_fallback = False
        configured_delta_s = self._yolo_pair_max_delta_s()
        app_delta_limit_ns = (
            100_000_000
            if not isfinite(configured_delta_s)
            else min(
                100_000_000,
                max(0, int(round(configured_delta_s * 1_000_000_000.0))),
            )
        )
        app_delta_limit_s = app_delta_limit_ns * 1.0e-9
        pair = match_nearest_pair(
            left_samples,
            right_samples,
            max_delta_s=app_delta_limit_s + 1.0e-9,
            timestamp=self._yolo_match_timestamp_s,
        )
        timing_rejection = ""
        if pair is None and left_samples and right_samples:
            app_timestamp_invalid = any(
                self._yolo_match_timestamp_s(observation) is None
                for observation in (*left_samples, *right_samples)
            )
            if app_timestamp_invalid:
                timing_rejection = "APP_RECEIVE_TIMESTAMP_INVALID"
            else:
                # The window only prefers close partners; the nearest pair is
                # still measured and its excess becomes an advisory.
                pair = match_nearest_pair(
                    left_samples,
                    right_samples,
                    max_delta_s=float("inf"),
                    timestamp=self._yolo_match_timestamp_s,
                )
        if pair is not None:
            try:
                exact_app_delta_ns = abs(
                    pair.right.app_receive_monotonic_ns
                    - pair.left.app_receive_monotonic_ns
                )
            except (TypeError, ValueError):
                pair = None
                timing_rejection = "APP_RECEIVE_TIMESTAMP_INVALID"
            else:
                del exact_app_delta_ns

        validated = self._validated_correspondence_pair(pair, left_samples, right_samples)
        if timing_rejection:
            self._last_correspondence_rejection_reason = timing_rejection
        if validated is not None:
            advisories.extend(correspondence_advisories(validated.left, validated.right))
            if validated.delta_s > app_delta_limit_s:
                advisories.append(
                    f"camera pair delta {validated.delta_s:.6f}s exceeds configured limit"
                )
            vertical_delta = abs(validated.left.center[1] - validated.right.center[1])
            if vertical_delta > self._yolo_vertical_max_delta_px():
                advisories.append(
                    f"vertical bbox delta {vertical_delta:.3f}px exceeds configured limit"
                )
        self._last_pair_advisories = tuple(dict.fromkeys(advisories))
        return validated

    def _validated_correspondence_pair(self, pair, left_samples, right_samples):
        if pair is None:
            if not left_samples:
                self._last_correspondence_rejection_reason = "missing_left_detection"
            elif not right_samples:
                self._last_correspondence_rejection_reason = "missing_right_detection"
            else:
                self._last_correspondence_rejection_reason = ""
            return None
        reason = correspondence_rejection_reason(pair.left, pair.right)
        self._last_correspondence_rejection_reason = "" if reason is None else reason
        return pair if reason is None else None

    def _recent_detection_pair_available(self) -> bool:
        pair = self._matched_detection_pair()
        if pair is None:
            return False
        return True

    def _calibration_auto_capture_is_active(self) -> bool:
        owner = getattr(self, "_calibration_workflow_controller_owner", None)
        return bool(getattr(owner, "_auto_capture_active", False))

    def _refresh_calibration_preview(self, camera: str) -> None:
        image = self.latest_calibration_preview_images.get(camera)
        if image is None:
            return
        metadata = self.camera_frame_metadata.get(camera) or {}
        if (
            self._calibration_auto_capture_is_active()
            and not is_nal1_non_idr_auto_capture_metadata(metadata)
        ):
            return
        if camera == "left":
            label = self.calibration_left_preview
        elif camera == "right":
            label = self.calibration_right_preview
        else:
            return
        waiting = self._calibration_overlay_image(
            image,
            found=False,
            corners=[],
            status="checkerboard: detecting...",
        )
        self._set_calibration_preview_pixmap(camera, label, waiting)
        self._queue_calibration_preview_detection(camera, image)

    def _refresh_all_calibration_previews(self, *_args) -> None:
        self._refresh_calibration_preview("left")
        self._refresh_calibration_preview("right")

    def _handle_tab_changed(self, index: int) -> None:
        if index == self.parameter_tab_index:
            self._refresh_camera_parameters_tab()
        if index == self.calibration_tab_index and self._is_checkerboard_calibration_visible():
            self.calibration_preview_times["left"] = 0.0
            self.calibration_preview_times["right"] = 0.0
            self._refresh_all_calibration_previews()
        if hasattr(self, "triangulation_quality_plot"):
            self.triangulation_quality_plot.set_tab_visible(self._is_triangulation_quality_visible())
        if index == self.time_tab_index:
            self._update_clock_sync_display()
        if index == self.results_tab_index:
            self._refresh_active_recording_sheet()

    def _is_triangulation_quality_visible(self) -> bool:
        return (
            self.tabs.currentIndex() == self.depth_tab_index
            and hasattr(self, "measurement_tabs")
            and self.measurement_tabs.currentIndex() == self.triangulation_quality_tab_index
        )

    def _is_checkerboard_calibration_visible(self) -> bool:
        return (
            self.tabs.currentIndex() == self.calibration_tab_index
            and hasattr(self, "calibration_tabs")
            and self.calibration_tabs.currentIndex() == self.checkerboard_calibration_tab_index
        )

    def _calibration_overlay_image(
        self,
        image: QtGui.QImage,
        *,
        found: bool,
        corners: list[tuple[float, float]],
        status: str,
    ) -> QtGui.QImage:
        annotated = image.convertToFormat(QtGui.QImage.Format_RGB888).copy()
        painter = QtGui.QPainter(annotated)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        if found and corners:
            pen = QtGui.QPen(QtGui.QColor(ACCENT), 3)
            painter.setPen(pen)
            painter.setBrush(QtGui.QBrush(QtGui.QColor(ACCENT)))
            columns, rows = self._checkerboard_config().pattern_size
            if len(corners) == columns * rows:
                for row in range(rows):
                    start = row * columns
                    for column in range(columns - 1):
                        painter.drawLine(
                            QtCore.QPointF(*corners[start + column]),
                            QtCore.QPointF(*corners[start + column + 1]),
                        )
                for column in range(columns):
                    for row in range(rows - 1):
                        painter.drawLine(
                            QtCore.QPointF(*corners[row * columns + column]),
                            QtCore.QPointF(*corners[(row + 1) * columns + column]),
                        )
            for x, y in corners:
                painter.drawEllipse(QtCore.QPointF(x, y), 5.0, 5.0)
        else:
            painter.setPen(QtGui.QPen(QtGui.QColor(ERROR), 4))
            painter.drawRect(2, 2, max(0, annotated.width() - 4), max(0, annotated.height() - 4))
        painter.fillRect(0, 0, annotated.width(), 28, QtGui.QColor(0, 0, 0, 170))
        painter.setPen(QtGui.QPen(QtGui.QColor(230, 230, 230), 1))
        painter.drawText(8, 20, status)
        painter.end()
        return annotated

    def _queue_calibration_preview_detection(self, camera: str, image: QtGui.QImage) -> int:
        self._ensure_calibration_preview_worker()
        self._calibration_preview_next_request_id += 1
        request_id = self._calibration_preview_next_request_id
        self.calibration_preview_request_ids[camera] = request_id
        self.calibration_preview_requested.emit(
            {
                "camera": camera,
                "request_id": request_id,
                "image": image.convertToFormat(QtGui.QImage.Format_RGB888).copy(),
                "pattern_size": self._checkerboard_config().pattern_size,
                "metadata": dict(self.camera_frame_metadata.get(camera) or {}),
            }
        )
        return request_id

    def _ensure_calibration_preview_worker(self) -> None:
        thread = getattr(self, "_calibration_preview_thread", None)
        worker = getattr(self, "_calibration_preview_worker", None)
        if thread is not None and worker is not None and thread.isRunning():
            return
        if thread is not None:
            self._stop_calibration_preview_worker()

        worker = CheckerboardPreviewWorker()
        thread = QtCore.QThread(self.widget)
        worker.moveToThread(thread)
        self.calibration_preview_requested.connect(worker.submit)
        worker.result_ready.connect(self._handle_calibration_preview_result)
        thread.finished.connect(worker.deleteLater)
        self._calibration_preview_worker = worker
        self._calibration_preview_thread = thread
        thread.start()

    def _handle_calibration_preview_result(self, payload: object) -> None:
        if not isinstance(payload, dict):
            return
        camera = str(payload.get("camera") or "")
        if camera not in {"left", "right"}:
            return
        request_id = int(payload.get("request_id") or 0)
        if request_id != self.calibration_preview_request_ids.get(camera):
            return
        image = payload.get("image")
        if not isinstance(image, QtGui.QImage):
            return
        metadata = payload.get("metadata")
        self.calibration_preview_detection_results[camera] = {
            "request_id": request_id,
            "image": image.copy(),
            "found": bool(payload.get("found")),
            "corners": list(payload.get("corners") or []),
            "status": str(payload.get("status") or "checkerboard: not found"),
            "metadata": dict(metadata) if isinstance(metadata, dict) else {},
        }
        if not (
            self._is_checkerboard_calibration_visible()
            or self._workstation_calibration_preview_active
        ):
            return
        corners = payload.get("corners")
        overlay = self._calibration_overlay_image(
            image,
            found=bool(payload.get("found")),
            corners=list(corners or []),
            status=str(payload.get("status") or "checkerboard: not found"),
        )
        label = self.calibration_left_preview if camera == "left" else self.calibration_right_preview
        self._set_calibration_preview_pixmap(camera, label, overlay)

    def _set_calibration_preview_pixmap(
        self,
        camera: str,
        label: QtWidgets.QLabel,
        image: QtGui.QImage,
    ) -> None:
        self.calibration_preview_overlay_images[camera] = image.copy()
        pixmap = QtGui.QPixmap.fromImage(image)
        label.setPixmap(pixmap.scaled(label.size(), QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation))
        self.calibration_preview_changed.emit(
            {
                "camera": camera,
                "image": image.copy(),
                "kind": "overlay",
            }
        )

    def _rescale_calibration_preview_pixmaps(self) -> None:
        for camera, label in (
            ("left", self.calibration_left_preview),
            ("right", self.calibration_right_preview),
        ):
            image = self.calibration_preview_overlay_images.get(camera)
            if image is None:
                continue
            pixmap = QtGui.QPixmap.fromImage(image)
            label.setPixmap(pixmap.scaled(label.size(), QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation))

    def workstation_refresh_calibration_preview(self) -> dict[str, object]:
        self._workstation_calibration_preview_active = True
        self.calibration_preview_times["left"] = 0.0
        self.calibration_preview_times["right"] = 0.0
        self._refresh_all_calibration_previews()
        return self.workstation_calibration_preview_snapshot()

    def _auto_load_intrinsics(self) -> None:
        if not bool(self._workstation_setting("auto_load_intrinsics", self.settings.auto_load_intrinsics)):
            return
        zoom_level = self.zoom_level_spin.value()
        try:
            result, source_path, error = self._auto_load_intrinsics_candidate(zoom_level)
        except Exception as exc:
            result = None
            source_path = None
            error = f"unexpected intrinsic profile resolution failure: {exc}"
        if result is None:
            detail = error or f"no valid profile for zoom {zoom_level:02d}"
            self.calibration_status_label.setText(
                f"Failed to auto-load intrinsics for zoom {zoom_level:02d}: {detail}"
            )
            self._log(f"[캘리브레이션] intrinsic 자동 로드를 건너뜁니다: {detail}")
            self._refresh_operation_header()
            self._refresh_zoom_intrinsics_tab()
            return
        state_names = (
            "last_calibration_result",
            "last_calibration_zoom_level",
            "_candidate_intrinsics_profile_path",
            "_active_intrinsics_profile_path",
            "active_intrinsics",
            "_active_intrinsics_source_zoom_level",
            "_active_intrinsics_profile_hash",
            "selected_extrinsics_profile",
            "active_extrinsics_profile",
            "checkerboard_extrinsics_candidate_profile",
            "optical_pose_extrinsics_candidate_profile",
            "checkerboard_extrinsics_profile",
            "optical_pose_extrinsics_profile",
            "_selected_extrinsics_profile_path",
            "_selected_extrinsics_profile_hash",
            "_active_extrinsics_profile_path",
            "_active_extrinsics_profile_hash",
            "_active_extrinsics_source_zoom_level",
            "_active_calibration_selection_mode",
        )
        previous_state = {name: getattr(self, name) for name in state_names}

        def restore_previous_state() -> None:
            restore_geometry_values(self, {name: (True, value) for name, value in previous_state.items()})

        try:
            self.last_calibration_result = result
            self.last_calibration_zoom_level = zoom_level
            self._candidate_intrinsics_profile_path = str(source_path)
            if self.apply_intrinsics_to_distance() is False:
                restore_previous_state()
                self._log(
                    f"[캘리브레이션] intrinsic 자동 적용이 quality/grid gate에서 차단됐습니다 "
                    f"(zoom={zoom_level:02d})"
                )
                self._refresh_operation_header()
                self._refresh_zoom_intrinsics_tab()
                return
            self.calibration_status_label.setText(
                f"Auto-loaded intrinsics for zoom {zoom_level:02d}: {source_path}"
            )
            self._log(
                f"[캘리브레이션] intrinsic을 자동으로 불러왔습니다 "
                f"(zoom={zoom_level:02d}, path={source_path})"
            )
        except Exception as exc:
            restore_previous_state()
            self.calibration_status_label.setText(f"Failed to auto-load intrinsics: {exc}")
            self._log(f"[캘리브레이션] intrinsic 자동 로드 실패: {exc}")
        self._refresh_operation_header()
        self._refresh_zoom_intrinsics_tab()

    def save_intrinsics(self, *, quality_warning_confirmed: bool = False) -> None:
        if self.last_calibration_result is None:
            self.calibration_status_label.setText("Run calibration before saving intrinsics.")
            return
        quality_state = "UNASSESSED"
        quality_reasons: tuple[str, ...] = ()
        quality_reader = getattr(self, "_calibration_quality_state", None)
        if callable(quality_reader):
            quality_state, quality_reasons = quality_reader(self.last_calibration_result)
        root, _left_dir, _right_dir = self._calibration_dirs()
        resolved_root = root.expanduser().resolve(strict=False)
        default_root = Path(self.default_calibration_root).expanduser().resolve(
            strict=False
        )
        try:
            resolved_root.relative_to(default_root)
        except ValueError:
            calibration_root_portability_probe = resolved_root
        else:
            calibration_root_portability_probe = (
                Path(self.app_root) / DEFAULT_CALIBRATION_ROOT
            )
        try:
            portable_workstation_data_path_text(
                calibration_root_portability_probe,
                data_root=self.app_root,
                field_name="calibration_root",
            )
        except ValueError as exc:
            self.calibration_status_label.setText(
                f"Cannot save intrinsics outside the portable data root: {exc}"
            )
            self._log(f"[캘리브레이션] intrinsic 저장 차단: {exc}")
            return
        zoom_level = self.last_calibration_zoom_level
        approved_output = (
            self._intrinsics_storage_path(zoom_level)
            if CalibrationQuality.approved_for_profile_save(quality_state, capture_zoom_known=zoom_level is not None)
            else None
        )
        metadata = {
            "note": "Generated by StereoWIRIS Camera Calibration tab",
            "timestamp_s": time.time(),
            "zoom_level": zoom_level,
            "capture_zoom_level": zoom_level,
            "zoom_source": "capture_metadata",
            "zoom_target": str(self.zoom_target_combo.currentData() or "both"),
        }
        profiles = CalibrationProfiles()
        saved = profiles.save_intrinsics_attempt(
            self.last_calibration_result, root=root, zoom_level=zoom_level,
            quality_state=quality_state, quality_reasons=quality_reasons,
            metadata=metadata, approved_output=approved_output, intrinsics_only=True,
        )
        output = saved.output
        self._last_calibration_attempt_path = str(output)
        if saved.approved:
            immutable_output = profiles.publish_saved_intrinsics(
                saved, root=root, latest_output=self._intrinsics_storage_base_path(),
            )
            self.last_calibration_zoom_level = zoom_level
            self._candidate_intrinsics_profile_path = str(immutable_output)
            self.calibration_status_label.setText(
                f"Saved approved intrinsics: immutable={immutable_output} | alias={output}"
            )
            self._add_zoom_intrinsics_comparison_path(immutable_output)
            self._save_current_settings(immutable_output)
        else:
            reason = ", ".join(quality_reasons) or (
                "capture-time camera zoom readback unavailable"
                if zoom_level is None
                else quality_state
            )
            self.calibration_status_label.setText(
                f"Saved non-applicable calibration attempt: {quality_state} ({reason}) | {output}"
            )
        self._log(f"[캘리브레이션] {self.calibration_status_label.text()}")
        self._refresh_operation_header()
        self._refresh_zoom_intrinsics_tab()

    def _validate_intrinsics_result(self, result: StereoIntrinsicsResult) -> tuple[str, ...]:
        return validate_intrinsics_result(result)

    def _confirm_intrinsics_profile_mismatch(
        self,
        path: Path,
        result: StereoIntrinsicsResult,
        metadata_zoom_level: int | None,
    ) -> bool:
        warnings: list[str] = []
        current_zoom_level = self.zoom_level_spin.value()
        if metadata_zoom_level is None:
            warnings.append(
                "파일 metadata.zoom_level 없음; 수동 검토 적용만 가능하며 다음 실행의 자동 로드는 차단됨"
            )
        elif metadata_zoom_level != current_zoom_level:
            warnings.append(
                f"파일 zoom_level={metadata_zoom_level:02d}, 현재 WIRIS zoom={current_zoom_level:02d}"
            )
        left_size = (result.left.width, result.left.height)
        right_size = (result.right.width, result.right.height)
        if left_size != right_size:
            warnings.append(f"좌/우 calibration 해상도가 다름: left={left_size}, right={right_size}")
        if warnings:
            warning_text = "; ".join(warnings)
            self.calibration_status_label.setText(
                f"Profile warning (non-blocking): {warning_text}"
            )
            self._log(
                f"[캘리브레이션] intrinsic profile mismatch warning; "
                f"manual apply remains allowed: {path}: {warning_text}"
            )
        return True

    def load_intrinsics(self) -> None:
        initial_path = self._intrinsics_storage_path(self.zoom_level_spin.value())
        selected, _selected_filter = QtWidgets.QFileDialog.getOpenFileName(
            self.widget,
            "Load intrinsics",
            str(initial_path if initial_path.exists() else initial_path.parent),
            "Intrinsics JSON (*.json);;All files (*)",
        )
        if not selected:
            return
        source_path = Path(selected)
        try:
            result = CalibrationProfiles.load_intrinsics(source_path)
            errors = self._validate_intrinsics_result(result)
            if errors:
                raise ValueError("; ".join(errors))
            metadata_zoom_level = self._intrinsics_file_zoom_level(source_path)
        except Exception as exc:
            self.calibration_status_label.setText(f"Failed to load intrinsics: {exc}")
            self._log(f"[캘리브레이션] intrinsic 파일 검증 실패: {source_path}: {exc}")
            return
        if not self._confirm_intrinsics_profile_mismatch(source_path, result, metadata_zoom_level):
            self._log(f"[캘리브레이션] intrinsic 적용을 취소했습니다: {source_path}")
            return
        try:
            path = import_workstation_asset_file(
                source_path,
                category="calibration",
                data_root=self.app_root,
            )
        except (OSError, ValueError) as exc:
            self.calibration_status_label.setText(f"Failed to import intrinsics: {exc}")
            self._log(
                f"[캘리브레이션] intrinsic portable import 실패: {source_path}: {exc}"
            )
            return
        try:
            session_copy = self._archive_loaded_intrinsics(path)
        except (OSError, RuntimeError, ValueError) as exc:
            self.calibration_status_label.setText(
                f"Failed to preserve intrinsics in current Session: {exc}"
            )
            self._log(f"[캘리브레이션] intrinsic Session 보관 실패: {path}: {exc}")
            return
        if path != source_path.resolve(strict=False):
            self._log(
                f"[캘리브레이션] 외부 intrinsic을 portable data root로 가져왔습니다: "
                f"source={source_path}, imported={path}"
            )
        self.last_calibration_result = result
        self.calibration_image_diagnostics = tuple(result.image_diagnostics)
        self.last_calibration_zoom_level = (
            self.zoom_level_spin.value() if metadata_zoom_level is None else metadata_zoom_level
        )
        self._candidate_intrinsics_profile_path = str(path)
        if self.apply_intrinsics_to_distance() is False:
            self._log(f"[캘리브레이션] intrinsic 적용이 quality/grid gate에서 차단됐습니다: {path}")
            self._refresh_zoom_intrinsics_tab()
            self._refresh_operation_header()
            return
        try:
            self._save_current_settings(path)
        except ValueError as exc:
            self.calibration_status_label.setText(
                f"Loaded intrinsics for this run; portable settings were not saved: {exc}"
            )
            self._log(
                f"[캘리브레이션] intrinsic은 현재 실행에 적용했지만 portable 설정 저장은 "
                f"차단됐습니다: {exc}"
            )
        else:
            self.calibration_status_label.setText(
                f"Loaded and applied intrinsics: {path.name}"
                + (f" | Session copy={session_copy.name}" if session_copy is not None else "")
            )
        self._add_zoom_intrinsics_comparison_path(path)
        self._log(f"[캘리브레이션] intrinsic 파일을 선택해 적용했습니다: {path}")
        self._refresh_zoom_intrinsics_tab()
        self._refresh_operation_header()

    def load_current_zoom_intrinsics(self) -> None:
        zoom_level = self.zoom_level_spin.value()
        result = self._load_current_zoom_intrinsics_for_apply()
        if result is None:
            path = self._intrinsics_storage_path(zoom_level)
            self.calibration_status_label.setText(f"Failed to load intrinsics for zoom {zoom_level:02d}: {path}")
            self._refresh_operation_header()
            self._refresh_zoom_intrinsics_tab()
            return
        self.last_calibration_result = result
        self.calibration_image_diagnostics = tuple(result.image_diagnostics)
        self.last_calibration_zoom_level = zoom_level
        zoom_path = self._intrinsics_storage_path(zoom_level)
        self._candidate_intrinsics_profile_path = str(
            zoom_path if zoom_path.exists() else self._intrinsics_storage_base_path()
        )
        if self.apply_intrinsics_to_distance() is False:
            self._log(
                f"[캘리브레이션] 현재 zoom intrinsic 적용이 quality/grid gate에서 차단됐습니다 "
                f"(zoom={zoom_level:02d})"
            )
            self._refresh_zoom_intrinsics_tab()
            self._refresh_operation_header()
            return
        self._save_current_settings()
        self.calibration_status_label.setText(f"Applied intrinsics for zoom {zoom_level:02d}")
        self._log(f"[캘리브레이션] intrinsic 파일을 적용했습니다 (zoom={zoom_level:02d})")
        self._refresh_zoom_intrinsics_tab()
        self._refresh_operation_header()

    def build_calibration_capture(self):
        calibration_group = QtWidgets.QGroupBox("체커보드 캡처")
        calibration_group.setObjectName("checkerboardIntrinsicCalibrationGroup")
        calib_form = QtWidgets.QGridLayout(calibration_group)
        calib_form.setHorizontalSpacing(6)
        calib_form.setVerticalSpacing(6)
        calib_form.setColumnStretch(5, 1)
        self.calib_cols_spin = QtWidgets.QSpinBox()
        self.calib_cols_spin.setRange(2, 30)
        self.calib_cols_spin.setValue(self.settings.calibration_inner_cols)
        self.calib_cols_spin.valueChanged.connect(self._refresh_all_calibration_previews)
        self.calib_rows_spin = QtWidgets.QSpinBox()
        self.calib_rows_spin.setRange(2, 30)
        self.calib_rows_spin.setValue(self.settings.calibration_inner_rows)
        self.calib_rows_spin.valueChanged.connect(self._refresh_all_calibration_previews)
        self.calib_square_spin = QtWidgets.QDoubleSpinBox()
        self.calib_square_spin.setRange(1.0, 500.0)
        self.calib_square_spin.setValue(self.settings.calibration_square_size_mm)
        self.calib_square_spin.setSuffix(" mm")
        self.calib_min_images_spin = QtWidgets.QSpinBox()
        self.calib_min_images_spin.setRange(3, 100)
        self.calib_min_images_spin.setValue(self.settings.calibration_min_images)
        self.calib_calculation_pairs_spin = QtWidgets.QSpinBox()
        self.calib_calculation_pairs_spin.setRange(0, 10000)
        self.calib_calculation_pairs_spin.setValue(self.settings.calibration_calculation_pair_count)
        self.calib_calculation_pairs_spin.setSpecialValueText("전체 pair")
        self.calib_calculation_pairs_spin.setSuffix(" pair")
        for spin, width in (
            (self.calib_cols_spin, 72),
            (self.calib_rows_spin, 72),
            (self.calib_square_spin, 96),
            (self.calib_calculation_pairs_spin, 96),
        ):
            spin.setProperty("compactFormField", True)
            spin.setMinimumWidth(width)
            spin.setSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        calibration_root = (
            self.settings.calibration_root.strip() or DEFAULT_CALIBRATION_ROOT
        )
        self.calib_dir_edit = QtWidgets.QLineEdit(calibration_root)
        self.calib_dir_edit.setPlaceholderText(
            f"Default: {self.default_calibration_root}"
        )
        self.calib_dir_edit.setProperty("compactFormField", True)
        self.calib_dir_edit.setMinimumWidth(260)
        self.capture_calibration_button = QtWidgets.QPushButton("Capture Live Pair")
        self.capture_calibration_button.setToolTip(
            "Capture the newest decoded native pair from the live RTSP streams; raw RTP packets are not captured."
        )
        self.capture_calibration_button.clicked.connect(self.capture_calibration_images)
        self.calibration_pair_max_delta_spin = QtWidgets.QDoubleSpinBox()
        self.calibration_pair_max_delta_spin.setRange(0.0, 1000.0)
        self.calibration_pair_max_delta_spin.setSingleStep(5.0)
        self.calibration_pair_max_delta_spin.setDecimals(1)
        self.calibration_pair_max_delta_spin.setValue(
            self.settings.calibration_pair_max_delta_s * 1000.0
        )
        self.calibration_pair_max_delta_spin.setSuffix(" ms")
        self.calibration_pair_max_delta_spin.setToolTip(
            "Live의 좌·우 decoded frame이 앱에 도착한 host monotonic 수신시각 "
            "차이가 이 값 이하인 pair만 calibration stereo R,t에 사용합니다. "
            "수신시각은 실제 노출시각을 증명하지 않으며, 초과 이미지는 카메라별 K,D용으로 저장합니다."
        )
        self.calibration_auto_interval_spin = QtWidgets.QDoubleSpinBox()
        self.calibration_auto_interval_spin.setRange(0.1, 10.0)
        self.calibration_auto_interval_spin.setSingleStep(0.1)
        self.calibration_auto_interval_spin.setDecimals(1)
        self.calibration_auto_interval_spin.setValue(self.settings.calibration_auto_interval_s)
        self.calibration_auto_interval_spin.setSuffix(" s")
        self.capture_calibration_auto_button = QtWidgets.QPushButton("자동 촬영 시작")
        self.capture_calibration_auto_button.clicked.connect(self.toggle_auto_calibration_capture)
        self.save_intrinsics_button = QtWidgets.QPushButton("저장")
        self.save_intrinsics_button.clicked.connect(self.save_intrinsics)
        self.apply_intrinsics_button = QtWidgets.QPushButton("계산에 적용")
        self.apply_intrinsics_button.clicked.connect(self.apply_intrinsics_to_distance)
        self.calibration_status_label = QtWidgets.QLabel("좌/우 라이브 페어에서 체커보드가 보이는 프레임을 캡처하세요.")
        self.calibration_status_label.setObjectName("statusStrip")
        self.calibration_status_label.setWordWrap(True)
        self.capture_calibration_button.setText("Capture Live Pair")
        self.save_intrinsics_button.setText("저장")
        self.apply_intrinsics_button.setText("계산에 적용")
        self.calibration_result_text = QtWidgets.QPlainTextEdit()
        self.calibration_result_text.setReadOnly(True)
        self.calibration_result_text.setMaximumBlockCount(200)
        calibration_preview_row = QtWidgets.QHBoxLayout()
        self.calibration_left_preview = self._build_calibration_preview_label("left preview")
        self.calibration_right_preview = self._build_calibration_preview_label("right preview")
        calibration_preview_row.addWidget(self.calibration_left_preview, 1)
        calibration_preview_row.addWidget(self.calibration_right_preview, 1)

        compact_calibration_labels = [
            QtWidgets.QLabel(text)
            for text in ("내부 열", "내부 행", "격자 크기", "저장 경로")
        ]
        for label in compact_calibration_labels:
            label.setProperty("compactFormLabel", True)
        cols_label, rows_label, square_label, dir_label = compact_calibration_labels

        calib_form.addWidget(cols_label, 0, 0)
        calib_form.addWidget(self.calib_cols_spin, 0, 1)
        calib_form.addWidget(rows_label, 0, 2)
        calib_form.addWidget(self.calib_rows_spin, 0, 3)
        calib_form.addWidget(square_label, 0, 4)
        calib_form.addWidget(self.calib_square_spin, 0, 5)
        calib_form.addWidget(dir_label, 1, 0)
        calib_form.addWidget(self.calib_dir_edit, 1, 1, 1, 5)
        calib_form.addWidget(self.capture_calibration_button, 1, 6)
        auto_interval_label = QtWidgets.QLabel("자동 간격")
        auto_interval_label.setProperty("compactFormLabel", True)
        pair_max_delta_label = QtWidgets.QLabel("최대 pair Δt")
        pair_max_delta_label.setProperty("compactFormLabel", True)
        calib_form.addWidget(auto_interval_label, 2, 0)
        calib_form.addWidget(self.calibration_auto_interval_spin, 2, 1)
        calib_form.addWidget(pair_max_delta_label, 2, 2)
        calib_form.addWidget(self.calibration_pair_max_delta_spin, 2, 3)
        calib_form.addWidget(self.capture_calibration_auto_button, 2, 4, 1, 3)
        return calibration_group, calibration_preview_row

    def build_calibration_validation(self, calibration_group, calibration_preview_row, checkerboard_calibration_layout, checkerboard_validation_layout, zoom_group):
        validation_source_group = QtWidgets.QGroupBox("입력 / 계산 목적")
        validation_source_form = QtWidgets.QGridLayout(validation_source_group)
        validation_source_form.setHorizontalSpacing(6)
        validation_source_form.setVerticalSpacing(6)
        validation_source_form.setColumnStretch(1, 1)
        validation_source_form.setColumnStretch(4, 1)
        self.checkerboard_validation_mode_combo = QtWidgets.QComboBox()
        self.checkerboard_validation_mode_combo.addItem("Pixhawk 자세 + GPS/RTK baseline", "pixhawk")
        self.checkerboard_validation_mode_combo.addItem("체커보드 Stereo 외부 파라미터", "checkerboard")
        self._set_combo_data(
            self.checkerboard_validation_mode_combo,
            self.settings.calibration_validation_mode,
        )
        self.checkerboard_validation_mode_combo.setToolTip(
            "Pixhawk 모드는 내부 파라미터만 계산하고, 체커보드 모드는 내부 파라미터와 stereo R/T를 함께 계산합니다."
        )
        validation_root = (
            self.settings.calibration_validation_root.strip()
            or self.calib_dir_edit.text()
        )
        self.checkerboard_validation_root_edit = QtWidgets.QLineEdit(validation_root)
        self.checkerboard_validation_root_edit.setMinimumWidth(260)
        self.checkerboard_validation_choose_root_button = QtWidgets.QPushButton("폴더 선택")
        self.checkerboard_validation_choose_root_button.clicked.connect(self.choose_checkerboard_validation_root)
        self.checkerboard_validation_scan_button = QtWidgets.QPushButton("스캔")
        self.checkerboard_validation_scan_button.clicked.connect(self.scan_checkerboard_validation_files)
        validation_json = (
            self.settings.calibration_validation_json.strip()
            or str(self._intrinsics_storage_path())
        )
        self.checkerboard_validation_json_edit = QtWidgets.QLineEdit(validation_json)
        self.checkerboard_validation_json_edit.setMinimumWidth(260)
        self.checkerboard_validation_choose_json_button = QtWidgets.QPushButton("JSON 선택")
        self.checkerboard_validation_choose_json_button.clicked.connect(self.choose_checkerboard_validation_json)
        self.checkerboard_validation_load_json_button = QtWidgets.QPushButton("기존 JSON 불러오기")
        self.checkerboard_validation_load_json_button.clicked.connect(self.load_checkerboard_validation_json)
        validation_source_form.addWidget(QtWidgets.QLabel("계산 방식"), 0, 0)
        validation_source_form.addWidget(self.checkerboard_validation_mode_combo, 0, 1, 1, 2)
        validation_source_form.addWidget(QtWidgets.QLabel("사진셋 root"), 1, 0)
        validation_source_form.addWidget(self.checkerboard_validation_root_edit, 1, 1, 1, 3)
        validation_source_form.addWidget(self.checkerboard_validation_choose_root_button, 1, 4)
        validation_source_form.addWidget(self.checkerboard_validation_scan_button, 1, 5)
        validation_source_form.addWidget(QtWidgets.QLabel("기존 intrinsics JSON"), 2, 0)
        validation_source_form.addWidget(self.checkerboard_validation_json_edit, 2, 1, 1, 3)
        validation_source_form.addWidget(self.checkerboard_validation_choose_json_button, 2, 4)
        validation_source_form.addWidget(self.checkerboard_validation_load_json_button, 2, 5)
        self.checkerboard_validation_scan_label = QtWidgets.QLabel("스캔 대기 중")
        self.checkerboard_validation_scan_label.setObjectName("statusStrip")
        self.checkerboard_validation_scan_label.setWordWrap(True)
        validation_source_form.addWidget(self.checkerboard_validation_scan_label, 3, 0, 1, 6)
        self.checkerboard_validation_mode_combo.currentIndexChanged.connect(
            lambda _index: self.scan_checkerboard_validation_files()
        )
        self.checkerboard_validation_root_edit.editingFinished.connect(self.scan_checkerboard_validation_files)

        validation_action_group = QtWidgets.QGroupBox("계산 / 검증")
        validation_action_form = QtWidgets.QGridLayout(validation_action_group)
        validation_action_form.setHorizontalSpacing(6)
        validation_action_form.setVerticalSpacing(6)
        validation_action_form.setColumnStretch(9, 1)
        self.calib_min_images_spin.setProperty("compactFormField", True)
        self.calib_min_images_spin.setMinimumWidth(72)
        self.calib_min_images_spin.setSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        self.calib_calculation_pairs_spin.setProperty("compactFormField", True)
        self.calib_calculation_pairs_spin.setMinimumWidth(96)
        self.calib_calculation_pairs_spin.setSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        self.checkerboard_validation_random_runs_spin = QtWidgets.QSpinBox()
        self.checkerboard_validation_random_runs_spin.setRange(1, 20)
        self.checkerboard_validation_random_runs_spin.setValue(self.settings.calibration_validation_runs)
        self.checkerboard_validation_random_pair_count_spin = QtWidgets.QSpinBox()
        self.checkerboard_validation_random_pair_count_spin.setRange(0, 10000)
        self.checkerboard_validation_random_pair_count_spin.setValue(self.settings.calibration_validation_pair_count)
        self.checkerboard_validation_random_pair_count_spin.setSpecialValueText("자동")
        self.checkerboard_validation_seed_spin = QtWidgets.QSpinBox()
        self.checkerboard_validation_seed_spin.setRange(0, 2_147_483_647)
        self.checkerboard_validation_seed_spin.setValue(self.settings.calibration_validation_seed)
        self.checkerboard_validation_seed_spin.setSpecialValueText("자동")
        self.run_calibration_button = QtWidgets.QPushButton("전체 사진으로 계산")
        self.run_calibration_button.clicked.connect(self.run_intrinsic_calibration)
        self.checkerboard_validation_random_button = QtWidgets.QPushButton("랜덤 검증")
        self.checkerboard_validation_random_button.clicked.connect(self.run_checkerboard_random_validation)
        self.checkerboard_validation_save_button = QtWidgets.QPushButton("계산 결과 저장")
        self.checkerboard_validation_save_button.clicked.connect(self.save_checkerboard_validation_result)
        self.checkerboard_validation_apply_button = QtWidgets.QPushButton("계산에 적용")
        self.checkerboard_validation_apply_button.clicked.connect(self.apply_checkerboard_validation_result)
        validation_action_form.addWidget(QtWidgets.QLabel("최소 이미지"), 0, 0)
        validation_action_form.addWidget(self.calib_min_images_spin, 0, 1)
        validation_action_form.addWidget(QtWidgets.QLabel("계산 pair"), 0, 2)
        validation_action_form.addWidget(self.calib_calculation_pairs_spin, 0, 3)
        validation_action_form.addWidget(QtWidgets.QLabel("반복"), 0, 4)
        validation_action_form.addWidget(self.checkerboard_validation_random_runs_spin, 0, 5)
        validation_action_form.addWidget(QtWidgets.QLabel("회당 pair"), 0, 6)
        validation_action_form.addWidget(self.checkerboard_validation_random_pair_count_spin, 0, 7)
        validation_action_form.addWidget(QtWidgets.QLabel("seed"), 0, 8)
        validation_action_form.addWidget(self.checkerboard_validation_seed_spin, 0, 9)
        validation_action_form.addWidget(self.run_calibration_button, 1, 0, 1, 2)
        validation_action_form.addWidget(self.checkerboard_validation_random_button, 1, 2, 1, 2)
        validation_action_form.addWidget(self.checkerboard_validation_save_button, 1, 4, 1, 2)
        validation_action_form.addWidget(self.checkerboard_validation_apply_button, 1, 6, 1, 2)
        self.checkerboard_validation_status_label = QtWidgets.QLabel("계산 결과 없음")
        self.checkerboard_validation_status_label.setObjectName("statusStrip")
        self.checkerboard_validation_status_label.setWordWrap(True)
        validation_action_form.addWidget(self.checkerboard_validation_status_label, 2, 0, 1, 10)
        self.calibration_progress_bar = QtWidgets.QProgressBar()
        self.calibration_progress_bar.setRange(0, 100)
        self.calibration_progress_bar.setValue(0)
        self.calibration_progress_bar.setFormat("대기 · %p%")
        self.calibration_progress_bar.setTextVisible(True)
        validation_action_form.addWidget(self.calibration_progress_bar, 3, 0, 1, 10)

        calibration_top_stack = QtWidgets.QVBoxLayout()
        calibration_top_stack.setSpacing(6)
        calibration_top_stack.addWidget(zoom_group)
        calibration_top_stack.addWidget(calibration_group)
        calibration_top_stack.addWidget(self.calibration_status_label)
        checkerboard_calibration_layout.addLayout(calibration_top_stack)
        checkerboard_calibration_layout.addLayout(calibration_preview_row, 2)
        checkerboard_validation_layout.addWidget(validation_source_group)
        checkerboard_validation_layout.addWidget(validation_action_group)
        checkerboard_validation_layout.addWidget(self.calibration_result_text, 1)

    def build_calibration_profiles(self, zoom_intrinsics_layout):
        zoom_intrinsics_split = QtWidgets.QHBoxLayout()
        zoom_intrinsics_split.setSpacing(8)
        zoom_intrinsics_view_group = QtWidgets.QGroupBox("내부 파라미터")
        zoom_intrinsics_view_layout = QtWidgets.QHBoxLayout(zoom_intrinsics_view_group)
        self.zoom_left_intrinsics_text = self._build_parameter_text()
        self.zoom_right_intrinsics_text = self._build_parameter_text()
        zoom_intrinsics_view_layout.addWidget(self.zoom_left_intrinsics_text, 1)
        zoom_intrinsics_view_layout.addWidget(self.zoom_right_intrinsics_text, 1)
        zoom_intrinsics_action_group = QtWidgets.QGroupBox("저장 / 적용")
        zoom_intrinsics_action_layout = QtWidgets.QVBoxLayout(zoom_intrinsics_action_group)
        self.zoom_intrinsics_status_label = QtWidgets.QLabel("줌: --")
        self.zoom_intrinsics_status_label.setObjectName("statusStrip")
        self.zoom_intrinsics_status_label.setWordWrap(True)
        self.zoom_intrinsics_path_label = QtWidgets.QLabel("파일: --")
        self.zoom_intrinsics_path_label.setObjectName("statusStrip")
        self.zoom_intrinsics_path_label.setWordWrap(True)
        zoom_intrinsics_action_layout.addWidget(self.zoom_intrinsics_status_label)
        zoom_intrinsics_action_layout.addWidget(self.zoom_intrinsics_path_label)
        zoom_intrinsics_action_layout.addWidget(self.save_intrinsics_button)
        zoom_intrinsics_action_layout.addWidget(self.apply_intrinsics_button)
        zoom_intrinsics_action_layout.addStretch(1)
        zoom_intrinsics_split.addWidget(zoom_intrinsics_view_group, 1)
        zoom_intrinsics_split.addWidget(zoom_intrinsics_action_group)
        zoom_intrinsics_layout.addLayout(zoom_intrinsics_split, 1)

        zoom_intrinsics_compare_group = QtWidgets.QGroupBox("저장 파일 비교")
        zoom_intrinsics_compare_layout = QtWidgets.QVBoxLayout(zoom_intrinsics_compare_group)
        zoom_intrinsics_compare_actions = QtWidgets.QHBoxLayout()
        self.add_zoom_intrinsics_file_button = QtWidgets.QPushButton("추가")
        self.add_zoom_intrinsics_file_button.clicked.connect(self.add_zoom_intrinsics_comparison_files)
        self.remove_zoom_intrinsics_file_button = QtWidgets.QPushButton("제거")
        self.remove_zoom_intrinsics_file_button.clicked.connect(self.remove_selected_zoom_intrinsics_comparison_files)
        self.apply_zoom_intrinsics_file_button = QtWidgets.QPushButton("선택 적용")
        self.apply_zoom_intrinsics_file_button.clicked.connect(self.apply_selected_zoom_intrinsics_comparison_file)
        zoom_intrinsics_compare_actions.addWidget(self.add_zoom_intrinsics_file_button)
        zoom_intrinsics_compare_actions.addWidget(self.remove_zoom_intrinsics_file_button)
        zoom_intrinsics_compare_actions.addWidget(self.apply_zoom_intrinsics_file_button)
        zoom_intrinsics_compare_actions.addStretch(1)
        self.zoom_intrinsics_compare_list = QtWidgets.QListWidget()
        self.zoom_intrinsics_compare_list.setObjectName("zoomIntrinsicsCompareList")
        self.zoom_intrinsics_compare_list.setSelectionMode(QtWidgets.QAbstractItemView.ExtendedSelection)
        self.zoom_intrinsics_compare_list.setDragDropMode(QtWidgets.QAbstractItemView.InternalMove)
        self.zoom_intrinsics_compare_list.setDefaultDropAction(QtCore.Qt.MoveAction)
        self.zoom_intrinsics_compare_list.setWordWrap(True)
        self.zoom_intrinsics_compare_list.setTextElideMode(QtCore.Qt.ElideNone)
        self.zoom_intrinsics_compare_list.setMinimumHeight(220)
        self.zoom_intrinsics_compare_list.itemDoubleClicked.connect(
            lambda _item: self.apply_selected_zoom_intrinsics_comparison_file()
        )
        zoom_intrinsics_compare_layout.addLayout(zoom_intrinsics_compare_actions)
        zoom_intrinsics_compare_layout.addWidget(self.zoom_intrinsics_compare_list, 1)
        zoom_intrinsics_layout.addWidget(zoom_intrinsics_compare_group, 1)

