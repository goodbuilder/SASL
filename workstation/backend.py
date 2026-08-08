from __future__ import annotations

from pathlib import Path
from typing import Any

from PyQt5 import QtCore, QtWidgets

from .evidence import EvidenceAdapter


class LegacyPanelBackend(QtCore.QObject):
    """Adapter around the existing panel; it never duplicates measurement logic."""

    command_error = QtCore.pyqtSignal(str)

    def __init__(self, panel: QtWidgets.QWidget, parent: QtCore.QObject | None = None) -> None:
        super().__init__(parent)
        self.panel = panel
        self.evidence = EvidenceAdapter(panel)

    @property
    def legacy_widget(self) -> QtWidgets.QWidget:
        return self.panel

    def _call(self, name: str, *args: object, **kwargs: object) -> Any:
        method = getattr(self.panel, name, None)
        if not callable(method):
            message = f"backend command unavailable: {name}"
            self.command_error.emit(message)
            raise RuntimeError(message)
        try:
            return method(*args, **kwargs)
        except Exception as exc:
            message = f"{name}: {exc}"
            self.command_error.emit(message)
            raise RuntimeError(message) from exc

    def toggle_camera(self) -> Any:
        return self._call("toggle_camera_connection")

    def toggle_detection(self) -> Any:
        return self._call("toggle_yolo_detection")

    def toggle_recording(self) -> Any:
        return self._call("toggle_session_recording")

    def save_session(self) -> Any:
        return self._call("save_distance_history_csv")

    def toggle_pixhawk(self) -> Any:
        return self._call("toggle_ros2_connection")

    def apply_camera_settings(self, left_url: str, right_url: str) -> bool:
        left_edit = getattr(self.panel, "left_url_edit", None)
        right_edit = getattr(self.panel, "right_url_edit", None)
        if left_edit is None or right_edit is None:
            message = "camera URL controls unavailable"
            self.command_error.emit(message)
            raise RuntimeError(message)
        left_edit.setText(str(left_url).strip())
        right_edit.setText(str(right_url).strip())
        saver = getattr(self.panel, "_save_current_settings", None)
        if callable(saver):
            saver()
        return True

    # Phase 1 operation facades.  These deliberately delegate to the existing
    # panel owner; Workstation does not create a second camera/telemetry worker.
    def camera_discover_host_ip(self) -> Any:
        return self._call("discover_pc_camera_ip")

    def camera_check_rtsp(self) -> Any:
        return self._call("check_cameras")

    def camera_check_control(self) -> Any:
        return self._call("check_wiris_control")

    def camera_enable_rtsp(self) -> Any:
        return self._call("enable_wiris_rtsp")

    def camera_swap(self) -> Any:
        return self._call("swap_cameras")

    def camera_save_settings(self) -> Any:
        return self._call("save_current_settings_from_ui")

    def time_set_pc(self) -> Any:
        """Set the selected WIRIS camera clock from the existing PC-time path."""

        return self._call("set_selected_wiris_time_from_pc")

    def time_run_photo_workflow(self) -> Any:
        """Run the existing CPTR/ICPT/FTP/EXIF clock verification workflow."""

        return self._call("capture_download_analyze_clock_photos")

    def time_check_sync(self) -> Any:
        """Refresh the existing RTCP/NTP synchronization evidence."""

        return self._call("refresh_clock_sync_status")

    def pixhawk_auto_detect(self) -> Any:
        return self._call("auto_detect_pixhawks")

    def pixhawk_toggle_mavlink(self) -> Any:
        return self._call("toggle_mavlink_connection")

    def gnss_base_discover(self) -> Any:
        return self._call("discover_gnss_base_serial_ports")

    def gnss_base_connect(self) -> Any:
        return self._call("connect_gnss_base_serial")

    def gnss_base_disconnect(self) -> Any:
        return self._call("disconnect_gnss_base_serial")

    def gnss_survey_start(self) -> Any:
        return self._call("start_gnss_base_survey_in")

    def gnss_survey_stop(self) -> Any:
        return self._call("stop_gnss_base_survey_in")

    def gnss_ntrip_start(self) -> Any:
        return self._call("start_gnss_network")

    def gnss_ntrip_stop(self) -> Any:
        # The legacy stop handler returns False while its worker is still
        # winding down.  That is a pending postcondition, not a rejected
        # command, so do not surface it as CommandRegistry failure.
        self._call("stop_gnss_network")
        return None

    def calibration_capture_images(self) -> Any:
        return self._call("capture_calibration_images")

    def calibration_run_intrinsic(self) -> Any:
        return self._call("run_intrinsic_calibration")

    def calibration_save_intrinsics(self) -> Any:
        return self._call("save_intrinsics")

    def calibration_load_intrinsics(self) -> Any:
        return self._call("load_intrinsics")

    def calibration_apply_intrinsics(self) -> Any:
        return self._call("apply_intrinsics_to_distance")

    def calibration_load_validation(self) -> Any:
        return self._call("load_checkerboard_validation_json")

    def calibration_run_validation(self) -> Any:
        return self._call("run_checkerboard_random_validation")

    def calibration_save_validation(self) -> Any:
        return self._call("save_checkerboard_validation_result")

    def calibration_apply_validation(self) -> Any:
        return self._call("apply_checkerboard_validation_result")

    def apply_measurement_settings(self, settings: dict[str, object]) -> Any:
        """Apply Workstation-owned controls through the existing panel owner."""

        return self._call("apply_workstation_measurement_settings", dict(settings))

    def save_measurement_settings(self) -> Any:
        return self._call("save_current_settings_from_ui")

    def workstation_settings_snapshot(self) -> Any:
        return self._call("workstation_settings_snapshot")

    def apply_workstation_system_settings(self, settings: dict[str, object]) -> Any:
        return self._call("apply_workstation_system_settings", dict(settings))

    def save_workstation_system_settings(self) -> Any:
        return self._call("save_current_settings_from_ui")

    def calibration_apply_setup(self, setup: dict[str, object]) -> Any:
        return self._call("apply_workstation_calibration_setup", dict(setup))

    def calibration_scan_validation(self) -> Any:
        return self._call("scan_checkerboard_validation_files")

    def calibration_save_setup(self) -> Any:
        return self._call("save_current_settings_from_ui")

    def alignment_scan_sources(self) -> Any:
        return self._call("refresh_baseline_field_browser")

    def alignment_start_sampling(self) -> Any:
        return self._call("start_baseline_sampling")

    def alignment_stop_sampling(self) -> Any:
        return self._call("stop_baseline_sampling")

    def alignment_save_profile(self) -> Any:
        return self._call("save_alignment_profile")

    def alignment_load_profile(self) -> Any:
        return self._call("load_alignment_profile")

    def alignment_swap_pixhawks(self) -> Any:
        return self._call("swap_pixhawks")

    # Advanced owner tools.  These are intentionally typed entry points for
    # Workstation; the panel remains the sole owner of TCP/MAVLink/ROS2
    # resources, dialogs, preview workers, and session writers.
    def advanced_wiris_catalog(self) -> Any:
        return self._call("workstation_wiris_command_catalog")

    def advanced_send_wiris_command(self, target: str, command: str, terminator: str) -> Any:
        return self._call("workstation_send_wiris_command", target, command, terminator)

    def advanced_send_zoom(self, target: str, action: str, level: int | None = None) -> Any:
        return self._call("workstation_send_zoom", target, action, level)

    def advanced_discover_custom_topic_watch(self) -> Any:
        return self._call("workstation_discover_custom_topic_watch")

    def advanced_discover_custom_mavlink_watch(self) -> Any:
        return self._call("workstation_discover_custom_mavlink_watch")

    def advanced_add_custom_topic_watch(self, name: str, topic: str, field: str) -> Any:
        return self._call("workstation_add_custom_topic_watch", name, topic, field)

    def advanced_remove_custom_topic_watch(self, key: str) -> Any:
        return self._call("workstation_remove_custom_topic_watch", key)

    def advanced_refresh_baseline_fields(self) -> Any:
        return self._call("workstation_refresh_baseline_field_browser")

    def advanced_describe_baseline_fields(self, topic_type: str) -> Any:
        return self._call("workstation_describe_baseline_fields", topic_type)

    def advanced_open_px4_shell(self, camera: str) -> Any:
        return self._call("workstation_open_px4_shell", camera)

    def advanced_refresh_px4_mag(self, camera: str, *, read_params: bool = True) -> Any:
        return self._call("workstation_refresh_px4_mag", camera, read_params=read_params)

    def advanced_refresh_px4_attitude_cal(self, camera: str, *, read_params: bool = True) -> Any:
        return self._call("workstation_refresh_px4_attitude_cal", camera, read_params=read_params)

    def advanced_apply_px4_mag_priorities(self, camera: str, priorities: dict[str, int]) -> Any:
        return self._call("workstation_apply_px4_mag_priorities", camera, dict(priorities))

    def advanced_save_px4_mag_params(self, camera: str) -> Any:
        return self._call("workstation_save_px4_mag_params", camera)

    def advanced_reboot_px4(self, camera: str) -> Any:
        return self._call("workstation_reboot_px4", camera)

    def advanced_refresh_calibration_preview(self) -> Any:
        return self._call("workstation_refresh_calibration_preview")

    def advanced_calibration_preview_snapshot(self) -> Any:
        return self._call("workstation_calibration_preview_snapshot")

    def select_legacy_context(self, context_id: str) -> None:
        tabs = getattr(self.panel, "tabs", None)
        if tabs is None:
            return
        index_attr = {
            "devices.cameras": "image_tab_index",
            "devices.pixhawk": "alignment_tab_index",
            "devices.gnss": "alignment_tab_index",
            "processing.alignment": "alignment_tab_index",
            "processing.calibration": "calibration_tab_index",
            "tasks.calibration": "calibration_tab_index",
            "tasks.recording": "results_tab_index",
            "tasks.export": "results_tab_index",
            "diagnostics.measurement": "depth_tab_index",
        }.get(context_id)
        if index_attr is None:
            return
        index = getattr(self.panel, index_attr, None)
        if isinstance(index, int):
            tabs.setCurrentIndex(index)

    def diagnostic_text(self) -> str:
        sources = []
        for attr in ("camera_yolo_log", "pixhawk_log", "wiris_log"):
            widget = getattr(self.panel, attr, None)
            if widget is None or not hasattr(widget, "toPlainText"):
                continue
            text = widget.toPlainText().strip()
            if text:
                sources.append(f"[{attr}]\n{text}")
        return "\n\n".join(sources)

    def shutdown_for_close(self) -> bool:
        method = getattr(self.panel, "shutdown_for_close", None)
        if callable(method):
            return bool(method())
        stop = getattr(self.panel, "stop_all", None)
        if callable(stop):
            stop(wait=True)
        wait = getattr(self.panel, "_wait_for_session_save", None)
        return True if not callable(wait) else bool(wait())
