from __future__ import annotations

from typing import Any

from PyQt5 import QtCore, QtWidgets

from .evidence import EvidenceAdapter


class StandalonePanelBackend(QtCore.QObject):
    """Backend for the Workstation-owned copy of the panel.

    This is the only backend adapter in the Workstation package.  It evolves
    with the standalone owner without importing or instantiating a sibling UI
    runtime.
    """

    command_error = QtCore.pyqtSignal(str)

    def __init__(self, panel: QtWidgets.QWidget, parent: QtCore.QObject | None = None) -> None:
        super().__init__(parent)
        self.panel = panel
        self.evidence = EvidenceAdapter(panel)

    @property
    def owner_widget(self) -> QtWidgets.QWidget:
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

    def apply_run_setup(self, values: dict[str, object]) -> Any:
        return self._call("apply_workstation_run_setup", dict(values))

    def run_setup_snapshot(self) -> Any:
        return self._call("workstation_run_setup_snapshot")

    def begin_measurement_session(self) -> Any:
        return self._call("begin_measurement_session")

    def measurement_session_snapshot(self) -> Any:
        return self._call("measurement_session_snapshot")

    def measurement_package_snapshot(self) -> Any:
        return self._call("measurement_package_snapshot")

    def finalize_measurement_package(self) -> Any:
        return self._call("finalize_measurement_package")

    def begin_next_measurement(self) -> Any:
        return self._call("begin_next_measurement")

    def begin_measurement(self) -> Any:
        return self._call("begin_measurement")

    def close_measurement_session(self) -> Any:
        return self._call("close_measurement_session")

    def begin_measurement_revision(self) -> Any:
        return self._call("begin_measurement_revision")

    def verify_measurement_package(self) -> Any:
        return self._call("verify_measurement_package")

    def verify_measurement_session(self) -> Any:
        return self._call("verify_session")

    def optical_pose_snapshot(self, *, include_frames: bool = True) -> Any:
        return self._call(
            "workstation_optical_pose_snapshot",
            include_frames=bool(include_frames),
        )

    def optical_pose_prepare_auto_select(
        self,
        mode: str,
        payload: dict[str, object] | None = None,
    ) -> Any:
        return self._call(
            "prepare_workstation_optical_pose_auto_selection",
            str(mode),
            dict(payload or {}),
        )

    def optical_pose_commit_auto_select(self, suggestion: dict[str, object]) -> Any:
        return self._call(
            "commit_workstation_optical_pose_auto_selection",
            dict(suggestion),
        )

    def optical_pose_capture_burst(self, payload: dict[str, object] | None = None) -> Any:
        return self._call(
            "capture_workstation_optical_pose_burst",
            dict(payload or {}),
        )

    def optical_pose_import_burst(self, payload: dict[str, object]) -> Any:
        return self._call("import_workstation_optical_pose_burst", dict(payload))

    def optical_pose_capture_burst_wiris_ftp(self, payload: dict[str, object] | None = None) -> Any:
        return self._call(
            "capture_workstation_optical_pose_burst_wiris_ftp",
            dict(payload or {}),
        )

    def optical_pose_capture_burst_wiris_ftp_oz(self, payload: dict[str, object] | None = None) -> Any:
        return self._call(
            "capture_workstation_optical_pose_burst_wiris_ftp_oz",
            dict(payload or {}),
        )

    def optical_pose_import_terrain(self, payload: dict[str, object]) -> Any:
        return self._call("import_workstation_optical_pose_terrain", dict(payload))

    def optical_pose_solve_horizon(self, payload: dict[str, object]) -> Any:
        return self._call("solve_workstation_optical_pose_horizon", dict(payload))

    def optical_pose_solve_terrain(self, payload: dict[str, object]) -> Any:
        return self._call("solve_workstation_optical_pose_terrain", dict(payload))

    def optical_pose_solve_gnss_mountain(self, payload: dict[str, object]) -> Any:
        return self._call("solve_workstation_optical_pose_gnss_mountain", dict(payload))

    def optical_pose_solve_drone_yaw(self, payload: dict[str, object]) -> Any:
        return self._call("solve_workstation_optical_pose_drone_yaw", dict(payload))

    def optical_pose_solve_drone_rtk_multiview(self, payload: dict[str, object]) -> Any:
        return self._call(
            "solve_workstation_optical_pose_drone_rtk_multiview",
            dict(payload),
        )

    def optical_pose_solve_combined(self, payload: dict[str, object]) -> Any:
        return self._call("solve_workstation_optical_pose_combined", dict(payload))

    def optical_pose_auto_select_horizon(self, payload: dict[str, object] | None = None) -> Any:
        return self._call(
            "auto_select_workstation_optical_pose_horizon",
            dict(payload or {}),
        )

    def optical_pose_auto_select_mountain(self, payload: dict[str, object] | None = None) -> Any:
        return self._call(
            "auto_select_workstation_optical_pose_mountain",
            dict(payload or {}),
        )

    def optical_pose_set_mountain_ridge_margin(self, payload: dict[str, object]) -> Any:
        return self._call(
            "set_workstation_optical_pose_mountain_ridge_margin",
            dict(payload),
        )

    def optical_pose_set_mountain_max_points(self, payload: dict[str, object]) -> Any:
        return self._call(
            "set_workstation_optical_pose_mountain_max_points",
            dict(payload),
        )

    def optical_pose_clear_auto_suggestions(self, payload: dict[str, object] | None = None) -> Any:
        return self._call(
            "clear_workstation_optical_pose_auto_suggestions",
            dict(payload or {}),
        )

    def optical_pose_register_reference_candidate(self, payload: dict[str, object]) -> Any:
        return self._call(
            "register_workstation_optical_pose_reference_candidate",
            dict(payload),
        )

    def optical_pose_accept_suggestion(self, payload: dict[str, object]) -> Any:
        return self._call(
            "accept_workstation_optical_pose_suggestion",
            dict(payload),
        )

    def optical_pose_record_annotation(self, payload: dict[str, object]) -> Any:
        return self._call(
            "record_workstation_optical_pose_annotation",
            dict(payload),
        )

    def optical_pose_apply_candidate(self, payload: dict[str, object]) -> Any:
        return self._call("apply_workstation_optical_pose_candidate", dict(payload))

    def optical_pose_revert(self, payload: dict[str, object] | None = None) -> Any:
        return self._call("revert_workstation_optical_pose", dict(payload or {}))

    def target_drone_configure(self, payload: dict[str, object]) -> Any:
        return self._call("configure_target_drone_stream", dict(payload))

    def target_drone_connect(self, payload: dict[str, object] | None = None) -> Any:
        return self._call("connect_target_drone_stream", None if payload is None else dict(payload))

    def target_drone_stop(self, *, timeout_s: float | None = None) -> Any:
        return self._call("stop_target_drone_stream", timeout_s=timeout_s)

    def target_drone_snapshot(self) -> Any:
        return self._call("target_drone_stream_snapshot")

    def drone_telemetry_configure(self, payload: dict[str, object]) -> Any:
        return self._call("configure_drone_telemetry", dict(payload))

    def drone_telemetry_connect(self, payload: dict[str, object] | None = None) -> Any:
        return self._call("connect_drone_telemetry", None if payload is None else dict(payload))

    def drone_telemetry_disconnect(self) -> Any:
        return self._call("disconnect_drone_telemetry")

    def drone_telemetry_save_settings(self, payload: dict[str, object]) -> Any:
        return self._call("save_drone_telemetry_settings", dict(payload))

    def drone_telemetry_snapshot(self) -> Any:
        return self._call("drone_telemetry_snapshot")

    def drone_telemetry_discover_usb(self) -> Any:
        return self._call("discover_drone_telemetry_usb")

    # Operation facades delegate to the standalone panel owner.  The
    # Workstation has one camera/telemetry worker per runtime; sibling
    # applications keep independent resource owners.
    def camera_discover_host_ip(self) -> Any:
        return self._call("discover_pc_camera_ip")

    def camera_apply_candidate(self, row: int, target: str) -> Any:
        return self._call("apply_camera_candidate", row, target)

    def camera_auto_connect(self) -> Any:
        return self._call("auto_connect_wiris")

    def camera_check_rtsp(self) -> Any:
        return self._call("check_cameras")

    def camera_check_control(self) -> Any:
        return self._call("check_wiris_control")

    def camera_enable_rtsp(self) -> Any:
        return self._call("enable_wiris_rtsp")

    def camera_reactivate(self, camera: str) -> Any:
        return self._call("reactivate_wiris_rtsp", camera)

    def camera_swap(self) -> Any:
        return self._call("swap_cameras")

    def camera_save_settings(self) -> Any:
        return self._call("save_current_settings_from_ui")

    def camera_refresh_image_format_status(self) -> Any:
        return self._call("refresh_wiris_image_format_status")

    def camera_apply_oz_only_image_format(self) -> Any:
        return self._call("apply_oz_only_image_format")

    def camera_set_image_format(self, set_command: str, value: bool) -> Any:
        return self._call("set_wiris_image_format", set_command, value)

    def time_set_pc(self) -> Any:
        """Set the selected WIRIS camera clock from the existing PC-time path."""

        return self._call("set_selected_wiris_time_from_pc")

    def time_check_sync(self) -> Any:
        """Refresh the existing RTCP/NTP synchronization evidence."""

        return self._call("refresh_clock_sync_status")

    def pixhawk_toggle_mavlink(self) -> Any:
        return self._call("toggle_mavlink_connection")

    def pixhawk_connect_mavlink(self) -> Any:
        return self._call("connect_mavlink_pixhawk")

    def pixhawk_disconnect_mavlink(self) -> Any:
        return self._call("disconnect_mavlink_pixhawk")

    def pixhawk_reset_level_calibration(self) -> Any:
        return self._call("reset_px4_level_calibration")

    def pixhawk_reset_calibration(self, camera: str, group: str) -> Any:
        return self._call("reset_px4_calibration_group", camera, group)

    def pixhawk_recover_eth_mavlink(self, camera: str) -> Any:
        """Recover one PX4 Ethernet MAVLink stream through its USB shell path."""

        return self._call("bootstrap_ethernet_mavlink_from_usb", camera)

    def pixhawk_discover_usb(self) -> Any:
        return self._call("discover_usb_pixhawk_connections")

    def pixhawk_discover_mavlink_candidates(self) -> Any:
        return self._call("discover_mavlink_url_candidates")

    def pixhawk_discover_host_ip(self) -> Any:
        return self._call("discover_pc_pixhawk_ip")

    def pixhawk_apply_candidate(self, source: str, row: int, target: str) -> Any:
        return self._call("apply_pixhawk_candidate", source, row, target)

    def gnss_base_discover(self) -> Any:
        return self._call("discover_gnss_base_serial_ports")

    def gnss_base_apply_candidate(self, row: int) -> Any:
        return self._call("apply_gnss_base_candidate", row)

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
        # The stop handler returns False while its worker is still
        # winding down.  That is a pending postcondition, not a rejected
        # command, so do not surface it as CommandRegistry failure.
        self._call("stop_gnss_network")
        return None

    def gnss_ntrip_forget_password(self) -> Any:
        return self._call("forget_gnss_ntrip_password")

    def set_gnss_ntrip_session_password(self, password: str) -> Any:
        return self._call("set_gnss_ntrip_session_password", password)

    def calibration_capture_images(self) -> Any:
        return self._call("capture_calibration_images")

    def calibration_capture_images_wiris_ftp(self) -> Any:
        return self._call("capture_calibration_images_wiris_ftp")

    def calibration_capture_images_wiris_ftp_oz(self) -> Any:
        return self._call("capture_calibration_images_wiris_ftp_oz")

    def calibration_capture_images_wiris_oz_capture_only(self) -> Any:
        return self._call("capture_calibration_images_wiris_oz_capture_only")

    def calibration_sync_images_wiris_oz_ftp(self) -> Any:
        return self._call("sync_calibration_images_wiris_oz_ftp")

    def calibration_toggle_auto_capture(self) -> Any:
        return self._call("toggle_auto_calibration_capture")

    def capture_pnp_hr_photos(self, purpose: str = "unspecified") -> Any:
        return self._call("capture_pnp_hr_photos", purpose)

    def capture_pnp_oz_photos(self, purpose: str = "unspecified") -> Any:
        return self._call("capture_pnp_oz_photos", purpose)

    def import_hr_files(self, capture_id: str, **values: object) -> Any:
        return self._call("import_hr_files", capture_id, **values)

    def ulog_snapshot(self) -> Any:
        return self._call("workstation_ulog_snapshot")

    def ulog_import(self, payload: dict[str, object]) -> Any:
        return self._call(
            "import_workstation_ulog",
            str(payload.get("source") or "p1"),
            str(payload.get("path") or ""),
            ownership=str(payload.get("ownership") or "UNKNOWN"),
            expected_sha256=payload.get("expected_sha256"),
        )

    def calibration_run_intrinsic(self) -> Any:
        return self._call("run_intrinsic_calibration")

    def calibration_run_extrinsics(self) -> Any:
        return self._call("run_stereo_extrinsics_calibration")

    def calibration_save_intrinsics(self, *, quality_warning_confirmed: bool = False) -> Any:
        return (
            self._call("save_intrinsics", quality_warning_confirmed=True)
            if quality_warning_confirmed
            else self._call("save_intrinsics")
        )

    def calibration_save_extrinsics(self, *, quality_warning_confirmed: bool = False) -> Any:
        return (
            self._call("save_stereo_extrinsics_profile", quality_warning_confirmed=True)
            if quality_warning_confirmed
            else self._call("save_stereo_extrinsics_profile")
        )

    def calibration_load_intrinsics(self) -> Any:
        return self._call("load_intrinsics")

    def calibration_load_extrinsics(self) -> Any:
        return self._call("load_stereo_extrinsics_profile")

    def calibration_load_current_zoom_intrinsics(self) -> Any:
        return self._call("load_current_zoom_intrinsics")

    def calibration_apply_intrinsics(self) -> Any:
        return self._call("apply_intrinsics_to_distance")

    def calibration_apply_bundle(self, *, quality_warning_confirmed: bool = False) -> Any:
        return (
            self._call("apply_calibration_bundle", quality_warning_confirmed=True)
            if quality_warning_confirmed
            else self._call("apply_calibration_bundle")
        )

    def calibration_load_validation(self) -> Any:
        return self._call("load_checkerboard_validation_json")

    def calibration_run_validation(self) -> Any:
        return self._call("run_checkerboard_random_validation")

    def calibration_save_validation(self, *, quality_warning_confirmed: bool = False) -> Any:
        return (
            self._call(
                "save_checkerboard_validation_result",
                quality_warning_confirmed=True,
            )
            if quality_warning_confirmed
            else self._call("save_checkerboard_validation_result")
        )

    def calibration_apply_validation(self, *, quality_warning_confirmed: bool = False) -> Any:
        return (
            self._call(
                "apply_checkerboard_validation_result",
                quality_warning_confirmed=True,
            )
            if quality_warning_confirmed
            else self._call("apply_checkerboard_validation_result")
        )

    def workstation_checkerboard_validation_snapshot(self) -> Any:
        return self._call("workstation_checkerboard_validation_snapshot")

    def workstation_calibration_quality_snapshot(self) -> Any:
        return self._call("workstation_calibration_quality_snapshot")

    def configure_workstation_checkerboard_validation(self, payload: dict[str, object]) -> Any:
        return self._call("configure_workstation_checkerboard_validation", dict(payload))

    def load_workstation_checkerboard_validation_json(self, payload: dict[str, object]) -> Any:
        return self._call("load_workstation_checkerboard_validation_json", dict(payload))

    def run_workstation_checkerboard_validation(self, payload: dict[str, object]) -> Any:
        return self._call("run_workstation_checkerboard_validation", dict(payload))

    def save_workstation_checkerboard_validation_result(
        self, *, quality_warning_confirmed: bool = False
    ) -> Any:
        return (
            self._call(
                "save_workstation_checkerboard_validation_result",
                quality_warning_confirmed=True,
            )
            if quality_warning_confirmed
            else self._call("save_workstation_checkerboard_validation_result")
        )

    def apply_workstation_checkerboard_validation_result(
        self, *, quality_warning_confirmed: bool = False
    ) -> Any:
        return (
            self._call(
                "apply_workstation_checkerboard_validation_result",
                quality_warning_confirmed=True,
            )
            if quality_warning_confirmed
            else self._call("apply_workstation_checkerboard_validation_result")
        )

    def apply_measurement_settings(self, settings: dict[str, object]) -> Any:
        """Apply Workstation-owned controls through the existing panel owner."""

        return self._call("apply_workstation_measurement_settings", dict(settings))

    def measurement_methods_snapshot(self) -> Any:
        return self._call("workstation_measurement_methods_snapshot")

    def select_measurement_method(self, payload: dict[str, object]) -> Any:
        return self._call("select_workstation_measurement_method", dict(payload))

    def configure_measurement_method_fill(self, payload: dict[str, object]) -> Any:
        return self._call(
            "configure_workstation_measurement_method_fill",
            dict(payload),
        )

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
        return self._call("save_workstation_calibration_setup")

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

    # Advanced owner tools. These are intentionally typed entry points for
    # Workstation; the panel remains the sole owner of TCP/MAVLink resources,
    # dialogs, preview workers, and session writers.
    def advanced_wiris_catalog(self) -> Any:
        return self._call("workstation_wiris_command_catalog")

    def query_lrf_once(self) -> Any:
        """Run one low-level WIRIS GLRD read for compatibility/advanced tools.

        Alignment no longer exposes an LRF widget or refresh command.
        """

        return self._call("query_wiris_lrf_once")

    def advanced_send_wiris_command(self, target: str, command: str, terminator: str) -> Any:
        return self._call("workstation_send_wiris_command", target, command, terminator)

    def advanced_send_zoom(self, target: str, action: str, level: int | None = None) -> Any:
        return self._call("workstation_send_zoom", target, action, level)

    def advanced_discover_custom_mavlink_watch(self) -> Any:
        return self._call("workstation_discover_custom_mavlink_watch")

    def advanced_add_custom_mavlink_watch(self, name: str, message: str, field: str) -> Any:
        return self._call("workstation_add_custom_mavlink_watch", name, message, field)

    def advanced_remove_custom_mavlink_watch(self, key: str) -> Any:
        return self._call("workstation_remove_custom_mavlink_watch", key)

    def advanced_refresh_baseline_fields(self) -> Any:
        return self._call("workstation_refresh_baseline_field_browser")

    def advanced_open_px4_shell(self, camera: str) -> Any:
        return self._call("workstation_open_px4_shell", camera)

    def advanced_refresh_px4_mag(self, camera: str, *, read_params: bool = True) -> Any:
        return self._call("workstation_refresh_px4_mag", camera, read_params=read_params)

    def advanced_refresh_px4_attitude_cal(self, camera: str, *, read_params: bool = True) -> Any:
        return self._call("workstation_refresh_px4_attitude_cal", camera, read_params=read_params)

    def advanced_refresh_px4_calibration_params(self, camera: str, *, read_params: bool = True) -> Any:
        return self._call("workstation_refresh_px4_calibration_params", camera, read_params=read_params)

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

    def ftp_diagnostic_text(self) -> str:
        """Return the dedicated WIRIS FTP lifecycle log for Diagnostics."""

        panel = self.panel
        lines = [str(line) for line in (getattr(panel, "ftp_log_lines", ()) or ()) if str(line).strip()]
        if not lines:
            # Keep the view useful for panels created before the dedicated
            # buffer was introduced, and for lightweight test doubles.
            widget = getattr(panel, "wiris_log", None)
            if widget is not None and hasattr(widget, "toPlainText"):
                lines = [
                    line
                    for line in widget.toPlainText().splitlines()
                    if "FTP" in line.upper()
                ]
        status_widget = getattr(panel, "calibration_status_label", None)
        status = str(status_widget.text()).strip() if status_widget is not None else ""
        if status and "FTP" in status.upper() and not any(
            status == line or line.endswith(status) for line in lines
        ):
            lines.append(f"[status] {status}")
        return "\n".join(lines) or "No FTP activity yet."

    def shutdown_for_close(self) -> bool:
        method = getattr(self.panel, "shutdown_for_close", None)
        if callable(method):
            return bool(method())
        stop = getattr(self.panel, "stop_all", None)
        if callable(stop):
            stop(wait=True)
        wait = getattr(self.panel, "_wait_for_session_save", None)
        return True if not callable(wait) else bool(wait())


def create_standalone_backend(
    *,
    auto_start: bool = False,
    auto_start_pixhawk: bool = False,
) -> StandalonePanelBackend:
    from Workstation.standalone_gui import WirisStereoDistancePanel

    panel = WirisStereoDistancePanel(auto_start=auto_start, auto_start_pixhawk=auto_start_pixhawk)
    return StandalonePanelBackend(panel)
