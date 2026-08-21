"""Workstation AppSettings apply/persist/snapshot controller.

Owns the surface that bridges Workstation-side setup values (system,
measurement/time-sync, calibration) onto the panel's owner widgets, and the
inverse: reading owner widget state back into a persisted ``AppSettings``.
This module performs no coordinate-frame reinterpretation.
"""

from __future__ import annotations

from pathlib import Path

from PyQt5 import QtGui, QtWidgets

from Workstation.drone_telemetry_controller import normalize_drone_rtcm_source
from Workstation.workstation_core.gnss_operation import (
    GnssOperationConfig,
    GnssOperationMode,
    GnssOperationState,
)
from Workstation.workstation_core.network import DEFAULT_CAMERA_URLS, DEFAULT_HOST_ADDRESS, DEFAULT_INTERFACE
from Workstation.workstation_core.settings import (
    AppSettings,
    resolve_settings_path,
    save_settings as _save_shared_settings,
)
from Workstation.workstation_core.measurement_methods import (
    normalize_measurement_method_fallback_order,
    normalize_measurement_method_id,
)
from Workstation.workstation_core.yolo_stream import DEFAULT_YOLO_WEIGHTS

SETTINGS_CONTROLLER_METHODS = [
    "save_current_settings_from_ui",
    "save_workstation_calibration_setup",
    "workstation_settings_snapshot",
    "apply_workstation_system_settings",
    "apply_workstation_measurement_settings",
    "apply_workstation_calibration_setup",
    "workstation_calibration_preview_snapshot",
    "_save_current_settings",
]


class WorkstationSettingsController:
    def __init__(self, panel):
        self.panel = panel

    def save_current_settings_from_ui(self) -> None:
        panel = self.panel
        panel._save_current_settings()
        panel._log("[저장] 현재 StereoWIRIS 설정을 저장했습니다.")

    def save_workstation_calibration_setup(self) -> None:
        """Persist the calibration setup surfaced by the Workstation shell."""

        panel = self.panel
        panel._save_current_settings()
        panel._log("[저장] Workstation 캘리브레이션 설정을 저장했습니다.")

    def workstation_settings_snapshot(self) -> dict[str, object]:
        """Return the complete editable AppSettings contract for Workstation."""

        panel = self.panel
        snapshot = dict(panel.settings.__dict__)
        snapshot["gnss_operation"] = panel.settings.gnss_operation.to_mapping()
        for key, value in panel._workstation_setting_overrides.items():
            snapshot[key] = value
        return snapshot

    def apply_workstation_system_settings(self, values: dict[str, object]) -> None:
        """Apply the complete Workstation system-settings surface to owner widgets."""

        panel = self.panel

        def has(name: str) -> bool:
            return name in values and values[name] is not None

        def set_text(name: str, attribute: str) -> None:
            if not has(name):
                return
            widget = getattr(panel, attribute, None)
            setter = getattr(widget, "setText", None)
            if callable(setter):
                setter(str(values[name] or ""))

        def set_value(name: str, attribute: str) -> None:
            if not has(name):
                return
            widget = getattr(panel, attribute, None)
            setter = getattr(widget, "setValue", None)
            if callable(setter):
                value = values[name]
                if isinstance(widget, QtWidgets.QDoubleSpinBox):
                    value = float(value)
                elif isinstance(widget, QtWidgets.QSpinBox):
                    value = int(value)
                setter(value)

        def set_numeric_widget(widget: object, value: object) -> None:
            setter = getattr(widget, "setValue", None)
            if not callable(setter):
                return
            if isinstance(widget, QtWidgets.QDoubleSpinBox):
                setter(float(value))
            elif isinstance(widget, QtWidgets.QSpinBox):
                setter(int(value))
            else:
                setter(value)

        def set_checked(name: str, attribute: str) -> None:
            if not has(name):
                return
            widget = getattr(panel, attribute, None)
            setter = getattr(widget, "setChecked", None)
            if callable(setter):
                setter(bool(values[name]))

        def set_combo(name: str, attribute: str, default: str = "") -> None:
            if not has(name):
                return
            panel._set_combo_data(getattr(panel, attribute), str(values[name] or default))

        # Keep an in-memory mirror until the next explicit Save.  This makes
        # navigating away and back in Workstation lossless while leaving the
        # JSON file unchanged until the user chooses Save.
        for name, value in values.items():
            if name != "gnss_operation":
                if name == "measurement_method_fallback_order":
                    value = normalize_measurement_method_fallback_order(value)
                elif name == "measurement_preferred_method":
                    value = normalize_measurement_method_id(value)
                panel._workstation_setting_overrides[name] = value
        if (
            not str(getattr(panel, "_measurement_id", "") or "")
            and {
                "measurement_preferred_method",
                "measurement_method_fallback_order",
            }
            & set(values)
        ):
            panel._measurement_method_config_token = None

        simple_text = {
            "interface_name": "interface_edit",
            "camera_host_address": "host_address_edit",
            "left_rtsp_url": "left_url_edit",
            "right_rtsp_url": "right_url_edit",
            "left_control_host": "left_zoom_host_edit",
            "right_control_host": "right_zoom_host_edit",
            "left_stip_ip": "left_stip_ip_edit",
            "right_stip_ip": "right_stip_ip_edit",
            "stip_subnet_mask": "stip_mask_edit",
            "yolo_weights_path": "weights_edit",
            "pixhawk_host_address": "pixhawk_host_address_edit",
            "left_pixhawk_eth_ip": "left_pixhawk_eth_ip_edit",
            "right_pixhawk_eth_ip": "right_pixhawk_eth_ip_edit",
            "left_mavlink_url": "left_mavlink_url_edit",
            "right_mavlink_url": "right_mavlink_url_edit",
            "left_pixhawk_bootstrap_url": "left_pixhawk_bootstrap_url_edit",
            "right_pixhawk_bootstrap_url": "right_pixhawk_bootstrap_url_edit",
            "gnss_base_serial_port": "gnss_base_serial_port_edit",
        }
        for name, attribute in simple_text.items():
            set_text(name, attribute)

        if has("drone_telemetry_url"):
            panel.drone_telemetry_url = str(values["drone_telemetry_url"] or "")

        simple_values = {
            "wiris_control_port": "zoom_port_spin",
            "left_control_port": "left_control_port_spin",
            "right_control_port": "right_control_port_spin",
            "wiris_control_timeout_s": "zoom_timeout_spin",
            "preview_width": "preview_width_spin",
            "rtsp_latency_ms": "rtsp_latency_spin",
            "yolo_input_size": "input_size_spin",
            "yolo_event_queue_size": "yolo_event_queue_size_spin",
            "yolo_frame_queue_size": "yolo_frame_queue_size_spin",
            "left_mavlink_baud": "left_mavlink_baud_spin",
            "right_mavlink_baud": "right_mavlink_baud_spin",
            "left_pixhawk_bootstrap_baud": "left_pixhawk_bootstrap_baud_spin",
            "right_pixhawk_bootstrap_baud": "right_pixhawk_bootstrap_baud_spin",
            "mavlink_stream_rate": "mavlink_stream_rate_spin",
            "gnss_base_baud": "gnss_base_baud_spin",
            "gnss_base_survey_in_min_duration_s": "gnss_base_survey_duration_spin",
            "gnss_base_survey_in_accuracy_m": "gnss_base_survey_accuracy_spin",
        }
        for name, attribute in simple_values.items():
            set_value(name, attribute)

        if has("drone_telemetry_baud"):
            panel.drone_telemetry_baud = int(values["drone_telemetry_baud"])
        simple_checks = {
            "stip_reboot": "stip_reboot_checkbox",
            "mavlink_configure_streams": "mavlink_configure_streams_checkbox",
        }
        for name, attribute in simple_checks.items():
            set_checked(name, attribute)

        if has("drone_telemetry_rtcm_forward"):
            panel.drone_telemetry_rtcm_forward = bool(values["drone_telemetry_rtcm_forward"])
        if has("drone_telemetry_rtcm_source"):
            panel.drone_telemetry_rtcm_source = normalize_drone_rtcm_source(
                values["drone_telemetry_rtcm_source"]
            )

        set_combo("yolo_device", "device_combo", "cpu")
        set_text("yolo_device", "device_edit")
        set_combo("rtsp_decoder", "decoder_combo", "auto")
        set_combo("stream_backend", "stream_backend_combo", "gstreamer")
        set_combo("rtsp_transport", "rtsp_transport_combo", "tcp")
        set_combo("pixhawk_ethernet_mode", "pixhawk_ethernet_mode_combo", "mavlink")

        if has("manual_focal_override"):
            panel.manual_focal_checkbox.setChecked(bool(values["manual_focal_override"]))

        if has("gnss_operation"):
            operation = GnssOperationConfig.from_mapping(values["gnss_operation"]).to_mapping()
            operation["routing"] = {
                **operation["routing"],
                "field_base_enabled": operation["operation_mode"]
                == GnssOperationMode.NETWORK_ASSISTED_FIXED_BASE.value,
            }
            config = GnssOperationConfig.from_mapping(operation)
            operation = config.to_mapping()
            panel._workstation_setting_overrides["gnss_operation"] = operation
            panel._set_combo_data(panel.gnss_operation_mode_combo, operation["operation_mode"])
            panel.gnss_site_label_edit.setText(str(operation.get("site_label", "")))
            panel.gnss_site_address_edit.setText(str(operation.get("site_address", "")))
            set_numeric_widget(panel.gnss_rover_min_fix_type_spin, operation["readiness"].get("minimum_fix_type", 3))
            set_checked("gnss_readiness_gate_enabled", "gnss_readiness_gate_checkbox")
            network = operation["network"]
            for name, attribute in (
                ("caster_host", "gnss_ntrip_host_edit"),
                ("mountpoint", "gnss_ntrip_mountpoint_edit"),
                ("username", "gnss_ntrip_username_edit"),
                ("username_env", "gnss_ntrip_username_env_edit"),
                ("password_env", "gnss_ntrip_password_env_edit"),
            ):
                widget = getattr(panel, attribute, None)
                if widget is not None:
                    widget.setText(str(network[name]))
            # The password itself is intentionally not part of ``network`` --
            # see NetworkConfig / ntrip_credentials.py -- so it never appears
            # in this ``values``/settings mapping and is not handled here.
            # The Settings workspace's own password field
            # (SettingsWorkspace.gnss_network_password_edit) lives on a
            # different object than ``panel`` and was never reachable via
            # ``getattr(panel, ...)``; see
            # GnssRuntimeController.set_gnss_ntrip_session_password, called
            # directly from main_window.py's ``_settings_apply`` where both
            # objects are actually in scope, for the real widget-to-widget
            # handoff.
            remember_widget = getattr(panel, "gnss_ntrip_remember_checkbox", None)
            if remember_widget is not None:
                remember_widget.setChecked(bool(network["remember_password"]))
            for name, attribute in (
                ("caster_port", "gnss_ntrip_port_spin"),
                ("acquisition_duration_s", "gnss_fixed_acquisition_duration_spin"),
                ("minimum_fixed_duration_s", "gnss_fixed_min_duration_spin"),
            ):
                source = network if name == "caster_port" else operation["fixed_base"]
                widget = getattr(panel, attribute, None)
                if widget is not None:
                    set_numeric_widget(widget, source[name])
            panel._set_combo_data(panel.gnss_ntrip_gga_policy_combo, network["gga_policy"])
            if hasattr(panel, "_refresh_ntrip_credential_status"):
                panel._refresh_ntrip_credential_status()
            panel._set_combo_data(panel.gnss_fixed_height_combo, operation["fixed_base"]["height_type"])
            if has("gnss_base_rtcm_injection_target"):
                panel._set_combo_data(
                    panel.gnss_base_rtcm_target_combo, str(values["gnss_base_rtcm_injection_target"])
                )
            manager_state = panel.gnss_operation_manager.state
            if manager_state in {
                GnssOperationState.DISCONNECTED,
                GnssOperationState.STOPPED,
                GnssOperationState.CONFIGURED,
                GnssOperationState.FAULT,
                GnssOperationState.INVALIDATED,
            }:
                panel.gnss_operation_manager.configure(config)
            else:
                panel._log(
                    "[GNSS] 활성 운용 중에는 새 Workstation GNSS 설정을 다음 운용 시작 시 적용합니다."
                )
            panel._handle_gnss_operation_mode_changed()

        panel._refresh_yolo_device_toggle()
        panel._refresh_pixhawk_ethernet_mode_controls()
        panel._set_focal_override_enabled(panel.manual_focal_checkbox.isChecked())
        panel._refresh_operation_header()
        panel._log("[설정] Workstation 전체 설정을 owner widget에 적용했습니다.")

    def apply_workstation_measurement_settings(self, values: dict[str, object]) -> None:
        """Apply Workstation Measurement and Time/Sync controls to owner widgets.

        This is intentionally a thin UI bridge.  The panel remains the single
        owner of AppSettings, stream workers, measurement gates, geometry, and
        camera synchronization limits;
        Workstation only supplies validated widget values.  Timing values keep
        their displayed units (milliseconds/percent) until the existing
        ``_save_current_settings`` conversion writes AppSettings.
        """

        panel = self.panel

        def has(name: str) -> bool:
            return name in values and values[name] is not None

        def set_value(name: str, attribute: str) -> None:
            if not has(name):
                return
            widget = getattr(panel, attribute, None)
            setter = getattr(widget, "setValue", None)
            if callable(setter):
                setter(values[name])

        def set_checked(name: str, attribute: str) -> None:
            if not has(name):
                return
            widget = getattr(panel, attribute, None)
            setter = getattr(widget, "setChecked", None)
            if callable(setter):
                setter(bool(values[name]))

        def set_text(name: str, attribute: str) -> None:
            if not has(name):
                return
            widget = getattr(panel, attribute, None)
            setter = getattr(widget, "setText", None)
            if callable(setter):
                setter(str(values[name] or ""))

        if has("class_gate_enabled") or has("class_gate"):
            current_enabled = bool(panel.yolo_gate_class_checkbox.isChecked())
            current_target = panel.yolo_gate_class_edit.text().strip()
            requested_enabled = (
                bool(values["class_gate_enabled"])
                if has("class_gate_enabled")
                else current_enabled
            )
            requested_target = (
                str(values["class_gate"] or "").strip()
                if has("class_gate")
                else current_target
            )
            pending_start = getattr(panel, "_pending_camera_start", None)
            detection_running = bool(getattr(panel, "yolo_enabled", False)) or bool(
                isinstance(pending_start, tuple) and pending_start and pending_start[0]
            )
            if detection_running and (requested_enabled, requested_target) != (
                current_enabled,
                current_target,
            ):
                raise RuntimeError(
                    "target class gate applies when Detection starts; stop Detection before changing it"
                )

        if has("intrinsics_source"):
            source = str(values["intrinsics_source"] or "checkerboard")
            if source not in {"manual", "checkerboard"}:
                raise ValueError(f"unsupported intrinsics source: {source}")
            panel._set_combo_data(panel.intrinsics_source_combo, source)
            panel.manual_focal_checkbox.setChecked(source == "manual")
        elif has("manual_focal_override"):
            panel.manual_focal_checkbox.setChecked(bool(values["manual_focal_override"]))
        if has("distance_mode"):
            mode = str(values["distance_mode"] or "pixhawk")
            if mode not in {"pixhawk", "checkerboard"}:
                raise ValueError(f"unsupported distance mode: {mode}")
            panel._set_combo_data(panel.distance_mode_combo, mode)

        set_value("focal_px", "focal_spin")
        set_value("yolo_input_size_px", "input_size_spin")
        set_value("yolo_confidence", "conf_spin")
        set_value("yolo_iou", "iou_spin")
        set_checked("class_gate_enabled", "yolo_gate_class_checkbox")
        set_text("class_gate", "yolo_gate_class_edit")
        set_checked("matching_gate_enabled", "yolo_gate_matching_checkbox")
        if has("time_basis"):
            panel._set_combo_data(panel.yolo_gate_time_basis_combo, str(values["time_basis"] or "camera_rtcp"))
        set_checked("time_delta_gate_enabled", "yolo_gate_time_delta_checkbox")
        set_value("max_pair_delta_s", "yolo_gate_time_delta_spin")
        set_checked("vertical_delta_gate_enabled", "yolo_gate_vertical_delta_checkbox")
        set_value("max_vertical_delta_px", "yolo_gate_vertical_delta_spin")
        set_value("camera_sync_history_window_s", "camera_sync_history_window_spin")
        set_value("camera_sync_receive_pair_max_delta_ms", "camera_sync_receive_pair_max_delta_spin")
        set_value("camera_sync_rtcp_pair_max_delta_ms", "camera_sync_rtcp_pair_max_delta_spin")
        set_value("camera_sync_max_sr_age_s", "camera_sync_max_sr_age_spin")
        set_value("camera_sync_max_pc_offset_ms", "camera_sync_max_pc_offset_spin")
        set_value("camera_sync_clock_rate_error_pct", "camera_sync_clock_rate_error_spin")
        set_value("camera_sync_required_sr_count", "camera_sync_required_sr_count_spin")
        set_value("camera_sync_warmup_timeout_s", "camera_sync_warmup_timeout_spin")

        panel._set_focal_override_enabled(panel.manual_focal_checkbox.isChecked())
        panel._refresh_yolo_gate_controls()
        panel._refresh_distance()
        panel._refresh_camera_parameters_tab()
        panel._refresh_operation_header()
        panel._log(
            "[설정] Workstation Measurement/Time-Sync 설정 적용: "
            f"intrinsics={panel._intrinsics_source()}, "
            f"extrinsics={panel._distance_mode()}, "
            "public output=baseline_center_frd [Forward,Right,Down,Range]"
        )

    def apply_workstation_calibration_setup(self, setup: dict[str, object]) -> None:
        """Apply Workstation calibration setup fields to the existing owner."""

        panel = self.panel

        def set_value(name: str, attribute: str) -> None:
            if name not in setup or setup[name] is None:
                return
            widget = getattr(panel, attribute, None)
            setter = getattr(widget, "setValue", None)
            if callable(setter):
                setter(setup[name])

        def set_text(name: str, attribute: str) -> None:
            if name not in setup or setup[name] is None:
                return
            widget = getattr(panel, attribute, None)
            setter = getattr(widget, "setText", None)
            if callable(setter):
                setter(str(setup[name]))

        set_value("zoom_level", "zoom_level_spin")
        set_value("inner_cols", "calib_cols_spin")
        set_value("inner_rows", "calib_rows_spin")
        set_value("square_size_mm", "calib_square_spin")
        set_value("min_images", "calib_min_images_spin")
        set_value("calculation_pair_count", "calib_calculation_pairs_spin")
        set_value("auto_interval_s", "calibration_auto_interval_spin")
        set_value("auto_target_pairs", "calibration_auto_target_spin")
        if "calibration_root" in setup:
            panel._calibration_session_root = None
            panel._calibration_session_roots = {}
        set_text("calibration_root", "calib_dir_edit")
        if "validation_mode" in setup and setup["validation_mode"] is not None:
            panel._set_combo_data(
                panel.checkerboard_validation_mode_combo,
                str(setup["validation_mode"] or "pixhawk"),
            )
        set_text("validation_root", "checkerboard_validation_root_edit")
        set_text("validation_json", "checkerboard_validation_json_edit")
        set_value("validation_runs", "checkerboard_validation_random_runs_spin")
        set_value("validation_pair_count", "checkerboard_validation_random_pair_count_spin")
        set_value("validation_seed", "checkerboard_validation_seed_spin")
        panel._refresh_all_calibration_previews()
        panel._refresh_zoom_intrinsics_tab()
        panel.scan_checkerboard_validation_files()
        panel._refresh_operation_header()
        panel._log(
            "[캘리브레이션] Workstation setup 적용: "
            f"checkerboard={panel.calib_cols_spin.value()}x{panel.calib_rows_spin.value()}, "
            f"square={panel.calib_square_spin.value():.3f} mm, zoom={panel.zoom_level_spin.value():02d}; "
            "image geometry remains OpenCV optical RDF"
        )

    def workstation_calibration_preview_snapshot(self) -> dict[str, object]:
        panel = self.panel
        images: dict[str, QtGui.QImage] = {}
        raw_images: dict[str, QtGui.QImage] = {}
        overlay_images: dict[str, QtGui.QImage] = {}
        for camera in ("left", "right"):
            raw = panel.latest_calibration_preview_images.get(camera)
            overlay = panel.calibration_preview_overlay_images.get(camera)
            if isinstance(raw, QtGui.QImage) and not raw.isNull():
                raw_images[camera] = raw.copy()
            if isinstance(overlay, QtGui.QImage) and not overlay.isNull():
                overlay_images[camera] = overlay.copy()
                images[camera] = overlay.copy()
            elif isinstance(raw, QtGui.QImage) and not raw.isNull():
                images[camera] = raw.copy()
        return {
            "images": images,
            "raw_images": raw_images,
            "overlay_images": overlay_images,
            "axis_contract": "checkerboard image pixels; left/right camera optical RDF labels remain unchanged",
        }

    def _save_current_settings(self, intrinsics_path: Path | None = None) -> None:
        panel = self.panel
        path = intrinsics_path
        if path is None:
            configured_path = str(panel._workstation_setting("intrinsics_path", "") or "").strip()
            if configured_path:
                path = resolve_settings_path(configured_path, base_dir=panel.app_root)
            else:
                root, _left_dir, _right_dir = panel._calibration_dirs()
                path = root / "intrinsics.json"
        relative = panel._app_relative_path_text(path)
        panel.settings = AppSettings(
            intrinsics_path=relative,
            yolo_weights_path=panel.weights_edit.text().strip() or DEFAULT_YOLO_WEIGHTS,
            interface_name=panel.interface_edit.text().strip() or DEFAULT_INTERFACE,
            camera_host_address=panel.host_address_edit.text().strip() or DEFAULT_HOST_ADDRESS,
            left_rtsp_url=panel.left_url_edit.text().strip() or DEFAULT_CAMERA_URLS[0],
            right_rtsp_url=panel.right_url_edit.text().strip() or DEFAULT_CAMERA_URLS[1],
            left_control_host=panel.left_zoom_host_edit.text().strip(),
            right_control_host=panel.right_zoom_host_edit.text().strip(),
            left_stip_ip=panel.left_stip_ip_edit.text().strip(),
            right_stip_ip=panel.right_stip_ip_edit.text().strip(),
            stip_subnet_mask=panel.stip_mask_edit.text().strip(),
            stip_reboot=panel.stip_reboot_checkbox.isChecked(),
            wiris_control_port=panel.zoom_port_spin.value(),
            left_control_port=panel.left_control_port_spin.value(),
            right_control_port=panel.right_control_port_spin.value(),
            wiris_control_timeout_s=panel.zoom_timeout_spin.value(),
            yolo_device=panel.device_edit.text().strip() or "cpu",
            yolo_input_size=panel.input_size_spin.value(),
            yolo_conf_threshold=panel.conf_spin.value(),
            yolo_iou_threshold=panel.iou_spin.value(),
            yolo_gate_class_enabled=panel.yolo_gate_class_checkbox.isChecked(),
            yolo_gate_class_text=panel.yolo_gate_class_edit.text().strip(),
            yolo_gate_matching_enabled=panel.yolo_gate_matching_checkbox.isChecked(),
            yolo_gate_time_delta_enabled=panel.yolo_gate_time_delta_checkbox.isChecked(),
            yolo_gate_vertical_delta_enabled=panel.yolo_gate_vertical_delta_checkbox.isChecked(),
            yolo_gate_max_vertical_delta_px=panel.yolo_gate_vertical_delta_spin.value(),
            yolo_gate_max_time_delta_s=panel.yolo_gate_time_delta_spin.value(),
            yolo_gate_time_basis=str(panel.yolo_gate_time_basis_combo.currentData() or "camera_rtc_required"),
            preview_width=panel.preview_width_spin.value(),
            calibration_zoom_level=panel.zoom_level_spin.value(),
            calibration_inner_cols=panel.calib_cols_spin.value(),
            calibration_inner_rows=panel.calib_rows_spin.value(),
            calibration_square_size_mm=panel.calib_square_spin.value(),
            calibration_min_images=panel.calib_min_images_spin.value(),
            calibration_calculation_pair_count=panel.calib_calculation_pairs_spin.value(),
            calibration_auto_interval_s=panel.calibration_auto_interval_spin.value(),
            calibration_auto_target_pairs=panel.calibration_auto_target_spin.value(),
            calibration_root=panel.calib_dir_edit.text().strip(),
            calibration_validation_mode=str(
                panel.checkerboard_validation_mode_combo.currentData() or "pixhawk"
            ),
            calibration_validation_root=panel.checkerboard_validation_root_edit.text().strip(),
            calibration_validation_json=panel.checkerboard_validation_json_edit.text().strip(),
            calibration_validation_runs=panel.checkerboard_validation_random_runs_spin.value(),
            calibration_validation_pair_count=panel.checkerboard_validation_random_pair_count_spin.value(),
            calibration_validation_seed=panel.checkerboard_validation_seed_spin.value(),
            rtsp_latency_ms=panel.rtsp_latency_spin.value(),
            yolo_event_queue_size=panel.yolo_event_queue_size_spin.value(),
            yolo_frame_queue_size=panel.yolo_frame_queue_size_spin.value(),
            rtsp_decoder=panel.decoder_combo.currentData(),
            stream_backend=panel.stream_backend_combo.currentData(),
            rtsp_transport=panel.rtsp_transport_combo.currentData(),
            camera_sync_history_window_s=panel.camera_sync_history_window_spin.value(),
            camera_sync_receive_pair_max_delta_s=panel.camera_sync_receive_pair_max_delta_spin.value() / 1000.0,
            camera_sync_rtcp_pair_max_delta_s=panel.camera_sync_rtcp_pair_max_delta_spin.value() / 1000.0,
            camera_sync_max_sr_age_s=panel.camera_sync_max_sr_age_spin.value(),
            camera_sync_max_pc_offset_s=panel.camera_sync_max_pc_offset_spin.value() / 1000.0,
            camera_sync_max_clock_rate_error=panel.camera_sync_clock_rate_error_spin.value() / 100.0,
            camera_sync_required_sr_count=panel.camera_sync_required_sr_count_spin.value(),
            camera_sync_warmup_timeout_s=panel.camera_sync_warmup_timeout_spin.value(),
            pixhawk_host_address=panel.pixhawk_host_address_edit.text().strip(),
            left_pixhawk_eth_ip=panel.left_pixhawk_eth_ip_edit.text().strip(),
            right_pixhawk_eth_ip=panel.right_pixhawk_eth_ip_edit.text().strip(),
            left_mavlink_url=panel.left_mavlink_url_edit.text().strip(),
            right_mavlink_url=panel.right_mavlink_url_edit.text().strip(),
            left_mavlink_baud=panel.left_mavlink_baud_spin.value(),
            right_mavlink_baud=panel.right_mavlink_baud_spin.value(),
            drone_telemetry_url=str(getattr(panel, "drone_telemetry_url", "") or ""),
            drone_telemetry_baud=int(getattr(panel, "drone_telemetry_baud", 57_600)),
            drone_telemetry_rtcm_forward=bool(getattr(panel, "drone_telemetry_rtcm_forward", True)),
            drone_telemetry_rtcm_source=normalize_drone_rtcm_source(
                getattr(panel, "drone_telemetry_rtcm_source", "base")
            ),
            left_pixhawk_bootstrap_url=panel.left_pixhawk_bootstrap_url_edit.text().strip(),
            right_pixhawk_bootstrap_url=panel.right_pixhawk_bootstrap_url_edit.text().strip(),
            left_pixhawk_bootstrap_baud=panel.left_pixhawk_bootstrap_baud_spin.value(),
            right_pixhawk_bootstrap_baud=panel.right_pixhawk_bootstrap_baud_spin.value(),
            mavlink_configure_streams=panel.mavlink_configure_streams_checkbox.isChecked(),
            mavlink_stream_rate=panel.mavlink_stream_rate_spin.value(),
            pixhawk_ethernet_mode=panel._pixhawk_ethernet_mode(),
            auto_load_intrinsics=bool(
                panel._workstation_setting("auto_load_intrinsics", panel.settings.auto_load_intrinsics)
            ),
            manual_focal_override=panel.manual_focal_checkbox.isChecked(),
            use_triangulation=bool(
                panel._workstation_setting("use_triangulation", panel.settings.use_triangulation)
            ),
            triangulation_extrinsics_source=panel._triangulation_extrinsics_source(),
            measurement_preferred_method=str(
                panel._workstation_setting(
                    "measurement_preferred_method",
                    panel.settings.measurement_preferred_method,
                )
            ),
            measurement_method_fallback_order=tuple(
                panel._workstation_setting(
                    "measurement_method_fallback_order",
                    panel.settings.measurement_method_fallback_order,
                )
            ),
            measurement_display_mode=panel._measurement_display_mode(),
            use_rtk_baseline=bool(
                panel._workstation_setting("use_rtk_baseline", panel.settings.use_rtk_baseline)
            ),
            require_rtk_fixed=bool(
                panel._workstation_setting("require_rtk_fixed", panel.settings.require_rtk_fixed)
            ),
            gnss_base_serial_port=panel.gnss_base_serial_port_edit.text().strip() or "/dev/ttyACM0",
            gnss_base_baud=panel.gnss_base_baud_spin.value(),
            gnss_base_survey_in_min_duration_s=panel.gnss_base_survey_duration_spin.value(),
            gnss_base_survey_in_accuracy_m=panel.gnss_base_survey_accuracy_spin.value(),
            gnss_base_rtcm_injection_target=str(panel.gnss_base_rtcm_target_combo.currentData() or "disabled"),
            gnss_operation=(
                panel._build_gnss_operation_config()
                if hasattr(panel, "gnss_operation_mode_combo")
                else panel.settings.gnss_operation
            ),
        )
        _save_shared_settings(panel.settings, panel.settings_path)
