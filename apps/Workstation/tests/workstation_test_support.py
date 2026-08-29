"""Shared imports and fakes for the split Workstation integration tests."""

from __future__ import annotations

import json

import os

import subprocess

import sys

import threading

import time

from contextlib import contextmanager

from pathlib import Path

from types import SimpleNamespace

import pytest

from PyQt5 import QtCore, QtWidgets

from wiris_stereo_distance.gui import _parse_ui_mode

from Workstation.command_registry import CommandRegistry, CommandSpec

from Workstation.evidence import EvidenceAdapter

from sasl_common.lifecycle import LifecycleCoordinator, LifecycleStep

from Workstation.main_window import WorkstationMainWindow

from Workstation.status_model import BackendStatusAdapter, StatusSeverity

from Workstation.standalone_backend import (
    StandalonePanelBackend,
    create_standalone_backend,
)


class _FakeBackend:
    def __init__(self) -> None:
        self.calls: list[object] = []
        checker_components = {
            f"{side}.{dof}": {
                "provided": True,
                "editable": False,
                "locked": True,
                "source": "checkerboard",
                "compatible_fill_source": (
                    "live_pixhawk" if dof.endswith("_deg") else "live_gnss"
                ),
                "fill_enabled": False,
                "source_ready": True,
                "reason": "provided by method",
            }
            for side in ("left", "right")
            for dof in ("roll_deg", "pitch_deg", "yaw_deg", "north_m", "east_m", "down_m")
        }
        horizon_components = {
            f"{side}.{dof}": {
                "provided": dof in {"roll_deg", "pitch_deg"},
                "editable": dof not in {"roll_deg", "pitch_deg"},
                "locked": dof in {"roll_deg", "pitch_deg"},
                "source": (
                    "candidate:horizon-test"
                    if dof in {"roll_deg", "pitch_deg"}
                    else "live_pixhawk"
                    if dof == "yaw_deg"
                    else "live_gnss"
                ),
                "compatible_fill_source": (
                    "live_pixhawk" if dof.endswith("_deg") else "live_gnss"
                ),
                "fill_enabled": dof not in {"roll_deg", "pitch_deg"},
                "source_ready": True,
                "reason": (
                    "provided by method"
                    if dof in {"roll_deg", "pitch_deg"}
                    else "fill source selected"
                ),
            }
            for side in ("left", "right")
            for dof in ("roll_deg", "pitch_deg", "yaw_deg", "north_m", "east_m", "down_m")
        }
        self._measurement_method_snapshot = {
            "selected_method_id": "checkerboard_stereo",
            "selected_instance_id": "profile:test-checkerboard",
            "selection_reason": "preferred",
            "ready_methods": [
                {
                    "method_id": "checkerboard_stereo",
                    "label": "Checkerboard stereo R,t",
                    "instance_id": "profile:test-checkerboard",
                    "ready": True,
                    "fallback_rank": 2,
                    "component_sources": {"left.roll_deg": "checkerboard"},
                }
            ],
            "fill_policy_locked": False,
            "method_capabilities": [
                {
                    "method_id": "checkerboard_stereo",
                    "label": "Checkerboard stereo R,t",
                    "instance_id": "profile:test-checkerboard",
                    "ready": True,
                    "components": checker_components,
                },
                {
                    "method_id": "horizon_hybrid",
                    "label": "Horizon R/P + live Pixhawk/GNSS",
                    "instance_id": "candidate:horizon-test",
                    "ready": True,
                    "components": horizon_components,
                },
            ],
        }
        left_bootstrap_baud = QtWidgets.QSpinBox()
        left_bootstrap_baud.setRange(1, 10_000_000)
        left_bootstrap_baud.setValue(115_200)
        right_bootstrap_baud = QtWidgets.QSpinBox()
        right_bootstrap_baud.setRange(1, 10_000_000)
        right_bootstrap_baud.setValue(115_200)
        self.panel = SimpleNamespace(
            camera_connected=False,
            yolo_enabled=False,
            camera_frame_times={"left": [], "right": []},
            camera_stream_stats={"left": {}, "right": {}},
            pixhawk_attitude_times={"left": [], "right": []},
            last_distance_quality="BAD",
            last_distance_status="거리: 좌/우 검출 매칭 대기 중",
            recording_session_id="",
            _measurement_session_hierarchy_enabled=False,
            _measurement_run_state="IDLE",
            _manual_recording_active=False,
            distance_history=[],
            left_url_edit=QtWidgets.QLineEdit("rtsp://left"),
            right_url_edit=QtWidgets.QLineEdit("rtsp://right"),
            _session_save_thread=None,
            _session_save_retry_required=False,
            _session_save_finalized=False,
            recording_active=False,
            camera_frame_metadata={"left": {}, "right": {}},
            camera_sync_status="not-verified",
            camera_sync_reason="STUS 동기화 후 RTCP/NTP 검증이 필요합니다.",
            selected_runtime_frame_pair_delta_s=None,
            selected_runtime_frame_pair_basis="",
            selected_runtime_frame_pair_camera_rtc_delta_s=None,
            latest_gps={},
            latest_rtk_baseline=None,
            custom_watch_specs=[],
            _camera_candidate_rows=[],
            camera_network_discovery_thread=None,
            camera_auto_detect_thread=None,
            workstation_camera_discovery_status="NOT RUN",
            _pixhawk_candidate_rows_by_source={"pc": [], "mavlink": []},
            pixhawk_usb_discovery_thread=None,
            pixhawk_host_ip_discovery_thread=None,
            workstation_pixhawk_discovery_status="NOT RUN",
            left_pixhawk_bootstrap_url_edit=QtWidgets.QLineEdit("/dev/ttyACM0"),
            right_pixhawk_bootstrap_url_edit=QtWidgets.QLineEdit("/dev/ttyACM1"),
            left_pixhawk_bootstrap_baud_spin=left_bootstrap_baud,
            right_pixhawk_bootstrap_baud_spin=right_bootstrap_baud,
            pixhawk_usb_bootstrap_thread=None,
            pixhawk_usb_bootstrap_status={
                "left": {"state": "IDLE", "detail": "대기", "updated_s": 10.0},
                "right": {"state": "IDLE", "detail": "대기", "updated_s": 10.0},
            },
        )
        self.panel._session_recording_active = lambda: bool(self.panel.recording_active)
        self.panel.data_storage_snapshot = self.data_storage_snapshot

    def toggle_camera(self) -> None:
        self.calls.append("camera")

    def toggle_detection(self) -> None:
        self.calls.append("detection")

    def toggle_recording(self) -> None:
        self.calls.append("recording")

    def save_session(self) -> None:
        self.calls.append("save")

    def measurement_run_snapshot(self) -> dict[str, object]:
        return {
            "state": str(self.panel._measurement_run_state),
            "detail": "ready",
            "storage": self.data_storage_snapshot(),
        }

    def data_storage_snapshot(self) -> dict[str, object]:
        return {
            "state": "OK",
            "root": "/tmp/measurement_data",
            "priority_pending": 0,
            "bulk_pending": 0,
            "dropped_records": 0,
            "streams": {},
        }

    def retry_data_storage(self) -> dict[str, object]:
        self.calls.append("retry_data_storage")
        return self.data_storage_snapshot()

    def optical_pose_reset(self) -> None:
        self.calls.append("optical_pose_reset")

    def capture_pnp_hr_photos(self) -> bool:
        self.calls.append("hr_capture")
        return True

    def apply_camera_settings(self, left: str, right: str) -> bool:
        self.calls.append(("settings", left, right))
        return True

    def camera_discover_host_ip(self) -> None:
        self.calls.append("camera_discover_host_ip")

    def camera_apply_candidate(self, row: int, target: str) -> None:
        self.calls.append(("camera_apply_candidate", row, target))

    def camera_auto_connect(self) -> None:
        self.calls.append("camera_auto_connect")

    def camera_check_rtsp(self) -> None:
        self.calls.append("camera_check_rtsp")

    def camera_check_control(self) -> None:
        self.calls.append("camera_check_control")

    def camera_enable_rtsp(self) -> None:
        self.calls.append("camera_enable_rtsp")

    def camera_reactivate(self, camera: str) -> None:
        self.calls.append(("camera_reactivate", camera))

    def camera_swap(self) -> None:
        self.calls.append("camera_swap")

    def camera_save_settings(self) -> None:
        self.calls.append("camera_save_settings")

    def time_set_pc(self) -> None:
        self.calls.append("time_set_pc")

    def time_check_sync(self) -> None:
        self.calls.append("time_check_sync")

    def pixhawk_toggle_mavlink(self) -> None:
        self.calls.append("pixhawk_toggle_mavlink")

    def pixhawk_reset_level_calibration(self) -> None:
        self.calls.append("pixhawk_reset_level_calibration")

    def pixhawk_reset_calibration(self, camera: str, group: str) -> None:
        self.calls.append(("pixhawk_reset_calibration", camera, group))

    def pixhawk_recover_eth_mavlink(self, camera: str) -> None:
        self.calls.append(("pixhawk_recover_eth_mavlink", camera))

    def pixhawk_discover_usb(self) -> None:
        self.calls.append("pixhawk_discover_usb")

    def pixhawk_discover_mavlink_candidates(self) -> None:
        self.calls.append("pixhawk_discover_mavlink_candidates")

    def pixhawk_discover_host_ip(self) -> None:
        self.calls.append("pixhawk_discover_host_ip")

    def pixhawk_apply_candidate(self, source: str, row: int, target: str) -> None:
        self.calls.append(("pixhawk_apply_candidate", source, row, target))

    def pixhawk_assign_network_candidate(self, candidate_id: str, side: str) -> None:
        self.calls.append(("pixhawk_assign_network_candidate", candidate_id, side))

    def gnss_base_discover(self) -> None:
        self.calls.append("gnss_base_discover")

    def gnss_base_apply_candidate(self, row: int) -> None:
        self.calls.append(("gnss_base_apply_candidate", row))

    def gnss_base_connect(self) -> None:
        self.calls.append("gnss_base_connect")

    def gnss_base_disconnect(self) -> None:
        self.calls.append("gnss_base_disconnect")

    def gnss_survey_start(self) -> None:
        self.calls.append("gnss_survey_start")

    def gnss_survey_stop(self) -> None:
        self.calls.append("gnss_survey_stop")

    def gnss_ntrip_start(self) -> None:
        self.calls.append("gnss_ntrip_start")

    def gnss_ntrip_stop(self) -> None:
        self.calls.append("gnss_ntrip_stop")

    def calibration_capture_images(self) -> None:
        self.calls.append("calibration_capture_images")

    def calibration_toggle_auto_capture(self) -> None:
        self.calls.append("calibration_toggle_auto_capture")

    def calibration_run_intrinsic(self, input_request=None) -> None:
        self.last_calibration_input_request = dict(input_request or {})
        self.calls.append("calibration_run_intrinsic")

    def calibration_run_extrinsics(self, input_request=None) -> None:
        self.last_calibration_input_request = dict(input_request or {})
        self.calls.append("calibration_run_extrinsics")

    def calibration_save_intrinsics(self) -> None:
        self.calls.append("calibration_save_intrinsics")

    def calibration_save_extrinsics(self) -> None:
        self.calls.append("calibration_save_extrinsics")

    def calibration_load_intrinsics(self) -> None:
        self.calls.append("calibration_load_intrinsics")

    def calibration_load_extrinsics(self) -> None:
        self.calls.append("calibration_load_extrinsics")

    def calibration_load_current_zoom_intrinsics(self) -> None:
        self.calls.append("calibration_load_current_zoom_intrinsics")

    def calibration_apply_intrinsics(self) -> None:
        self.calls.append("calibration_apply_intrinsics")

    def calibration_apply_bundle(self) -> None:
        self.calls.append("calibration_apply_bundle")

    def calibration_load_validation(self) -> None:
        self.calls.append("calibration_load_validation")

    def calibration_run_validation(self) -> None:
        self.calls.append("calibration_run_validation")

    def calibration_save_validation(self) -> None:
        self.calls.append("calibration_save_validation")

    def calibration_apply_validation(self) -> None:
        self.calls.append("calibration_apply_validation")

    def apply_measurement_settings(self, settings: dict[str, object]) -> None:
        self.calls.append(("measurement_apply_settings", settings))

    def measurement_methods_snapshot(self) -> dict[str, object]:
        return dict(self._measurement_method_snapshot)

    def select_measurement_method(self, payload: dict[str, object]) -> dict[str, object]:
        self.calls.append(("measurement_method_select", dict(payload)))
        self._measurement_method_snapshot["selected_method_id"] = str(payload["method_id"])
        self._measurement_method_snapshot["selection_reason"] = "manual"
        return dict(self._measurement_method_snapshot)

    def configure_measurement_method_fill(
        self,
        payload: dict[str, object],
    ) -> dict[str, object]:
        values = dict(payload)
        self.calls.append(("measurement_method_fill", values))
        method_id = str(values["method_id"])
        component = str(values["component"])
        enabled = bool(values["enabled"])
        row = next(
            item
            for item in self._measurement_method_snapshot["method_capabilities"]
            if item["method_id"] == method_id
        )
        state = row["components"][component]
        state["fill_enabled"] = enabled
        state["source"] = state["compatible_fill_source"] if enabled else "none"
        return dict(self._measurement_method_snapshot)

    def save_measurement_settings(self) -> None:
        self.calls.append("measurement_save_settings")

    def workstation_settings_snapshot(self) -> dict[str, object]:
        from Workstation.workstation_core.settings import AppSettings

        defaults = AppSettings()
        snapshot = dict(defaults.__dict__)
        snapshot["gnss_operation"] = defaults.gnss_operation.to_mapping()
        return snapshot

    def apply_workstation_system_settings(self, settings: dict[str, object]) -> None:
        self.calls.append(("settings_apply", dict(settings)))

    def set_gnss_ntrip_session_password(
        self,
        password: str,
        *,
        caster_host: str | None = None,
        mountpoint: str | None = None,
        username: str | None = None,
    ) -> None:
        self.calls.append(
            (
                "set_gnss_ntrip_session_password",
                password,
                caster_host,
                mountpoint,
                username,
            )
        )

    def save_workstation_system_settings(self) -> None:
        self.calls.append("settings_save")

    def calibration_apply_setup(self, setup: dict[str, object]) -> None:
        self.calls.append(("calibration_apply_setup", setup))

    def calibration_scan_validation(self) -> None:
        self.calls.append("calibration_scan_validation")

    def calibration_save_setup(self) -> None:
        self.calls.append("calibration_save_setup")

    def alignment_scan_sources(self) -> None:
        self.calls.append("alignment_scan_sources")

    def alignment_start_sampling(self) -> None:
        self.calls.append("alignment_start_sampling")

    def alignment_stop_sampling(self) -> None:
        self.calls.append("alignment_stop_sampling")

    def alignment_save_profile(self) -> None:
        self.calls.append("alignment_save_profile")

    def alignment_load_profile(self) -> None:
        self.calls.append("alignment_load_profile")

    def alignment_swap_pixhawks(self) -> None:
        self.calls.append("alignment_swap_pixhawks")

    def query_lrf_once(self) -> None:
        self.calls.append("query_lrf_once")

    def advanced_wiris_catalog(self) -> tuple[dict[str, str], ...]:
        return (
            {
                "category": "Basic",
                "label": "Check connection",
                "command": "HIWS",
                "terminator": "\n",
                "description": "Connection check",
            },
        )

    def advanced_send_wiris_command(self, target: str, command: str, terminator: str) -> None:
        self.calls.append(("advanced_wiris", target, command, terminator))

    def advanced_send_zoom(self, target: str, action: str, level: int | None = None) -> None:
        self.calls.append(("advanced_zoom", target, action, level))

    def advanced_discover_custom_mavlink_watch(self) -> None:
        self.calls.append("advanced_watch_discover_mavlink")

    def advanced_add_custom_mavlink_watch(self, name: str, message: str, field: str) -> str:
        self.calls.append(("advanced_watch_add_mavlink", name, message, field))
        return name or f"mavlink:{message}.{field}"

    def advanced_remove_custom_mavlink_watch(self, key: str) -> None:
        self.calls.append(("advanced_watch_remove", key))

    def advanced_refresh_baseline_fields(self) -> dict[str, object]:
        self.calls.append("advanced_baseline_scan")
        return {
            "mavlink_messages": (("GPS_RAW_INT", "lat, lon, alt"),),
            "axis_contract": "producer fields preserved",
        }

    def advanced_open_px4_shell(self, camera: str) -> None:
        self.calls.append(("advanced_px4_shell", camera))

    def advanced_refresh_px4_mag(self, camera: str, *, read_params: bool = True) -> None:
        self.calls.append(("advanced_px4_mag", camera, read_params))

    def advanced_refresh_px4_attitude_cal(self, camera: str, *, read_params: bool = True) -> None:
        self.calls.append(("advanced_px4_attitude", camera, read_params))

    def advanced_apply_px4_mag_priorities(self, camera: str, priorities: dict[str, int]) -> None:
        self.calls.append(("advanced_px4_apply_mag", camera, priorities))

    def advanced_save_px4_mag_params(self, camera: str) -> None:
        self.calls.append(("advanced_px4_save_mag", camera))

    def advanced_reboot_px4(self, camera: str) -> None:
        self.calls.append(("advanced_px4_reboot", camera))

    def advanced_refresh_calibration_preview(self) -> dict[str, object]:
        self.calls.append("advanced_preview_refresh")
        return {"images": {}, "axis_contract": "checkerboard pixels preserved"}

    def advanced_calibration_preview_snapshot(self) -> dict[str, object]:
        return {"images": {}, "axis_contract": "checkerboard pixels preserved"}

    def diagnostic_text(self) -> str:
        return "[fake] source-backed log"

    def ftp_diagnostic_text(self) -> str:
        return "[fake] FTP source-backed log"

    def shutdown_for_close(self) -> bool:
        self.calls.append("shutdown")
        return True
