"""Small, testable experiment-protocol readiness rules for Workstation.

The evaluator only classifies existing evidence.  It does not create
telemetry, change measurement settings, or reject a session command.  Values
remain in their producer contracts: camera sync is RTCP/NTP plus app receive
evidence, Pixhawk attitude is body FRD to world NED, GNSS baseline is NED,
and calibration geometry is OpenCV optical RDF.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite
from typing import Mapping


BACKEND_FACADE_METHODS = (
    "toggle_camera",
    "toggle_detection",
    "toggle_recording",
    "save_session",
    "camera_discover_host_ip",
    "camera_apply_candidate",
    "camera_auto_connect",
    "camera_check_rtsp",
    "camera_check_control",
    "camera_enable_rtsp",
    "camera_swap",
    "camera_save_settings",
    "time_set_pc",
    "time_check_sync",
    "pixhawk_toggle_mavlink",
    "pixhawk_recover_eth_mavlink",
    "pixhawk_discover_usb",
    "pixhawk_discover_mavlink_candidates",
    "pixhawk_discover_host_ip",
    "pixhawk_apply_candidate",
    "gnss_base_discover",
    "gnss_base_apply_candidate",
    "gnss_base_connect",
    "gnss_base_disconnect",
    "gnss_survey_start",
    "gnss_survey_stop",
    "gnss_ntrip_start",
    "gnss_ntrip_stop",
    "calibration_capture_images",
    "calibration_run_intrinsic",
    "calibration_save_intrinsics",
    "calibration_load_intrinsics",
    "calibration_apply_intrinsics",
    "calibration_load_validation",
    "calibration_run_validation",
    "calibration_save_validation",
    "calibration_apply_validation",
    "apply_measurement_settings",
    "save_measurement_settings",
    "workstation_settings_snapshot",
    "apply_workstation_system_settings",
    "save_workstation_system_settings",
    "calibration_apply_setup",
    "calibration_scan_validation",
    "calibration_save_setup",
    "alignment_scan_sources",
    "alignment_start_sampling",
    "alignment_stop_sampling",
    "alignment_save_profile",
    "alignment_load_profile",
    "alignment_swap_pixhawks",
    "advanced_wiris_catalog",
    "advanced_send_wiris_command",
    "advanced_send_zoom",
    "advanced_discover_custom_mavlink_watch",
    "advanced_add_custom_mavlink_watch",
    "advanced_remove_custom_mavlink_watch",
    "advanced_refresh_baseline_fields",
    "advanced_open_px4_shell",
    "advanced_refresh_px4_mag",
    "advanced_refresh_px4_attitude_cal",
    "advanced_apply_px4_mag_priorities",
    "advanced_save_px4_mag_params",
    "advanced_reboot_px4",
    "advanced_refresh_calibration_preview",
    "advanced_calibration_preview_snapshot",
    "diagnostic_text",
    "shutdown_for_close",
)


def missing_backend_facades(backend: object) -> tuple[str, ...]:
    """Return missing public methods for the standalone backend contract."""

    return tuple(name for name in BACKEND_FACADE_METHODS if not callable(getattr(backend, name, None)))


@dataclass(frozen=True)
class ProtocolCheck:
    key: str
    label: str
    passed: bool
    detail: str


@dataclass(frozen=True)
class ProtocolReadiness:
    state: str
    checks: tuple[ProtocolCheck, ...]

    @property
    def blocking(self) -> tuple[ProtocolCheck, ...]:
        return tuple(check for check in self.checks if not check.passed)


def _status_state(snapshot: Mapping[str, object], key: str) -> str:
    status = snapshot.get(key)
    return str(getattr(status, "state", "UNKNOWN") or "UNKNOWN").upper()


def _has_numeric(value: object) -> bool:
    try:
        return isfinite(float(value))
    except (TypeError, ValueError):
        return False


def evaluate_recording_protocol(
    snapshot: Mapping[str, object],
    *,
    distance_mode: str,
    calibration: Mapping[str, object],
    baseline: Mapping[str, object],
    require_rtk_fixed: bool,
    minimum_fix_type: int | None = None,
    physical_alignment: Mapping[str, object] | None = None,
    final_sync_verified: bool | None = None,
) -> ProtocolReadiness:
    """Classify the minimum evidence expected before an experiment run.

    The result is intentionally advisory.  A session may still be started,
    but the operator can see which prerequisite was absent when capture began.
    """

    checks: list[ProtocolCheck] = []
    camera_states = (_status_state(snapshot, "camera.left"), _status_state(snapshot, "camera.right"))
    checks.append(
        ProtocolCheck(
            "cameras",
            "Both camera streams",
            all(state == "LIVE" for state in camera_states),
            f"P1={camera_states[0]}, P2={camera_states[1]} · first frame evidence",
        )
    )

    sync_state = _status_state(snapshot, "synchronization")
    checks.append(
        ProtocolCheck(
            "synchronization",
            "RTCP/NTP pair sync",
            sync_state == "SYNCED",
            f"state={sync_state} · live camera frame timing evidence",
        )
    )

    detection_state = _status_state(snapshot, "detection")
    checks.append(
        ProtocolCheck(
            "detection",
            "Live detection",
            detection_state == "RUNNING",
            f"state={detection_state} · target/detection callback evidence",
        )
    )

    calibration_active = bool(calibration.get("active"))
    left_calibration = calibration.get("left")
    right_calibration = calibration.get("right")
    intrinsics_ready = (
        isinstance(left_calibration, Mapping)
        and isinstance(right_calibration, Mapping)
        and str(left_calibration.get("state", "NO DATA")).upper() == "AVAILABLE"
        and str(right_calibration.get("state", "NO DATA")).upper() == "AVAILABLE"
    )
    checks.append(
        ProtocolCheck(
            "calibration",
            "Active stereo calibration",
            calibration_active and intrinsics_ready,
            f"active={calibration_active}, intrinsics={'ready' if intrinsics_ready else 'missing'} · OpenCV optical RDF",
        )
    )

    mode = str(distance_mode or "pixhawk").lower()
    if mode == "checkerboard":
        extrinsics = calibration.get("extrinsics")
        checkerboard_ready = isinstance(extrinsics, Mapping) and str(extrinsics.get("state", "NO DATA")).upper() == "AVAILABLE"
        checks.append(
            ProtocolCheck(
                "geometry",
                "Checkerboard stereo R/T",
                checkerboard_ready,
                "left-camera optical RDF → right-camera optical RDF",
            )
        )
    else:
        pixhawk_states = (_status_state(snapshot, "pixhawk.left"), _status_state(snapshot, "pixhawk.right"))
        checks.append(
            ProtocolCheck(
                "pixhawk",
                "P1/P2 MAVLink attitude",
                all(state == "LIVE" for state in pixhawk_states),
                f"P1={pixhawk_states[0]}, P2={pixhawk_states[1]} · body FRD → world NED",
            )
        )
        baseline_state = str(baseline.get("state", "NO DATA") or "NO DATA").upper()
        baseline_available = _has_numeric(baseline.get("length_m")) and baseline_state not in {"NO DATA", "UNKNOWN"}
        baseline_good = baseline_state == "GOOD"
        baseline_passed = baseline_good if require_rtk_fixed else baseline_available
        baseline_detail = f"state={baseline_state}, length={baseline.get('length_m', '--')} m"
        body_frd_state = str(baseline.get("p1_body_frd_state", "WAITING") or "WAITING").upper()
        baseline_detail += (
            f" · P1→P2 body FRD projection={body_frd_state}"
            " (raw NED remains the baseline input)"
        )
        if require_rtk_fixed:
            baseline_detail += " · RTK_FIXED required"
        elif baseline_available and not baseline_good:
            baseline_detail += " · degraded fix accepted by current setting"
        checks.append(ProtocolCheck("baseline", "GNSS/RTK NED baseline", baseline_passed, baseline_detail))
        if minimum_fix_type is not None:
            try:
                threshold = int(minimum_fix_type)
                left_fix = int(baseline.get("left_fix_type"))
                right_fix = int(baseline.get("right_fix_type"))
                fix_passed = left_fix >= threshold and right_fix >= threshold
                fix_detail = f"P1={left_fix}, P2={right_fix} · minimum fix_type={threshold}"
            except (TypeError, ValueError):
                fix_passed = False
                fix_detail = f"P1={baseline.get('left_fix_type', '--')}, P2={baseline.get('right_fix_type', '--')} · minimum fix_type={minimum_fix_type}"
            checks.append(ProtocolCheck("fix_type", "Numeric GNSS fix_type", fix_passed, fix_detail))

    if physical_alignment is not None:
        alignment_state = str(physical_alignment.get("state", "WAITING") or "WAITING").upper()
        checks.append(
            ProtocolCheck(
                "physical_alignment",
                "Physical alignment evidence",
                alignment_state == "PASS",
                f"state={alignment_state} · hardware adjustment only; no software R/P/Y correction",
            )
        )
    if final_sync_verified is not None:
        checks.append(
            ProtocolCheck(
                "final_synchronization",
                "Final WIRIS time-sync checkpoint",
                bool(final_sync_verified),
                "explicit post-alignment RTCP/NTP verification",
            )
        )

    return ProtocolReadiness(
        state="READY" if all(check.passed for check in checks) else "BLOCKED",
        checks=tuple(checks),
    )
