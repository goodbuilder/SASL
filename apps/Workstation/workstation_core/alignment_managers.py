from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
from math import radians, sin, sqrt, tan
from statistics import mean
from typing import Any

import yaml

from Workstation.workstation_core.attitude_geometry import relative_attitude
from Workstation.workstation_core.baseline_alignment import (
    BaselineQuality,
    BaselineVector,
    baseline_from_gps,
    evaluate_baseline_quality,
)
from Workstation.workstation_core.pixhawk_attitude import (
    CameraExtrinsicState,
    MagSensorState,
    magnetometer_value_to_gauss,
    magnetometer_value_units,
)
from Workstation.workstation_core.rtk_geometry import GpsPositionState


@dataclass(frozen=True)
class CoordinateSystemSettings:
    global_frame: str = "ENU"
    camera_frame: str = "RDF"
    origin: str = "Left Camera"
    frd_to_camera_rdf: tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]] = (
        (0.0, 1.0, 0.0),
        (0.0, 0.0, 1.0),
        (1.0, 0.0, 0.0),
    )


@dataclass(frozen=True)
class ValidationResult:
    baseline_length_difference_m: float | None
    baseline_vector_difference_m: float | None
    yaw_difference_deg: float | None
    pitch_difference_deg: float | None
    roll_difference_deg: float | None
    baseline_error_contribution_m: float | None
    yaw_error_contribution_m: float | None
    pitch_error_contribution_m: float | None
    status: str


class GNSSMonitor:
    def __init__(self) -> None:
        self.left_source = "GPS1"
        self.right_source = "GPS1"
        self.gps_positions: dict[str, GpsPositionState] = {}
        self.gps_sensors: dict[str, dict[str, Any]] = {"left": {}, "right": {}}
        self.baseline_samples: list[tuple[BaselineVector, GpsPositionState, GpsPositionState]] = []

    def update_position(self, side: str, state: GpsPositionState) -> None:
        self.gps_positions[side] = state

    def update_sensor(self, side: str, slot: str, state: Any) -> None:
        self.gps_sensors.setdefault(side, {})[slot] = state

    def selected_position(self, side: str) -> GpsPositionState | None:
        source = self.left_source if side == "left" else self.right_source
        if source == "GPS1":
            return self.gps_positions.get(side)
        candidate = self.gps_positions.get(f"{side}_GPS2")
        return candidate if isinstance(candidate, GpsPositionState) else None

    def current_baseline(self, *, max_timestamp_delta_ms: float = 200.0) -> tuple[BaselineVector, GpsPositionState, GpsPositionState] | None:
        left = self.selected_position("left")
        right = self.selected_position("right")
        if left is None or right is None:
            return None
        baseline = baseline_from_gps(left, right)
        if baseline is None or baseline.timestamp_delta_ms > max_timestamp_delta_ms:
            return None
        return baseline, left, right

    def append_current_sample(self) -> tuple[BaselineVector, GpsPositionState, GpsPositionState] | None:
        sample = self.current_baseline()
        if sample is not None:
            self.baseline_samples.append(sample)
        return sample

    def quality(self, *, duration_s: float) -> BaselineQuality | None:
        return evaluate_baseline_quality(self.baseline_samples, duration_s=duration_s)


class BaselineManager:
    def __init__(self, gnss: GNSSMonitor) -> None:
        self.gnss = gnss

    def current(self) -> tuple[BaselineVector, GpsPositionState, GpsPositionState] | None:
        return self.gnss.current_baseline()


class MagnetometerMonitor:
    def __init__(self) -> None:
        self.sensors: dict[str, dict[str, MagSensorState]] = {"left": {}, "right": {}}

    def update(self, side: str, state: MagSensorState) -> None:
        self.sensors.setdefault(side, {})[state.sensor_slot] = state

    def field_strength(self, side: str, slot: str) -> float | None:
        state = self.sensors.get(side, {}).get(slot)
        return None if state is None else state.magnitude

    def can_difference(self, side: str) -> float | None:
        can1_state = self.sensors.get(side, {}).get("CAN1")
        can2_state = self.sensors.get(side, {}).get("CAN2")
        if can1_state is None or can2_state is None:
            return None
        can1 = magnetometer_value_to_gauss(can1_state.magnitude, can1_state.topic)
        can2 = magnetometer_value_to_gauss(can2_state.magnitude, can2_state.topic)
        if can1 is None or can2 is None:
            return None
        return abs(can1 - can2)

    def can_difference_display(self, side: str) -> tuple[float | None, str]:
        """Return CAN1/CAN2 difference in a truthful display unit."""
        can1_state = self.sensors.get(side, {}).get("CAN1")
        can2_state = self.sensors.get(side, {}).get("CAN2")
        if can1_state is None or can2_state is None:
            return None, ""
        can1 = can1_state.magnitude
        can2 = can2_state.magnitude
        if can1 is None or can2 is None:
            return None, ""
        can1_units = magnetometer_value_units(can1_state.topic)
        can2_units = magnetometer_value_units(can2_state.topic)
        if can1_units and can1_units == can2_units:
            return abs(float(can1) - float(can2)), can1_units
        difference = self.can_difference(side)
        return difference, "G"


class AttitudeMonitor:
    def __init__(self) -> None:
        self.states: dict[str, CameraExtrinsicState] = {}

    def update(self, side: str, state: CameraExtrinsicState) -> None:
        self.states[side] = state

    def deltas(self) -> tuple[float, float, float] | None:
        left = self.states.get("left")
        right = self.states.get("right")
        if left is None or right is None:
            return None
        try:
            relative = relative_attitude(left.q, right.q)
        except (TypeError, ValueError, OverflowError):
            return None
        return relative.roll_deg, relative.pitch_deg, relative.yaw_deg


class CoordinateSystemManager:
    def __init__(self) -> None:
        self.settings = CoordinateSystemSettings()

    def update(
        self,
        *,
        global_frame: str,
        camera_frame: str,
        origin: str,
        frd_to_camera_rdf: tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]] | None = None,
    ) -> None:
        self.settings = CoordinateSystemSettings(
            global_frame=global_frame,
            camera_frame=camera_frame,
            origin=origin,
            frd_to_camera_rdf=frd_to_camera_rdf or self.settings.frd_to_camera_rdf,
        )

    def tf_tree_text(self) -> str:
        return (
            f"global({self.settings.global_frame}) -> origin({self.settings.origin}) -> "
            f"Pixhawk FRD -> camera({self.settings.camera_frame})"
        )


class AlignmentValidator:
    def validate(
        self,
        *,
        stored_baseline_enu_m: tuple[float, float, float] | None,
        current_baseline: BaselineVector | None,
        stored_rpy_deg: tuple[float, float, float] | None,
        current_rpy_delta_deg: tuple[float, float, float] | None,
        distance_m: float,
    ) -> ValidationResult:
        baseline_length_difference = None
        baseline_vector_difference = None
        if stored_baseline_enu_m is not None and current_baseline is not None:
            baseline_vector_difference = sqrt(
                (current_baseline.east_m - stored_baseline_enu_m[0]) ** 2
                + (current_baseline.north_m - stored_baseline_enu_m[1]) ** 2
                + (current_baseline.up_m - stored_baseline_enu_m[2]) ** 2
            )
            stored_length = sqrt(sum(component * component for component in stored_baseline_enu_m))
            baseline_length_difference = abs(current_baseline.length_m - stored_length)

        roll_diff = pitch_diff = yaw_diff = None
        if stored_rpy_deg is not None and current_rpy_delta_deg is not None:
            roll_diff = abs(current_rpy_delta_deg[0] - stored_rpy_deg[0])
            pitch_diff = abs(current_rpy_delta_deg[1] - stored_rpy_deg[1])
            yaw_diff = abs(current_rpy_delta_deg[2] - stored_rpy_deg[2])

        baseline_error = baseline_vector_difference
        yaw_error = None if yaw_diff is None else abs(distance_m * tan(radians(yaw_diff)))
        pitch_error = None if pitch_diff is None else abs(distance_m * tan(radians(pitch_diff)))
        fail = (
            (baseline_vector_difference is not None and baseline_vector_difference >= 0.05)
            or (yaw_diff is not None and yaw_diff >= 1.0)
            or (pitch_diff is not None and pitch_diff >= 1.0)
        )
        return ValidationResult(
            baseline_length_difference_m=baseline_length_difference,
            baseline_vector_difference_m=baseline_vector_difference,
            yaw_difference_deg=yaw_diff,
            pitch_difference_deg=pitch_diff,
            roll_difference_deg=roll_diff,
            baseline_error_contribution_m=baseline_error,
            yaw_error_contribution_m=yaw_error,
            pitch_error_contribution_m=pitch_error,
            status="FAIL" if fail else "PASS",
        )


class AlignmentProfileManager:
    def build_payload(
        self,
        *,
        baseline: BaselineVector | None,
        baseline_quality: BaselineQuality | None,
        left_attitude: CameraExtrinsicState | None,
        right_attitude: CameraExtrinsicState | None,
        coordinate_settings: CoordinateSystemSettings,
        gps_mapping: dict[str, str],
        magnetometer_mapping: dict[str, str],
        physical_alignment: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        baseline_payload: dict[str, Any] = {}
        if baseline is not None:
            baseline_payload = {
                "frame": "ENU",
                "vector_m": [baseline.east_m, baseline.north_m, baseline.up_m],
                "length_m": baseline.length_m,
                "azimuth_deg": baseline.azimuth_deg,
                "elevation_deg": baseline.elevation_deg,
            }
        quality_payload = None
        if baseline_quality is not None:
            quality_payload = {
                "grade": baseline_quality.grade,
                "rtk_fixed_ratio": baseline_quality.rtk_fixed_ratio,
                "baseline_std_m": baseline_quality.baseline_length_std_m,
                "timestamp_delta_ms": baseline_quality.timestamp_delta_ms,
            }
        payload = {
            "alignment_profile": {
                "created_at": datetime.now(timezone.utc).isoformat(),
                "baseline": baseline_payload,
                "baseline_quality": quality_payload,
                "left_attitude": _attitude_payload(left_attitude),
                "right_attitude": _attitude_payload(right_attitude),
                "coordinate_system": {
                    "global_frame": coordinate_settings.global_frame,
                    "camera_frame": coordinate_settings.camera_frame,
                    "origin": coordinate_settings.origin,
                    "frd_to_camera_rdf": coordinate_settings.frd_to_camera_rdf,
                },
                "gps_source_mapping": gps_mapping,
                "magnetometer_source_mapping": magnetometer_mapping,
            }
        }
        if physical_alignment is not None:
            # This is evidence for the Workstation physical-adjustment loop;
            # it is deliberately kept separate from the legacy ENU profile
            # and does not become a correction transform.
            payload["alignment_profile"]["physical_alignment"] = dict(physical_alignment)
        return payload

    def dump(self, payload: dict[str, Any]) -> str:
        return yaml.safe_dump(payload, sort_keys=False, allow_unicode=True)

    def load(self, text: str) -> dict[str, Any]:
        payload = yaml.safe_load(text) or {}
        if not isinstance(payload, dict) or "alignment_profile" not in payload:
            raise ValueError("alignment_profile YAML 형식이 아닙니다.")
        return payload


def _attitude_payload(state: CameraExtrinsicState | None) -> dict[str, Any]:
    if state is None:
        return {}
    return {
        "roll_deg": state.roll_deg,
        "pitch_deg": state.pitch_deg,
        "yaw_deg": state.yaw_deg,
        "quaternion_wxyz": list(state.q),
        "timestamp_s": state.timestamp_s,
        "topic": state.topic,
    }
