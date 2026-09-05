"""GNSS antenna-midpoint local-level FRD reference geometry.

This module intentionally does not produce the public camera/measurement
``baseline_center_frd`` frame.  Its origin is the midpoint of the selected
P1/P2 GNSS antennas.  Right is horizontal P1 -> P2, Down is world-NED Down,
and Forward is ``Right x Down``.  Inputs and outputs are metres.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from math import cos, radians
from typing import Iterable

from Workstation.workstation_core.baseline_alignment import baseline_from_gps
from Workstation.workstation_core.coordinate_contract import (
    GNSS_BASELINE_CENTER_FRD_AXIS_CONTRACT,
    GNSS_BASELINE_CENTER_FRD_FRAME,
    GNSS_BASELINE_CENTER_FRD_VECTOR_CONTRACT,
)
from Workstation.workstation_core.rtk_geometry import EARTH_RADIUS_M, GpsPositionState


GNSS_BASELINE_CENTER_ORIGIN_CONTRACT = "midpoint(P1_GNSS_antenna,P2_GNSS_antenna)"
GNSS_BASELINE_CENTER_RIGHT_CONTRACT = "horizontal normalized P1_GNSS->P2_GNSS"
GNSS_BASELINE_CENTER_DOWN_CONTRACT = "world NED Down"
GNSS_BASELINE_CENTER_FORWARD_CONTRACT = "Right x Down"
GNSS_BASELINE_CENTER_ANGLE_CONTRACT = (
    "acute horizontal angle between P1_GNSS->P2_GNSS baseline and midpoint->Drone_GNSS vector"
)
GNSS_BASELINE_CENTER_TARGET_ANGLE_DEG = 90.0

DEFAULT_MAXIMUM_GNSS_AGE_S = 2.0
DEFAULT_MAXIMUM_GNSS_TIMESTAMP_SPREAD_S = 1.0
MINIMUM_HORIZONTAL_BASELINE_M = 1.0e-6
GNSS_BASELINE_CENTER_RECORD_SCHEMA = "workstation-gnss-baseline-center-record-v1"


Vector3 = tuple[float, float, float]


@dataclass(frozen=True)
class GnssBaselineCenterFrdGeometry:
    """One finite Drone position in the GNSS-derived local-level frame."""

    forward_m: float
    right_m: float
    down_m: float
    range_m: float
    baseline_ned_m: Vector3
    drone_from_p1_ned_m: Vector3
    midpoint_to_drone_ned_m: Vector3
    horizontal_baseline_m: float
    frame: str = GNSS_BASELINE_CENTER_FRD_FRAME
    axis_contract: str = GNSS_BASELINE_CENTER_FRD_AXIS_CONTRACT
    vector_contract: str = GNSS_BASELINE_CENTER_FRD_VECTOR_CONTRACT
    origin_contract: str = GNSS_BASELINE_CENTER_ORIGIN_CONTRACT
    right_axis_contract: str = GNSS_BASELINE_CENTER_RIGHT_CONTRACT
    down_axis_contract: str = GNSS_BASELINE_CENTER_DOWN_CONTRACT
    forward_axis_contract: str = GNSS_BASELINE_CENTER_FORWARD_CONTRACT

    @property
    def vector_m(self) -> Vector3:
        return self.forward_m, self.right_m, self.down_m

    @property
    def baseline_drone_angle_deg(self) -> float | None:
        """Return the acute horizontal angle between baseline and Drone vector.

        ``right_m`` is the signed component along the horizontal P1->P2
        baseline and ``forward_m`` is its horizontal perpendicular component.
        The installation target is a line angle, so both directions of the
        baseline are treated equivalently and NED Down is intentionally not
        included.
        """

        horizontal_drone = math.hypot(self.forward_m, self.right_m)
        if horizontal_drone <= MINIMUM_HORIZONTAL_BASELINE_M:
            return None
        return math.degrees(math.atan2(abs(self.forward_m), abs(self.right_m)))

    @property
    def baseline_drone_angle_error_deg(self) -> float | None:
        """Return the absolute deviation from the perpendicular 90° target."""

        angle = self.baseline_drone_angle_deg
        if angle is None:
            return None
        return abs(GNSS_BASELINE_CENTER_TARGET_ANGLE_DEG - angle)


@dataclass(frozen=True)
class GnssBaselineCenterFrdEstimate:
    """UI-ready geometry plus receive-time/fix evidence.

    ``CURRENT`` and ``STALE`` retain a finite geometry. ``WAITING`` never
    substitutes numerical zero for missing or invalid inputs.
    """

    state: str
    reason: str
    geometry: GnssBaselineCenterFrdGeometry | None = None
    p1_age_s: float | None = None
    p2_age_s: float | None = None
    drone_age_s: float | None = None
    maximum_age_s: float | None = None
    timestamp_spread_s: float | None = None
    p1_fix_type: int | None = None
    p2_fix_type: int | None = None
    drone_fix_type: int | None = None
    p1_fix_label: str = "UNKNOWN"
    p2_fix_label: str = "UNKNOWN"
    drone_fix_label: str = "UNKNOWN"

    @property
    def forward_m(self) -> float | None:
        return None if self.geometry is None else self.geometry.forward_m

    @property
    def right_m(self) -> float | None:
        return None if self.geometry is None else self.geometry.right_m

    @property
    def down_m(self) -> float | None:
        return None if self.geometry is None else self.geometry.down_m

    @property
    def range_m(self) -> float | None:
        return None if self.geometry is None else self.geometry.range_m

    @property
    def baseline_drone_angle_deg(self) -> float | None:
        return None if self.geometry is None else self.geometry.baseline_drone_angle_deg

    @property
    def baseline_drone_angle_error_deg(self) -> float | None:
        return None if self.geometry is None else self.geometry.baseline_drone_angle_error_deg

    def to_mapping(self) -> dict[str, object]:
        geometry = self.geometry
        return {
            "state": self.state,
            "reason": self.reason,
            "forward_m": self.forward_m,
            "right_m": self.right_m,
            "down_m": self.down_m,
            "range_m": self.range_m,
            "baseline_drone_angle_deg": self.baseline_drone_angle_deg,
            "baseline_drone_angle_error_deg": self.baseline_drone_angle_error_deg,
            "baseline_drone_angle_contract": GNSS_BASELINE_CENTER_ANGLE_CONTRACT,
            "baseline_drone_target_angle_deg": GNSS_BASELINE_CENTER_TARGET_ANGLE_DEG,
            "maximum_age_s": self.maximum_age_s,
            "timestamp_spread_s": self.timestamp_spread_s,
            "timestamp_spread_ms": (
                None
                if self.timestamp_spread_s is None
                else self.timestamp_spread_s * 1000.0
            ),
            "ages_s": {
                "p1": self.p1_age_s,
                "p2": self.p2_age_s,
                "drone": self.drone_age_s,
            },
            "fix_types": {
                "p1": self.p1_fix_type,
                "p2": self.p2_fix_type,
                "drone": self.drone_fix_type,
            },
            "fix_labels": {
                "p1": self.p1_fix_label,
                "p2": self.p2_fix_label,
                "drone": self.drone_fix_label,
            },
            "frame": GNSS_BASELINE_CENTER_FRD_FRAME,
            "axis_contract": GNSS_BASELINE_CENTER_FRD_AXIS_CONTRACT,
            "vector_contract": GNSS_BASELINE_CENTER_FRD_VECTOR_CONTRACT,
            "origin_contract": GNSS_BASELINE_CENTER_ORIGIN_CONTRACT,
            "right_axis_contract": GNSS_BASELINE_CENTER_RIGHT_CONTRACT,
            "down_axis_contract": GNSS_BASELINE_CENTER_DOWN_CONTRACT,
            "forward_axis_contract": GNSS_BASELINE_CENTER_FORWARD_CONTRACT,
            "baseline_ned_m": None if geometry is None else geometry.baseline_ned_m,
            "drone_from_p1_ned_m": (
                None if geometry is None else geometry.drone_from_p1_ned_m
            ),
            "midpoint_to_drone_ned_m": (
                None if geometry is None else geometry.midpoint_to_drone_ned_m
            ),
            "horizontal_baseline_m": (
                None if geometry is None else geometry.horizontal_baseline_m
            ),
        }


def _vector3(values: Iterable[float], *, name: str) -> Vector3:
    parsed = tuple(float(value) for value in values)
    if len(parsed) != 3:
        raise ValueError(f"{name} must contain [North, East, Down]")
    if not all(math.isfinite(value) for value in parsed):
        raise ValueError(f"{name} must contain finite metres")
    return parsed[0], parsed[1], parsed[2]


def gnss_baseline_center_frd_from_ned(
    baseline_ned_m: Iterable[float],
    drone_from_p1_ned_m: Iterable[float],
) -> GnssBaselineCenterFrdGeometry:
    """Transform local-NED inputs into GNSS antenna-midpoint local-level FRD.

    ``baseline_ned_m`` is P2 minus P1. ``drone_from_p1_ned_m`` is Drone
    minus P1.  Both are ordered ``[North, East, Down]`` in metres.
    """

    baseline = _vector3(baseline_ned_m, name="baseline_ned_m")
    drone_from_p1 = _vector3(
        drone_from_p1_ned_m,
        name="drone_from_p1_ned_m",
    )
    horizontal = math.hypot(baseline[0], baseline[1])
    if horizontal <= MINIMUM_HORIZONTAL_BASELINE_M:
        raise ValueError("horizontal P1->P2 GNSS baseline is undefined")

    right_ned = (baseline[0] / horizontal, baseline[1] / horizontal, 0.0)
    forward_ned = (right_ned[1], -right_ned[0], 0.0)
    midpoint_to_drone = (
        drone_from_p1[0] - baseline[0] * 0.5,
        drone_from_p1[1] - baseline[1] * 0.5,
        drone_from_p1[2] - baseline[2] * 0.5,
    )
    forward = sum(midpoint_to_drone[index] * forward_ned[index] for index in range(3))
    right = sum(midpoint_to_drone[index] * right_ned[index] for index in range(3))
    down = midpoint_to_drone[2]
    distance = math.sqrt(forward * forward + right * right + down * down)
    return GnssBaselineCenterFrdGeometry(
        forward_m=forward,
        right_m=right,
        down_m=down,
        range_m=distance,
        baseline_ned_m=baseline,
        drone_from_p1_ned_m=drone_from_p1,
        midpoint_to_drone_ned_m=midpoint_to_drone,
        horizontal_baseline_m=horizontal,
    )


def _valid_gps_position(gps: GpsPositionState) -> bool:
    if not gps.has_position:
        return False
    try:
        latitude = float(gps.latitude_deg)
        longitude = float(gps.longitude_deg)
        altitude = float(gps.altitude_m)
    except (TypeError, ValueError):
        return False
    return (
        math.isfinite(latitude)
        and -90.0 <= latitude <= 90.0
        and math.isfinite(longitude)
        and -180.0 <= longitude <= 180.0
        and math.isfinite(altitude)
    )


def gnss_baseline_center_frd_from_gps(
    p1: GpsPositionState,
    p2: GpsPositionState,
    drone: GpsPositionState,
) -> GnssBaselineCenterFrdGeometry | None:
    """Build geometry from current WGS84/MSL GNSS states.

    The local tangent approximation intentionally matches the existing
    Telemetry midpoint-to-Drone range calculation for compatibility.
    """

    if not all(_valid_gps_position(state) for state in (p1, p2, drone)):
        return None
    baseline = baseline_from_gps(p1, p2)
    if baseline is None:
        return None
    reference_lat_rad = radians((float(p1.latitude_deg) + float(p2.latitude_deg)) / 2.0)
    drone_north_m = radians(float(drone.latitude_deg) - float(p1.latitude_deg)) * EARTH_RADIUS_M
    drone_east_m = (
        radians(float(drone.longitude_deg) - float(p1.longitude_deg))
        * EARTH_RADIUS_M
        * cos(reference_lat_rad)
    )
    drone_down_m = -(float(drone.altitude_m) - float(p1.altitude_m))
    try:
        return gnss_baseline_center_frd_from_ned(
            (baseline.north_m, baseline.east_m, -baseline.up_m),
            (drone_north_m, drone_east_m, drone_down_m),
        )
    except ValueError:
        return None


def _validated_limit(value: float | None, *, name: str) -> float | None:
    if value is None:
        return None
    parsed = float(value)
    if not math.isfinite(parsed) or parsed < 0.0:
        raise ValueError(f"{name} must be finite and non-negative")
    return parsed


def estimate_gnss_baseline_center_frd(
    p1: GpsPositionState | None,
    p2: GpsPositionState | None,
    drone: GpsPositionState | None,
    *,
    now_s: float | None = None,
    maximum_age_s: float | None = DEFAULT_MAXIMUM_GNSS_AGE_S,
    maximum_timestamp_spread_s: float | None = (
        DEFAULT_MAXIMUM_GNSS_TIMESTAMP_SPREAD_S
    ),
) -> GnssBaselineCenterFrdEstimate:
    """Return WAITING/CURRENT/STALE GNSS-derived geometry and provenance."""

    age_limit = _validated_limit(maximum_age_s, name="maximum_age_s")
    spread_limit = _validated_limit(
        maximum_timestamp_spread_s,
        name="maximum_timestamp_spread_s",
    )
    states = (p1, p2, drone)
    fix_values = tuple(
        state.fix_type if isinstance(state, GpsPositionState) else None
        for state in states
    )
    fix_labels = tuple(
        state.rtk_label if isinstance(state, GpsPositionState) else "UNKNOWN"
        for state in states
    )
    if not all(isinstance(state, GpsPositionState) for state in states):
        missing = [
            label
            for label, state in (("P1", p1), ("P2", p2), ("Drone", drone))
            if state is None
        ]
        invalid = [
            label
            for label, state in (("P1", p1), ("P2", p2), ("Drone", drone))
            if state is not None and not isinstance(state, GpsPositionState)
        ]
        details: list[str] = []
        if missing:
            details.append("missing " + ", ".join(missing))
        if invalid:
            details.append("invalid " + ", ".join(invalid))
        return GnssBaselineCenterFrdEstimate(
            state="WAITING",
            reason="GNSS position waiting: " + "; ".join(details),
            p1_fix_type=fix_values[0],
            p2_fix_type=fix_values[1],
            drone_fix_type=fix_values[2],
            p1_fix_label=fix_labels[0],
            p2_fix_label=fix_labels[1],
            drone_fix_label=fix_labels[2],
        )

    assert isinstance(p1, GpsPositionState)
    assert isinstance(p2, GpsPositionState)
    assert isinstance(drone, GpsPositionState)

    geometry = gnss_baseline_center_frd_from_gps(p1, p2, drone)
    if geometry is None:
        return GnssBaselineCenterFrdEstimate(
            state="WAITING",
            reason="Three finite GNSS positions and a non-zero horizontal P1->P2 baseline are required",
            p1_fix_type=fix_values[0],
            p2_fix_type=fix_values[1],
            drone_fix_type=fix_values[2],
            p1_fix_label=fix_labels[0],
            p2_fix_label=fix_labels[1],
            drone_fix_label=fix_labels[2],
        )

    timestamps: tuple[float, float, float]
    try:
        timestamps = (
            float(p1.timestamp_s),
            float(p2.timestamp_s),
            float(drone.timestamp_s),
        )
    except (TypeError, ValueError):
        timestamps = (float("nan"), float("nan"), float("nan"))
    if not all(math.isfinite(value) for value in timestamps):
        return GnssBaselineCenterFrdEstimate(
            state="STALE",
            reason="GNSS receive timestamp is missing or non-finite",
            geometry=geometry,
            p1_fix_type=fix_values[0],
            p2_fix_type=fix_values[1],
            drone_fix_type=fix_values[2],
            p1_fix_label=fix_labels[0],
            p2_fix_label=fix_labels[1],
            drone_fix_label=fix_labels[2],
        )

    spread_s = max(timestamps) - min(timestamps)
    ages: tuple[float, float, float] | None = None
    reasons: list[str] = []
    if now_s is not None:
        observed_s = float(now_s)
        if not math.isfinite(observed_s):
            raise ValueError("now_s must be finite")
        ages = tuple(observed_s - timestamp for timestamp in timestamps)
        for label, age in zip(("P1", "P2", "Drone"), ages):
            if age < 0.0:
                reasons.append(f"{label} GNSS timestamp is {abs(age):.3f}s in the future")
            elif age_limit is not None and age > age_limit:
                reasons.append(f"{label} GNSS age {age:.3f}s exceeds {age_limit:.3f}s")
    elif age_limit is not None:
        raise ValueError("now_s is required when maximum_age_s is enabled")
    if spread_limit is not None and spread_s > spread_limit:
        reasons.append(
            f"three-GNSS timestamp spread {spread_s:.3f}s exceeds {spread_limit:.3f}s"
        )

    angle_undefined = geometry.baseline_drone_angle_deg is None
    if angle_undefined:
        reasons.insert(0, "midpoint-to-Drone horizontal GNSS vector is undefined")

    return GnssBaselineCenterFrdEstimate(
        state="WAITING" if angle_undefined else "STALE" if reasons else "CURRENT",
        reason="; ".join(reasons) if reasons else "Three GNSS positions are current",
        geometry=geometry,
        p1_age_s=None if ages is None else ages[0],
        p2_age_s=None if ages is None else ages[1],
        drone_age_s=None if ages is None else ages[2],
        maximum_age_s=None if ages is None else max(ages),
        timestamp_spread_s=spread_s,
        p1_fix_type=fix_values[0],
        p2_fix_type=fix_values[1],
        drone_fix_type=fix_values[2],
        p1_fix_label=fix_labels[0],
        p2_fix_label=fix_labels[1],
        drone_fix_label=fix_labels[2],
    )


def _source_record(
    prefix: str,
    state: GpsPositionState | None,
    *,
    selection: str,
    age_s: float | None,
) -> dict[str, object]:
    if state is None:
        return {
            f"{prefix}_source_selection": str(selection),
            f"{prefix}_position_available": False,
            f"{prefix}_namespace": None,
            f"{prefix}_topic": None,
            f"{prefix}_latitude_deg": None,
            f"{prefix}_longitude_deg": None,
            f"{prefix}_altitude_msl_m": None,
            f"{prefix}_fix_type": None,
            f"{prefix}_fix_label": "UNKNOWN",
            f"{prefix}_satellites_used": None,
            f"{prefix}_eph_m": None,
            f"{prefix}_epv_m": None,
            f"{prefix}_age_s": age_s,
            f"{prefix}_app_receive_monotonic_ns": None,
            f"{prefix}_gps_utc_us": None,
            f"{prefix}_time_source": None,
            f"{prefix}_mavlink_source_system": None,
            f"{prefix}_mavlink_source_component": None,
            f"{prefix}_mavlink_sequence": None,
            f"{prefix}_mavlink_message_id": None,
        }
    try:
        timestamp_s = float(state.timestamp_s)
    except (TypeError, ValueError):
        timestamp_s = float("nan")
    timestamp_ns = (
        int(round(timestamp_s * 1_000_000_000.0))
        if math.isfinite(timestamp_s)
        else None
    )

    def finite_or_none(value: object) -> float | None:
        if value is None:
            return None
        try:
            parsed = float(value)
        except (TypeError, ValueError):
            return None
        return parsed if math.isfinite(parsed) else None

    return {
        f"{prefix}_source_selection": str(selection),
        f"{prefix}_position_available": bool(state.has_position),
        f"{prefix}_namespace": str(state.namespace),
        f"{prefix}_topic": str(state.topic),
        f"{prefix}_latitude_deg": finite_or_none(state.latitude_deg),
        f"{prefix}_longitude_deg": finite_or_none(state.longitude_deg),
        f"{prefix}_altitude_msl_m": finite_or_none(state.altitude_m),
        f"{prefix}_fix_type": state.fix_type,
        f"{prefix}_fix_label": state.rtk_label,
        f"{prefix}_satellites_used": state.satellites_used,
        f"{prefix}_eph_m": finite_or_none(state.eph_m),
        f"{prefix}_epv_m": finite_or_none(state.epv_m),
        f"{prefix}_age_s": finite_or_none(age_s),
        f"{prefix}_app_receive_monotonic_ns": timestamp_ns,
        f"{prefix}_gps_utc_us": state.gps_utc_us,
        f"{prefix}_time_source": str(state.time_source),
        f"{prefix}_mavlink_source_system": state.mavlink_source_system,
        f"{prefix}_mavlink_source_component": state.mavlink_source_component,
        f"{prefix}_mavlink_sequence": state.mavlink_sequence,
        f"{prefix}_mavlink_message_id": state.mavlink_message_id,
    }


def build_gnss_baseline_center_record(
    estimate: GnssBaselineCenterFrdEstimate,
    *,
    p1: GpsPositionState | None,
    p2: GpsPositionState | None,
    drone: GpsPositionState,
    p1_selection: str,
    p2_selection: str,
    sequence: int,
) -> dict[str, object]:
    """Build one self-contained, JSON-safe recording payload.

    Recording/session clocks and generation identity are added by the
    Recording owner. Missing source/geometry values remain ``None``.
    """

    geometry = estimate.geometry
    row: dict[str, object] = {
        "gnss_baseline_center_record_schema": GNSS_BASELINE_CENTER_RECORD_SCHEMA,
        "event_type": "gnss_baseline_center_derived",
        "trigger": "primary_drone_gps_received",
        "sequence": max(0, int(sequence)),
        "geometry_state": str(estimate.state),
        "geometry_reason": str(estimate.reason),
        "coordinate_frame": GNSS_BASELINE_CENTER_FRD_FRAME,
        "coordinate_axis_contract": GNSS_BASELINE_CENTER_FRD_AXIS_CONTRACT,
        "coordinate_vector_contract": GNSS_BASELINE_CENTER_FRD_VECTOR_CONTRACT,
        "coordinate_origin_contract": GNSS_BASELINE_CENTER_ORIGIN_CONTRACT,
        "right_axis_contract": GNSS_BASELINE_CENTER_RIGHT_CONTRACT,
        "down_axis_contract": GNSS_BASELINE_CENTER_DOWN_CONTRACT,
        "forward_axis_contract": GNSS_BASELINE_CENTER_FORWARD_CONTRACT,
        "gnss_baseline_center_forward_m": estimate.forward_m,
        "gnss_baseline_center_right_m": estimate.right_m,
        "gnss_baseline_center_down_m": estimate.down_m,
        "gnss_baseline_center_range_m": estimate.range_m,
        "maximum_source_age_s": estimate.maximum_age_s,
        "three_source_timestamp_spread_s": estimate.timestamp_spread_s,
        "three_source_timestamp_spread_ms": (
            None
            if estimate.timestamp_spread_s is None
            else estimate.timestamp_spread_s * 1000.0
        ),
        "horizontal_baseline_m": (
            None if geometry is None else geometry.horizontal_baseline_m
        ),
        "baseline_north_m": None if geometry is None else geometry.baseline_ned_m[0],
        "baseline_east_m": None if geometry is None else geometry.baseline_ned_m[1],
        "baseline_down_m": None if geometry is None else geometry.baseline_ned_m[2],
        "drone_from_p1_north_m": (
            None if geometry is None else geometry.drone_from_p1_ned_m[0]
        ),
        "drone_from_p1_east_m": (
            None if geometry is None else geometry.drone_from_p1_ned_m[1]
        ),
        "drone_from_p1_down_m": (
            None if geometry is None else geometry.drone_from_p1_ned_m[2]
        ),
        "midpoint_to_drone_north_m": (
            None if geometry is None else geometry.midpoint_to_drone_ned_m[0]
        ),
        "midpoint_to_drone_east_m": (
            None if geometry is None else geometry.midpoint_to_drone_ned_m[1]
        ),
        "midpoint_to_drone_down_m": (
            None if geometry is None else geometry.midpoint_to_drone_ned_m[2]
        ),
    }
    row.update(
        _source_record(
            "p1",
            p1,
            selection=p1_selection,
            age_s=estimate.p1_age_s,
        )
    )
    row.update(
        _source_record(
            "p2",
            p2,
            selection=p2_selection,
            age_s=estimate.p2_age_s,
        )
    )
    row.update(
        _source_record(
            "drone",
            drone,
            selection="GPS1_PRIMARY",
            age_s=estimate.drone_age_s,
        )
    )
    return row


__all__ = [
    "DEFAULT_MAXIMUM_GNSS_AGE_S",
    "DEFAULT_MAXIMUM_GNSS_TIMESTAMP_SPREAD_S",
    "GNSS_BASELINE_CENTER_DOWN_CONTRACT",
    "GNSS_BASELINE_CENTER_FORWARD_CONTRACT",
    "GNSS_BASELINE_CENTER_RECORD_SCHEMA",
    "GNSS_BASELINE_CENTER_FRD_AXIS_CONTRACT",
    "GNSS_BASELINE_CENTER_FRD_FRAME",
    "GNSS_BASELINE_CENTER_FRD_VECTOR_CONTRACT",
    "GNSS_BASELINE_CENTER_ORIGIN_CONTRACT",
    "GNSS_BASELINE_CENTER_RIGHT_CONTRACT",
    "GnssBaselineCenterFrdEstimate",
    "GnssBaselineCenterFrdGeometry",
    "build_gnss_baseline_center_record",
    "estimate_gnss_baseline_center_frd",
    "gnss_baseline_center_frd_from_gps",
    "gnss_baseline_center_frd_from_ned",
]
