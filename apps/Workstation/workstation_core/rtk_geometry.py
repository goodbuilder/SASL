from __future__ import annotations

from dataclasses import dataclass
from math import atan2, cos, isfinite, radians, sin, sqrt
from typing import Any, Iterable

from Workstation.workstation_core.attitude_geometry import world_ned_vector_to_body_frd
from Workstation.workstation_core.coordinate_contract import BODY_FRAME, WORLD_FRAME

EARTH_RADIUS_M = 6_378_137.0


@dataclass(frozen=True)
class GpsPositionState:
    camera: str
    namespace: str
    timestamp_s: float
    topic: str
    latitude_deg: float | None = None
    longitude_deg: float | None = None
    altitude_m: float | None = None
    fix_type: int | None = None
    satellites_used: int | None = None
    eph_m: float | None = None
    epv_m: float | None = None
    heading_deg: float | None = None
    heading_accuracy_deg: float | None = None
    px4_timestamp_s: float | None = None
    raw: Any | None = None
    px4_timestamp_us: int | None = None
    px4_timestamp_sample_us: int | None = None
    gps_utc_us: int | None = None
    gps_timestamp_time_relative_us: int | None = None
    altitude_ellipsoid_m: float | None = None
    velocity_n_m_s: float | None = None
    velocity_e_m_s: float | None = None
    velocity_d_m_s: float | None = None
    velocity_ned_valid: bool | None = None
    hdop: float | None = None
    vdop: float | None = None
    device_id: int | None = None
    time_source: str = "mavlink"
    mavlink_source_system: int | None = None
    mavlink_source_component: int | None = None
    mavlink_sequence: int | None = None
    mavlink_message_id: int | None = None

    @property
    def has_position(self) -> bool:
        return self.latitude_deg is not None and self.longitude_deg is not None and self.altitude_m is not None

    @property
    def rtk_label(self) -> str:
        # PX4/ublox fix_type commonly uses 5=float RTK, 6=fixed RTK. Keep generic fallback.
        if self.fix_type == 6:
            return "RTK_FIXED"
        if self.fix_type == 5:
            return "RTK_FLOAT"
        if self.fix_type is None:
            return "UNKNOWN"
        if self.fix_type >= 3:
            return "3D_FIX"
        return "NO_FIX"


def gps_topic(namespace: str) -> str:
    normalized = namespace.strip().strip("/")
    return f"/{normalized}/fmu/out/vehicle_gps_position" if normalized else "/fmu/out/vehicle_gps_position"


def _scaled_degrees(value: Any) -> float | None:
    if value is None:
        return None
    number = float(value)
    # PX4 vehicle_gps_position lat/lon are usually int32 degrees * 1e7.
    if abs(number) > 1000.0:
        return number * 1e-7
    return number


def _scaled_altitude_m(value: Any) -> float | None:
    if value is None:
        return None
    number = float(value)
    # PX4 altitude is often millimetres. Values above 20 km are almost certainly mm.
    if abs(number) > 20_000.0:
        return number * 1e-3
    return number


def _scaled_accuracy_m(value: Any) -> float | None:
    if value is None:
        return None
    number = float(value)
    # PX4 eph/epv are commonly metres in vehicle_gps_position. Accept mm-like values defensively.
    if abs(number) > 1000.0:
        return number * 1e-3
    return number


def _heading_degrees(value: Any) -> float | None:
    if value is None:
        return None
    number = float(value)
    # Some PX4 fields are radians, some scaled integers depending on msg/version.
    if abs(number) <= 2.0 * 3.141592653589793:
        return number * 180.0 / 3.141592653589793
    if abs(number) > 1000.0:
        return number * 1e-5
    return number


def gps_state_from_message(*, camera: str, namespace: str, message: Any, timestamp_s: float) -> GpsPositionState:
    px4_timestamp = getattr(message, "timestamp", None)
    px4_timestamp_us = int(px4_timestamp) if px4_timestamp is not None else None
    timestamp_sample = getattr(message, "timestamp_sample", None)
    gps_utc = getattr(message, "time_utc_usec", None)
    gps_utc_us = int(gps_utc) if gps_utc not in (None, 0) else None
    timestamp_time_relative = getattr(message, "timestamp_time_relative", None)
    return GpsPositionState(
        camera=camera,
        namespace=namespace.strip().strip("/"),
        timestamp_s=float(timestamp_s),
        topic=gps_topic(namespace),
        latitude_deg=_scaled_degrees(getattr(message, "lat", getattr(message, "latitude_deg", None))),
        longitude_deg=_scaled_degrees(getattr(message, "lon", getattr(message, "longitude_deg", None))),
        altitude_m=_scaled_altitude_m(getattr(message, "alt", getattr(message, "altitude_msl_m", None))),
        fix_type=int(getattr(message, "fix_type")) if hasattr(message, "fix_type") else None,
        satellites_used=int(getattr(message, "satellites_used")) if hasattr(message, "satellites_used") else None,
        eph_m=_scaled_accuracy_m(getattr(message, "eph", None)),
        epv_m=_scaled_accuracy_m(getattr(message, "epv", None)),
        heading_deg=_heading_degrees(getattr(message, "heading", getattr(message, "heading_motion", None))),
        heading_accuracy_deg=_heading_degrees(getattr(message, "heading_accuracy", None)),
        px4_timestamp_s=float(px4_timestamp_us) * 1e-6 if px4_timestamp_us is not None else None,
        raw=message,
        px4_timestamp_us=px4_timestamp_us,
        px4_timestamp_sample_us=int(timestamp_sample) if timestamp_sample is not None else None,
        gps_utc_us=gps_utc_us,
        gps_timestamp_time_relative_us=(
            int(timestamp_time_relative) if timestamp_time_relative is not None else None
        ),
        altitude_ellipsoid_m=_scaled_altitude_m(
            getattr(message, "altitude_ellipsoid_m", getattr(message, "alt_ellipsoid", None))
        ),
        velocity_n_m_s=getattr(message, "vel_n_m_s", None),
        velocity_e_m_s=getattr(message, "vel_e_m_s", None),
        velocity_d_m_s=getattr(message, "vel_d_m_s", None),
        velocity_ned_valid=(
            bool(getattr(message, "vel_ned_valid")) if hasattr(message, "vel_ned_valid") else None
        ),
        hdop=getattr(message, "hdop", None),
        vdop=getattr(message, "vdop", None),
        device_id=(int(getattr(message, "device_id")) if hasattr(message, "device_id") else None),
        time_source="mavlink",
    )


@dataclass(frozen=True)
class RtkBaselineEstimate:
    baseline_m: float
    north_m: float
    east_m: float
    down_m: float
    left_age_s: float
    right_age_s: float
    fix_quality: str
    warnings: tuple[str, ...] = ()
    gps_pair_delta_s: float | None = None


@dataclass(frozen=True)
class GpsPairReadiness:
    """Fail-closed readiness for one raw P1/P2 GPS pair.

    The samples remain WGS84 geodetic evidence and their derived baseline
    remains world NED.  This decision only controls whether the pair may feed
    measurement geometry; it never discards or rewrites the raw samples.
    Timestamps are application receive-monotonic seconds.
    """

    usable: bool
    reasons: tuple[str, ...]
    left_age_s: float | None
    right_age_s: float | None
    pair_delta_s: float | None


def validate_gps_pair_for_baseline(
    left: GpsPositionState | None,
    right: GpsPositionState | None,
    *,
    now_s: float,
    maximum_age_s: float,
    maximum_pair_delta_s: float,
    minimum_fix_type: int = 3,
) -> GpsPairReadiness:
    """Validate freshness/skew before a P1→P2 NED pair feeds geometry."""

    now = float(now_s)
    maximum_age = float(maximum_age_s)
    maximum_pair_delta = float(maximum_pair_delta_s)
    minimum_fix = int(minimum_fix_type)
    if not isfinite(now):
        raise ValueError("now_s must be finite")
    if not isfinite(maximum_age) or maximum_age < 0.0:
        raise ValueError("maximum_age_s must be finite and non-negative")
    if not isfinite(maximum_pair_delta) or maximum_pair_delta < 0.0:
        raise ValueError("maximum_pair_delta_s must be finite and non-negative")
    if minimum_fix < 0:
        raise ValueError("minimum_fix_type must be non-negative")

    reasons: list[str] = []
    ages: dict[str, float | None] = {"left": None, "right": None}
    timestamps: dict[str, float] = {}
    for side, state in (("left", left), ("right", right)):
        if state is None:
            reasons.append(f"{side} GPS sample missing")
            continue
        position = (state.latitude_deg, state.longitude_deg, state.altitude_m)
        try:
            position_valid = state.has_position and all(
                isfinite(float(value)) for value in position
            )
        except (TypeError, ValueError):
            position_valid = False
        if not position_valid:
            reasons.append(f"{side} GPS position missing or invalid")
        if state.fix_type is None or int(state.fix_type) < minimum_fix:
            reasons.append(f"{side} GPS fix < {minimum_fix}")
        try:
            timestamp = float(state.timestamp_s)
        except (TypeError, ValueError):
            timestamp = float("nan")
        if not isfinite(timestamp):
            reasons.append(f"{side} GPS timestamp invalid")
            continue
        timestamps[side] = timestamp
        age = now - timestamp
        ages[side] = age
        if age < 0.0:
            reasons.append(f"{side} GPS sample timestamp is in the future")
        elif age > maximum_age:
            reasons.append(
                f"{side} GPS sample stale ({age:.3f}s > {maximum_age:.3f}s)"
            )

    pair_delta = None
    if set(timestamps) == {"left", "right"}:
        pair_delta = abs(timestamps["left"] - timestamps["right"])
        if pair_delta > maximum_pair_delta:
            reasons.append(
                "GPS pair timestamp skew "
                f"({pair_delta:.3f}s > {maximum_pair_delta:.3f}s)"
            )

    stable_reasons = tuple(dict.fromkeys(reasons))
    return GpsPairReadiness(
        usable=not stable_reasons,
        reasons=stable_reasons,
        left_age_s=ages["left"],
        right_age_s=ages["right"],
        pair_delta_s=pair_delta,
    )


@dataclass(frozen=True)
class BodyFrdBaselineProjection:
    """P1→P2 baseline re-expressed in the P1 Pixhawk body-FRD frame.

    ``RtkBaselineEstimate`` remains the authoritative raw GPS/RTK result in
    local world NED.  This object is a separate, attitude-derived display and
    recording projection; it must not be used as camera optical RDF or public
    ``baseline_center_frd`` data.
    """

    forward_m: float
    right_m: float
    down_m: float
    attitude_timestamp_s: float | None = None
    attitude_age_s: float | None = None
    source_frame: str = WORLD_FRAME
    target_frame: str = BODY_FRAME
    direction: str = "P1->P2"
    axis_contract: str = "P1->P2 in P1 body FRD: [Forward, Right, Down]; metres"

    @property
    def vector_m(self) -> tuple[float, float, float]:
        return self.forward_m, self.right_m, self.down_m

    @property
    def length_m(self) -> float:
        return sqrt(sum(value * value for value in self.vector_m))


def project_ned_baseline_to_p1_body_frd(
    ned_m: tuple[float, float, float],
    p1_body_to_world_q: Iterable[float],
    *,
    attitude_timestamp_s: float | None = None,
    attitude_age_s: float | None = None,
) -> BodyFrdBaselineProjection:
    """Convert a raw P1→P2 local-NED vector to P1 body FRD.

    Input order is explicitly ``[North, East, Down]`` in metres.  The PX4
    attitude quaternion is ``body FRD → world NED``; the inverse rotation
    therefore produces ``[Forward, Right, Down]`` in P1 body coordinates.
    """

    values = tuple(float(value) for value in ned_m)
    if len(values) != 3:
        raise ValueError("ned_m must contain exactly [north, east, down]")
    if not all(isfinite(value) for value in values):
        raise ValueError("ned_m must contain finite metres")
    forward, right, down = world_ned_vector_to_body_frd(values, p1_body_to_world_q)
    return BodyFrdBaselineProjection(
        forward_m=forward,
        right_m=right,
        down_m=down,
        attitude_timestamp_s=None if attitude_timestamp_s is None else float(attitude_timestamp_s),
        attitude_age_s=None if attitude_age_s is None else max(0.0, float(attitude_age_s)),
    )


def ned_delta_m(left: GpsPositionState, right: GpsPositionState) -> tuple[float, float, float] | None:
    """Return the right/P2 position minus left/P1 position in local NED metres."""

    if not left.has_position or not right.has_position:
        return None
    lat0 = radians((left.latitude_deg + right.latitude_deg) / 2.0)  # type: ignore[operator]
    d_lat = radians(right.latitude_deg - left.latitude_deg)  # type: ignore[operator]
    d_lon = radians(right.longitude_deg - left.longitude_deg)  # type: ignore[operator]
    north = d_lat * EARTH_RADIUS_M
    east = d_lon * EARTH_RADIUS_M * cos(lat0)
    down = -(right.altitude_m - left.altitude_m)  # type: ignore[operator]
    return north, east, down


def estimate_rtk_baseline(
    left: GpsPositionState | None,
    right: GpsPositionState | None,
    *,
    now_s: float,
    require_fixed: bool,
) -> RtkBaselineEstimate | None:
    if left is None or right is None:
        return None
    delta = ned_delta_m(left, right)
    if delta is None:
        return None
    north, east, down = delta
    baseline = sqrt(north * north + east * east + down * down)
    left_age = max(0.0, now_s - left.timestamp_s)
    right_age = max(0.0, now_s - right.timestamp_s)
    warnings: list[str] = []
    if left.fix_type != 6:
        warnings.append(f"left {left.rtk_label}")
    if right.fix_type != 6:
        warnings.append(f"right {right.rtk_label}")
    if require_fixed and any("RTK_FIXED" not in state.rtk_label for state in (left, right)):
        quality = "BAD"
    elif warnings:
        quality = "DEGRADED"
    else:
        quality = "GOOD"
    return RtkBaselineEstimate(
        baseline_m=baseline,
        north_m=north,
        east_m=east,
        down_m=down,
        left_age_s=left_age,
        right_age_s=right_age,
        fix_quality=quality,
        warnings=tuple(warnings),
        gps_pair_delta_s=abs(float(left.timestamp_s) - float(right.timestamp_s)),
    )
