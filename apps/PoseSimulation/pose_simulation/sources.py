"""Input sample types and adapters used by PoseSimulation.

This module is deliberately independent from ``stereo_ground_sim``.  It
accepts the self-describing camera diagnostic JSON currently emitted by the
Simulation, but only consumes its observed optical pose fields.  The desired
pose fields are never used as an actual measurement.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import Any, Mapping

from .coordinate_adapter import Wgs84ReferenceOrigin, wgs84_to_gazebo_world_enu
from .pose_math import (
    Matrix3,
    Pose3D,
    Vector3,
    as_vector3,
    GAZEBO_LINK_FLU_FROM_OPTICAL_RDF,
    matrix_multiply,
    quaternion_xyzw_to_matrix,
    validate_rotation_matrix,
)


WORLD_FRAME = "gazebo_world_enu"
CAMERA_OPTICAL_FRAME = "camera_optical_rdf"

RTK_STATUS_VALID_FIXED = "VALID_RTK_FIXED"
RTK_STATUS_VALID_NON_FIXED = "VALID_NON_FIXED"
RTK_STATUS_INVALID_FIX = "INVALID_FIX"
RTK_STATUS_ACCURACY_REJECTED = "ACCURACY_REJECTED"
RTK_STATUS_STALE = "STALE"
RTK_STATUS_NONFINITE = "NONFINITE"
RTK_STATUS_JUMP_REJECTED = "JUMP_REJECTED"
RTK_STATUS_COORDINATE_TRANSFORM_FAILED = "COORDINATE_TRANSFORM_FAILED"
RTK_STATUS_TIME_ALIGNMENT_FAILED = "TIME_ALIGNMENT_FAILED"
RTK_STATUS_TARGET_POINT_UNRESOLVED = "TARGET_POINT_UNRESOLVED"


class SampleParseError(ValueError):
    """Raised when an input sample does not satisfy its source contract."""


def _finite(value: Any, name: str) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        raise SampleParseError(f"{name} is not numeric") from None
    if not math.isfinite(parsed):
        raise SampleParseError(f"{name} is non-finite")
    return parsed


def _int_value(value: Any, name: str) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        raise SampleParseError(f"{name} is not an integer") from None


def stamp_ns_from_ros_header(header: Any) -> int:
    if header is None or not hasattr(header, "stamp"):
        raise SampleParseError("ROS header stamp is missing")
    stamp = header.stamp
    try:
        seconds = int(stamp.sec)
        nanoseconds = int(stamp.nanosec)
    except (AttributeError, TypeError, ValueError):
        raise SampleParseError("ROS header stamp is invalid") from None
    if nanoseconds < 0 or nanoseconds >= 1_000_000_000:
        raise SampleParseError("ROS header nanoseconds are out of range")
    return seconds * 1_000_000_000 + nanoseconds


@dataclass(frozen=True)
class PoseSourceSample:
    source: str
    frame_id: str
    pose: Pose3D
    source_timestamp_ns: int
    receive_timestamp_ns: int
    entity_name: str = ""
    validity: str = "VALID"

    @property
    def position_world_enu_m(self) -> Vector3:
        return self.pose.position_world_enu_m

    @property
    def rotation_world_from_frame(self) -> Matrix3:
        return self.pose.rotation_world_from_frame


def pose_source_from_pose_stamped(
    message: Any,
    *,
    source: str,
    receive_timestamp_ns: int,
    frame_id: str = WORLD_FRAME,
    entity_name: str = "",
) -> PoseSourceSample:
    """Convert a ``geometry_msgs/PoseStamped`` into a validated world pose."""

    source_timestamp_ns = stamp_ns_from_ros_header(message.header)
    try:
        message_frame_id = str(getattr(message.header, "frame_id", "") or "")
        if message_frame_id and message_frame_id != frame_id:
            raise SampleParseError(
                f"{source} frame mismatch: expected {frame_id}, got {message_frame_id}"
            )
        position = as_vector3(
            (message.pose.position.x, message.pose.position.y, message.pose.position.z),
            f"{source}.position_world_enu_m",
        )
        quaternion = (
            _finite(message.pose.orientation.x, f"{source}.quaternion.x"),
            _finite(message.pose.orientation.y, f"{source}.quaternion.y"),
            _finite(message.pose.orientation.z, f"{source}.quaternion.z"),
            _finite(message.pose.orientation.w, f"{source}.quaternion.w"),
        )
        rotation = quaternion_xyzw_to_matrix(quaternion)
        validate_rotation_matrix(rotation)
    except (AttributeError, TypeError, ValueError) as exc:
        raise SampleParseError(f"{source} pose is invalid: {exc}") from exc
    pose = Pose3D(position, rotation, frame_id, source_timestamp_ns)
    return PoseSourceSample(
        source=source,
        frame_id=frame_id,
        pose=pose,
        source_timestamp_ns=source_timestamp_ns,
        receive_timestamp_ns=int(receive_timestamp_ns),
        entity_name=entity_name,
    )


def pose_source_from_camera_diagnostic_json(
    payload: Mapping[str, Any] | str,
    *,
    source: str,
    receive_timestamp_ns: int,
    camera_link_rotation_from_optical: Matrix3 = GAZEBO_LINK_FLU_FROM_OPTICAL_RDF,
    expected_entity_name: str | None = None,
) -> PoseSourceSample:
    """Read observed RDF optical world pose from camera diagnostic JSON.

    The parser intentionally rejects payloads that contain only a desired
    pose.  A desired pose is not an observation and cannot be used as the
    actual Gazebo camera input.
    """

    if isinstance(payload, str):
        try:
            parsed: Any = json.loads(payload)
        except json.JSONDecodeError as exc:
            raise SampleParseError(f"{source} JSON is invalid: {exc}") from exc
    else:
        parsed = dict(payload)
    if not isinstance(parsed, dict):
        raise SampleParseError(f"{source} payload must be an object")
    payload_frame = parsed.get("frame_id", WORLD_FRAME)
    if payload_frame and str(payload_frame) != WORLD_FRAME:
        raise SampleParseError(
            f"{source} frame mismatch: expected {WORLD_FRAME}, got {payload_frame}"
        )
    try:
        position_values = parsed["observed_optical_position_world_enu_m"]
        quaternion_values = parsed["observed_optical_orientation_world_from_entity_xyzw"]
        source_timestamp_ns = _int_value(parsed["gazebo_stamp_ns"], f"{source}.gazebo_stamp_ns")
        entity_name = str(parsed.get("observed_optical_entity_name", ""))
        if expected_entity_name and entity_name != expected_entity_name:
            raise SampleParseError(
                f"{source} entity mismatch: expected {expected_entity_name}, got {entity_name}"
            )
        if len(position_values) != 3 or len(quaternion_values) != 4:
            raise SampleParseError(f"{source} observed optical pose has wrong dimensions")
        position = as_vector3(position_values, f"{source}.position_world_enu_m")
        quaternion = tuple(
            _finite(value, f"{source}.quaternion") for value in quaternion_values
        )
        # The observed entity is Gazebo's ``optical_link``.  In the current
        # SDF that link uses FLU axes, while the application contract is RDF;
        # apply the verified fixed link-to-optical extrinsic explicitly.
        rotation = matrix_multiply(
            quaternion_xyzw_to_matrix(quaternion),
            camera_link_rotation_from_optical,
        )
        validate_rotation_matrix(rotation)
    except KeyError as exc:
        raise SampleParseError(
            f"{source} observed optical field is missing: {exc.args[0]}"
        ) from None
    except (TypeError, ValueError) as exc:
        if isinstance(exc, SampleParseError):
            raise
        raise SampleParseError(f"{source} observed optical pose is invalid: {exc}") from exc
    if source_timestamp_ns < 0:
        raise SampleParseError(f"{source}.gazebo_stamp_ns must be non-negative")
    pose = Pose3D(position, rotation, CAMERA_OPTICAL_FRAME, source_timestamp_ns)
    return PoseSourceSample(
        source=source,
        frame_id=CAMERA_OPTICAL_FRAME,
        pose=pose,
        source_timestamp_ns=source_timestamp_ns,
        receive_timestamp_ns=int(receive_timestamp_ns),
        entity_name=entity_name,
    )


def _first_attribute(message: Any, names: tuple[str, ...], field_name: str) -> Any:
    for name in names:
        if hasattr(message, name):
            return getattr(message, name)
    raise SampleParseError(f"RTK message field {field_name} is unavailable")


@dataclass(frozen=True)
class RtkSample:
    source: str
    source_message_type: str
    source_timestamp_us: int
    receive_timestamp_ns: int
    latitude_deg: float
    longitude_deg: float
    altitude_msl_m: float
    altitude_ellipsoid_m: float | None
    eph_m: float
    epv_m: float
    fix_type: int
    satellites_used: int | None

    @property
    def source_timestamp_ns(self) -> int:
        return self.source_timestamp_us * 1_000

    def altitude_for_datum(self, altitude_field: str) -> tuple[float, str]:
        field = str(altitude_field).strip()
        if field == "altitude_ellipsoid_m":
            if self.altitude_ellipsoid_m is None:
                raise SampleParseError("RTK ellipsoid altitude is unavailable")
            return self.altitude_ellipsoid_m, "ellipsoid"
        if field in {"altitude_msl_m", "altitude_m"}:
            return self.altitude_msl_m, "msl"
        raise SampleParseError(f"unsupported RTK altitude field: {field}")

    def status(
        self,
        *,
        minimum_fix_type: int = 5,
        require_rtk_fixed: bool = False,
        max_horizontal_accuracy_m: float = math.inf,
        max_vertical_accuracy_m: float = math.inf,
    ) -> str:
        values = (
            self.latitude_deg,
            self.longitude_deg,
            self.altitude_msl_m,
            self.eph_m,
            self.epv_m,
        )
        if not all(math.isfinite(value) for value in values):
            return RTK_STATUS_NONFINITE
        if self.fix_type < int(minimum_fix_type):
            return RTK_STATUS_INVALID_FIX
        if require_rtk_fixed and self.fix_type < 6:
            return RTK_STATUS_VALID_NON_FIXED
        if math.isfinite(max_horizontal_accuracy_m) and self.eph_m > max_horizontal_accuracy_m:
            return RTK_STATUS_ACCURACY_REJECTED
        if math.isfinite(max_vertical_accuracy_m) and self.epv_m > max_vertical_accuracy_m:
            return RTK_STATUS_ACCURACY_REJECTED
        return RTK_STATUS_VALID_FIXED if self.fix_type >= 6 else RTK_STATUS_VALID_NON_FIXED


def rtk_sample_from_message(
    message: Any,
    *,
    source: str,
    receive_timestamp_ns: int,
) -> RtkSample:
    """Adapt current PX4 ``SensorGps`` or a future ``VehicleGpsPosition``."""

    timestamp_sample = _first_attribute(
        message, ("timestamp_sample", "timestamp"), "timestamp_sample"
    )
    latitude = _first_attribute(message, ("latitude_deg", "latitude", "lat"), "latitude")
    longitude = _first_attribute(message, ("longitude_deg", "longitude", "lon"), "longitude")
    altitude_msl = _first_attribute(
        message,
        ("altitude_msl_m", "altitude_msl", "alt_msl_m", "alt"),
        "altitude_msl",
    )
    altitude_ellipsoid = None
    for name in ("altitude_ellipsoid_m", "altitude_ellipsoid", "alt_ellipsoid"):
        if hasattr(message, name):
            try:
                candidate = float(getattr(message, name))
            except (TypeError, ValueError):
                candidate = math.nan
            altitude_ellipsoid = candidate
            break
    eph = getattr(message, "eph", math.nan)
    epv = getattr(message, "epv", math.nan)
    fix_type = _first_attribute(message, ("fix_type", "fix_status"), "fix_type")
    satellites = getattr(message, "satellites_used", None)
    try:
        return RtkSample(
            source=source,
            source_message_type=type(message).__module__ + "." + type(message).__name__,
            source_timestamp_us=_int_value(timestamp_sample, "timestamp_sample"),
            receive_timestamp_ns=int(receive_timestamp_ns),
            latitude_deg=_finite(latitude, "latitude_deg"),
            longitude_deg=_finite(longitude, "longitude_deg"),
            altitude_msl_m=_finite(altitude_msl, "altitude_msl_m"),
            altitude_ellipsoid_m=(
                None if altitude_ellipsoid is None else float(altitude_ellipsoid)
            ),
            eph_m=float(eph),
            epv_m=float(epv),
            fix_type=_int_value(fix_type, "fix_type"),
            satellites_used=(None if satellites is None else _int_value(satellites, "satellites_used")),
        )
    except (TypeError, ValueError) as exc:
        raise SampleParseError(f"RTK message is invalid: {exc}") from exc


def rtk_world_position(
    sample: RtkSample,
    *,
    altitude_field: str,
    reference_origin: Wgs84ReferenceOrigin,
    gazebo_origin_in_reference_enu_m: Vector3,
) -> Vector3:
    altitude_m, sample_datum = sample.altitude_for_datum(altitude_field)
    if sample_datum != reference_origin.altitude_datum:
        raise SampleParseError(
            "RTK altitude datum does not match the configured WGS84 reference "
            f"datum: sample={sample_datum}, reference={reference_origin.altitude_datum}"
        )
    return wgs84_to_gazebo_world_enu(
        sample.latitude_deg,
        sample.longitude_deg,
        altitude_m,
        reference_origin,
        gazebo_origin_in_reference_enu_m,
    )


__all__ = [
    "CAMERA_OPTICAL_FRAME",
    "RTK_STATUS_ACCURACY_REJECTED",
    "PoseSourceSample",
    "RTK_STATUS_COORDINATE_TRANSFORM_FAILED",
    "RTK_STATUS_INVALID_FIX",
    "RTK_STATUS_JUMP_REJECTED",
    "RTK_STATUS_NONFINITE",
    "RTK_STATUS_STALE",
    "RTK_STATUS_TARGET_POINT_UNRESOLVED",
    "RTK_STATUS_TIME_ALIGNMENT_FAILED",
    "RTK_STATUS_VALID_FIXED",
    "RTK_STATUS_VALID_NON_FIXED",
    "RtkSample",
    "SampleParseError",
    "WORLD_FRAME",
    "pose_source_from_camera_diagnostic_json",
    "pose_source_from_pose_stamped",
    "rtk_sample_from_message",
    "rtk_world_position",
    "stamp_ns_from_ros_header",
]
