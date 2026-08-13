from __future__ import annotations

from collections.abc import Mapping
import hashlib
import json
import math
from dataclasses import asdict, dataclass
from typing import Any, Iterable

from Workstation.workstation_core.coordinate_contract import (
    BASELINE_TRANSLATION_FRAME,
    BODY_FRAME,
    CAMERA_FRAME,
    CAMERA_MOUNT_PROFILE,
    MEASUREMENT_AXIS_CONTRACT,
    MEASUREMENT_FRAME,
    PUBLIC_MEASUREMENT_VECTOR_CONTRACT,
    WORLD_FRAME,
)
from Workstation.workstation_core.rtk_geometry import ned_delta_m
from Workstation.workstation_core.stereo import StereoCalibration
from Workstation.workstation_core.triangulation import (
    camera_translation_right_in_left_camera,
    invert_left_to_right_camera_transform,
)


MetadataScalar = str | int | float | bool | None

QUATERNION_NORM_EPSILON = 1.0e-12
CAMERA_MATRIX_TOLERANCE = 1.0e-12
# Runtime calibration validation allows serialization noise; 1e-9 remains a
# synthetic geometry test tolerance, not a hardware/calibration acceptance gate.
ROTATION_MATRIX_TOLERANCE = 1.0e-6
CHECKERBOARD_FRAME_CONTRACT = (
    "opencv_optical:RDF(right,down,forward); "
    "X_right = R_left_to_right * X_left + T_left_to_right_m"
)

DETECTION_METADATA_ALLOWLIST = (
    "source_timestamp_s",
    "timestamp_source",
    "timestamp_reference",
    "rtp_timestamp",
    "rtp_ssrc",
    "rtp_marker_seq",
    "rtp_pts_s",
    "rtcp_sr_ntp_raw",
    "rtcp_sr_ntp_unix_s",
    "rtcp_sr_rtp_timestamp",
    "rtcp_clock_rate_hz",
    "rtcp_sr_age_s",
    "rtcp_sr_received_wall_s",
    "rtcp_sr_pc_offset_s",
    "rtcp_frame_ntp_unix_s",
    "rtcp_frame_received_wall_s",
    "rtcp_frame_receive_delay_s",
    "rtcp_frame_transport_delay_s",
    "rtcp_camera_ntp_rate",
    "rtcp_camera_media_rate",
    "rtcp_sr_received_count",
    "rtsp_transport",
    "rtcp_timing_invalid_reason",
)


def _finite_float(value: Any, *, name: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed):
        raise ValueError(f"{name} must be finite")
    return parsed


def _optional_finite_float(value: Any, *, name: str) -> float | None:
    return None if value is None else _finite_float(value, name=name)


def _optional_int(value: Any) -> int | None:
    return None if value is None else int(value)


def _finite_tuple(values: Iterable[Any], *, name: str, length: int | None = None) -> tuple[float, ...]:
    parsed = tuple(_finite_float(value, name=name) for value in values)
    if length is not None and len(parsed) != length:
        raise ValueError(f"{name} must contain {length} values")
    return parsed


def _determinant_3x3(matrix: tuple[tuple[float, float, float], ...]) -> float:
    return (
        matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1])
        - matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0])
        + matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0])
    )


def _validate_rotation_matrix(
    matrix: tuple[tuple[float, float, float], ...],
    *,
    name: str,
) -> None:
    max_orthogonality_error = max(
        abs(
            sum(matrix[row][left] * matrix[row][right] for row in range(3))
            - (1.0 if left == right else 0.0)
        )
        for left in range(3)
        for right in range(3)
    )
    if max_orthogonality_error > ROTATION_MATRIX_TOLERANCE:
        raise ValueError(f"{name} must be orthonormal within 1e-6")
    if abs(_determinant_3x3(matrix) - 1.0) > ROTATION_MATRIX_TOLERANCE:
        raise ValueError(f"{name} determinant must equal +1 within 1e-6")


def _canonical_json_bytes(payload: Any) -> bytes:
    return json.dumps(
        payload,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=False,
        allow_nan=False,
    ).encode("utf-8")


def _sha256_payload(payload: Any) -> str:
    return hashlib.sha256(_canonical_json_bytes(payload)).hexdigest()


def _canonical_detection_metadata_items(
    metadata_items: Any,
    *,
    project_allowlisted_keys: bool = False,
) -> tuple[tuple[str, MetadataScalar], ...]:
    """Validate and order immutable detection timestamp metadata."""

    if metadata_items is None:
        return ()
    if type(metadata_items) not in (tuple, list):
        raise TypeError("timestamp metadata must be a tuple or list of key/value pairs")

    values_by_key: dict[str, MetadataScalar] = {}
    for raw_item in metadata_items:
        if type(raw_item) not in (tuple, list):
            raise TypeError("each timestamp metadata item must be a tuple or list")
        if len(raw_item) != 2:
            raise ValueError("each timestamp metadata item must contain exactly two values")
        key, value = raw_item
        if type(key) is not str:
            raise TypeError("timestamp metadata keys must be strings")
        if key not in DETECTION_METADATA_ALLOWLIST:
            if project_allowlisted_keys:
                continue
            raise ValueError(f"timestamp metadata key is not allowed: {key}")
        if key in values_by_key:
            raise ValueError(f"duplicate timestamp metadata key: {key}")
        if value is None or type(value) in (str, bool, int):
            values_by_key[key] = value
        elif type(value) is float:
            values_by_key[key] = _finite_float(value, name=f"timestamp_metadata.{key}")
        else:
            raise TypeError(f"timestamp_metadata.{key} must be an immutable scalar")

    return tuple(
        (key, values_by_key[key])
        for key in DETECTION_METADATA_ALLOWLIST
        if key in values_by_key
    )


def _metadata_items(metadata: Any) -> tuple[tuple[str, MetadataScalar], ...]:
    if metadata is None:
        return ()
    if isinstance(metadata, Mapping):
        raw_items = tuple(metadata.items())
    elif type(metadata) in (tuple, list):
        raw_items = metadata
    else:
        raise TypeError("observation timestamp metadata must be a Mapping, tuple, or list")
    return _canonical_detection_metadata_items(
        raw_items,
        project_allowlisted_keys=True,
    )


@dataclass(frozen=True)
class DetectionSnapshot:
    camera: str
    app_receive_monotonic_s: float
    width_px: int
    height_px: int
    bbox_xyxy_px: tuple[float, float, float, float]
    center_px: tuple[float, float]
    confidence: float
    class_id: int
    class_name: str | None
    pairing_timestamp_basis: str
    pairing_timestamp_s: float
    timestamp_metadata_items: tuple[tuple[str, MetadataScalar], ...] = ()
    candidate_class_ids: tuple[int, ...] = ()
    pixel_frame: str = CAMERA_FRAME
    pixel_unit: str = "px"

    def __post_init__(self) -> None:
        object.__setattr__(self, "camera", str(self.camera))
        object.__setattr__(self, "app_receive_monotonic_s", _finite_float(self.app_receive_monotonic_s, name="app_receive_monotonic_s"))
        object.__setattr__(self, "width_px", int(self.width_px))
        object.__setattr__(self, "height_px", int(self.height_px))
        object.__setattr__(self, "bbox_xyxy_px", _finite_tuple(self.bbox_xyxy_px, name="bbox_xyxy_px", length=4))
        object.__setattr__(self, "center_px", _finite_tuple(self.center_px, name="center_px", length=2))
        object.__setattr__(self, "confidence", _finite_float(self.confidence, name="confidence"))
        object.__setattr__(self, "class_id", int(self.class_id))
        object.__setattr__(self, "class_name", None if self.class_name is None else str(self.class_name))
        object.__setattr__(self, "pairing_timestamp_basis", str(self.pairing_timestamp_basis))
        object.__setattr__(self, "pairing_timestamp_s", _finite_float(self.pairing_timestamp_s, name="pairing_timestamp_s"))
        object.__setattr__(
            self,
            "timestamp_metadata_items",
            _canonical_detection_metadata_items(self.timestamp_metadata_items),
        )
        object.__setattr__(
            self,
            "candidate_class_ids",
            tuple(int(class_id) for class_id in self.candidate_class_ids),
        )
        object.__setattr__(self, "pixel_frame", str(self.pixel_frame))
        object.__setattr__(self, "pixel_unit", str(self.pixel_unit))
        _finite_float(self.app_receive_monotonic_s, name="app_receive_monotonic_s")
        _finite_float(self.pairing_timestamp_s, name="pairing_timestamp_s")
        _finite_tuple(self.bbox_xyxy_px, name="bbox_xyxy_px", length=4)
        _finite_tuple(self.center_px, name="center_px", length=2)
        _finite_float(self.confidence, name="confidence")
        if self.width_px <= 0 or self.height_px <= 0:
            raise ValueError("detection image dimensions must be positive")
        if self.camera not in {"left", "right"}:
            raise ValueError("detection camera must be left or right")
        x1, y1, x2, y2 = self.bbox_xyxy_px
        if not (0.0 <= x1 < x2 <= float(self.width_px)):
            raise ValueError("bbox_xyxy_px must satisfy 0 <= x1 < x2 <= width")
        if not (0.0 <= y1 < y2 <= float(self.height_px)):
            raise ValueError("bbox_xyxy_px must satisfy 0 <= y1 < y2 <= height")
        if not 0.0 <= self.confidence <= 1.0:
            raise ValueError("confidence must be between 0 and 1")
        if self.class_id < 0 or any(class_id < 0 for class_id in self.candidate_class_ids):
            raise ValueError("class IDs must be non-negative")
        if self.pixel_frame != CAMERA_FRAME or self.pixel_unit != "px":
            raise ValueError("detection pixels must use camera_optical_rdf pixel coordinates")
    @classmethod
    def from_observation(
        cls,
        observation: Any,
        *,
        pairing_timestamp_basis: str,
        pairing_timestamp_s: float,
    ) -> "DetectionSnapshot":
        if cls is not DetectionSnapshot:
            raise TypeError("DetectionSnapshot.from_observation does not support subclasses")
        x1 = _finite_float(observation.x1, name="x1")
        y1 = _finite_float(observation.y1, name="y1")
        x2 = _finite_float(observation.x2, name="x2")
        y2 = _finite_float(observation.y2, name="y2")
        return cls(
            camera=str(observation.camera),
            app_receive_monotonic_s=_finite_float(observation.timestamp_s, name="timestamp_s"),
            width_px=int(observation.width),
            height_px=int(observation.height),
            bbox_xyxy_px=(x1, y1, x2, y2),
            center_px=((x1 + x2) / 2.0, (y1 + y2) / 2.0),
            confidence=_finite_float(observation.confidence, name="confidence"),
            class_id=int(observation.class_id),
            class_name=None if observation.class_name is None else str(observation.class_name),
            pairing_timestamp_basis=str(pairing_timestamp_basis),
            pairing_timestamp_s=_finite_float(pairing_timestamp_s, name="pairing_timestamp_s"),
            timestamp_metadata_items=_metadata_items(getattr(observation, "timestamp_metadata", None)),
            candidate_class_ids=tuple(getattr(observation, "candidate_class_ids", ()) or ()),
        )

    @property
    def timestamp_s(self) -> float:
        return self.app_receive_monotonic_s

    @property
    def width(self) -> int:
        return self.width_px

    @property
    def height(self) -> int:
        return self.height_px

    @property
    def x1(self) -> float:
        return self.bbox_xyxy_px[0]

    @property
    def y1(self) -> float:
        return self.bbox_xyxy_px[1]

    @property
    def x2(self) -> float:
        return self.bbox_xyxy_px[2]

    @property
    def y2(self) -> float:
        return self.bbox_xyxy_px[3]

    @property
    def center(self) -> tuple[float, float]:
        return self.center_px

    @property
    def timestamp_metadata(self) -> dict[str, MetadataScalar]:
        """Return a detached compatibility copy; the snapshot stores tuples only."""

        return dict(self.timestamp_metadata_items)

    @property
    def same_class_candidate_count(self) -> int:
        candidates = self.candidate_class_ids or (self.class_id,)
        return sum(class_id == self.class_id for class_id in candidates)


@dataclass(frozen=True)
class AttitudeSnapshot:
    camera: str
    app_receive_monotonic_s: float
    quaternion_wxyz: tuple[float, float, float, float]
    namespace: str
    topic: str
    px4_timestamp_s: float | None
    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    px4_timestamp_us: int | None = None
    px4_timestamp_sample_us: int | None = None
    delta_q_reset: tuple[float, float, float, float] | None = None
    quat_reset_counter: int | None = None
    quaternion_order: str = "wxyz"
    orientation_from_frame: str = BODY_FRAME
    orientation_to_frame: str = WORLD_FRAME
    mavlink_source_system: int | None = None
    mavlink_source_component: int | None = None
    mavlink_sequence: int | None = None
    mavlink_message_id: int | None = None

    def __post_init__(self) -> None:
        object.__setattr__(self, "camera", str(self.camera))
        object.__setattr__(self, "app_receive_monotonic_s", _finite_float(self.app_receive_monotonic_s, name="attitude timestamp"))
        object.__setattr__(
            self,
            "quaternion_wxyz",
            _finite_tuple(self.quaternion_wxyz, name="quaternion_wxyz", length=4),
        )
        object.__setattr__(self, "namespace", str(self.namespace))
        object.__setattr__(self, "topic", str(self.topic))
        object.__setattr__(self, "px4_timestamp_s", _optional_finite_float(self.px4_timestamp_s, name="px4_timestamp_s"))
        object.__setattr__(self, "roll_deg", _finite_float(self.roll_deg, name="roll_deg"))
        object.__setattr__(self, "pitch_deg", _finite_float(self.pitch_deg, name="pitch_deg"))
        object.__setattr__(self, "yaw_deg", _finite_float(self.yaw_deg, name="yaw_deg"))
        object.__setattr__(self, "px4_timestamp_us", _optional_int(self.px4_timestamp_us))
        object.__setattr__(self, "px4_timestamp_sample_us", _optional_int(self.px4_timestamp_sample_us))
        if self.delta_q_reset is not None:
            object.__setattr__(
                self,
                "delta_q_reset",
                _finite_tuple(self.delta_q_reset, name="delta_q_reset", length=4),
            )
        object.__setattr__(self, "quat_reset_counter", _optional_int(self.quat_reset_counter))
        object.__setattr__(self, "quaternion_order", str(self.quaternion_order))
        object.__setattr__(self, "orientation_from_frame", str(self.orientation_from_frame))
        object.__setattr__(self, "orientation_to_frame", str(self.orientation_to_frame))
        _finite_float(self.app_receive_monotonic_s, name="attitude timestamp")
        q = _finite_tuple(self.quaternion_wxyz, name="quaternion_wxyz", length=4)
        if math.sqrt(sum(value * value for value in q)) <= QUATERNION_NORM_EPSILON:
            raise ValueError("quaternion_wxyz norm must be greater than 1e-12")
        _optional_finite_float(self.px4_timestamp_s, name="px4_timestamp_s")
        for name, value in (("roll_deg", self.roll_deg), ("pitch_deg", self.pitch_deg), ("yaw_deg", self.yaw_deg)):
            _finite_float(value, name=name)
        if self.quaternion_order != "wxyz":
            raise ValueError("quaternion order must be wxyz")
        if self.camera not in {"left", "right"}:
            raise ValueError("attitude camera must be left or right")
        if self.orientation_from_frame != BODY_FRAME or self.orientation_to_frame != WORLD_FRAME:
            raise ValueError("attitude must rotate PX4 body FRD components into world NED")

    @classmethod
    def from_state(cls, state: Any) -> "AttitudeSnapshot":
        return cls(
            camera=str(state.camera),
            app_receive_monotonic_s=_finite_float(state.timestamp_s, name="attitude timestamp"),
            quaternion_wxyz=tuple(_finite_tuple(state.q, name="quaternion_wxyz", length=4)),  # type: ignore[arg-type]
            namespace=str(state.namespace),
            topic=str(state.topic),
            px4_timestamp_s=_optional_finite_float(getattr(state, "px4_timestamp_s", None), name="px4_timestamp_s"),
            roll_deg=_finite_float(state.roll_deg, name="roll_deg"),
            pitch_deg=_finite_float(state.pitch_deg, name="pitch_deg"),
            yaw_deg=_finite_float(state.yaw_deg, name="yaw_deg"),
            px4_timestamp_us=_optional_int(getattr(state, "px4_timestamp_us", None)),
            px4_timestamp_sample_us=_optional_int(getattr(state, "px4_timestamp_sample_us", None)),
            delta_q_reset=(
                None
                if getattr(state, "delta_q_reset", None) is None
                else tuple(_finite_tuple(state.delta_q_reset, name="delta_q_reset", length=4))
            ),
            quat_reset_counter=_optional_int(getattr(state, "quat_reset_counter", None)),
            mavlink_source_system=_optional_int(getattr(state, "mavlink_source_system", None)),
            mavlink_source_component=_optional_int(getattr(state, "mavlink_source_component", None)),
            mavlink_sequence=_optional_int(getattr(state, "mavlink_sequence", None)),
            mavlink_message_id=_optional_int(getattr(state, "mavlink_message_id", None)),
        )

    @property
    def timestamp_s(self) -> float:
        return self.app_receive_monotonic_s

    @property
    def q(self) -> tuple[float, float, float, float]:
        return self.quaternion_wxyz


@dataclass(frozen=True)
class GpsSnapshot:
    camera: str
    app_receive_monotonic_s: float
    latitude_deg: float | None
    longitude_deg: float | None
    altitude_m: float | None
    fix_type: int | None
    satellites_used: int | None
    eph_m: float | None
    epv_m: float | None
    rtk_label: str
    namespace: str
    topic: str
    px4_timestamp_s: float | None
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
    time_source: str = ""
    heading_deg: float | None = None
    heading_accuracy_deg: float | None = None
    position_frame: str = "wgs84_geodetic"
    angle_unit: str = "deg"
    altitude_unit: str = "m"
    mavlink_source_system: int | None = None
    mavlink_source_component: int | None = None
    mavlink_sequence: int | None = None
    mavlink_message_id: int | None = None

    def __post_init__(self) -> None:
        object.__setattr__(self, "camera", str(self.camera))
        object.__setattr__(self, "app_receive_monotonic_s", _finite_float(self.app_receive_monotonic_s, name="GPS timestamp"))
        for name in (
            "latitude_deg",
            "longitude_deg",
            "altitude_m",
            "eph_m",
            "epv_m",
            "px4_timestamp_s",
            "heading_deg",
            "heading_accuracy_deg",
            "altitude_ellipsoid_m",
            "velocity_n_m_s",
            "velocity_e_m_s",
            "velocity_d_m_s",
            "hdop",
            "vdop",
        ):
            object.__setattr__(self, name, _optional_finite_float(getattr(self, name), name=name))
        object.__setattr__(self, "fix_type", None if self.fix_type is None else int(self.fix_type))
        object.__setattr__(self, "satellites_used", None if self.satellites_used is None else int(self.satellites_used))
        for name in (
            "px4_timestamp_us",
            "px4_timestamp_sample_us",
            "gps_utc_us",
            "gps_timestamp_time_relative_us",
            "device_id",
        ):
            object.__setattr__(self, name, _optional_int(getattr(self, name)))
        object.__setattr__(
            self,
            "velocity_ned_valid",
            None if self.velocity_ned_valid is None else bool(self.velocity_ned_valid),
        )
        object.__setattr__(self, "time_source", str(self.time_source))
        for name in ("rtk_label", "namespace", "topic", "position_frame", "angle_unit", "altitude_unit"):
            object.__setattr__(self, name, str(getattr(self, name)))
        _finite_float(self.app_receive_monotonic_s, name="GPS timestamp")
        for name, value in (
            ("latitude_deg", self.latitude_deg),
            ("longitude_deg", self.longitude_deg),
            ("altitude_m", self.altitude_m),
            ("eph_m", self.eph_m),
            ("epv_m", self.epv_m),
            ("px4_timestamp_s", self.px4_timestamp_s),
            ("heading_deg", self.heading_deg),
            ("heading_accuracy_deg", self.heading_accuracy_deg),
        ):
            _optional_finite_float(value, name=name)
        if self.camera not in {"left", "right"}:
            raise ValueError("GPS camera must be left or right")
        if self.latitude_deg is not None and not -90.0 <= self.latitude_deg <= 90.0:
            raise ValueError("latitude_deg must be between -90 and 90")
        if self.longitude_deg is not None and not -180.0 <= self.longitude_deg <= 180.0:
            raise ValueError("longitude_deg must be between -180 and 180")
        if self.eph_m is not None and self.eph_m < 0.0:
            raise ValueError("eph_m must be non-negative")
        if self.epv_m is not None and self.epv_m < 0.0:
            raise ValueError("epv_m must be non-negative")
        if self.position_frame != "wgs84_geodetic" or self.angle_unit != "deg" or self.altitude_unit != "m":
            raise ValueError("GPS snapshot frame/unit contract mismatch")

    @classmethod
    def from_state(cls, state: Any) -> "GpsSnapshot":
        return cls(
            camera=str(state.camera),
            app_receive_monotonic_s=_finite_float(state.timestamp_s, name="GPS timestamp"),
            latitude_deg=_optional_finite_float(state.latitude_deg, name="latitude_deg"),
            longitude_deg=_optional_finite_float(state.longitude_deg, name="longitude_deg"),
            altitude_m=_optional_finite_float(state.altitude_m, name="altitude_m"),
            fix_type=None if state.fix_type is None else int(state.fix_type),
            satellites_used=None if state.satellites_used is None else int(state.satellites_used),
            eph_m=_optional_finite_float(state.eph_m, name="eph_m"),
            epv_m=_optional_finite_float(state.epv_m, name="epv_m"),
            rtk_label=str(state.rtk_label),
            namespace=str(state.namespace),
            topic=str(state.topic),
            px4_timestamp_s=_optional_finite_float(getattr(state, "px4_timestamp_s", None), name="px4_timestamp_s"),
            px4_timestamp_us=_optional_int(getattr(state, "px4_timestamp_us", None)),
            px4_timestamp_sample_us=_optional_int(getattr(state, "px4_timestamp_sample_us", None)),
            gps_utc_us=_optional_int(getattr(state, "gps_utc_us", None)),
            gps_timestamp_time_relative_us=_optional_int(
                getattr(state, "gps_timestamp_time_relative_us", None)
            ),
            altitude_ellipsoid_m=_optional_finite_float(
                getattr(state, "altitude_ellipsoid_m", None), name="altitude_ellipsoid_m"
            ),
            velocity_n_m_s=_optional_finite_float(
                getattr(state, "velocity_n_m_s", None), name="velocity_n_m_s"
            ),
            velocity_e_m_s=_optional_finite_float(
                getattr(state, "velocity_e_m_s", None), name="velocity_e_m_s"
            ),
            velocity_d_m_s=_optional_finite_float(
                getattr(state, "velocity_d_m_s", None), name="velocity_d_m_s"
            ),
            velocity_ned_valid=getattr(state, "velocity_ned_valid", None),
            hdop=_optional_finite_float(getattr(state, "hdop", None), name="hdop"),
            vdop=_optional_finite_float(getattr(state, "vdop", None), name="vdop"),
            device_id=_optional_int(getattr(state, "device_id", None)),
            time_source=str(getattr(state, "time_source", "")),
            heading_deg=_optional_finite_float(getattr(state, "heading_deg", None), name="heading_deg"),
            heading_accuracy_deg=_optional_finite_float(
                getattr(state, "heading_accuracy_deg", None), name="heading_accuracy_deg"
            ),
            mavlink_source_system=_optional_int(getattr(state, "mavlink_source_system", None)),
            mavlink_source_component=_optional_int(getattr(state, "mavlink_source_component", None)),
            mavlink_sequence=_optional_int(getattr(state, "mavlink_sequence", None)),
            mavlink_message_id=_optional_int(getattr(state, "mavlink_message_id", None)),
        )

    @property
    def timestamp_s(self) -> float:
        return self.app_receive_monotonic_s

    @property
    def has_position(self) -> bool:
        return self.latitude_deg is not None and self.longitude_deg is not None and self.altitude_m is not None


@dataclass(frozen=True)
class CameraIntrinsicsSnapshot:
    width_px: int
    height_px: int
    camera_matrix: tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]
    distortion_coefficients: tuple[float, ...]
    reprojection_error_px: float
    image_count: int
    pixel_unit: str = "px"
    camera_frame: str = CAMERA_FRAME

    def __post_init__(self) -> None:
        object.__setattr__(self, "width_px", int(self.width_px))
        object.__setattr__(self, "height_px", int(self.height_px))
        matrix = tuple(tuple(_finite_tuple(row, name="camera_matrix", length=3)) for row in self.camera_matrix)
        object.__setattr__(self, "camera_matrix", matrix)
        object.__setattr__(
            self,
            "distortion_coefficients",
            _finite_tuple(self.distortion_coefficients, name="distortion_coefficients"),
        )
        object.__setattr__(self, "reprojection_error_px", _finite_float(self.reprojection_error_px, name="reprojection_error_px"))
        object.__setattr__(self, "image_count", int(self.image_count))
        object.__setattr__(self, "pixel_unit", str(self.pixel_unit))
        object.__setattr__(self, "camera_frame", str(self.camera_frame))
        if self.width_px <= 0 or self.height_px <= 0:
            raise ValueError("intrinsics dimensions must be positive")
        if len(self.camera_matrix) != 3 or any(len(row) != 3 for row in self.camera_matrix):
            raise ValueError("camera_matrix must be 3x3")
        for row in self.camera_matrix:
            _finite_tuple(row, name="camera_matrix", length=3)
        _finite_tuple(self.distortion_coefficients, name="distortion_coefficients")
        _finite_float(self.reprojection_error_px, name="reprojection_error_px")
        if self.image_count < 0:
            raise ValueError("image_count must be non-negative")
        if not self.distortion_coefficients:
            raise ValueError("distortion_coefficients must not be empty")
        if self.camera_matrix[0][0] <= 0.0 or self.camera_matrix[1][1] <= 0.0:
            raise ValueError("camera_matrix focal lengths must be positive")
        if any(abs(self.camera_matrix[2][index] - expected) > CAMERA_MATRIX_TOLERANCE for index, expected in enumerate((0.0, 0.0, 1.0))):
            raise ValueError("camera_matrix last row must be [0, 0, 1]")
        if self.pixel_unit != "px" or self.camera_frame != CAMERA_FRAME:
            raise ValueError("intrinsics frame/unit contract mismatch")

    @classmethod
    def from_intrinsics(cls, intrinsics: Any) -> "CameraIntrinsicsSnapshot":
        matrix = tuple(tuple(_finite_tuple(row, name="camera_matrix", length=3)) for row in intrinsics.camera_matrix)
        return cls(
            width_px=int(intrinsics.width),
            height_px=int(intrinsics.height),
            camera_matrix=matrix,  # type: ignore[arg-type]
            distortion_coefficients=tuple(_finite_tuple(intrinsics.distortion_coefficients, name="distortion_coefficients")),
            reprojection_error_px=_finite_float(intrinsics.reprojection_error_px, name="reprojection_error_px"),
            image_count=int(intrinsics.image_count),
        )

    @property
    def width(self) -> int:
        return self.width_px

    @property
    def height(self) -> int:
        return self.height_px


@dataclass(frozen=True)
class CheckerboardExtrinsicsSnapshot:
    rotation_left_to_right: tuple[
        tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]
    ]
    translation_left_to_right_m: tuple[float, float, float]
    reprojection_error_px: float
    pair_count: int
    frame_contract: str

    def __post_init__(self) -> None:
        rotation = tuple(
            tuple(_finite_tuple(row, name="rotation_left_to_right", length=3))
            for row in self.rotation_left_to_right
        )
        object.__setattr__(self, "rotation_left_to_right", rotation)
        object.__setattr__(
            self,
            "translation_left_to_right_m",
            _finite_tuple(self.translation_left_to_right_m, name="translation_left_to_right_m", length=3),
        )
        object.__setattr__(self, "reprojection_error_px", _finite_float(self.reprojection_error_px, name="checkerboard reprojection_error_px"))
        object.__setattr__(self, "pair_count", int(self.pair_count))
        object.__setattr__(self, "frame_contract", str(self.frame_contract))
        if len(self.rotation_left_to_right) != 3 or any(len(row) != 3 for row in self.rotation_left_to_right):
            raise ValueError("rotation_left_to_right must be 3x3")
        for row in self.rotation_left_to_right:
            _finite_tuple(row, name="rotation_left_to_right", length=3)
        _finite_tuple(self.translation_left_to_right_m, name="translation_left_to_right_m", length=3)
        _finite_float(self.reprojection_error_px, name="checkerboard reprojection_error_px")
        if self.pair_count < 0:
            raise ValueError("pair_count must be non-negative")
        _validate_rotation_matrix(self.rotation_left_to_right, name="rotation_left_to_right")
        if self.frame_contract != CHECKERBOARD_FRAME_CONTRACT:
            raise ValueError("checkerboard frame_contract does not match the active RDF transform contract")

    @classmethod
    def from_extrinsics(cls, extrinsics: Any) -> "CheckerboardExtrinsicsSnapshot":
        rotation = tuple(
            tuple(_finite_tuple(row, name="rotation_left_to_right", length=3))
            for row in extrinsics.rotation_left_to_right
        )
        return cls(
            rotation_left_to_right=rotation,  # type: ignore[arg-type]
            translation_left_to_right_m=tuple(
                _finite_tuple(extrinsics.translation_left_to_right_m, name="translation_left_to_right_m", length=3)
            ),  # type: ignore[arg-type]
            reprojection_error_px=_finite_float(
                extrinsics.reprojection_error_px, name="checkerboard reprojection_error_px"
            ),
            pair_count=int(extrinsics.pair_count),
            frame_contract=str(extrinsics.frame_contract),
        )


@dataclass(frozen=True)
class StereoIntrinsicsSnapshot:
    left: CameraIntrinsicsSnapshot
    right: CameraIntrinsicsSnapshot
    checkerboard_extrinsics: CheckerboardExtrinsicsSnapshot | None
    active_zoom_level: int
    selected_profile_path: str
    content_sha256: str
    intrinsics_content_sha256: str = ""
    selected_extrinsics_profile_path: str = ""
    extrinsics_content_sha256: str = ""
    intrinsics_source_zoom_level: int | None = None
    extrinsics_source_zoom_level: int | None = None
    selection_mode: str = "legacy_combined"
    validation_gate: str = "disabled"

    def __post_init__(self) -> None:
        object.__setattr__(self, "active_zoom_level", int(self.active_zoom_level))
        object.__setattr__(self, "selected_profile_path", str(self.selected_profile_path))
        object.__setattr__(self, "content_sha256", str(self.content_sha256))
        object.__setattr__(
            self,
            "intrinsics_source_zoom_level",
            self.active_zoom_level
            if self.intrinsics_source_zoom_level is None
            else int(self.intrinsics_source_zoom_level),
        )
        object.__setattr__(
            self,
            "extrinsics_source_zoom_level",
            None
            if self.extrinsics_source_zoom_level is None
            else int(self.extrinsics_source_zoom_level),
        )
        object.__setattr__(
            self,
            "selected_extrinsics_profile_path",
            str(self.selected_extrinsics_profile_path),
        )
        object.__setattr__(self, "selection_mode", str(self.selection_mode))
        object.__setattr__(self, "validation_gate", str(self.validation_gate))
        if not isinstance(self.left, CameraIntrinsicsSnapshot) or not isinstance(
            self.right, CameraIntrinsicsSnapshot
        ):
            raise TypeError("left/right intrinsics must be immutable CameraIntrinsicsSnapshot values")
        if self.checkerboard_extrinsics is not None and not isinstance(
            self.checkerboard_extrinsics, CheckerboardExtrinsicsSnapshot
        ):
            raise TypeError("checkerboard_extrinsics must be an immutable snapshot")
        if self.active_zoom_level < 0:
            raise ValueError("active_zoom_level must be non-negative")
        if len(self.content_sha256) != 64 or any(character not in "0123456789abcdef" for character in self.content_sha256):
            raise ValueError("content_sha256 must be a lowercase SHA-256 hex digest")
        intrinsic_digest = self.intrinsics_content_sha256 or _sha256_payload(
            {"left": asdict(self.left), "right": asdict(self.right)}
        )
        object.__setattr__(self, "intrinsics_content_sha256", intrinsic_digest)
        extrinsics_digest = self.extrinsics_content_sha256
        if self.checkerboard_extrinsics is not None and not extrinsics_digest:
            extrinsics_digest = _sha256_payload(asdict(self.checkerboard_extrinsics))
        object.__setattr__(self, "extrinsics_content_sha256", extrinsics_digest)
        for name, digest in (
            ("intrinsics_content_sha256", intrinsic_digest),
            ("extrinsics_content_sha256", extrinsics_digest),
        ):
            if digest and (
                len(digest) != 64
                or any(character not in "0123456789abcdef" for character in digest)
            ):
                raise ValueError(f"{name} must be a lowercase SHA-256 hex digest")
        if self.intrinsics_source_zoom_level is not None and self.intrinsics_source_zoom_level < 0:
            raise ValueError("intrinsics_source_zoom_level must be non-negative")
        if self.extrinsics_source_zoom_level is not None and self.extrinsics_source_zoom_level < 0:
            raise ValueError("extrinsics_source_zoom_level must be non-negative")
        if self.validation_gate != "disabled":
            raise ValueError("calibration validation_gate must remain disabled")

    @classmethod
    def from_result(
        cls,
        result: Any,
        *,
        active_zoom_level: int,
        selected_profile_path: str,
        selected_extrinsics_profile_path: str = "",
        intrinsics_source_zoom_level: int | None = None,
        extrinsics_source_zoom_level: int | None = None,
        selection_mode: str = "legacy_combined",
    ) -> "StereoIntrinsicsSnapshot":
        left = CameraIntrinsicsSnapshot.from_intrinsics(result.left)
        right = CameraIntrinsicsSnapshot.from_intrinsics(result.right)
        extrinsics = (
            None
            if result.extrinsics is None
            else CheckerboardExtrinsicsSnapshot.from_extrinsics(result.extrinsics)
        )
        content = {
            "left": asdict(left),
            "right": asdict(right),
            "checkerboard_extrinsics": None if extrinsics is None else asdict(extrinsics),
        }
        return cls(
            left=left,
            right=right,
            checkerboard_extrinsics=extrinsics,
            active_zoom_level=int(active_zoom_level),
            selected_profile_path=str(selected_profile_path),
            content_sha256=_sha256_payload(content),
            intrinsics_content_sha256=_sha256_payload(
                {"left": asdict(left), "right": asdict(right)}
            ),
            selected_extrinsics_profile_path=str(selected_extrinsics_profile_path),
            extrinsics_content_sha256=(
                "" if extrinsics is None else _sha256_payload(asdict(extrinsics))
            ),
            intrinsics_source_zoom_level=(
                int(active_zoom_level)
                if intrinsics_source_zoom_level is None
                else int(intrinsics_source_zoom_level)
            ),
            extrinsics_source_zoom_level=(
                None
                if extrinsics_source_zoom_level is None
                else int(extrinsics_source_zoom_level)
            ),
            selection_mode=str(selection_mode),
            validation_gate="disabled",
        )

    @property
    def extrinsics(self) -> CheckerboardExtrinsicsSnapshot | None:
        return self.checkerboard_extrinsics


@dataclass(frozen=True)
class BaselineSnapshot:
    source: str
    baseline_m: float
    ned_right_from_left_m: tuple[float, float, float] | None
    translation_right_in_left_rdf_m: tuple[float, float, float] | None
    left_basis_quaternion_wxyz: tuple[float, float, float, float] | None
    fix_quality: str
    warnings: tuple[str, ...]
    usable_for_translation: bool
    unusable_reason: str
    gps_pair_delta_s: float | None
    left_age_at_capture_s: float | None
    right_age_at_capture_s: float | None
    left_age_from_detection_midpoint_s: float | None
    right_age_from_detection_midpoint_s: float | None
    baseline_frame: str = WORLD_FRAME
    translation_frame: str = BASELINE_TRANSLATION_FRAME
    unit: str = "m"
    left_basis_attitude_monotonic_s: float | None = None
    gps_selection_basis: str = "latest_app_monotonic_at_attempt_capture"
    gps_clock_domain: str = "app_monotonic_receive"

    def __post_init__(self) -> None:
        object.__setattr__(self, "source", str(self.source))
        object.__setattr__(self, "baseline_m", _finite_float(self.baseline_m, name="baseline_m"))
        if self.ned_right_from_left_m is not None:
            object.__setattr__(
                self,
                "ned_right_from_left_m",
                _finite_tuple(self.ned_right_from_left_m, name="ned_right_from_left_m", length=3),
            )
        if self.translation_right_in_left_rdf_m is not None:
            object.__setattr__(
                self,
                "translation_right_in_left_rdf_m",
                _finite_tuple(
                    self.translation_right_in_left_rdf_m,
                    name="translation_right_in_left_rdf_m",
                    length=3,
                ),
            )
        if self.left_basis_quaternion_wxyz is not None:
            object.__setattr__(
                self,
                "left_basis_quaternion_wxyz",
                _finite_tuple(
                    self.left_basis_quaternion_wxyz,
                    name="left_basis_quaternion_wxyz",
                    length=4,
                ),
            )
        object.__setattr__(self, "warnings", tuple(str(value) for value in self.warnings))
        object.__setattr__(self, "fix_quality", str(self.fix_quality))
        object.__setattr__(self, "usable_for_translation", bool(self.usable_for_translation))
        object.__setattr__(self, "unusable_reason", str(self.unusable_reason))
        for name in (
            "gps_pair_delta_s",
            "left_age_at_capture_s",
            "right_age_at_capture_s",
            "left_age_from_detection_midpoint_s",
            "right_age_from_detection_midpoint_s",
        ):
            object.__setattr__(self, name, _optional_finite_float(getattr(self, name), name=name))
        object.__setattr__(self, "baseline_frame", str(self.baseline_frame))
        object.__setattr__(self, "translation_frame", str(self.translation_frame))
        object.__setattr__(self, "unit", str(self.unit))
        object.__setattr__(
            self,
            "left_basis_attitude_monotonic_s",
            _optional_finite_float(
                self.left_basis_attitude_monotonic_s,
                name="left_basis_attitude_monotonic_s",
            ),
        )
        object.__setattr__(self, "gps_selection_basis", str(self.gps_selection_basis))
        object.__setattr__(self, "gps_clock_domain", str(self.gps_clock_domain))
        _finite_float(self.baseline_m, name="baseline_m")
        if self.ned_right_from_left_m is not None:
            _finite_tuple(self.ned_right_from_left_m, name="ned_right_from_left_m", length=3)
        if self.translation_right_in_left_rdf_m is not None:
            _finite_tuple(
                self.translation_right_in_left_rdf_m,
                name="translation_right_in_left_rdf_m",
                length=3,
            )
        if self.left_basis_quaternion_wxyz is not None:
            _finite_tuple(self.left_basis_quaternion_wxyz, name="left_basis_quaternion_wxyz", length=4)
        for name, value in (
            ("gps_pair_delta_s", self.gps_pair_delta_s),
            ("left_age_at_capture_s", self.left_age_at_capture_s),
            ("right_age_at_capture_s", self.right_age_at_capture_s),
            ("left_age_from_detection_midpoint_s", self.left_age_from_detection_midpoint_s),
            ("right_age_from_detection_midpoint_s", self.right_age_from_detection_midpoint_s),
        ):
            _optional_finite_float(value, name=name)
        if self.baseline_m < 0.0:
            raise ValueError("baseline_m must be non-negative")
        if (
            self.baseline_frame != WORLD_FRAME
            or self.translation_frame != BASELINE_TRANSLATION_FRAME
            or self.unit != "m"
        ):
            raise ValueError("baseline frame/unit contract mismatch")


@dataclass(frozen=True)
class GeometryConfigSnapshot:
    focal_length_px: float
    max_time_delta_s: float | None
    max_vertical_delta_px: float | None
    require_same_class: bool
    timestamp_basis: str
    manual_focal_override: bool
    require_rtk_fixed_setting: bool = False

    def __post_init__(self) -> None:
        object.__setattr__(self, "focal_length_px", _finite_float(self.focal_length_px, name="focal_length_px"))
        object.__setattr__(self, "max_time_delta_s", _optional_finite_float(self.max_time_delta_s, name="max_time_delta_s"))
        object.__setattr__(self, "max_vertical_delta_px", _optional_finite_float(self.max_vertical_delta_px, name="max_vertical_delta_px"))
        object.__setattr__(self, "require_same_class", bool(self.require_same_class))
        object.__setattr__(self, "timestamp_basis", str(self.timestamp_basis))
        object.__setattr__(self, "manual_focal_override", bool(self.manual_focal_override))
        object.__setattr__(self, "require_rtk_fixed_setting", bool(self.require_rtk_fixed_setting))
        _finite_float(self.focal_length_px, name="focal_length_px")
        _optional_finite_float(self.max_time_delta_s, name="max_time_delta_s")
        _optional_finite_float(self.max_vertical_delta_px, name="max_vertical_delta_px")

    def to_stereo_calibration(self, *, baseline_m: float) -> StereoCalibration:
        return StereoCalibration(
            focal_length_px=self.focal_length_px,
            baseline_m=_finite_float(baseline_m, name="baseline_m"),
            max_time_delta_s=float("inf") if self.max_time_delta_s is None else self.max_time_delta_s,
            max_vertical_delta_px=(
                float("inf") if self.max_vertical_delta_px is None else self.max_vertical_delta_px
            ),
            require_same_class=self.require_same_class,
            timestamp_basis=self.timestamp_basis,
        )


@dataclass(frozen=True)
class MeasurementAttempt:
    attempt_id: int
    attempt_session_id: str
    detection_pair_key: tuple[MetadataScalar, ...]
    created_monotonic_s: float
    created_wall_s: float
    detection_midpoint_monotonic_s: float
    timestamp_basis: str
    triangulation_source: str
    left_detection: DetectionSnapshot
    right_detection: DetectionSnapshot
    left_attitude: AttitudeSnapshot | None
    right_attitude: AttitudeSnapshot | None
    left_gps: GpsSnapshot | None
    right_gps: GpsSnapshot | None
    intrinsics: StereoIntrinsicsSnapshot | None
    baseline: BaselineSnapshot | None
    geometry_config: GeometryConfigSnapshot
    left_attitude_age_s: float | None
    right_attitude_age_s: float | None
    attitude_pair_delta_s: float | None
    relative_roll_deg: float | None
    relative_pitch_deg: float | None
    relative_yaw_deg: float | None
    source_quality: str
    warnings: tuple[str, ...]
    world_frame: str = WORLD_FRAME
    body_frame: str = BODY_FRAME
    camera_frame: str = CAMERA_FRAME
    camera_mount_profile: str = CAMERA_MOUNT_PROFILE
    measurement_frame: str = MEASUREMENT_FRAME
    measurement_axis_contract: str = MEASUREMENT_AXIS_CONTRACT
    public_vector_contract: str = PUBLIC_MEASUREMENT_VECTOR_CONTRACT
    measurement_unit: str = "m"

    def __post_init__(self) -> None:
        object.__setattr__(self, "attempt_id", int(self.attempt_id))
        object.__setattr__(self, "attempt_session_id", str(self.attempt_session_id))
        object.__setattr__(self, "detection_pair_key", tuple(self.detection_pair_key))
        for name in ("created_monotonic_s", "created_wall_s", "detection_midpoint_monotonic_s"):
            object.__setattr__(self, name, _finite_float(getattr(self, name), name=name))
        for name in (
            "left_attitude_age_s",
            "right_attitude_age_s",
            "attitude_pair_delta_s",
            "relative_roll_deg",
            "relative_pitch_deg",
            "relative_yaw_deg",
        ):
            object.__setattr__(self, name, _optional_finite_float(getattr(self, name), name=name))
        for name in (
            "timestamp_basis",
            "triangulation_source",
            "source_quality",
            "world_frame",
            "body_frame",
            "camera_frame",
            "camera_mount_profile",
            "measurement_frame",
            "measurement_axis_contract",
            "public_vector_contract",
            "measurement_unit",
        ):
            object.__setattr__(self, name, str(getattr(self, name)))
        object.__setattr__(self, "warnings", tuple(str(value) for value in self.warnings))
        if type(self.left_detection) is not DetectionSnapshot or type(
            self.right_detection
        ) is not DetectionSnapshot:
            raise TypeError("detections must be exact immutable DetectionSnapshot values")
        for value in (self.left_attitude, self.right_attitude):
            if value is not None and not isinstance(value, AttitudeSnapshot):
                raise TypeError("attitudes must be immutable AttitudeSnapshot values")
        for value in (self.left_gps, self.right_gps):
            if value is not None and not isinstance(value, GpsSnapshot):
                raise TypeError("GPS inputs must be immutable GpsSnapshot values")
        if self.intrinsics is not None and not isinstance(self.intrinsics, StereoIntrinsicsSnapshot):
            raise TypeError("intrinsics must be an immutable StereoIntrinsicsSnapshot")
        if self.baseline is not None and not isinstance(self.baseline, BaselineSnapshot):
            raise TypeError("baseline must be an immutable BaselineSnapshot")
        if not isinstance(self.geometry_config, GeometryConfigSnapshot):
            raise TypeError("geometry_config must be an immutable GeometryConfigSnapshot")
        if self.attempt_id <= 0:
            raise ValueError("attempt_id must be positive")
        if not self.attempt_session_id:
            raise ValueError("attempt_session_id must not be empty")
        for name, value in (
            ("created_monotonic_s", self.created_monotonic_s),
            ("created_wall_s", self.created_wall_s),
            ("detection_midpoint_monotonic_s", self.detection_midpoint_monotonic_s),
            ("left_attitude_age_s", self.left_attitude_age_s),
            ("right_attitude_age_s", self.right_attitude_age_s),
            ("attitude_pair_delta_s", self.attitude_pair_delta_s),
            ("relative_roll_deg", self.relative_roll_deg),
            ("relative_pitch_deg", self.relative_pitch_deg),
            ("relative_yaw_deg", self.relative_yaw_deg),
        ):
            if value is None:
                continue
            _finite_float(value, name=name)
        for value in self.detection_pair_key:
            if isinstance(value, float) and not math.isfinite(value):
                raise ValueError("detection_pair_key must not contain NaN or Inf")
            if value is not None and not isinstance(value, (str, int, float, bool)):
                raise ValueError("detection_pair_key values must be scalar")
        if self.triangulation_source not in {"pixhawk", "checkerboard"}:
            raise ValueError("triangulation_source must be pixhawk or checkerboard")
        expected_contracts = {
            "world_frame": WORLD_FRAME,
            "body_frame": BODY_FRAME,
            "camera_frame": CAMERA_FRAME,
            "camera_mount_profile": CAMERA_MOUNT_PROFILE,
            "measurement_frame": MEASUREMENT_FRAME,
            "measurement_axis_contract": MEASUREMENT_AXIS_CONTRACT,
            "public_vector_contract": PUBLIC_MEASUREMENT_VECTOR_CONTRACT,
            "measurement_unit": "m",
        }
        for field_name, expected in expected_contracts.items():
            if getattr(self, field_name) != expected:
                raise ValueError(f"{field_name} must be {expected}")
        if self.left_detection.camera != "left" or self.right_detection.camera != "right":
            raise ValueError("attempt detections must preserve left/right camera identity")
        for field_name, value, expected_camera in (
            ("left_attitude", self.left_attitude, "left"),
            ("right_attitude", self.right_attitude, "right"),
            ("left_gps", self.left_gps, "left"),
            ("right_gps", self.right_gps, "right"),
        ):
            if value is not None and value.camera != expected_camera:
                raise ValueError(f"{field_name} must preserve {expected_camera} camera identity")
        if self.triangulation_source == "checkerboard":
            if self.baseline is not None and not self.baseline.source.startswith("checkerboard"):
                raise ValueError("checkerboard attempts must use checkerboard baseline provenance")
        if self.triangulation_source == "pixhawk" and self.baseline is not None:
            if not self.baseline.source.startswith("gps-rtk"):
                raise ValueError("Pixhawk attempts must use GPS/RTK baseline provenance")
            basis = self.baseline.left_basis_quaternion_wxyz
            if self.baseline.translation_right_in_left_rdf_m is not None:
                if self.left_attitude is None or basis != self.left_attitude.quaternion_wxyz:
                    raise ValueError("baseline translation and right-ray target must use the same selected left quaternion")
                if self.baseline.left_basis_attitude_monotonic_s != self.left_attitude.timestamp_s:
                    raise ValueError("baseline translation and right-ray target must use the same selected left attitude timestamp")

    @property
    def left(self) -> DetectionSnapshot:
        return self.left_detection

    @property
    def right(self) -> DetectionSnapshot:
        return self.right_detection

    @property
    def baseline_m(self) -> float:
        return 0.0 if self.baseline is None else self.baseline.baseline_m

    @property
    def baseline_source(self) -> str:
        if self.baseline is not None:
            return self.baseline.source
        return "checkerboard-waiting" if self.triangulation_source == "checkerboard" else "gps-rtk-waiting"

    @property
    def translation_right_in_left_m(self) -> tuple[float, float, float] | None:
        return None if self.baseline is None else self.baseline.translation_right_in_left_rdf_m

    @property
    def pair_sequence(self) -> int:
        return self.attempt_id

    @property
    def pair_id(self) -> str:
        return f"{self.attempt_session_id}:{self.attempt_id:020d}"

    @property
    def missing_geometry_inputs(self) -> tuple[str, ...]:
        missing: list[str] = []
        if self.intrinsics is None:
            missing.append("intrinsics")
        if self.triangulation_source == "checkerboard":
            extrinsics = None if self.intrinsics is None else self.intrinsics.checkerboard_extrinsics
            if extrinsics is None:
                missing.extend(("checkerboard_rotation", "checkerboard_translation"))
            return tuple(missing)
        if self.left_attitude is None:
            missing.append("left_attitude")
        if self.right_attitude is None:
            missing.append("right_attitude")
        if self.left_gps is None:
            missing.append("left_gps")
        if self.right_gps is None:
            missing.append("right_gps")
        if self.baseline is None or self.baseline.ned_right_from_left_m is None:
            missing.append("ned_baseline")
        if self.translation_right_in_left_m is None:
            missing.append("translation_right_in_left_m")
        return tuple(missing)

    @property
    def is_geometry_ready(self) -> bool:
        return not self.missing_geometry_inputs

    def stereo_calibration(self) -> StereoCalibration:
        return self.geometry_config.to_stereo_calibration(baseline_m=self.baseline_m)

    def canonical_payload(self) -> dict[str, Any]:
        return asdict(self)

    def canonical_sha256(self) -> str:
        return _sha256_payload(self.canonical_payload())

    def content_canonical_payload(self) -> dict[str, Any]:
        payload = asdict(self)
        for field_name in (
            "attempt_id",
            "attempt_session_id",
            "created_monotonic_s",
            "created_wall_s",
        ):
            payload.pop(field_name)
        if payload["intrinsics"] is not None:
            payload["intrinsics"].pop("selected_profile_path")
        return payload

    def content_sha256(self) -> str:
        return _sha256_payload(self.content_canonical_payload())

    def instance_canonical_payload(self) -> dict[str, Any]:
        return {
            "measurement_attempt_content_sha256": self.content_sha256(),
            "instance": asdict(self),
        }

    def instance_sha256(self) -> str:
        return _sha256_payload(self.instance_canonical_payload())


def detection_pair_key(
    *,
    triangulation_source: str,
    timestamp_basis: str,
    left: Any,
    right: Any,
    left_pairing_timestamp_s: float,
    right_pairing_timestamp_s: float,
) -> tuple[MetadataScalar, ...]:
    """Return an identity stronger than the legacy (left_ts, right_ts, method) key."""

    return (
        str(triangulation_source),
        str(timestamp_basis),
        _finite_float(left.timestamp_s, name="left timestamp"),
        _finite_float(right.timestamp_s, name="right timestamp"),
        _finite_float(left_pairing_timestamp_s, name="left pairing timestamp"),
        _finite_float(right_pairing_timestamp_s, name="right pairing timestamp"),
        *(_finite_float(value, name="left bbox") for value in (left.x1, left.y1, left.x2, left.y2)),
        *(_finite_float(value, name="right bbox") for value in (right.x1, right.y1, right.x2, right.y2)),
        int(left.class_id),
        int(right.class_id),
    )


def build_pixhawk_baseline_snapshot(
    *,
    left_gps: GpsSnapshot,
    right_gps: GpsSnapshot,
    selected_left_quaternion_wxyz: tuple[float, float, float, float] | None,
    selected_right_quaternion_wxyz: tuple[float, float, float, float] | None,
    selected_left_attitude_monotonic_s: float | None,
    created_monotonic_s: float,
    detection_midpoint_monotonic_s: float,
) -> BaselineSnapshot:
    ned = ned_delta_m(left_gps, right_gps)  # type: ignore[arg-type]
    baseline_m = 0.0 if ned is None else math.sqrt(sum(value * value for value in ned))
    warnings: list[str] = []
    if left_gps.fix_type != 6:
        warnings.append(f"left {left_gps.rtk_label}")
    if right_gps.fix_type != 6:
        warnings.append(f"right {right_gps.rtk_label}")
    fix_quality = "DEGRADED" if warnings else "GOOD"
    fixes_usable = (
        left_gps.has_position
        and right_gps.has_position
        and left_gps.fix_type is not None
        and right_gps.fix_type is not None
        and left_gps.fix_type >= 3
        and right_gps.fix_type >= 3
    )
    usable = fixes_usable and baseline_m > 0.0
    if not fixes_usable:
        unusable_reason = "GPS fix < 3D"
    elif baseline_m <= 0.0:
        unusable_reason = "GPS baseline length <= 0"
    else:
        unusable_reason = ""
    translation = None
    basis = None
    if usable and ned is not None and selected_left_quaternion_wxyz is not None:
        basis = tuple(selected_left_quaternion_wxyz)
        right_q = selected_right_quaternion_wxyz or selected_left_quaternion_wxyz
        translation = camera_translation_right_in_left_camera(
            baseline_right_in_left_world_m=ned,
            left_body_to_world_q=selected_left_quaternion_wxyz,
            right_body_to_world_q=right_q,
            scale_to_m=None,
        )
    return BaselineSnapshot(
        source="gps-rtk" if usable else "gps-rtk-blocked",
        baseline_m=baseline_m,
        ned_right_from_left_m=ned,
        translation_right_in_left_rdf_m=translation,
        left_basis_quaternion_wxyz=basis,
        fix_quality=fix_quality,
        warnings=tuple(warnings),
        usable_for_translation=translation is not None,
        unusable_reason=unusable_reason,
        gps_pair_delta_s=abs(left_gps.timestamp_s - right_gps.timestamp_s),
        left_age_at_capture_s=max(0.0, created_monotonic_s - left_gps.timestamp_s),
        right_age_at_capture_s=max(0.0, created_monotonic_s - right_gps.timestamp_s),
        left_age_from_detection_midpoint_s=abs(detection_midpoint_monotonic_s - left_gps.timestamp_s),
        right_age_from_detection_midpoint_s=abs(detection_midpoint_monotonic_s - right_gps.timestamp_s),
        left_basis_attitude_monotonic_s=selected_left_attitude_monotonic_s if translation is not None else None,
    )


def build_checkerboard_baseline_snapshot(
    intrinsics: StereoIntrinsicsSnapshot | None,
) -> BaselineSnapshot | None:
    extrinsics = None if intrinsics is None else intrinsics.checkerboard_extrinsics
    if extrinsics is None:
        return None
    _rotation, translation = invert_left_to_right_camera_transform(
        extrinsics.rotation_left_to_right,
        extrinsics.translation_left_to_right_m,
    )
    baseline_m = math.sqrt(sum(value * value for value in translation))
    usable = baseline_m > 0.0
    return BaselineSnapshot(
        source="checkerboard" if usable else "checkerboard-waiting",
        baseline_m=baseline_m,
        ned_right_from_left_m=None,
        translation_right_in_left_rdf_m=translation if usable else None,
        left_basis_quaternion_wxyz=None,
        fix_quality="NOT_USED",
        warnings=() if usable else ("checkerboard baseline length <= 0",),
        usable_for_translation=usable,
        unusable_reason="" if usable else "checkerboard baseline length <= 0",
        gps_pair_delta_s=None,
        left_age_at_capture_s=None,
        right_age_at_capture_s=None,
        left_age_from_detection_midpoint_s=None,
        right_age_from_detection_midpoint_s=None,
        left_basis_attitude_monotonic_s=None,
        gps_selection_basis="NOT_USED",
        gps_clock_domain="NOT_USED",
    )
