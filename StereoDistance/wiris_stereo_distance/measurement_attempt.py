from __future__ import annotations

import hashlib
import json
import math
from dataclasses import asdict, dataclass
from typing import Any, Iterable

from wiris_stereo_distance.coordinate_contract import (
    BODY_FRAME,
    CAMERA_FRAME,
    CAMERA_MOUNT_PROFILE,
    MEASUREMENT_AXIS_CONTRACT,
    MEASUREMENT_FRAME,
    PUBLIC_MEASUREMENT_VECTOR_CONTRACT,
    WORLD_FRAME,
)
from wiris_stereo_distance.rtk_geometry import ned_delta_m
from wiris_stereo_distance.stereo import StereoCalibration
from wiris_stereo_distance.triangulation import (
    camera_translation_right_in_left_camera,
    invert_left_to_right_camera_transform,
)


MetadataScalar = str | int | float | bool | None

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
    "rtcp_timing_invalid_reason",
)


def _finite_float(value: Any, *, name: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed):
        raise ValueError(f"{name} must be finite")
    return parsed


def _optional_finite_float(value: Any, *, name: str) -> float | None:
    return None if value is None else _finite_float(value, name=name)


def _finite_tuple(values: Iterable[Any], *, name: str, length: int | None = None) -> tuple[float, ...]:
    parsed = tuple(_finite_float(value, name=name) for value in values)
    if length is not None and len(parsed) != length:
        raise ValueError(f"{name} must contain {length} values")
    return parsed


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


def _metadata_items(metadata: Any) -> tuple[tuple[str, MetadataScalar], ...]:
    source = dict(metadata or {})
    items: list[tuple[str, MetadataScalar]] = []
    for key in DETECTION_METADATA_ALLOWLIST:
        if key not in source:
            continue
        value = source[key]
        if value is None or isinstance(value, (str, bool, int)):
            items.append((key, value))
            continue
        if isinstance(value, float):
            items.append((key, _finite_float(value, name=f"timestamp_metadata.{key}")))
            continue
        raise ValueError(f"timestamp_metadata.{key} must be a scalar")
    return tuple(items)


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
            tuple((str(key), value) for key, value in self.timestamp_metadata_items),
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
        for key, value in self.timestamp_metadata_items:
            if key not in DETECTION_METADATA_ALLOWLIST:
                raise ValueError(f"timestamp metadata key is not allowed: {key}")
            if isinstance(value, float) and not math.isfinite(value):
                raise ValueError(f"timestamp_metadata.{key} must be finite")

    @classmethod
    def from_observation(
        cls,
        observation: Any,
        *,
        pairing_timestamp_basis: str,
        pairing_timestamp_s: float,
    ) -> "DetectionSnapshot":
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
    quaternion_order: str = "wxyz"
    orientation_from_frame: str = BODY_FRAME
    orientation_to_frame: str = WORLD_FRAME

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
        object.__setattr__(self, "quaternion_order", str(self.quaternion_order))
        object.__setattr__(self, "orientation_from_frame", str(self.orientation_from_frame))
        object.__setattr__(self, "orientation_to_frame", str(self.orientation_to_frame))
        _finite_float(self.app_receive_monotonic_s, name="attitude timestamp")
        q = _finite_tuple(self.quaternion_wxyz, name="quaternion_wxyz", length=4)
        if math.sqrt(sum(value * value for value in q)) <= 0.0:
            raise ValueError("quaternion_wxyz must have non-zero length")
        _optional_finite_float(self.px4_timestamp_s, name="px4_timestamp_s")
        for name, value in (("roll_deg", self.roll_deg), ("pitch_deg", self.pitch_deg), ("yaw_deg", self.yaw_deg)):
            _finite_float(value, name=name)
        if self.quaternion_order != "wxyz":
            raise ValueError("quaternion order must be wxyz")

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
    position_frame: str = "wgs84_geodetic"
    angle_unit: str = "deg"
    altitude_unit: str = "m"

    def __post_init__(self) -> None:
        object.__setattr__(self, "camera", str(self.camera))
        object.__setattr__(self, "app_receive_monotonic_s", _finite_float(self.app_receive_monotonic_s, name="GPS timestamp"))
        for name in ("latitude_deg", "longitude_deg", "altitude_m", "eph_m", "epv_m", "px4_timestamp_s"):
            object.__setattr__(self, name, _optional_finite_float(getattr(self, name), name=name))
        object.__setattr__(self, "fix_type", None if self.fix_type is None else int(self.fix_type))
        object.__setattr__(self, "satellites_used", None if self.satellites_used is None else int(self.satellites_used))
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
        ):
            _optional_finite_float(value, name=name)

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

    def __post_init__(self) -> None:
        object.__setattr__(self, "active_zoom_level", int(self.active_zoom_level))
        object.__setattr__(self, "selected_profile_path", str(self.selected_profile_path))
        object.__setattr__(self, "content_sha256", str(self.content_sha256))
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

    @classmethod
    def from_result(
        cls,
        result: Any,
        *,
        active_zoom_level: int,
        selected_profile_path: str,
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
    translation_frame: str = MEASUREMENT_FRAME
    unit: str = "m"

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
        if not isinstance(self.left_detection, DetectionSnapshot) or not isinstance(
            self.right_detection, DetectionSnapshot
        ):
            raise TypeError("detections must be immutable DetectionSnapshot values")
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
        if self.triangulation_source == "checkerboard":
            if any(value is not None for value in (self.left_attitude, self.right_attitude, self.left_gps, self.right_gps)):
                raise ValueError("checkerboard attempts must not consume Pixhawk attitude or GPS")
        if self.triangulation_source == "pixhawk" and self.baseline is not None:
            basis = self.baseline.left_basis_quaternion_wxyz
            if self.baseline.translation_right_in_left_rdf_m is not None:
                if self.left_attitude is None or basis != self.left_attitude.quaternion_wxyz:
                    raise ValueError("baseline translation and right-ray target must use the same selected left quaternion")

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

    def stereo_calibration(self) -> StereoCalibration:
        return self.geometry_config.to_stereo_calibration(baseline_m=self.baseline_m)

    def canonical_payload(self) -> dict[str, Any]:
        return asdict(self)

    def canonical_sha256(self) -> str:
        return _sha256_payload(self.canonical_payload())


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
    )
