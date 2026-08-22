"""Immutable operating-camera intrinsics and applicability evidence.

This module deliberately keeps two independent decisions separate:

* the operating image must use the exact per-camera ``K,D`` profile; and
* a stereo ``R,t`` estimated at another zoom may still be selected.

Pixels and intrinsics use OpenCV optical RDF (right, down, forward).  Zoom,
serial, stabilization, and image-size checks only establish whether a saved
``K,D`` profile applies to the current optical stream.  They do not transform
or reinterpret stereo extrinsics, PX4 body FRD, or world NED values.
"""

from __future__ import annotations

import hashlib
import json
import math
from dataclasses import dataclass
from typing import Any, Mapping


OPERATING_INTRINSICS_PROFILE_SCHEMA = "operating-intrinsics-profile-v1"
OPERATING_INTRINSICS_MATH_SCHEMA = "operating-intrinsics-math-v1"
OPERATING_INTRINSICS_APPLICABILITY_SCHEMA = "operating-intrinsics-applicability-v1"
OPENCV_OPTICAL_FRAME = "opencv_optical:RDF(right,down,forward)"
NOMINAL_INTRINSICS_QUALITY = "NOMINAL_UNVERIFIED"


def _finite_float(value: object, *, name: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def _optional_finite_float(value: object | None, *, name: str) -> float | None:
    return None if value is None else _finite_float(value, name=name)


def _optional_nonnegative_int(value: object | None, *, name: str) -> int | None:
    if value is None:
        return None
    result = int(value)
    if result < 0:
        raise ValueError(f"{name} must be non-negative")
    return result


def _optional_text(value: object | None) -> str | None:
    if value is None:
        return None
    result = str(value).strip()
    return result or None


def _canonical_json_value(value: Any) -> Any:
    """Return a JSON-compatible, deterministic copy and reject NaN/Inf."""

    if value is None or isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, float):
        return _finite_float(value, name="JSON number")
    if isinstance(value, Mapping):
        return {
            str(key): _canonical_json_value(item)
            for key, item in sorted(value.items(), key=lambda pair: str(pair[0]))
        }
    if isinstance(value, (tuple, list)):
        return [_canonical_json_value(item) for item in value]
    raise TypeError(f"unsupported provenance value: {type(value).__name__}")


def _canonical_json_text(value: Any) -> str:
    return json.dumps(
        _canonical_json_value(value),
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=False,
        allow_nan=False,
    )


def _sha256_payload(value: Any) -> str:
    return hashlib.sha256(_canonical_json_text(value).encode("utf-8")).hexdigest()


def _validate_optional_sha256(value: str, *, name: str) -> None:
    if value and (
        len(value) != 64
        or any(character not in "0123456789abcdef" for character in value)
    ):
        raise ValueError(f"{name} must be a lowercase SHA-256 digest")


@dataclass(frozen=True)
class OperatingCameraIntrinsics:
    """One camera's exact operating ``K,D`` and optical-state provenance.

    Tuples and scalar strings are used throughout so a frozen instance cannot
    retain a mutable list or mapping supplied by its caller.
    """

    side: str
    width_px: int
    height_px: int
    camera_matrix: tuple[
        tuple[float, float, float],
        tuple[float, float, float],
        tuple[float, float, float],
    ]
    distortion_coefficients: tuple[float, ...]
    source_zoom_index: int | None
    source_zoom_ratio: float | None = None
    camera_serial: str | None = None
    stabilization_state: str | None = None
    optical_channel: str = "visible_oz"
    distortion_model: str = "opencv_pinhole"
    camera_frame: str = OPENCV_OPTICAL_FRAME

    def __post_init__(self) -> None:
        side = str(self.side).strip().lower()
        if side not in {"left", "right"}:
            raise ValueError("side must be left or right")
        object.__setattr__(self, "side", side)

        width_px = int(self.width_px)
        height_px = int(self.height_px)
        if width_px <= 0 or height_px <= 0:
            raise ValueError("intrinsics image dimensions must be positive")
        object.__setattr__(self, "width_px", width_px)
        object.__setattr__(self, "height_px", height_px)

        matrix = tuple(
            tuple(_finite_float(value, name="camera_matrix") for value in row)
            for row in self.camera_matrix
        )
        if len(matrix) != 3 or any(len(row) != 3 for row in matrix):
            raise ValueError("camera_matrix must be 3x3")
        if matrix[0][0] <= 0.0 or matrix[1][1] <= 0.0:
            raise ValueError("camera_matrix fx and fy must be positive")
        if any(
            abs(matrix[2][index] - expected) > 1.0e-9
            for index, expected in enumerate((0.0, 0.0, 1.0))
        ):
            raise ValueError("camera_matrix last row must be [0, 0, 1]")
        object.__setattr__(self, "camera_matrix", matrix)

        distortion = tuple(
            _finite_float(value, name="distortion_coefficients")
            for value in self.distortion_coefficients
        )
        if not distortion:
            raise ValueError("distortion_coefficients must not be empty")
        object.__setattr__(self, "distortion_coefficients", distortion)

        source_zoom_index = _optional_nonnegative_int(
            self.source_zoom_index,
            name="source_zoom_index",
        )
        source_zoom_ratio = _optional_finite_float(
            self.source_zoom_ratio,
            name="source_zoom_ratio",
        )
        if source_zoom_ratio is not None and source_zoom_ratio <= 0.0:
            raise ValueError("source_zoom_ratio must be positive")
        object.__setattr__(self, "source_zoom_index", source_zoom_index)
        object.__setattr__(self, "source_zoom_ratio", source_zoom_ratio)
        object.__setattr__(self, "camera_serial", _optional_text(self.camera_serial))
        object.__setattr__(
            self,
            "stabilization_state",
            _optional_text(self.stabilization_state),
        )

        optical_channel = str(self.optical_channel).strip()
        distortion_model = str(self.distortion_model).strip()
        if not optical_channel or not distortion_model:
            raise ValueError("optical_channel and distortion_model must not be empty")
        object.__setattr__(self, "optical_channel", optical_channel)
        object.__setattr__(self, "distortion_model", distortion_model)
        if str(self.camera_frame) != OPENCV_OPTICAL_FRAME:
            raise ValueError("camera_frame must preserve the OpenCV optical RDF contract")

    @classmethod
    def from_camera_intrinsics(
        cls,
        side: str,
        intrinsics: Any,
        *,
        source_zoom_index: int | None,
        source_zoom_ratio: float | None = None,
        camera_serial: str | None = None,
        stabilization_state: str | None = None,
        optical_channel: str = "visible_oz",
        distortion_model: str = "opencv_pinhole",
    ) -> "OperatingCameraIntrinsics":
        return cls(
            side=side,
            width_px=int(intrinsics.width),
            height_px=int(intrinsics.height),
            camera_matrix=tuple(tuple(row) for row in intrinsics.camera_matrix),
            distortion_coefficients=tuple(intrinsics.distortion_coefficients),
            source_zoom_index=source_zoom_index,
            source_zoom_ratio=source_zoom_ratio,
            camera_serial=camera_serial,
            stabilization_state=stabilization_state,
            optical_channel=optical_channel,
            distortion_model=distortion_model,
        )

    def to_mapping(self) -> dict[str, Any]:
        return {
            "side": self.side,
            "width_px": self.width_px,
            "height_px": self.height_px,
            "camera_matrix": [list(row) for row in self.camera_matrix],
            "distortion_coefficients": list(self.distortion_coefficients),
            "source_zoom_index": self.source_zoom_index,
            "source_zoom_ratio": self.source_zoom_ratio,
            "camera_serial": self.camera_serial,
            "stabilization_state": self.stabilization_state,
            "optical_channel": self.optical_channel,
            "distortion_model": self.distortion_model,
            "camera_frame": self.camera_frame,
        }

    @classmethod
    def from_mapping(cls, value: Mapping[str, Any]) -> "OperatingCameraIntrinsics":
        return cls(
            side=str(value["side"]),
            width_px=int(value["width_px"]),
            height_px=int(value["height_px"]),
            camera_matrix=tuple(tuple(row) for row in value["camera_matrix"]),
            distortion_coefficients=tuple(value["distortion_coefficients"]),
            source_zoom_index=value.get("source_zoom_index"),
            source_zoom_ratio=value.get("source_zoom_ratio"),
            camera_serial=value.get("camera_serial"),
            stabilization_state=value.get("stabilization_state"),
            optical_channel=str(value.get("optical_channel") or "visible_oz"),
            distortion_model=str(value.get("distortion_model") or "opencv_pinhole"),
            camera_frame=str(value.get("camera_frame") or OPENCV_OPTICAL_FRAME),
        )


@dataclass(frozen=True)
class OperatingIntrinsicsProfile:
    """Deeply immutable left/right operating intrinsics profile."""

    left: OperatingCameraIntrinsics
    right: OperatingCameraIntrinsics
    profile_id: str = ""
    source_path: str = ""
    source_content_sha256: str = ""
    provenance_json: str = "{}"
    schema_version: str = OPERATING_INTRINSICS_PROFILE_SCHEMA

    def __post_init__(self) -> None:
        if not isinstance(self.left, OperatingCameraIntrinsics) or not isinstance(
            self.right,
            OperatingCameraIntrinsics,
        ):
            raise TypeError("left and right must be OperatingCameraIntrinsics")
        if self.left.side != "left" or self.right.side != "right":
            raise ValueError("profile must preserve left/right camera identity")
        if str(self.schema_version) != OPERATING_INTRINSICS_PROFILE_SCHEMA:
            raise ValueError(
                f"schema_version must be {OPERATING_INTRINSICS_PROFILE_SCHEMA}"
            )
        object.__setattr__(self, "profile_id", str(self.profile_id).strip())
        object.__setattr__(self, "source_path", str(self.source_path))
        source_digest = str(self.source_content_sha256)
        _validate_optional_sha256(source_digest, name="source_content_sha256")
        object.__setattr__(self, "source_content_sha256", source_digest)

        try:
            provenance = json.loads(str(self.provenance_json or "{}"))
        except (TypeError, ValueError) as exc:
            raise ValueError(f"provenance_json must contain valid JSON: {exc}") from exc
        if not isinstance(provenance, dict):
            raise ValueError("provenance_json must contain a JSON object")
        object.__setattr__(self, "provenance_json", _canonical_json_text(provenance))

    @classmethod
    def from_stereo_result(
        cls,
        result: Any,
        *,
        source_zoom_index: int | None,
        source_zoom_ratio: float | None = None,
        left_camera_serial: str | None = None,
        right_camera_serial: str | None = None,
        left_stabilization_state: str | None = None,
        right_stabilization_state: str | None = None,
        profile_id: str = "",
        source_path: str = "",
        source_content_sha256: str = "",
        provenance: Mapping[str, Any] | None = None,
    ) -> "OperatingIntrinsicsProfile":
        """Copy a legacy stereo result without inventing missing provenance.

        In particular, ``source_zoom_index=None`` remains unknown.  Callers
        must not substitute a UI-selected or requested zoom for absent legacy
        metadata.
        """

        return cls(
            left=OperatingCameraIntrinsics.from_camera_intrinsics(
                "left",
                result.left,
                source_zoom_index=source_zoom_index,
                source_zoom_ratio=source_zoom_ratio,
                camera_serial=left_camera_serial,
                stabilization_state=left_stabilization_state,
            ),
            right=OperatingCameraIntrinsics.from_camera_intrinsics(
                "right",
                result.right,
                source_zoom_index=source_zoom_index,
                source_zoom_ratio=source_zoom_ratio,
                camera_serial=right_camera_serial,
                stabilization_state=right_stabilization_state,
            ),
            profile_id=profile_id,
            source_path=source_path,
            source_content_sha256=source_content_sha256,
            provenance_json=_canonical_json_text(dict(provenance or {})),
        )

    @property
    def provenance(self) -> dict[str, Any]:
        """Return a detached copy; mutations cannot change this profile."""

        return json.loads(self.provenance_json)

    @property
    def intrinsics_math_sha256(self) -> str:
        return _sha256_payload(
            {
                "schema_version": OPERATING_INTRINSICS_MATH_SCHEMA,
                "left": self._math_mapping(self.left),
                "right": self._math_mapping(self.right),
            }
        )

    @property
    def profile_sha256(self) -> str:
        return _sha256_payload(self._profile_payload())

    @staticmethod
    def _math_mapping(camera: OperatingCameraIntrinsics) -> dict[str, Any]:
        return {
            "side": camera.side,
            "width_px": camera.width_px,
            "height_px": camera.height_px,
            "camera_matrix": [list(row) for row in camera.camera_matrix],
            "distortion_coefficients": list(camera.distortion_coefficients),
            "optical_channel": camera.optical_channel,
            "distortion_model": camera.distortion_model,
            "camera_frame": camera.camera_frame,
        }

    def _profile_payload(self) -> dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "profile_id": self.profile_id,
            "source_path": self.source_path,
            "source_content_sha256": self.source_content_sha256,
            "provenance": self.provenance,
            "left": self.left.to_mapping(),
            "right": self.right.to_mapping(),
            "intrinsics_math_sha256": self.intrinsics_math_sha256,
        }

    def to_mapping(self) -> dict[str, Any]:
        payload = self._profile_payload()
        payload["profile_sha256"] = self.profile_sha256
        return payload

    @classmethod
    def from_mapping(cls, value: Mapping[str, Any]) -> "OperatingIntrinsicsProfile":
        profile = cls(
            left=OperatingCameraIntrinsics.from_mapping(value["left"]),
            right=OperatingCameraIntrinsics.from_mapping(value["right"]),
            profile_id=str(value.get("profile_id") or ""),
            source_path=str(value.get("source_path") or ""),
            source_content_sha256=str(value.get("source_content_sha256") or ""),
            provenance_json=_canonical_json_text(value.get("provenance") or {}),
            schema_version=str(value.get("schema_version") or ""),
        )
        expected_math = str(value.get("intrinsics_math_sha256") or "")
        expected_profile = str(value.get("profile_sha256") or "")
        if expected_math and expected_math != profile.intrinsics_math_sha256:
            raise ValueError("intrinsics_math_sha256 does not match profile payload")
        if expected_profile and expected_profile != profile.profile_sha256:
            raise ValueError("profile_sha256 does not match profile payload")
        return profile


def build_nominal_operating_intrinsics_profile(
    *,
    zoom_index: int,
    left_width_px: int,
    left_height_px: int,
    right_width_px: int,
    right_height_px: int,
    left_zoom_ratio: float | None = None,
    right_zoom_ratio: float | None = None,
) -> OperatingIntrinsicsProfile:
    """Build an explicitly uncalibrated, zero-distortion pinhole profile.

    The WIRIS nominal focal table is defined on a 1280x720 visible raster.
    Horizontal and vertical focal lengths are scaled independently to retain
    native OpenCV pixel units.  This is runtime geometry evidence, never a
    camera-calibration PASS claim.
    """

    from .wiris_zoom import nominal_focal_length_px

    zoom = int(zoom_index)
    nominal_fx = nominal_focal_length_px(zoom)
    if nominal_fx is None:
        raise ValueError(f"no nominal focal estimate for zoom index {zoom}")

    def camera(side: str, width: int, height: int, ratio: float | None) -> OperatingCameraIntrinsics:
        width = int(width)
        height = int(height)
        if width <= 0 or height <= 0:
            raise ValueError("nominal intrinsics require positive native raster dimensions")
        fx = float(nominal_fx) * float(width) / 1280.0
        fy = float(nominal_fx) * float(height) / 720.0
        return OperatingCameraIntrinsics(
            side=side,
            width_px=width,
            height_px=height,
            camera_matrix=(
                (fx, 0.0, (float(width) - 1.0) / 2.0),
                (0.0, fy, (float(height) - 1.0) / 2.0),
                (0.0, 0.0, 1.0),
            ),
            distortion_coefficients=(0.0, 0.0, 0.0, 0.0, 0.0),
            source_zoom_index=zoom,
            source_zoom_ratio=ratio,
            distortion_model="opencv_pinhole_zero_distortion_assumed",
        )

    profile = OperatingIntrinsicsProfile(
        left=camera("left", left_width_px, left_height_px, left_zoom_ratio),
        right=camera("right", right_width_px, right_height_px, right_zoom_ratio),
        profile_id=f"wiris-nominal-zoom-{zoom:02d}",
        source_path="generated:wiris_zoom_nominal_pinhole",
        provenance_json=_canonical_json_text(
            {
                "source": "wiris_zoom_nominal_pinhole",
                "quality": NOMINAL_INTRINSICS_QUALITY,
                "precision_qualified": False,
                "runtime_truth_used": False,
                "reference_raster_px": [1280, 720],
                "reference_fx_px": nominal_fx,
                "scaling": "fx=reference_fx*width/1280;fy=reference_fx*height/720",
                "principal_point": "((width-1)/2,(height-1)/2)",
                "distortion_assumption": "zero",
            }
        ),
    )
    return profile


def nominal_profile_to_stereo_result(profile: OperatingIntrinsicsProfile) -> Any:
    """Adapt one nominal operating profile to the legacy triangulation value."""

    from .calibration import CameraIntrinsics, CheckerboardConfig, StereoIntrinsicsResult

    def camera(value: OperatingCameraIntrinsics) -> CameraIntrinsics:
        return CameraIntrinsics(
            width=value.width_px,
            height=value.height_px,
            camera_matrix=[list(row) for row in value.camera_matrix],
            distortion_coefficients=list(value.distortion_coefficients),
            reprojection_error_px=float("nan"),
            image_count=0,
        )

    return StereoIntrinsicsResult(
        left=camera(profile.left),
        right=camera(profile.right),
        board=CheckerboardConfig(inner_cols=2, inner_rows=2, square_size_m=1.0),
        extrinsics=None,
        calibration_quality={
            "state": NOMINAL_INTRINSICS_QUALITY,
            "precision_qualified": False,
            "profile_sha256": profile.profile_sha256,
        },
    )

@dataclass(frozen=True)
class OperatingCameraEvidence:
    """Observed optical state for one live camera stream."""

    side: str
    zoom_index: int | None
    zoom_ratio: float | None
    zoom_updated_monotonic_s: float | None
    frame_width_px: int | None
    frame_height_px: int | None
    camera_serial: str | None
    stabilization_state: str | None

    def __post_init__(self) -> None:
        side = str(self.side).strip().lower()
        if side not in {"left", "right"}:
            raise ValueError("side must be left or right")
        object.__setattr__(self, "side", side)
        object.__setattr__(
            self,
            "zoom_index",
            _optional_nonnegative_int(self.zoom_index, name="zoom_index"),
        )
        ratio = _optional_finite_float(self.zoom_ratio, name="zoom_ratio")
        if ratio is not None and ratio <= 0.0:
            raise ValueError("zoom_ratio must be positive")
        object.__setattr__(self, "zoom_ratio", ratio)
        object.__setattr__(
            self,
            "zoom_updated_monotonic_s",
            _optional_finite_float(
                self.zoom_updated_monotonic_s,
                name="zoom_updated_monotonic_s",
            ),
        )
        for field_name in ("frame_width_px", "frame_height_px"):
            value = getattr(self, field_name)
            if value is not None:
                normalized = int(value)
                if normalized <= 0:
                    raise ValueError(f"{field_name} must be positive")
                object.__setattr__(self, field_name, normalized)
        object.__setattr__(self, "camera_serial", _optional_text(self.camera_serial))
        object.__setattr__(
            self,
            "stabilization_state",
            _optional_text(self.stabilization_state),
        )


@dataclass(frozen=True)
class IntrinsicsApplicability:
    """Decision record for geometry while preserving unrestricted raw capture."""

    state: str
    reasons: tuple[str, ...]
    geometry_allowed: bool
    raw_recording_allowed: bool
    operating_zoom_index: int | None
    extrinsics_source_zoom_index: int | None
    cross_zoom: bool | None
    left_zoom_age_s: float | None
    right_zoom_age_s: float | None
    intrinsics_math_sha256: str
    profile_sha256: str
    schema_version: str = OPERATING_INTRINSICS_APPLICABILITY_SCHEMA

    def __post_init__(self) -> None:
        state = str(self.state).upper()
        if state not in {"READY", "BLOCKED"}:
            raise ValueError("state must be READY or BLOCKED")
        object.__setattr__(self, "state", state)
        object.__setattr__(self, "reasons", tuple(str(reason) for reason in self.reasons))
        object.__setattr__(self, "geometry_allowed", bool(self.geometry_allowed))
        object.__setattr__(self, "raw_recording_allowed", bool(self.raw_recording_allowed))
        if self.raw_recording_allowed is not True:
            raise ValueError("raw_recording_allowed must remain true")
        if self.geometry_allowed != (state == "READY"):
            raise ValueError("geometry_allowed must agree with state")
        if self.geometry_allowed != (not self.reasons):
            raise ValueError("geometry_allowed must agree with reasons")
        object.__setattr__(
            self,
            "operating_zoom_index",
            _optional_nonnegative_int(
                self.operating_zoom_index,
                name="operating_zoom_index",
            ),
        )
        object.__setattr__(
            self,
            "extrinsics_source_zoom_index",
            _optional_nonnegative_int(
                self.extrinsics_source_zoom_index,
                name="extrinsics_source_zoom_index",
            ),
        )
        for field_name in ("left_zoom_age_s", "right_zoom_age_s"):
            value = getattr(self, field_name)
            if value is not None:
                normalized = _finite_float(value, name=field_name)
                if normalized < 0.0:
                    raise ValueError(f"{field_name} must be non-negative")
                object.__setattr__(self, field_name, normalized)
        for field_name in ("intrinsics_math_sha256", "profile_sha256"):
            digest = str(getattr(self, field_name))
            _validate_optional_sha256(digest, name=field_name)
            object.__setattr__(self, field_name, digest)
        if str(self.schema_version) != OPERATING_INTRINSICS_APPLICABILITY_SCHEMA:
            raise ValueError("unexpected applicability schema_version")

    def to_mapping(self) -> dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "state": self.state,
            "reasons": list(self.reasons),
            "geometry_allowed": self.geometry_allowed,
            "raw_recording_allowed": self.raw_recording_allowed,
            "operating_zoom_index": self.operating_zoom_index,
            "extrinsics_source_zoom_index": self.extrinsics_source_zoom_index,
            "cross_zoom": self.cross_zoom,
            "zoom_age_s": {
                "left": self.left_zoom_age_s,
                "right": self.right_zoom_age_s,
            },
            "intrinsics_math_sha256": self.intrinsics_math_sha256,
            "profile_sha256": self.profile_sha256,
        }


def _normalized_state(value: str | None) -> str | None:
    normalized = _optional_text(value)
    return None if normalized is None else normalized.casefold()


def evaluate_intrinsics_applicability(
    profile: OperatingIntrinsicsProfile | None,
    left_evidence: OperatingCameraEvidence,
    right_evidence: OperatingCameraEvidence,
    *,
    now_monotonic_s: float,
    max_zoom_age_s: float = 5.0,
    extrinsics_source_zoom_index: int | None = None,
) -> IntrinsicsApplicability:
    """Check exact operating ``K,D`` evidence without gating cross-zoom R,t.

    Missing legacy zoom/serial/stabilization provenance remains unknown and
    blocks derived geometry.  Raw recording remains allowed for every result.
    ``extrinsics_source_zoom_index`` is reported only; a 3x R,t with 12x K,D
    is intentionally just as applicable as a same-zoom 7x/7x selection.
    """

    if left_evidence.side != "left" or right_evidence.side != "right":
        raise ValueError("evidence must preserve left/right camera identity")
    now = _finite_float(now_monotonic_s, name="now_monotonic_s")
    maximum_age = _finite_float(max_zoom_age_s, name="max_zoom_age_s")
    if maximum_age < 0.0:
        raise ValueError("max_zoom_age_s must be non-negative")
    extrinsics_zoom = _optional_nonnegative_int(
        extrinsics_source_zoom_index,
        name="extrinsics_source_zoom_index",
    )

    reasons: list[str] = []
    ages: dict[str, float | None] = {"left": None, "right": None}
    if profile is None:
        reasons.append("operating_intrinsics_profile_missing")
    else:
        nominal_profile = profile.provenance.get("quality") == NOMINAL_INTRINSICS_QUALITY
        for side, expected, observed in (
            ("left", profile.left, left_evidence),
            ("right", profile.right, right_evidence),
        ):
            if expected.source_zoom_index is None:
                reasons.append(f"{side}_intrinsics_zoom_unknown")
            if observed.zoom_index is None:
                reasons.append(f"{side}_zoom_readback_unknown")
            if observed.zoom_updated_monotonic_s is None:
                reasons.append(f"{side}_zoom_readback_age_unknown")
            else:
                age = now - observed.zoom_updated_monotonic_s
                if age < 0.0:
                    reasons.append(f"{side}_zoom_readback_from_future")
                else:
                    ages[side] = age
                    if age > maximum_age:
                        reasons.append(f"{side}_zoom_readback_stale")
            if (
                expected.source_zoom_index is not None
                and observed.zoom_index is not None
                and expected.source_zoom_index != observed.zoom_index
            ):
                reasons.append(f"{side}_operating_zoom_mismatch")

            if observed.frame_width_px is None or observed.frame_height_px is None:
                reasons.append(f"{side}_frame_resolution_unknown")
            elif (
                observed.frame_width_px != expected.width_px
                or observed.frame_height_px != expected.height_px
            ):
                reasons.append(f"{side}_frame_resolution_mismatch")

            if nominal_profile:
                continue
            if expected.camera_serial is None:
                reasons.append(f"{side}_profile_serial_unknown")
            elif observed.camera_serial is None:
                reasons.append(f"{side}_camera_serial_unknown")
            elif expected.camera_serial != observed.camera_serial:
                reasons.append(f"{side}_camera_serial_mismatch")

            expected_stabilization = _normalized_state(expected.stabilization_state)
            observed_stabilization = _normalized_state(observed.stabilization_state)
            if expected_stabilization is None:
                reasons.append(f"{side}_profile_stabilization_unknown")
            elif observed_stabilization is None:
                reasons.append(f"{side}_camera_stabilization_unknown")
            elif expected_stabilization != observed_stabilization:
                reasons.append(f"{side}_camera_stabilization_mismatch")

    observed_zoom = None
    if (
        left_evidence.zoom_index is not None
        and right_evidence.zoom_index is not None
    ):
        if left_evidence.zoom_index != right_evidence.zoom_index:
            reasons.append("left_right_zoom_disagree")
        else:
            observed_zoom = left_evidence.zoom_index

    # De-duplicate without losing the stable diagnostic order.
    reason_tuple = tuple(dict.fromkeys(reasons))
    cross_zoom = (
        None
        if observed_zoom is None or extrinsics_zoom is None
        else observed_zoom != extrinsics_zoom
    )
    return IntrinsicsApplicability(
        state="READY" if not reason_tuple else "BLOCKED",
        reasons=reason_tuple,
        geometry_allowed=not reason_tuple,
        raw_recording_allowed=True,
        operating_zoom_index=observed_zoom,
        extrinsics_source_zoom_index=extrinsics_zoom,
        cross_zoom=cross_zoom,
        left_zoom_age_s=ages["left"],
        right_zoom_age_s=ages["right"],
        intrinsics_math_sha256=("" if profile is None else profile.intrinsics_math_sha256),
        profile_sha256=("" if profile is None else profile.profile_sha256),
    )
