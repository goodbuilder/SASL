"""High-resolution (HR) evidence and per-camera HR-to-OZ bridge models.

The visible HR still channel and the operating-zoom (OZ) video channel are
different optical frames.  This module never treats their pixels or camera
coordinates as interchangeable.  For each physical left/right camera the
bridge contract is explicitly::

    X_OZ = R_OZ_from_HR @ X_HR + T_OZ_from_HR_m

Both frames use OpenCV optical RDF axes ``[Right, Down, Forward]``.  A full
SE(3) bridge estimates rotation and translation in metres.  A rotation-only
bridge stores a zero translation solely as a serialization sentinel: its
translation is *not observable* and must not be used to transfer a stereo
baseline.

HR image imports retain the exact immutable file bytes.  Their JSON evidence
contains only byte count and SHA-256, so callers can persist the exact bytes as
a separate artifact without re-encoding the image.
"""

from __future__ import annotations

import hashlib
import json
import math
import mimetypes
import os
import tempfile
import time
import uuid
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

from .optical_pose import (
    SE3Transform,
    StereoPoseRdf,
    transfer_stereo_pose_through_bridges,
)


HR_CAPTURE_EVENT_SCHEMA = "workstation-hr-capture-event-v1"
HR_OZ_BRIDGE_PROFILE_SCHEMA = "workstation-hr-oz-bridge-profile-v1"
HR_OZ_COORDINATE_CONTRACT = (
    "opencv_optical:RDF(right,down,forward); "
    "X_OZ = R_OZ_from_HR * X_HR + T_OZ_from_HR_m"
)


class CapturePurpose(str, Enum):
    PRE = "pre"
    POST = "post"
    CHECKPOINT = "checkpoint"
    UNSPECIFIED = "unspecified"


class BridgeObservationModel(str, Enum):
    FULL_SE3 = "full_se3"
    ROTATION_ONLY = "rotation_only"


def _required_text(value: object, *, name: str) -> str:
    text = str(value or "").strip()
    if not text:
        raise ValueError(f"{name} is required")
    return text


def _side(value: object) -> str:
    side = str(value).strip().lower()
    if side not in {"left", "right"}:
        raise ValueError("side must be left or right")
    return side


def _finite(value: object, *, name: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result


def _nonnegative(value: object, *, name: str) -> float:
    result = _finite(value, name=name)
    if result < 0.0:
        raise ValueError(f"{name} must be non-negative")
    return result


def _sha256_bytes(content: bytes) -> str:
    return hashlib.sha256(content).hexdigest()


def _validate_sha256(value: object, *, name: str) -> str:
    digest = str(value).strip()
    if len(digest) != 64 or any(character not in "0123456789abcdef" for character in digest):
        raise ValueError(f"{name} must be a lowercase SHA-256 digest")
    return digest


def _json_safe(value: Any) -> Any:
    if value is None or isinstance(value, (bool, int, str)):
        return value
    if isinstance(value, float):
        return _finite(value, name="JSON number")
    if isinstance(value, Enum):
        return value.value
    if isinstance(value, Mapping):
        return {
            str(key): _json_safe(item)
            for key, item in sorted(value.items(), key=lambda pair: str(pair[0]))
        }
    if isinstance(value, (tuple, list)):
        return [_json_safe(item) for item in value]
    raise TypeError(f"unsupported JSON evidence value: {type(value).__name__}")


def _canonical_json_bytes(value: Mapping[str, Any]) -> bytes:
    return json.dumps(
        _json_safe(value),
        ensure_ascii=False,
        allow_nan=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")


def _mapping_sha256(value: Mapping[str, Any]) -> str:
    return _sha256_bytes(_canonical_json_bytes(value))


def _decode_image_resolution(content: bytes) -> tuple[int, int]:
    """Decode only enough image data to verify width/height.

    OpenCV is imported lazily so evidence models and stored profiles remain
    usable on analysis machines that do not install the optional bridge
    solver dependencies.
    """

    try:
        import cv2
        import numpy as np
    except ImportError as exc:  # pragma: no cover - exercised on minimal deployments
        raise RuntimeError("OpenCV and NumPy are required to inspect an imported HR image") from exc
    encoded = np.frombuffer(content, dtype=np.uint8)
    image = cv2.imdecode(encoded, cv2.IMREAD_UNCHANGED)
    if image is None or image.ndim < 2:
        raise ValueError("HR file bytes do not decode as an image")
    return int(image.shape[1]), int(image.shape[0])


@dataclass(frozen=True)
class HrImportedImage:
    """Exact immutable bytes and provenance for one imported HR still."""

    side: str
    source_name: str
    camera_serial: str
    width_px: int
    height_px: int
    file_bytes: bytes = field(repr=False)
    media_type: str = "application/octet-stream"
    imported_utc_ns: int = 0

    def __post_init__(self) -> None:
        object.__setattr__(self, "side", _side(self.side))
        object.__setattr__(self, "source_name", _required_text(self.source_name, name="source_name"))
        object.__setattr__(
            self,
            "camera_serial",
            _required_text(self.camera_serial, name="camera_serial"),
        )
        width = int(self.width_px)
        height = int(self.height_px)
        if width <= 0 or height <= 0:
            raise ValueError("HR image dimensions must be positive")
        object.__setattr__(self, "width_px", width)
        object.__setattr__(self, "height_px", height)
        if not isinstance(self.file_bytes, bytes) or not self.file_bytes:
            raise ValueError("file_bytes must be non-empty immutable bytes")
        object.__setattr__(self, "file_bytes", bytes(self.file_bytes))
        object.__setattr__(self, "media_type", _required_text(self.media_type, name="media_type"))
        imported = int(self.imported_utc_ns)
        if imported <= 0:
            raise ValueError("imported_utc_ns must be positive")
        object.__setattr__(self, "imported_utc_ns", imported)

    @property
    def byte_size(self) -> int:
        return len(self.file_bytes)

    @property
    def sha256(self) -> str:
        return _sha256_bytes(self.file_bytes)

    def to_mapping(self) -> dict[str, Any]:
        """Return serializable evidence without embedding the large bytes."""

        return {
            "side": self.side,
            "source_name": self.source_name,
            "camera_serial": self.camera_serial,
            "width_px": self.width_px,
            "height_px": self.height_px,
            "byte_size": self.byte_size,
            "sha256": self.sha256,
            "media_type": self.media_type,
            "imported_utc_ns": self.imported_utc_ns,
            "exact_file_bytes_stored_separately": True,
        }

    def write_exact(self, destination: str | Path) -> Path:
        """Atomically materialize the original bytes without image re-encoding."""

        target = Path(destination)
        target.parent.mkdir(parents=True, exist_ok=True)
        descriptor, temporary_name = tempfile.mkstemp(
            prefix=f".{target.name}.", suffix=".tmp", dir=target.parent
        )
        temporary = Path(temporary_name)
        try:
            with os.fdopen(descriptor, "wb") as handle:
                handle.write(self.file_bytes)
                handle.flush()
                os.fsync(handle.fileno())
            if _sha256_bytes(temporary.read_bytes()) != self.sha256:
                raise IOError("materialized HR image SHA-256 verification failed")
            os.replace(temporary, target)
            try:
                directory_descriptor = os.open(
                    str(target.parent),
                    os.O_RDONLY | getattr(os, "O_DIRECTORY", 0),
                )
            except OSError:
                directory_descriptor = None
            if directory_descriptor is not None:
                try:
                    os.fsync(directory_descriptor)
                finally:
                    os.close(directory_descriptor)
        finally:
            if temporary.exists():
                temporary.unlink()
        return target


def import_hr_image_file(
    path: str | Path,
    *,
    side: str,
    camera_serial: str,
    expected_sha256: str | None = None,
    expected_width_px: int | None = None,
    expected_height_px: int | None = None,
    imported_utc_ns: int | None = None,
) -> HrImportedImage:
    """Read one HR artifact exactly and validate hash/resolution if supplied."""

    source = Path(path).expanduser().resolve(strict=True)
    if not source.is_file():
        raise ValueError("HR import source must be a regular file")
    return import_hr_image_bytes(
        source.read_bytes(),
        source_name=source.name,
        side=side,
        camera_serial=camera_serial,
        media_type=mimetypes.guess_type(source.name)[0] or "application/octet-stream",
        expected_sha256=expected_sha256,
        expected_width_px=expected_width_px,
        expected_height_px=expected_height_px,
        imported_utc_ns=imported_utc_ns,
    )


def import_hr_image_bytes(
    content: bytes,
    *,
    source_name: str,
    side: str,
    camera_serial: str,
    media_type: str | None = None,
    expected_sha256: str | None = None,
    expected_width_px: int | None = None,
    expected_height_px: int | None = None,
    imported_utc_ns: int | None = None,
) -> HrImportedImage:
    """Create verified evidence from already downloaded, unmodified bytes."""

    if not isinstance(content, bytes) or not content:
        raise ValueError("HR imported content must be non-empty immutable bytes")
    digest = _sha256_bytes(content)
    if expected_sha256 is not None and digest != _validate_sha256(
        expected_sha256, name="expected_sha256"
    ):
        raise ValueError("HR imported file SHA-256 mismatch")
    width, height = _decode_image_resolution(content)
    if expected_width_px is not None and width != int(expected_width_px):
        raise ValueError("HR imported file width mismatch")
    if expected_height_px is not None and height != int(expected_height_px):
        raise ValueError("HR imported file height mismatch")
    return HrImportedImage(
        side=side,
        source_name=source_name,
        camera_serial=camera_serial,
        width_px=width,
        height_px=height,
        file_bytes=content,
        media_type=media_type
        or mimetypes.guess_type(str(source_name))[0]
        or "application/octet-stream",
        imported_utc_ns=time.time_ns() if imported_utc_ns is None else int(imported_utc_ns),
    )


@dataclass(frozen=True)
class HrCaptureEvent:
    """Immutable capture/import event; either side may be absent after failure."""

    event_id: str
    capture_id: str
    purpose: CapturePurpose | str
    captured_utc_ns: int
    left: HrImportedImage | None = None
    right: HrImportedImage | None = None
    note: str = ""

    def __post_init__(self) -> None:
        object.__setattr__(self, "event_id", _required_text(self.event_id, name="event_id"))
        object.__setattr__(self, "capture_id", _required_text(self.capture_id, name="capture_id"))
        purpose = CapturePurpose(str(getattr(self.purpose, "value", self.purpose)).lower())
        object.__setattr__(self, "purpose", purpose)
        captured = int(self.captured_utc_ns)
        if captured <= 0:
            raise ValueError("captured_utc_ns must be positive")
        object.__setattr__(self, "captured_utc_ns", captured)
        if self.left is not None and self.left.side != "left":
            raise ValueError("left HR evidence must have side='left'")
        if self.right is not None and self.right.side != "right":
            raise ValueError("right HR evidence must have side='right'")
        object.__setattr__(self, "note", str(self.note))

    def _hash_payload(self) -> dict[str, Any]:
        return {
            "schema": HR_CAPTURE_EVENT_SCHEMA,
            "event_id": self.event_id,
            "capture_id": self.capture_id,
            "purpose": self.purpose.value,
            "captured_utc_ns": self.captured_utc_ns,
            "left": None if self.left is None else self.left.to_mapping(),
            "right": None if self.right is None else self.right.to_mapping(),
            "note": self.note,
        }

    @property
    def event_sha256(self) -> str:
        return _mapping_sha256(self._hash_payload())

    def to_mapping(self) -> dict[str, Any]:
        payload = self._hash_payload()
        payload["event_sha256"] = self.event_sha256
        return payload


def create_hr_capture_event(
    capture_id: str,
    *,
    purpose: CapturePurpose | str = CapturePurpose.UNSPECIFIED,
    left: HrImportedImage | None = None,
    right: HrImportedImage | None = None,
    captured_utc_ns: int | None = None,
    note: str = "",
) -> HrCaptureEvent:
    return HrCaptureEvent(
        event_id=str(uuid.uuid4()),
        capture_id=capture_id,
        purpose=purpose,
        captured_utc_ns=time.time_ns() if captured_utc_ns is None else int(captured_utc_ns),
        left=left,
        right=right,
        note=note,
    )


def hr_frame(side: str) -> str:
    return f"{_side(side)}_hr_optical_rdf"


def oz_frame(side: str) -> str:
    return f"{_side(side)}_oz_optical_rdf"


@dataclass(frozen=True)
class HrOzSideBridge:
    """One side's HR-to-OZ transform and strict camera/intrinsics binding."""

    side: str
    transform_oz_from_hr: SE3Transform
    observation_model: BridgeObservationModel | str
    hr_camera_serial: str
    oz_camera_serial: str
    hr_intrinsics_sha256: str
    oz_intrinsics_sha256: str
    method: str
    view_count: int
    point_count: int
    rms_reprojection_error_px: float | None = None
    angular_rms_error_deg: float | None = None
    per_view_errors_px: tuple[float, ...] = ()

    def __post_init__(self) -> None:
        side = _side(self.side)
        object.__setattr__(self, "side", side)
        if self.transform_oz_from_hr.output_frame != oz_frame(side):
            raise ValueError(f"{side} bridge output frame must be {oz_frame(side)!r}")
        if self.transform_oz_from_hr.input_frame != hr_frame(side):
            raise ValueError(f"{side} bridge input frame must be {hr_frame(side)!r}")
        model = BridgeObservationModel(
            str(getattr(self.observation_model, "value", self.observation_model)).lower()
        )
        object.__setattr__(self, "observation_model", model)
        if model is BridgeObservationModel.ROTATION_ONLY and any(
            abs(value) > 1.0e-15
            for value in self.transform_oz_from_hr.translation_output_from_input_m
        ):
            raise ValueError(
                "rotation-only HR/OZ bridge must store zero translation; "
                "translation is not observable"
            )
        object.__setattr__(
            self,
            "hr_camera_serial",
            _required_text(self.hr_camera_serial, name="hr_camera_serial"),
        )
        object.__setattr__(
            self,
            "oz_camera_serial",
            _required_text(self.oz_camera_serial, name="oz_camera_serial"),
        )
        object.__setattr__(
            self,
            "hr_intrinsics_sha256",
            _validate_sha256(self.hr_intrinsics_sha256, name="hr_intrinsics_sha256"),
        )
        object.__setattr__(
            self,
            "oz_intrinsics_sha256",
            _validate_sha256(self.oz_intrinsics_sha256, name="oz_intrinsics_sha256"),
        )
        object.__setattr__(self, "method", _required_text(self.method, name="method"))
        for name in ("view_count", "point_count"):
            count = int(getattr(self, name))
            if count < 0:
                raise ValueError(f"{name} must be non-negative")
            object.__setattr__(self, name, count)
        if self.rms_reprojection_error_px is not None:
            object.__setattr__(
                self,
                "rms_reprojection_error_px",
                _nonnegative(
                    self.rms_reprojection_error_px,
                    name="rms_reprojection_error_px",
                ),
            )
        if self.angular_rms_error_deg is not None:
            object.__setattr__(
                self,
                "angular_rms_error_deg",
                _nonnegative(self.angular_rms_error_deg, name="angular_rms_error_deg"),
            )
        object.__setattr__(
            self,
            "per_view_errors_px",
            tuple(
                _nonnegative(value, name="per_view_errors_px")
                for value in self.per_view_errors_px
            ),
        )

    @property
    def translation_observable(self) -> bool:
        return self.observation_model is BridgeObservationModel.FULL_SE3

    @property
    def translation_contract(self) -> str:
        if self.translation_observable:
            return "estimated_full_se3_translation_m"
        return "zero_serialization_sentinel_translation_not_observable"

    @property
    def coordinate_contract(self) -> str:
        return HR_OZ_COORDINATE_CONTRACT

    def transform_point(self, point_hr_rdf_m: Iterable[Any]) -> tuple[float, float, float]:
        return self.transform_oz_from_hr.transform_point(point_hr_rdf_m)

    def to_mapping(self) -> dict[str, Any]:
        return {
            "side": self.side,
            "coordinate_contract": self.coordinate_contract,
            "observation_model": self.observation_model.value,
            "translation_observable": self.translation_observable,
            "translation_contract": self.translation_contract,
            "transform_oz_from_hr": {
                "output_frame": self.transform_oz_from_hr.output_frame,
                "input_frame": self.transform_oz_from_hr.input_frame,
                "rotation_output_from_input": [
                    list(row) for row in self.transform_oz_from_hr.rotation_output_from_input
                ],
                "translation_output_from_input_m": list(
                    self.transform_oz_from_hr.translation_output_from_input_m
                ),
            },
            "hr_camera_serial": self.hr_camera_serial,
            "oz_camera_serial": self.oz_camera_serial,
            "hr_intrinsics_sha256": self.hr_intrinsics_sha256,
            "oz_intrinsics_sha256": self.oz_intrinsics_sha256,
            "method": self.method,
            "view_count": self.view_count,
            "point_count": self.point_count,
            "rms_reprojection_error_px": self.rms_reprojection_error_px,
            "angular_rms_error_deg": self.angular_rms_error_deg,
            "per_view_errors_px": list(self.per_view_errors_px),
        }

    @classmethod
    def from_mapping(cls, payload: Mapping[str, Any]) -> "HrOzSideBridge":
        transform = payload["transform_oz_from_hr"]
        if not isinstance(transform, Mapping):
            raise ValueError("transform_oz_from_hr must be a mapping")
        bridge = cls(
            side=str(payload["side"]),
            transform_oz_from_hr=SE3Transform(
                output_frame=str(transform["output_frame"]),
                input_frame=str(transform["input_frame"]),
                rotation_output_from_input=transform["rotation_output_from_input"],
                translation_output_from_input_m=transform["translation_output_from_input_m"],
            ),
            observation_model=str(payload["observation_model"]),
            hr_camera_serial=str(payload["hr_camera_serial"]),
            oz_camera_serial=str(payload["oz_camera_serial"]),
            hr_intrinsics_sha256=str(payload["hr_intrinsics_sha256"]),
            oz_intrinsics_sha256=str(payload["oz_intrinsics_sha256"]),
            method=str(payload["method"]),
            view_count=int(payload["view_count"]),
            point_count=int(payload["point_count"]),
            rms_reprojection_error_px=payload.get("rms_reprojection_error_px"),
            angular_rms_error_deg=payload.get("angular_rms_error_deg"),
            per_view_errors_px=tuple(payload.get("per_view_errors_px") or ()),
        )
        if payload.get("coordinate_contract") != bridge.coordinate_contract:
            raise ValueError("HR/OZ side bridge coordinate_contract mismatch")
        if payload.get("translation_observable") is not bridge.translation_observable:
            raise ValueError("HR/OZ side bridge translation_observable mismatch")
        if payload.get("translation_contract") != bridge.translation_contract:
            raise ValueError("HR/OZ side bridge translation_contract mismatch")
        return bridge


@dataclass(frozen=True)
class HrOzBridgeProfile:
    """Immutable pair of per-side bridges with a tamper-evident hash."""

    profile_id: str
    left: HrOzSideBridge
    right: HrOzSideBridge
    created_utc_ns: int
    source_event_ids: tuple[str, ...] = ()
    note: str = ""

    def __post_init__(self) -> None:
        object.__setattr__(self, "profile_id", _required_text(self.profile_id, name="profile_id"))
        if self.left.side != "left" or self.right.side != "right":
            raise ValueError("bridge profile must contain left then right side bridges")
        created = int(self.created_utc_ns)
        if created <= 0:
            raise ValueError("created_utc_ns must be positive")
        object.__setattr__(self, "created_utc_ns", created)
        object.__setattr__(
            self,
            "source_event_ids",
            tuple(_required_text(value, name="source_event_id") for value in self.source_event_ids),
        )
        object.__setattr__(self, "note", str(self.note))

    @property
    def stereo_translation_transfer_allowed(self) -> bool:
        return self.left.translation_observable and self.right.translation_observable

    def _hash_payload(self) -> dict[str, Any]:
        return {
            "schema": HR_OZ_BRIDGE_PROFILE_SCHEMA,
            "profile_id": self.profile_id,
            "coordinate_contract": HR_OZ_COORDINATE_CONTRACT,
            "left": self.left.to_mapping(),
            "right": self.right.to_mapping(),
            "created_utc_ns": self.created_utc_ns,
            "source_event_ids": list(self.source_event_ids),
            "note": self.note,
            "stereo_translation_transfer_allowed": self.stereo_translation_transfer_allowed,
        }

    @property
    def profile_sha256(self) -> str:
        return _mapping_sha256(self._hash_payload())

    def to_mapping(self) -> dict[str, Any]:
        payload = self._hash_payload()
        payload["profile_sha256"] = self.profile_sha256
        return payload

    @classmethod
    def from_mapping(cls, payload: Mapping[str, Any]) -> "HrOzBridgeProfile":
        expected_schema = payload.get("schema")
        if expected_schema != HR_OZ_BRIDGE_PROFILE_SCHEMA:
            raise ValueError(f"unsupported HR/OZ bridge schema: {expected_schema!r}")
        if payload.get("coordinate_contract") != HR_OZ_COORDINATE_CONTRACT:
            raise ValueError("HR/OZ bridge coordinate_contract mismatch")
        left_payload = payload.get("left")
        right_payload = payload.get("right")
        if not isinstance(left_payload, Mapping) or not isinstance(right_payload, Mapping):
            raise ValueError("HR/OZ bridge left/right entries must be mappings")
        profile = cls(
            profile_id=str(payload["profile_id"]),
            left=HrOzSideBridge.from_mapping(left_payload),
            right=HrOzSideBridge.from_mapping(right_payload),
            created_utc_ns=int(payload["created_utc_ns"]),
            source_event_ids=tuple(payload.get("source_event_ids") or ()),
            note=str(payload.get("note") or ""),
        )
        if (
            payload.get("stereo_translation_transfer_allowed")
            is not profile.stereo_translation_transfer_allowed
        ):
            raise ValueError("HR/OZ bridge stereo_translation_transfer_allowed mismatch")
        claimed = _validate_sha256(payload.get("profile_sha256"), name="profile_sha256")
        if claimed != profile.profile_sha256:
            raise ValueError("HR/OZ bridge profile_sha256 mismatch")
        return profile


@dataclass(frozen=True)
class BridgeCompatibilityEvidence:
    side: str
    hr_camera_serial: str
    oz_camera_serial: str
    hr_intrinsics_sha256: str
    oz_intrinsics_sha256: str

    def __post_init__(self) -> None:
        object.__setattr__(self, "side", _side(self.side))
        object.__setattr__(
            self,
            "hr_camera_serial",
            _required_text(self.hr_camera_serial, name="hr_camera_serial"),
        )
        object.__setattr__(
            self,
            "oz_camera_serial",
            _required_text(self.oz_camera_serial, name="oz_camera_serial"),
        )
        object.__setattr__(
            self,
            "hr_intrinsics_sha256",
            _validate_sha256(self.hr_intrinsics_sha256, name="hr_intrinsics_sha256"),
        )
        object.__setattr__(
            self,
            "oz_intrinsics_sha256",
            _validate_sha256(self.oz_intrinsics_sha256, name="oz_intrinsics_sha256"),
        )


@dataclass(frozen=True)
class BridgeApplicability:
    state: str
    geometry_allowed: bool
    stereo_translation_transfer_allowed: bool
    raw_evidence_allowed: bool
    reasons: tuple[str, ...]
    profile_sha256: str


def evaluate_bridge_applicability(
    profile: HrOzBridgeProfile | None,
    left: BridgeCompatibilityEvidence | None,
    right: BridgeCompatibilityEvidence | None,
) -> BridgeApplicability:
    """Require exact per-side serial and intrinsics-hash identity.

    Mismatch blocks geometry but never blocks retaining raw HR/OZ evidence.
    A compatible rotation-only profile allows rotational geometry while
    explicitly disallowing stereo-translation transfer.
    """

    if profile is None:
        return BridgeApplicability(
            state="BLOCKED",
            geometry_allowed=False,
            stereo_translation_transfer_allowed=False,
            raw_evidence_allowed=True,
            reasons=("hr_oz_bridge_profile_missing",),
            profile_sha256="",
        )
    reasons: list[str] = []
    for side_name, evidence, expected in (
        ("left", left, profile.left),
        ("right", right, profile.right),
    ):
        if evidence is None:
            reasons.append(f"{side_name}_bridge_evidence_missing")
            continue
        if evidence.side != side_name:
            reasons.append(f"{side_name}_bridge_evidence_side_mismatch")
            continue
        for field_name in (
            "hr_camera_serial",
            "oz_camera_serial",
            "hr_intrinsics_sha256",
            "oz_intrinsics_sha256",
        ):
            if getattr(evidence, field_name) != getattr(expected, field_name):
                reasons.append(f"{side_name}_{field_name}_mismatch")
    ready = not reasons
    return BridgeApplicability(
        state="READY" if ready else "BLOCKED",
        geometry_allowed=ready,
        stereo_translation_transfer_allowed=(
            ready and profile.stereo_translation_transfer_allowed
        ),
        raw_evidence_allowed=True,
        reasons=tuple(reasons),
        profile_sha256=profile.profile_sha256,
    )


def transfer_hr_stereo_pose_to_oz(
    hr_stereo_pose: StereoPoseRdf,
    profile: HrOzBridgeProfile,
) -> StereoPoseRdf:
    """Transfer ``H_right_HR<-left_HR`` into the two OZ optical frames.

    Full per-side SE(3) bridges are required because independent unknown HR/OZ
    center offsets contribute directly to the resulting stereo translation.
    Rotation-only bridge zero sentinels are therefore rejected instead of
    being mistaken for measured coincident optical centers.
    """

    if not profile.stereo_translation_transfer_allowed:
        raise ValueError(
            "HR-to-OZ stereo translation transfer requires full SE3 bridges "
            "on both sides; rotation-only translation is not observable"
        )
    return transfer_stereo_pose_through_bridges(
        hr_stereo_pose,
        left_target_from_source=profile.left.transform_oz_from_hr,
        right_target_from_source=profile.right.transform_oz_from_hr,
    )


@dataclass(frozen=True)
class BridgeCameraIntrinsics:
    """Fixed intrinsics bound to one side and either HR or OZ channel."""

    side: str
    channel: str
    width_px: int
    height_px: int
    camera_matrix: tuple[tuple[float, float, float], ...]
    distortion_coefficients: tuple[float, ...]
    camera_serial: str
    intrinsics_sha256: str

    def __post_init__(self) -> None:
        object.__setattr__(self, "side", _side(self.side))
        channel = str(self.channel).strip().lower()
        if channel not in {"hr", "oz"}:
            raise ValueError("channel must be hr or oz")
        object.__setattr__(self, "channel", channel)
        width = int(self.width_px)
        height = int(self.height_px)
        if width <= 0 or height <= 0:
            raise ValueError("intrinsics image dimensions must be positive")
        object.__setattr__(self, "width_px", width)
        object.__setattr__(self, "height_px", height)
        matrix = tuple(
            tuple(_finite(value, name="camera_matrix") for value in row)
            for row in self.camera_matrix
        )
        if len(matrix) != 3 or any(len(row) != 3 for row in matrix):
            raise ValueError("camera_matrix must be 3x3")
        if matrix[0][0] <= 0.0 or matrix[1][1] <= 0.0:
            raise ValueError("camera_matrix fx/fy must be positive")
        if any(
            abs(matrix[2][index] - expected) > 1.0e-9
            for index, expected in enumerate((0.0, 0.0, 1.0))
        ):
            raise ValueError("camera_matrix last row must be [0,0,1]")
        object.__setattr__(self, "camera_matrix", matrix)
        distortion = tuple(
            _finite(value, name="distortion_coefficients")
            for value in self.distortion_coefficients
        )
        if not distortion:
            raise ValueError("distortion_coefficients must not be empty")
        object.__setattr__(self, "distortion_coefficients", distortion)
        object.__setattr__(
            self,
            "camera_serial",
            _required_text(self.camera_serial, name="camera_serial"),
        )
        object.__setattr__(
            self,
            "intrinsics_sha256",
            _validate_sha256(self.intrinsics_sha256, name="intrinsics_sha256"),
        )

    @property
    def optical_frame(self) -> str:
        return hr_frame(self.side) if self.channel == "hr" else oz_frame(self.side)


@dataclass(frozen=True)
class BridgeCorrespondenceView:
    """Same metric target observed in one HR/OZ image pair.

    ``object_points_m`` are right-handed target-local coordinates in metres.
    Pixel tuples are OpenCV ``(u right, v down)`` in the corresponding raw
    image.  Point ordering must be identical in all three arrays.
    """

    object_points_m: tuple[tuple[float, float, float], ...]
    hr_pixels_px: tuple[tuple[float, float], ...]
    oz_pixels_px: tuple[tuple[float, float], ...]

    def __post_init__(self) -> None:
        objects = tuple(
            tuple(_finite(value, name="object_points_m") for value in point)
            for point in self.object_points_m
        )
        hr_pixels = tuple(
            tuple(_finite(value, name="hr_pixels_px") for value in point)
            for point in self.hr_pixels_px
        )
        oz_pixels = tuple(
            tuple(_finite(value, name="oz_pixels_px") for value in point)
            for point in self.oz_pixels_px
        )
        if any(len(point) != 3 for point in objects):
            raise ValueError("each object point must have three metric coordinates")
        if any(len(point) != 2 for point in hr_pixels + oz_pixels):
            raise ValueError("each image point must be an OpenCV (u,v) pair")
        if len(objects) < 4 or not (len(objects) == len(hr_pixels) == len(oz_pixels)):
            raise ValueError("each HR/OZ view needs at least four ordered correspondences")
        object.__setattr__(self, "object_points_m", objects)
        object.__setattr__(self, "hr_pixels_px", hr_pixels)
        object.__setattr__(self, "oz_pixels_px", oz_pixels)


def solve_full_se3_hr_to_oz_bridge(
    views: Sequence[BridgeCorrespondenceView],
    *,
    hr_intrinsics: BridgeCameraIntrinsics,
    oz_intrinsics: BridgeCameraIntrinsics,
    method: str = "opencv_stereo_calibrate_extended_fix_intrinsic",
) -> HrOzSideBridge:
    """Estimate ``H_OZ<-HR`` with fixed K,D using OpenCV.

    At least three target poses are required.  Metric object coordinates make
    translation observable in metres.  ``cv2.CALIB_FIX_INTRINSIC`` is always
    set; the supplied K,D arrays are evidence inputs, not optimization
    variables.  OpenCV's single ``imageSize`` argument is set to the OZ image
    size; it is not used to rescale either fixed camera matrix.
    """

    if hr_intrinsics.channel != "hr" or oz_intrinsics.channel != "oz":
        raise ValueError("solver requires HR intrinsics first and OZ intrinsics second")
    if hr_intrinsics.side != oz_intrinsics.side:
        raise ValueError("HR and OZ intrinsics must belong to the same side")
    view_tuple = tuple(views)
    if len(view_tuple) < 3:
        raise ValueError("full SE3 HR/OZ bridge requires at least three correspondence views")

    try:
        import cv2
        import numpy as np
    except ImportError as exc:  # pragma: no cover - exercised on minimal deployments
        raise RuntimeError("OpenCV and NumPy are required for HR/OZ bridge solving") from exc

    object_points = [
        np.asarray(view.object_points_m, dtype=np.float32).reshape(-1, 1, 3)
        for view in view_tuple
    ]
    hr_points = [
        np.asarray(view.hr_pixels_px, dtype=np.float32).reshape(-1, 1, 2)
        for view in view_tuple
    ]
    oz_points = [
        np.asarray(view.oz_pixels_px, dtype=np.float32).reshape(-1, 1, 2)
        for view in view_tuple
    ]
    hr_matrix = np.asarray(hr_intrinsics.camera_matrix, dtype=np.float64)
    hr_distortion = np.asarray(hr_intrinsics.distortion_coefficients, dtype=np.float64)
    oz_matrix = np.asarray(oz_intrinsics.camera_matrix, dtype=np.float64)
    oz_distortion = np.asarray(oz_intrinsics.distortion_coefficients, dtype=np.float64)
    criteria = (
        cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
        100,
        1.0e-10,
    )
    try:
        result = cv2.stereoCalibrateExtended(
            object_points,
            hr_points,
            oz_points,
            hr_matrix,
            hr_distortion,
            oz_matrix,
            oz_distortion,
            (oz_intrinsics.width_px, oz_intrinsics.height_px),
            np.eye(3, dtype=np.float64),
            np.zeros((3, 1), dtype=np.float64),
            flags=cv2.CALIB_FIX_INTRINSIC,
            criteria=criteria,
        )
    except cv2.error as exc:
        raise ValueError(f"OpenCV HR/OZ bridge solve failed: {exc}") from exc
    rms = _nonnegative(result[0], name="OpenCV bridge RMS")
    rotation = result[5]
    translation = result[6]
    per_view = result[-1] if len(result) >= 10 else None
    per_view_errors = (
        ()
        if per_view is None
        else tuple(_nonnegative(value, name="per_view_errors_px") for value in np.ravel(per_view))
    )
    transform = SE3Transform(
        output_frame=oz_frame(hr_intrinsics.side),
        input_frame=hr_frame(hr_intrinsics.side),
        rotation_output_from_input=tuple(
            tuple(float(value) for value in row) for row in np.asarray(rotation)
        ),
        translation_output_from_input_m=tuple(
            float(value) for value in np.ravel(translation)
        ),
    )
    return HrOzSideBridge(
        side=hr_intrinsics.side,
        transform_oz_from_hr=transform,
        observation_model=BridgeObservationModel.FULL_SE3,
        hr_camera_serial=hr_intrinsics.camera_serial,
        oz_camera_serial=oz_intrinsics.camera_serial,
        hr_intrinsics_sha256=hr_intrinsics.intrinsics_sha256,
        oz_intrinsics_sha256=oz_intrinsics.intrinsics_sha256,
        method=method,
        view_count=len(view_tuple),
        point_count=sum(len(view.object_points_m) for view in view_tuple),
        rms_reprojection_error_px=rms,
        per_view_errors_px=per_view_errors,
    )


def _unit_vector(values: Iterable[Any], *, name: str) -> tuple[float, float, float]:
    vector = tuple(_finite(value, name=name) for value in values)
    if len(vector) != 3:
        raise ValueError(f"{name} must contain three values")
    norm = math.sqrt(sum(value * value for value in vector))
    if norm <= 0.0:
        raise ValueError(f"{name} must be non-zero")
    return tuple(value / norm for value in vector)  # type: ignore[return-value]


def solve_rotation_only_hr_to_oz_bridge(
    hr_rays_rdf: Sequence[Iterable[Any]],
    oz_rays_rdf: Sequence[Iterable[Any]],
    *,
    side: str,
    hr_camera_serial: str,
    oz_camera_serial: str,
    hr_intrinsics_sha256: str,
    oz_intrinsics_sha256: str,
    method: str = "unit_ray_wahba_svd",
) -> HrOzSideBridge:
    """Estimate only ``R_OZ<-HR`` from paired optical RDF unit rays.

    Translation is mathematically unobservable from bearing pairs alone.
    Consequently the returned zero ``T`` is an explicit sentinel and
    ``translation_observable`` is false.
    """

    hr_rays = tuple(_unit_vector(ray, name="hr_ray_rdf") for ray in hr_rays_rdf)
    oz_rays = tuple(_unit_vector(ray, name="oz_ray_rdf") for ray in oz_rays_rdf)
    if len(hr_rays) != len(oz_rays) or len(hr_rays) < 3:
        raise ValueError("rotation-only bridge requires at least three paired rays")
    try:
        import numpy as np
    except ImportError as exc:  # pragma: no cover - exercised on minimal deployments
        raise RuntimeError("NumPy is required for rotation-only HR/OZ bridge solving") from exc
    hr = np.asarray(hr_rays, dtype=np.float64).T
    oz = np.asarray(oz_rays, dtype=np.float64).T
    covariance = oz @ hr.T
    u, singular_values, vt = np.linalg.svd(covariance)
    if singular_values[1] <= max(1.0, singular_values[0]) * 1.0e-12:
        raise ValueError("paired rays are degenerate; need non-parallel directions")
    correction = np.eye(3, dtype=np.float64)
    correction[2, 2] = 1.0 if np.linalg.det(u @ vt) >= 0.0 else -1.0
    rotation = u @ correction @ vt
    rotated = rotation @ hr
    dots = np.clip(np.sum(rotated * oz, axis=0), -1.0, 1.0)
    # Exact synthetic/duplicate directions can land a few machine epsilons
    # below 1.0 after SVD depending on the supported NumPy build.  Treat only
    # that floating-point neighbourhood as the mathematically exact endpoint;
    # real angular residuals remain unchanged.
    dots = np.where(dots >= 1.0 - 8.0 * np.finfo(np.float64).eps, 1.0, dots)
    angular_errors_deg = np.degrees(np.arccos(dots))
    angular_rms_deg = float(np.sqrt(np.mean(np.square(angular_errors_deg))))
    resolved_side = _side(side)
    transform = SE3Transform(
        output_frame=oz_frame(resolved_side),
        input_frame=hr_frame(resolved_side),
        rotation_output_from_input=tuple(
            tuple(float(value) for value in row) for row in rotation
        ),
        translation_output_from_input_m=(0.0, 0.0, 0.0),
    )
    return HrOzSideBridge(
        side=resolved_side,
        transform_oz_from_hr=transform,
        observation_model=BridgeObservationModel.ROTATION_ONLY,
        hr_camera_serial=hr_camera_serial,
        oz_camera_serial=oz_camera_serial,
        hr_intrinsics_sha256=hr_intrinsics_sha256,
        oz_intrinsics_sha256=oz_intrinsics_sha256,
        method=method,
        view_count=1,
        point_count=len(hr_rays),
        angular_rms_error_deg=angular_rms_deg,
    )


__all__ = [
    "BridgeApplicability",
    "BridgeCameraIntrinsics",
    "BridgeCompatibilityEvidence",
    "BridgeCorrespondenceView",
    "BridgeObservationModel",
    "CapturePurpose",
    "HR_CAPTURE_EVENT_SCHEMA",
    "HR_OZ_BRIDGE_PROFILE_SCHEMA",
    "HR_OZ_COORDINATE_CONTRACT",
    "HrCaptureEvent",
    "HrImportedImage",
    "HrOzBridgeProfile",
    "HrOzSideBridge",
    "create_hr_capture_event",
    "evaluate_bridge_applicability",
    "hr_frame",
    "import_hr_image_bytes",
    "import_hr_image_file",
    "oz_frame",
    "solve_full_se3_hr_to_oz_bridge",
    "solve_rotation_only_hr_to_oz_bridge",
    "transfer_hr_stereo_pose_to_oz",
]
