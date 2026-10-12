"""Native pixel observation validation and sampling, without a live owner.

Coordinates are image u/v pixels; undistortion returns normalized optical
RDF ray coordinates. Artifact identity/dimensions are supplied explicitly.
"""

from __future__ import annotations
from typing import TYPE_CHECKING, Any, Mapping, Sequence
import copy
import math
from dataclasses import fields as _dataclass_fields

if TYPE_CHECKING:
    from Workstation.workstation_core.native_frame_store import NativeFrameArtifact
from Workstation.workstation_core.rtk_geometry import ned_delta_m
from .policy import (
    DRONE_RTK_FRAME_PAIR_MAX_DELTA_S,
    DRONE_RTK_FRAME_GNSS_MAX_ABS_DELTA_S,
)
from Workstation.workstation_core.rtk_geometry import GpsPositionState
from .geometry_values import _finite_vector
from .policy import CLOUD_FRAME_PAIR_MAX_DELTA_S, _MOUNTAIN_MINIMUM_GNSS_FIX_TYPE


def validate_annotation_identity(artifact, side: str, annotation: Mapping[str, Any]) -> None:
    if artifact is None:
        raise RuntimeError(f"{side}: no frozen native frame")
    if str(annotation.get("frame_id") or "") != artifact.frame_id:
        raise ValueError(f"{side}: annotation frame_id does not match frozen evidence")
    if str(annotation.get("image_sha256") or "") != artifact.sha256:
        raise ValueError(f"{side}: annotation image_sha256 does not match frozen PNG evidence")
    width = int(annotation.get("source_width_px", -1))
    height = int(annotation.get("source_height_px", -1))
    if (width, height) != (artifact.width, artifact.height):
        raise ValueError(f"{side}: annotation native dimensions do not match frozen evidence")


def annotation_points(annotation: Mapping[str, Any], key: str) -> tuple[tuple[float, float], ...]:
    raw = annotation.get(key)
    if not isinstance(raw, (tuple, list)):
        return ()
    width = int(annotation["source_width_px"])
    height = int(annotation["source_height_px"])
    parsed: list[tuple[float, float]] = []
    for index, point in enumerate(raw):
        if isinstance(point, Mapping):
            value = (point.get("u_px"), point.get("v_px"))
        else:
            value = point
        (u, v) = _finite_vector(value, length=2, name=f"{key}[{index}]")
        if not (0.0 <= u <= width - 1 and 0.0 <= v <= height - 1):
            raise ValueError(f"{key}[{index}] is outside the native image")
        parsed.append((u, v))
    return tuple(parsed)


def sample_horizon_pixels(points: Sequence[tuple[float, float]]) -> tuple[tuple[float, float], ...]:
    if len(points) != 2:
        return tuple(points)
    (first, second) = points
    return tuple(
        (
            (first[0] + (second[0] - first[0]) * index / 32.0, first[1] + (second[1] - first[1]) * index / 32.0)
            for index in range(33)
        )
    )


def undistort_pixels(camera: Any, points: Sequence[Sequence[float]]) -> tuple[tuple[float, float], ...]:
    try:
        import cv2
        import numpy as np
    except Exception as exc:
        raise RuntimeError("OpenCV and NumPy are required for optical-pose solving") from exc
    raw = np.asarray(points, dtype=np.float64).reshape(-1, 1, 2)
    normalized = cv2.undistortPoints(
        raw,
        np.asarray(camera.camera_matrix, dtype=np.float64),
        np.asarray(camera.distortion_coefficients, dtype=np.float64),
    ).reshape(-1, 2)
    return tuple(((float(point[0]), float(point[1])) for point in normalized))


def coerce_gps_state(value: Any, *, side: str, now_s: float) -> GpsPositionState | None:
    """Accept live ``GpsPositionState`` or a serialized telemetry row."""

    if isinstance(value, GpsPositionState):
        return value
    if not isinstance(value, Mapping):
        return None
    latitude = value.get("latitude_deg", value.get("lat"))
    longitude = value.get("longitude_deg", value.get("lon"))
    altitude = value.get("altitude_m", value.get("alt"))
    if latitude is None or longitude is None or altitude is None:
        return None
    timestamp = value.get("timestamp_s")
    if timestamp is None:
        age = value.get("age_s")
        timestamp = float(now_s) - max(0.0, float(age or 0.0))
    try:
        fix_value = value.get("fix_type")
        fix_type = None if fix_value in (None, "") else int(fix_value)
        return GpsPositionState(
            camera=side,
            namespace=str(value.get("namespace") or "telemetry"),
            timestamp_s=float(timestamp),
            topic=str(value.get("topic") or "telemetry/gps"),
            latitude_deg=float(latitude),
            longitude_deg=float(longitude),
            altitude_m=float(altitude),
            fix_type=fix_type,
            satellites_used=(
                None
                if value.get("satellites_used", value.get("satellites")) in (None, "")
                else int(value.get("satellites_used", value.get("satellites")))
            ),
        )
    except (TypeError, ValueError):
        return None


def _gps_state_observation_mapping(value: Any, *, role: str) -> dict[str, Any] | None:
    """JSON-safe deep snapshot of one live ``GpsPositionState``.

    Returns ``None`` when no live sample exists for this role -- the caller
    must not fabricate a position.  ``state.raw`` (the live mavlink message
    object) is deliberately excluded: observation history must never hold a
    reference back into mutable/live telemetry state, only a plain-value copy
    taken at this instant.
    """

    if isinstance(value, Mapping):
        snapshot = copy.deepcopy(dict(value))
        snapshot.setdefault("role", role)
        return snapshot
    if not isinstance(value, GpsPositionState):
        return None
    return {
        "role": role,
        "camera": value.camera,
        "namespace": value.namespace,
        "topic": value.topic,
        "latitude_deg": value.latitude_deg,
        "longitude_deg": value.longitude_deg,
        "altitude_m": value.altitude_m,
        "altitude_ellipsoid_m": value.altitude_ellipsoid_m,
        "fix_type": value.fix_type,
        "rtk_label": value.rtk_label,
        "satellites_used": value.satellites_used,
        "eph_m": value.eph_m,
        "epv_m": value.epv_m,
        "timestamp_s": float(value.timestamp_s),
        "time_source": value.time_source,
        "px4_timestamp_s": value.px4_timestamp_s,
        "px4_timestamp_us": value.px4_timestamp_us,
        "px4_timestamp_sample_us": value.px4_timestamp_sample_us,
        "gps_utc_us": value.gps_utc_us,
        "gps_timestamp_time_relative_us": value.gps_timestamp_time_relative_us,
        "velocity_n_m_s": value.velocity_n_m_s,
        "velocity_e_m_s": value.velocity_e_m_s,
        "velocity_d_m_s": value.velocity_d_m_s,
        "velocity_ned_valid": value.velocity_ned_valid,
        "hdop": value.hdop,
        "vdop": value.vdop,
        "device_id": value.device_id,
        "has_position": value.has_position,
    }


_GPS_STATE_FIELD_NAMES = frozenset(field.name for field in _dataclass_fields(GpsPositionState))


def _gps_state_from_capture_mapping(row: Any) -> GpsPositionState | None:
    """Reconstruct one immutable ``GpsPositionState`` from stored evidence.

    ``row`` is the JSON-safe deep snapshot produced by
    ``_gps_state_observation_mapping`` for a *past* Capture, never live
    telemetry. Returns ``None`` when no usable lat/lon/alt was recorded --
    the caller must not fabricate a position from a partial snapshot.
    """

    if not isinstance(row, Mapping):
        return None
    if row.get("latitude_deg") is None or row.get("longitude_deg") is None or row.get("altitude_m") is None:
        return None
    kwargs = {key: value for key, value in row.items() if key in _GPS_STATE_FIELD_NAMES}
    return GpsPositionState(**kwargs)


def _mountain_fix_type(value: Any, *, rtk_label: Any) -> int | None:
    """Resolve auditable GNSS fix quality for the Mountain translation gate.

    Capture observations carry the numeric MAVLink/PX4 ``fix_type``. Explicit
    baseline overrides may instead retain an older RTK label, so the known
    labels are mapped to their minimum equivalent without treating an unknown
    label as usable evidence.
    """

    if value not in (None, ""):
        try:
            fix_type = int(value)
        except (TypeError, ValueError, OverflowError):
            return None
        return fix_type if fix_type >= 0 else None
    return {
        "3D_FIX": 3,
        "RTK_FLOAT": 5,
        "RTK_FIXED": 6,
    }.get(str(rtk_label or "").strip().upper())


def _finite_gps_position(state: GpsPositionState) -> bool:
    if not state.has_position:
        return False
    try:
        return all(math.isfinite(float(value)) for value in (state.latitude_deg, state.longitude_deg, state.altitude_m))
    except (TypeError, ValueError, OverflowError):
        return False


def _frame_clock_domain_verified(artifact: NativeFrameArtifact) -> bool:
    """True only when ``artifact.timestamp_s`` is provably host-monotonic.

    ``timestamp_reference`` is set by
    ``camera_pane.infer_camera_timestamp_reference``
    (workstation_core/camera_pane.py:13-28) and resolves to "app-receive"
    only when the frame producer supplied no NTP/source timestamp -- i.e.
    ``timestamp_s`` was stamped with ``time.monotonic()`` at
    ``CameraPane._show_frame`` (workstation_core/camera_pane.py:210). Every
    live ``GpsPositionState.timestamp_s`` in this codebase is stamped with
    that same ``time.monotonic()`` call, in the same process, at MAVLink
    message receipt (``PixhawkTelemetrySubscriber._handle_mavlink_message``,
    workstation_core/pixhawk_attitude.py:861-905). Only when both sides carry
    that "app-receive" marker are the two timestamps known to share a clock
    and therefore safe to subtract.
    """

    return artifact.timestamp_reference == "app-receive"


def _drone_rtk_sync_mapping(
    *,
    left_artifact: NativeFrameArtifact,
    right_artifact: NativeFrameArtifact,
    drone_state: GpsPositionState | None,
) -> dict[str, Any]:
    bounds = {
        "left_right_frame_max_delta_s": DRONE_RTK_FRAME_PAIR_MAX_DELTA_S,
        "frame_to_drone_gnss_max_abs_delta_s": DRONE_RTK_FRAME_GNSS_MAX_ABS_DELTA_S,
    }

    def unverified(reason: str) -> dict[str, Any]:
        return {
            "state": "SYNC_UNVERIFIED",
            "left_frame_to_drone_gnss_s": None,
            "right_frame_to_drone_gnss_s": None,
            "left_right_frame_delta_s": None,
            **bounds,
            "reason": reason,
        }

    if drone_state is None:
        return unverified("no Drone GNSS sample was available at capture")
    if not (_frame_clock_domain_verified(left_artifact) and _frame_clock_domain_verified(right_artifact)):
        return unverified(
            "frame/GNSS clock-domain identity not confirmed "
            f"(left.timestamp_reference={left_artifact.timestamp_reference!r}, "
            f"right.timestamp_reference={right_artifact.timestamp_reference!r})"
        )
    try:
        drone_timestamp_s = float(drone_state.timestamp_s)
        left_timestamp_s = float(left_artifact.timestamp_s)
        right_timestamp_s = float(right_artifact.timestamp_s)
    except (TypeError, ValueError, OverflowError):
        return unverified("frame/GNSS timestamps are not finite numbers")
    if not all(math.isfinite(value) for value in (drone_timestamp_s, left_timestamp_s, right_timestamp_s)):
        return unverified("frame/GNSS timestamps are not finite numbers")

    left_right_delta_s = right_timestamp_s - left_timestamp_s
    if abs(left_right_delta_s) > DRONE_RTK_FRAME_PAIR_MAX_DELTA_S:
        return unverified(
            f"left/right frame delta {abs(left_right_delta_s):.6f}s exceeds {DRONE_RTK_FRAME_PAIR_MAX_DELTA_S:.6f}s"
        )
    left_delta_s = drone_timestamp_s - left_timestamp_s
    right_delta_s = drone_timestamp_s - right_timestamp_s
    if max(abs(left_delta_s), abs(right_delta_s)) > DRONE_RTK_FRAME_GNSS_MAX_ABS_DELTA_S:
        return unverified(f"frame/Drone GNSS delta exceeds {DRONE_RTK_FRAME_GNSS_MAX_ABS_DELTA_S:.6f}s")
    return {
        "state": "SYNC_OK",
        "left_frame_to_drone_gnss_s": left_delta_s,
        "right_frame_to_drone_gnss_s": right_delta_s,
        "left_right_frame_delta_s": left_right_delta_s,
        **bounds,
        "reason": "",
    }


def _drone_rtk_observation_state(
    *,
    left_gnss: dict[str, Any] | None,
    right_gnss: dict[str, Any] | None,
    drone_gnss: dict[str, Any] | None,
    sync: Mapping[str, Any],
) -> str:
    """Evidence-completeness state -- never a claim about pose accuracy."""

    for row in (left_gnss, right_gnss, drone_gnss):
        if row is None or not row.get("has_position"):
            return "INCOMPLETE"
    all_fixed = all(row.get("rtk_label") == "RTK_FIXED" for row in (left_gnss, right_gnss, drone_gnss))
    if all_fixed and sync.get("state") == "SYNC_OK":
        return "EVIDENCE_READY"
    return "WARNING"


def resolve_mountain_baseline(payload, *, observation):
    """Resolve one auditable right-minus-left GNSS baseline in world NED.

    Priority is (1) an explicit payload override -- always tagged as an
    override, never silently mistaken for capture evidence -- then (2)
    the Left/Right GNSS snapshot captured with *this exact* frozen burst
    (``self._drone_rtk_observations``, matched by ``burst_id``). There is
    no fallback to ``panel.latest_gps``/``panel.latest_rtk_baseline``: a
    frozen Mountain image must never be paired with whatever GNSS
    happens to be live on the panel at solve time.
    """

    raw = payload.get("gnss_baseline_ned_m")
    source = "payload.gnss_baseline_ned_m (explicit override)"
    details: dict[str, Any] = {}
    if raw is None:
        raw = payload.get("baseline_ned_m")
        if raw is not None:
            source = "payload.baseline_ned_m (explicit override)"

    if raw is not None:
        if isinstance(raw, Mapping):
            vector = raw.get("ned_m", raw.get("vector_m"))
            if vector is None:
                vector = (
                    raw.get("north_m"),
                    raw.get("east_m"),
                    raw.get("down_m"),
                )
            details.update(dict(raw))
        else:
            vector = raw
        baseline_ned = _finite_vector(
            vector or (),
            length=3,
            name="gnss_baseline_ned_m",
        )
        details["capture_observation_id"] = None
        details.setdefault("fix_quality", payload.get("gnss_fix_quality", "UNKNOWN"))
        details.setdefault("left_rtk_label", payload.get("gnss_left_rtk_label", "UNKNOWN"))
        details.setdefault("right_rtk_label", payload.get("gnss_right_rtk_label", "UNKNOWN"))
        details.setdefault("left_fix_type", payload.get("gnss_left_fix_type"))
        details.setdefault("right_fix_type", payload.get("gnss_right_fix_type"))
        details.setdefault("left_age_s", payload.get("gnss_left_age_s"))
        details.setdefault("right_age_s", payload.get("gnss_right_age_s"))
        details["position_evidence"] = "explicit finite NED baseline override"
    else:
        if observation is None:
            raise RuntimeError(
                "frozen Mountain image has no capture-associated GNSS baseline evidence; "
                "capture a burst with GNSS evidence, or supply an explicit "
                "gnss_baseline_ned_m override"
            )
        gnss = observation.get("gnss")
        gnss = gnss if isinstance(gnss, Mapping) else {}
        left_row = gnss.get("left")
        right_row = gnss.get("right")
        left_state = _gps_state_from_capture_mapping(left_row)
        right_state = _gps_state_from_capture_mapping(right_row)
        if left_state is None or right_state is None:
            raise RuntimeError(
                "frozen Mountain image has no capture-associated GNSS baseline evidence "
                f"(capture observation {observation.get('observation_id')} is missing "
                "left/right GNSS position)"
            )
        if not _finite_gps_position(left_state) or not _finite_gps_position(right_state):
            raise RuntimeError(
                "frozen Mountain image has no usable capture-associated GNSS baseline "
                f"evidence (capture observation {observation.get('observation_id')} has "
                "non-finite left/right GNSS position)"
            )
        delta = ned_delta_m(left_state, right_state)
        if delta is None:
            raise RuntimeError(
                "frozen Mountain image has no capture-associated GNSS baseline evidence "
                "(left/right GNSS position incomplete)"
            )
        baseline_ned = _finite_vector(
            delta,
            length=3,
            name="capture-associated gnss_baseline_ned_m",
        )
        source = f"capture_observation:{observation.get('observation_id')}"
        left_label = str((left_row or {}).get("rtk_label") or "UNKNOWN")
        right_label = str((right_row or {}).get("rtk_label") or "UNKNOWN")
        details = {
            "capture_observation_id": observation.get("observation_id"),
            "left_rtk_label": left_label,
            "right_rtk_label": right_label,
            "left_fix_type": left_state.fix_type,
            "right_fix_type": right_state.fix_type,
            "fix_quality": ("RTK_FIXED" if left_label == "RTK_FIXED" and right_label == "RTK_FIXED" else "DEGRADED"),
            "left_timestamp_s": (left_row or {}).get("timestamp_s"),
            "right_timestamp_s": (right_row or {}).get("timestamp_s"),
            "left_eph_m": (left_row or {}).get("eph_m"),
            "right_eph_m": (right_row or {}).get("eph_m"),
            "left_epv_m": (left_row or {}).get("epv_m"),
            "right_epv_m": (right_row or {}).get("epv_m"),
            "position_evidence": "same frozen-burst capture observation",
        }

    baseline_m = math.sqrt(sum(value * value for value in baseline_ned))
    if baseline_m <= 1.0e-9:
        raise ValueError("GNSS + Mountain requires a non-zero finite GNSS baseline")
    left_fix_type = _mountain_fix_type(
        details.get("left_fix_type"),
        rtk_label=details.get("left_rtk_label"),
    )
    right_fix_type = _mountain_fix_type(
        details.get("right_fix_type"),
        rtk_label=details.get("right_rtk_label"),
    )
    fix_quality = str(details.get("fix_quality") or "UNKNOWN").upper()
    apply_eligible = (
        left_fix_type is not None
        and left_fix_type >= _MOUNTAIN_MINIMUM_GNSS_FIX_TYPE
        and right_fix_type is not None
        and right_fix_type >= _MOUNTAIN_MINIMUM_GNSS_FIX_TYPE
    )
    gate_reasons = tuple(
        f"{side} fix_type {fix_type!r} < {_MOUNTAIN_MINIMUM_GNSS_FIX_TYPE}"
        for side, fix_type in (("left", left_fix_type), ("right", right_fix_type))
        if fix_type is None or fix_type < _MOUNTAIN_MINIMUM_GNSS_FIX_TYPE
    )
    details.update(
        {
            "source": source,
            "ned_m": list(baseline_ned),
            "baseline_m": baseline_m,
            "fix_quality": fix_quality,
            "left_fix_type": left_fix_type,
            "right_fix_type": right_fix_type,
            "minimum_fix_type": _MOUNTAIN_MINIMUM_GNSS_FIX_TYPE,
            "apply_eligible": True,
            "translation_gate_reasons": [],
            "quality_warnings": list(gate_reasons),
            "quality_passed": apply_eligible,
            "coordinate_contract": "right GNSS antenna minus left GNSS antenna; world NED metres",
        }
    )
    return baseline_ned, details


def frozen_pair_simultaneity(frozen):
    """Return (verified, |right-left| delta s, reason) for the frozen pair.

    ``verified`` is ``True`` when both artifacts share the app-receive
    clock and the delta is within ``CLOUD_FRAME_PAIR_MAX_DELTA_S``,
    ``False`` when the delta is known and exceeds it, and ``None`` when
    the clock domain cannot be confirmed (evidence only, not a gate).
    """

    artifacts = []
    for side in ("left", "right"):
        row = frozen.get(side)
        artifact = row.get("artifact") if isinstance(row, Mapping) else None
        if artifact is None:
            return None, None, f"{side} frozen artifact unavailable"
        artifacts.append(artifact)
    left_artifact, right_artifact = artifacts
    if not (_frame_clock_domain_verified(left_artifact) and _frame_clock_domain_verified(right_artifact)):
        return None, None, "frame clock domain not confirmed as app-receive"
    try:
        delta_s = abs(float(right_artifact.timestamp_s) - float(left_artifact.timestamp_s))
    except (TypeError, ValueError):
        return None, None, "frame timestamps unavailable"
    if not math.isfinite(delta_s):
        return None, None, "frame timestamps non-finite"
    if delta_s > CLOUD_FRAME_PAIR_MAX_DELTA_S:
        return (
            False,
            delta_s,
            (f"left/right frame delta {delta_s:.3f}s exceeds {CLOUD_FRAME_PAIR_MAX_DELTA_S:.3f}s"),
        )
    return True, delta_s, ""
