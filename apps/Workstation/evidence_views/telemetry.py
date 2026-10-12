"""Pure telemetry evidence projections from explicit captured values.

Attitude remains wxyz body FRD -> NED, geodetic positions remain WGS84
angular degrees/heights in metres, and detection coordinates remain u/v
pixels. now is host monotonic seconds supplied by the boundary.
"""

from __future__ import annotations

from collections.abc import Mapping
from Workstation.workstation_core.coordinate_contract import BODY_FRAME, WORLD_FRAME
from .values import _number, _integer, _age, _safe_len

ATTITUDE_FIELDS = (
    "mavlink_message_id",
    "mavlink_sequence",
    "mavlink_source_component",
    "mavlink_source_system",
    "pitch_deg",
    "px4_timestamp_s",
    "px4_timestamp_sample_us",
    "px4_timestamp_us",
    "q",
    "roll_deg",
    "timestamp_s",
    "topic",
    "yaw_deg",
)
GPS_FIELDS = (
    "altitude_ellipsoid_m",
    "altitude_m",
    "eph_m",
    "epv_m",
    "fix_type",
    "gps_utc_us",
    "hdop",
    "heading_deg",
    "latitude_deg",
    "longitude_deg",
    "mavlink_message_id",
    "mavlink_sequence",
    "mavlink_source_component",
    "mavlink_source_system",
    "px4_timestamp_s",
    "px4_timestamp_sample_us",
    "px4_timestamp_us",
    "rtk_label",
    "satellites_used",
    "timestamp_s",
    "topic",
    "vdop",
)
DETECTION_FIELDS = (
    "candidate_class_ids",
    "center",
    "class_id",
    "class_name",
    "confidence",
    "height",
    "timestamp_s",
    "width",
    "x1",
    "x2",
    "y1",
    "y2",
)


def project_attitude(
    side: str, *, state: Mapping[str, object] | None, now: float, provenance: Mapping[str, object]
) -> dict[str, object]:
    if state is None:
        return {
            "side": side,
            "state": "WAITING",
            "age_s": None,
            "frame_contract": "PX4 vehicle attitude quaternion rotates body FRD to world NED",
            "source": "MAVLink attitude telemetry",
        }
    return {
        "side": side,
        "state": "LIVE",
        "age_s": _age(state.get("timestamp_s", None), now),
        "roll_deg": _number(state.get("roll_deg", None)),
        "pitch_deg": _number(state.get("pitch_deg", None)),
        "yaw_deg": _number(state.get("yaw_deg", None)),
        "q_wxyz": tuple(state.get("q", ()) or ()),
        "app_timestamp_s": _number(state.get("timestamp_s", None)),
        "px4_timestamp_s": _number(state.get("px4_timestamp_s", None)),
        "px4_timestamp_us": _integer(state.get("px4_timestamp_us", None)),
        "px4_timestamp_sample_us": _integer(state.get("px4_timestamp_sample_us", None)),
        "mavlink_source_system": _integer(state.get("mavlink_source_system", None)),
        "mavlink_source_component": _integer(state.get("mavlink_source_component", None)),
        "mavlink_sequence": _integer(state.get("mavlink_sequence", None)),
        "mavlink_message_id": _integer(state.get("mavlink_message_id", None)),
        "source": "MAVLink attitude telemetry / CameraExtrinsicState",
        "clock": "app receive monotonic; PX4 timestamp when present",
        "frame_contract": "PX4 vehicle attitude quaternion rotates body FRD to world NED",
        "orientation_contract": f"orientation: {BODY_FRAME} → {WORLD_FRAME}; roll/pitch/yaw are degrees",
        **provenance,
    }


def project_gps(
    side: str,
    *,
    state: Mapping[str, object] | None,
    now: float,
    source: str,
    provenance: Mapping[str, object],
    frozen: Mapping[str, object] | None,
) -> dict[str, object]:
    if state is None:
        return {
            "side": side,
            "receiver_source": source,
            "state": "NO DATA",
            "source": "GpsPositionState / GPS receiver",
            "position_contract": "WGS84 latitude/longitude degrees; altitude metres",
        }
    fix_type = _integer(state.get("fix_type", None))
    rtk_label = str(state.get("rtk_label", "UNKNOWN") or "UNKNOWN")
    return {
        "side": side,
        "receiver_source": source,
        "state": rtk_label,
        "rtk_label": rtk_label,
        "position_frozen": frozen is not None,
        "position_frozen_packets": None if frozen is None else _integer(frozen.get("packets")),
        "position_frozen_cause": None
        if frozen is None
        else "autopilot re-sending one stale fix (GPS UTC frozen)"
        if frozen.get("utc_frozen")
        else "receiver position output frozen while GPS UTC advances",
        "fix_type": fix_type,
        "satellites": _integer(state.get("satellites_used", None)),
        "eph_m": _number(state.get("eph_m", None)),
        "epv_m": _number(state.get("epv_m", None)),
        "hdop": _number(state.get("hdop", None)),
        "vdop": _number(state.get("vdop", None)),
        "h_acc_m": _number(state.get("eph_m", None)),
        "v_acc_m": _number(state.get("epv_m", None)),
        "latitude_deg": _number(state.get("latitude_deg", None)),
        "longitude_deg": _number(state.get("longitude_deg", None)),
        "altitude_m": _number(state.get("altitude_m", None)),
        "altitude_ellipsoid_m": _number(state.get("altitude_ellipsoid_m", None)),
        "heading_deg": _number(state.get("heading_deg", None)),
        "timestamp_s": _number(state.get("timestamp_s", None)),
        "age_s": _age(state.get("timestamp_s", None), now),
        "px4_timestamp_s": _number(state.get("px4_timestamp_s", None)),
        "px4_timestamp_us": _integer(state.get("px4_timestamp_us", None)),
        "px4_timestamp_sample_us": _integer(state.get("px4_timestamp_sample_us", None)),
        "gps_utc_us": _integer(state.get("gps_utc_us", None)),
        "mavlink_source_system": _integer(state.get("mavlink_source_system", None)),
        "mavlink_source_component": _integer(state.get("mavlink_source_component", None)),
        "mavlink_sequence": _integer(state.get("mavlink_sequence", None)),
        "mavlink_message_id": _integer(state.get("mavlink_message_id", None)),
        "source": "MAVLink GPS telemetry / GpsPositionState",
        "clock": "GPS app receive monotonic; PX4/GPS source timestamps retained",
        "position_contract": "WGS84 latitude/longitude degrees; altitude metres",
        **provenance,
    }


def project_detection(
    side: str,
    *,
    observation: Mapping[str, object] | None,
    stats: Mapping[str, object],
    metadata: Mapping[str, object],
    enabled: bool,
    now: float,
) -> dict[str, object]:
    result: dict[str, object] = {
        "side": side,
        "runtime_state": "TARGET PRESENT" if observation is not None else "NO TARGET",
        "enabled": bool(enabled),
        "decoded_hz": _number(stats.get("decoded_hz")),
        "inferred_hz": _number(stats.get("inferred_hz")),
        "displayed_hz": _number(stats.get("displayed_hz")),
        "actual_device": str(stats.get("yolo_actual_device", "") or ""),
        "requested_device": str(stats.get("yolo_requested_device", "") or ""),
        "cpu_fallback": bool(stats.get("yolo_cpu_fallback", False)),
        "source": "DetectionObservation / YoloStreamWorker stats",
        "clock": "detection app receive monotonic; source timestamp metadata retained",
        "pixel_contract": "bbox x1/y1/x2/y2 and center are image pixels (u/v), not metres",
    }
    if observation is None:
        return result
    result.update(
        {
            "timestamp_s": _number(observation.get("timestamp_s", None)),
            "age_s": _age(observation.get("timestamp_s", None), now),
            "width_px": _integer(observation.get("width", None)),
            "height_px": _integer(observation.get("height", None)),
            "x1_px": _number(observation.get("x1", None)),
            "y1_px": _number(observation.get("y1", None)),
            "x2_px": _number(observation.get("x2", None)),
            "y2_px": _number(observation.get("y2", None)),
            "center_u_px": _number(observation.get("center", (None, None))[0]),
            "center_v_px": _number(observation.get("center", (None, None))[1]),
            "confidence": _number(observation.get("confidence", None)),
            "class_id": _integer(observation.get("class_id", None)),
            "class_name": str(observation.get("class_name", "") or ""),
            "candidate_count": _safe_len(observation.get("candidate_class_ids", ())),
            "source_timestamp_s": _number(metadata.get("source_timestamp_s")),
            "timestamp_source": str(metadata.get("timestamp_source", "") or ""),
            "timestamp_reference": str(metadata.get("timestamp_reference", "") or ""),
        }
    )
    return result
