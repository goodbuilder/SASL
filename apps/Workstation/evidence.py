from __future__ import annotations

"""Read-only projections of the existing StereoWIRIS panel state.

The workstation is a presentation shell.  This module deliberately keeps the
projection boundary boring: it reads existing model objects and widgets,
retains their timestamp/frame contracts, and never performs a measurement,
coordinate conversion, save, or calibration operation.
"""

import time
import csv
import json
from pathlib import Path
from typing import Any, Mapping

from Workstation.workstation_core.attitude_geometry import relative_attitude

from Workstation.workstation_core.coordinate_contract import (
    BASELINE_TRANSLATION_FRAME,
    BODY_FRAME,
    CAMERA_FRAME,
    MEASUREMENT_AXIS_CONTRACT,
    MEASUREMENT_FRAME,
    WORLD_FRAME,
)


def _widget_text(widget: object | None, default: str = "") -> str:
    method = getattr(widget, "text", None)
    if callable(method):
        try:
            return str(method())
        except Exception:
            return default
    return default


def _widget_value(widget: object | None, default: object = None) -> object:
    method = getattr(widget, "value", None)
    if callable(method):
        try:
            return method()
        except Exception:
            return default
    return default


def _widget_checked(widget: object | None, default: bool = False) -> bool:
    method = getattr(widget, "isChecked", None)
    if callable(method):
        try:
            return bool(method())
        except Exception:
            return default
    return default


def _combo_data(widget: object | None, default: object = None) -> object:
    method = getattr(widget, "currentData", None)
    if callable(method):
        try:
            return method()
        except Exception:
            return default
    return default


def _number(value: object, default: float | None = None) -> float | None:
    if value in (None, ""):
        return default
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _integer(value: object, default: int | None = None) -> int | None:
    if value in (None, ""):
        return default
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def _age(timestamp_s: object, now_s: float) -> float | None:
    value = _number(timestamp_s)
    if value is None:
        return None
    return max(0.0, now_s - value)


def _safe_len(value: object) -> int:
    try:
        return len(value)  # type: ignore[arg-type]
    except (TypeError, AttributeError):
        return 0


def _matrix_value(matrix: object, row: int, column: int) -> float | None:
    try:
        return float(matrix[row][column])  # type: ignore[index]
    except (IndexError, KeyError, TypeError, ValueError):
        return None


def mavlink_link_provenance(panel: object, side: str) -> dict[str, object]:
    """Return link identity without changing any telemetry value semantics."""

    subscriber = getattr(panel, "mavlink_pixhawk", None)
    if subscriber is None:
        return {}
    urls = getattr(subscriber, "urls", {}) or {}
    connections = getattr(subscriber, "connections", {}) or {}
    connection = connections.get(side) if isinstance(connections, dict) else None
    heartbeat_sides = getattr(subscriber, "_heartbeat_logged", set()) or set()

    def optional_id(value: object) -> int | None:
        try:
            parsed = int(value)
        except (TypeError, ValueError):
            return None
        return parsed if parsed > 0 else None

    return {
        "telemetry_transport": "MAVLink",
        "mavlink_endpoint": str(urls.get(side, "") or "") if isinstance(urls, dict) else "",
        "mavlink_link_open": connection is not None,
        "mavlink_heartbeat_seen": side in heartbeat_sides,
        "mavlink_target_system": optional_id(getattr(connection, "target_system", None)),
        "mavlink_target_component": optional_id(getattr(connection, "target_component", None)),
        "mavlink_identity_contract": (
            "endpoint + target system/component identify the link; packet source system/component/sequence "
            "are recorded when available; values retain producer frames/units"
        ),
    }


class EvidenceAdapter:
    """Expose source-backed, UI-safe snapshots for one panel owner."""

    def __init__(self, panel: object) -> None:
        self.panel = panel

    def _app_root(self) -> Path:
        """Return the data root owned by this panel's application runtime."""

        configured = getattr(self.panel, "app_root", None)
        if configured not in (None, ""):
            return Path(configured)
        # Lightweight test owners may omit app_root; the repository-owned
        # Workstation core root is the only fallback.
        from Workstation.workstation_core.settings import APP_ROOT

        return Path(APP_ROOT)

    def attitude(self, side: str, *, now_s: float | None = None) -> dict[str, object]:
        now = time.monotonic() if now_s is None else float(now_s)
        states = getattr(self.panel, "latest_extrinsics", {}) or {}
        state = states.get(side) if isinstance(states, dict) else None
        if state is None:
            return {
                "side": side,
                "state": "WAITING",
                "age_s": None,
                "frame_contract": "PX4 vehicle attitude quaternion rotates body FRD to world NED",
                "source": "MAVLink attitude telemetry",
            }
        provenance = (
            mavlink_link_provenance(self.panel, side)
            if str(getattr(state, "topic", "")).startswith("mavlink/")
            else {}
        )
        return {
            "side": side,
            "state": "LIVE",
            "age_s": _age(getattr(state, "timestamp_s", None), now),
            "roll_deg": _number(getattr(state, "roll_deg", None)),
            "pitch_deg": _number(getattr(state, "pitch_deg", None)),
            "yaw_deg": _number(getattr(state, "yaw_deg", None)),
            "q_wxyz": tuple(getattr(state, "q", ()) or ()),
            "app_timestamp_s": _number(getattr(state, "timestamp_s", None)),
            "px4_timestamp_s": _number(getattr(state, "px4_timestamp_s", None)),
            "px4_timestamp_us": _integer(getattr(state, "px4_timestamp_us", None)),
            "px4_timestamp_sample_us": _integer(getattr(state, "px4_timestamp_sample_us", None)),
            "mavlink_source_system": _integer(getattr(state, "mavlink_source_system", None)),
            "mavlink_source_component": _integer(getattr(state, "mavlink_source_component", None)),
            "mavlink_sequence": _integer(getattr(state, "mavlink_sequence", None)),
            "mavlink_message_id": _integer(getattr(state, "mavlink_message_id", None)),
            "source": "MAVLink attitude telemetry / CameraExtrinsicState",
            "clock": "app receive monotonic; PX4 timestamp when present",
            "frame_contract": "PX4 vehicle attitude quaternion rotates body FRD to world NED",
            "orientation_contract": f"orientation: {BODY_FRAME} → {WORLD_FRAME}; roll/pitch/yaw are degrees",
            **provenance,
        }

    def gps(self, side: str, *, now_s: float | None = None) -> dict[str, object]:
        now = time.monotonic() if now_s is None else float(now_s)
        states = getattr(self.panel, "latest_gps", {}) or {}
        state = states.get(side) if isinstance(states, dict) else None
        if state is None:
            return {
                "side": side,
                "state": "NO DATA",
                "source": "GpsPositionState / GPS receiver",
                "position_contract": "WGS84 latitude/longitude degrees; altitude metres",
            }
        provenance = (
            mavlink_link_provenance(self.panel, side)
            if str(getattr(state, "topic", "")).startswith("mavlink/")
            else {}
        )
        fix_type = _integer(getattr(state, "fix_type", None))
        rtk_label = str(getattr(state, "rtk_label", "UNKNOWN") or "UNKNOWN")
        return {
            "side": side,
            "state": rtk_label,
            "rtk_label": rtk_label,
            "fix_type": fix_type,
            "satellites": _integer(getattr(state, "satellites_used", None)),
            "eph_m": _number(getattr(state, "eph_m", None)),
            "epv_m": _number(getattr(state, "epv_m", None)),
            "latitude_deg": _number(getattr(state, "latitude_deg", None)),
            "longitude_deg": _number(getattr(state, "longitude_deg", None)),
            "altitude_m": _number(getattr(state, "altitude_m", None)),
            "altitude_ellipsoid_m": _number(getattr(state, "altitude_ellipsoid_m", None)),
            "heading_deg": _number(getattr(state, "heading_deg", None)),
            "timestamp_s": _number(getattr(state, "timestamp_s", None)),
            "age_s": _age(getattr(state, "timestamp_s", None), now),
            "px4_timestamp_s": _number(getattr(state, "px4_timestamp_s", None)),
            "px4_timestamp_us": _integer(getattr(state, "px4_timestamp_us", None)),
            "px4_timestamp_sample_us": _integer(getattr(state, "px4_timestamp_sample_us", None)),
            "gps_utc_us": _integer(getattr(state, "gps_utc_us", None)),
            "mavlink_source_system": _integer(getattr(state, "mavlink_source_system", None)),
            "mavlink_source_component": _integer(getattr(state, "mavlink_source_component", None)),
            "mavlink_sequence": _integer(getattr(state, "mavlink_sequence", None)),
            "mavlink_message_id": _integer(getattr(state, "mavlink_message_id", None)),
            "source": "MAVLink GPS telemetry / GpsPositionState",
            "clock": "GPS app receive monotonic; PX4/GPS source timestamps retained",
            "position_contract": "WGS84 latitude/longitude degrees; altitude metres",
            **provenance,
        }

    def baseline(self) -> dict[str, object]:
        baseline = getattr(self.panel, "latest_rtk_baseline", None)
        projection = getattr(self.panel, "latest_p1_body_frd_baseline", None)
        quality = getattr(self.panel, "latest_baseline_quality", None)
        quality_values: dict[str, object] = {
            "sample_count": _integer(getattr(quality, "sample_count", None)),
            "fixed_ratio": _number(getattr(quality, "rtk_fixed_ratio", None)),
            "length_std_m": _number(getattr(quality, "baseline_length_std_m", None)),
            "timestamp_delta_ms": _number(getattr(quality, "timestamp_delta_ms", None)),
            "reasons": tuple(getattr(quality, "reasons", ()) or ()),
        }
        projection_values: dict[str, object] = {
            "p1_body_frd_state": "WAITING",
            "p1_body_frd_forward_m": None,
            "p1_body_frd_right_m": None,
            "p1_body_frd_down_m": None,
            "p1_body_frd_attitude_age_s": None,
            "p1_body_frd_source": "P1 attitude-derived projection; waiting for P1 body FRD attitude",
            "p1_body_frd_frame": BODY_FRAME,
            "p1_body_frd_axis_contract": "P1->P2 in P1 body FRD: [Forward, Right, Down]; metres",
        }
        if projection is not None:
            projection_values.update(
                {
                    "p1_body_frd_state": "READY",
                    "p1_body_frd_forward_m": _number(getattr(projection, "forward_m", None)),
                    "p1_body_frd_right_m": _number(getattr(projection, "right_m", None)),
                    "p1_body_frd_down_m": _number(getattr(projection, "down_m", None)),
                    "p1_body_frd_attitude_age_s": _number(getattr(projection, "attitude_age_s", None)),
                    "p1_body_frd_source": "BodyFrdBaselineProjection from raw NED + P1 attitude",
                    "p1_body_frd_frame": str(getattr(projection, "target_frame", BODY_FRAME)),
                    "p1_body_frd_axis_contract": str(
                        getattr(
                            projection,
                            "axis_contract",
                            "P1->P2 in P1 body FRD: [Forward, Right, Down]; metres",
                        )
                    ),
                }
            )
        if baseline is None:
            return {
                "state": "NO DATA",
                "source": "RtkBaselineEstimate from left/right GpsPositionState",
                "axis_contract": "NED = north / east / down; metres",
                "quality": quality_values,
                "left_fix_type": _integer(getattr((getattr(self.panel, "latest_gps", {}) or {}).get("left"), "fix_type", None)),
                "right_fix_type": _integer(getattr((getattr(self.panel, "latest_gps", {}) or {}).get("right"), "fix_type", None)),
                **projection_values,
            }
        return {
            "state": str(getattr(baseline, "fix_quality", "UNKNOWN") or "UNKNOWN"),
            "north_m": _number(getattr(baseline, "north_m", None)),
            "east_m": _number(getattr(baseline, "east_m", None)),
            "down_m": _number(getattr(baseline, "down_m", None)),
            "length_m": _number(getattr(baseline, "baseline_m", None)),
            "left_age_s": _number(getattr(baseline, "left_age_s", None)),
            "right_age_s": _number(getattr(baseline, "right_age_s", None)),
            "warnings": tuple(getattr(baseline, "warnings", ()) or ()),
            "source": "RtkBaselineEstimate from left/right GpsPositionState",
            "axis_contract": "NED = north / east / down; metres",
            "position_frame": "GPS/RTK WGS84 delta projected to local NED; not camera or FRD",
            "quality": quality_values,
            "left_fix_type": _integer(getattr((getattr(self.panel, "latest_gps", {}) or {}).get("left"), "fix_type", None)),
            "right_fix_type": _integer(getattr((getattr(self.panel, "latest_gps", {}) or {}).get("right"), "fix_type", None)),
            **projection_values,
        }

    def gnss_operation(self, *, now_s: float | None = None) -> dict[str, object]:
        """Project Base GNSS, Survey-in, NTRIP, and RTCM evidence.

        Base coordinates retain their producer WGS84/height contract. Survey-in
        ECEF means are explicitly named and never moved into NED, FRD, or camera
        fields. Rover position and baseline evidence remain owned by ``gps`` and
        ``baseline`` respectively.
        """

        now = time.monotonic() if now_s is None else float(now_s)
        stats = getattr(self.panel, "gnss_base_rtcm_stats", {}) or {}
        stats = stats if isinstance(stats, Mapping) else {}
        injection = stats.get("injection", {})
        injection = injection if isinstance(injection, Mapping) else {}
        routes: dict[str, dict[str, object]] = {}
        for side in ("left", "right"):
            route = injection.get(side, {})
            route = route if isinstance(route, Mapping) else {}
            last_delivery = _number(route.get("last_delivery_s"))
            routes[side] = {
                "frames": _integer(route.get("frames"), 0),
                "bytes": _integer(route.get("bytes"), 0),
                "packets": _integer(route.get("packets"), 0),
                "fail": _integer(route.get("fail"), 0),
                "delivery_age_s": None if last_delivery is None else max(0.0, now - last_delivery),
            }
        manager = getattr(self.panel, "gnss_operation_manager", None)
        metadata_method = getattr(manager, "metadata", None)
        try:
            metadata = metadata_method() if callable(metadata_method) else {}
        except Exception:
            metadata = {}
        metadata = metadata if isinstance(metadata, Mapping) else {}
        config = getattr(manager, "config", None)
        config_method = getattr(config, "to_mapping", None)
        try:
            config_values = config_method() if callable(config_method) else {}
        except Exception:
            config_values = {}
        config_values = config_values if isinstance(config_values, Mapping) else {}
        local_config = config_values.get("local_base", {})
        local_config = local_config if isinstance(local_config, Mapping) else {}
        network_config = config_values.get("network", {})
        network_config = network_config if isinstance(network_config, Mapping) else {}
        fixed_config = config_values.get("fixed_base", {})
        fixed_config = fixed_config if isinstance(fixed_config, Mapping) else {}

        candidate_rows = getattr(self.panel, "_gnss_base_candidate_rows", ()) or ()
        candidates: list[dict[str, object]] = []
        for row in candidate_rows:
            if not isinstance(row, Mapping):
                continue
            messages = row.get("messages", "")
            if isinstance(messages, (tuple, list)):
                messages = ", ".join(str(message) for message in messages)
            candidates.append(
                {
                    "port": str(row.get("port", "") or ""),
                    "target": str(row.get("target", "") or ""),
                    "baud": _integer(row.get("baud")),
                    "status": str(row.get("status", "") or ""),
                    "messages": str(messages or ""),
                    "ubx": bool(row.get("ubx", False)),
                }
            )

        source_identity = metadata.get("correction_source")
        source_identity = source_identity if isinstance(source_identity, Mapping) else {}
        survey = getattr(self.panel, "latest_gnss_base_survey_in", None)
        survey = survey if isinstance(survey, Mapping) else {}
        survey_samples = getattr(self.panel, "gnss_base_survey_samples", ()) or ()
        if not survey and survey_samples:
            try:
                _sample_time, duration_s, accuracy_m, active, valid = survey_samples[-1]
                survey = {
                    "duration_s": duration_s,
                    "mean_accuracy_m": accuracy_m,
                    "active": active,
                    "valid": valid,
                }
            except (IndexError, TypeError, ValueError):
                survey = {}

        coordinate_samples = getattr(self.panel, "gnss_coordinate_samples", ()) or ()
        latest_coordinate = None
        try:
            if coordinate_samples:
                latest_coordinate = coordinate_samples[-1]
        except (IndexError, TypeError):
            latest_coordinate = None
        coordinate = {
            "latitude_deg": _number(getattr(latest_coordinate, "latitude_deg", None)),
            "longitude_deg": _number(getattr(latest_coordinate, "longitude_deg", None)),
            "ellipsoid_height_m": _number(getattr(latest_coordinate, "ellipsoid_height_m", None)),
            "msl_altitude_m": _number(getattr(latest_coordinate, "msl_altitude_m", None)),
            "selected_height_m": _number(getattr(latest_coordinate, "selected_height_m", None)),
            "height_type": str(getattr(getattr(latest_coordinate, "height_type", None), "value", "UNKNOWN")),
            "fixed": None if latest_coordinate is None else bool(getattr(latest_coordinate, "fixed", False)),
            "device_id": str(getattr(latest_coordinate, "device_id", "") or ""),
            "age_s": _age(getattr(latest_coordinate, "timestamp_s", None), now),
            "position_contract": "WGS84 latitude/longitude degrees; heights metres with explicit height type",
        }

        profile = getattr(self.panel, "gnss_fixed_base_profile", None)
        profile_method = getattr(profile, "to_mapping", None)
        try:
            profile_values = profile_method() if callable(profile_method) else {}
        except Exception:
            profile_values = {}
        profile_values = profile_values if isinstance(profile_values, Mapping) else {}
        readiness = None
        readiness_method = getattr(manager, "readiness", None)
        if callable(readiness_method):
            try:
                readiness = readiness_method()
            except Exception:
                readiness = None
        state = getattr(manager, "state", "UNKNOWN")
        state = str(getattr(state, "value", state) or "UNKNOWN")
        elapsed_s = max(0.0, now - float(stats.get("start_s") or now))
        return {
            "state": state,
            "mode": str(metadata.get("gnss_operation_mode", config_values.get("operation_mode", "UNKNOWN"))),
            "experiment_site": {
                "label": str(config_values.get("site_label", "") or ""),
                "address": str(config_values.get("site_address", "") or ""),
                "coordinate_contract": "WGS84 geodetic latitude/longitude; Base height is explicit ellipsoid/MSL",
            },
            "source_identity": dict(source_identity),
            "invalidation_reason": str(metadata.get("invalidation_reason", "") or ""),
            "base_serial": {
                "connected": bool(getattr(self.panel, "gnss_base_serial_connected", False)),
                "device": str(source_identity.get("base_device", local_config.get("device", "")) or ""),
                "baud": _integer(local_config.get("baud")),
            },
            "base_candidates": tuple(candidates),
            "base_discovery_active": getattr(self.panel, "gnss_base_detect_thread", None) is not None,
            "base_discovery_status": str(
                getattr(self.panel, "workstation_gnss_base_discovery_status", "NOT RUN") or "NOT RUN"
            ),
            "survey_in": {
                "active": bool(survey.get("active", False)),
                "valid": bool(survey.get("valid", False)),
                "duration_s": _integer(survey.get("duration_s"), 0),
                "target_duration_s": _number(local_config.get("survey_in_min_duration_s")),
                "mean_accuracy_m": _number(survey.get("mean_accuracy_m")),
                "target_accuracy_m": _number(local_config.get("survey_in_accuracy_limit_m")),
                "observations": _integer(survey.get("observations")),
                "mean_ecef_m": (
                    _number(survey.get("mean_x_m")),
                    _number(survey.get("mean_y_m")),
                    _number(survey.get("mean_z_m")),
                ),
                "axis_contract": "ECEF [X,Y,Z] metres from UBX NAV-SVIN; not NED, FRD, or camera RDF",
            },
            "ntrip": {
                "connected": bool(getattr(self.panel, "gnss_network_connected", False)),
                "status": str(getattr(self.panel, "gnss_network_status_line", "") or ""),
                "caster_host": str(source_identity.get("caster_host", network_config.get("caster_host", "")) or ""),
                "caster_port": _integer(source_identity.get("caster_port", network_config.get("caster_port"))),
                "mountpoint": str(source_identity.get("mountpoint", network_config.get("mountpoint", "")) or ""),
                "gga_policy": str(source_identity.get("gga_policy", network_config.get("gga_policy", "")) or ""),
                "tls": bool(network_config.get("tls", False)),
                "username_env": str(network_config.get("username_env", "") or ""),
                "password_env": str(network_config.get("password_env", "") or ""),
            },
            "base_coordinate": coordinate,
            "fixed_base": {
                "sample_count": _safe_len(coordinate_samples),
                "profile": dict(profile_values),
                "target_acquisition_duration_s": _number(fixed_config.get("acquisition_duration_s")),
                "target_fixed_duration_s": _number(fixed_config.get("minimum_fixed_duration_s")),
                "target_fixed_ratio": _number(fixed_config.get("minimum_fixed_ratio")),
                "maximum_horizontal_scatter_m": _number(fixed_config.get("maximum_horizontal_scatter_m")),
                "maximum_vertical_scatter_m": _number(fixed_config.get("maximum_vertical_scatter_m")),
                "height_type": str(fixed_config.get("height_type", "UNKNOWN") or "UNKNOWN"),
                "reconfigure_ack": bool(getattr(self.panel, "gnss_fixed_base_reconfigure_ack", False)),
                "reconfigure_readback": bool(getattr(self.panel, "gnss_fixed_base_reconfigure_readback", False)),
                "reconfigure_rtcm": bool(getattr(self.panel, "gnss_fixed_base_reconfigure_rtcm", False)),
            },
            "base_frames": _integer(stats.get("frames"), 0),
            "base_bytes": _integer(stats.get("bytes"), 0),
            "average_frame_rate_hz": float(stats.get("frames") or 0) / elapsed_s if elapsed_s > 0.0 else 0.0,
            "average_byte_rate_bps": float(stats.get("bytes") or 0) / elapsed_s if elapsed_s > 0.0 else 0.0,
            "last_frame_age_s": _age(stats.get("last_frame_s"), now),
            "routes": routes,
            "readiness": (
                readiness.to_metadata()
                if readiness is not None and callable(getattr(readiness, "to_metadata", None))
                else {}
            ),
            "contract": "RTCM base stream → MAVLink GPS_RTCM_DATA route; bytes/packets are producer counters",
        }

    def camera_connection_candidates(self) -> dict[str, object]:
        """Project discovered camera endpoints without exposing owner-only objects."""

        rows = getattr(self.panel, "_camera_candidate_rows", ()) or ()
        candidates: list[dict[str, object]] = []
        for row in rows:
            if not isinstance(row, Mapping):
                continue
            interface = row.get("interface")
            candidates.append(
                {
                    "kind": str(row.get("kind", "") or ""),
                    "address": str(row.get("address", "") or ""),
                    "host": str(row.get("host", "") or ""),
                    "port": str(row.get("port", "") or ""),
                    "status": str(row.get("status", "") or ""),
                    "interface": str(getattr(interface, "name", "") or ""),
                    "host_ip": str(row.get("host_ip", "") or ""),
                }
            )
        return {
            "candidates": tuple(candidates),
            "discovery_active": any(
                getattr(self.panel, name, None) is not None
                for name in ("camera_network_discovery_thread", "camera_auto_detect_thread")
            ),
            "status": str(getattr(self.panel, "workstation_camera_discovery_status", "NOT RUN") or "NOT RUN"),
            "contract": "Network/RTSP/control endpoint metadata; no image coordinates or camera-frame values",
        }

    def pixhawk_connection_candidates(self) -> dict[str, object]:
        """Project PC-interface, USB, and MAVLink candidates for explicit P1/P2 assignment."""

        grouped = getattr(self.panel, "_pixhawk_candidate_rows_by_source", None)
        if not isinstance(grouped, Mapping):
            grouped = {"mavlink": getattr(self.panel, "_pixhawk_candidate_rows", ()) or ()}
        candidates: list[dict[str, object]] = []
        for source in ("pc", "mavlink"):
            rows = grouped.get(source, ()) or ()
            for source_row, row in enumerate(rows):
                if not isinstance(row, Mapping):
                    continue
                candidates.append(
                    {
                        "source": source,
                        "source_row": source_row,
                        "kind": str(row.get("kind", "") or ""),
                        "value": str(row.get("value", "") or ""),
                        "baud": _integer(row.get("baud")),
                        "status": str(row.get("status", "") or ""),
                        "interface": str(row.get("interface", "") or ""),
                    }
                )
        return {
            "candidates": tuple(candidates),
            "discovery_active": any(
                getattr(self.panel, name, None) is not None
                for name in ("pixhawk_usb_discovery_thread", "pixhawk_host_ip_discovery_thread")
            ),
            "status": str(getattr(self.panel, "workstation_pixhawk_discovery_status", "NOT RUN") or "NOT RUN"),
            "contract": "Host/serial/MAVLink endpoint metadata; no attitude, position, or baseline values",
        }

    def pixhawk_recovery(self, side: str, *, now_s: float | None = None) -> dict[str, object]:
        """Project ETH MAV recovery controls and post-recovery telemetry evidence.

        The USB values identify the temporary PX4 shell/control transport. The
        telemetry values remain independent MAVLink receive evidence; this
        projection does not move or reinterpret attitude, GPS, or baseline
        coordinates.
        """

        if side not in {"left", "right"}:
            raise ValueError(f"Unknown Pixhawk side: {side}")
        now = time.monotonic() if now_s is None else float(now_s)
        panel = self.panel
        url_widget = getattr(panel, f"{side}_pixhawk_bootstrap_url_edit", None)
        baud_widget = getattr(panel, f"{side}_pixhawk_bootstrap_baud_spin", None)
        status_by_side = getattr(panel, "pixhawk_usb_bootstrap_status", {}) or {}
        record = status_by_side.get(side, {}) if isinstance(status_by_side, Mapping) else {}
        record = record if isinstance(record, Mapping) else {}
        active = getattr(panel, "pixhawk_usb_bootstrap_thread", None) is not None
        state = str(record.get("state", "IDLE") or "IDLE").upper()
        if active:
            state = "RUNNING"
        detail = str(record.get("detail", "") or "")
        updated_s = _number(record.get("updated_s"))
        provenance = mavlink_link_provenance(panel, side)
        attitude = self.attitude(side, now_s=now)
        attitude_age = _number(attitude.get("age_s"))
        attitude_ok = (
            str(attitude.get("state", "WAITING")).upper() == "LIVE"
            and attitude_age is not None
            and attitude_age < 2.0
        )
        link_open = bool(provenance.get("mavlink_link_open", False))
        heartbeat_ok = bool(provenance.get("mavlink_heartbeat_seen", False))
        heartbeat_state = "OK" if heartbeat_ok else "WAITING" if link_open else "NO LINK"
        attitude_state = "OK" if attitude_ok else "WAITING"
        if not detail:
            detail = {
                "IDLE": "대기",
                "RUNNING": "USB shell로 PX4 Ethernet MAVLink 시작 중",
                "RECOVERED": "PX4 Ethernet MAVLink 시작 완료; UDP telemetry 확인 중",
                "ERROR": "결과 없음",
            }.get(state, "결과 없음")
        return {
            "side": side,
            "usb_url": _widget_text(url_widget, "--") or "--",
            "usb_baud": _integer(_widget_value(baud_widget)),
            "state": state,
            "detail": detail,
            "updated_age_s": None if updated_s is None else max(0.0, now - updated_s),
            "active": active,
            "mavlink_link_open": link_open,
            "heartbeat_state": heartbeat_state,
            "attitude_state": attitude_state,
            "telemetry": f"Heartbeat {heartbeat_state} · ATTITUDE {attitude_state}",
            "mavlink_endpoint": str(provenance.get("mavlink_endpoint", "") or ""),
            "contract": "USB control path starts PX4 Ethernet MAVLink; telemetry remains MAVLink attitude evidence",
        }

    def physical_alignment(self) -> dict[str, object]:
        """Return physical-alignment evidence and iteration history."""

        snapshot_method = getattr(self.panel, "workstation_physical_alignment_snapshot", None)
        if callable(snapshot_method):
            try:
                snapshot = snapshot_method()
            except Exception:
                snapshot = None
            if isinstance(snapshot, Mapping):
                return dict(snapshot)

        quality = getattr(self.panel, "latest_physical_alignment_quality", None)
        quality_mapping = (
            quality.to_mapping()
            if quality is not None and callable(getattr(quality, "to_mapping", None))
            else dict(quality)
            if isinstance(quality, Mapping)
            else {}
        )
        left = (getattr(self.panel, "latest_extrinsics", {}) or {}).get("left")
        right = (getattr(self.panel, "latest_extrinsics", {}) or {}).get("right")
        relative_rpy: tuple[float, float, float] | None = None
        if left is not None and right is not None:
            try:
                relative = relative_attitude(left.q, right.q)
                relative_rpy = (relative.roll_deg, relative.pitch_deg, relative.yaw_deg)
            except (AttributeError, TypeError, ValueError):
                relative_rpy = None
        baseline = self.baseline()
        current_frd = tuple(
            baseline.get(key)
            for key in (
                "p1_body_frd_forward_m",
                "p1_body_frd_right_m",
                "p1_body_frd_down_m",
            )
        )
        if not all(value is not None for value in current_frd):
            current_frd = None
        gps = {side: self.gps(side) for side in ("left", "right")}
        return {
            "state": "LIVE" if relative_rpy is not None and baseline.get("state") not in {"NO DATA", "UNKNOWN"} else "WAITING",
            "sampling_active": bool(getattr(self.panel, "baseline_sampling_active", False)),
            "sample_count": _safe_len(getattr(self.panel, "physical_alignment_samples", ())),
            "current_baseline_ned_m": tuple(
                baseline.get(key) for key in ("north_m", "east_m", "down_m")
            ),
            "current_baseline_frd_m": current_frd,
            "relative_rpy_deg": relative_rpy,
            "fix_type": {side: gps[side].get("fix_type") for side in ("left", "right")},
            "quality": quality_mapping,
            "iterations": tuple(getattr(self.panel, "alignment_iteration_history", ()) or ()),
            "physical_adjustment_only": True,
            "coordinate_contract": (
                "raw baseline NED [north,east,down] is preserved; P1→P2 FRD [forward,right,down] "
                "is an attitude-derived observation; ΔR/P/Y is [roll,pitch,yaw]"
            ),
        }

    def detection(self, side: str, *, now_s: float | None = None) -> dict[str, object]:
        now = time.monotonic() if now_s is None else float(now_s)
        observation = getattr(self.panel, f"latest_{side}", None)
        stats_all = getattr(self.panel, "camera_stream_stats", {}) or {}
        stats = stats_all.get(side, {}) if isinstance(stats_all, dict) else {}
        stats = stats if isinstance(stats, dict) else {}
        result: dict[str, object] = {
            "side": side,
            "runtime_state": "TARGET PRESENT" if observation is not None else "NO TARGET",
            "enabled": bool(getattr(self.panel, "yolo_enabled", False)),
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
        metadata = getattr(observation, "timestamp_metadata", {}) or {}
        result.update(
            {
                "timestamp_s": _number(getattr(observation, "timestamp_s", None)),
                "age_s": _age(getattr(observation, "timestamp_s", None), now),
                "width_px": _integer(getattr(observation, "width", None)),
                "height_px": _integer(getattr(observation, "height", None)),
                "x1_px": _number(getattr(observation, "x1", None)),
                "y1_px": _number(getattr(observation, "y1", None)),
                "x2_px": _number(getattr(observation, "x2", None)),
                "y2_px": _number(getattr(observation, "y2", None)),
                "center_u_px": _number(getattr(observation, "center", (None, None))[0]),
                "center_v_px": _number(getattr(observation, "center", (None, None))[1]),
                "confidence": _number(getattr(observation, "confidence", None)),
                "class_id": _integer(getattr(observation, "class_id", None)),
                "class_name": str(getattr(observation, "class_name", "") or ""),
                "candidate_count": _safe_len(getattr(observation, "candidate_class_ids", ())),
                "source_timestamp_s": _number(metadata.get("source_timestamp_s")),
                "timestamp_source": str(metadata.get("timestamp_source", "") or ""),
                "timestamp_reference": str(metadata.get("timestamp_reference", "") or ""),
            }
        )
        return result

    def detection_config(self) -> dict[str, object]:
        settings = getattr(self.panel, "settings", None)

        def setting(name: str, default: object = None) -> object:
            value = getattr(settings, name, default)
            return default if value is None else value

        weights = _widget_text(getattr(self.panel, "weights_edit", None), str(setting("yolo_weights_path", "")))
        device = _combo_data(getattr(self.panel, "device_combo", None), setting("yolo_device", "cpu"))
        device = str(device or "cpu")
        gate_names = []
        gate_values = {
            "class": _widget_checked(getattr(self.panel, "yolo_gate_class_checkbox", None), bool(setting("yolo_gate_class_enabled", False))),
            "time": _widget_checked(getattr(self.panel, "yolo_gate_matching_checkbox", None), bool(setting("yolo_gate_matching_enabled", False))),
            "Δt": _widget_checked(getattr(self.panel, "yolo_gate_time_delta_checkbox", None), bool(setting("yolo_gate_time_delta_enabled", False))),
            "vertical Δ": _widget_checked(getattr(self.panel, "yolo_gate_vertical_delta_checkbox", None), bool(setting("yolo_gate_vertical_delta_enabled", False))),
        }
        gate_names.extend(name for name, enabled in gate_values.items() if enabled)
        return {
            "weights_path": weights,
            "device": device,
            "input_size_px": _integer(_widget_value(getattr(self.panel, "input_size_spin", None), setting("yolo_input_size"))),
            "confidence": _number(_widget_value(getattr(self.panel, "conf_spin", None), setting("yolo_conf_threshold"))),
            "iou": _number(_widget_value(getattr(self.panel, "iou_spin", None), setting("yolo_iou_threshold"))),
            "class_gate": _widget_text(getattr(self.panel, "yolo_gate_class_edit", None), str(setting("yolo_gate_class_text", ""))),
            "gate_enabled": tuple(gate_names),
            "max_delta_s": _number(_widget_value(getattr(self.panel, "yolo_gate_time_delta_spin", None), setting("yolo_gate_max_time_delta_s"))),
            "time_basis": str(_combo_data(getattr(self.panel, "yolo_gate_time_basis_combo", None), setting("yolo_gate_time_basis", "")) or ""),
            "max_vertical_delta_px": _number(_widget_value(getattr(self.panel, "yolo_gate_vertical_delta_spin", None), setting("yolo_gate_max_vertical_delta_px"))),
            "source": "AppSettings + existing YOLO configuration widgets",
        }

    def measurement_settings(self) -> dict[str, object]:
        """Project editable Measurement and Time/Sync settings.

        The values are read from the panel's existing controls and settings
        object.  Workstation may send this same shape back through the
        backend facade, but this projection never mutates the owner.
        Units are explicit: focal values are pixels, timing thresholds
        are seconds or milliseconds as labelled, and calibration/extrinsic
        selectors name their frame source rather than converting coordinates.
        """

        settings = getattr(self.panel, "settings", None)

        def setting(name: str, default: object = None) -> object:
            value = getattr(settings, name, default)
            return default if value is None else value

        intrinsics_source = _combo_data(
            getattr(self.panel, "intrinsics_source_combo", None),
            "manual" if _widget_checked(getattr(self.panel, "manual_focal_checkbox", None), bool(setting("manual_focal_override", False))) else "checkerboard",
        )
        distance_mode = _combo_data(
            getattr(self.panel, "distance_mode_combo", None),
            setting("triangulation_extrinsics_source", "pixhawk"),
        )
        return {
            "intrinsics_source": str(intrinsics_source or "checkerboard"),
            "manual_focal_override": _widget_checked(
                getattr(self.panel, "manual_focal_checkbox", None),
                bool(setting("manual_focal_override", False)),
            ),
            "focal_px": _number(
                _widget_value(getattr(self.panel, "focal_spin", None), None),
                900.0,
            ),
            "distance_mode": str(distance_mode or "pixhawk"),
            "yolo_input_size_px": _integer(
                _widget_value(getattr(self.panel, "input_size_spin", None), setting("yolo_input_size"))
            ),
            "yolo_confidence": _number(
                _widget_value(getattr(self.panel, "conf_spin", None), setting("yolo_conf_threshold"))
            ),
            "yolo_iou": _number(
                _widget_value(getattr(self.panel, "iou_spin", None), setting("yolo_iou_threshold"))
            ),
            "class_gate_enabled": _widget_checked(
                getattr(self.panel, "yolo_gate_class_checkbox", None),
                bool(setting("yolo_gate_class_enabled", False)),
            ),
            "class_gate": _widget_text(
                getattr(self.panel, "yolo_gate_class_edit", None),
                str(setting("yolo_gate_class_text", "")),
            ),
            "matching_gate_enabled": _widget_checked(
                getattr(self.panel, "yolo_gate_matching_checkbox", None),
                bool(setting("yolo_gate_matching_enabled", False)),
            ),
            "time_basis": str(
                _combo_data(
                    getattr(self.panel, "yolo_gate_time_basis_combo", None),
                    setting("yolo_gate_time_basis", "camera_rtcp"),
                )
                or "camera_rtcp"
            ),
            "time_delta_gate_enabled": _widget_checked(
                getattr(self.panel, "yolo_gate_time_delta_checkbox", None),
                bool(setting("yolo_gate_time_delta_enabled", False)),
            ),
            "max_pair_delta_s": _number(
                _widget_value(getattr(self.panel, "yolo_gate_time_delta_spin", None), setting("yolo_gate_max_time_delta_s"))
            ),
            "vertical_delta_gate_enabled": _widget_checked(
                getattr(self.panel, "yolo_gate_vertical_delta_checkbox", None),
                bool(setting("yolo_gate_vertical_delta_enabled", False)),
            ),
            "max_vertical_delta_px": _number(
                _widget_value(
                    getattr(self.panel, "yolo_gate_vertical_delta_spin", None),
                    setting("yolo_gate_max_vertical_delta_px"),
                )
            ),
            "camera_sync_history_window_s": _number(
                _widget_value(
                    getattr(self.panel, "camera_sync_history_window_spin", None),
                    setting("camera_sync_history_window_s"),
                )
            ),
            "camera_sync_receive_pair_max_delta_ms": _number(
                _widget_value(
                    getattr(self.panel, "camera_sync_receive_pair_max_delta_spin", None),
                    _number(setting("camera_sync_receive_pair_max_delta_s"), 0.0) * 1000.0,
                )
            ),
            "camera_sync_rtcp_pair_max_delta_ms": _number(
                _widget_value(
                    getattr(self.panel, "camera_sync_rtcp_pair_max_delta_spin", None),
                    _number(setting("camera_sync_rtcp_pair_max_delta_s"), 0.0) * 1000.0,
                )
            ),
            "camera_sync_max_sr_age_s": _number(
                _widget_value(
                    getattr(self.panel, "camera_sync_max_sr_age_spin", None),
                    setting("camera_sync_max_sr_age_s"),
                )
            ),
            "camera_sync_max_pc_offset_ms": _number(
                _widget_value(
                    getattr(self.panel, "camera_sync_max_pc_offset_spin", None),
                    _number(setting("camera_sync_max_pc_offset_s"), 0.0) * 1000.0,
                )
            ),
            "camera_sync_clock_rate_error_pct": _number(
                _widget_value(
                    getattr(self.panel, "camera_sync_clock_rate_error_spin", None),
                    _number(setting("camera_sync_max_clock_rate_error"), 0.0) * 100.0,
                )
            ),
            "camera_sync_required_sr_count": _integer(
                _widget_value(
                    getattr(self.panel, "camera_sync_required_sr_count_spin", None),
                    setting("camera_sync_required_sr_count"),
                )
            ),
            "camera_sync_warmup_timeout_s": _number(
                _widget_value(
                    getattr(self.panel, "camera_sync_warmup_timeout_spin", None),
                    setting("camera_sync_warmup_timeout_s"),
                )
            ),
            "source": "existing Measurement/Time-Sync owner widgets + AppSettings",
            "coordinate_contract": f"intrinsics/extrinsics source selectors; public output {MEASUREMENT_FRAME} [{MEASUREMENT_AXIS_CONTRACT}]",
        }

    def calibration_setup(self) -> dict[str, object]:
        """Project editable calibration setup fields without changing them."""

        return {
            "zoom_level": _integer(_widget_value(getattr(self.panel, "zoom_level_spin", None))),
            "inner_cols": _integer(_widget_value(getattr(self.panel, "calib_cols_spin", None))),
            "inner_rows": _integer(_widget_value(getattr(self.panel, "calib_rows_spin", None))),
            "square_size_mm": _number(_widget_value(getattr(self.panel, "calib_square_spin", None))),
            "min_images": _integer(_widget_value(getattr(self.panel, "calib_min_images_spin", None))),
            "calibration_root": _widget_text(getattr(self.panel, "calib_dir_edit", None)),
            "validation_mode": str(
                _combo_data(getattr(self.panel, "checkerboard_validation_mode_combo", None), "pixhawk")
                or "pixhawk"
            ),
            "validation_root": _widget_text(getattr(self.panel, "checkerboard_validation_root_edit", None)),
            "validation_json": _widget_text(getattr(self.panel, "checkerboard_validation_json_edit", None)),
            "validation_runs": _integer(
                _widget_value(getattr(self.panel, "checkerboard_validation_random_runs_spin", None))
            ),
            "validation_pair_count": _integer(
                _widget_value(getattr(self.panel, "checkerboard_validation_random_pair_count_spin", None))
            ),
            "validation_seed": _integer(
                _widget_value(getattr(self.panel, "checkerboard_validation_seed_spin", None))
            ),
            "source": "existing calibration setup widgets",
            "frame_contract": f"checkerboard image pixels/{CAMERA_FRAME}; stereo R/T remains left-camera optical RDF → right-camera optical RDF",
        }

    def session_catalog(self) -> tuple[dict[str, object], ...]:
        """List finalized/retryable session directories for review only."""

        sessions_root = self._app_root() / "recordings" / "sessions"
        if not sessions_root.is_dir():
            return ()
        entries: list[dict[str, object]] = []
        for path in sorted((item for item in sessions_root.iterdir() if item.is_dir()), key=lambda item: item.name, reverse=True):
            manifest_path = path / "session_manifest.json"
            manifest: dict[str, object] = {}
            if manifest_path.is_file():
                try:
                    payload = json.loads(manifest_path.read_text(encoding="utf-8"))
                    if isinstance(payload, dict):
                        manifest = payload
                except (OSError, ValueError):
                    manifest = {}
            published = path / "published"
            published_target = ""
            if published.is_symlink():
                try:
                    published_target = str(published.resolve().relative_to(path.resolve()))
                except (OSError, ValueError):
                    published_target = str(published)
            resource_files = manifest.get("resource_files", {})
            if not isinstance(resource_files, dict):
                resource_files = {}
            artifact_roles = manifest.get("artifact_roles", {})
            if not isinstance(artifact_roles, dict):
                artifact_roles = {}
            artifact_states = manifest.get("artifacts", {})
            if not isinstance(artifact_states, dict):
                artifact_states = {}
            artifact_names = {
                str(name)
                for name in (*resource_files.values(), *artifact_roles.keys(), *artifact_states.keys())
                if name not in (None, "")
            }
            artifacts = tuple(
                sorted(artifact_names)
            )
            entries.append(
                {
                    "session_id": path.name,
                    "path": str(path),
                    "status": str(manifest.get("status", "unknown") or "unknown"),
                    "phase": str(manifest.get("phase", "") or ""),
                    "generation_id": str(manifest.get("generation_id", "") or ""),
                    "error": str(manifest.get("error", "") or ""),
                    "retryable": bool(manifest.get("retryable", False)),
                    "published_target": published_target,
                    "artifacts": artifacts,
                    "source": "session_manifest.json + published generation pointer",
                    "time_axis": str(manifest.get("time_axis", "recording_time_ns") or "recording_time_ns"),
                }
            )
        return tuple(entries)

    def session_artifact_rows(
        self,
        session_id: str,
        artifact_name: str,
        *,
        limit: int = 100,
    ) -> dict[str, object]:
        """Read stored CSV rows without reinterpreting timestamps or frames."""

        sessions_root = (self._app_root() / "recordings" / "sessions").resolve()
        output_dir = (sessions_root / Path(str(session_id))).resolve()
        try:
            output_dir.relative_to(sessions_root)
        except ValueError:
            return {"columns": (), "rows": (), "error": "session path outside recordings root"}
        candidate = output_dir / Path(str(artifact_name)).name
        if not candidate.exists() and (output_dir / "published").is_symlink():
            candidate = output_dir / "published" / Path(str(artifact_name)).name
        if not candidate.is_file():
            return {"columns": (), "rows": (), "error": f"artifact not found: {artifact_name}"}
        if candidate.suffix.lower() != ".csv":
            return {
                "columns": (),
                "rows": (),
                "error": "only CSV artifacts are previewed; binary/video/XLSX outputs remain unchanged",
                "path": str(candidate),
            }
        try:
            with candidate.open("r", encoding="utf-8-sig", newline="") as handle:
                reader = csv.DictReader(handle)
                columns = tuple(reader.fieldnames or ())
                rows = tuple(dict(row) for _, row in zip(range(max(0, int(limit))), reader))
        except (OSError, UnicodeError, csv.Error) as exc:
            return {"columns": (), "rows": (), "error": str(exc), "path": str(candidate)}
        return {
            "columns": columns,
            "rows": rows,
            "error": "",
            "path": str(candidate),
            "time_axis": "recording_time_ns; source columns preserved",
            "coordinate_contract": "stored CSV fields are displayed as saved; no NED/ENU/camera-frame conversion",
        }

    def recording(self, *, now_s: float | None = None) -> dict[str, object]:
        now = time.monotonic() if now_s is None else float(now_s)
        session_id = str(getattr(self.panel, "recording_session_id", "") or "")
        output_dir: Path | None = None
        if session_id:
            method = getattr(self.panel, "_recording_session_dir", None)
            if callable(method):
                try:
                    output_dir = Path(method(session_id))
                except (OSError, TypeError, ValueError):
                    output_dir = None
        paths = getattr(self.panel, "video_recording_paths", {}) or {}
        video_paths = {str(side): str(path) for side, path in paths.items()} if isinstance(paths, dict) else {}
        artifacts: list[str] = []
        if output_dir is not None and output_dir.exists():
            try:
                artifacts = sorted(path.name for path in output_dir.iterdir() if path.is_file())[:64]
            except OSError:
                artifacts = []
        active = bool(getattr(self.panel, "distance_recording_enabled", False) or getattr(self.panel, "video_recording_enabled", False))
        active_method = getattr(self.panel, "_session_recording_active", None)
        if callable(active_method):
            try:
                active = bool(active_method())
            except Exception:
                pass
        elapsed_method = getattr(self.panel, "_recording_elapsed_s", None)
        elapsed = None
        if callable(elapsed_method):
            try:
                elapsed = _number(elapsed_method(now))
            except Exception:
                elapsed = None
        status_label = _widget_text(getattr(self.panel, "recording_status_label", None), "")
        save_thread = getattr(self.panel, "_session_save_thread", None)
        retry = bool(getattr(self.panel, "_session_save_retry_required", False))
        finalized = bool(getattr(self.panel, "_session_save_finalized", False))
        if save_thread is not None:
            finalization = "FINALIZING"
        elif retry:
            finalization = "RETRYABLE"
        elif finalized:
            finalization = "FINALIZED"
        elif active:
            finalization = "ACTIVE"
        else:
            finalization = "IDLE"
        return {
            "state": finalization,
            "active": active,
            "session_id": session_id,
            "output_dir": str(output_dir) if output_dir is not None else "",
            "video_paths": video_paths,
            "artifacts": tuple(artifacts),
            "elapsed_s": elapsed,
            "status_label": status_label,
            "retry_required": retry,
            "finalized": finalized,
            "sample_counts": {
                "pairs": _safe_len(getattr(self.panel, "distance_recording_rows", ())),
                "camera_left": _safe_len((getattr(self.panel, "camera_recording_rows", {}) or {}).get("left", ())),
                "camera_right": _safe_len((getattr(self.panel, "camera_recording_rows", {}) or {}).get("right", ())),
                "px4_left": _safe_len((getattr(self.panel, "px4_recording_rows", {}) or {}).get("left", ())),
                "px4_right": _safe_len((getattr(self.panel, "px4_recording_rows", {}) or {}).get("right", ())),
                "gnss_base": _safe_len(getattr(self.panel, "gnss_base_recording_rows", ())),
            },
            "recovered": getattr(self.panel, "_recovered_session_metadata", None) is not None,
            "source": "session spool rows + recording session/save worker",
            "clock": "session monotonic anchor; wall-clock ID retained",
        }

    @staticmethod
    def _camera_intrinsics(camera: object | None) -> dict[str, object]:
        if camera is None:
            return {"state": "NO DATA"}
        matrix = getattr(camera, "camera_matrix", None)
        return {
            "state": "AVAILABLE",
            "width_px": _integer(getattr(camera, "width", None)),
            "height_px": _integer(getattr(camera, "height", None)),
            "fx_px": _matrix_value(matrix, 0, 0),
            "fy_px": _matrix_value(matrix, 1, 1),
            "cx_px": _matrix_value(matrix, 0, 2),
            "cy_px": _matrix_value(matrix, 1, 2),
            "reprojection_error_px": _number(getattr(camera, "reprojection_error_px", None)),
            "image_count": _integer(getattr(camera, "image_count", None)),
            "distortion_coefficients": tuple(getattr(camera, "distortion_coefficients", ()) or ()),
            "source": "StereoIntrinsicsResult / CameraIntrinsics",
            "frame_contract": f"{CAMERA_FRAME}; camera matrix units are pixels",
        }

    def calibration(self) -> dict[str, object]:
        active = getattr(self.panel, "active_intrinsics", None)
        latest = getattr(self.panel, "last_calibration_result", None)
        result = active or latest
        calculation_active = getattr(self.panel, "checkerboard_calibration_thread", None) is not None
        calibration_status = _widget_text(getattr(self.panel, "calibration_status_label", None), "")
        calculation_status = _widget_text(
            getattr(self.panel, "checkerboard_validation_status_label", None),
            "",
        )
        status_label = calculation_status if calculation_active and calculation_status else calibration_status
        profile_path = str(getattr(self.panel, "_active_intrinsics_profile_path", "") or "")
        if not profile_path and result is not None:
            profile_path = "memory/latest-calibration"
        zoom = _integer(getattr(self.panel, "last_calibration_zoom_level", None))
        if zoom is None:
            zoom_widget = getattr(self.panel, "zoom_level_spin", None)
            zoom = _integer(_widget_value(zoom_widget))
        extrinsics = None if result is None else getattr(result, "extrinsics", None)
        extrinsics_payload: dict[str, object] = {}
        if extrinsics is not None:
            translation = tuple(getattr(extrinsics, "translation_left_to_right_m", ()) or ())
            extrinsics_payload = {
                "state": "AVAILABLE",
                "pair_count": _integer(getattr(extrinsics, "pair_count", None)),
                "reprojection_error_px": _number(getattr(extrinsics, "reprojection_error_px", None)),
                "translation_left_to_right_m": translation,
                "frame_contract": str(getattr(extrinsics, "frame_contract", "") or f"{CAMERA_FRAME}; left→right R/T"),
                "baseline_m": sum(float(value) * float(value) for value in translation) ** 0.5 if translation else None,
            }
        return {
            "state": (
                "CALCULATING"
                if calculation_active
                else "ACTIVE"
                if active is not None
                else "AVAILABLE"
                if result is not None
                else "NO DATA"
            ),
            "active": active is not None,
            "calculation_active": calculation_active,
            "profile_path": profile_path,
            "zoom": zoom,
            "status_label": status_label,
            "source": "checkerboard calculation worker + active_intrinsics/last_calibration_result",
            "left": self._camera_intrinsics(None if result is None else getattr(result, "left", None)),
            "right": self._camera_intrinsics(None if result is None else getattr(result, "right", None)),
            "extrinsics": extrinsics_payload,
            "validation_mode": str(getattr(self.panel, "checkerboard_validation_result_mode", "") or ""),
            "validation_summary": dict(getattr(self.panel, "checkerboard_validation_summary", {}) or {}),
            "frame_contract": f"{CAMERA_FRAME}; calibration intrinsics/extrinsics stay in OpenCV optical RDF",
            "measurement_contract": f"public output {MEASUREMENT_FRAME} [{MEASUREMENT_AXIS_CONTRACT}]; baseline {BASELINE_TRANSLATION_FRAME}",
        }

    def summary(self, *, now_s: float | None = None) -> dict[str, object]:
        now = time.monotonic() if now_s is None else float(now_s)
        return {
            "attitude": {side: self.attitude(side, now_s=now) for side in ("left", "right")},
            "gps": {side: self.gps(side, now_s=now) for side in ("left", "right")},
            "baseline": self.baseline(),
            "detection_config": self.detection_config(),
            "detection": {side: self.detection(side, now_s=now) for side in ("left", "right")},
            "recording": self.recording(now_s=now),
            "calibration": self.calibration(),
        }
