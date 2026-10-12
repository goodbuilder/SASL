"""Pure Base/NTRIP/readiness evidence from one detached capture.

NAV-SVIN XYZ is ECEF metres; NAV-PVT latitude/longitude is WGS84 degrees and
ellipsoid/MSL heights remain separate. RTCM counters/clocks keep their source.
"""

from collections.abc import Mapping
from dataclasses import dataclass

from Workstation.workstation_core.gnss_monitor import evaluate_ntrip_health

from .snapshot import snapshot_mapping, thaw_snapshot
from .values import _age, _integer, _number

COORDINATE_FIELDS = (
    "latitude_deg",
    "longitude_deg",
    "ellipsoid_height_m",
    "msl_altitude_m",
    "selected_height_m",
    "fixed",
    "device_id",
    "timestamp_s",
    "fix_type",
    "carrier_solution",
    "fix_stage_label",
)
NTRIP_STAT_FIELDS = (
    "connected_at_monotonic_s",
    "last_chunk_monotonic_s",
    "bytes_received",
    "chunks_received",
    "last_gga_monotonic_s",
    "gga_sent_count",
)


@dataclass(frozen=True)
class GnssOperationInputs:
    operation: Mapping
    base: Mapping
    fixed: Mapping
    network: Mapping
    stats: Mapping
    rover_gps: Mapping

    def __post_init__(self):
        for name in self.__dataclass_fields__:
            object.__setattr__(self, name, snapshot_mapping(getattr(self, name)))

    def project(self, *, now):
        return thaw_snapshot(
            _project(
                operation=self.operation,
                base=self.base,
                fixed=self.fixed,
                network=self.network,
                stats=self.stats,
                rover_gps=self.rover_gps,
                now=now,
            )
        )


def _project(*, operation, base, fixed, network, stats, rover_gps, now):
    injection = stats.get("injection", {})
    injection = injection if isinstance(injection, Mapping) else {}
    routes: dict[str, dict[str, object]] = {}
    for side in ("left", "right", "drone"):
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
    metadata = operation.get("metadata", {})
    config_values = operation.get("config", {})
    local_config = config_values.get("local_base", {})
    local_config = local_config if isinstance(local_config, Mapping) else {}
    network_config = config_values.get("network", {})
    network_config = network_config if isinstance(network_config, Mapping) else {}
    readiness_config = config_values.get("readiness", {})
    readiness_config = readiness_config if isinstance(readiness_config, Mapping) else {}
    fixed_config = config_values.get("fixed_base", {})
    fixed_config = fixed_config if isinstance(fixed_config, Mapping) else {}
    candidate_rows = base.get("candidates", ())
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
    survey = base.get("survey", {})
    survey_samples = base.get("survey_samples", ())
    if not survey and survey_samples:
        try:
            (_sample_time, duration_s, accuracy_m, active, valid) = survey_samples[-1]
            survey = {"duration_s": duration_s, "mean_accuracy_m": accuracy_m, "active": active, "valid": valid}
        except (IndexError, TypeError, ValueError):
            survey = {}
    latest_coordinate = base.get("coordinate")
    coordinate = {
        "latitude_deg": _number((latest_coordinate or {}).get("latitude_deg", None)),
        "longitude_deg": _number((latest_coordinate or {}).get("longitude_deg", None)),
        "ellipsoid_height_m": _number((latest_coordinate or {}).get("ellipsoid_height_m", None)),
        "msl_altitude_m": _number((latest_coordinate or {}).get("msl_altitude_m", None)),
        "selected_height_m": _number((latest_coordinate or {}).get("selected_height_m", None)),
        "height_type": str((latest_coordinate or {}).get("height_type", "UNKNOWN")),
        "fixed": None if latest_coordinate is None else bool((latest_coordinate or {}).get("fixed", False)),
        "device_id": str((latest_coordinate or {}).get("device_id", "") or ""),
        "age_s": _age((latest_coordinate or {}).get("timestamp_s", None), now),
        "position_contract": "WGS84 latitude/longitude degrees; heights metres with explicit height type",
        "fix_type": (latest_coordinate or {}).get("fix_type", None),
        "carrier_solution": (latest_coordinate or {}).get("carrier_solution", None),
        "fix_stage_label": "--"
        if latest_coordinate is None
        else str((latest_coordinate or {}).get("fix_stage_label", "UNKNOWN")),
    }
    satellite_quality = base.get("satellite_quality", {})
    satellites = {
        "count": base.get("satellite_count", None),
        "count_source": str(base.get("satellite_count_source", "") or ""),
        "used": _integer(satellite_quality.get("used")),
        "tracked": _integer(satellite_quality.get("tracked")),
        "avg_cno_dbhz": _number(satellite_quality.get("avg_cno_dbhz")),
    }
    profile_values = fixed.get("profile", {})
    live_profile_values = fixed.get("live_profile", {})
    readiness = operation.get("readiness", {})
    state = operation.get("state", "UNKNOWN")
    elapsed_s = max(0.0, now - float(stats.get("start_s") or now))
    operation_mode = str(
        metadata.get("gnss_operation_mode", config_values.get("operation_mode", "UNKNOWN")) or "UNKNOWN"
    )
    network_stats = network.get("stats", {})
    ntrip_health = evaluate_ntrip_health(
        network_mode=operation_mode in {"NETWORK_ASSISTED_FIXED_BASE", "NETWORK_RTK_DIRECT"},
        connected=bool(network.get("connected", False)),
        running=bool(network.get("running")),
        phase=str(network.get("phase", "DISCONNECTED") or "DISCONNECTED"),
        connected_at_s=network_stats.get("connected_at_monotonic_s", None),
        last_chunk_s=network_stats.get("last_chunk_monotonic_s", None),
        last_rtcm_s=stats.get("last_frame_s"),
        now_s=now,
        transport_stale_after_s=_number(network_config.get("read_timeout_s"), 10.0) or 10.0,
        correction_stale_after_s=_number(readiness_config.get("maximum_correction_age_s"), 3.0) or 3.0,
        bytes_received=network_stats.get("bytes_received", 0),
        chunks_received=network_stats.get("chunks_received", 0),
        reconnect_count=network.get("reconnect_count", 0),
        last_error=str(network.get("last_error", "") or ""),
    )
    ntrip_message_types = stats.get("message_types")
    ntrip_message_types = (
        {str(key): _integer(value, 0) for (key, value) in ntrip_message_types.items()}
        if isinstance(ntrip_message_types, Mapping)
        else {}
    )
    ntrip_base_position_present = any(
        int(key) in {1005, 1006} for key in ntrip_message_types if key.lstrip("-").isdigit()
    )
    ntrip_msm_present = any(1070 <= int(key) <= 1129 for key in ntrip_message_types if key.lstrip("-").isdigit())
    rover_fixed_since = operation.get("rover_fixed_since", {})
    rover: dict[str, dict[str, object]] = {}
    for side in ("left", "right"):
        gps = rover_gps[side]
        fix_type = gps.get("fix_type")
        fixed_since_s = _number(rover_fixed_since.get(side))
        rover[side] = {
            "state": str(gps.get("state", "NO DATA") or "NO DATA"),
            "rtk_label": str(gps.get("rtk_label", "UNKNOWN") or "UNKNOWN"),
            "fix_type": fix_type,
            "rtk_fixed": None if fix_type is None else fix_type == 6,
            "age_s": gps.get("age_s"),
            "fixed_age_s": None if fixed_since_s is None else max(0.0, now - fixed_since_s),
        }
    return {
        "state": state,
        "mode": operation_mode,
        "experiment_site": {
            "label": str(config_values.get("site_label", "") or ""),
            "address": str(config_values.get("site_address", "") or ""),
            "coordinate_contract": "WGS84 geodetic latitude/longitude; Base height is explicit ellipsoid/MSL",
        },
        "source_identity": dict(source_identity),
        "invalidation_reason": str(metadata.get("invalidation_reason", "") or ""),
        "base_serial": {
            "connected": bool(base.get("connected", False)),
            "device": str(source_identity.get("base_device", local_config.get("device", "")) or ""),
            "baud": _integer(local_config.get("baud")),
        },
        "base_candidates": tuple(candidates),
        "base_discovery_active": bool(base.get("discovery_active")),
        "base_discovery_status": str(base.get("discovery_status", "NOT RUN") or "NOT RUN"),
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
            "connected": bool(network.get("connected", False)),
            "running": bool(network.get("running")),
            "health_state": ntrip_health.state.value,
            "status": str(network.get("status", "") or ""),
            "caster_host": str(source_identity.get("caster_host", network_config.get("caster_host", "")) or ""),
            "caster_port": _integer(source_identity.get("caster_port", network_config.get("caster_port"))),
            "mountpoint": str(source_identity.get("mountpoint", network_config.get("mountpoint", "")) or ""),
            "gga_policy": str(source_identity.get("gga_policy", network_config.get("gga_policy", "")) or ""),
            "tls": bool(network_config.get("tls", False)),
            "username": str(network_config.get("username", "") or ""),
            "username_env": str(network_config.get("username_env", "") or ""),
            "password_env": str(network_config.get("password_env", "") or ""),
            "session_age_s": ntrip_health.session_age_s,
            "last_chunk_age_s": ntrip_health.transport_age_s,
            "last_gga_age_s": _age(network_stats.get("last_gga_monotonic_s", None), now),
            "gga_sent_count": _integer(network_stats.get("gga_sent_count", 0), 0),
            "gga_skip_reason": str(network.get("gga_skip_reason", "") or ""),
            "gga_skip_age_s": _age(network.get("gga_skip_monotonic_s", None), now),
            "bytes_received": ntrip_health.bytes_received,
            "chunks_received": ntrip_health.chunks_received,
            "byte_rate_bps": float(ntrip_health.bytes_received) / ntrip_health.session_age_s
            if ntrip_health.session_age_s and ntrip_health.session_age_s > 0.0
            else 0.0,
            "rtcm_age_s": ntrip_health.rtcm_age_s,
            "rtcm_frames": _integer(stats.get("frames"), 0),
            "rtcm_message_types": ntrip_message_types,
            "rtcm_base_position_present": ntrip_base_position_present,
            "rtcm_msm_present": ntrip_msm_present,
            "reconnect_count": ntrip_health.reconnect_count,
            "last_error": ntrip_health.last_error,
            "transport_stale_after_s": _number(network_config.get("read_timeout_s"), 10.0),
            "correction_stale_after_s": _number(readiness_config.get("maximum_correction_age_s"), 3.0),
        },
        "base_coordinate": coordinate,
        "satellites": satellites,
        "fixed_base": {
            "sample_count": base.get("sample_count", 0),
            "profile": dict(profile_values),
            "live_profile": dict(live_profile_values),
            "target_acquisition_duration_s": _number(fixed_config.get("acquisition_duration_s")),
            "target_fixed_duration_s": _number(fixed_config.get("minimum_fixed_duration_s")),
            "target_fixed_ratio": _number(fixed_config.get("minimum_fixed_ratio")),
            "maximum_horizontal_scatter_m": _number(fixed_config.get("maximum_horizontal_scatter_m")),
            "maximum_vertical_scatter_m": _number(fixed_config.get("maximum_vertical_scatter_m")),
            "height_type": str(fixed_config.get("height_type", "UNKNOWN") or "UNKNOWN"),
            "reconfigure_ack": bool(fixed.get("ack", False)),
            "reconfigure_readback": bool(fixed.get("readback", False)),
            "reconfigure_rtcm": bool(fixed.get("rtcm", False)),
        },
        "base_frames": _integer(stats.get("frames"), 0),
        "base_bytes": _integer(stats.get("bytes"), 0),
        "average_frame_rate_hz": float(stats.get("frames") or 0) / elapsed_s if elapsed_s > 0.0 else 0.0,
        "average_byte_rate_bps": float(stats.get("bytes") or 0) / elapsed_s if elapsed_s > 0.0 else 0.0,
        "last_frame_age_s": _age(stats.get("last_frame_s"), now),
        "routes": routes,
        "rover": rover,
        "readiness_limits": {
            "minimum_fix_type": _integer(readiness_config.get("minimum_fix_type"), 3),
            "minimum_fixed_duration_s": _number(readiness_config.get("minimum_fixed_duration_s"), 30.0),
            "maximum_correction_age_s": _number(readiness_config.get("maximum_correction_age_s"), 3.0),
        },
        "readiness": dict(readiness),
        "contract": "RTCM base stream → MAVLink GPS_RTCM_DATA route; bytes/packets are producer counters",
    }
