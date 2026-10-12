"""Read-only projections of the existing StereoWIRIS panel state.

The workstation is a presentation shell.  This module deliberately keeps the
projection boundary boring: it reads existing model objects and widgets,
retains their timestamp/frame contracts, and never performs a measurement,
save, or calibration operation. Explicit presentation-only coordinate
derivations remain separate from their authoritative raw values.
"""

from __future__ import annotations

import math
import time
from collections.abc import Mapping
from pathlib import Path

from Workstation.evidence_views.artifacts import (
    _artifact_reason as _artifact_reason,
)
from Workstation.evidence_views.artifacts import (
    _existing_artifact_byte_total as _existing_artifact_byte_total,
)
from Workstation.evidence_views.artifacts import (
    _mapping_state as _mapping_state,
)
from Workstation.evidence_views.artifacts import (
    _package_artifact_projection as _package_artifact_projection,
)
from Workstation.evidence_views.artifacts import (
    _package_error_projection as _package_error_projection,
)
from Workstation.evidence_views.artifacts import (
    _package_manifest_reference as _package_manifest_reference,
)
from Workstation.evidence_views.artifacts import (
    _package_session_projection as _package_session_projection,
)
from Workstation.evidence_views.artifacts import (
    _reason_values as _reason_values,
)
from Workstation.evidence_views.artifacts import (
    _recording_session_directories as _recording_session_directories,
)
from Workstation.evidence_views.artifacts import (
    _resolve_recording_session_directory as _resolve_recording_session_directory,
)
from Workstation.evidence_views.artifacts import (
    _run_ledger_artifact_names as _run_ledger_artifact_names,
)
from Workstation.evidence_views.artifacts import (
    _safe_session_relative_path as _safe_session_relative_path,
)
from Workstation.evidence_views.artifacts import (
    resolve_session_artifact_path as resolve_session_artifact_path,
)
from Workstation.evidence_views.calibration import (
    CAMERA_FIELDS,
    EXTRINSICS_FIELDS,
    PROFILE_FIELDS,
    CalibrationInputs,
    project_camera_intrinsics,
)
from Workstation.evidence_views.catalog import SessionEvidenceQuery
from Workstation.evidence_views.connections import (
    HEALTH_FIELDS,
    project_camera_candidates,
    project_network_roles,
    project_pixhawk_candidates,
    project_pixhawk_recovery,
)
from Workstation.evidence_views.geometry import (
    BASELINE_FIELDS,
    BASELINE_PROJECTION_FIELDS,
    BASELINE_QUALITY_FIELDS,
    InvalidGpsInput,
    project_baseline,
    project_baseline_yaw,
    project_drone_geometry,
    project_physical_alignment,
)
from Workstation.evidence_views.gnss import COORDINATE_FIELDS, NTRIP_STAT_FIELDS, GnssOperationInputs
from Workstation.evidence_views.magnetometers import (
    MAG_SAMPLE_FIELDS,
    MAG_SLOT_FIELDS,
    MagnetometerInputs,
    MagnetometerProjection,
)
from Workstation.evidence_views.recording import RecordingInputs
from Workstation.evidence_views.settings import (
    project_calibration_setup,
    project_detection_config,
    project_measurement_settings,
)
from Workstation.evidence_views.snapshot import snapshot_attributes, snapshot_mapping, thaw_snapshot
from Workstation.evidence_views.telemetry import (
    ATTITUDE_FIELDS,
    DETECTION_FIELDS,
    GPS_FIELDS,
    project_attitude,
    project_detection,
    project_gps,
)
from Workstation.evidence_views.values import _integer, _number, _safe_len
from Workstation.workstation_core.rtk_geometry import GpsPositionState


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


def _copy_gps_state(state):
    # Geometry consumes only immutable scalar GPS values. In particular the
    # producer's raw MAVLink message is not part of this evidence boundary.
    if state is None:
        return None
    if not isinstance(state, GpsPositionState):
        return InvalidGpsInput(_number(getattr(state, "timestamp_s", None)))
    names = (
        "camera",
        "namespace",
        "timestamp_s",
        "topic",
        "latitude_deg",
        "longitude_deg",
        "altitude_m",
        "altitude_ellipsoid_m",
        "fix_type",
        "satellites_used",
        "eph_m",
        "epv_m",
    )
    return GpsPositionState(**{name: getattr(state, name) for name in names})


def _capture_camera_intrinsics(camera):
    if camera is None:
        return None
    values = dict(snapshot_attributes(camera, CAMERA_FIELDS))
    matrix = getattr(camera, "camera_matrix", None)
    values.update(
        {
            "fx_px": _matrix_value(matrix, 0, 0),
            "fy_px": _matrix_value(matrix, 1, 1),
            "cx_px": _matrix_value(matrix, 0, 2),
            "cy_px": _matrix_value(matrix, 1, 2),
        }
    )
    return snapshot_mapping(values)


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
        provenance = (
            mavlink_link_provenance(self.panel, side)
            if state is not None and str(getattr(state, "topic", "")).startswith("mavlink/")
            else {}
        )
        return project_attitude(
            side, state=snapshot_attributes(state, ATTITUDE_FIELDS), now=now, provenance=snapshot_mapping(provenance)
        )

    def gps(self, side: str, *, now_s: float | None = None, source: str = "GPS1") -> dict[str, object]:
        now = time.monotonic() if now_s is None else float(now_s)
        states = getattr(self.panel, "latest_gps", {}) or {}
        source = str(source or "GPS1").upper()
        if source not in {"GPS1", "GPS2"}:
            source = "GPS1"
        state_key = side if source == "GPS1" else f"{side}_GPS2"
        state = states.get(state_key) if isinstance(states, dict) else None
        provenance = (
            mavlink_link_provenance(self.panel, side)
            if state is not None and str(getattr(state, "topic", "")).startswith("mavlink/")
            else {}
        )
        frozen = None if state is None else self._gnss_position_frozen(state_key)
        return project_gps(
            side,
            state=snapshot_attributes(state, GPS_FIELDS),
            now=now,
            source=source,
            provenance=snapshot_mapping(provenance),
            frozen=None if frozen is None else snapshot_mapping(frozen),
        )

    def _gnss_position_frozen(self, key: str) -> Mapping[str, object] | None:
        """Return the producer-side frozen-position evidence for one GPS key."""

        store = getattr(self.panel, "_gnss_position_frozen", {}) or {}
        entry = store.get(key) if isinstance(store, Mapping) else None
        if not isinstance(entry, Mapping):
            return None
        packets = _integer(entry.get("packets"))
        return entry if packets is not None and packets > 0 else None

    def _magnetometer_yaw_fields(
        self,
        side: str,
        *,
        xmag: object,
        ymag: object,
        sensor_device_id: int | None,
        sample_available: bool,
        now_s: float,
    ) -> dict[str, object]:
        return self._magnetometer_projection(side, now_s)._magnetometer_yaw_fields(
            side,
            xmag=xmag,
            ymag=ymag,
            sensor_device_id=sensor_device_id,
            sample_available=sample_available,
            now_s=now_s,
        )

    def _magnetometer_corrected_yaw_fields(
        self,
        side: str,
        *,
        xmag: object,
        ymag: object,
        zmag: object,
        sensor_device_id: int | None,
        sample_available: bool,
    ) -> dict[str, object]:
        return self._magnetometer_projection(side, 0.0, needs_attitude=False)._magnetometer_corrected_yaw_fields(
            side, xmag=xmag, ymag=ymag, zmag=zmag, sensor_device_id=sensor_device_id, sample_available=sample_available
        )

    def magnetometers(self, side: str, *, now_s: float | None = None) -> tuple[dict[str, object], ...]:
        now = time.monotonic() if now_s is None else float(now_s)
        return self._magnetometer_projection(side, now).magnetometers(side, now_s=now)

    def can_magnetometers(self, side: str, *, now_s: float | None = None) -> tuple[dict[str, object], ...]:
        now = time.monotonic() if now_s is None else float(now_s)
        return self._magnetometer_projection(side, now).can_magnetometers(side, now_s=now)

    def magnetometers_for_imu_view(self, side: str, *, now_s: float | None = None) -> tuple[dict[str, object], ...]:
        now = time.monotonic() if now_s is None else float(now_s)
        return self._magnetometer_projection(side, now).magnetometers_for_imu_view(side, now_s=now)

    def magnetometers_for_cal_mag_view(self, side: str, *, now_s: float | None = None) -> tuple[dict[str, object], ...]:
        now = time.monotonic() if now_s is None else float(now_s)
        return self._magnetometer_projection(side, now).magnetometers_for_cal_mag_view(side, now_s=now)

    def baseline(self) -> dict[str, object]:
        baseline = getattr(self.panel, "latest_rtk_baseline", None)
        projection = getattr(self.panel, "latest_p1_body_frd_baseline", None)
        quality = getattr(self.panel, "latest_baseline_quality", None)
        selector = getattr(self.panel, "_selected_baseline_gps_states", None)
        if callable(selector):
            selected_left, selected_right = selector()
        else:
            latest = getattr(self.panel, "latest_gps", {}) or {}
            selected_left = latest.get("left") if isinstance(latest, dict) else None
            selected_right = latest.get("right") if isinstance(latest, dict) else None
        source_reader = getattr(self.panel, "_pixhawk_gnss_source", None)
        left_source = str(source_reader("left")) if callable(source_reader) else "GPS1"
        right_source = str(source_reader("right")) if callable(source_reader) else "GPS1"
        return project_baseline(
            baseline=snapshot_attributes(baseline, BASELINE_FIELDS),
            projection=snapshot_attributes(projection, BASELINE_PROJECTION_FIELDS),
            quality=snapshot_attributes(quality, BASELINE_QUALITY_FIELDS),
            selected_left=snapshot_attributes(selected_left, ("fix_type",)),
            selected_right=snapshot_attributes(selected_right, ("fix_type",)),
            left_source=left_source,
            right_source=right_source,
        )

    def baseline_yaw_alignment(self, *, now_s: float | None = None) -> dict[str, object]:
        """Project horizontal Baseline-P1/P2 angles with source freshness.

        This latest-state read model is an operator aid for stationary/slow
        physical alignment.  It does not claim that GNSS and ATTITUDE samples
        are simultaneous, and it never replaces the raw NED baseline or raw
        PX4 attitude evidence.
        """

        now = time.monotonic() if now_s is None else float(now_s)
        baseline = self.baseline()
        attitudes = {side: self.attitude(side, now_s=now) for side in ("left", "right")}
        sources = {
            "left": str(baseline.get("left_source") or "GPS1"),
            "right": str(baseline.get("right_source") or "GPS1"),
        }
        gps = {side: self.gps(side, source=sources[side], now_s=now) for side in ("left", "right")}

        manager = getattr(self.panel, "gnss_operation_manager", None)
        readiness = getattr(getattr(manager, "config", None), "readiness", None)
        try:
            maximum_gps_age_s = float(getattr(readiness, "maximum_gps_sample_age_s", 1.0))
        except (TypeError, ValueError):
            maximum_gps_age_s = 1.0
        if not math.isfinite(maximum_gps_age_s) or maximum_gps_age_s < 0.0:
            maximum_gps_age_s = 1.0

        return project_baseline_yaw(
            baseline=snapshot_mapping(baseline),
            attitudes=snapshot_mapping(attitudes),
            gps=snapshot_mapping(gps),
            sources=snapshot_mapping(sources),
            maximum_gps_age_s=maximum_gps_age_s,
        )

    def drone_pixhawk_geometry(self, *, now_s: float | None = None) -> dict[str, object]:
        """Project Dr/P1/P2 relative vectors and horizontal triangle angles.

        The midpoint vector uses the existing GNSS local-level FRD geometry.
        The P1/P2 vectors are the same GNSS world-NED vectors expressed in the
        corresponding PX4 body-FRD frame.  The two point angles use only the
        horizontal North/East components and are independent of Pixhawk yaw.
        """

        now = time.monotonic() if now_s is None else float(now_s)
        if not math.isfinite(now):
            raise ValueError("now_s must be finite")

        latest_gps = getattr(self.panel, "latest_gps", {}) or {}
        latest_gps = latest_gps if isinstance(latest_gps, Mapping) else {}

        def selected_gps(side: str) -> tuple[GpsPositionState | None, str]:
            selector = getattr(self.panel, "_selected_rover_gps_state", None)
            if callable(selector):
                try:
                    selected = selector(side)
                except Exception:
                    selected = None
                if not isinstance(selected, GpsPositionState):
                    selected = None
                if side == "drone":
                    if selected is None:
                        selected = latest_gps.get("drone")
                    return (
                        selected if isinstance(selected, GpsPositionState) else None,
                        "DRONE",
                    )
                source_reader = getattr(self.panel, "_pixhawk_gnss_source", None)
                source = str(source_reader(side)).upper() if callable(source_reader) else "GPS1"
                return selected, source if source in {"GPS1", "GPS2"} else "GPS1"
            if side == "drone":
                selected = latest_gps.get("drone")
                return (
                    selected if isinstance(selected, GpsPositionState) else None,
                    "DRONE",
                )
            source_reader = getattr(self.panel, "_pixhawk_gnss_source", None)
            source = str(source_reader(side)).upper() if callable(source_reader) else "GPS1"
            source = source if source in {"GPS1", "GPS2"} else "GPS1"
            key = side if source == "GPS1" else f"{side}_GPS2"
            selected = latest_gps.get(key)
            return selected if isinstance(selected, GpsPositionState) else None, source

        p1, p1_source = selected_gps("left")
        p2, p2_source = selected_gps("right")
        drone, _drone_source = selected_gps("drone")
        if not callable(getattr(self.panel, "_selected_rover_gps_state", None)):
            drone = latest_gps.get("drone")

        return project_drone_geometry(
            p1=_copy_gps_state(p1),
            p2=_copy_gps_state(p2),
            drone=_copy_gps_state(drone),
            p1_source=p1_source,
            p2_source=p2_source,
            attitudes=snapshot_mapping(
                {"p1": self.attitude("left", now_s=now), "p2": self.attitude("right", now_s=now)}
            ),
            drone_frozen=None
            if self._gnss_position_frozen("drone") is None
            else snapshot_mapping(self._gnss_position_frozen("drone")),
            now=now,
        )

    def gnss_operation(self, *, now_s: float | None = None) -> dict[str, object]:
        now = time.monotonic() if now_s is None else float(now_s)
        return self._capture_gnss_operation(now).project(now=now)

    def _capture_gnss_operation(self, now):
        manager = getattr(self.panel, "gnss_operation_manager", None)

        def mapping_call(owner, method):
            reader = getattr(owner, method, None)
            try:
                value = reader() if callable(reader) else {}
            except Exception:
                value = {}
            return value if isinstance(value, Mapping) else {}

        metadata = mapping_call(manager, "metadata")
        config = mapping_call(getattr(manager, "config", None), "to_mapping")
        readiness = None
        reader = getattr(manager, "readiness", None)
        if callable(reader):
            try:
                readiness = reader()
            except Exception:
                pass
        readiness = (
            readiness.to_metadata()
            if readiness is not None and callable(getattr(readiness, "to_metadata", None))
            else {}
        )
        state = getattr(manager, "state", "UNKNOWN")
        coordinate_samples = getattr(self.panel, "gnss_coordinate_samples", ()) or ()
        latest = None
        try:
            if coordinate_samples:
                latest = coordinate_samples[-1]
        except (IndexError, TypeError):
            pass
        coordinate = None if latest is None else dict(snapshot_attributes(latest, COORDINATE_FIELDS))
        if coordinate is not None:
            coordinate["height_type"] = str(getattr(getattr(latest, "height_type", None), "value", "UNKNOWN"))

        def mapping(value):
            return value if isinstance(value, Mapping) else {}

        return GnssOperationInputs(
            operation={
                "metadata": metadata,
                "config": config,
                "readiness": readiness,
                "state": str(getattr(state, "value", state) or "UNKNOWN"),
                "rover_fixed_since": mapping(getattr(self.panel, "gnss_rover_fixed_since", {}) or {}),
            },
            base={
                "connected": bool(getattr(self.panel, "gnss_base_serial_connected", False)),
                "candidates": tuple(
                    snapshot_mapping(row, ("port", "target", "baud", "status", "messages", "ubx"))
                    for row in (getattr(self.panel, "_gnss_base_candidate_rows", ()) or ())
                    if isinstance(row, Mapping)
                ),
                "discovery_active": getattr(self.panel, "gnss_base_detect_thread", None) is not None,
                "discovery_status": str(
                    getattr(self.panel, "workstation_gnss_base_discovery_status", "NOT RUN") or "NOT RUN"
                ),
                "survey": mapping(getattr(self.panel, "latest_gnss_base_survey_in", None)),
                "survey_samples": tuple(getattr(self.panel, "gnss_base_survey_samples", ()) or ()),
                "coordinate": coordinate,
                "sample_count": _safe_len(coordinate_samples),
                "satellite_count": getattr(self.panel, "gnss_base_satellite_count", None),
                "satellite_count_source": str(getattr(self.panel, "gnss_base_satellite_count_source", "") or ""),
                "satellite_quality": mapping(getattr(self.panel, "gnss_base_satellite_quality", None)),
            },
            fixed={
                "profile": mapping_call(getattr(self.panel, "gnss_fixed_base_profile", None), "to_mapping"),
                "live_profile": mapping_call(getattr(self.panel, "gnss_fixed_base_live_profile", None), "to_mapping"),
                "ack": bool(getattr(self.panel, "gnss_fixed_base_reconfigure_ack", False)),
                "readback": bool(getattr(self.panel, "gnss_fixed_base_reconfigure_readback", False)),
                "rtcm": bool(getattr(self.panel, "gnss_fixed_base_reconfigure_rtcm", False)),
            },
            network={
                "connected": bool(getattr(self.panel, "gnss_network_connected", False)),
                "running": getattr(self.panel, "gnss_network_thread", None) is not None,
                "phase": str(getattr(self.panel, "gnss_network_phase", "DISCONNECTED") or "DISCONNECTED"),
                "reconnect_count": getattr(self.panel, "gnss_network_reconnect_count", 0),
                "last_error": str(getattr(self.panel, "gnss_network_last_error", "") or ""),
                "status": str(getattr(self.panel, "gnss_network_status_line", "") or ""),
                "gga_skip_reason": str(getattr(self.panel, "_gnss_last_gga_skip_reason", "") or ""),
                "gga_skip_monotonic_s": getattr(self.panel, "_gnss_last_gga_skip_monotonic_s", None),
                "stats": snapshot_attributes(getattr(self.panel, "gnss_network_stats", None), NTRIP_STAT_FIELDS) or {},
            },
            stats=mapping(getattr(self.panel, "gnss_base_rtcm_stats", {}) or {}),
            rover_gps={side: self.gps(side, now_s=now) for side in ("left", "right")},
        )

    def camera_connection_candidates(self) -> dict[str, object]:
        """Project discovered camera endpoints without exposing owner-only objects."""

        rows = getattr(self.panel, "_camera_candidate_rows", ()) or ()
        rows = tuple(
            {
                **dict(snapshot_mapping(row, ("kind", "address", "host", "port", "status", "host_ip"))),
                "interface": snapshot_attributes(row.get("interface"), ("name", "mac")),
            }
            for row in rows
            if isinstance(row, Mapping)
        )
        raw_network_roles = getattr(self.panel, "network_role_assignments", {}) or {}
        if not isinstance(raw_network_roles, Mapping) or not raw_network_roles:
            interface_edit = getattr(self.panel, "interface_edit", None)
            interface_name = (
                interface_edit.text().strip() if interface_edit is not None and hasattr(interface_edit, "text") else ""
            )
            raw_network_roles = {"camera": {"interface": interface_name, "stable_id": ""}}
        return project_camera_candidates(
            snapshot_mapping({"rows": rows})["rows"],
            discovery_active=any(
                getattr(self.panel, name, None) is not None
                for name in ("camera_network_discovery_thread", "camera_auto_detect_thread")
            ),
            status=str(getattr(self.panel, "workstation_camera_discovery_status", "NOT RUN") or "NOT RUN"),
            network=snapshot_mapping(self._network_role_projection(raw_network_roles)),
        )

    def pixhawk_connection_candidates(self) -> dict[str, object]:
        """Project PC-interface, USB, and MAVLink candidates for explicit P1/P2 assignment."""

        grouped = getattr(self.panel, "_pixhawk_candidate_rows_by_source", None)
        if not isinstance(grouped, Mapping):
            grouped = {"mavlink": getattr(self.panel, "_pixhawk_candidate_rows", ()) or ()}
        pixhawk_assignments = getattr(self.panel, "pixhawk_network_assignments", {}) or {}
        raw_network_roles = getattr(self.panel, "network_role_assignments", {}) or {}
        if not isinstance(raw_network_roles, Mapping) or not raw_network_roles:
            raw_network_roles = {
                "p1": {
                    "interface": str((pixhawk_assignments.get("left") or {}).get("interface") or ""),
                    "stable_id": "",
                },
                "p2": {
                    "interface": str((pixhawk_assignments.get("right") or {}).get("interface") or ""),
                    "stable_id": "",
                },
            }
        candidate_fields = (
            "kind",
            "value",
            "resolved_device",
            "baud",
            "system_id",
            "component_id",
            "observed_monotonic_s",
            "status",
            "interface",
            "candidate_id",
            "carrier",
            "local_cidr",
            "local_ip",
            "p1_route",
            "p2_route",
            "p1_route_state",
            "p2_route_state",
            "p1_endpoint",
            "p2_endpoint",
            "mac",
            "driver",
            "device_path",
        )
        grouped = {
            source: tuple(
                snapshot_mapping(row, candidate_fields) if isinstance(row, Mapping) else None
                for row in (grouped.get(source, ()) or ())
            )
            for source in ("network", "pc", "mavlink")
        }
        return thaw_snapshot(
            project_pixhawk_candidates(
                snapshot_mapping(grouped),
                discovery_active=any(
                    getattr(self.panel, name, None) is not None
                    for name in (
                        "pixhawk_usb_discovery_thread",
                        "pixhawk_host_ip_discovery_thread",
                        "pixhawk_network_discovery_thread",
                    )
                ),
                status=str(getattr(self.panel, "workstation_pixhawk_discovery_status", "NOT RUN") or "NOT RUN"),
                revision=int(getattr(self.panel, "pixhawk_discovery_revision", 0) or 0),
                pixhawk_assignments=snapshot_mapping(pixhawk_assignments),
                assignment_status=snapshot_mapping(getattr(self.panel, "pixhawk_network_assignment_status", {}) or {}),
                network=snapshot_mapping(self._network_role_projection(raw_network_roles)),
            )
        )

    def _network_role_projection(self, raw_network_roles: Mapping[str, object]) -> dict[str, object]:
        return project_network_roles(
            snapshot_mapping(raw_network_roles),
            status=snapshot_mapping(getattr(self.panel, "network_role_status", {}) or {}),
            active=getattr(self.panel, "network_role_operation", None) is not None,
        )

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
        status_by_side = getattr(panel, "pixhawk_usb_recovery_status", {}) or {}
        record = status_by_side.get(side, {}) if isinstance(status_by_side, Mapping) else {}
        record = record if isinstance(record, Mapping) else {}
        active = getattr(panel, "pixhawk_usb_recovery_thread", None) is not None
        subscriber = getattr(panel, "mavlink_pixhawk", None)
        health_reader = getattr(subscriber, "mavlink_health_snapshot", None)
        try:
            health = health_reader(side, now_s=now) if callable(health_reader) else None
        except Exception:
            health = None
        values = None if health is None else dict(snapshot_attributes(health, HEALTH_FIELDS))
        if values is not None:
            state = getattr(health, "state", "")
            values["state"] = str(getattr(state, "value", state))
        return project_pixhawk_recovery(
            side,
            record=snapshot_mapping(record),
            active=active,
            health=None if values is None else snapshot_mapping(values),
            provenance=snapshot_mapping(mavlink_link_provenance(panel, side)),
            attitude=snapshot_mapping(self.attitude(side, now_s=now) if health is None else {}),
            identities=snapshot_mapping({"rows": getattr(panel, "pixhawk_usb_identity_evidence", ()) or ()})["rows"],
            usb_defaults=snapshot_mapping({"url": _widget_text(url_widget, "--"), "baud": _widget_value(baud_widget)}),
            now=now,
        )

    def physical_alignment(self) -> dict[str, object]:
        """Return physical-alignment evidence and iteration history."""

        snapshot_method = getattr(self.panel, "workstation_physical_alignment_snapshot", None)
        if callable(snapshot_method):
            try:
                snapshot = snapshot_method()
            except Exception:
                snapshot = None
            if isinstance(snapshot, Mapping):
                return thaw_snapshot(snapshot_mapping(snapshot))

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
        try:
            left_q = None if left is None else tuple(left.q)
            right_q = None if right is None else tuple(right.q)
        except (AttributeError, TypeError, ValueError):
            left_q = right_q = None
        return thaw_snapshot(
            project_physical_alignment(
                quality_mapping=snapshot_mapping(quality_mapping),
                left_q=left_q,
                right_q=right_q,
                baseline=snapshot_mapping(self.baseline()),
                gps=snapshot_mapping({side: self.gps(side) for side in ("left", "right")}),
                sampling_active=bool(getattr(self.panel, "baseline_sampling_active", False)),
                sample_count=_safe_len(getattr(self.panel, "physical_alignment_samples", ())),
                iterations=snapshot_mapping(
                    {"rows": tuple(getattr(self.panel, "alignment_iteration_history", ()) or ())}
                )["rows"],
            )
        )

    def detection(self, side: str, *, now_s: float | None = None) -> dict[str, object]:
        now = time.monotonic() if now_s is None else float(now_s)
        observation = getattr(self.panel, f"latest_{side}", None)
        stats_all = getattr(self.panel, "camera_stream_stats", {}) or {}
        stats = stats_all.get(side, {}) if isinstance(stats_all, dict) else {}
        stats = stats if isinstance(stats, dict) else {}
        metadata = getattr(observation, "timestamp_metadata", {}) or {}
        return project_detection(
            side,
            observation=snapshot_attributes(observation, DETECTION_FIELDS),
            stats=snapshot_mapping(
                stats,
                (
                    "decoded_hz",
                    "inferred_hz",
                    "displayed_hz",
                    "yolo_actual_device",
                    "yolo_requested_device",
                    "yolo_cpu_fallback",
                ),
            ),
            metadata=snapshot_mapping(metadata, ("source_timestamp_s", "timestamp_source", "timestamp_reference")),
            enabled=bool(getattr(self.panel, "yolo_enabled", False)),
            now=now,
        )

    def detection_config(self) -> dict[str, object]:
        settings = getattr(self.panel, "settings", None)

        def setting(name: str, default: object = None) -> object:
            value = getattr(settings, name, default)
            value = default if value is None else value
            resolver = getattr(self.panel, "_workstation_setting", None)
            return resolver(name, value) if callable(resolver) else value

        controls = {
            "weights": _widget_text(getattr(self.panel, "weights_edit", None), str(setting("yolo_weights_path", ""))),
            "device": _combo_data(getattr(self.panel, "device_combo", None), setting("yolo_device", "cpu")),
            "class": _widget_checked(
                getattr(self.panel, "yolo_gate_class_checkbox", None), bool(setting("yolo_gate_class_enabled", False))
            ),
            "time": _widget_checked(
                getattr(self.panel, "yolo_gate_matching_checkbox", None),
                bool(setting("yolo_gate_matching_enabled", False)),
            ),
            "Δt": _widget_checked(
                getattr(self.panel, "yolo_gate_time_delta_checkbox", None),
                bool(setting("yolo_gate_time_delta_enabled", False)),
            ),
            "vertical Δ": _widget_checked(
                getattr(self.panel, "yolo_gate_vertical_delta_checkbox", None),
                bool(setting("yolo_gate_vertical_delta_enabled", False)),
            ),
            "input_size_px": _widget_value(getattr(self.panel, "input_size_spin", None), setting("yolo_input_size")),
            "confidence": _widget_value(getattr(self.panel, "conf_spin", None), setting("yolo_conf_threshold")),
            "iou": _widget_value(getattr(self.panel, "iou_spin", None), setting("yolo_iou_threshold")),
            "class_gate": _widget_text(
                getattr(self.panel, "yolo_gate_class_edit", None), str(setting("yolo_gate_class_text", ""))
            ),
            "max_delta_s": _widget_value(
                getattr(self.panel, "yolo_gate_time_delta_spin", None), setting("yolo_gate_max_time_delta_s")
            ),
            "time_basis": _combo_data(
                getattr(self.panel, "yolo_gate_time_basis_combo", None),
                setting("yolo_gate_time_basis", "app_receive_monotonic"),
            ),
            "max_vertical_delta_px": _widget_value(
                getattr(self.panel, "yolo_gate_vertical_delta_spin", None), setting("yolo_gate_max_vertical_delta_px")
            ),
        }
        return project_detection_config(snapshot_mapping(controls))

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
            value = default if value is None else value
            resolver = getattr(self.panel, "_workstation_setting", None)
            return resolver(name, value) if callable(resolver) else value

        controls = {
            "intrinsics_source": _combo_data(
                getattr(self.panel, "intrinsics_source_combo", None),
                "manual"
                if _widget_checked(
                    getattr(self.panel, "manual_focal_checkbox", None), bool(setting("manual_focal_override", False))
                )
                else "checkerboard",
            ),
            "distance_mode": _combo_data(
                getattr(self.panel, "distance_mode_combo", None), setting("triangulation_extrinsics_source", "pixhawk")
            ),
            "manual_focal_override": _widget_checked(
                getattr(self.panel, "manual_focal_checkbox", None), bool(setting("manual_focal_override", False))
            ),
            "focal_px": _widget_value(getattr(self.panel, "focal_spin", None), None),
            "measurement_preferred_method": setting("measurement_preferred_method", "pixhawk_rtk_live"),
            "measurement_method_fallback_order": setting("measurement_method_fallback_order", ()),
            "measurement_baseline_yaw_reverse": setting("measurement_baseline_yaw_reverse", False),
            "pixhawk_left_gnss_source": setting("pixhawk_left_gnss_source", "GPS1"),
            "pixhawk_right_gnss_source": setting("pixhawk_right_gnss_source", "GPS1"),
            "yolo_input_size_px": _widget_value(
                getattr(self.panel, "input_size_spin", None), setting("yolo_input_size")
            ),
            "yolo_confidence": _widget_value(getattr(self.panel, "conf_spin", None), setting("yolo_conf_threshold")),
            "yolo_iou": _widget_value(getattr(self.panel, "iou_spin", None), setting("yolo_iou_threshold")),
            "class_gate_enabled": _widget_checked(
                getattr(self.panel, "yolo_gate_class_checkbox", None), bool(setting("yolo_gate_class_enabled", False))
            ),
            "class_gate": _widget_text(
                getattr(self.panel, "yolo_gate_class_edit", None), str(setting("yolo_gate_class_text", ""))
            ),
            "time_basis": _combo_data(
                getattr(self.panel, "yolo_gate_time_basis_combo", None),
                setting("yolo_gate_time_basis", "app_receive_monotonic"),
            ),
            "max_pair_delta_s": _widget_value(
                getattr(self.panel, "yolo_gate_time_delta_spin", None), setting("yolo_gate_max_time_delta_s")
            ),
            "vertical_delta_gate_enabled": _widget_checked(
                getattr(self.panel, "yolo_gate_vertical_delta_checkbox", None),
                bool(setting("yolo_gate_vertical_delta_enabled", False)),
            ),
            "max_vertical_delta_px": _widget_value(
                getattr(self.panel, "yolo_gate_vertical_delta_spin", None), setting("yolo_gate_max_vertical_delta_px")
            ),
            "camera_sync_history_window_s": _widget_value(
                getattr(self.panel, "camera_sync_history_window_spin", None), setting("camera_sync_history_window_s")
            ),
            "camera_sync_receive_pair_max_delta_ms": _widget_value(
                getattr(self.panel, "camera_sync_receive_pair_max_delta_spin", None),
                _number(setting("camera_sync_receive_pair_max_delta_s"), 0.0) * 1000.0,
            ),
            "camera_sync_rtcp_pair_max_delta_ms": _widget_value(
                getattr(self.panel, "camera_sync_rtcp_pair_max_delta_spin", None),
                _number(setting("camera_sync_rtcp_pair_max_delta_s"), 0.0) * 1000.0,
            ),
            "camera_sync_max_sr_age_s": _widget_value(
                getattr(self.panel, "camera_sync_max_sr_age_spin", None), setting("camera_sync_max_sr_age_s")
            ),
            "camera_sync_max_pc_offset_ms": _widget_value(
                getattr(self.panel, "camera_sync_max_pc_offset_spin", None),
                _number(setting("camera_sync_max_pc_offset_s"), 0.0) * 1000.0,
            ),
            "camera_sync_clock_rate_error_pct": _widget_value(
                getattr(self.panel, "camera_sync_clock_rate_error_spin", None),
                _number(setting("camera_sync_max_clock_rate_error"), 0.0) * 100.0,
            ),
            "camera_sync_required_sr_count": _widget_value(
                getattr(self.panel, "camera_sync_required_sr_count_spin", None),
                setting("camera_sync_required_sr_count"),
            ),
            "camera_sync_warmup_timeout_s": _widget_value(
                getattr(self.panel, "camera_sync_warmup_timeout_spin", None), setting("camera_sync_warmup_timeout_s")
            ),
            "camera_sync_mode": _combo_data(
                getattr(self.panel, "camera_sync_mode_combo", None), setting("camera_sync_mode", "manual")
            ),
            "camera_sync_interval_s": _widget_value(
                getattr(self.panel, "camera_sync_interval_spin", None),
                _number(setting("camera_sync_interval_s"), 1800.0) / 60.0,
            ),
        }
        return project_measurement_settings(snapshot_mapping(controls))

    def image_format_status(self) -> dict[str, object]:
        """Latest left/right GIVI/GIRJ/GISR/GIHR/GILC snapshot, if queried."""

        return thaw_snapshot(snapshot_mapping(getattr(self.panel, "wiris_image_format_status", {}) or {}))

    def calibration_setup(self) -> dict[str, object]:
        """Project editable calibration setup fields without changing them."""

        settings = getattr(self.panel, "settings", None)
        pair_max_delta_s = _number(
            getattr(settings, "calibration_pair_max_delta_s", 0.100),
            0.100,
        )
        controls = {
            "zoom_level": _widget_value(getattr(self.panel, "zoom_level_spin", None)),
            "inner_cols": _widget_value(getattr(self.panel, "calib_cols_spin", None)),
            "inner_rows": _widget_value(getattr(self.panel, "calib_rows_spin", None)),
            "square_size_mm": _widget_value(getattr(self.panel, "calib_square_spin", None)),
            "min_images": _widget_value(getattr(self.panel, "calib_min_images_spin", None)),
            "calculation_pair_count": _widget_value(getattr(self.panel, "calib_calculation_pairs_spin", None)),
            "pair_max_delta_ms": _widget_value(
                getattr(self.panel, "calibration_pair_max_delta_spin", None), float(pair_max_delta_s or 0.0) * 1000.0
            ),
            "auto_interval_s": _widget_value(getattr(self.panel, "calibration_auto_interval_spin", None)),
            "calibration_root": _widget_text(getattr(self.panel, "calib_dir_edit", None)),
            "calibration_root_resolved": str(self.panel._calibration_dirs()[0])
            if hasattr(self.panel, "_calibration_dirs")
            else "",
            "validation_mode": _combo_data(getattr(self.panel, "checkerboard_validation_mode_combo", None), "pixhawk"),
            "validation_root": _widget_text(getattr(self.panel, "checkerboard_validation_root_edit", None)),
            "validation_json": _widget_text(getattr(self.panel, "checkerboard_validation_json_edit", None)),
            "validation_runs": _widget_value(getattr(self.panel, "checkerboard_validation_random_runs_spin", None)),
            "validation_pair_count": _widget_value(
                getattr(self.panel, "checkerboard_validation_random_pair_count_spin", None)
            ),
            "validation_seed": _widget_value(getattr(self.panel, "checkerboard_validation_seed_spin", None)),
        }
        return project_calibration_setup(snapshot_mapping(controls))

    def session_catalog(self) -> tuple[dict[str, object], ...]:
        return SessionEvidenceQuery(self._app_root()).session_catalog()

    def session_artifact_rows(
        self, session_id: str, artifact_name: str, *, measurement_id: str | None = None, limit: int = 100
    ) -> dict[str, object]:
        return SessionEvidenceQuery(self._app_root()).session_artifact_rows(
            session_id,
            artifact_name,
            measurement_id=measurement_id,
            limit=limit,
        )

    def recording(self, *, now_s: float | None = None) -> dict[str, object]:
        now = time.monotonic() if now_s is None else float(now_s)
        return self._capture_recording(now).project()

    def _capture_recording(self, now):
        hierarchy = bool(getattr(self.panel, "_measurement_session_hierarchy_enabled", True))
        storage, run = {}, {}
        if not hierarchy:
            storage_reader = getattr(self.panel, "data_storage_snapshot", None)
            run_reader = getattr(self.panel, "measurement_run_snapshot", None)
            try:
                storage = storage_reader() if callable(storage_reader) else {}
            except Exception as exc:
                storage = {"state": "UNAVAILABLE", "error": str(exc), "streams": {}}
            try:
                run = run_reader() if callable(run_reader) else {}
            except Exception:
                run = {}
        session_id = str(getattr(self.panel, "recording_session_id", "") or "") if hierarchy else ""
        output_dir = None
        if hierarchy and session_id:
            reader = getattr(self.panel, "_recording_session_dir", None)
            if callable(reader):
                try:
                    output_dir = Path(reader(session_id))
                except (OSError, TypeError, ValueError):
                    pass
        artifacts = []
        if output_dir is not None and output_dir.exists():
            try:
                artifacts = sorted(path.name for path in output_dir.iterdir() if path.is_file())[:64]
            except OSError:
                pass
        active = bool(
            getattr(self.panel, "distance_recording_enabled", False)
            or getattr(self.panel, "video_recording_enabled", False)
        )
        active_reader = getattr(self.panel, "_session_recording_active", None)
        if hierarchy and callable(active_reader):
            try:
                active = bool(active_reader())
            except Exception:
                pass
        reader = getattr(self.panel, "_recording_elapsed_s", None)
        try:
            elapsed = _number(reader(now)) if callable(reader) else None
        except Exception:
            elapsed = None
        paths = getattr(self.panel, "video_recording_paths", {}) or {}
        video_paths = {str(side): str(path) for side, path in paths.items()} if isinstance(paths, dict) else {}
        return RecordingInputs(
            hierarchy,
            active,
            identity={
                "session_id": session_id,
                "measurement_id": str(getattr(self.panel, "_measurement_id", "") or ""),
                "mode": str(getattr(self.panel, "_recording_mode", "manual") or "manual"),
            },
            files={
                "output_dir": str(output_dir) if output_dir is not None else "",
                "artifacts": tuple(artifacts),
                "video_paths": video_paths,
            },
            progress={
                "elapsed_s": elapsed,
                "status_label": _widget_text(getattr(self.panel, "recording_status_label", None), ""),
                "saving": getattr(self.panel, "_session_save_thread", None) is not None,
                "retry": bool(getattr(self.panel, "_session_save_retry_required", False)),
                "finalized": bool(getattr(self.panel, "_session_save_finalized", False)),
                "recovered": getattr(self.panel, "_recovered_session_metadata", None) is not None,
            },
            sample_counts={
                "pairs": _safe_len(getattr(self.panel, "distance_recording_rows", ())),
                "camera_left": _safe_len((getattr(self.panel, "camera_recording_rows", {}) or {}).get("left", ())),
                "camera_right": _safe_len((getattr(self.panel, "camera_recording_rows", {}) or {}).get("right", ())),
                "px4_left": _safe_len((getattr(self.panel, "px4_recording_rows", {}) or {}).get("left", ())),
                "px4_right": _safe_len((getattr(self.panel, "px4_recording_rows", {}) or {}).get("right", ())),
                "gnss_base": _safe_len(getattr(self.panel, "gnss_base_recording_rows", ())),
            },
            storage=storage,
            run=run,
        )

    @staticmethod
    def _camera_intrinsics(camera: object | None) -> dict[str, object]:
        return project_camera_intrinsics(_capture_camera_intrinsics(camera))

    def calibration(self) -> dict[str, object]:
        return self._capture_calibration().project()

    def _capture_calibration(self):
        active = getattr(self.panel, "active_intrinsics", None)
        latest = getattr(self.panel, "last_calibration_result", None)
        result = active if active is not None else latest
        candidate_profile = getattr(self.panel, "selected_extrinsics_profile", None)
        profile = (
            getattr(self.panel, "active_extrinsics_profile", None)
            if active is not None and hasattr(self.panel, "active_extrinsics_profile")
            else candidate_profile
        )
        extrinsics = None if result is None else getattr(result, "extrinsics", None)
        if extrinsics is None and profile is not None:
            extrinsics = getattr(profile, "extrinsics", None)
        extrinsics_values = None if extrinsics is None else dict(snapshot_attributes(extrinsics, EXTRINSICS_FIELDS))
        if extrinsics_values is not None:
            extrinsics_values["translation_left_to_right_m"] = tuple(
                getattr(extrinsics, "translation_left_to_right_m", ()) or ()
            )
            extrinsics_values["rotation_left_to_right"] = tuple(
                tuple(row) for row in (getattr(extrinsics, "rotation_left_to_right", ()) or ())
            )
        controller = getattr(self.panel, "_calibration_workflow_controller_owner", None)
        calibration_status = _widget_text(getattr(self.panel, "calibration_status_label", None), "")
        capture_status, capture_error = {}, ""
        for owner in (self.panel, controller):
            reader = getattr(owner, "workstation_calibration_capture_snapshot", None)
            if not callable(reader):
                continue
            try:
                value = reader()
            except Exception as exc:
                capture_error = str(exc)
                break
            if isinstance(value, Mapping):
                capture_status = dict(value)
                break
        if not capture_status:
            capture_status = {
                "schema": "workstation-calibration-capture-status-v1",
                "capture_root": "",
                "capture_set_id": "",
                "detection": {},
                "run": {},
                "capture_set": {
                    "state": "UNAVAILABLE",
                    "error": capture_error or "calibration capture snapshot unavailable",
                },
                "last": {"state": "UNKNOWN", "message": calibration_status},
            }
        runs = getattr(self.panel, "checkerboard_validation_run_results", ()) or ()
        runs = tuple(dict(row) for row in runs if isinstance(row, dict)) if isinstance(runs, (tuple, list)) else ()
        quality = getattr(self.panel, "calibration_quality", None)
        result_quality = getattr(result, "calibration_quality", None) if result is not None else None
        return CalibrationInputs(
            geometry={
                "active": active is not None,
                "candidate_available": latest is not None,
                "result_available": result is not None,
                "left": _capture_camera_intrinsics(None if result is None else getattr(result, "left", None)),
                "right": _capture_camera_intrinsics(None if result is None else getattr(result, "right", None)),
                "extrinsics": extrinsics_values,
            },
            profiles={
                "selected": snapshot_attributes(profile, PROFILE_FIELDS),
                "candidate_id": str(getattr(candidate_profile, "profile_id", "") or ""),
                "active_intrinsics_path": str(getattr(self.panel, "_active_intrinsics_profile_path", "") or ""),
                "candidate_intrinsics_path": str(getattr(self.panel, "_candidate_intrinsics_profile_path", "") or ""),
                "active_intrinsics_zoom": getattr(self.panel, "_active_intrinsics_source_zoom_level", None),
                "candidate_intrinsics_zoom": getattr(self.panel, "last_calibration_zoom_level", None),
                "active_extrinsics_zoom": getattr(self.panel, "_active_extrinsics_source_zoom_level", None),
                "active_extrinsics_path": getattr(self.panel, "_active_extrinsics_profile_path", ""),
                "selected_extrinsics_path": getattr(self.panel, "_selected_extrinsics_profile_path", ""),
                "intrinsics_hash": str(getattr(self.panel, "_active_intrinsics_profile_hash", "") or ""),
                "active_extrinsics_hash": getattr(self.panel, "_active_extrinsics_profile_hash", ""),
                "selected_extrinsics_hash": getattr(self.panel, "_selected_extrinsics_profile_hash", ""),
                "selection_mode": getattr(self.panel, "_active_calibration_selection_mode", ""),
            },
            capture={
                "active": bool(getattr(controller, "_auto_capture_active", False)),
                "saved_pairs": getattr(controller, "_auto_capture_saved_pairs", None),
                "status": capture_status,
            },
            display={
                "calibration_status": calibration_status,
                "calculation_status": _widget_text(
                    getattr(self.panel, "checkerboard_validation_status_label", None), ""
                ),
                "calculation_active": getattr(self.panel, "checkerboard_calibration_thread", None) is not None,
                "zoom": _widget_value(getattr(self.panel, "zoom_level_spin", None)),
                "actual_zoom": (getattr(self.panel, "actual_visible_zoom", {}) or {})
                if isinstance(getattr(self.panel, "actual_visible_zoom", {}), dict)
                else {},
                "progress": (getattr(self.panel, "calibration_progress", {}) or {})
                if isinstance(getattr(self.panel, "calibration_progress", {}), dict)
                else {},
            },
            validation={
                "selected_mode": str(
                    _combo_data(getattr(self.panel, "checkerboard_validation_mode_combo", None), "pixhawk") or "pixhawk"
                ),
                "result_mode": str(getattr(self.panel, "checkerboard_validation_result_mode", "") or ""),
                "runs": runs,
                "summary": dict(getattr(self.panel, "checkerboard_validation_summary", {}) or {}),
                "quality": quality if isinstance(quality, Mapping) else None,
                "result_quality": result_quality if isinstance(result_quality, Mapping) else None,
            },
        )

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

    def _magnetometer_projection(self, side, now, *, needs_attitude=True):
        def side_value(name, default=None):
            store = getattr(self.panel, name, {}) or {}
            return store.get(side, default) if isinstance(store, Mapping) else default

        def samples(name):
            values = side_value(name, {})
            return (
                {key: snapshot_attributes(value, MAG_SAMPLE_FIELDS) for key, value in values.items()}
                if isinstance(values, Mapping)
                else {}
            )

        status = side_value("_px4_mag_status_cache")
        params = getattr(status, "params", None)
        params = params if isinstance(params, Mapping) else None
        slots = tuple(snapshot_attributes(slot, MAG_SLOT_FIELDS) for slot in (getattr(status, "slots", ()) or ()))
        attitude_status = side_value("_px4_attitude_cal_status_cache")
        board = getattr(attitude_status, "params", None)
        board = board if isinstance(board, Mapping) else None
        if board is None or _integer(board.get("SENS_BOARD_ROT")) is None:
            rotation = side_value("_px4_board_rotation_params")
            if isinstance(rotation, Mapping) and _integer(rotation.get("SENS_BOARD_ROT")) is not None:
                board = {**(board or {}), "SENS_BOARD_ROT": rotation["SENS_BOARD_ROT"]}
        inputs = MagnetometerInputs(
            samples("latest_mag_sensors"),
            samples("latest_can_mag_sensors"),
            slots,
            side_value("latest_vehicle_mag_device_ids"),
            params,
            board,
            self.attitude(side, now_s=now) if needs_attitude else {},
        )
        return MagnetometerProjection(inputs, now=now)
