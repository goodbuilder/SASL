"""Active Drone Pixhawk telemetry and RTCM routing owner.

This path is intentionally separate from ``target_drone_controller``.  The
target-drone artifact is receive-only; this controller owns the explicitly
operator-enabled third link that receives GPS telemetry and may receive Base
RTCM through MAVLink ``GPS_RTCM_DATA``.
"""

from __future__ import annotations

import math
import time
from collections.abc import Mapping

from PyQt5 import QtCore

from Workstation.device_runtime_workers import (
    DroneNetworkDiscoveryWorker,
    PixhawkConnectionWorker,
    PixhawkUsbDiscoveryWorker,
)
from Workstation.workstation_core.gnss_baseline_center import (
    DEFAULT_MAXIMUM_GNSS_AGE_S,
    DEFAULT_MAXIMUM_GNSS_TIMESTAMP_SPREAD_S,
    estimate_gnss_baseline_center_frd,
)
from Workstation.workstation_core.ideal_disparity import estimate_ideal_disparity
from Workstation.workstation_core.wiris_zoom import nominal_focal_length_px


DRONE_RTCM_SOURCE_BASE = "base"
DRONE_RTCM_SOURCE_NTRIP = "ntrip"
DRONE_RTCM_SOURCE_LABELS = {
    DRONE_RTCM_SOURCE_BASE: "Base GNSS",
    DRONE_RTCM_SOURCE_NTRIP: "Network NTRIP",
}


def normalize_drone_rtcm_source(value: object) -> str:
    """Normalize the correction source selected for the active Drone link."""

    normalized = str(value or "").strip().lower().replace("_", " ")
    if normalized in {"ntrip", "network", "network ntrip", "network-ntrip"}:
        return DRONE_RTCM_SOURCE_NTRIP
    return DRONE_RTCM_SOURCE_BASE


def drone_rtcm_source_label(value: object) -> str:
    return DRONE_RTCM_SOURCE_LABELS[normalize_drone_rtcm_source(value)]


DRONE_TELEMETRY_CONTROLLER_METHODS = (
    "configure_drone_telemetry",
    "connect_drone_telemetry",
    "disconnect_drone_telemetry",
    "save_drone_telemetry_settings",
    "drone_telemetry_snapshot",
    "discover_drone_telemetry_usb",
    "discover_drone_telemetry_network",
    "use_drone_telemetry_network_candidate",
)


def _positive_baud(value: object) -> int:
    if isinstance(value, bool):
        raise ValueError("drone telemetry baud must be a positive integer")
    parsed = int(value)
    if parsed <= 0:
        raise ValueError("drone telemetry baud must be a positive integer")
    return parsed


def _drone_usb_endpoint(panel: object) -> str:
    """Read the Drone USB serial path from the bootstrap widget it lives in.

    USB transport reuses the bootstrap endpoint that recovery and firmware work
    already own, so there is one device path per board to keep current.
    """

    widget = getattr(panel, "drone_pixhawk_bootstrap_url_edit", None)
    reader = getattr(widget, "text", None)
    if callable(reader):
        return str(reader()).strip()
    return str(getattr(panel, "drone_pixhawk_bootstrap_url", "") or "").strip()


def _drone_usb_baud(panel: object) -> int:
    widget = getattr(panel, "drone_pixhawk_bootstrap_baud_spin", None)
    reader = getattr(widget, "value", None)
    if callable(reader):
        return int(reader())
    return int(getattr(panel, "drone_pixhawk_bootstrap_baud", 57_600))


def _set_drone_usb_endpoint(panel: object, device: str, baud: int) -> None:
    url_widget = getattr(panel, "drone_pixhawk_bootstrap_url_edit", None)
    url_setter = getattr(url_widget, "setText", None)
    if callable(url_setter):
        url_setter(device)
    else:
        setattr(panel, "drone_pixhawk_bootstrap_url", device)
    baud_widget = getattr(panel, "drone_pixhawk_bootstrap_baud_spin", None)
    baud_setter = getattr(baud_widget, "setValue", None)
    if callable(baud_setter):
        baud_setter(baud)
    else:
        setattr(panel, "drone_pixhawk_bootstrap_baud", baud)


def _gps_mapping(gps: object, *, now_s: float) -> dict[str, object]:
    if gps is None:
        return {
            "state": "NO DATA",
            "rtk_label": "UNKNOWN",
            "age_s": None,
            "latitude_deg": None,
            "longitude_deg": None,
            "altitude_m": None,
            "fix_type": None,
            "satellites": None,
            "eph_m": None,
            "epv_m": None,
            "topic": "",
        }
    timestamp_s = getattr(gps, "timestamp_s", None)
    age_s = None
    if timestamp_s is not None:
        try:
            age_s = max(0.0, float(now_s) - float(timestamp_s))
        except (TypeError, ValueError):
            age_s = None
    position_valid = bool(getattr(gps, "has_position", False))
    return {
        "state": "LIVE" if position_valid else "NO POSITION",
        "rtk_label": str(getattr(gps, "rtk_label", "UNKNOWN")),
        "age_s": age_s,
        "latitude_deg": getattr(gps, "latitude_deg", None),
        "longitude_deg": getattr(gps, "longitude_deg", None),
        "altitude_m": getattr(gps, "altitude_m", None),
        "fix_type": getattr(gps, "fix_type", None),
        "satellites": getattr(gps, "satellites_used", None),
        "eph_m": getattr(gps, "eph_m", None),
        "epv_m": getattr(gps, "epv_m", None),
        "topic": getattr(gps, "topic", ""),
        "source_system": getattr(gps, "mavlink_source_system", None),
        "source_component": getattr(gps, "mavlink_source_component", None),
    }


class DroneTelemetryController:
    """Configure and own one active third Pixhawk MAVLink endpoint."""

    def __init__(self, panel) -> None:
        self.panel = panel

    def _sync_rtcm_pipeline_route(self, *, ensure_injection: bool = False) -> None:
        owner = getattr(self.panel, "_gnss_runtime_controller_owner", None)
        if owner is None:
            return
        if ensure_injection:
            owner._ensure_pixhawk_rtcm_worker()
        owner._sync_rtcm_pipeline_config()

    def configure_drone_telemetry(self, payload: Mapping[str, object] | None = None) -> dict[str, object]:
        values = dict(payload or {})
        transport = str(
            values.get(
                "drone_pixhawk_transport",
                getattr(self.panel, "drone_pixhawk_transport", "radio"),
            )
            or "radio"
        ).strip().lower()
        if transport not in {"ethernet", "radio", "usb"}:
            transport = "radio"
        endpoint = str(
            values.get("drone_telemetry_url", values.get("endpoint", getattr(self.panel, "drone_telemetry_url", "")))
            or ""
        ).strip()
        if transport == "ethernet" and not endpoint:
            raise ValueError("Drone Telemetry MAVLink URL이 비어 있습니다.")
        baud = _positive_baud(
            values.get("drone_telemetry_baud", values.get("baud", getattr(self.panel, "drone_telemetry_baud", 57_600)))
        )
        forward_rtcm = bool(
            values.get(
                "drone_telemetry_rtcm_forward",
                values.get("forward_rtcm", getattr(self.panel, "drone_telemetry_rtcm_forward", True)),
            )
        )
        rtcm_source = normalize_drone_rtcm_source(
            values.get(
                "drone_telemetry_rtcm_source",
                values.get("rtcm_source", getattr(self.panel, "drone_telemetry_rtcm_source", "base")),
            )
        )
        # Radio and USB each keep their own saved serial endpoint; apply the
        # submitted one to whichever the selected transport will open.
        serial_device = values.get("drone_pixhawk_serial_device")
        if serial_device is not None and transport in {"radio", "usb"}:
            serial_baud = _positive_baud(
                values.get("drone_pixhawk_serial_baud", baud)
            )
            if transport == "radio":
                self.panel.drone_pixhawk_radio_device = str(serial_device).strip()
                self.panel.drone_pixhawk_radio_baud = serial_baud
            else:
                _set_drone_usb_endpoint(self.panel, str(serial_device).strip(), serial_baud)
        subscriber = getattr(self.panel, "mavlink_pixhawk", None)
        existing_endpoint = ""
        if subscriber is not None:
            existing_endpoint = str(getattr(subscriber, "urls", {}).get("drone", "") or "")
            if (
                transport == "ethernet"
                and "drone" in getattr(subscriber, "connections", {})
                and existing_endpoint != endpoint
            ):
                subscriber.stop("drone")
                self.panel.drone_telemetry_connection_state = "DISCONNECTED"
        self.panel.drone_telemetry_url = endpoint
        self.panel.drone_telemetry_baud = baud
        self.panel.drone_telemetry_rtcm_forward = forward_rtcm
        self.panel.drone_telemetry_rtcm_source = rtcm_source
        self.panel.drone_pixhawk_transport = transport
        self._sync_rtcm_pipeline_route()
        if subscriber is not None:
            if transport == "radio":
                subscriber.urls["drone"] = str(
                    getattr(self.panel, "drone_pixhawk_radio_device", "") or ""
                ).strip()
                subscriber.baudrates["drone"] = int(
                    getattr(self.panel, "drone_pixhawk_radio_baud", 57_600)
                )
            elif transport == "usb":
                subscriber.urls["drone"] = _drone_usb_endpoint(self.panel)
                subscriber.baudrates["drone"] = _drone_usb_baud(self.panel)
            else:
                subscriber.urls["drone"] = endpoint
                subscriber.baudrates["drone"] = baud
            setter = getattr(subscriber, "set_transport", None)
            if callable(setter):
                setter("drone", transport)
        self.panel._log(
            f"[픽스호크] Drone Telemetry 설정: transport={transport} "
            f"endpoint={subscriber.urls['drone'] if subscriber is not None else endpoint} @ "
            f"{subscriber.baudrates['drone'] if subscriber is not None else baud} baud | "
            f"RTCM source={drone_rtcm_source_label(rtcm_source)} | "
            f"forward={'ON' if forward_rtcm else 'OFF'}"
        )
        return self.drone_telemetry_snapshot()

    def save_drone_telemetry_settings(self, payload: Mapping[str, object] | None = None) -> dict[str, object]:
        snapshot = self.configure_drone_telemetry(payload)
        saver = getattr(self.panel, "_save_current_settings", None)
        if callable(saver):
            saver()
        return snapshot

    def connect_drone_telemetry(self, payload: Mapping[str, object] | None = None) -> dict[str, object]:
        if payload:
            self.configure_drone_telemetry(payload)
        connector = getattr(self.panel, "connect_mavlink_pixhawk", None)
        if not callable(connector):
            raise RuntimeError("공용 Pixhawk Connect owner가 없습니다.")
        self.panel.drone_telemetry_connection_state = "CONNECTING"
        self.panel.drone_telemetry_last_error = ""
        connector("drone")
        return self.drone_telemetry_snapshot()

    def _apply_connection_result(self, _result: object) -> None:
        subscriber = getattr(self.panel, "mavlink_pixhawk", None)
        if subscriber is None:
            self._connection_failed("MAVLink subscriber가 사라졌습니다.")
            return
        try:
            subscriber.start_receive_timer()
        except Exception as exc:
            self._connection_failed(str(exc))
            return
        self.panel.drone_telemetry_connection_state = "CONNECTED"
        self.panel.drone_telemetry_last_error = ""
        self._sync_rtcm_pipeline_route(ensure_injection=True)
        self.panel._log("[픽스호크] Drone Telemetry 링크가 열렸습니다. Heartbeat/GPS 수신 대기 중입니다.")
        self.panel._refresh_operation_header()

    def _connection_failed(self, error: str) -> None:
        subscriber = getattr(self.panel, "mavlink_pixhawk", None)
        if subscriber is not None:
            try:
                subscriber.stop("drone")
            except Exception:
                pass
        self.panel.drone_telemetry_connection_state = "ERROR"
        self.panel.drone_telemetry_last_error = str(error)
        self._sync_rtcm_pipeline_route()
        self.panel._log(f"[픽스호크] Drone Telemetry MAVLink 연결 실패: {error}")
        self.panel._refresh_operation_header()

    def _clear_connection_worker(self) -> None:
        self.panel.drone_telemetry_connection_thread = None
        self.panel.drone_telemetry_connection_worker = None

    def discover_drone_telemetry_network(self) -> None:
        if getattr(self.panel, "drone_telemetry_network_discovery_thread", None) is not None:
            raise RuntimeError("Drone network discovery is already running")
        self.panel.drone_telemetry_network_discovery_generation = int(
            getattr(self.panel, "drone_telemetry_network_discovery_generation", 0)
        ) + 1
        self.panel.drone_telemetry_network_candidates = ()
        self.panel.drone_telemetry_network_discovery_state = "SEARCHING"
        thread = QtCore.QThread(self.panel)
        worker = DroneNetworkDiscoveryWorker(timeout_s=3.0)
        worker.moveToThread(thread)
        thread.started.connect(worker.run)
        worker.discovered.connect(self._apply_drone_telemetry_network_discovery)
        worker.failed.connect(self._drone_telemetry_network_discovery_failed)
        worker.finished.connect(thread.quit)
        worker.finished.connect(worker.deleteLater)
        thread.finished.connect(thread.deleteLater)
        thread.finished.connect(self._clear_drone_telemetry_network_discovery)
        self.panel.drone_telemetry_network_discovery_thread = thread
        self.panel.drone_telemetry_network_discovery_worker = worker
        self.panel._log("[픽스호크] Drone read-only network discovery를 시작했습니다.")
        thread.start()

    def _apply_drone_telemetry_network_discovery(self, result: object) -> None:
        rows = tuple(dict(row) for row in (result or ()) if isinstance(row, Mapping))
        self.panel.drone_telemetry_network_candidates = rows
        verified = [row for row in rows if bool(row.get("selectable"))]
        if len(verified) == 1:
            state = "FOUND 1 DRONE"
        elif len(verified) > 1:
            state = f"AMBIGUOUS {len(verified)} DRONES"
        elif rows:
            state = "FOUND OTHER SYSTEMS ONLY"
        else:
            state = "EMPTY"
        self.panel.drone_telemetry_network_discovery_state = state
        self.panel._log(f"[픽스호크] Drone network discovery: {state}")

    def _drone_telemetry_network_discovery_failed(self, error: str) -> None:
        self.panel.drone_telemetry_network_candidates = ()
        self.panel.drone_telemetry_network_discovery_state = f"FAILED · {error}"
        self.panel._log(f"[픽스호크] Drone network discovery 실패: {error}")

    def _clear_drone_telemetry_network_discovery(self) -> None:
        self.panel.drone_telemetry_network_discovery_thread = None
        self.panel.drone_telemetry_network_discovery_worker = None

    def use_drone_telemetry_network_candidate(self, candidate_id: object) -> dict[str, object]:
        selected_id = str(candidate_id or "")
        matches = [
            dict(row)
            for row in getattr(self.panel, "drone_telemetry_network_candidates", ())
            if str(row.get("candidate_id") or "") == selected_id
        ]
        if len(matches) != 1:
            raise ValueError("Select one current Drone network candidate")
        row = matches[0]
        if not bool(row.get("selectable")) or int(row.get("system_id") or -1) != 3:
            raise ValueError("Selected network source is not verified as Drone sysid 3")
        endpoint = str(row.get("endpoint") or "").strip()
        if not endpoint.startswith("udpin:0.0.0.0:"):
            raise ValueError("Selected Drone endpoint is invalid")
        return {"candidate_id": selected_id, "endpoint": endpoint, "candidate": row}

    def disconnect_drone_telemetry(self) -> dict[str, object]:
        disconnect = getattr(self.panel, "disconnect_mavlink_pixhawk", None)
        if callable(disconnect):
            disconnect("drone")
        self.panel.drone_telemetry_connection_state = "DISCONNECTED"
        self._sync_rtcm_pipeline_route()
        self.panel._log("[픽스호크] Drone Telemetry MAVLink 연결을 해제했습니다.")
        self.panel._refresh_operation_header()
        return self.drone_telemetry_snapshot()

    def discover_drone_telemetry_usb(self) -> None:
        """Probe USB serial candidates for a telemetry radio off the UI thread.

        Reuses the same MAVLink-heartbeat probe as the P1/P2 USB discovery
        (``PixhawkUsbDiscoveryWorker``): a paired telemetry radio only answers
        the heartbeat once the air-side radio and its autopilot are powered,
        so an unconfirmed serial candidate is still surfaced (with a warning)
        rather than silently dropped.
        """

        if getattr(self.panel, "drone_telemetry_usb_discovery_thread", None) is not None:
            self.panel._log("[픽스호크] Drone Telemetry USB 탐색이 이미 진행 중입니다.")
            return
        self.panel.drone_telemetry_usb_discovery_state = "SEARCHING"
        self.panel._log("[픽스호크] Drone Telemetry USB 라디오 탐색을 시작합니다.")
        thread = QtCore.QThread(self.panel)
        worker = PixhawkUsbDiscoveryWorker(timeout_s=0.8)
        worker.moveToThread(thread)
        thread.started.connect(worker.run)
        worker.discovered.connect(self._apply_drone_telemetry_usb_discovery)
        worker.failed.connect(self._drone_telemetry_usb_discovery_failed)
        worker.finished.connect(thread.quit)
        worker.finished.connect(worker.deleteLater)
        thread.finished.connect(thread.deleteLater)
        thread.finished.connect(self._clear_drone_telemetry_usb_discovery)
        self.panel.drone_telemetry_usb_discovery_thread = thread
        self.panel.drone_telemetry_usb_discovery_worker = worker
        thread.start()

    def _apply_drone_telemetry_usb_discovery(self, result: object) -> None:
        data = dict(result or {}) if isinstance(result, dict) else {}
        candidates = list(data.get("candidates") or [])
        found = list(data.get("found") or [])
        self.panel.drone_telemetry_usb_candidates = tuple(candidates)
        if found:
            device, baud = found[0]
            self.panel.drone_telemetry_usb_discovery_state = (
                f"CANDIDATE heartbeat {device} @ {baud} · select for Drone then Connect"
            )
            self.panel._log(
                f"[픽스호크] Drone Radio 후보 발견: {device} @ {baud} baud. "
                "자동 배정하지 않습니다; Drone에 선택한 뒤 Connect를 누르세요."
            )
        elif candidates:
            device = candidates[0]
            self.panel.drone_telemetry_usb_discovery_state = f"CANDIDATE no-heartbeat {device}"
            self.panel._log(
                f"[픽스호크] USB 시리얼 장치는 찾았지만 MAVLink heartbeat 응답이 없습니다: {device} "
                "(드론/자동조종장치 전원 및 라디오 페어링을 확인하세요)"
            )
        else:
            self.panel.drone_telemetry_usb_discovery_state = "FOUND 0 USB ENDPOINTS"
            self.panel._log(
                "[픽스호크] Drone Telemetry USB 장치가 보이지 않습니다: "
                "/dev/serial/by-id/*, /dev/ttyACM*, /dev/ttyUSB* 없음"
            )

    def _drone_telemetry_usb_discovery_failed(self, error: str) -> None:
        self.panel.drone_telemetry_usb_discovery_state = f"ERROR · {error}"
        self.panel._log(f"[픽스호크] Drone Telemetry USB 탐색 실패: {error}")

    def _clear_drone_telemetry_usb_discovery(self) -> None:
        self.panel.drone_telemetry_usb_discovery_thread = None
        self.panel.drone_telemetry_usb_discovery_worker = None

    def drone_telemetry_snapshot(self) -> dict[str, object]:
        now_s = time.monotonic()
        subscriber = getattr(self.panel, "mavlink_pixhawk", None)
        connections = getattr(subscriber, "connections", {}) if subscriber is not None else {}
        heartbeat = getattr(subscriber, "_heartbeat_logged", set()) if subscriber is not None else set()
        drone_connection = isinstance(connections, Mapping) and "drone" in connections
        heartbeat_received = "drone" in heartbeat
        if drone_connection and heartbeat_received:
            connection_state = "LIVE"
        elif drone_connection:
            connection_state = "CONNECTED"
        elif getattr(self.panel, "pixhawk_connection_thread", None) is not None:
            connection_state = str(getattr(self.panel, "drone_telemetry_connection_state", "CONNECTING"))
        else:
            connection_state = str(getattr(self.panel, "drone_telemetry_connection_state", "DISCONNECTED"))
        gps_states = getattr(self.panel, "latest_gps", {}) or {}
        drone_gps = gps_states.get("drone") if isinstance(gps_states, Mapping) else None
        selected_reader = getattr(self.panel, "_selected_rover_gps_state", None)
        left_gps = (
            selected_reader("left")
            if callable(selected_reader)
            else gps_states.get("left")
            if isinstance(gps_states, Mapping)
            else None
        )
        right_gps = (
            selected_reader("right")
            if callable(selected_reader)
            else gps_states.get("right")
            if isinstance(gps_states, Mapping)
            else None
        )
        focal_length_px, focal_source = self._focal_length_px()
        gnss_baseline_center = estimate_gnss_baseline_center_frd(
            left_gps,
            right_gps,
            drone_gps,
            now_s=now_s,
            maximum_age_s=DEFAULT_MAXIMUM_GNSS_AGE_S,
            maximum_timestamp_spread_s=(
                DEFAULT_MAXIMUM_GNSS_TIMESTAMP_SPREAD_S
            ),
        )
        estimate = estimate_ideal_disparity(
            left_gps,
            right_gps,
            drone_gps,
            focal_length_px=focal_length_px,
            now_s=now_s,
            maximum_gps_age_s=DEFAULT_MAXIMUM_GNSS_AGE_S,
            maximum_gps_timestamp_spread_s=(
                DEFAULT_MAXIMUM_GNSS_TIMESTAMP_SPREAD_S
            ),
            gnss_estimate=gnss_baseline_center,
        )
        baseline = estimate.baseline
        injection = self._drone_injection_stats()
        zoom_levels = self._zoom_levels()
        rtcm_source = normalize_drone_rtcm_source(
            getattr(self.panel, "drone_telemetry_rtcm_source", DRONE_RTCM_SOURCE_BASE)
        )
        target_distance = estimate.target_distance_m
        target_source = (
            "P1/P2 baseline midpoint → Drone GNSS"
            if target_distance is not None
            else "waiting for three GNSS positions"
        )
        drone_transport = str(getattr(self.panel, "drone_pixhawk_transport", "radio") or "radio")
        if drone_transport == "radio":
            drone_endpoint = str(getattr(self.panel, "drone_pixhawk_radio_device", "") or "")
            drone_baud = int(getattr(self.panel, "drone_pixhawk_radio_baud", 57_600))
        elif drone_transport == "usb":
            drone_endpoint = _drone_usb_endpoint(self.panel)
            drone_baud = _drone_usb_baud(self.panel)
        else:
            drone_endpoint = str(getattr(self.panel, "drone_telemetry_url", "") or "")
            drone_baud = int(getattr(self.panel, "drone_telemetry_baud", 57_600))
        return {
            "source_id": "drone_telemetry",
            "usb_discovery_state": str(getattr(self.panel, "drone_telemetry_usb_discovery_state", "") or ""),
            "usb_discovery_active": getattr(self.panel, "drone_telemetry_usb_discovery_thread", None) is not None,
            "network_discovery_state": str(
                getattr(self.panel, "drone_telemetry_network_discovery_state", "IDLE") or "IDLE"
            ),
            "network_discovery_active": getattr(
                self.panel, "drone_telemetry_network_discovery_thread", None
            ) is not None,
            "network_candidates": tuple(
                dict(row)
                for row in getattr(self.panel, "drone_telemetry_network_candidates", ())
                if isinstance(row, Mapping)
            ),
            "transport": drone_transport,
            "endpoint": drone_endpoint,
            "baud": drone_baud,
            "connection_state": connection_state,
            "heartbeat_received": heartbeat_received,
            "forward_rtcm": bool(getattr(self.panel, "drone_telemetry_rtcm_forward", True)),
            "rtcm_source": rtcm_source,
            "rtcm_source_label": drone_rtcm_source_label(rtcm_source),
            "last_error": str(getattr(self.panel, "drone_telemetry_last_error", "") or ""),
            "position": _gps_mapping(drone_gps, now_s=now_s),
            "left_position": _gps_mapping(left_gps, now_s=now_s),
            "right_position": _gps_mapping(right_gps, now_s=now_s),
            "rtcm": injection,
            "focal_length_px": focal_length_px,
            "focal_source": focal_source,
            "zoom_levels": {"left": zoom_levels[0], "right": zoom_levels[1]},
            "target_distance_m": target_distance,
            "automatic_target_distance_m": target_distance,
            "target_distance_used_m": target_distance,
            "target_distance_source": target_source,
            "gnss_baseline_center_frd": gnss_baseline_center.to_mapping(),
            "ideal_disparity": {
                "state": estimate.state,
                "reason": estimate.reason,
                "baseline_m": estimate.baseline_m,
                "east_m": None if baseline is None else baseline.east_m,
                "north_m": None if baseline is None else baseline.north_m,
                "up_m": None if baseline is None else baseline.up_m,
                "timestamp_delta_ms": None if baseline is None else baseline.timestamp_delta_ms,
                "focal_length_px": estimate.focal_length_px,
                "target_distance_m": estimate.target_distance_m,
                "disparity_px": estimate.disparity_px,
            },
        }

    def _drone_injection_stats(self) -> dict[str, object]:
        stats = getattr(self.panel, "gnss_base_rtcm_stats", {}) or {}
        injection = stats.get("injection", {}) if isinstance(stats, Mapping) else {}
        drone = injection.get("drone", {}) if isinstance(injection, Mapping) else {}
        if not isinstance(drone, Mapping):
            drone = {}
        last_delivery_s = drone.get("last_delivery_s")
        age_s = None
        if last_delivery_s is not None:
            try:
                age_s = max(0.0, time.monotonic() - float(last_delivery_s))
            except (TypeError, ValueError):
                pass
        return {
            "frames": int(drone.get("frames") or 0),
            "bytes": int(drone.get("bytes") or 0),
            "packets": int(drone.get("packets") or 0),
            "fail": int(drone.get("fail") or 0),
            "last_delivery_age_s": age_s,
        }

    def _zoom_levels(self) -> tuple[int | None, int | None]:
        actual = getattr(self.panel, "actual_visible_zoom", {}) or {}
        selected_widget = getattr(self.panel, "zoom_level_spin", None)
        selected = None
        if selected_widget is not None:
            try:
                selected = int(selected_widget.value())
            except (AttributeError, TypeError, ValueError):
                selected = None
        calibration_zoom = getattr(self.panel, "_active_intrinsics_source_zoom_level", None)
        if calibration_zoom is None:
            calibration_zoom = getattr(self.panel, "last_calibration_zoom_level", None)
        try:
            calibration_zoom = None if calibration_zoom is None else int(calibration_zoom)
        except (TypeError, ValueError):
            calibration_zoom = None
        levels: list[int | None] = []
        for side in ("left", "right"):
            state = actual.get(side, {}) if isinstance(actual, Mapping) else {}
            try:
                level = int(state.get("index")) if isinstance(state, Mapping) and state.get("index") is not None else None
            except (TypeError, ValueError):
                level = None
            levels.append(level if level is not None else calibration_zoom if calibration_zoom is not None else selected)
        return levels[0], levels[1]

    def _focal_length_px(self) -> tuple[float | None, str]:
        zoom_levels = self._zoom_levels()
        calibration_zoom = getattr(self.panel, "_active_intrinsics_source_zoom_level", None)
        if calibration_zoom is None:
            calibration_zoom = getattr(self.panel, "last_calibration_zoom_level", None)
        try:
            calibration_zoom = None if calibration_zoom is None else int(calibration_zoom)
        except (TypeError, ValueError):
            calibration_zoom = None
        calibrated_focal: float | None = None
        calibrated_source = ""
        for name, source in (
            ("active_intrinsics", "active stereo calibration fx average"),
            ("last_calibration_result", "latest stereo calibration fx average"),
        ):
            result = getattr(self.panel, name, None)
            if result is None:
                continue
            try:
                left_fx = float(result.left.camera_matrix[0][0])
                right_fx = float(result.right.camera_matrix[0][0])
            except (AttributeError, IndexError, TypeError, ValueError):
                continue
            if all(math.isfinite(value) and value > 0.0 for value in (left_fx, right_fx)):
                calibrated_focal = (left_fx + right_fx) / 2.0
                calibrated_source = source
                break
        if calibrated_focal is not None and (
            calibration_zoom is None
            or all(level is None or level == calibration_zoom for level in zoom_levels)
        ):
            zoom_text = "" if calibration_zoom is None else f" (zoom {calibration_zoom})"
            return calibrated_focal, f"{calibrated_source}{zoom_text}"
        nominal = [nominal_focal_length_px(level) for level in zoom_levels]
        if all(value is not None and value > 0.0 for value in nominal):
            assert nominal[0] is not None and nominal[1] is not None
            level_text = f"L{zoom_levels[0]} / R{zoom_levels[1]}"
            return (nominal[0] + nominal[1]) / 2.0, f"WIRIS zoom nominal fx ({level_text})"
        if calibrated_focal is not None:
            zoom_text = "" if calibration_zoom is None else f" (zoom {calibration_zoom})"
            return calibrated_focal, f"{calibrated_source}{zoom_text}"
        return None, "zoom/calibration focal length unavailable"
