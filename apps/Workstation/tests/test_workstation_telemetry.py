from __future__ import annotations

import time
from types import SimpleNamespace

from PyQt5 import QtWidgets

from Workstation.drone_telemetry_controller import DroneTelemetryController
from Workstation.gnss_runtime_controller import GnssRuntimeController
from Workstation.workspaces import TelemetryWorkspace
from Workstation.workstation_core.gnss_operation import GnssOperationMode
from Workstation.workstation_core.ideal_disparity import (
    estimate_ideal_disparity,
    target_distance_from_gps,
)
from Workstation.workstation_core.rtk_geometry import GpsPositionState


def _gps(
    side: str,
    *,
    longitude_deg: float,
    latitude_deg: float = 37.0,
    altitude_m: float = 100.0,
    timestamp_s: float | None = None,
) -> GpsPositionState:
    return GpsPositionState(
        camera=side,
        namespace=f"{side}-pixhawk",
        timestamp_s=time.monotonic() if timestamp_s is None else timestamp_s,
        topic="mavlink/GPS_RAW_INT",
        latitude_deg=latitude_deg,
        longitude_deg=longitude_deg,
        altitude_m=altitude_m,
        fix_type=6,
        satellites_used=20,
    )


def test_ideal_disparity_uses_live_gps_baseline_and_fx() -> None:
    left = _gps("left", longitude_deg=127.0)
    right = _gps("right", longitude_deg=127.00001)
    drone = _gps("drone", longitude_deg=127.00010)
    estimate = estimate_ideal_disparity(
        left,
        right,
        drone,
        focal_length_px=900.0,
    )

    assert estimate.state == "READY"
    assert estimate.baseline_m is not None
    assert estimate.target_distance_m == target_distance_from_gps(left, right, drone)
    assert estimate.disparity_px is not None
    assert estimate.disparity_px == estimate.baseline_m * 900.0 / estimate.target_distance_m


def test_ideal_disparity_reports_missing_drone_gnss_without_fabricating_pixels() -> None:
    estimate = estimate_ideal_disparity(
        _gps("left", longitude_deg=127.0),
        _gps("right", longitude_deg=127.00001),
        None,
        focal_length_px=900.0,
    )

    assert estimate.state == "WAITING"
    assert estimate.disparity_px is None
    assert "Drone GNSS" in estimate.reason


def test_drone_telemetry_snapshot_uses_zoom_and_three_gnss_positions() -> None:
    panel = SimpleNamespace(
        mavlink_pixhawk=None,
        latest_gps={
            "left": _gps("left", longitude_deg=127.0),
            "right": _gps("right", longitude_deg=127.00001),
            "drone": _gps("drone", longitude_deg=127.00010),
        },
        actual_visible_zoom={"left": {"index": 8}, "right": {"index": 8}},
        drone_telemetry_url="udpin:0.0.0.0:14552",
        drone_telemetry_baud=57_600,
        drone_telemetry_rtcm_forward=True,
        drone_telemetry_rtcm_source="ntrip",
        drone_telemetry_connection_state="CONNECTED",
        drone_telemetry_last_error="",
        gnss_base_rtcm_stats={},
    )

    snapshot = DroneTelemetryController(panel).drone_telemetry_snapshot()

    assert snapshot["zoom_levels"] == {"left": 8, "right": 8}
    assert snapshot["focal_length_px"] == 8_242.0
    assert snapshot["focal_source"] == "WIRIS zoom nominal fx (L8 / R8)"
    assert snapshot["rtcm_source"] == "ntrip"
    assert snapshot["rtcm_source_label"] == "Network NTRIP"
    assert snapshot["ideal_disparity"]["state"] == "READY"
    assert snapshot["target_distance_source"] == "P1/P2 baseline midpoint → Drone GNSS"
    assert snapshot["ideal_disparity"]["disparity_px"] is not None


def test_drone_rtcm_route_selects_base_or_network_ntrip_source() -> None:
    panel = SimpleNamespace(
        gnss_active_operation_config=SimpleNamespace(
            routing=SimpleNamespace(left_enabled=True, right_enabled=False, drone_enabled=True)
        ),
        drone_telemetry_rtcm_forward=True,
        drone_telemetry_rtcm_source="base",
        mavlink_pixhawk=SimpleNamespace(connections={"drone": object()}),
    )
    controller = GnssRuntimeController(panel)

    panel.gnss_active_operation_config.routing.drone_enabled = False
    assert controller._gnss_base_rtcm_target_sides() == ("left",)
    panel.gnss_active_operation_config.routing.drone_enabled = True
    assert controller._gnss_base_rtcm_target_sides() == ("left", "drone")
    assert controller._gnss_base_rtcm_target_sides(source="ntrip") == ("left",)
    panel.drone_telemetry_rtcm_source = "ntrip"
    assert controller._gnss_base_rtcm_target_sides(source="ntrip") == ("left", "drone")
    assert controller._gnss_base_rtcm_target_sides(
        source="ntrip", include_rover_targets=False
    ) == ("drone",)


def test_network_assisted_mode_can_route_ntrip_frames_to_drone_without_base_rover_route() -> None:
    forwarded: list[tuple[object, dict[str, object]]] = []
    panel = SimpleNamespace(
        gnss_network_rtcm_parser=SimpleNamespace(feed=lambda _payload: [b"rtcm-frame"]),
        _gnss_active_operation_config=lambda: SimpleNamespace(
            operation_mode=GnssOperationMode.NETWORK_ASSISTED_FIXED_BASE,
            routing=SimpleNamespace(drone_enabled=True),
        ),
        gnss_base_serial_worker=None,
        gnss_base_serial_connected=False,
        gnss_base_status_labels={"network": object()},
        _set_health_label=lambda *_args: None,
        drone_telemetry_rtcm_source="ntrip",
        drone_telemetry_rtcm_forward=True,
        _handle_gnss_base_rtcm_frame=lambda frame, **kwargs: forwarded.append((frame, kwargs)),
    )

    GnssRuntimeController(panel)._handle_gnss_network_chunk(b"chunk")

    assert forwarded == [
        (
            b"rtcm-frame",
            {"source": "ntrip", "include_rover_targets": False},
        )
    ]


def test_telemetry_workspace_renders_position_rtcm_and_ideal_disparity() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    snapshot = {
        "endpoint": "udpin:0.0.0.0:14552",
        "baud": 57600,
        "connection_state": "LIVE",
        "heartbeat_received": True,
        "forward_rtcm": True,
        "left_position": {
            "state": "LIVE",
            "fix_type": 6,
            "rtk_label": "RTK_FIXED",
            "latitude_deg": 37.0,
            "longitude_deg": 127.0,
            "altitude_m": 100.0,
            "satellites": 20,
            "age_s": 0.1,
        },
        "right_position": {
            "state": "LIVE",
            "fix_type": 6,
            "rtk_label": "RTK_FIXED",
            "latitude_deg": 37.0,
            "longitude_deg": 127.00001,
            "altitude_m": 100.0,
            "satellites": 20,
            "age_s": 0.1,
        },
        "position": {"state": "LIVE", "rtk_label": "RTK_FLOAT", "age_s": 0.2},
        "rtcm": {"frames": 4, "bytes": 800, "packets": 8, "fail": 0, "last_delivery_age_s": 0.1},
        "focal_length_px": 900.0,
        "focal_source": "test",
        "rtcm_source": "ntrip",
        "rtcm_source_label": "Network NTRIP",
        "ideal_disparity": {
            "state": "READY",
            "reason": "three GNSS positions + active zoom focal length; rectified parallel-camera model",
            "baseline_m": 1.0,
            "timestamp_delta_ms": 2.0,
            "target_distance_m": 10.0,
            "disparity_px": 90.0,
        },
        "target_distance_source": "P1/P2 baseline midpoint → Drone GNSS",
    }

    class Backend:
        def drone_telemetry_snapshot(self):
            return snapshot

        def workstation_settings_snapshot(self):
            return {}

    workspace = TelemetryWorkspace(Backend(), lambda _command_id: True)
    workspace.refresh({})

    assert workspace.position_model.rowCount() == 3
    assert workspace.position_model.item(2, 0).text() == "Drone Pixhawk"
    assert workspace.rtcm_values["frames"].text() == "4"
    assert workspace.rtcm_values["route"].text() == "enabled · Network NTRIP RTCM → Drone Pixhawk"
    assert workspace.disparity_values["disparity"].text() == "90.000 px"
    assert "P1/P2 baseline midpoint" in workspace.disparity_values["target"].text()
    assert not hasattr(workspace, "target_distance_spin")
    assert workspace.rtcm_source_combo.currentData() == "ntrip"
    assert workspace.payload()["drone_telemetry_rtcm_source"] == "ntrip"
    assert not workspace.operation_buttons["telemetry.connect"].isEnabled()
    assert workspace.operation_buttons["telemetry.disconnect"].isEnabled()

    workspace.deleteLater()
    app.processEvents()
