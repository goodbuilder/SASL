from __future__ import annotations

import math
import time
from types import SimpleNamespace

import pytest
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
    assert snapshot["gnss_baseline_center_frd"]["state"] == "CURRENT"
    assert snapshot["gnss_baseline_center_frd"]["range_m"] == snapshot["target_distance_m"]
    assert snapshot["gnss_baseline_center_frd"]["frame"] == (
        "gnss_baseline_center_local_level_frd"
    )
    assert snapshot["gnss_baseline_center_frd"]["frame"] != "baseline_center_frd"
    geometry = snapshot["gnss_baseline_center_frd"]
    expected_angle = math.degrees(
        math.atan2(abs(float(geometry["forward_m"])), abs(float(geometry["right_m"])))
    )
    assert geometry["baseline_drone_angle_deg"] == pytest.approx(expected_angle)
    assert geometry["baseline_drone_angle_error_deg"] == pytest.approx(abs(90.0 - expected_angle))
    assert snapshot["target_distance_source"] == "P1/P2 baseline midpoint → Drone GNSS"
    assert snapshot["ideal_disparity"]["disparity_px"] is not None


def test_drone_snapshot_shares_one_second_three_gnss_stale_decision() -> None:
    now_s = time.monotonic()
    panel = SimpleNamespace(
        mavlink_pixhawk=None,
        latest_gps={
            "left": _gps("left", longitude_deg=127.0, timestamp_s=now_s - 1.001),
            "right": _gps("right", longitude_deg=127.00001, timestamp_s=now_s - 0.5),
            "drone": _gps("drone", longitude_deg=127.00010, timestamp_s=now_s),
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

    geometry = snapshot["gnss_baseline_center_frd"]
    assert geometry["state"] == "STALE"
    assert geometry["forward_m"] is not None
    assert geometry["timestamp_spread_ms"] > 1000.0
    assert snapshot["ideal_disparity"]["state"] == "STALE"
    assert snapshot["ideal_disparity"]["disparity_px"] is None


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
        "gnss_baseline_center_frd": {
            "state": "CURRENT",
            "reason": "Three GNSS positions are current",
            "forward_m": 10.0,
            "right_m": -2.0,
            "down_m": 4.0,
            "range_m": 10.954451,
            "baseline_drone_angle_deg": math.degrees(math.atan2(10.0, 2.0)),
            "baseline_drone_angle_error_deg": abs(90.0 - math.degrees(math.atan2(10.0, 2.0))),
            "maximum_age_s": 0.2,
            "timestamp_spread_ms": 1000.0,
            "fix_labels": {
                "p1": "RTK_FIXED",
                "p2": "RTK_FIXED",
                "drone": "RTK_FLOAT",
            },
            "frame": "gnss_baseline_center_local_level_frd",
        },
    }

    class Backend:
        def drone_telemetry_snapshot(self):
            return snapshot

        def workstation_settings_snapshot(self):
            return {}

    workspace = TelemetryWorkspace(Backend(), lambda _command_id: True)
    workspace.resize(1200, 900)
    workspace.show()
    app.processEvents()
    workspace.refresh({})

    assert workspace.position_model.rowCount() == 3
    assert workspace.position_model.item(2, 0).text() == "Drone Pixhawk"
    assert workspace.rtcm_values["frames"].text() == "4"
    assert workspace.rtcm_values["route"].text() == "enabled · Network NTRIP RTCM → Drone Pixhawk"
    assert workspace.disparity_values["disparity"].text() == "90.000 px"
    assert not hasattr(workspace, "gnss_baseline_center_group")
    assert "P1/P2 baseline midpoint" in workspace.disparity_values["target"].text()
    assert not hasattr(workspace, "target_distance_spin")
    assert workspace.rtcm_source_combo.currentData() == "ntrip"
    assert workspace.payload()["drone_telemetry_rtcm_source"] == "ntrip"
    assert not workspace.operation_buttons["telemetry.connect"].isEnabled()
    assert workspace.operation_buttons["telemetry.disconnect"].isEnabled()

    workspace.deleteLater()
    app.processEvents()


def test_moved_midpoint_table_keeps_geometry_focus_and_scroll_stable() -> None:
    from Workstation.baseline_yaw90_pane import BaselineYaw90Pane

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    geometry = {"state": "CURRENT", "forward_m": 9.999, "right_m": -0.001,
                "down_m": 0.0, "range_m": 9.999, "baseline_drone_angle_deg": 89.99,
                "baseline_drone_angle_error_deg": 0.01, "timestamp_spread_ms": 999.9}
    class Backend:
        def baseline_yaw90_snapshot(self):
            return {"midpoint": geometry, "reverse_editable": True}
    scroll = QtWidgets.QScrollArea()
    scroll.setWidgetResizable(True)
    workspace = BaselineYaw90Pane(Backend())
    scroll.setWidget(workspace)
    scroll.resize(1000, 600)
    workspace.reference_toggle.setChecked(True)
    scroll.show()
    app.processEvents()
    workspace.refresh()
    workspace.reverse_checkbox.setFocus()
    workspace.reference_table.selectRow(2)
    scroll.verticalScrollBar().setValue(60)
    before = (workspace.reference_table.height(), scroll.verticalScrollBar().value())
    geometry.update(state="STALE", forward_m=-12345.678, reason="old GNSS " * 100)
    workspace.refresh()
    app.processEvents()
    assert workspace.reference_table.item(0, 1).text() == "-12345.678"
    assert workspace.reference_table.item(0, 2).text() == "STALE"
    assert workspace.reference_table.currentRow() == 2
    assert before == (workspace.reference_table.height(), scroll.verticalScrollBar().value())
    assert app.focusWidget() is workspace.reverse_checkbox
    geometry.clear()
    geometry["state"] = "WAITING"
    workspace.refresh()
    assert workspace.reference_table.item(0, 1).text() == "--"
    assert before == (workspace.reference_table.height(), scroll.verticalScrollBar().value())
    scroll.close()
    scroll.deleteLater()
    app.processEvents()


def test_telemetry_refresh_preserves_dirty_url_until_apply_accepts_it() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    snapshot = {
        "transport": "ethernet",
        "endpoint": "udpin:0.0.0.0:14552",
        "connection_state": "DISCONNECTED",
        "network_candidates": (),
    }

    class Backend:
        def drone_telemetry_snapshot(self):
            return snapshot

        def workstation_settings_snapshot(self):
            return {}

    workspace = TelemetryWorkspace(Backend(), lambda _command_id: True)
    workspace.endpoint_edit.setText("udpin:0.0.0.0:14551")
    workspace._endpoint_dirty = True
    workspace.refresh({})
    assert workspace.endpoint_edit.text() == "udpin:0.0.0.0:14551"

    workspace.accept_endpoint({"endpoint": "udpin:0.0.0.0:14551"})
    workspace.refresh({})
    assert workspace.endpoint_edit.text() == "udpin:0.0.0.0:14552"

    snapshot["endpoint"] = "udpin:0.0.0.0:14551"
    workspace.accept_endpoint(snapshot)
    workspace.refresh({})
    assert workspace.endpoint_edit.text() == "udpin:0.0.0.0:14551"


def test_telemetry_network_candidate_requires_selection_and_fills_url_dirty() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    snapshot = {
        "endpoint": "udpin:0.0.0.0:14552",
        "connection_state": "DISCONNECTED",
        "network_discovery_state": "FOUND 1 DRONE",
        "network_candidates": (
            {
                "candidate_id": "drone-3",
                "state": "DRONE VERIFIED",
                "interface": "eth0",
                "source_ip": "10.0.0.3",
                "listen_port": 14551,
                "system_id": 3,
                "heartbeat_count": 3,
                "selectable": True,
            },
        ),
    }

    class Backend:
        def drone_telemetry_snapshot(self):
            return snapshot

        def workstation_settings_snapshot(self):
            return {}

    workspace = TelemetryWorkspace(Backend(), lambda _command_id: True)
    workspace.refresh({})
    with pytest.raises(ValueError, match="Select one"):
        workspace.selected_network_candidate_id()
    workspace.network_candidate_table.selectRow(0)
    assert workspace.selected_network_candidate_id() == "drone-3"
    workspace.set_candidate_endpoint({"endpoint": "udpin:0.0.0.0:14551"})
    assert workspace.endpoint_edit.text() == "udpin:0.0.0.0:14551"
    assert workspace._endpoint_dirty


def test_telemetry_refresh_keeps_operator_transport_until_backend_accepts_it() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    snapshot = {
        "transport": "radio",
        "endpoint": "/dev/serial/by-id/drone-radio",
        "connection_state": "DISCONNECTED",
        "network_candidates": (),
    }

    class Backend:
        def drone_telemetry_snapshot(self):
            return snapshot

        def workstation_settings_snapshot(self):
            return {}

    workspace = TelemetryWorkspace(Backend(), lambda _command_id: True)
    workspace.refresh({})
    assert workspace.transport_combo.currentData() == "radio"
    assert workspace.serial_device_edit.text() == "/dev/serial/by-id/drone-radio"

    # The operator picks USB; the backend still reports radio until Configure/
    # Connect/Save pushes the payload, so a refresh must not snap it back.
    workspace.transport_combo.setCurrentIndex(workspace.transport_combo.findData("usb"))
    app.processEvents()
    workspace.serial_device_edit.setText("/dev/ttyACM2")
    workspace.refresh({})
    assert workspace.transport_combo.currentData() == "usb"
    assert workspace.serial_device_edit.isEnabled()
    assert not workspace.endpoint_edit.isEnabled()
    assert workspace.serial_device_edit.text() == "/dev/ttyACM2"
    assert workspace.payload()["drone_pixhawk_transport"] == "usb"

    # Once the backend accepted the change, refresh follows the backend again.
    snapshot["transport"] = "usb"
    snapshot["endpoint"] = "/dev/ttyACM2"
    workspace.accept_endpoint(snapshot)
    snapshot["transport"] = "ethernet"
    snapshot["endpoint"] = "udpin:0.0.0.0:14552"
    workspace.refresh({})
    assert workspace.transport_combo.currentData() == "ethernet"
    assert workspace.endpoint_edit.isEnabled()
    assert workspace.endpoint_edit.text() == "udpin:0.0.0.0:14552"


def test_telemetry_refresh_without_transport_key_defaults_to_radio() -> None:
    QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    snapshot = {
        "endpoint": "/dev/serial/by-id/drone-radio",
        "connection_state": "DISCONNECTED",
        "network_candidates": (),
    }

    class Backend:
        def drone_telemetry_snapshot(self):
            return snapshot

        def workstation_settings_snapshot(self):
            return {}

    workspace = TelemetryWorkspace(Backend(), lambda _command_id: True)
    workspace.refresh({})
    assert workspace.transport_combo.currentData() == "radio"
    assert workspace.serial_device_edit.isEnabled()
    assert workspace.serial_device_edit.text() == "/dev/serial/by-id/drone-radio"


def test_drone_telemetry_position_rows_show_frozen_gnss_state() -> None:
    from Workstation.drone_telemetry_controller import gnss_frozen_state_text

    panel = SimpleNamespace(
        mavlink_pixhawk=None,
        latest_gps={
            "left": _gps("left", longitude_deg=127.0),
            "right": _gps("right", longitude_deg=127.00001),
            "drone": _gps("drone", longitude_deg=127.00010),
        },
        _gnss_position_frozen={
            "drone": {
                "packets": 131,
                "utc_frozen": False,
                "fix_type": 4,
                "satellites": 31,
                "ground_speed_m_s": 0.0,
            }
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

    assert snapshot["position"]["state"] == "FROZEN ×131 (수신기 정지 · fix=4 sats=31 v=0.00m/s)"
    assert snapshot["position"]["frozen"]["packets"] == 131
    assert snapshot["left_position"]["state"] == "LIVE"
    assert snapshot["left_position"]["frozen"] is None
    assert gnss_frozen_state_text({"packets": 3, "utc_frozen": True}) == "FROZEN ×3 (AP 재전송)"
    assert gnss_frozen_state_text(None) is None
    assert gnss_frozen_state_text({"packets": 0}) is None

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    class Backend:
        def drone_telemetry_snapshot(self):
            return snapshot

        def workstation_settings_snapshot(self):
            return {}

    workspace = TelemetryWorkspace(Backend(), lambda _command_id: True)
    workspace.refresh({})
    assert workspace.position_model.item(2, 0).text() == "Drone Pixhawk"
    assert workspace.position_model.item(2, 1).text() == "FROZEN ×131 (수신기 정지 · fix=4 sats=31 v=0.00m/s)"
    assert workspace.position_model.item(0, 1).text() == "LIVE"
    workspace.deleteLater()
    app.processEvents()
