import time
from pathlib import Path

from wiris_stereo_distance.pixhawk_attitude import quaternion_to_euler_deg
from wiris_stereo_distance.stereo import (
    DetectionObservation,
    StereoCalibration,
    estimate_disparity_distance,
    estimate_distance,
    match_stereo_observations,
)


def detection(
    camera: str,
    *,
    x1: float,
    y1: float,
    x2: float,
    y2: float,
    t: float = 10.0,
    cls: int = 0,
    timestamp_metadata: dict | None = None,
):
    return DetectionObservation(
        camera=camera,
        timestamp_s=t,
        width=1280,
        height=720,
        x1=x1,
        y1=y1,
        x2=x2,
        y2=y2,
        confidence=0.9,
        class_id=cls,
        class_name=f"class {cls}",
        timestamp_metadata=dict(timestamp_metadata or {}),
    )


def process_events_until(app, predicate, *, timeout_s: float = 2.0) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        app.processEvents()
        if predicate():
            return True
        time.sleep(0.01)
    app.processEvents()
    return bool(predicate())


def test_estimate_distance_from_bbox_centers() -> None:
    left = detection("left", x1=500, y1=200, x2=540, y2=260)
    right = detection("right", x1=450, y1=202, x2=490, y2=262)

    estimate = estimate_distance(left, right, StereoCalibration(focal_length_px=900, baseline_m=0.3))

    assert estimate is not None
    assert estimate.disparity_px == 50
    assert estimate.distance_m == 5.4


def test_stereo_match_is_separate_from_disparity_distance_formula() -> None:
    left = detection("left", x1=500, y1=200, x2=540, y2=260)
    right = detection("right", x1=450, y1=202, x2=490, y2=262)
    calibration = StereoCalibration(focal_length_px=0.0, baseline_m=0.0)

    match = match_stereo_observations(left, right, calibration)

    assert match is not None
    assert match.disparity_px == 50
    assert estimate_disparity_distance(match, calibration) is None
    estimate = estimate_disparity_distance(match, StereoCalibration(focal_length_px=900, baseline_m=0.3))
    assert estimate is not None
    assert estimate.distance_m == 5.4


def test_rejects_stale_or_bad_matches() -> None:
    left = detection("left", x1=500, y1=200, x2=540, y2=260, t=10.0, cls=1)
    stale_right = detection("right", x1=450, y1=200, x2=490, y2=260, t=11.0, cls=1)
    wrong_class = detection("right", x1=450, y1=200, x2=490, y2=260, t=10.0, cls=2)
    high_vertical_error = detection("right", x1=450, y1=500, x2=490, y2=560, t=10.0, cls=1)
    calibration = StereoCalibration(focal_length_px=900, baseline_m=0.3, max_time_delta_s=0.25, max_vertical_delta_px=80)

    assert estimate_distance(left, stale_right, calibration) is None
    assert estimate_distance(left, wrong_class, calibration) is None
    assert estimate_distance(left, high_vertical_error, calibration) is None


def test_detection_observation_from_yolo_payload() -> None:
    payload = {
        "timestamp_s": 1.25,
        "width": 640,
        "height": 480,
        "best_detection": {
            "x1": 1,
            "y1": 2,
            "x2": 5,
            "y2": 10,
            "confidence": 0.95,
            "class_id": 3,
            "class_name": "person",
        },
        "timestamp_metadata": {"timestamp_reference": "rtcp-ntp", "rtcp_frame_ntp_unix_s": 1800000000.0},
    }

    observation = DetectionObservation.from_payload("left", payload)

    assert observation is not None
    assert observation.center == (3.0, 6.0)
    assert observation.class_id == 3
    assert observation.class_name == "person"
    assert observation.timestamp_metadata["timestamp_reference"] == "rtcp-ntp"


def test_camera_pane_detection_label_shows_center_with_two_decimals() -> None:
    from PyQt5 import QtCore, QtWidgets

    from wiris_stereo_distance.camera_pane import CameraPane
    from wiris_stereo_distance.stereo import DetectionObservation

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    pane = CameraPane("left")

    pane._set_detection_label(
        DetectionObservation(
            camera="left",
            timestamp_s=1.0,
            width=640,
            height=480,
            x1=1.0,
            y1=2.0,
            x2=5.0,
            y2=10.0,
            confidence=0.95,
            class_id=3,
            class_name="person",
        )
    )

    assert pane.detection_label.text() == "최고 bbox: class=person conf=0.95 center=(3.00, 6.00) px"

    pane.deleteLater()
    _ = app


def test_bbox_center_adjustment_csv_updates_yolo_overlay_and_parameters(tmp_path, monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    matrix_path = tmp_path / "Cent_Bbox_Adj_Mtrx.csv"
    matrix_path.write_text(
        "\n".join(
            (
                "0.9,0,0,10",
                "0,0.8,0,20",
                "0,0,1,0",
                "0.7,0,0,30",
                "0,0.6,0,40",
                "0,0,1,0",
            )
        ),
        encoding="utf-8",
    )

    widget._load_bbox_center_adjustment_path(matrix_path)
    widget.left_pane._set_detection_label(detection("left", x1=90, y1=80, x2=110, y2=120))
    widget._refresh_camera_parameters_tab()

    assert "corrected=(100.00, 100.00) px" in widget.left_pane.detection_label.text()
    assert "bbox_center_adjustment: active" in widget.stereo_geometry_text.toPlainText()
    assert "left: u'=0.900000u" in widget.stereo_geometry_text.toPlainText()
    assert "right: u'=0.700000u" in widget.stereo_geometry_text.toPlainText()

    widget.clear_bbox_center_adjustment()
    assert "2D bbox 보정: 없음" in widget.bbox_adjust_status_label.text()

    widget.deleteLater()
    _ = app


def test_range_only_reprojection_output_policy_preserves_raw_lateral() -> None:
    from types import SimpleNamespace

    import wiris_stereo_distance.gui as gui

    panel = SimpleNamespace(
        bbox_reprojection_profile=SimpleNamespace(output_policy="range_only_forward_fusion")
    )

    fused = gui.WirisStereoDistancePanel._apply_reprojection_output_policy(
        panel,
        raw_point_center_m=(3.0, -2.0, 800.0),
        corrected_point_center_m=(10.0, 5.0, 760.0),
    )

    assert fused == (3.0, -2.0, 760.0)


def test_identity_quaternion_is_zero_euler() -> None:
    attitude = quaternion_to_euler_deg([1, 0, 0, 0])

    assert attitude.roll_deg == 0
    assert attitude.pitch_deg == 0
    assert attitude.yaw_deg == 0


def test_mavlink_attitude_maps_to_camera_extrinsic() -> None:
    from math import radians

    from wiris_stereo_distance.pixhawk_attitude import camera_extrinsic_from_euler_rad

    extrinsic = camera_extrinsic_from_euler_rad(
        camera="left",
        namespace="udpin:0.0.0.0:14550",
        roll_rad=radians(10.0),
        pitch_rad=radians(-5.0),
        yaw_rad=radians(30.0),
        timestamp_s=12.0,
    )

    assert extrinsic.camera == "left"
    assert extrinsic.namespace == "udpin:0.0.0.0:14550"
    assert round(extrinsic.roll_deg, 3) == 10.0
    assert round(extrinsic.pitch_deg, 3) == -5.0
    assert round(extrinsic.yaw_deg, 3) == 30.0
    assert extrinsic.topic == "mavlink/ATTITUDE"


def test_mavlink_gps_raw_int_maps_to_gps_state() -> None:
    from types import SimpleNamespace

    from wiris_stereo_distance.pixhawk_attitude import DualPixhawkMavlinkSubscriber

    class FakeMavlinkMessage(SimpleNamespace):
        def get_type(self):
            return "GPS_RAW_INT"

    subscriber = DualPixhawkMavlinkSubscriber(left_url="udpin:0.0.0.0:14550", right_url="")
    captured = []
    subscriber.gps_changed.connect(lambda camera, gps: captured.append((camera, gps)))
    subscriber._handle_mavlink_message(
        "left",
        FakeMavlinkMessage(
            lat=370000000,
            lon=1270000000,
            alt=123450,
            fix_type=6,
            satellites_visible=18,
            eph=4,
            epv=5,
            cog=9012,
        ),
    )

    assert len(captured) == 1
    camera, gps = captured[0]
    assert camera == "left"
    assert gps.latitude_deg == 37.0
    assert gps.longitude_deg == 127.0
    assert gps.altitude_m == 123.45
    assert gps.fix_type == 6
    assert gps.satellites_used == 18
    assert gps.eph_m == 0.04
    assert gps.epv_m == 0.05
    assert gps.heading_deg == 90.12


def test_mavlink_start_px4_uxrce_client_shell_sends_px4_shell_command() -> None:
    from types import SimpleNamespace

    from wiris_stereo_distance.pixhawk_attitude import DualPixhawkMavlinkSubscriber

    sent = []

    class FakeMav:
        def serial_control_send(self, device, flags, timeout, baudrate, count, data):
            sent.append((device, flags, timeout, baudrate, count, bytes(data[:count]).decode("utf-8")))

    connection = SimpleNamespace(
        mav=FakeMav(),
        mavlink=SimpleNamespace(SERIAL_CONTROL_DEV_SHELL=10, SERIAL_CONTROL_FLAG_RESPOND=1, SERIAL_CONTROL_FLAG_EXCLUSIVE=4),
    )
    subscriber = DualPixhawkMavlinkSubscriber(left_url="udpin:0.0.0.0:14550", right_url="")
    subscriber.connections["left"] = connection

    subscriber.start_px4_uxrce_client_shell(host_ip="192.168.0.254", port=8888, namespaces={"left": "uav_0"})

    assert {item[0] for item in sent} == {10}
    assert {item[1] for item in sent} == {5}
    assert "".join(item[5] for item in sent) == (
        "uxrce_dds_client stop\n"
        "uxrce_dds_client start -t udp -p 8888 -h 192.168.0.254 -n uav_0\n"
    )


def test_mavlink_start_px4_uxrce_client_shell_retries_after_first_heartbeat() -> None:
    from types import SimpleNamespace

    from wiris_stereo_distance.pixhawk_attitude import DualPixhawkMavlinkSubscriber

    sent = []

    class FakeMav:
        def __init__(self) -> None:
            self.fail_next = True

        def serial_control_send(self, device, flags, timeout, baudrate, count, data):
            if self.fail_next:
                self.fail_next = False
                raise OSError("peer address is unknown")
            sent.append(bytes(data[:count]).decode("utf-8"))

    class FakeHeartbeat:
        def get_type(self):
            return "HEARTBEAT"

    connection = SimpleNamespace(
        mav=FakeMav(),
        mavlink=SimpleNamespace(SERIAL_CONTROL_DEV_SHELL=10, SERIAL_CONTROL_FLAG_RESPOND=1, SERIAL_CONTROL_FLAG_EXCLUSIVE=4),
        target_system=1,
        target_component=1,
    )
    subscriber = DualPixhawkMavlinkSubscriber(left_url="udpin:0.0.0.0:14550", right_url="")
    subscriber.connections["left"] = connection

    subscriber.start_px4_uxrce_client_shell(host_ip="10.41.10.1", port=8888, namespaces={"left": "uav_0"})
    assert sent == []

    subscriber._handle_mavlink_message("left", FakeHeartbeat())

    assert "".join(sent) == (
        "uxrce_dds_client stop\n"
        "uxrce_dds_client start -t udp -p 8888 -h 10.41.10.1 -n uav_0\n"
    )


def test_mavlink_param_int_encoding_preserves_unsigned_ip_values() -> None:
    from wiris_stereo_distance.pixhawk_attitude import _mavlink_int_to_param_float, _mavlink_param_float_to_int

    value = 3232235558

    assert _mavlink_param_float_to_int(_mavlink_int_to_param_float(value)) == value


def test_mavlink_send_shell_command_targets_selected_pixhawk() -> None:
    from types import SimpleNamespace

    from wiris_stereo_distance.pixhawk_attitude import DualPixhawkMavlinkSubscriber

    sent = []

    class FakeMav:
        def serial_control_send(self, device, flags, timeout, baudrate, count, data):
            sent.append(bytes(data[:count]).decode("utf-8"))

    subscriber = DualPixhawkMavlinkSubscriber(left_url="", right_url="")
    subscriber.connections["right"] = SimpleNamespace(
        mav=FakeMav(),
        mavlink=SimpleNamespace(SERIAL_CONTROL_DEV_SHELL=10, SERIAL_CONTROL_FLAG_RESPOND=1, SERIAL_CONTROL_FLAG_EXCLUSIVE=4),
    )

    subscriber.send_shell_command("right", "listener vehicle_attitude")

    assert sent == ["listener vehicle_attitude\n"]


def test_mavlink_reboot_autopilot_sends_preflight_reboot_command() -> None:
    from types import SimpleNamespace

    from wiris_stereo_distance.pixhawk_attitude import DualPixhawkMavlinkSubscriber

    sent = []

    class FakeMav:
        def command_long_send(self, *args):
            sent.append(args)

    subscriber = DualPixhawkMavlinkSubscriber(left_url="", right_url="")
    subscriber.connections["right"] = SimpleNamespace(
        mav=FakeMav(),
        mavlink=SimpleNamespace(MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN=246),
        target_system=2,
        target_component=1,
    )

    subscriber.reboot_autopilot("right")

    assert sent == [(2, 1, 246, 0, 1, 0, 0, 0, 0, 0, 0)]


def test_mavlink_configure_streams_sends_px4_shell_commands() -> None:
    from types import SimpleNamespace

    from wiris_stereo_distance.pixhawk_attitude import DualPixhawkMavlinkSubscriber

    sent = []

    class FakeMav:
        def serial_control_send(self, device, flags, timeout, baudrate, count, data):
            sent.append(bytes(data[:count]).decode("utf-8"))

    connection = SimpleNamespace(
        mav=FakeMav(),
        mavlink=SimpleNamespace(SERIAL_CONTROL_DEV_SHELL=10, SERIAL_CONTROL_FLAG_RESPOND=1, SERIAL_CONTROL_FLAG_EXCLUSIVE=4),
    )
    subscriber = DualPixhawkMavlinkSubscriber(left_url="udpin:0.0.0.0:14550", right_url="")
    subscriber.connections["left"] = connection

    subscriber.configure_mavlink_streams(
        host_ip="192.168.0.254",
        ports={"left": 14550},
        sysids={"left": 1},
        rate=2_000_000,
    )

    assert "".join(sent) == (
        "param set MAV_SYS_ID 1\n"
        "mavlink start -x -u 14550 -t 192.168.0.254 -o 14550 -r 2000000 -m normal\n"
        "mavlink status\n"
    )


def test_mavlink_subscriber_starts_and_stops_selected_side(monkeypatch) -> None:
    import sys
    from types import SimpleNamespace

    from PyQt5 import QtWidgets

    from wiris_stereo_distance.pixhawk_attitude import DualPixhawkMavlinkSubscriber

    class FakeConnection:
        def __init__(self, url: str, baud: int) -> None:
            self.url = url
            self.baud = baud
            self.closed = False

        def close(self) -> None:
            self.closed = True

    created = []

    def fake_mavlink_connection(url, *, baud, autoreconnect, robust_parsing):
        connection = FakeConnection(url, baud)
        created.append((connection, autoreconnect, robust_parsing))
        return connection

    monkeypatch.setitem(
        sys.modules,
        "pymavlink",
        SimpleNamespace(mavutil=SimpleNamespace(mavlink_connection=fake_mavlink_connection)),
    )
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    subscriber = DualPixhawkMavlinkSubscriber(
        left_url="/dev/ttyACM0",
        right_url="/dev/ttyACM1",
        left_baud=57600,
        right_baud=115200,
    )

    subscriber.start("left")

    assert set(subscriber.connections) == {"left"}
    assert subscriber.connections["left"].url == "/dev/ttyACM0"
    assert subscriber.connections["left"].baud == 57600

    subscriber.start("right")

    assert set(subscriber.connections) == {"left", "right"}
    assert subscriber.connections["right"].url == "/dev/ttyACM1"
    assert subscriber.connections["right"].baud == 115200

    left_connection = subscriber.connections["left"]
    right_connection = subscriber.connections["right"]
    subscriber.stop("left")

    assert left_connection.closed
    assert not right_connection.closed
    assert set(subscriber.connections) == {"right"}

    subscriber.stop()

    assert right_connection.closed
    assert subscriber.connections == {}
    _ = app



def test_mavlink_serial_url_uses_default_stream_port() -> None:
    from PyQt5 import QtCore, QtWidgets

    from wiris_stereo_distance.gui import WirisStereoDistancePanel

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = WirisStereoDistancePanel()

    assert widget._mavlink_stream_port("/dev/ttyACM1", default=14551) == 14551
    assert widget._mavlink_stream_port("udpin:0.0.0.0:14550", default=14551) == 14550

    widget.deleteLater()
    _ = app


def test_main_windows_use_project_window_title(monkeypatch) -> None:
    from PyQt5 import QtCore, QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    windows = [
        gui.WirisStereoDistanceGui(),
        gui.IntegratedWirisPixhawkConsole(),
    ]

    for window in windows:
        assert window.windowTitle() == gui.APP_WINDOW_TITLE == "영상감시시스템 개발"
        window.close()
        window.deleteLater()
    _ = app


def test_initial_status_waits_for_manual_hardware_checks(monkeypatch) -> None:
    from PyQt5 import QtCore, QtWidgets

    import wiris_stereo_distance.gui as gui

    checked = []
    started = []
    monkeypatch.setattr(gui, "interface_addresses", lambda _interface: "eth-test 192.168.0.254")
    monkeypatch.setattr(gui.WirisStereoDistancePanel, "check_cameras", lambda self: checked.append(True))
    monkeypatch.setattr(gui.WirisStereoDistancePanel, "start_yolo", lambda self: started.append(True))

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel(auto_start=False)
    widget.wiris_log.clear()
    widget.camera_yolo_log.clear()
    widget.pixhawk_log.clear()

    widget._initial_status()

    assert checked == []
    assert started == []
    assert widget.camera_connection_label.text() == gui.STARTUP_CAMERA_STATUS
    assert widget.pixhawk_connection_label.text() == gui.STARTUP_PIXHAWK_STATUS
    assert "eth-test 192.168.0.254" in widget.wiris_log.toPlainText()
    assert gui.STARTUP_READY_MESSAGE in widget.wiris_log.toPlainText()
    assert gui.STARTUP_READY_MESSAGE not in widget.pixhawk_log.toPlainText()
    assert gui.STARTUP_READY_MESSAGE not in widget.camera_yolo_log.toPlainText()

    widget.deleteLater()
    _ = app


def test_wiris_pixhawk_and_yolo_logs_are_routed_separately() -> None:
    from PyQt5 import QtWidgets

    from wiris_stereo_distance.gui import WirisStereoDistancePanel

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = WirisStereoDistancePanel()
    widget.wiris_log.clear()
    widget.pixhawk_log.clear()
    widget.camera_yolo_log.clear()

    widget._log("[카메라] WIRIS RTSP frame-ok")
    widget._log("[픽스호크] MAVLink 세션을 시작했습니다.")
    widget._log("[YOLO] weights 파일 선택: custom.pt")

    assert "WIRIS RTSP frame-ok" in widget.wiris_log.toPlainText()
    assert "WIRIS RTSP frame-ok" not in widget.pixhawk_log.toPlainText()
    assert "MAVLink 세션" in widget.pixhawk_log.toPlainText()
    assert "MAVLink 세션" not in widget.wiris_log.toPlainText()
    assert "weights 파일 선택" in widget.camera_yolo_log.toPlainText()
    assert "weights 파일 선택" not in widget.wiris_log.toPlainText()
    assert "weights 파일 선택" not in widget.pixhawk_log.toPlainText()

    widget.deleteLater()
    _ = app


def test_manual_pixhawk_ros2_candidate_applies_namespace(monkeypatch) -> None:
    from types import SimpleNamespace

    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()

    widget._apply_pixhawk_discovery(SimpleNamespace(domain_id=7, namespaces=("uav_0", "uav_1"), topics=()))
    widget.pixhawk_candidate_table.selectRow(1)
    widget.apply_selected_pixhawk_candidate("left")

    assert widget.domain_spin.value() == 7
    assert widget.left_namespace_edit.text() == "uav_1"

    widget.deleteLater()
    _ = app


def test_usb_pixhawk_discovery_lists_candidates_without_assignment(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    monkeypatch.setattr(gui, "usb_mavlink_device_candidates", lambda: ["/dev/ttyACM0"])
    monkeypatch.setattr(gui, "probe_usb_mavlink_devices", lambda candidates, timeout_s=0.8: [("/dev/ttyACM0", 57600)])
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    left_before = widget.left_mavlink_url_edit.text()
    right_before = widget.right_mavlink_url_edit.text()

    widget.discover_usb_pixhawk_connections()

    assert widget.left_mavlink_url_edit.text() == left_before
    assert widget.right_mavlink_url_edit.text() == right_before
    assert widget.pixhawk_candidate_table.rowCount() == 1
    assert widget.pixhawk_candidate_table.item(0, 0).text() == "USB"
    assert widget.pixhawk_candidate_table.item(0, 1).text() == "/dev/ttyACM0"
    assert widget.pixhawk_candidate_table.item(0, 2).text() == "57600"
    assert widget.pixhawk_candidate_table.item(0, 3).text() == "MAVLink heartbeat"
    widget.pixhawk_candidate_table.selectRow(0)
    assert not widget.apply_pixhawk_candidate_host_button.isEnabled()
    assert widget.apply_pixhawk_candidate_left_button.isEnabled()
    assert widget.apply_pixhawk_candidate_right_button.isEnabled()
    assert "MAVLink URL에 적용" in widget.apply_pixhawk_candidate_left_button.toolTip()
    assert widget.clear_pixhawk_candidate_button.isEnabled()

    widget.clear_pixhawk_candidate_button.click()

    assert widget.pixhawk_candidate_table.rowCount() == 0
    assert not widget.clear_pixhawk_candidate_button.isEnabled()
    assert not widget.apply_pixhawk_candidate_host_button.isEnabled()
    assert not widget.apply_pixhawk_candidate_left_button.isEnabled()
    assert not widget.apply_pixhawk_candidate_right_button.isEnabled()

    widget.deleteLater()
    _ = app


def test_mavlink_url_candidates_apply_without_connecting(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget.left_mavlink_url_edit.setText("/dev/ttyACM0")

    widget.discover_mavlink_url_candidates()

    assert widget.pixhawk_candidate_table.rowCount() == 2
    assert widget.pixhawk_candidate_table.item(0, 0).text() == "MAVLink"
    assert widget.pixhawk_candidate_table.item(0, 1).text() == "udpin:0.0.0.0:14550"
    assert widget.pixhawk_candidate_table.item(0, 3).text() == "Ethernet UDP listen"
    widget.pixhawk_candidate_table.selectRow(0)
    assert widget.apply_pixhawk_candidate_left_button.isEnabled()
    assert "MAVLink URL 후보" in widget.apply_pixhawk_candidate_left_button.toolTip()

    widget.apply_selected_pixhawk_candidate("left")

    assert widget.left_mavlink_url_edit.text() == "udpin:0.0.0.0:14550"
    assert widget.mavlink_pixhawk is None
    assert "Pixhawk 1 MAVLink 연결을 누르세요" in widget.pixhawk_connection_label.text()

    widget.deleteLater()
    _ = app


def test_ros2_pixhawk_discovery_includes_mavlink_url_candidates(monkeypatch) -> None:
    from types import SimpleNamespace

    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()

    widget._apply_pixhawk_discovery(SimpleNamespace(domain_id=7, namespaces=("uav_0",), topics=()))

    kinds = [widget.pixhawk_candidate_table.item(row, 0).text() for row in range(widget.pixhawk_candidate_table.rowCount())]
    values = [widget.pixhawk_candidate_table.item(row, 1).text() for row in range(widget.pixhawk_candidate_table.rowCount())]
    assert kinds == ["ROS2", "MAVLink", "MAVLink"]
    assert values == ["uav_0", "udpin:0.0.0.0:14550", "udpin:0.0.0.0:14551"]

    widget.deleteLater()
    _ = app


def test_usb_pixhawk_discovery_reports_permission_denied(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    monkeypatch.setattr(gui, "usb_mavlink_device_candidates", lambda: ["/dev/ttyACM0"])
    monkeypatch.setattr(gui, "probe_usb_mavlink_devices", lambda candidates, timeout_s=0.8: [])
    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_usb_pixhawk_probe_status", lambda self, device: "permission denied: add user to dialout")
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()

    widget.discover_usb_pixhawk_connections()

    assert widget.pixhawk_candidate_table.item(0, 3).text() == "permission denied: add user to dialout"

    widget.deleteLater()
    _ = app


def test_ros2_pixhawk_candidate_enables_apply_buttons(monkeypatch) -> None:
    from types import SimpleNamespace

    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()

    widget._apply_pixhawk_discovery(SimpleNamespace(domain_id=7, namespaces=("uav_0",), topics=()))
    widget.pixhawk_candidate_table.selectRow(0)

    assert not widget.apply_pixhawk_candidate_host_button.isEnabled()
    assert widget.apply_pixhawk_candidate_left_button.isEnabled()
    assert widget.apply_pixhawk_candidate_right_button.isEnabled()

    widget.deleteLater()
    _ = app


def test_usb_pixhawk_candidate_applies_selected_side_without_connecting(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    monkeypatch.setattr(gui, "usb_mavlink_device_candidates", lambda: ["/dev/ttyACM0"])
    monkeypatch.setattr(gui, "probe_usb_mavlink_devices", lambda candidates, timeout_s=0.8: [("/dev/ttyACM0", 921600)])
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()

    widget.discover_usb_pixhawk_connections()
    widget.pixhawk_candidate_table.selectRow(0)
    widget.apply_selected_pixhawk_candidate("right")

    assert widget.right_mavlink_url_edit.text() == "/dev/ttyACM0"
    assert widget.right_mavlink_baud_spin.value() == 921600
    assert widget.mavlink_pixhawk is None
    assert "Pixhawk 2 MAVLink 연결을 누르세요" in widget.pixhawk_connection_label.text()

    widget.deleteLater()
    _ = app


def test_usb_pixhawk_candidate_disconnects_active_side_before_manual_reconnect(monkeypatch) -> None:
    from types import SimpleNamespace

    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    stopped = []
    logs = []
    widget._log = logs.append
    widget.mavlink_pixhawk = SimpleNamespace(
        connections={"right": object()},
        stop=lambda camera=None: stopped.append(camera),
    )
    widget._set_pixhawk_candidate_rows(
        [{"kind": "USB", "value": "/dev/ttyACM0", "baud": 57600, "status": "MAVLink heartbeat"}]
    )

    widget.pixhawk_candidate_table.selectRow(0)
    widget.apply_selected_pixhawk_candidate("right")

    assert stopped == ["right"]
    assert widget.right_mavlink_url_edit.text() == "/dev/ttyACM0"
    assert any("다시 연결하세요" in message for message in logs)

    widget.deleteLater()
    _ = app


def test_persistent_pixhawk_eth_command_uses_side_ip_and_host(monkeypatch) -> None:
    from ipaddress import ip_interface
    from types import SimpleNamespace

    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    monkeypatch.setattr(
        gui,
        "local_ipv4_interfaces",
        lambda **_kwargs: [SimpleNamespace(name="pixhawk0", address=ip_interface("192.168.0.254/24"))],
    )
    widget = gui.WirisStereoDistancePanel()
    widget.pixhawk_host_address_edit.setText("192.168.0.254")
    widget.right_pixhawk_eth_ip_edit.setText("192.168.0.3")
    widget.right_mavlink_url_edit.setText("/dev/ttyACM0")

    command = widget._persistent_pixhawk_eth_command("right")
    extras_text = widget._persistent_pixhawk_extras_text("right")
    net_cfg_text = widget._persistent_pixhawk_net_cfg_text("right")

    assert "ifconfig eth0 192.168.0.3 netmask 255.255.255.0 up" in command
    assert "param set MAV_2_CONFIG 0" in command
    assert "param set UXRCE_DDS_CFG 1000" in command
    assert "param set UXRCE_DDS_KEY 2" in command
    assert "param set UXRCE_DDS_DOM_ID 0" in command
    assert "param set UXRCE_DDS_AG_IP 3232235774" in command
    assert "param set UXRCE_DDS_PTCFG 0" in command
    assert "param set UXRCE_DDS_SYNCC 0" in command
    assert "param set UXRCE_DDS_SYNCT 1" in command
    assert "uxrce_dds_client stop" in command
    assert "uxrce_dds_client start -t udp -p 8888 -h 192.168.0.254 -n uav_1" in command
    assert "mavlink start -x" not in command
    assert "echo 'ifup eth0' > /fs/microsd/etc/extras.txt" in command
    assert "echo 'ifconfig eth0 192.168.0.3 netmask 255.255.255.0 up' >> /fs/microsd/etc/extras.txt" in command
    assert "echo 'uxrce_dds_client start -t udp -p 8888 -h 192.168.0.254 -n uav_1' >> /fs/microsd/etc/extras.txt" in command
    assert "cat /fs/microsd/etc/extras.txt" in command
    assert "ifconfig eth0 192.168.0.3 netmask 255.255.255.0 up" in extras_text
    assert "uxrce_dds_client start -t udp -p 8888 -h 192.168.0.254 -n uav_1" in extras_text
    assert "mavlink start -x" not in extras_text
    assert "echo '" not in extras_text
    assert "IPADDR=192.168.0.3" in net_cfg_text
    assert "ROUTER=192.168.0.254" in net_cfg_text
    assert "BOOTPROTO=static" in net_cfg_text

    widget.deleteLater()
    _ = app


def test_persistent_pixhawk_eth_uses_mavlink_ftp_for_extras(monkeypatch) -> None:
    from ipaddress import ip_interface
    from types import SimpleNamespace

    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    monkeypatch.setattr(
        gui,
        "local_ipv4_interfaces",
        lambda **_kwargs: [SimpleNamespace(name="pixhawk0", address=ip_interface("10.0.0.10/24"))],
    )
    widget = gui.WirisStereoDistancePanel()
    widget.pixhawk_host_address_edit.setText("10.0.0.10")
    widget.left_pixhawk_eth_ip_edit.setText("10.0.0.2")
    widget.left_mavlink_url_edit.setText("/dev/ttyACM0")
    sent_shell = []
    ftp_writes = []
    widget.mavlink_pixhawk = SimpleNamespace(
        connections={"left": object()},
        send_shell_command=lambda camera, command: sent_shell.append((camera, command)),
        write_remote_file_mavftp=lambda camera, path, content: ftp_writes.append((camera, path, content)),
    )

    widget.apply_persistent_pixhawk_eth("left")

    assert sent_shell and sent_shell[0][0] == "left"
    assert ftp_writes == [
        ("left", "/fs/microsd/etc/extras.txt", widget._persistent_pixhawk_extras_text("left")),
        ("left", "/fs/microsd/net.cfg", widget._persistent_pixhawk_net_cfg_text("left")),
    ]
    assert "ifconfig eth0 10.0.0.2 netmask 255.255.255.0 up" in ftp_writes[0][2]
    assert "IPADDR=10.0.0.2" in ftp_writes[1][2]
    assert "ROUTER=10.0.0.10" in ftp_writes[1][2]

    widget.deleteLater()
    _ = app


def test_pixhawk_host_ip_auto_corrects_to_local_interface_for_pixhawk_subnet(monkeypatch) -> None:
    from ipaddress import ip_interface
    from types import SimpleNamespace

    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    monkeypatch.setattr(
        gui,
        "local_ipv4_interfaces",
        lambda **_kwargs: [
            SimpleNamespace(name="enp46s0", address=ip_interface("10.0.0.10/24")),
            SimpleNamespace(name="wlo1", address=ip_interface("192.168.0.38/24")),
        ],
    )
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget.pixhawk_host_address_edit.setText("10.41.10.1")
    widget.left_pixhawk_eth_ip_edit.setText("192.168.0.2")
    widget.right_pixhawk_eth_ip_edit.setText("192.168.0.3")

    host_ip = widget._pixhawk_host_ip_for_uxrce()

    assert host_ip == "192.168.0.38"
    assert widget.pixhawk_host_address_edit.text() == "192.168.0.38"

    widget.deleteLater()
    _ = app


def test_pixhawk_host_ip_keeps_existing_local_address(monkeypatch) -> None:
    from ipaddress import ip_interface
    from types import SimpleNamespace

    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    monkeypatch.setattr(
        gui,
        "local_ipv4_interfaces",
        lambda **_kwargs: [
            SimpleNamespace(name="enp46s0", address=ip_interface("10.0.0.10/24")),
            SimpleNamespace(name="wlo1", address=ip_interface("192.168.0.38/24")),
        ],
    )
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget.pixhawk_host_address_edit.setText("10.0.0.10")
    widget.left_pixhawk_eth_ip_edit.setText("192.168.0.2")

    host_ip = widget._pixhawk_host_ip_for_uxrce()

    assert host_ip == "10.0.0.10"
    assert widget.pixhawk_host_address_edit.text() == "10.0.0.10"

    widget.deleteLater()
    _ = app


def test_discover_pc_pixhawk_ip_button_fills_host_from_pixhawk_subnet(monkeypatch) -> None:
    from ipaddress import ip_interface
    from types import SimpleNamespace

    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    monkeypatch.setattr(
        gui,
        "local_ipv4_interfaces",
        lambda **_kwargs: [
            SimpleNamespace(name="enp46s0", address=ip_interface("10.0.0.10/24")),
            SimpleNamespace(name="wlo1", address=ip_interface("192.168.0.38/24")),
        ],
    )
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget._save_current_settings = lambda: None
    widget.pixhawk_host_address_edit.setText("10.41.10.1")
    widget.left_pixhawk_eth_ip_edit.setText("192.168.0.2")
    widget.right_pixhawk_eth_ip_edit.setText("192.168.0.3")

    widget.detect_pixhawk_host_ip_button.click()

    assert widget.pixhawk_host_address_edit.text() == "10.41.10.1"
    assert "후보 표시됨" in widget.pixhawk_connection_label.text()
    assert widget.pixhawk_candidate_table.rowCount() == 2
    assert widget.pixhawk_candidate_table.item(0, 0).text() == "PC_IP"
    assert widget.pixhawk_candidate_table.item(0, 1).text() == "10.0.0.10"
    assert widget.pixhawk_candidate_table.item(0, 4).text() == "enp46s0"
    assert widget.pixhawk_candidate_table.item(1, 1).text() == "192.168.0.38"
    assert "Wi-Fi" in widget.pixhawk_candidate_table.item(1, 3).text()

    widget.pixhawk_candidate_table.selectRow(0)
    assert widget.apply_pixhawk_candidate_host_button.isEnabled()
    assert not widget.apply_pixhawk_candidate_left_button.isEnabled()
    assert not widget.apply_pixhawk_candidate_right_button.isEnabled()
    widget.apply_selected_pixhawk_candidate("host")

    assert widget.pixhawk_host_address_edit.text() == "10.0.0.10"

    widget.deleteLater()
    _ = app


def test_pixhawk_host_ip_prefers_active_mavlink_remote_subnet(monkeypatch) -> None:
    from ipaddress import ip_interface
    from types import SimpleNamespace

    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    monkeypatch.setattr(
        gui,
        "local_ipv4_interfaces",
        lambda **_kwargs: [
            SimpleNamespace(name="enp46s0", address=ip_interface("10.0.0.10/24")),
            SimpleNamespace(name="wlo1", address=ip_interface("192.168.0.38/24")),
        ],
    )
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget.pixhawk_host_address_edit.setText("10.0.0.10")
    widget.left_pixhawk_eth_ip_edit.setText("10.0.0.2")
    widget.right_pixhawk_eth_ip_edit.setText("10.0.0.3")

    host_ip = widget._pixhawk_host_ip_for_uxrce(extra_hints=("192.168.0.3",))

    assert host_ip == "192.168.0.38"
    assert widget.pixhawk_host_address_edit.text() == "192.168.0.38"

    widget.deleteLater()
    _ = app


def test_pixhawk_common_setup_has_pc_ip_discovery_button(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()

    assert widget.detect_pixhawk_host_ip_button.text() == "PC IP 탐색"
    assert widget.detect_pixhawk_host_ip_button.parent() is widget.pixhawk_common_setup_group

    widget.deleteLater()
    _ = app



def test_extract_field_path_supports_nested_and_indexed_values() -> None:
    from types import SimpleNamespace

    from wiris_stereo_distance.pixhawk_attitude import extract_field_path

    message = SimpleNamespace(position=[1.0, 2.0, 3.0], nested=SimpleNamespace(value=-4.5))

    assert extract_field_path(message, "position[2]") == 3.0
    assert extract_field_path(message, "nested.value") == -4.5


def test_custom_watch_update_sets_label_and_plot() -> None:
    from PyQt5 import QtWidgets

    from wiris_stereo_distance.gui import WirisStereoDistancePanel
    from wiris_stereo_distance.pixhawk_attitude import CustomTopicSample

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = WirisStereoDistancePanel()

    widget._update_custom_watch(
        CustomTopicSample(
            key="local_z",
            topic="/uav_0/fmu/out/vehicle_local_position",
            field_path="z",
            value_text="-1.25",
            numeric_value=-1.25,
            timestamp_s=0.0,
        )
    )

    assert widget.custom_watch_plot.points[-1].series == "local_z"
    assert widget.custom_watch_plot.points[-1].value == -1.25

    widget.deleteLater()
    _ = app


def test_remove_custom_topic_watch_removes_spec_combo_item_and_plot_series() -> None:
    from PyQt5 import QtWidgets

    from wiris_stereo_distance.gui import TelemetryHistoryPoint, WirisStereoDistancePanel

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = WirisStereoDistancePanel()
    widget._add_custom_topic_watch(key="local_z", topic="/uav_0/fmu/out/vehicle_local_position", field="z")
    widget.custom_watch_plot.add_point(TelemetryHistoryPoint(0.0, "local_z", -1.0))
    widget.custom_watch_plot.add_point(TelemetryHistoryPoint(1.0, "left:local_z", -2.0))

    widget.remove_custom_topic_watch("local_z")

    assert widget.custom_watch_specs == []
    assert widget.custom_watch_remove_combo.findData("local_z") < 0
    assert list(widget.custom_watch_plot.points) == []

    widget.deleteLater()
    _ = app


def test_custom_watch_controls_keep_plot_area_compact() -> None:
    from PyQt5 import QtWidgets

    from wiris_stereo_distance.gui import WirisStereoDistancePanel

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = WirisStereoDistancePanel()

    assert widget.custom_watch_remove_combo.minimumWidth() == 260
    assert widget.custom_watch_plot.minimumHeight() == 360

    widget.deleteLater()
    _ = app


def test_form_labels_and_buttons_use_consistent_alignment() -> None:
    from PyQt5 import QtCore, QtWidgets

    from wiris_stereo_distance.gui import FORM_BUTTON_MIN_WIDTH, FORM_LABEL_MIN_WIDTH, WirisStereoDistancePanel

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = WirisStereoDistancePanel()

    regular_label_texts = {"픽스호크 Host IP", "왼쪽 사진", "명령"}
    labels = {
        label.text(): label
        for label in widget.findChildren(QtWidgets.QLabel)
        if label.text() in regular_label_texts
    }
    assert set(labels) == regular_label_texts
    for label in labels.values():
        assert label.minimumWidth() >= FORM_LABEL_MIN_WIDTH
        assert label.alignment() & QtCore.Qt.AlignRight

    compact_path_labels = [
        label
        for label in widget.findChildren(QtWidgets.QLabel)
        if label.text() == "저장 경로" and label.property("compactFormLabel")
    ]
    assert len(compact_path_labels) == 1
    assert compact_path_labels[0].minimumWidth() < FORM_LABEL_MIN_WIDTH
    assert compact_path_labels[0].alignment() & QtCore.Qt.AlignRight

    assert widget.clock_left_file_button.minimumWidth() >= FORM_BUTTON_MIN_WIDTH
    assert widget.browse_weights_button.minimumWidth() >= FORM_BUTTON_MIN_WIDTH
    assert widget.clock_left_file_button.text() == "파일 선택"
    assert widget.clock_left_folder_button.text() == "폴더 선택"
    assert widget.clock_download_dir_button.text() == "폴더 선택"
    assert widget.browse_weights_button.text() == "파일 선택"

    widget.deleteLater()
    _ = app


def test_clock_tab_exposes_cptr_ftp_workflow_controls(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()

    assert widget.clock_workflow_button.text() == "전체 실행"
    assert widget.clock_cptr_button.text() == "CPTR 촬영"
    assert widget.clock_icpt_button.text() == "ICPT 확인"
    assert widget.clock_download_latest_button.text() == "FTP 다운로드"
    assert widget.clock_ftp_user_edit.text() == "anonymous"
    assert widget.clock_ftp_port_spin.value() == 21
    assert widget.clock_ftp_remote_dir_edit.text() == "/"
    assert Path(widget.clock_download_dir_edit.text()) == gui.APP_ROOT / "clock_captures"

    widget.deleteLater()
    _ = app


def test_default_calibration_and_clock_paths_are_app_root_relative(monkeypatch, tmp_path) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    saved = []
    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    monkeypatch.setattr(gui, "load_settings", lambda: gui.AppSettings())
    monkeypatch.setattr(gui, "save_settings", lambda settings: saved.append(settings))
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()

    assert Path(widget.calib_dir_edit.text()) == gui.APP_ROOT / "calibration_captures"
    assert Path(widget.clock_download_dir_edit.text()) == gui.APP_ROOT / "clock_captures"

    widget._save_current_settings()

    assert saved[-1].intrinsics_path == "calibration_captures/intrinsics.json"
    assert saved[-1].clock_download_dir == "clock_captures"

    widget.deleteLater()
    _ = app


def test_clock_cptr_command_sends_to_both_wiris(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    sent = []

    def fake_send(command):
        sent.append((command.host, command.port, command.command))
        return "OK"

    monkeypatch.setattr(gui, "send_tcp_line", fake_send)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget.left_zoom_host_edit.setText("10.0.0.11")
    widget.right_zoom_host_edit.setText("10.0.0.12")
    widget.left_control_port_spin.setValue(2240)
    widget.right_control_port_spin.setValue(2241)

    widget.capture_clock_photos()

    assert sorted(sent) == [
        ("10.0.0.11", 2240, "CPTR"),
        ("10.0.0.12", 2241, "CPTR"),
    ]
    assert "CPTR 전송" in widget.clock_status_label.text()

    widget.deleteLater()
    _ = app


def test_clock_ftp_download_latest_pair_populates_photo_inputs(monkeypatch, tmp_path) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    class FakeFtp:
        def __init__(self):
            self.host = ""

        def __enter__(self):
            return self

        def __exit__(self, exc_type, exc, tb):
            return False

        def connect(self, host, port, timeout=None):
            self.host = host
            self.port = port
            self.timeout = timeout

        def login(self, user="", passwd="", acct=""):
            self.user = user
            self.passwd = passwd

        def cwd(self, dirname):
            self.dirname = dirname

        def nlst(self):
            return ["old.jpg", "latest.jpg", "notes.txt"]

        def voidcmd(self, command):
            if "latest.jpg" in command:
                return "213 20260524010203"
            return "213 20260524000102"

        def retrbinary(self, command, callback):
            callback(f"{self.host}:{command}".encode("utf-8"))

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    monkeypatch.setattr(gui.ftplib, "FTP", FakeFtp)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget.left_zoom_host_edit.setText("10.0.0.11")
    widget.right_zoom_host_edit.setText("10.0.0.12")
    widget.clock_download_dir_edit.setText(str(tmp_path))
    widget.clock_ftp_remote_dir_edit.setText("/media")

    paths = widget._download_latest_clock_photo_pair()
    widget._set_clock_downloaded_paths(paths)

    assert paths["left"].name == "latest.jpg"
    assert paths["right"].name == "latest.jpg"
    assert paths["left"].read_bytes() == b"10.0.0.11:RETR latest.jpg"
    assert paths["right"].read_bytes() == b"10.0.0.12:RETR latest.jpg"
    assert widget.clock_left_file_edit.text() == str(paths["left"])
    assert widget.clock_right_file_edit.text() == str(paths["right"])

    widget.deleteLater()
    _ = app


def test_camera_network_settings_are_single_compact_row() -> None:
    from PyQt5 import QtCore, QtWidgets

    from wiris_stereo_distance.gui import NETWORK_FIELD_MIN_WIDTHS, WirisStereoDistancePanel

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = WirisStereoDistancePanel()
    layout = widget.camera_network_form

    expected = (
        ("인터페이스", widget.interface_edit, "interface", 0, 1),
        ("PC IP", widget.host_address_edit, "host", 2, 3),
        ("스캔 기본 포트", widget.zoom_port_spin, "port", 4, 5),
        ("타임아웃", widget.zoom_timeout_spin, "timeout", 6, 7),
    )
    assert layout.parentWidget().title() == "카메라망 / NIC"
    for label_text, field, key, label_column, field_column in expected:
        label = layout.itemAtPosition(0, label_column).widget()
        assert label.text() == label_text
        assert label.property("compactNetworkLabel") is True
        assert label.alignment() & QtCore.Qt.AlignRight
        assert layout.itemAtPosition(0, field_column).widget() is field
        assert field.property("compactNetworkField") is True
        assert field.minimumWidth() == NETWORK_FIELD_MIN_WIDTHS[key]

    network_actions = layout.itemAtPosition(1, 0).layout()
    assert network_actions.itemAt(0).widget() is widget.detect_pc_ip_button
    assert network_actions.itemAt(1).widget() is widget.apply_pc_ip_button
    assert widget.stip_mask_edit.parentWidget().title() == "IP Address / IP Mask 변경"
    assert widget.stip_mask_edit.property("compactNetworkField") is True
    assert widget.stip_mask_edit.minimumWidth() == NETWORK_FIELD_MIN_WIDTHS["mask"]
    assert widget.stip_reboot_checkbox.parentWidget().title() == "IP Address / IP Mask 변경"
    assert widget.stip_reboot_checkbox.text() == "STIP 후 재부팅"
    assert widget.stip_reboot_checkbox.isCheckable()
    assert widget.stip_reboot_checkbox.sizePolicy().horizontalPolicy() == QtWidgets.QSizePolicy.Expanding

    widget.deleteLater()
    _ = app


def test_telnet_port_spins_ignore_mouse_wheel(monkeypatch) -> None:
    from types import SimpleNamespace

    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()

    for spin in (widget.zoom_port_spin, widget.left_control_port_spin, widget.right_control_port_spin):
        spin.setValue(2240)
        event = SimpleNamespace(ignored=False, ignore=lambda: setattr(event, "ignored", True))
        spin.wheelEvent(event)
        assert event.ignored is True
        assert spin.value() == 2240

    widget.deleteLater()
    _ = app


def test_visible_buttons_do_not_overlap_at_compact_width(monkeypatch) -> None:
    from PyQt5 import QtCore, QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget.resize(900, 820)
    widget.show()
    app.processEvents()

    for tab_index in range(widget.tabs.count()):
        widget.tabs.setCurrentIndex(tab_index)
        app.processEvents()
        visible_buttons = [
            button
            for button in widget.findChildren(QtWidgets.QPushButton)
            if button.isVisibleTo(widget)
        ]
        rects = []
        for button in visible_buttons:
            top_left = widget.mapFromGlobal(button.mapToGlobal(QtCore.QPoint(0, 0)))
            rects.append((button, QtCore.QRect(top_left, button.size())))
        for index, (left_button, left_rect) in enumerate(rects):
            for right_button, right_rect in rects[index + 1 :]:
                if left_rect.intersected(right_rect).isEmpty():
                    continue
                assert False, (
                    f"{widget.tabs.tabText(tab_index)} buttons overlap: "
                    f"{left_button.text()} {left_rect.getRect()} / "
                    f"{right_button.text()} {right_rect.getRect()}"
                )

    widget.tabs.setCurrentIndex(widget.depth_tab_index)
    app.processEvents()
    assert widget.runtime_header_layout.direction() == QtWidgets.QBoxLayout.TopToBottom

    widget.close()
    widget.deleteLater()
    _ = app


def test_action_button_rows_and_runtime_header_orientations(monkeypatch) -> None:
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")

    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget.resize(1190, 820)
    widget.show()
    app.processEvents()

    network_buttons = [widget.detect_pc_ip_button, widget.apply_pc_ip_button]
    network_widths = [button.geometry().width() for button in network_buttons]
    assert max(network_widths) - min(network_widths) <= 1
    verification_buttons = [
        widget.check_control_button,
        widget.enable_rtsp_button,
        widget.check_button,
        widget.camera_connect_button,
    ]
    verification_widths = [button.geometry().width() for button in verification_buttons]
    assert max(verification_widths) - min(verification_widths) <= 1
    auto_buttons = [widget.auto_wiris_connect_button, widget.swap_camera_button]
    auto_widths = [button.geometry().width() for button in auto_buttons]
    assert max(auto_widths) - min(auto_widths) <= 1
    assert widget.left_stip_button.parentWidget().title() == "IP Address / IP Mask 변경"
    assert widget.right_stip_button.parentWidget().title() == "IP Address / IP Mask 변경"
    assert widget.left_stip_button.text() == "STIP 적용"
    assert widget.right_stip_button.text() == "STIP 적용"
    assert widget.stip_mask_edit.parentWidget().title() == "IP Address / IP Mask 변경"
    assert widget.wiris_pair_settings_group.title() == "좌/우 WIRIS 설정"
    wiris_pair_form = widget.wiris_pair_settings_group.layout()
    assert wiris_pair_form.itemAtPosition(1, 1).widget() is widget.left_zoom_host_edit
    assert wiris_pair_form.itemAtPosition(1, 2).widget() is widget.right_zoom_host_edit
    assert wiris_pair_form.itemAtPosition(2, 1).widget() is widget.left_control_port_spin
    assert wiris_pair_form.itemAtPosition(2, 2).widget() is widget.right_control_port_spin
    assert widget.detect_control_button.parentWidget().title() == "좌/우 WIRIS 설정"
    assert widget.right_detect_control_button.parentWidget().title() == "좌/우 WIRIS 설정"
    assert widget.left_control_apply_button.parentWidget().title() == "좌/우 WIRIS 설정"
    assert widget.right_control_apply_button.parentWidget().title() == "좌/우 WIRIS 설정"
    assert widget.left_control_apply_button.text() == "Telnet 적용"
    assert widget.right_control_apply_button.text() == "Telnet 적용"
    assert abs(widget.detect_control_button.geometry().top() - widget.left_control_apply_button.geometry().top()) <= 1
    assert abs(widget.right_detect_control_button.geometry().top() - widget.right_control_apply_button.geometry().top()) <= 1
    assert wiris_pair_form.itemAtPosition(4, 1).widget() is widget.left_url_edit
    assert wiris_pair_form.itemAtPosition(4, 2).widget() is widget.right_url_edit
    assert wiris_pair_form.itemAtPosition(5, 1).widget() is widget.left_rtsp_ip_edit
    assert wiris_pair_form.itemAtPosition(5, 2).widget() is widget.right_rtsp_ip_edit
    assert widget.left_rtsp_apply_button.parentWidget().title() == "좌/우 WIRIS 설정"
    assert widget.right_rtsp_apply_button.parentWidget().title() == "좌/우 WIRIS 설정"
    assert widget.detect_rtsp_button.parentWidget().title() == "좌/우 WIRIS 설정"
    assert widget.right_detect_rtsp_button.parentWidget().title() == "좌/우 WIRIS 설정"
    assert widget.left_rtsp_apply_button.text() == "RTSP 적용"
    assert widget.right_rtsp_apply_button.text() == "RTSP 적용"
    assert widget.left_control_apply_button.property("role") == widget.left_rtsp_apply_button.property("role")
    assert widget.right_control_apply_button.property("role") == widget.right_rtsp_apply_button.property("role")
    assert abs(widget.detect_rtsp_button.geometry().top() - widget.left_rtsp_apply_button.geometry().top()) <= 1
    assert abs(widget.right_detect_rtsp_button.geometry().top() - widget.right_rtsp_apply_button.geometry().top()) <= 1
    assert widget.left_rtsp_ip_edit.parentWidget().title() == "좌/우 WIRIS 설정"
    assert widget.right_rtsp_ip_edit.parentWidget().title() == "좌/우 WIRIS 설정"
    assert widget.runtime_camera_status_group.title() == "카메라 상태"
    assert widget.runtime_camera_status_group.layout().count() == 2
    assert widget.runtime_camera_settings_group.layout().count() == 2
    assert widget.clear_history_button.text() == "그래프/기록 초기화"
    assert widget.left_runtime_stats_label.parentWidget().title() == "카메라 상태"
    assert widget.right_runtime_stats_label.parentWidget().title() == "카메라 상태"

    connection_tab_index = [widget.tabs.tabText(index) for index in range(widget.tabs.count())].index("연결")
    widget.tabs.setCurrentIndex(connection_tab_index)
    widget.setup_tabs.setCurrentIndex(widget.pixhawk_connection_tab_index)
    app.processEvents()
    pixhawk_buttons = [
        widget.swap_pixhawk_button,
        widget.left_mavlink_button,
        widget.right_mavlink_button,
        widget.pixhawk_button,
    ]
    pixhawk_widths = [button.geometry().width() for button in pixhawk_buttons]
    pixhawk_tops = [button.geometry().top() for button in pixhawk_buttons]
    assert max(pixhawk_widths) - min(pixhawk_widths) <= 1
    assert max(pixhawk_tops) - min(pixhawk_tops) <= 1
    assert widget.apply_candidate_left_button.text() == "Left에 적용"
    assert widget.apply_candidate_right_button.text() == "Right에 적용"
    assert widget.apply_pixhawk_candidate_host_button.text() == "Host에 적용"
    assert widget.apply_pixhawk_candidate_left_button.text() == "P1에 적용"
    assert widget.apply_pixhawk_candidate_right_button.text() == "P2에 적용"
    assert widget.clear_pixhawk_candidate_button.text() == "Clear"
    assert widget.pixhawk_auto_detect_button.parentWidget().title() == "1 ROS2 탐색 후보"
    assert widget.pixhawk_usb_discover_button.parentWidget().title() == "1 ROS2 탐색 후보"
    assert widget.pixhawk_usb_discover_button.text() == "USB 연결 탐색"
    assert not hasattr(widget, "usb_setup_target_combo")
    pixhawk_candidate_group = widget.findChild(QtWidgets.QGroupBox, "pixhawkCandidateGroup")
    assert pixhawk_candidate_group is not None
    assert pixhawk_candidate_group.height() >= pixhawk_candidate_group.sizeHint().height()
    pixhawk_candidate_header = widget.pixhawk_candidate_table.horizontalHeader()
    assert pixhawk_candidate_header.sectionResizeMode(0) == QtWidgets.QHeaderView.Stretch
    pixhawk_candidate_widths = [
        widget.pixhawk_candidate_table.columnWidth(column)
        for column in range(widget.pixhawk_candidate_table.columnCount())
    ]
    assert len(pixhawk_candidate_widths) == 5
    assert max(pixhawk_candidate_widths) - min(pixhawk_candidate_widths) <= 1
    pixhawk_candidate_rows = [
        (
            min(
                widget.pixhawk_auto_detect_button.geometry().top(),
                widget.pixhawk_usb_discover_button.geometry().top(),
            ),
            max(
                widget.pixhawk_auto_detect_button.geometry().bottom(),
                widget.pixhawk_usb_discover_button.geometry().bottom(),
            ),
        ),
        (
            widget.pixhawk_candidate_table.geometry().top(),
            widget.pixhawk_candidate_table.geometry().bottom(),
        ),
        (
            min(
                widget.clear_pixhawk_candidate_button.geometry().top(),
                widget.apply_pixhawk_candidate_host_button.geometry().top(),
                widget.apply_pixhawk_candidate_left_button.geometry().top(),
                widget.apply_pixhawk_candidate_right_button.geometry().top(),
            ),
            max(
                widget.clear_pixhawk_candidate_button.geometry().bottom(),
                widget.apply_pixhawk_candidate_host_button.geometry().bottom(),
                widget.apply_pixhawk_candidate_left_button.geometry().bottom(),
                widget.apply_pixhawk_candidate_right_button.geometry().bottom(),
            ),
        ),
    ]
    for previous_row, current_row in zip(pixhawk_candidate_rows, pixhawk_candidate_rows[1:]):
        assert previous_row[1] < current_row[0]
    assert widget.clear_pixhawk_candidate_button.geometry().left() < widget.apply_pixhawk_candidate_host_button.geometry().left()
    assert widget.pixhawk_common_setup_group.title() == "0 공통 준비"
    assert widget.pixhawk_pair_settings_group.title() == "2 Pixhawk 1/2 사전 설정"
    assert widget.pixhawk_connection_actions_group.title() == "3 연결 실행"
    assert widget.pixhawk_advanced_diagnostics_group.title() == "4 고급 / 진단"
    left_pixhawk_group = widget.findChild(QtWidgets.QGroupBox, "leftPixhawkWorkflowGroup")
    right_pixhawk_group = widget.findChild(QtWidgets.QGroupBox, "rightPixhawkWorkflowGroup")
    assert left_pixhawk_group is not None
    assert right_pixhawk_group is not None
    assert left_pixhawk_group.title() == "Pixhawk 1"
    assert right_pixhawk_group.title() == "Pixhawk 2"
    assert abs(left_pixhawk_group.geometry().width() - right_pixhawk_group.geometry().width()) <= 1
    left_pixhawk_form = left_pixhawk_group.layout()
    right_pixhawk_form = right_pixhawk_group.layout()
    assert left_pixhawk_form.itemAtPosition(0, 1).widget() is widget.left_mavlink_url_edit
    assert right_pixhawk_form.itemAtPosition(0, 1).widget() is widget.right_mavlink_url_edit
    assert left_pixhawk_form.itemAtPosition(1, 1).widget() is widget.left_mavlink_baud_spin
    assert right_pixhawk_form.itemAtPosition(1, 1).widget() is widget.right_mavlink_baud_spin
    assert left_pixhawk_form.itemAtPosition(2, 1).widget() is widget.left_pixhawk_eth_ip_edit
    assert right_pixhawk_form.itemAtPosition(2, 1).widget() is widget.right_pixhawk_eth_ip_edit
    assert left_pixhawk_form.itemAtPosition(3, 1).widget() is widget.left_namespace_edit
    assert right_pixhawk_form.itemAtPosition(3, 1).widget() is widget.right_namespace_edit
    assert widget.left_px4_shell_button.parentWidget().title() == "4 고급 / 진단"
    assert widget.right_px4_shell_button.parentWidget().title() == "4 고급 / 진단"
    assert widget.left_persist_eth_button.parentWidget().title() == "4 고급 / 진단"
    assert widget.right_persist_eth_button.parentWidget().title() == "4 고급 / 진단"
    assert widget.left_reboot_pixhawk_button.parentWidget().title() == "4 고급 / 진단"
    assert widget.right_reboot_pixhawk_button.parentWidget().title() == "4 고급 / 진단"
    assert widget.left_mavlink_button.parentWidget().title() == "3 연결 실행"
    assert widget.right_mavlink_button.parentWidget().title() == "3 연결 실행"
    assert widget.left_uxrce_param_button.parentWidget().title() == "3 연결 실행"
    assert widget.right_uxrce_param_button.parentWidget().title() == "3 연결 실행"
    assert abs(widget.left_px4_shell_button.geometry().top() - widget.left_persist_eth_button.geometry().top()) <= 1
    assert abs(widget.right_px4_shell_button.geometry().top() - widget.right_persist_eth_button.geometry().top()) <= 1
    assert abs(widget.left_reboot_pixhawk_button.geometry().top() - widget.right_reboot_pixhawk_button.geometry().top()) <= 1
    assert (
        widget.pixhawk_pair_settings_group.height()
        >= widget.pixhawk_pair_settings_group.sizeHint().height()
    )

    def pixhawk_side_row_bounds(side_form: QtWidgets.QGridLayout, row: int) -> tuple[int, int]:
        row_widgets = []
        for column in range(2):
            item = side_form.itemAtPosition(row, column)
            if item is None:
                continue
            widget_item = item.widget()
            if widget_item is not None:
                row_widgets.append(widget_item)
                continue
            layout_item = item.layout()
            if layout_item is None:
                continue
            for index in range(layout_item.count()):
                child = layout_item.itemAt(index).widget()
                if child is not None:
                    row_widgets.append(child)
        assert row_widgets
        return (
            min(row_widget.geometry().top() for row_widget in row_widgets),
            max(row_widget.geometry().bottom() for row_widget in row_widgets),
        )

    for side_form in (left_pixhawk_form, right_pixhawk_form):
        pixhawk_side_row_bounds_by_row = [pixhawk_side_row_bounds(side_form, row) for row in range(4)]
        for previous_row, current_row in zip(
            pixhawk_side_row_bounds_by_row,
            pixhawk_side_row_bounds_by_row[1:],
        ):
            assert previous_row[1] < current_row[0]

    widget.tabs.setCurrentIndex(widget.depth_tab_index)
    app.processEvents()
    assert widget.runtime_header_layout.direction() == QtWidgets.QBoxLayout.LeftToRight

    stat_widths = [widget.left_runtime_stats_label.geometry().width(), widget.right_runtime_stats_label.geometry().width()]
    assert max(stat_widths) - min(stat_widths) <= 1

    widget.close()
    widget.deleteLater()
    _ = app


def test_session_buttons_toggle_labels_and_roles(monkeypatch) -> None:
    from types import SimpleNamespace

    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()

    assert not hasattr(widget, "stop_button")
    assert not hasattr(widget, "camera_disconnect_button")
    assert not hasattr(widget, "pixhawk_disconnect_button")
    assert not hasattr(widget, "mavlink_disconnect_button")
    assert not hasattr(widget, "mavlink_button")

    assert widget.start_button.text() == "YOLO 측정 시작"
    assert widget.start_button.property("role") == "primary"
    widget.yolo_enabled = True
    widget.camera_connected = True
    widget._refresh_session_toggle_buttons()
    assert widget.start_button.text() == "YOLO 측정 중지"
    assert widget.start_button.property("role") == "danger"
    assert widget.camera_connect_button.text() == "카메라 Preview 해제"
    assert widget.camera_connect_button.property("role") == "danger"

    widget.pixhawk = object()
    widget.mavlink_pixhawk = SimpleNamespace(connections={"left": object()})
    widget._refresh_session_toggle_buttons()
    assert widget.pixhawk_button.text() == "ROS2 해제"
    assert widget.left_mavlink_button.text() == "P1 MAVLink 해제"
    assert widget.right_mavlink_button.text() == "P2 MAVLink 연결"

    widget.deleteLater()
    _ = app


def test_camera_pane_stop_requests_worker_without_blocking_thread_wait() -> None:
    from PyQt5 import QtCore, QtWidgets

    from wiris_stereo_distance.camera_pane import CameraPane

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    pane = CameraPane("left")
    calls = []

    class FakeWorker:
        def request_stop(self) -> None:
            calls.append("request_stop")

        def stop(self) -> None:
            raise AssertionError("blocking stop must not be called")

    class FakeThread:
        def wait(self, _timeout=None):
            raise AssertionError("thread wait must not be called for normal stop")

        def quit(self) -> None:
            raise AssertionError("thread quit must not be called for normal stop")

    pane.worker = FakeWorker()
    pane.thread = FakeThread()

    started_s = time.monotonic()
    pane.stop()
    elapsed_s = time.monotonic() - started_s

    assert elapsed_s < 0.05
    assert calls == ["request_stop"]
    assert pane.worker is None
    assert pane.thread is None

    pane.deleteLater()
    _ = app


def test_mavlink_connection_routes_sensor_signals_to_alignment(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    class FakeSignal:
        def __init__(self) -> None:
            self.slots = []

        def connect(self, slot) -> None:
            self.slots.append(slot)

        def is_connected_to(self, method) -> bool:
            return any(
                getattr(slot, "__self__", None) is getattr(method, "__self__", None)
                and getattr(slot, "__func__", None) is getattr(method, "__func__", None)
                for slot in self.slots
            )

    class FakeMavlinkSubscriber:
        latest_extrinsics = {}
        latest_gps = {}

        def __init__(self, *args, **kwargs) -> None:
            self.extrinsic_changed = FakeSignal()
            self.gps_changed = FakeSignal()
            self.mag_sensor_changed = FakeSignal()
            self.gps_sensor_changed = FakeSignal()
            self.gnss_relative_changed = FakeSignal()
            self.custom_watch_changed = FakeSignal()
            self.shell_output = FakeSignal()
            self.status_changed = FakeSignal()
            self.urls = {"left": kwargs.get("left_url", ""), "right": kwargs.get("right_url", "")}
            self.baudrates = {"left": kwargs.get("left_baud", 57600), "right": kwargs.get("right_baud", 57600)}
            self.connections = {"left": object(), "right": object()}
            self.configure_calls = []
            self.uxrce_client_shell_calls = []
            self.stopped = False

        def start(self, camera=None) -> None:
            pass

        def configure_mavlink_streams(self, **kwargs) -> None:
            self.configure_calls.append(kwargs)

        def start_px4_uxrce_client_shell(self, **kwargs) -> None:
            self.uxrce_client_shell_calls.append(kwargs)

        def stop(self, camera=None) -> None:
            self.stopped = True

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    monkeypatch.setattr(gui, "DualPixhawkMavlinkSubscriber", FakeMavlinkSubscriber)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget.mavlink_configure_streams_checkbox.setChecked(False)
    widget.mavlink_configure_uxrce_agent_checkbox.setChecked(False)

    widget.connect_mavlink_pixhawk()

    subscriber = widget.mavlink_pixhawk
    assert subscriber is not None
    assert subscriber.mag_sensor_changed.is_connected_to(widget._update_mag_sensor)
    assert subscriber.gps_sensor_changed.is_connected_to(widget._update_gps_sensor)
    assert subscriber.gnss_relative_changed.is_connected_to(widget._update_gnss_relative)

    widget.deleteLater()
    _ = app


def test_mavlink_connection_requests_and_logs_actual_eth0_ip(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    class FakeSignal:
        def __init__(self) -> None:
            self.slots = []

        def connect(self, slot) -> None:
            self.slots.append(slot)

    class FakeMavlinkSubscriber:
        def __init__(self, *args, **kwargs) -> None:
            self.extrinsic_changed = FakeSignal()
            self.gps_changed = FakeSignal()
            self.mag_sensor_changed = FakeSignal()
            self.gps_sensor_changed = FakeSignal()
            self.gnss_relative_changed = FakeSignal()
            self.custom_watch_changed = FakeSignal()
            self.shell_output = FakeSignal()
            self.status_changed = FakeSignal()
            self.urls = {"left": kwargs.get("left_url", ""), "right": kwargs.get("right_url", "")}
            self.baudrates = {"left": kwargs.get("left_baud", 57600), "right": kwargs.get("right_baud", 57600)}
            self.connections = {}
            self.shell_commands = []

        def start(self, camera=None) -> None:
            cameras = ("left", "right") if camera is None else (camera,)
            for side in cameras:
                if self.urls.get(side):
                    self.connections[side] = object()

        def send_shell_command(self, camera, command) -> None:
            self.shell_commands.append((camera, command))

        def stop(self, camera=None) -> None:
            self.connections.clear()

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    monkeypatch.setattr(gui, "DualPixhawkMavlinkSubscriber", FakeMavlinkSubscriber)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    logs = []
    widget._log = logs.append
    widget.mavlink_configure_streams_checkbox.setChecked(False)
    widget.mavlink_configure_uxrce_agent_checkbox.setChecked(False)
    widget.left_mavlink_url_edit.setText("/dev/ttyACM0")
    widget.right_mavlink_url_edit.setText("")
    widget.left_pixhawk_eth_ip_edit.setText("10.0.0.2")

    widget.connect_mavlink_pixhawk("left")

    assert widget.mavlink_pixhawk.shell_commands == [("left", "ifconfig eth0")]

    widget._append_px4_shell_output("left", "eth0 Link encap:Ethernet\n inet addr:10.0.0.2\n")

    assert any("실제 eth0 IP 확인: 10.0.0.2" in message for message in logs)

    widget.deleteLater()
    _ = app


def test_manual_mavlink_connection_skips_blocking_setup(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    class FakeSignal:
        def connect(self, _slot) -> None:
            pass

    class FakeMavlinkSubscriber:
        def __init__(self, *args, **kwargs) -> None:
            self.extrinsic_changed = FakeSignal()
            self.gps_changed = FakeSignal()
            self.mag_sensor_changed = FakeSignal()
            self.gps_sensor_changed = FakeSignal()
            self.gnss_relative_changed = FakeSignal()
            self.custom_watch_changed = FakeSignal()
            self.shell_output = FakeSignal()
            self.status_changed = FakeSignal()
            self.urls = {"left": kwargs.get("left_url", ""), "right": kwargs.get("right_url", "")}
            self.baudrates = {"left": kwargs.get("left_baud", 57600), "right": kwargs.get("right_baud", 57600)}
            self.connections = {}
            self.configure_calls = []
            self.param_sets = []

        def start(self, camera=None) -> None:
            cameras = ("left", "right") if camera is None else (camera,)
            for side in cameras:
                if self.urls.get(side):
                    self.connections[side] = object()

        def configure_mavlink_streams(self, **kwargs) -> None:
            self.configure_calls.append(kwargs)

        def set_param_int(self, camera, name, value):
            self.param_sets.append((camera, name, value))
            return value

        def send_shell_command(self, _camera, _command) -> None:
            pass

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    monkeypatch.setattr(gui, "DualPixhawkMavlinkSubscriber", FakeMavlinkSubscriber)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    logs = []
    widget._log = logs.append
    widget.mavlink_configure_streams_checkbox.setChecked(True)
    widget.mavlink_configure_uxrce_agent_checkbox.setChecked(True)
    widget.left_mavlink_url_edit.setText("udpin:0.0.0.0:14550")
    widget.right_mavlink_url_edit.setText("")

    widget.connect_mavlink_pixhawk("left")

    assert widget.mavlink_pixhawk is not None
    assert widget.mavlink_pixhawk.configure_calls == []
    assert widget.mavlink_pixhawk.param_sets == []
    assert any("일반 MAVLink 연결에서는 UI 응답성" in message for message in logs)

    widget.deleteLater()
    _ = app


def test_ros2_session_starts_mavlink_gps2_auxiliary_when_sensor_gps_missing(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    class FakeSignal:
        def connect(self, _slot) -> None:
            pass

    class FakeMavlinkSubscriber:
        def __init__(self, *args, **kwargs) -> None:
            self.extrinsic_changed = FakeSignal()
            self.gps_changed = FakeSignal()
            self.mag_sensor_changed = FakeSignal()
            self.gps_sensor_changed = FakeSignal()
            self.gnss_relative_changed = FakeSignal()
            self.custom_watch_changed = FakeSignal()
            self.shell_output = FakeSignal()
            self.status_changed = FakeSignal()
            self.urls = {"left": kwargs.get("left_url", ""), "right": kwargs.get("right_url", "")}
            self.baudrates = {"left": kwargs.get("left_baud", 57600), "right": kwargs.get("right_baud", 57600)}
            self.connections = {}
            self.started = []

        def start(self, camera=None) -> None:
            cameras = ("left", "right") if camera is None else (camera,)
            for side in cameras:
                if self.urls.get(side):
                    self.connections[side] = object()
                    self.started.append(side)

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    monkeypatch.setattr(gui, "DualPixhawkMavlinkSubscriber", FakeMavlinkSubscriber)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    logs = []
    widget._log = logs.append
    widget.pixhawk = object()
    widget.left_mavlink_url_edit.setText("udpin:0.0.0.0:14550")
    widget.right_mavlink_url_edit.setText("")
    widget.latest_gps_sensors["left"] = {}

    widget._start_mavlink_sensor_auxiliary_if_needed()

    assert widget.mavlink_pixhawk is not None
    assert widget.mavlink_pixhawk.started == ["left"]
    assert "MAVLink 센서 보조" in widget.pixhawk_connection_label.text()
    assert any("GPS2/RM3100 보조 수신" in message for message in logs)

    widget.deleteLater()
    _ = app


def test_ros2_session_starts_mavlink_auxiliary_when_magnetometer_missing(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui
    from wiris_stereo_distance.pixhawk_attitude import GpsSensorState

    class FakeSignal:
        def connect(self, _slot) -> None:
            pass

    class FakeMavlinkSubscriber:
        def __init__(self, *args, **kwargs) -> None:
            self.extrinsic_changed = FakeSignal()
            self.gps_changed = FakeSignal()
            self.mag_sensor_changed = FakeSignal()
            self.gps_sensor_changed = FakeSignal()
            self.gnss_relative_changed = FakeSignal()
            self.custom_watch_changed = FakeSignal()
            self.shell_output = FakeSignal()
            self.status_changed = FakeSignal()
            self.urls = {"left": kwargs.get("left_url", ""), "right": kwargs.get("right_url", "")}
            self.baudrates = {"left": kwargs.get("left_baud", 57600), "right": kwargs.get("right_baud", 57600)}
            self.connections = {}
            self.started = []

        def start(self, camera=None) -> None:
            cameras = ("left", "right") if camera is None else (camera,)
            for side in cameras:
                if self.urls.get(side):
                    self.connections[side] = object()
                    self.started.append(side)

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    monkeypatch.setattr(gui, "DualPixhawkMavlinkSubscriber", FakeMavlinkSubscriber)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget.pixhawk = object()
    widget.left_mavlink_url_edit.setText("udpin:0.0.0.0:14550")
    widget.right_mavlink_url_edit.setText("")
    widget.latest_gps_sensors["left"] = {
        "GPS2": GpsSensorState("left", "uav_0", "GPS2", 1.0, "/uav_0/fmu/out/sensor_gps")
    }
    widget.latest_mag_sensors["left"] = {}

    widget._start_mavlink_sensor_auxiliary_if_needed()

    assert widget.mavlink_pixhawk is not None
    assert widget.mavlink_pixhawk.started == ["left"]

    widget.deleteLater()
    _ = app


def test_mavlink_auxiliary_deduplicates_same_url_to_active_ros2_side(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui
    from wiris_stereo_distance.pixhawk_attitude import GpsSensorState

    class FakeSignal:
        def connect(self, _slot) -> None:
            pass

    class FakeMavlinkSubscriber:
        def __init__(self, *args, **kwargs) -> None:
            self.extrinsic_changed = FakeSignal()
            self.gps_changed = FakeSignal()
            self.mag_sensor_changed = FakeSignal()
            self.gps_sensor_changed = FakeSignal()
            self.gnss_relative_changed = FakeSignal()
            self.custom_watch_changed = FakeSignal()
            self.shell_output = FakeSignal()
            self.status_changed = FakeSignal()
            self.urls = {"left": kwargs.get("left_url", ""), "right": kwargs.get("right_url", "")}
            self.baudrates = {"left": kwargs.get("left_baud", 57600), "right": kwargs.get("right_baud", 57600)}
            self.connections = {}
            self.started = []

        def start(self, camera=None) -> None:
            cameras = ("left", "right") if camera is None else (camera,)
            for side in cameras:
                if self.urls.get(side):
                    self.connections[side] = object()
                    self.started.append(side)

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    monkeypatch.setattr(gui, "DualPixhawkMavlinkSubscriber", FakeMavlinkSubscriber)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget.pixhawk = object()
    widget.left_mavlink_url_edit.setText("/dev/ttyACM0")
    widget.right_mavlink_url_edit.setText("/dev/ttyACM0")
    widget.latest_gps_sensors["right"] = {
        "GPS1": GpsSensorState("right", "uav_1", "GPS1", 1.0, "/uav_1/fmu/out/vehicle_gps_position")
    }

    widget._start_mavlink_sensor_auxiliary_if_needed()

    assert widget.mavlink_pixhawk is not None
    assert widget.mavlink_pixhawk.started == ["right"]

    widget.deleteLater()
    _ = app


def test_ros2_only_alignment_marks_unpublished_gps2_and_mag_topics(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget.pixhawk = object()
    widget.ros2_sensor_topic_publishers = {
        "left": {
            "vehicle_gps_position": True,
            "sensor_gps": False,
            "sensor_mag": False,
            "vehicle_magnetometer": False,
        },
        "right": {
            "vehicle_gps_position": True,
            "sensor_gps": False,
            "sensor_mag": False,
            "vehicle_magnetometer": False,
        },
    }

    widget._refresh_ros2_only_alignment_availability()

    assert "ROS2 미게시" in widget.alignment_overview_labels["left_gps2"].text()
    assert "ROS2 미게시" in widget.alignment_overview_labels["left_can1"].text()
    assert "ROS2 sensor_gps(_0/_1) 미게시" in widget.f9p_slot_labels["left"]["GPS2"].text()
    assert "ROS2 미게시" in widget.alignment_mag_labels["left_CAN1"]["device"].text()

    widget.deleteLater()
    _ = app


def test_ros2_failure_starts_mavlink_recovery_and_fallback(monkeypatch) -> None:
    from ipaddress import ip_interface
    from types import SimpleNamespace

    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    class FakeSignal:
        def __init__(self) -> None:
            self.slots = []

        def connect(self, slot) -> None:
            self.slots.append(slot)

    class FailingRos2Subscriber:
        def __init__(self, *args, **kwargs) -> None:
            self.extrinsic_changed = FakeSignal()
            self.gps_changed = FakeSignal()
            self.mag_sensor_changed = FakeSignal()
            self.gps_sensor_changed = FakeSignal()
            self.gnss_relative_changed = FakeSignal()
            self.custom_watch_changed = FakeSignal()
            self.status_changed = FakeSignal()

        def start(self) -> None:
            raise RuntimeError("ROS2 topic sample timeout")

    class FakeMavlinkSubscriber:
        instances = []

        def __init__(self, *, left_url, right_url, left_baud=57600, right_baud=57600, custom_watches=()) -> None:
            self.urls = {"left": left_url, "right": right_url}
            self.baudrates = {"left": left_baud, "right": right_baud}
            self.connections = {}
            self.configure_calls = []
            self.param_sets = []
            self.extrinsic_changed = FakeSignal()
            self.gps_changed = FakeSignal()
            self.mag_sensor_changed = FakeSignal()
            self.gps_sensor_changed = FakeSignal()
            self.gnss_relative_changed = FakeSignal()
            self.custom_watch_changed = FakeSignal()
            self.shell_output = FakeSignal()
            self.status_changed = FakeSignal()
            self.instances.append(self)

        def start(self, camera=None) -> None:
            cameras = ("left", "right") if camera is None else (camera,)
            for side in cameras:
                if self.urls.get(side):
                    self.connections[side] = object()

        def configure_mavlink_streams(self, **kwargs) -> None:
            self.configure_calls.append(kwargs)

        def set_param_int(self, camera, name, value):
            self.param_sets.append((camera, name, value))
            return value

        def stop(self, camera=None) -> None:
            if camera is None:
                self.connections.clear()
            else:
                self.connections.pop(camera, None)

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_ensure_micro_xrce_agent", lambda self: None)
    monkeypatch.setattr(
        gui,
        "local_ipv4_interfaces",
        lambda **_kwargs: [SimpleNamespace(name="pixhawk0", address=ip_interface("10.41.10.1/24"))],
    )
    monkeypatch.setattr(gui, "DualPixhawkAttitudeSubscriber", FailingRos2Subscriber)
    monkeypatch.setattr(gui, "DualPixhawkMavlinkSubscriber", FakeMavlinkSubscriber)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget.pixhawk_host_address_edit.setText("10.41.10.1")
    widget.left_namespace_edit.setText("uav_0")
    widget.right_namespace_edit.setText("uav_1")

    connected = widget.connect_pixhawk()

    assert connected is False
    assert widget.pixhawk is None
    assert widget.mavlink_pixhawk is not None
    assert set(widget.mavlink_pixhawk.connections) == {"left", "right"}
    assert widget.mavlink_pixhawk.configure_calls == []
    assert ("left", "MAV_2_CONFIG", 0) in widget.mavlink_pixhawk.param_sets
    assert ("right", "MAV_2_CONFIG", 0) in widget.mavlink_pixhawk.param_sets
    assert ("left", "UXRCE_DDS_AG_IP", 170461697) in widget.mavlink_pixhawk.param_sets
    assert ("right", "UXRCE_DDS_AG_IP", 170461697) in widget.mavlink_pixhawk.param_sets
    assert ("left", "UXRCE_DDS_KEY", 1) in widget.mavlink_pixhawk.param_sets
    assert ("right", "UXRCE_DDS_KEY", 2) in widget.mavlink_pixhawk.param_sets
    assert ("left", "UXRCE_DDS_PTCFG", 0) in widget.mavlink_pixhawk.param_sets
    assert ("right", "UXRCE_DDS_SYNCT", 1) in widget.mavlink_pixhawk.param_sets
    assert widget._ros2_retry_after_uxrce_agent_setup is False
    assert "MAVLink" in widget.pixhawk_connection_label.text()

    widget.deleteLater()
    _ = app


def test_auto_start_pixhawk_selects_baseline_and_connects(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel(auto_start_pixhawk=True)
    calls = []
    monkeypatch.setattr(widget, "connect_mavlink_pixhawk", lambda: calls.append("connect"))

    widget.tabs.setCurrentIndex(widget.depth_tab_index)
    widget.alignment_tabs.setCurrentIndex(widget.baseline_alignment_tab_index)
    widget._auto_start_pixhawk()

    assert calls == ["connect"]
    assert widget.tabs.currentIndex() == widget.alignment_tab_index
    assert widget.alignment_tabs.currentIndex() == widget.baseline_alignment_tab_index

    widget.deleteLater()
    _ = app


def test_binary_options_use_checkable_toggle_buttons(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()

    toggles = [
        widget.use_rtk_baseline_checkbox,
        widget.require_rtk_fixed_checkbox,
        widget.manual_focal_checkbox,
        widget.same_class_checkbox,
        widget.mavlink_configure_streams_checkbox,
        widget.mavlink_configure_uxrce_agent_checkbox,
        widget.stip_reboot_checkbox,
    ]
    for button in toggles:
        assert isinstance(button, QtWidgets.QPushButton)
        assert button.isCheckable()
        assert button.property("role") == "toggle"

    assert isinstance(widget.distance_mode_combo, QtWidgets.QComboBox)
    assert widget.distance_mode_combo.findData("disparity") == -1
    assert widget.distance_mode_combo.findData("calibration") == -1
    assert widget.distance_mode_combo.itemText(widget.distance_mode_combo.findData("pixhawk")) == "Pixhawk 자세 + GPS/RTK baseline"
    assert not hasattr(widget, "require_attitude_checkbox")
    assert isinstance(widget.measurement_display_combo, QtWidgets.QComboBox)
    assert widget.measurement_display_combo.itemText(widget.measurement_display_combo.findData("detection")) == "계산 선택 프레임"
    assert widget.measurement_display_combo.itemText(widget.measurement_display_combo.findData("preview")) == "실시간 Preview"

    widget.deleteLater()
    _ = app


def test_manual_focal_tooltips_include_visible_zoom_fx_estimates(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()

    assert widget.focal_spin.maximum() >= 30906
    for tooltip in (
        widget.focal_spin.toolTip(),
        widget.manual_focal_checkbox.toolTip(),
        widget.zoom_level_spin.toolTip(),
    ):
        assert "index / ratio / estimated fx" in tooltip
        assert "0 /  1.0x / 1,030 px" in tooltip
        assert "14 / 30.0x / 30,906 px" in tooltip

    widget.deleteLater()
    _ = app


def test_yolo_device_dropdown_updates_worker_device(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    expected_input_size = widget.input_size_spin.value()
    expected_preview_width = widget.preview_width_spin.value()
    expected_preview_rate = widget.preview_rate_spin.value()

    assert isinstance(widget.device_combo, QtWidgets.QComboBox)
    assert widget.device_combo.currentText() == "CUDA"
    widget.device_combo.setCurrentIndex(widget.device_combo.findText("CPU"))
    assert widget.device_combo.currentText() == "CPU"
    assert widget.device_edit.text() == gui.YOLO_CPU_DEVICE
    assert widget.yolo_kwargs()["device"] == gui.YOLO_CPU_DEVICE
    assert widget.yolo_kwargs()["input_size"] == expected_input_size
    assert widget.yolo_kwargs()["display_rate_hz"] == expected_preview_rate
    assert widget.yolo_kwargs()["preview_max_width"] == expected_preview_width

    widget.device_combo.setCurrentIndex(widget.device_combo.findText("CUDA"))
    assert widget.device_combo.currentText() == "CUDA"
    assert widget.device_edit.text() == gui.YOLO_CUDA_DEVICE
    assert widget.yolo_kwargs()["device"] == gui.YOLO_CUDA_DEVICE

    widget.deleteLater()
    _ = app


def test_yolo_advanced_fields_have_visible_names(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()

    group = widget.findChild(QtWidgets.QGroupBox, "yoloAdvancedSettingsGroup")
    assert group is not None
    assert not group.isCheckable()
    labels = {label.text(): label for label in group.findChildren(QtWidgets.QLabel)}
    expected = {
        "입력 크기(px)": widget.input_size_spin,
        "신뢰도 임계값": widget.conf_spin,
        "IoU 임계값": widget.iou_spin,
        "미리보기 폭(px)": widget.preview_width_spin,
        "미리보기 Hz": widget.preview_rate_spin,
        "RTP 지연(ms)": widget.rtsp_latency_spin,
        "디코더": widget.decoder_combo,
        "수신 백엔드": widget.stream_backend_combo,
        "RTSP 전송": widget.rtsp_transport_combo,
    }
    assert expected.keys() <= labels.keys()
    for label_text, field in expected.items():
        assert labels[label_text].buddy() is field
        assert labels[label_text].property("compactAdvancedLabel") is True
        assert field.accessibleName() == label_text
    assert widget.findChild(QtWidgets.QPushButton, "saveYoloAdvancedSettingsButton") is None
    assert [widget.stream_backend_combo.itemText(index) for index in range(widget.stream_backend_combo.count())] == [
        "GStreamer",
        "OpenCV/FFmpeg",
        "OpenCV Auto",
        "PyAV",
    ]

    widget.deleteLater()
    _ = app


def test_settings_save_entry_is_global_only(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    saved = []

    def fake_save(self, intrinsics_path=None) -> None:
        saved.append(intrinsics_path)

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_save_current_settings", fake_save)

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()

    assert widget.findChild(QtWidgets.QPushButton, "saveYoloAdvancedSettingsButton") is None
    assert widget.findChild(QtWidgets.QPushButton, "saveMeasurementSettingsButton") is None
    assert widget.save_settings_button.text() == "전체 설정 저장"

    widget.save_settings_button.click()

    assert saved == [None]

    widget.deleteLater()
    _ = app


def test_measurement_runtime_settings_and_graph_are_collapsible(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()

    settings_group = widget.findChild(QtWidgets.QGroupBox, "runtimeCameraSettingsGroup")
    graph_group = widget.findChild(QtWidgets.QGroupBox, "runtimeDistanceGraphGroup")

    assert settings_group is not None
    assert graph_group is not None
    assert settings_group.isCheckable()
    assert graph_group.isCheckable()
    assert settings_group.isChecked()
    assert graph_group.isChecked()
    assert "RTSP 전송=UDP" in widget.runtime_stream_settings_label.text()
    assert "디코더=Auto" in widget.runtime_stream_settings_label.text()

    widget._log_state("left", "GStreamer 디코더: nvh264dec")
    assert "디코더=nvh264dec" in widget.runtime_stream_settings_label.text()

    widget.rtsp_transport_combo.setCurrentIndex(widget.rtsp_transport_combo.findText("TCP"))
    assert "RTSP 전송=TCP" in widget.runtime_stream_settings_label.text()
    assert "디코더=nvh264dec" in widget.runtime_stream_settings_label.text()

    settings_group.setChecked(False)
    assert widget.runtime_stream_settings_label.isHidden()
    settings_group.setChecked(True)
    assert not widget.runtime_stream_settings_label.isHidden()

    graph_group.setChecked(False)
    assert widget.distance_plot.isHidden()
    graph_group.setChecked(True)
    assert not widget.distance_plot.isHidden()

    widget.deleteLater()
    _ = app


def test_camera_pane_ignores_stale_thread_finished_signal() -> None:
    from PyQt5 import QtCore, QtWidgets

    from wiris_stereo_distance.camera_pane import CameraPane

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    pane = CameraPane("left")
    old_thread = QtCore.QThread()
    current_thread = QtCore.QThread()
    pane.stream_generation = 2
    pane.thread = current_thread
    pane.worker = object()

    pane._clear_thread_if_current(1, old_thread)

    assert pane.thread is current_thread
    assert pane.worker is not None

    pane._clear_thread_if_current(2, current_thread)

    assert pane.thread is None
    assert pane.worker is None

    pane.deleteLater()
    old_thread.deleteLater()
    current_thread.deleteLater()
    _ = app


def test_operation_status_header_is_not_rendered(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()

    assert not widget.findChildren(QtWidgets.QGroupBox, "operationHeader")
    assert not hasattr(widget, "operation_status_chips")
    assert not hasattr(widget, "next_action_button")

    widget.deleteLater()
    _ = app


def test_control_detection_applies_discovered_wiris_port(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    monkeypatch.setattr(gui, "check_tcp_port", lambda *_args, **_kwargs: True)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget.zoom_port_spin.setValue(2240)

    widget._apply_control_candidates(
        [
            {"host": "10.0.0.231", "port": 4321, "interface": None, "ok": True, "response": "OK"},
            {"host": "10.0.0.232", "port": 5432, "interface": None, "ok": True, "response": "OK"},
        ],
        [],
    )

    assert widget.zoom_port_spin.value() == 4321
    assert widget.left_control_port_spin.value() == 4321
    assert widget.right_control_port_spin.value() == 5432
    assert widget.left_zoom_host_edit.text() == "10.0.0.231"
    assert widget.right_zoom_host_edit.text() == "10.0.0.232"
    assert widget.camera_connection_label.text() == "camera control: left=control-open, right=control-open"

    widget.deleteLater()
    _ = app


def test_control_detection_target_side_only_updates_that_camera(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    monkeypatch.setattr(gui, "check_tcp_port", lambda *_args, **_kwargs: True)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget.left_zoom_host_edit.setText("10.0.0.231")
    widget.left_control_port_spin.setValue(2240)

    widget._apply_control_candidates(
        [{"host": "10.0.0.232", "port": 5432, "interface": None, "ok": True, "response": "OK"}],
        [],
        target_side="right",
    )

    assert widget.left_zoom_host_edit.text() == "10.0.0.231"
    assert widget.left_control_port_spin.value() == 2240
    assert widget.right_zoom_host_edit.text() == "10.0.0.232"
    assert widget.right_control_port_spin.value() == 5432

    widget.deleteLater()
    _ = app


def test_control_check_only_warns_when_rtsp_host_differs(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    monkeypatch.setattr(gui, "interface_has_address", lambda *_args, **_kwargs: True)
    monkeypatch.setattr(gui, "check_tcp_port", lambda host, *_args, **_kwargs: host == "10.0.0.231")
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget.left_zoom_host_edit.setText("10.0.0.231")
    widget.right_zoom_host_edit.setText("10.0.0.232")
    widget.left_url_edit.setText("rtsp://10.0.0.250:8554/visible")
    widget.right_url_edit.setText("rtsp://10.0.0.251:8554/visible")
    widget.left_rtsp_ip_edit.setText("10.0.0.250")
    widget.right_rtsp_ip_edit.setText("10.0.0.251")
    enabled = []
    widget.enable_wiris_rtsp = lambda: enabled.append(True)
    logs = []
    widget._log = logs.append

    widget.check_wiris_control()

    assert widget.left_url_edit.text() == "rtsp://10.0.0.250:8554/visible"
    assert widget.left_rtsp_ip_edit.text() != "10.0.0.231"
    assert widget.right_url_edit.text() == "rtsp://10.0.0.251:8554/visible"
    assert enabled == []
    assert any("OSD IP Address(10.0.0.231)" in message and "RTSP URL host(10.0.0.250)" in message for message in logs)

    widget.deleteLater()
    _ = app


def test_auto_wiris_connect_uses_discovered_port_before_enabling_rtsp(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget.zoom_port_spin.setValue(2240)
    enabled_ports = []
    widget.enable_wiris_rtsp = lambda: enabled_ports.append(
        (widget.left_control_port_spin.value(), widget.right_control_port_spin.value())
    )

    widget._apply_auto_wiris_connect_candidates(
        [
            {"host": "10.0.0.231", "port": 4321, "interface": None, "ok": True, "response": "OK"},
            {"host": "10.0.0.232", "port": 5432, "interface": None, "ok": True, "response": "OK"},
        ],
        [],
    )

    assert widget.zoom_port_spin.value() == 4321
    assert enabled_ports == [(4321, 5432)]
    assert widget.left_zoom_host_edit.text() == "10.0.0.231"
    assert widget.right_zoom_host_edit.text() == "10.0.0.232"

    widget.deleteLater()
    _ = app


def test_camera_stream_stats_format_includes_resolution() -> None:
    from wiris_stereo_distance.gui import format_camera_stream_stats

    preview_stats = {
        "decoded_hz": 29.75,
        "displayed_hz": 15.25,
        "inferred_hz": 0.0,
        "width": 1920,
        "height": 1080,
        "enable_detection": False,
    }
    yolo_stats = {
        "decoded_hz": 28.25,
        "displayed_hz": 0.0,
        "inferred_hz": 11.5,
        "width": 1280,
        "height": 720,
        "enable_detection": True,
    }

    assert format_camera_stream_stats("left", "connected", preview_stats) == (
        "왼쪽=연결 1920x1080 수신 29.8 Hz, 표시 15.2 Hz"
    )
    assert format_camera_stream_stats("right", "connected", yolo_stats) == (
        "오른쪽=연결 1280x720 수신 28.2 Hz, YOLO 11.5 Hz"
    )


def test_runtime_camera_stats_format_includes_receive_and_yolo_hz() -> None:
    from wiris_stereo_distance.gui import format_runtime_camera_stats

    stats = {
        "decoded_hz": 19.75,
        "inferred_hz": 7.25,
        "width": 1920,
        "height": 1080,
        "enable_detection": True,
    }

    assert format_runtime_camera_stats("left", stats, fallback_hz=12.3, stale=False) == (
        "왼쪽: 1920x1080 수신 19.8 Hz | YOLO 7.2 Hz"
    )
    assert format_runtime_camera_stats("right", {}, fallback_hz=12.34, stale=True) == (
        "오른쪽: -- 수신 12.3 Hz | YOLO -- | 지연"
    )


def test_rtk_alignment_view_separates_antenna_and_camera_baseline(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui
    from wiris_stereo_distance.pixhawk_attitude import CameraExtrinsicState
    from wiris_stereo_distance.rtk_geometry import GpsPositionState, RtkBaselineEstimate

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()

    baseline = RtkBaselineEstimate(
        baseline_m=0.374,
        north_m=0.1,
        east_m=0.3,
        down_m=0.2,
        left_age_s=0.01,
        right_age_s=0.02,
        fix_quality="GOOD",
    )
    widget.latest_rtk_baseline = baseline
    widget._refresh_rtk_alignment_view()

    assert widget.rtk_alignment_plot.baseline is baseline
    assert widget.rtk_alignment_plot.camera_translation_m is None
    assert "안테나 |T|=0.374 m" in widget.rtk_alignment_detail_label.text()
    assert "GPS-only 모드" in widget.rtk_alignment_detail_label.text()
    assert "카메라 기준 Right/Down/Forward 변환은 픽스호크 자세 수신 후" in widget.rtk_alignment_detail_label.text()

    left_attitude = CameraExtrinsicState(
        camera="left",
        namespace="/uav_0",
        roll_deg=6.0,
        pitch_deg=-9.0,
        yaw_deg=18.0,
        timestamp_s=0.0,
        topic="/uav_0/fmu/out/vehicle_attitude",
        q=(1.0, 0.0, 0.0, 0.0),
    )
    widget.latest_extrinsics["left"] = left_attitude
    widget._refresh_rtk_alignment_view()

    from math import sqrt

    assert widget.rtk_alignment_plot.camera_translation_m is not None
    assert abs(sqrt(sum(component * component for component in widget.rtk_alignment_plot.camera_translation_m)) - 0.3) < 1e-6
    assert widget.rtk_alignment_plot.left_attitude is left_attitude
    assert widget.rtk_alignment_plot.right_attitude is None
    assert widget.rtk_alignment_plot.minimumHeight() == 260
    assert widget.rtk_alignment_plot._attitude_arrow_components(left_attitude) == (0.4, -0.2, 6.0)
    assert widget.rtk_alignment_plot._format_relative_attitude_line() == "상대 ΔR/P/Y: 대기 중"
    assert "카메라 기준 Right=0.241 m" in widget.rtk_alignment_detail_label.text()
    assert "카메라 좌우 성분 |Right|=0.241 m" in widget.rtk_alignment_detail_label.text()

    right_attitude = CameraExtrinsicState(
        camera="right",
        namespace="/uav_1",
        roll_deg=0.0,
        pitch_deg=0.0,
        yaw_deg=0.0,
        timestamp_s=0.0,
        topic="/uav_1/fmu/out/vehicle_attitude",
        q=(1.0, 0.0, 0.0, 0.0),
    )
    widget.latest_extrinsics["right"] = right_attitude
    widget._refresh_rtk_alignment_view()

    assert widget.rtk_alignment_plot.right_attitude is right_attitude
    assert widget.rtk_alignment_plot._format_relative_attitude_line() == "상대 ΔR/P/Y: +0.00° / +0.00° / +0.00°"
    assert "상대 자세 ΔR/P/Y=+0.00/+0.00/+0.00°" in widget.rtk_alignment_detail_label.text()

    widget.deleteLater()
    _ = app


def test_live_gps_baseline_visualizes_without_rtk_fixed_or_reference_toggle(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui
    from wiris_stereo_distance.rtk_geometry import GpsPositionState

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()

    widget.use_rtk_baseline_checkbox.setChecked(False)
    widget.require_rtk_fixed_checkbox.setChecked(False)
    now_s = time.monotonic()
    widget._update_gps(
        "left",
        GpsPositionState(
            camera="left",
            namespace="/uav_0",
            timestamp_s=now_s,
            topic="/uav_0/fmu/out/vehicle_gps_position",
            latitude_deg=37.0,
            longitude_deg=127.0,
            altitude_m=50.0,
            fix_type=3,
            eph_m=0.02,
        ),
    )
    widget._update_gps(
        "right",
        GpsPositionState(
            camera="right",
            namespace="/uav_1",
            timestamp_s=now_s,
            topic="/uav_1/fmu/out/vehicle_gps_position",
            latitude_deg=37.0,
            longitude_deg=127.000003,
            altitude_m=50.0,
            fix_type=3,
            eph_m=0.02,
        ),
    )

    assert widget.latest_rtk_baseline is not None
    assert widget.latest_rtk_baseline.fix_quality == "DEGRADED"
    assert widget.rtk_alignment_plot.baseline is widget.latest_rtk_baseline
    assert "GPS baseline:" in widget.rtk_baseline_label.text()
    assert "계산 OFF, 표시만" in widget.rtk_baseline_label.text()
    assert "left 3D_FIX" in widget.rtk_baseline_label.text()
    assert "right 3D_FIX" in widget.rtk_baseline_label.text()
    assert "RTK Fixed: NO" in widget.rtk_fixed_status_label.text()
    assert "GPS baseline 계산 OFF" in widget.rtk_fixed_status_label.text()
    assert "ENU E/N/U=" in widget.rtk_alignment_detail_label.text()
    assert "NED N/E/D=" in widget.rtk_alignment_detail_label.text()
    assert "GPS 위치 대기" not in widget.rtk_alignment_detail_label.text()

    widget.deleteLater()
    _ = app


def test_gps_only_baseline_can_drive_pixhawk_translation_when_fixed_not_required(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui
    from wiris_stereo_distance.pixhawk_attitude import CameraExtrinsicState
    from wiris_stereo_distance.rtk_geometry import GpsPositionState

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()

    widget.use_rtk_baseline_checkbox.setChecked(True)
    widget.require_rtk_fixed_checkbox.setChecked(False)
    now_s = time.monotonic()
    widget._update_gps(
        "left",
        GpsPositionState("left", "/uav_0", now_s, "gps", 37.0, 127.0, 10.0, fix_type=3, satellites_used=8, eph_m=2.5),
    )
    widget._update_gps(
        "right",
        GpsPositionState("right", "/uav_1", now_s, "gps", 37.0, 127.000003, 10.0, fix_type=3, satellites_used=9, eph_m=2.2),
    )
    widget.latest_extrinsics["left"] = CameraExtrinsicState(
        camera="left",
        namespace="/uav_0",
        roll_deg=0.0,
        pitch_deg=0.0,
        yaw_deg=0.0,
        timestamp_s=now_s,
        topic="/uav_0/fmu/out/vehicle_attitude",
        q=(1.0, 0.0, 0.0, 0.0),
    )

    translation = widget._translation_right_in_left_m()

    assert translation is not None
    assert widget.latest_rtk_baseline is not None
    assert abs(sum(component * component for component in translation) ** 0.5 - widget.latest_rtk_baseline.baseline_m) < 1e-6
    assert widget.latest_rtk_baseline is not None
    assert widget.latest_rtk_baseline.fix_quality == "DEGRADED"
    assert "RTK Fixed: NO" in widget.rtk_fixed_status_label.text()
    assert "사용 가능" in widget.rtk_fixed_status_label.text()
    assert "baseline_source_detail: live GPS/RTK baseline" in widget._format_stereo_geometry_text()

    widget.deleteLater()
    _ = app


def test_rtk_fixed_required_blocks_gps_only_translation(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui
    from wiris_stereo_distance.pixhawk_attitude import CameraExtrinsicState
    from wiris_stereo_distance.rtk_geometry import GpsPositionState

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()

    widget.use_rtk_baseline_checkbox.setChecked(True)
    widget.require_rtk_fixed_checkbox.setChecked(True)
    now_s = time.monotonic()
    widget._update_gps(
        "left",
        GpsPositionState("left", "/uav_0", now_s, "gps", 37.0, 127.0, 10.0, fix_type=3, satellites_used=8, eph_m=2.5),
    )
    widget._update_gps(
        "right",
        GpsPositionState("right", "/uav_1", now_s, "gps", 37.0, 127.000003, 10.0, fix_type=3, satellites_used=9, eph_m=2.2),
    )
    widget.latest_extrinsics["left"] = CameraExtrinsicState(
        camera="left",
        namespace="/uav_0",
        roll_deg=0.0,
        pitch_deg=0.0,
        yaw_deg=0.0,
        timestamp_s=now_s,
        topic="/uav_0/fmu/out/vehicle_attitude",
        q=(1.0, 0.0, 0.0, 0.0),
    )

    assert widget._translation_right_in_left_m() is None
    assert "RTK Fixed required" in widget.rtk_fixed_status_label.text()
    assert widget._active_baseline_m()[1] == "gps-rtk-blocked"

    widget.deleteLater()
    _ = app


def test_pixhawk_translation_scales_to_measurement_baseline(monkeypatch) -> None:
    from math import sqrt

    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui
    from wiris_stereo_distance.pixhawk_attitude import CameraExtrinsicState
    from wiris_stereo_distance.rtk_geometry import GpsPositionState, RtkBaselineEstimate

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()

    widget.use_rtk_baseline_checkbox.setChecked(True)
    widget.latest_rtk_baseline = RtkBaselineEstimate(
        baseline_m=0.374,
        north_m=0.0,
        east_m=0.3,
        down_m=0.0,
        left_age_s=0.01,
        right_age_s=0.02,
        fix_quality="GOOD",
    )
    now_s = time.monotonic()
    widget.latest_gps["left"] = GpsPositionState("left", "/uav_0", now_s, "gps", 37.0, 127.0, 10.0, fix_type=6)
    widget.latest_gps["right"] = GpsPositionState("right", "/uav_1", now_s, "gps", 37.0, 127.000003, 10.0, fix_type=6)
    widget.latest_extrinsics["left"] = CameraExtrinsicState(
        camera="left",
        namespace="/uav_0",
        roll_deg=0.0,
        pitch_deg=0.0,
        yaw_deg=0.0,
        timestamp_s=now_s,
        topic="mavlink/ATTITUDE",
        q=(1.0, 0.0, 0.0, 0.0),
    )

    translation = widget._translation_right_in_left_m()
    assert translation is not None
    assert abs(sqrt(sum(component * component for component in translation)) - 0.3) < 1e-6
    assert translation == (0.3, 0.0, 0.0)

    widget.deleteLater()
    _ = app


def test_pixhawk_translation_uses_nominal_shared_rig_without_mount_correction_ui(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui
    from wiris_stereo_distance.pixhawk_attitude import CameraExtrinsicState
    from wiris_stereo_distance.rtk_geometry import GpsPositionState, RtkBaselineEstimate

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()

    widget.latest_rtk_baseline = RtkBaselineEstimate(
        baseline_m=1.0,
        north_m=0.0,
        east_m=1.0,
        down_m=0.0,
        left_age_s=0.01,
        right_age_s=0.02,
        fix_quality="GOOD",
    )
    now_s = time.monotonic()
    widget.latest_gps["left"] = GpsPositionState("left", "/uav_0", now_s, "gps", 37.0, 127.0, 10.0, fix_type=6)
    widget.latest_gps["right"] = GpsPositionState("right", "/uav_1", now_s, "gps", 37.0, 127.00001, 10.0, fix_type=6)
    widget.latest_extrinsics["left"] = CameraExtrinsicState(
        camera="left",
        namespace="/uav_0",
        roll_deg=0.0,
        pitch_deg=0.0,
        yaw_deg=0.0,
        timestamp_s=now_s,
        topic="mavlink/ATTITUDE",
        q=(1.0, 0.0, 0.0, 0.0),
    )

    translation = widget._translation_right_in_left_m()

    assert translation is not None
    assert tuple(round(component, 6) for component in translation) == (1.0, 0.0, 0.0)
    assert "장착=GPS/RTK baseline only" in widget._distance_parameters_summary()
    assert "수동 baseline 입력은 사용하지 않고" in widget.mount_assumption_label.text()

    widget.deleteLater()
    _ = app


def test_pixhawk_translation_waits_without_gps_baseline(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui
    from wiris_stereo_distance.pixhawk_attitude import CameraExtrinsicState

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()

    widget.latest_rtk_baseline = None
    now_s = time.monotonic()
    widget.latest_extrinsics["left"] = CameraExtrinsicState(
        camera="left",
        namespace="/uav_0",
        roll_deg=0.0,
        pitch_deg=0.0,
        yaw_deg=0.0,
        timestamp_s=now_s,
        topic="mavlink/ATTITUDE",
        q=(1.0, 0.0, 0.0, 0.0),
    )

    assert widget._translation_right_in_left_m() is None

    widget.deleteLater()
    _ = app


def test_extract_field_path_accepts_ground_analysis_colon_style() -> None:
    from types import SimpleNamespace

    from wiris_stereo_distance.pixhawk_attitude import extract_field_path

    message = SimpleNamespace(nested=SimpleNamespace(value=7.5), q=[1.0, 0.0, 0.0, 0.0])

    assert extract_field_path(message, "nested:value") == 7.5
    assert extract_field_path(message, "q[0]") == 1.0


def test_add_custom_topic_watch_uses_ground_analysis_key_format() -> None:
    from PyQt5 import QtWidgets

    from wiris_stereo_distance.gui import WirisStereoDistancePanel

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = WirisStereoDistancePanel()

    widget._add_custom_topic_watch(
        key="ros2:/uav_0/fmu/out/vehicle_local_position:z",
        topic="/uav_0/fmu/out/vehicle_local_position",
        field="z",
    )

    assert widget.custom_watch_specs[-1] == {
        "key": "ros2:/uav_0/fmu/out/vehicle_local_position:z",
        "topic": "/uav_0/fmu/out/vehicle_local_position",
        "field": "z",
        "source": "ros2",
    }
    assert widget.custom_watch_remove_combo.findData("ros2:/uav_0/fmu/out/vehicle_local_position:z") >= 0

    widget.deleteLater()
    _ = app


def test_mavlink_custom_watch_emits_selected_field() -> None:
    from types import SimpleNamespace

    from wiris_stereo_distance.pixhawk_attitude import DualPixhawkMavlinkSubscriber

    class FakeAttitude(SimpleNamespace):
        def get_type(self):
            return "ATTITUDE"

    subscriber = DualPixhawkMavlinkSubscriber(
        left_url="",
        right_url="",
        custom_watches=(
            {
                "source": "mavlink",
                "message": "ATTITUDE",
                "field": "roll",
                "key": "mavlink:ATTITUDE.roll",
            },
        ),
    )
    captured = []
    subscriber.custom_watch_changed.connect(captured.append)

    subscriber._handle_mavlink_message("left", FakeAttitude(roll=1.25, pitch=0.0, yaw=0.0))

    assert len(captured) == 1
    assert captured[0].key == "left:mavlink:ATTITUDE.roll"
    assert captured[0].topic == "mavlink/ATTITUDE"
    assert captured[0].field_path == "roll"
    assert captured[0].numeric_value == 1.25


def test_mavlink_graphable_fields_include_attitude_roll() -> None:
    from wiris_stereo_distance.pixhawk_attitude import mavlink_graphable_fields_for_message

    assert "roll" in mavlink_graphable_fields_for_message("ATTITUDE")

def test_attitude_plot_updates_with_single_pixhawk_sample() -> None:
    from PyQt5 import QtWidgets

    from wiris_stereo_distance.gui import WirisStereoDistancePanel
    from wiris_stereo_distance.pixhawk_attitude import CameraExtrinsicState

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = WirisStereoDistancePanel()

    widget._update_extrinsic(
        "left",
        CameraExtrinsicState(
            camera="left",
            namespace="/dev/ttyACM0",
            roll_deg=1.0,
            pitch_deg=2.0,
            yaw_deg=3.0,
            timestamp_s=0.0,
            topic="mavlink/ATTITUDE",
            q=(1.0, 0.0, 0.0, 0.0),
        ),
    )

    roll_point = widget.attitude_history["roll"][-1]
    assert roll_point.left_deg == 1.0
    assert roll_point.right_deg is None
    left_label, right_label, delta_label = widget.attitude_compare_labels["roll"]
    assert left_label.text() == "1.0000 deg"
    assert right_label.text() == "대기 중"
    assert delta_label.text() == "대기 중"

    widget.deleteLater()
    _ = app


def test_attitude_plot_auto_y_axis_uses_current_min_and_max() -> None:
    from wiris_stereo_distance.gui import AttitudeHistoryPlot

    assert AttitudeHistoryPlot.auto_y_bounds([-2.0, 8.0, 3.0]) == (-2.0, 8.0)


def test_attitude_tab_has_no_manual_y_axis_controls() -> None:
    from PyQt5 import QtWidgets

    from wiris_stereo_distance.gui import WirisStereoDistancePanel

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = WirisStereoDistancePanel()

    assert not hasattr(widget, "attitude_y_axis_combos")
    assert all(group.title() != "그래프 Y축" for group in widget.findChildren(QtWidgets.QGroupBox))

    widget.deleteLater()
    _ = app


def test_mavlink_heartbeat_logs_once_per_camera() -> None:
    from types import SimpleNamespace

    from wiris_stereo_distance.pixhawk_attitude import DualPixhawkMavlinkSubscriber

    class FakeHeartbeat(SimpleNamespace):
        def get_type(self):
            return "HEARTBEAT"

    subscriber = DualPixhawkMavlinkSubscriber(left_url="udpin:0.0.0.0:14550", right_url="")
    messages = []
    subscriber.status_changed.connect(messages.append)

    subscriber._handle_mavlink_message("left", FakeHeartbeat())
    subscriber._handle_mavlink_message("left", FakeHeartbeat())

    assert messages == ["왼쪽 MAVLink heartbeat 수신"]


def test_mavlink_heartbeat_requests_rm3100_sensor_streams_once() -> None:
    from types import SimpleNamespace

    from wiris_stereo_distance.pixhawk_attitude import (
        DualPixhawkMavlinkSubscriber,
        MAVLINK_MESSAGE_IDS,
        MAVLINK_RM3100_STREAM_REQUESTS,
    )

    class FakeHeartbeat(SimpleNamespace):
        def get_type(self):
            return "HEARTBEAT"

    class FakeMav:
        def __init__(self):
            self.commands = []

        def command_long_send(self, *args):
            self.commands.append(args)

    fake_mav = FakeMav()
    fake_connection = SimpleNamespace(
        mav=fake_mav,
        mavlink=SimpleNamespace(MAV_CMD_SET_MESSAGE_INTERVAL=511),
        target_system=7,
        target_component=3,
    )
    subscriber = DualPixhawkMavlinkSubscriber(left_url="udpin:0.0.0.0:14550", right_url="")
    subscriber.connections["left"] = fake_connection
    messages = []
    subscriber.status_changed.connect(messages.append)

    subscriber._handle_mavlink_message("left", FakeHeartbeat())
    subscriber._handle_mavlink_message("left", FakeHeartbeat())

    assert [command[4] for command in fake_mav.commands] == [
        MAVLINK_MESSAGE_IDS[name] for name, _rate_hz in MAVLINK_RM3100_STREAM_REQUESTS
    ]
    assert [command[5] for command in fake_mav.commands] == [
        int(1_000_000 / rate_hz) for _name, rate_hz in MAVLINK_RM3100_STREAM_REQUESTS
    ]
    assert all(command[:4] == (7, 3, 511, 0) for command in fake_mav.commands)
    assert len(fake_mav.commands) == len(MAVLINK_RM3100_STREAM_REQUESTS)
    assert messages[0].startswith("왼쪽 MAVLink sensor stream 요청:")
    assert messages[1] == "왼쪽 MAVLink heartbeat 수신"


def test_mavlink_serial_control_emits_shell_output() -> None:
    from types import SimpleNamespace

    from wiris_stereo_distance.pixhawk_attitude import DualPixhawkMavlinkSubscriber

    class FakeSerialControl(SimpleNamespace):
        def get_type(self):
            return "SERIAL_CONTROL"

    subscriber = DualPixhawkMavlinkSubscriber(left_url="", right_url="")
    outputs = []
    subscriber.shell_output.connect(lambda camera, text: outputs.append((camera, text)))

    subscriber._handle_mavlink_message("left", FakeSerialControl(count=12, data=list(b"px4> status\n")))

    assert outputs == [("left", "px4> status\n")]


def test_pixhawk_topic_discovery_extracts_domain_and_namespaces(monkeypatch) -> None:
    from types import SimpleNamespace

    import wiris_stereo_distance.pixhawk_attitude as pixhawk_attitude

    def fake_run(args, **kwargs):
        domain_id = kwargs["env"]["ROS_DOMAIN_ID"]
        if args == ["ros2", "topic", "info", "/uav_0/fmu/out/vehicle_attitude"]:
            return SimpleNamespace(returncode=0, stdout="Type: px4_msgs/msg/VehicleAttitude\nPublisher count: 1\n")
        if args == ["ros2", "topic", "info", "/uav_1/fmu/out/sensor_gps"]:
            return SimpleNamespace(returncode=0, stdout="Type: px4_msgs/msg/SensorGps\nPublisher count: 1\n")
        assert args == ["ros2", "topic", "list"]
        if domain_id == "3":
            return SimpleNamespace(
                returncode=0,
                stdout="/uav_0/fmu/out/vehicle_attitude\n/uav_1/fmu/out/sensor_gps\n",
            )
        return SimpleNamespace(returncode=0, stdout="")

    monkeypatch.setattr(pixhawk_attitude.subprocess, "run", fake_run)

    discovery = pixhawk_attitude.discover_pixhawk_topics(
        current_domain_id=0,
        extra_domain_ids=(3,),
        timeout_s=0.1,
    )

    assert discovery is not None
    assert discovery.domain_id == 3
    assert discovery.namespaces == ("uav_0", "uav_1")


def test_pixhawk_topic_discovery_ignores_subscription_only_topics(monkeypatch) -> None:
    from types import SimpleNamespace

    import wiris_stereo_distance.pixhawk_attitude as pixhawk_attitude

    def fake_run(args, **kwargs):
        if args == ["ros2", "topic", "list"]:
            return SimpleNamespace(returncode=0, stdout="/uav_0/fmu/out/vehicle_attitude\n")
        if args == ["ros2", "topic", "info", "/uav_0/fmu/out/vehicle_attitude"]:
            return SimpleNamespace(returncode=0, stdout="Type: px4_msgs/msg/VehicleAttitude\nPublisher count: 0\n")
        raise AssertionError(args)

    monkeypatch.setattr(pixhawk_attitude.subprocess, "run", fake_run)

    discovery = pixhawk_attitude.discover_pixhawk_topics(
        current_domain_id=0,
        extra_domain_ids=(),
        timeout_s=0.1,
    )

    assert discovery is None


def test_camera_extrinsic_maps_pixhawk_namespace_to_camera() -> None:
    from wiris_stereo_distance.pixhawk_attitude import camera_extrinsic_from_quaternion

    extrinsic = camera_extrinsic_from_quaternion(
        camera="left",
        namespace="/uav_0/",
        q=[1, 0, 0, 0],
        timestamp_s=123.0,
    )

    assert extrinsic.camera == "left"
    assert extrinsic.namespace == "uav_0"
    assert extrinsic.topic == "/uav_0/fmu/out/vehicle_attitude"
    assert extrinsic.timestamp_s == 123.0
    assert extrinsic.roll_deg == 0
    assert extrinsic.q == (1.0, 0.0, 0.0, 0.0)


def test_dual_pixhawk_start_treats_non_attitude_messages_as_optional(monkeypatch) -> None:
    from types import SimpleNamespace

    import wiris_stereo_distance.pixhawk_attitude as pixhawk_attitude
    from wiris_stereo_distance.pixhawk_attitude import DualPixhawkAttitudeSubscriber

    class FakeNode:
        def __init__(self) -> None:
            self.topics = []

        def create_subscription(self, _msg_type, topic, _callback, _qos):
            self.topics.append(topic)
            return topic

    class FakeExecutor:
        def add_node(self, _node) -> None:
            return None

    fake_node = FakeNode()

    def fake_get_message(name: str):
        if name == "px4_msgs/msg/VehicleAttitude":
            return object
        raise AttributeError(name)

    monkeypatch.setattr(pixhawk_attitude, "get_message", fake_get_message)
    monkeypatch.setattr(pixhawk_attitude, "SingleThreadedExecutor", FakeExecutor)
    monkeypatch.setattr(
        pixhawk_attitude,
        "rclpy",
        SimpleNamespace(ok=lambda: True, create_node=lambda _name: fake_node),
    )

    subscriber = DualPixhawkAttitudeSubscriber(domain_id=0, left_namespace="uav_0", right_namespace="uav_1")
    subscriber.start()

    assert fake_node.topics == [
        "/uav_0/fmu/out/vehicle_attitude",
        "/uav_1/fmu/out/vehicle_attitude",
    ]


def test_dual_pixhawk_start_uses_sensor_gps_for_vehicle_gps_position_when_needed(monkeypatch) -> None:
    from types import SimpleNamespace

    import wiris_stereo_distance.pixhawk_attitude as pixhawk_attitude
    from wiris_stereo_distance.pixhawk_attitude import DualPixhawkAttitudeSubscriber

    class FakeNode:
        def __init__(self) -> None:
            self.topics = []

        def create_subscription(self, _msg_type, topic, _callback, _qos):
            self.topics.append(topic)
            return topic

    class FakeExecutor:
        def add_node(self, _node) -> None:
            return None

    fake_node = FakeNode()

    def fake_get_message(name: str):
        if name in {"px4_msgs/msg/VehicleAttitude", "px4_msgs/msg/SensorGps"}:
            return object
        raise AttributeError(name)

    monkeypatch.setattr(pixhawk_attitude, "get_message", fake_get_message)
    monkeypatch.setattr(pixhawk_attitude, "SingleThreadedExecutor", FakeExecutor)
    monkeypatch.setattr(
        pixhawk_attitude,
        "rclpy",
        SimpleNamespace(ok=lambda: True, create_node=lambda _name: fake_node),
    )

    subscriber = DualPixhawkAttitudeSubscriber(domain_id=0, left_namespace="uav_0", right_namespace="")
    subscriber.start()

    assert "/uav_0/fmu/out/vehicle_gps_position" in fake_node.topics
    assert "/uav_0/fmu/out/sensor_gps" in fake_node.topics
    assert "/uav_0/fmu/out/sensor_gps_0" in fake_node.topics
    assert "/uav_0/fmu/out/sensor_gps_1" in fake_node.topics


def test_dual_pixhawk_start_subscribes_sensor_mag_instances(monkeypatch) -> None:
    from types import SimpleNamespace

    import wiris_stereo_distance.pixhawk_attitude as pixhawk_attitude
    from wiris_stereo_distance.pixhawk_attitude import DualPixhawkAttitudeSubscriber

    class FakeNode:
        def __init__(self) -> None:
            self.topics = []

        def create_subscription(self, _msg_type, topic, _callback, _qos):
            self.topics.append(topic)
            return topic

    class FakeExecutor:
        def add_node(self, _node) -> None:
            return None

    fake_node = FakeNode()

    def fake_get_message(name: str):
        if name in {"px4_msgs/msg/VehicleAttitude", "px4_msgs/msg/SensorMag"}:
            return object
        raise AttributeError(name)

    monkeypatch.setattr(pixhawk_attitude, "get_message", fake_get_message)
    monkeypatch.setattr(pixhawk_attitude, "SingleThreadedExecutor", FakeExecutor)
    monkeypatch.setattr(
        pixhawk_attitude,
        "rclpy",
        SimpleNamespace(ok=lambda: True, create_node=lambda _name: fake_node),
    )

    subscriber = DualPixhawkAttitudeSubscriber(domain_id=0, left_namespace="uav_0", right_namespace="")
    subscriber.start()

    assert "/uav_0/fmu/out/sensor_mag" in fake_node.topics
    for instance in range(5):
        assert f"/uav_0/fmu/out/sensor_mag_{instance}" in fake_node.topics


def test_dual_pixhawk_vehicle_gps_sensor_gps_fallback_emits_both_states() -> None:
    from types import SimpleNamespace

    from wiris_stereo_distance.pixhawk_attitude import DualPixhawkAttitudeSubscriber

    subscriber = DualPixhawkAttitudeSubscriber(domain_id=0, left_namespace="uav_0", right_namespace="")
    gps_positions = []
    gps_sensors = []
    subscriber.gps_changed.connect(lambda camera, state: gps_positions.append((camera, state)))
    subscriber.gps_sensor_changed.connect(lambda camera, state: gps_sensors.append((camera, state)))

    subscriber._handle_gps_position_message(
        "left",
        "uav_0",
        SimpleNamespace(
            timestamp=1_000_000,
            lat=370000000,
            lon=1270000000,
            alt=123450,
            fix_type=6,
            satellites_used=18,
            eph=0.4,
            epv=0.5,
            device_id=42,
        ),
        emit_sensor_gps=True,
    )

    assert gps_positions[0][0] == "left"
    assert gps_positions[0][1].topic == "/uav_0/fmu/out/vehicle_gps_position"
    assert gps_positions[0][1].latitude_deg == 37.0
    assert gps_sensors[0][0] == "left"
    assert gps_sensors[0][1].topic == "/uav_0/fmu/out/vehicle_gps_position"
    assert gps_sensors[0][1].sensor_slot == "GPS1"


def test_dual_pixhawk_vehicle_magnetometer_vector_maps_to_mag_sensor() -> None:
    from types import SimpleNamespace

    from wiris_stereo_distance.pixhawk_attitude import DualPixhawkAttitudeSubscriber, vehicle_magnetometer_topic

    subscriber = DualPixhawkAttitudeSubscriber(domain_id=0, left_namespace="uav_0", right_namespace="")
    mags = []
    subscriber.mag_sensor_changed.connect(lambda camera, state: mags.append((camera, state)))

    subscriber._handle_mag_message(
        "left",
        "uav_0",
        SimpleNamespace(timestamp=1_000_000, device_id=12, magnetometer_ga=[1.0, 2.0, 2.0]),
        topic=vehicle_magnetometer_topic("uav_0"),
    )

    assert mags[0][0] == "left"
    assert mags[0][1].topic == "/uav_0/fmu/out/vehicle_magnetometer"
    assert mags[0][1].sensor_slot == "SELECTED"
    assert mags[0][1].magnitude == 3.0


def test_rm3100_yaw_source_selection_sets_px4_primary_mag(monkeypatch) -> None:
    from types import SimpleNamespace

    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui
    from wiris_stereo_distance.pixhawk_attitude import MagSensorState

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    sent = []
    widget.mavlink_pixhawk = SimpleNamespace(send_shell_command=lambda camera, command: sent.append((camera, command)))
    widget.latest_mag_sensors["left"]["CAN1"] = MagSensorState(
        camera="left",
        namespace="uav_0",
        sensor_slot="CAN1",
        timestamp_s=10.0,
        topic="/uav_0/fmu/out/sensor_mag",
        device_id=12345,
        x=1.0,
        y=2.0,
        z=3.0,
        magnitude=3.74,
    )
    widget.rm3100_yaw_source_combos["left"].setCurrentIndex(
        widget.rm3100_yaw_source_combos["left"].findData("CAN1")
    )

    widget.apply_rm3100_yaw_source("left")

    assert sent == [("left", "param set CAL_MAG_PRIME 12345\nparam save\nparam show CAL_MAG_PRIME")]
    assert "CAN1 device=12345" in widget.rm3100_yaw_source_status_labels["left"].text()

    widget.deleteLater()
    _ = app


def test_rm3100_yaw_source_selection_uses_inferred_mavlink_device_id(monkeypatch) -> None:
    from types import SimpleNamespace

    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui
    from wiris_stereo_distance.pixhawk_attitude import MagSensorState
    from wiris_stereo_distance.px4_mag_manager import RM3100_BACKUP_ID

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    sent = []
    widget.mavlink_pixhawk = SimpleNamespace(send_shell_command=lambda camera, command: sent.append((camera, command)))
    widget.latest_mag_sensors["left"]["CAN2"] = MagSensorState(
        camera="left",
        namespace="mavlink:left",
        sensor_slot="CAN2",
        timestamp_s=10.0,
        topic="mavlink/SCALED_IMU2",
        device_id=None,
        x=1.0,
        y=2.0,
        z=3.0,
        magnitude=3.74,
    )
    widget.rm3100_yaw_source_combos["left"].setCurrentIndex(
        widget.rm3100_yaw_source_combos["left"].findData("CAN2")
    )

    widget.apply_rm3100_yaw_source("left")

    assert sent == [
        ("left", f"param set CAL_MAG_PRIME {RM3100_BACKUP_ID}\nparam save\nparam show CAL_MAG_PRIME")
    ]
    assert f"CAN2 device={RM3100_BACKUP_ID}" in widget.rm3100_yaw_source_status_labels["left"].text()
    assert "PX4 param inferred" in widget.rm3100_yaw_source_status_labels["left"].text()

    widget.deleteLater()
    _ = app


def test_px4_mag_manager_stays_empty_until_pixhawk_mag_source(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()

    for side in ("left", "right"):
        assert widget.px4_mag_tables[side].rowCount() == 0
        assert "프로토콜 수신 대기" in widget.px4_mag_status_labels[side].text()
        assert not widget.px4_mag_refresh_buttons[side].isEnabled()
        assert not widget.px4_mag_use_rm3100_buttons[side].isEnabled()
        assert not widget.px4_mag_save_buttons[side].isEnabled()
        assert not widget.px4_mag_reboot_buttons[side].isEnabled()

    widget.deleteLater()
    _ = app


def test_px4_mag_manager_tracks_actual_selected_mag_per_pixhawk(monkeypatch) -> None:
    from types import SimpleNamespace

    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    now_s = time.monotonic()

    widget._update_mag_sensor(
        "left",
        SimpleNamespace(
            topic="/uav_0/fmu/out/vehicle_magnetometer",
            device_id=111,
            sensor_slot="SELECTED",
            timestamp_s=now_s,
            x=None,
            y=None,
            z=None,
            magnitude=None,
        ),
    )
    widget._update_mag_sensor(
        "right",
        SimpleNamespace(
            topic="/uav_1/fmu/out/vehicle_magnetometer",
            device_id=222,
            sensor_slot="SELECTED",
            timestamp_s=now_s,
            x=None,
            y=None,
            z=None,
            magnitude=None,
        ),
    )

    assert widget.latest_vehicle_mag_device_ids == {"left": 111, "right": 222}
    assert "Pixhawk 1" in widget.px4_mag_status_labels["left"].text()
    assert "Actual selected MAG: 111" in widget.px4_mag_status_labels["left"].text()
    assert "Actual selected MAG: 222" not in widget.px4_mag_status_labels["left"].text()
    assert "Pixhawk 2" in widget.px4_mag_status_labels["right"].text()
    assert "Actual selected MAG: 222" in widget.px4_mag_status_labels["right"].text()

    widget.deleteLater()
    _ = app


def test_px4_mag_manager_uses_mavlink_params_when_connected(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui
    from wiris_stereo_distance.px4_mag_manager import GPS1_MAG_ID, GPS2_MAG_ID, RM3100_BACKUP_ID, RM3100_PRIMARY_ID

    class FakeMavlink:
        def __init__(self) -> None:
            self.connections = {"left": object()}
            self._heartbeat_logged = {"left"}
            self.values = {
                "UAVCAN_ENABLE": 2,
                "UAVCAN_SUB_MAG": 1,
                "EKF2_MAG_TYPE": 0,
                "CAL_MAG0_ID": GPS1_MAG_ID,
                "CAL_MAG0_PRIO": 75,
                "CAL_MAG0_ROT": 0,
                "CAL_MAG1_ID": GPS2_MAG_ID,
                "CAL_MAG1_PRIO": 75,
                "CAL_MAG1_ROT": 0,
                "CAL_MAG2_ID": RM3100_PRIMARY_ID,
                "CAL_MAG2_PRIO": 50,
                "CAL_MAG2_ROT": 0,
                "CAL_MAG3_ID": RM3100_BACKUP_ID,
                "CAL_MAG3_PRIO": 50,
                "CAL_MAG3_ROT": 0,
                "CAL_MAG4_ID": None,
                "CAL_MAG4_PRIO": None,
                "CAL_MAG4_ROT": None,
            }
            self.set_calls: list[tuple[str, str, int]] = []

        def read_param_int(self, camera: str, name: str, *, timeout_s: float = 2.0) -> int | None:
            assert camera == "left"
            return self.values.get(name)

        def set_param_int(self, camera: str, name: str, value: int, *, timeout_s: float = 2.0) -> int | None:
            assert camera == "left"
            self.values[name] = value
            self.set_calls.append((camera, name, value))
            return value

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget.mavlink_pixhawk = FakeMavlink()
    widget.latest_vehicle_mag_device_ids["left"] = GPS1_MAG_ID

    widget.use_rm3100_for_yaw_from_panel("left")

    assert ("left", "CAL_MAG0_PRIO", 0) in widget.mavlink_pixhawk.set_calls
    assert ("left", "CAL_MAG1_PRIO", 0) in widget.mavlink_pixhawk.set_calls
    assert ("left", "CAL_MAG2_PRIO", 100) in widget.mavlink_pixhawk.set_calls
    assert ("left", "CAL_MAG3_PRIO", 90) in widget.mavlink_pixhawk.set_calls
    assert "Actual selected MAG" in widget.px4_mag_status_labels["left"].text()

    widget.deleteLater()
    _ = app


def test_session_save_writes_session_folder_and_pixhawk_mag_csv(tmp_path, monkeypatch) -> None:
    import csv
    import json

    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui
    from wiris_stereo_distance.px4_mag_manager import RM3100_PRIMARY_ID

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget.calib_dir_edit.setText(str(tmp_path / "calibration_captures"))
    widget.recording_session_id = "20260605_153012"
    widget.recording_session_started_wall_s = 1_800_000_000.0
    widget.recording_session_started_monotonic_s = time.monotonic() - 3.0
    widget.latest_vehicle_mag_device_ids["left"] = RM3100_PRIMARY_ID
    widget.latest_vehicle_mag_device_ids["right"] = 396809
    widget.uam_analysis_rows = [{"row_index": "1", "error_distance_m": "0.25"}]

    widget.save_distance_history_csv()

    session_dir = tmp_path / "calibration_captures" / "sessions" / "20260605_153012"
    metadata_path = session_dir / "session_metadata.json"
    mag_path = session_dir / "pixhawk_mag_status.csv"
    uam_path = session_dir / "uam_analysis.csv"
    assert metadata_path.exists()
    assert mag_path.exists()
    assert uam_path.exists()
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    assert metadata["session_dir"] == str(session_dir)
    assert metadata["pixhawk_mag_status_sample_count"] == 10
    assert metadata["uam_analysis_sample_count"] == 1
    assert metadata["pixhawk_mag_actual_selected_device_ids"] == {
        "left": RM3100_PRIMARY_ID,
        "right": 396809,
    }
    rows = list(csv.DictReader(mag_path.open(newline="", encoding="utf-8")))
    assert len(rows) == 10
    assert {row["pixhawk_label"] for row in rows} == {"Pixhawk 1", "Pixhawk 2"}
    assert any(
        row["pixhawk_side"] == "left"
        and row["actual_selected_device_id"] == str(RM3100_PRIMARY_ID)
        and row["actual_selected"] == "true"
        for row in rows
    )
    assert any(row["pixhawk_side"] == "right" and row["actual_selected_device_id"] == "396809" for row in rows)
    uam_rows = list(csv.DictReader(uam_path.open(newline="", encoding="utf-8")))
    assert uam_rows == [{"row_index": "1", "error_distance_m": "0.25"}]

    widget.deleteLater()
    _ = app


def test_dual_pixhawk_start_allows_no_px4_message_types(monkeypatch) -> None:
    from types import SimpleNamespace

    import wiris_stereo_distance.pixhawk_attitude as pixhawk_attitude
    from wiris_stereo_distance.pixhawk_attitude import DualPixhawkAttitudeSubscriber

    class FakeNode:
        def __init__(self) -> None:
            self.topics = []

        def create_subscription(self, _msg_type, topic, _callback, _qos):
            self.topics.append(topic)
            return topic

    class FakeExecutor:
        def add_node(self, _node) -> None:
            return None

    fake_node = FakeNode()

    monkeypatch.setattr(
        pixhawk_attitude,
        "get_message",
        lambda name: (_ for _ in ()).throw(AttributeError(name)),
    )
    monkeypatch.setattr(pixhawk_attitude, "SingleThreadedExecutor", FakeExecutor)
    monkeypatch.setattr(
        pixhawk_attitude,
        "rclpy",
        SimpleNamespace(ok=lambda: True, create_node=lambda _name: fake_node),
    )

    subscriber = DualPixhawkAttitudeSubscriber(domain_id=0, left_namespace="uav_0", right_namespace="uav_1")
    subscriber.start()

    assert fake_node.topics == []


def test_estimate_distance_carries_extrinsic_sync_metadata() -> None:
    from wiris_stereo_distance.pixhawk_attitude import camera_extrinsic_from_quaternion

    left = detection("left", x1=500, y1=200, x2=540, y2=260, t=10.0)
    right = detection("right", x1=450, y1=202, x2=490, y2=262, t=10.1)
    left_ext = camera_extrinsic_from_quaternion(camera="left", namespace="uav_0", q=[1, 0, 0, 0], timestamp_s=20.0)
    right_ext = camera_extrinsic_from_quaternion(camera="right", namespace="uav_1", q=[1, 0, 0, 0], timestamp_s=20.2)

    estimate = estimate_distance(
        left,
        right,
        StereoCalibration(focal_length_px=900, baseline_m=0.3),
        left_extrinsic=left_ext,
        right_extrinsic=right_ext,
    )

    assert estimate is not None
    assert estimate.left_extrinsic is left_ext
    assert estimate.right_extrinsic is right_ext
    assert estimate.extrinsic_time_delta_s == 0.1999999999999993


def test_wiris_stereo_distance_source_is_independent_of_legacy_apps() -> None:
    from pathlib import Path

    forbidden = ("WIRISVisibleViewer", "PixhawkUorbConsole", "wiris_visible_viewer", "pixhawk_uorb_console")
    app_root = Path(__file__).resolve().parents[1]
    source_files = list((app_root / "wiris_stereo_distance").glob("*.py"))
    source_files.append(app_root / "main.py")

    offenders = []
    for path in source_files:
        text = path.read_text()
        for token in forbidden:
            if token in text:
                offenders.append(f"{path.name}:{token}")

    assert offenders == []


def test_zoom_command_template_rendering_and_validation() -> None:
    from wiris_stereo_distance.wiris_zoom import (
        DEFAULT_ZOOM_IN_TEMPLATE,
        DEFAULT_ZOOM_OUT_TEMPLATE,
        DEFAULT_ZOOM_SET_TEMPLATE,
        build_zoom_command,
        render_zoom_command,
    )

    assert DEFAULT_ZOOM_IN_TEMPLATE == "SZIN"
    assert DEFAULT_ZOOM_OUT_TEMPLATE == "SZOT"
    assert render_zoom_command(DEFAULT_ZOOM_SET_TEMPLATE, level=12) == "SZVN 12"
    command = build_zoom_command(host="10.0.0.231", port=2240, template=DEFAULT_ZOOM_IN_TEMPLATE)

    assert command.host == "10.0.0.231"
    assert command.port == 2240
    assert command.command == "SZIN"


def test_zoom_command_template_requires_level() -> None:
    from wiris_stereo_distance.wiris_zoom import render_zoom_command

    try:
        render_zoom_command("ZOOM {level}")
    except ValueError as exc:
        assert "level" in str(exc)
    else:
        raise AssertionError("expected ValueError")


def test_send_tcp_line_trims_wiris_null_terminated_response(monkeypatch) -> None:
    from wiris_stereo_distance.wiris_zoom import WirisZoomCommand, send_tcp_line

    sent_payloads = []

    class FakeSocket:
        def __enter__(self):
            return self

        def __exit__(self, exc_type, exc, tb):
            return False

        def settimeout(self, timeout):
            self.timeout = timeout

        def sendall(self, payload):
            sent_payloads.append(payload)

        def recv(self, _size):
            return b"OK\r\n\x00"

    monkeypatch.setattr("socket.create_connection", lambda *_args, **_kwargs: FakeSocket())

    response = send_tcp_line(WirisZoomCommand(host="10.0.0.231", port=2240, command="SZIN"))

    assert sent_payloads == [b"SZIN\n"]
    assert response == "OK"


def test_wiris_ethernet_mode_commands_are_control_protocol_commands() -> None:
    from wiris_stereo_distance.wiris_zoom import (
        GET_ETHERNET_MODE_COMMAND,
        REBOOT_COMMAND,
        SET_ETHERNET_MODE_COMMAND,
        SET_IP_ADDRESS_TEMPLATE,
        SET_SUBNET_MASK_TEMPLATE,
        build_wiris_command,
    )

    get_command = build_wiris_command(host="10.0.0.231", port=2240, command=GET_ETHERNET_MODE_COMMAND)
    set_command = build_wiris_command(host="10.0.0.231", port=2240, command=SET_ETHERNET_MODE_COMMAND)

    assert get_command.command == "GETH"
    assert set_command.command == "SETH TRUE"
    assert get_command.terminator == "\n"
    assert set_command.terminator == "\n"
    assert SET_IP_ADDRESS_TEMPLATE.format(address="10.0.0.231") == "STIP 10.0.0.231"
    assert SET_SUBNET_MASK_TEMPLATE.format(mask="255.255.255.0") == "STMS 255.255.255.0"
    assert REBOOT_COMMAND == "REBT"


def test_wiris_command_selector_shows_korean_description(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()

    index = widget.wiris_command_combo.findText("Basic / Check connection")
    assert index >= 0
    widget.wiris_command_combo.setCurrentIndex(index)

    assert widget.wiris_command_edit.text() == "HIWS"
    assert widget.wiris_command_terminator_combo.currentData() == gui.LINE_TERMINATOR
    assert "정상 응답은 OK" in widget.wiris_command_status_label.text()
    assert widget.wiris_command_status_label.wordWrap() is True
    assert widget.wiris_command_edit.toolTip() == widget.wiris_command_status_label.text()

    widget.deleteLater()
    _ = app


def test_enable_wiris_rtsp_uses_line_terminator_and_delayed_stream_check(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget.left_zoom_host_edit.setText("10.0.0.231")
    widget.right_zoom_host_edit.setText("10.0.0.232")
    monkeypatch.setattr(gui, "interface_has_address", lambda *_args, **_kwargs: True)
    monkeypatch.setattr(gui, "RTSP_ENABLE_SETTLE_DELAY_MS", 0)
    monkeypatch.setattr(gui, "RTSP_ENABLE_PORT_WAIT_MAX_ATTEMPTS", 0)
    sent = []

    def fake_send(command):
        sent.append((command.host, command.command, command.terminator))
        if command.command == gui.GET_ETHERNET_MODE_COMMAND:
            return "TRUE"
        return "OK"

    checked = []
    monkeypatch.setattr(gui, "send_tcp_line", fake_send)
    monkeypatch.setattr(gui, "check_tcp_port", lambda *_args, **_kwargs: True)
    widget.check_cameras = lambda: checked.append(True)

    widget.enable_wiris_rtsp()
    assert process_events_until(app, lambda: widget.rtsp_enable_thread is None)

    assert sent == [
        ("10.0.0.231", gui.GET_ETHERNET_MODE_COMMAND, gui.LINE_TERMINATOR),
        ("10.0.0.231", gui.SET_ETHERNET_MODE_COMMAND, gui.LINE_TERMINATOR),
        ("10.0.0.232", gui.GET_ETHERNET_MODE_COMMAND, gui.LINE_TERMINATOR),
        ("10.0.0.232", gui.SET_ETHERNET_MODE_COMMAND, gui.LINE_TERMINATOR),
    ]
    assert checked == [True]

    widget.deleteLater()
    _ = app


def test_enable_wiris_rtsp_auto_applies_camera_ip_before_commands(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget.interface_edit.setText("enx-test")
    widget.host_address_edit.setText("10.0.0.99/24")
    widget.left_zoom_host_edit.setText("10.0.0.231")
    widget.right_zoom_host_edit.setText("10.0.0.232")
    widget._save_current_settings = lambda: None
    monkeypatch.setattr(gui, "RTSP_ENABLE_SETTLE_DELAY_MS", 0)
    monkeypatch.setattr(gui, "RTSP_ENABLE_PORT_WAIT_MAX_ATTEMPTS", 0)

    calls = []
    monkeypatch.setattr(gui, "interface_link_exists", lambda *_args, **_kwargs: True)
    monkeypatch.setattr(gui, "interface_has_address", lambda *_args, **_kwargs: False)
    resolve_calls = []
    monkeypatch.setattr(
        gui,
        "resolve_nm_interface_address",
        lambda interface_name, host_hints: (
            resolve_calls.append((interface_name, tuple(host_hints))) or "10.0.0.10/24"
        ),
    )
    monkeypatch.setattr(
        gui,
        "apply_interface_address",
        lambda interface_name, address_text: (
            calls.append(("apply", interface_name, address_text)) or True,
            f"applied {address_text} to {interface_name}",
        ),
    )
    monkeypatch.setattr(gui, "interface_addresses", lambda interface_name: f"{interface_name} 10.0.0.10/24")

    sent = []

    def fake_send(command):
        calls.append(("send", command.host, command.command))
        sent.append((command.host, command.command, command.terminator))
        if command.command == gui.GET_ETHERNET_MODE_COMMAND:
            return "TRUE"
        return "OK"

    checked = []
    monkeypatch.setattr(gui, "send_tcp_line", fake_send)
    monkeypatch.setattr(gui, "check_tcp_port", lambda *_args, **_kwargs: True)
    widget.check_cameras = lambda: checked.append(True)

    widget.enable_wiris_rtsp()
    assert process_events_until(app, lambda: widget.rtsp_enable_thread is None)

    assert resolve_calls[0][0] == "enx-test"
    assert "10.0.0.231" in resolve_calls[0][1]
    assert calls[0] == ("apply", "enx-test", "10.0.0.10/24")
    assert sent == [
        ("10.0.0.231", gui.GET_ETHERNET_MODE_COMMAND, gui.LINE_TERMINATOR),
        ("10.0.0.231", gui.SET_ETHERNET_MODE_COMMAND, gui.LINE_TERMINATOR),
        ("10.0.0.232", gui.GET_ETHERNET_MODE_COMMAND, gui.LINE_TERMINATOR),
        ("10.0.0.232", gui.SET_ETHERNET_MODE_COMMAND, gui.LINE_TERMINATOR),
    ]
    assert checked == [True]

    widget.deleteLater()
    _ = app


def test_enable_wiris_rtsp_recovers_from_open_but_wrong_telnet_port(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    monkeypatch.setattr(gui, "interface_has_address", lambda *_args, **_kwargs: True)
    monkeypatch.setattr(gui, "check_tcp_port", lambda *_args, **_kwargs: True)
    monkeypatch.setattr(gui, "RTSP_ENABLE_SETTLE_DELAY_MS", 0)
    monkeypatch.setattr(gui, "RTSP_ENABLE_PORT_WAIT_MAX_ATTEMPTS", 0)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget.left_zoom_host_edit.setText("10.0.0.231")
    widget.right_zoom_host_edit.setText("10.0.0.232")
    widget.left_control_port_spin.setValue(2250)
    widget.right_control_port_spin.setValue(2250)
    saved = []
    monkeypatch.setattr(widget, "_save_current_settings", lambda *args, **kwargs: saved.append(True))
    sent = []

    def fake_send(command):
        sent.append((command.host, command.port, command.command))
        if command.command == gui.GET_ETHERNET_MODE_COMMAND and command.port == 2250:
            return "ERR"
        if command.command == gui.GET_ETHERNET_MODE_COMMAND and command.port == gui.DEFAULT_ZOOM_CONTROL_PORT:
            return "FALSE"
        if command.command == gui.SET_ETHERNET_MODE_COMMAND and command.port == gui.DEFAULT_ZOOM_CONTROL_PORT:
            return "OK"
        return "ERR"

    checked = []
    monkeypatch.setattr(gui, "send_tcp_line", fake_send)
    widget.check_cameras = lambda: checked.append(True)

    widget.enable_wiris_rtsp()
    assert process_events_until(app, lambda: widget.rtsp_enable_thread is None)

    assert widget.left_control_port_spin.value() == gui.DEFAULT_ZOOM_CONTROL_PORT
    assert widget.right_control_port_spin.value() == gui.DEFAULT_ZOOM_CONTROL_PORT
    assert saved == [True, True]
    assert ("10.0.0.231", 2250, gui.GET_ETHERNET_MODE_COMMAND) in sent
    assert ("10.0.0.231", gui.DEFAULT_ZOOM_CONTROL_PORT, gui.SET_ETHERNET_MODE_COMMAND) in sent
    assert ("10.0.0.232", 2250, gui.GET_ETHERNET_MODE_COMMAND) in sent
    assert ("10.0.0.232", gui.DEFAULT_ZOOM_CONTROL_PORT, gui.SET_ETHERNET_MODE_COMMAND) in sent
    assert "enable-sent" in widget.camera_connection_label.text()
    assert checked == [True]

    widget.deleteLater()
    _ = app


def test_enable_wiris_rtsp_rejects_err_response_without_stream_check(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    monkeypatch.setattr(gui, "interface_has_address", lambda *_args, **_kwargs: True)
    monkeypatch.setattr(gui, "RTSP_ENABLE_SETTLE_DELAY_MS", 0)
    monkeypatch.setattr(gui, "RTSP_ENABLE_PORT_WAIT_MAX_ATTEMPTS", 0)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget.left_zoom_host_edit.setText("10.0.0.231")
    widget.right_zoom_host_edit.setText("10.0.0.232")

    def fake_send(command):
        if command.command == gui.GET_ETHERNET_MODE_COMMAND:
            return "ERR"
        return "ERR"

    checked = []
    monkeypatch.setattr(gui, "send_tcp_line", fake_send)
    widget.check_cameras = lambda: checked.append(True)

    widget.enable_wiris_rtsp()
    assert process_events_until(app, lambda: widget.rtsp_enable_thread is None)

    assert "unsupported-port" in widget.camera_connection_label.text()
    assert checked == []

    widget.deleteLater()
    _ = app


def test_enable_wiris_rtsp_returns_before_slow_network_commands_finish(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    monkeypatch.setattr(gui, "interface_has_address", lambda *_args, **_kwargs: True)
    monkeypatch.setattr(gui, "check_tcp_port", lambda *_args, **_kwargs: True)
    monkeypatch.setattr(gui, "RTSP_ENABLE_SETTLE_DELAY_MS", 0)
    monkeypatch.setattr(gui, "RTSP_ENABLE_PORT_WAIT_MAX_ATTEMPTS", 0)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget.left_zoom_host_edit.setText("10.0.0.231")
    widget.right_zoom_host_edit.setText("10.0.0.232")

    def slow_send(command):
        time.sleep(0.25)
        return "TRUE" if command.command == gui.GET_ETHERNET_MODE_COMMAND else "OK"

    monkeypatch.setattr(gui, "send_tcp_line", slow_send)
    widget.check_cameras = lambda: None

    started_s = time.monotonic()
    widget.enable_wiris_rtsp()
    elapsed_s = time.monotonic() - started_s

    assert elapsed_s < 0.1
    assert widget.rtsp_enable_thread is not None
    assert process_events_until(app, lambda: widget.rtsp_enable_thread is None, timeout_s=2.0)

    widget.deleteLater()
    _ = app


def test_enable_wiris_rtsp_replaces_missing_camera_interface(monkeypatch) -> None:
    from ipaddress import ip_interface
    from types import SimpleNamespace

    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget.interface_edit.setText("enx-old")
    widget.host_address_edit.setText("10.0.0.10/24")
    widget.left_zoom_host_edit.setText("10.0.0.231")
    widget.right_zoom_host_edit.setText("10.0.0.232")
    widget._save_current_settings = lambda: None

    apply_calls = []
    monkeypatch.setattr(gui, "interface_link_exists", lambda interface_name: interface_name == "enx-new")
    monkeypatch.setattr(gui, "resolve_nm_interface_address", lambda *_args, **_kwargs: None)
    monkeypatch.setattr(gui, "interface_has_address", lambda *_args, **_kwargs: False)
    monkeypatch.setattr(
        gui,
        "discover_host_interface_candidates",
        lambda **_kwargs: [
            SimpleNamespace(
                interface=SimpleNamespace(name="enx-new", address=ip_interface("10.0.0.10/24")),
                neighbor_hosts=(),
            )
        ],
    )
    monkeypatch.setattr(
        gui,
        "apply_interface_address",
        lambda interface_name, address_text: (
            apply_calls.append((interface_name, address_text)) or True,
            f"applied {address_text} to {interface_name}",
        ),
    )
    monkeypatch.setattr(gui, "interface_addresses", lambda interface_name: f"{interface_name} 10.0.0.10/24")
    monkeypatch.setattr(
        gui,
        "send_tcp_line",
        lambda command: "TRUE" if command.command == gui.GET_ETHERNET_MODE_COMMAND else "OK",
    )
    monkeypatch.setattr(gui, "check_tcp_port", lambda *_args, **_kwargs: True)
    monkeypatch.setattr(gui, "RTSP_ENABLE_SETTLE_DELAY_MS", 0)
    monkeypatch.setattr(gui, "RTSP_ENABLE_PORT_WAIT_MAX_ATTEMPTS", 0)
    widget.check_cameras = lambda: None

    widget.enable_wiris_rtsp()
    assert process_events_until(app, lambda: widget.rtsp_enable_thread is None)

    assert widget.interface_edit.text() == "enx-new"
    assert apply_calls[0] == ("enx-new", "10.0.0.10/24")

    widget.deleteLater()
    _ = app


def test_discover_pc_camera_ip_uses_networkmanager_profile_without_applying(monkeypatch) -> None:
    from PyQt5 import QtWidgets
    from ipaddress import ip_interface

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget.interface_edit.setText("enx-test")
    widget.host_address_edit.setText("10.41.10.1/24")

    apply_calls = []
    monkeypatch.setattr(gui, "interface_link_exists", lambda *_args, **_kwargs: True)
    monkeypatch.setattr(gui, "resolve_nm_interface_address", lambda *_args, **_kwargs: ip_interface("10.0.0.10/24"))
    monkeypatch.setattr(
        gui,
        "apply_interface_address",
        lambda interface_name, address_text: (
            apply_calls.append((interface_name, address_text)) or True,
            f"applied {address_text} to {interface_name}",
        ),
    )

    widget.discover_pc_camera_ip()

    assert widget.host_address_edit.text() == "10.0.0.10/24"
    assert widget.camera_connection_label.text() == "camera host IP: found 10.0.0.10/24"
    assert apply_calls == []

    widget.deleteLater()
    _ = app


def test_dual_pixhawk_start_uses_actual_sensor_gps_topic_type_when_vehicle_type_exists(monkeypatch) -> None:
    from types import SimpleNamespace

    import wiris_stereo_distance.pixhawk_attitude as pixhawk_attitude
    from wiris_stereo_distance.pixhawk_attitude import DualPixhawkAttitudeSubscriber

    class FakeNode:
        def __init__(self) -> None:
            self.subscriptions = []

        def get_topic_names_and_types(self):
            return [("/uav_0/fmu/out/vehicle_gps_position", ["px4_msgs/msg/SensorGps"])]

        def create_subscription(self, msg_type, topic, callback, _qos):
            self.subscriptions.append((msg_type, topic, callback))
            return topic

    class FakeExecutor:
        def add_node(self, _node) -> None:
            return None

    fake_node = FakeNode()
    type_map = {
        "px4_msgs/msg/VehicleAttitude": "attitude-type",
        "px4_msgs/msg/SensorGps": "sensor-gps-type",
        "px4_msgs/msg/VehicleGpsPosition": "vehicle-gps-type",
    }

    monkeypatch.setattr(pixhawk_attitude, "get_message", lambda name: type_map[name])
    monkeypatch.setattr(pixhawk_attitude, "SingleThreadedExecutor", FakeExecutor)
    monkeypatch.setattr(
        pixhawk_attitude,
        "rclpy",
        SimpleNamespace(ok=lambda: True, create_node=lambda _name: fake_node),
    )

    subscriber = DualPixhawkAttitudeSubscriber(domain_id=0, left_namespace="uav_0", right_namespace="")
    subscriber.start()

    vehicle_gps_subscriptions = [sub for sub in fake_node.subscriptions if sub[1] == "/uav_0/fmu/out/vehicle_gps_position"]
    assert vehicle_gps_subscriptions[0][0] == "sensor-gps-type"


def test_mavlink_gps2_raw_maps_to_second_gps_sensor() -> None:
    from types import SimpleNamespace

    from wiris_stereo_distance.pixhawk_attitude import DualPixhawkMavlinkSubscriber

    class FakeMavlinkMessage(SimpleNamespace):
        def get_type(self):
            return "GPS2_RAW"

    subscriber = DualPixhawkMavlinkSubscriber(left_url="udpin:0.0.0.0:14550", right_url="")
    gps_positions = []
    gps_sensors = []
    subscriber.gps_changed.connect(lambda camera, gps: gps_positions.append((camera, gps)))
    subscriber.gps_sensor_changed.connect(lambda camera, gps: gps_sensors.append((camera, gps)))

    subscriber._handle_mavlink_message(
        "left",
        FakeMavlinkMessage(fix_type=6, satellites_visible=20, eph=3, epv=4, lat=370000001, lon=1270000001, alt=123000),
    )

    assert gps_positions[0][0] == "left_GPS2"
    assert gps_positions[0][1].topic == "mavlink/GPS2_RAW"
    assert gps_positions[0][1].latitude_deg == 37.0000001
    assert gps_sensors[0][0] == "left"
    assert gps_sensors[0][1].sensor_slot == "GPS2"
    assert gps_sensors[0][1].topic == "mavlink/GPS2_RAW"
    assert gps_sensors[0][1].eph_m == 0.03


def test_gps_sensor_labels_show_ros2_topic_and_mavlink_field(monkeypatch) -> None:
    import time

    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui
    from wiris_stereo_distance.pixhawk_attitude import GpsSensorState

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    now_s = time.monotonic()

    widget._update_gps_sensor(
        "left",
        GpsSensorState(
            camera="left",
            namespace="uav_0",
            sensor_slot="GPS1",
            timestamp_s=now_s,
            topic="/uav_0/fmu/out/vehicle_gps_position",
            device_id=42,
            fix_type=3,
            satellites_used=8,
            eph_m=2.5,
            epv_m=3.5,
            px4_timestamp_s=12.345,
        ),
    )
    widget._update_gps_sensor(
        "left",
        GpsSensorState(
            camera="left",
            namespace="uav_0",
            sensor_slot="GPS2",
            timestamp_s=now_s,
            topic="mavlink/GPS2_RAW",
            fix_type=6,
            satellites_used=18,
            eph_m=0.03,
            epv_m=0.04,
        ),
    )

    gps1_text = widget.f9p_slot_labels["left"]["GPS1"].text()
    gps2_text = widget.f9p_slot_labels["left"]["GPS2"].text()
    compare_text = widget.f9p_compare_labels["left"].text()

    assert "ROS2 /uav_0/fmu/out/vehicle_gps_position" in gps1_text
    assert "MAVLink GPS2_RAW" in gps2_text
    assert "px4_t=12.345s" in gps1_text
    assert "src=ROS2 /uav_0/fmu/out/vehicle_gps_position / MAVLink GPS2_RAW" in compare_text
    assert widget.f9p_slot_labels["left"]["GPS1"].wordWrap()

    widget.deleteLater()
    _ = app


def test_baseline_alignment_tab_collects_gps_and_computes_baseline(monkeypatch) -> None:
    import time

    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui
    from wiris_stereo_distance.pixhawk_attitude import GpsSensorState
    from wiris_stereo_distance.rtk_geometry import GpsPositionState

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    now_s = time.monotonic()

    widget._update_gps(
        "left",
        GpsPositionState(
            camera="left",
            namespace="uav_0",
            timestamp_s=now_s,
            topic="/uav_0/fmu/out/vehicle_gps_position",
            latitude_deg=37.0,
            longitude_deg=127.0,
            altitude_m=10.0,
            fix_type=3,
            satellites_used=8,
            eph_m=2.0,
            epv_m=3.0,
        ),
    )
    widget._update_gps(
        "right",
        GpsPositionState(
            camera="right",
            namespace="uav_1",
            timestamp_s=now_s,
            topic="/uav_1/fmu/out/vehicle_gps_position",
            latitude_deg=37.0,
            longitude_deg=127.000003,
            altitude_m=10.1,
            fix_type=3,
            satellites_used=9,
            eph_m=2.1,
            epv_m=3.1,
        ),
    )
    widget._update_gps_sensor(
        "left",
        GpsSensorState(
            camera="left",
            namespace="uav_0",
            sensor_slot="GPS1",
            timestamp_s=now_s,
            topic="mavlink/GPS_RAW_INT",
            fix_type=3,
            satellites_used=8,
            eph_m=2.0,
        ),
    )

    assert widget.alignment_tabs.tabText(widget.baseline_alignment_tab_index) == "GNSS / Baseline"
    assert [widget.baseline_tabs.tabText(index) for index in range(widget.baseline_tabs.count())] == [
        "GPS 인식",
        "ROS2 Topic / MAVLink Field",
        "Baseline 계산",
        "신뢰성 평가",
        "Baseline Profile",
    ]
    assert "3D_FIX" in widget.baseline_gps_card_labels["left"]["rtk"].text()
    assert "MAVLink GPS_RAW_INT" in widget.baseline_gps_card_labels["left"]["gps1"].text() or "인식" in widget.baseline_gps_card_labels["left"]["gps1"].text()
    assert widget.latest_baseline_vector is not None
    assert "m" in widget.baseline_calc_labels["length"].text()

    widget.deleteLater()
    _ = app


def test_baseline_alignment_sampling_updates_quality(monkeypatch) -> None:
    import time

    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui
    from wiris_stereo_distance.rtk_geometry import GpsPositionState

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    now_s = time.monotonic()

    widget.require_rtk_fixed_checkbox.setChecked(False)
    widget.start_baseline_sampling()
    widget._update_gps(
        "left",
        GpsPositionState("left", "/uav_0", now_s, "gps", 37.0, 127.0, 10.0, fix_type=6, eph_m=0.01, epv_m=0.02),
    )
    widget._update_gps(
        "right",
        GpsPositionState("right", "/uav_1", now_s, "gps", 37.0, 127.000003, 10.0, fix_type=6, eph_m=0.01, epv_m=0.02),
    )
    widget.stop_baseline_sampling()

    assert widget.latest_baseline_quality is not None
    assert "Quality:" in widget.baseline_quality_badge.text()

    widget.deleteLater()
    _ = app


def test_mavlink_gps_raw_yaw_emits_gnss_relative_heading() -> None:
    import math
    from types import SimpleNamespace

    from wiris_stereo_distance.pixhawk_attitude import DualPixhawkMavlinkSubscriber

    class FakeMavlinkMessage(SimpleNamespace):
        def get_type(self):
            return "GPS_RAW_INT"

    subscriber = DualPixhawkMavlinkSubscriber(left_url="udpin:0.0.0.0:14550", right_url="")
    gnss_samples = []
    subscriber.gnss_relative_changed.connect(lambda camera, state: gnss_samples.append((camera, state)))

    subscriber._handle_mavlink_message(
        "left",
        FakeMavlinkMessage(
            fix_type=6,
            satellites_visible=20,
            eph=3,
            epv=4,
            lat=370000001,
            lon=1270000001,
            alt=123000,
            yaw=9000,
            hdg_acc=250000,
        ),
    )

    assert gnss_samples[0][0] == "left"
    assert math.degrees(gnss_samples[0][1].heading_rad) == 90.0
    assert math.degrees(gnss_samples[0][1].heading_accuracy_rad) == 2.5
    assert gnss_samples[0][1].topic == "mavlink/GPS_RAW_INT"
    assert gnss_samples[0][1].carrier_solution_fixed is True
    assert gnss_samples[0][1].relative_position_valid is True
    assert gnss_samples[0][1].heading_valid is True


def test_mavlink_gps_rtk_maps_baseline_to_gnss_relative() -> None:
    import math
    from types import SimpleNamespace

    from wiris_stereo_distance.pixhawk_attitude import DualPixhawkMavlinkSubscriber

    class FakeMavlinkMessage(SimpleNamespace):
        def get_type(self):
            return "GPS2_RTK"

    subscriber = DualPixhawkMavlinkSubscriber(left_url="udpin:0.0.0.0:14550", right_url="")
    gnss_samples = []
    subscriber.gnss_relative_changed.connect(lambda camera, state: gnss_samples.append((camera, state)))

    subscriber._handle_mavlink_message(
        "left",
        FakeMavlinkMessage(
            rtk_receiver_id=2,
            baseline_coords_type=1,
            baseline_a_mm=1000,
            baseline_b_mm=1000,
            baseline_c_mm=0,
            accuracy=25,
        ),
    )

    assert gnss_samples[0][0] == "left"
    assert gnss_samples[0][1].device_id == 2
    assert gnss_samples[0][1].topic == "mavlink/GPS2_RTK"
    assert math.isclose(gnss_samples[0][1].position_length_m, math.sqrt(2), rel_tol=1e-9)
    assert gnss_samples[0][1].accuracy_length_m == 0.025
    assert math.degrees(gnss_samples[0][1].heading_rad) == 45.0
    assert gnss_samples[0][1].moving_base_mode is True
    assert gnss_samples[0][1].relative_position_valid is True


def test_mavlink_scaled_imu2_maps_to_can2_mag_sensor() -> None:
    from types import SimpleNamespace

    from wiris_stereo_distance.pixhawk_attitude import DualPixhawkMavlinkSubscriber

    class FakeMavlinkMessage(SimpleNamespace):
        def get_type(self):
            return "SCALED_IMU2"

    subscriber = DualPixhawkMavlinkSubscriber(left_url="udpin:0.0.0.0:14550", right_url="")
    mags = []
    subscriber.mag_sensor_changed.connect(lambda camera, mag: mags.append((camera, mag)))

    subscriber._handle_mavlink_message("left", FakeMavlinkMessage(xmag=3, ymag=4, zmag=12))

    assert mags[0][0] == "left"
    assert mags[0][1].sensor_slot == "CAN2"
    assert mags[0][1].topic == "mavlink/SCALED_IMU2"
    assert mags[0][1].magnitude == 13.0


def test_camera_parameters_tab_shows_intrinsics_and_live_extrinsics() -> None:
    import time

    from PyQt5 import QtCore, QtWidgets

    from wiris_stereo_distance.calibration import CameraIntrinsics, CheckerboardConfig, StereoIntrinsicsResult
    from wiris_stereo_distance.gui import WirisStereoDistancePanel
    from wiris_stereo_distance.pixhawk_attitude import CameraExtrinsicState

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = WirisStereoDistancePanel()
    widget.last_calibration_result = StereoIntrinsicsResult(
        left=CameraIntrinsics(
            width=1280,
            height=720,
            camera_matrix=[[900.0, 0.0, 640.0], [0.0, 901.0, 360.0], [0.0, 0.0, 1.0]],
            distortion_coefficients=[0.1, -0.2, 0.001, 0.002, 0.0],
            reprojection_error_px=0.42,
            image_count=18,
        ),
        right=CameraIntrinsics(
            width=1280,
            height=720,
            camera_matrix=[[910.0, 0.0, 641.0], [0.0, 911.0, 361.0], [0.0, 0.0, 1.0]],
            distortion_coefficients=[0.2, -0.1],
            reprojection_error_px=0.43,
            image_count=17,
        ),
        board=CheckerboardConfig(inner_cols=9, inner_rows=6, square_size_m=0.03),
    )
    widget.apply_intrinsics_to_distance()
    widget.use_rtk_baseline_checkbox.setChecked(False)
    widget.distance_mode_combo.setCurrentIndex(widget.distance_mode_combo.findData("pixhawk"))
    widget.latest_extrinsics["left"] = CameraExtrinsicState(
        camera="left",
        namespace="udpin:0.0.0.0:14550",
        roll_deg=1.0,
        pitch_deg=2.0,
        yaw_deg=3.0,
        timestamp_s=time.monotonic(),
        topic="mavlink/ATTITUDE",
        q=(1.0, 0.0, 0.0, 0.0),
    )

    widget._refresh_camera_parameters_tab()
    widget.last_calibration_zoom_level = widget.zoom_level_spin.value()
    widget._refresh_zoom_intrinsics_tab()
    widget.tabs.setCurrentIndex(widget.depth_tab_index)
    widget.measurement_tabs.setCurrentIndex(widget.stereo_settings_tab_index)
    widget.resize(1190, 820)
    widget.show()
    app.processEvents()

    tab_names = [widget.tabs.tabText(index) for index in range(widget.tabs.count())]
    assert "진단" not in tab_names
    assert tab_names == [
        "연결",
        "시간",
        "보정",
        "파라미터",
        "정렬",
        "YOLO",
        "측정",
        "저장·분석",
        "고급",
    ]
    assert [widget.setup_tabs.tabText(index) for index in range(widget.setup_tabs.count())] == [
        "WIRIS",
        "Pixhawk",
        "UAM",
    ]
    assert [widget.calibration_tabs.tabText(index) for index in range(widget.calibration_tabs.count())] == [
        "체커보드",
        "줌 내부 파라미터",
    ]
    assert [widget.alignment_tabs.tabText(index) for index in range(widget.alignment_tabs.count())] == [
        "개요",
        "GNSS / Baseline",
        "Magnetometer / Yaw",
        "Attitude",
        "Coordinate System",
        "Validation",
        "Alignment Profile",
    ]
    assert [widget.pixhawk_tabs.tabText(index) for index in range(widget.pixhawk_tabs.count())] == [
        "요약",
        "픽스호크 1",
        "픽스호크 2",
    ]
    assert [widget.advanced_tabs.tabText(index) for index in range(widget.advanced_tabs.count())] == [
        "WIRIS SDK",
        "필드 감시",
        "Bias 보정",
    ]
    assert [widget.parameter_tabs.tabText(index) for index in range(widget.parameter_tabs.count())] == [
        "내부",
        "외부/기하",
    ]
    assert [widget.measurement_tabs.tabText(index) for index in range(widget.measurement_tabs.count())] == [
        "실시간",
        "계산 설정",
        "삼각측량 품질",
    ]
    assert [widget.save_tabs.tabText(index) for index in range(widget.save_tabs.count())] == [
        "세션 저장",
        "UAM 비교",
        "기록 샘플",
    ]
    assert widget.advanced_tabs.tabText(widget.wiris_command_tab_index) == "WIRIS SDK"
    assert widget.advanced_tabs.tabText(widget.center_bias_tab_index) == "Bias 보정"
    assert widget.setup_tabs.tabText(widget.pixhawk_connection_tab_index) == "Pixhawk"
    assert widget.setup_tabs.tabText(widget.uam_connection_tab_index) == "UAM"
    assert widget.tabs.tabText(widget.camera_parameters_tab_index) == "파라미터"
    assert widget.parameter_tabs.tabText(widget.intrinsic_parameters_tab_index) == "내부"
    assert widget.parameter_tabs.tabText(widget.extrinsic_parameters_tab_index) == "외부/기하"
    assert widget.save_tabs.tabText(widget.save_session_tab_index) == "세션 저장"
    assert widget.save_tabs.tabText(widget.uam_analysis_tab_index) == "UAM 비교"
    assert widget.save_tabs.tabText(widget.recording_samples_tab_index) == "기록 샘플"
    assert widget.calibration_tabs.tabText(widget.checkerboard_calibration_tab_index) == "체커보드"
    assert widget.calibration_tabs.tabText(widget.zoom_intrinsics_tab_index) == "줌 내부 파라미터"
    assert widget.intrinsics_settings_group.isCheckable()
    assert widget.intrinsics_settings_group.isChecked()
    assert widget.extrinsics_settings_group.title() == "기본: 외부 파라미터 / Baseline"
    assert widget.extrinsics_settings_group.isVisible()
    assert isinstance(widget.distance_mode_combo, QtWidgets.QComboBox)
    assert widget.distance_mode_combo.currentText() == "Pixhawk 자세 + GPS/RTK baseline"
    assert widget.measurement_display_combo.currentText() == "계산 선택 프레임"
    assert widget.stereo_parameter_splitter.orientation() == QtCore.Qt.Horizontal
    assert widget.stereo_parameter_splitter.count() == 2
    sizes = widget.stereo_parameter_splitter.sizes()
    assert len(sizes) == 2
    assert sizes[0] > 0 and sizes[1] > 0
    assert abs(sizes[0] - sizes[1]) < max(sizes)
    assert widget.distance_mode_combo.itemText(widget.distance_mode_combo.findData("pixhawk")) == "Pixhawk 자세 + GPS/RTK baseline"
    assert widget.save_intrinsics_button.parentWidget().title() == "저장 / 적용"
    assert widget.apply_intrinsics_button.parentWidget().title() == "저장 / 적용"
    assert widget.apply_intrinsics_button.text() == "계산에 적용"
    assert widget.measurement_tabs.tabText(widget.stereo_settings_tab_index) == "계산 설정"
    assert "source: active calibration result" in widget.left_intrinsics_text.toPlainText()
    assert "fx: 900.000000" in widget.left_intrinsics_text.toPlainText()
    assert "distortion_coefficients" in widget.right_intrinsics_text.toPlainText()
    assert "source: MAVLink (mavlink/ATTITUDE)" in widget.left_extrinsics_text.toPlainText()
    assert "roll:  1.000000 deg" in widget.left_extrinsics_text.toPlainText()
    assert "intrinsics_source: active calibration result" in widget.stereo_geometry_text.toPlainText()
    assert "focal_source: calibrated intrinsics average fx" in widget.stereo_geometry_text.toPlainText()
    assert "baseline_source_detail: GPS/RTK baseline disabled" in widget.stereo_geometry_text.toPlainText()
    assert "distance_method: 3D ray fallback: Pixhawk 자세 대기 중" in widget.stereo_geometry_text.toPlainText()
    assert "use_3d_distance: true" in widget.stereo_geometry_text.toPlainText()
    assert "계산 파라미터:" in widget.distance_parameters_label.text()
    assert "baseline_used" in widget.stereo_geometry_text.toPlainText()
    assert "source: active calibration result for zoom" in widget.zoom_left_intrinsics_text.toPlainText()
    assert "source: active calibration result for zoom" in widget.zoom_right_intrinsics_text.toPlainText()

    widget.deleteLater()
    _ = app


def test_checkerboard_zoom_controls_are_above_calibration_and_collapsible(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget.resize(1190, 820)
    widget.show()
    widget.tabs.setCurrentIndex(widget.calibration_tab_index)
    widget.calibration_tabs.setCurrentIndex(widget.checkerboard_calibration_tab_index)
    app.processEvents()

    zoom_group = widget.findChild(QtWidgets.QGroupBox, "checkerboardZoomControlGroup")
    calibration_group = next(
        group for group in widget.findChildren(QtWidgets.QGroupBox) if group.title() == "체커보드 내부 파라미터 보정"
    )

    assert zoom_group is not None
    assert zoom_group.isCheckable()
    assert zoom_group.isChecked()
    assert zoom_group.geometry().top() < calibration_group.geometry().top()
    assert widget.zoom_in_button.isVisible()
    assert widget.calib_cols_spin.value() == 13
    assert widget.calib_rows_spin.value() == 8
    assert widget.calib_square_spin.value() == 40.0
    assert widget.calib_min_images_spin.value() == 20

    calib_form = calibration_group.layout()
    assert isinstance(calib_form, QtWidgets.QGridLayout)

    def form_position(control: QtWidgets.QWidget) -> tuple[int, int, int, int]:
        index = calib_form.indexOf(control)
        assert index >= 0
        return calib_form.getItemPosition(index)

    assert calib_form.rowCount() == 2
    assert form_position(widget.calib_cols_spin) == (0, 1, 1, 1)
    assert form_position(widget.calib_rows_spin) == (0, 3, 1, 1)
    assert form_position(widget.calib_square_spin) == (0, 5, 1, 1)
    assert form_position(widget.calib_min_images_spin) == (0, 7, 1, 1)
    assert form_position(widget.calib_dir_edit) == (1, 1, 1, 5)
    assert form_position(widget.capture_calibration_button) == (1, 6, 1, 1)
    assert form_position(widget.run_calibration_button) == (1, 7, 1, 1)
    assert widget.calibration_status_label.objectName() == "statusStrip"
    assert calib_form.indexOf(widget.calibration_status_label) == -1
    assert widget.calibration_status_label.geometry().top() > calibration_group.geometry().bottom()

    for column, label_text in ((0, "내부 열"), (2, "내부 행"), (4, "격자 크기"), (6, "최소 이미지")):
        label = calib_form.itemAtPosition(0, column).widget()
        assert label.text() == label_text
        assert label.property("compactFormLabel") is True
    dir_label = calib_form.itemAtPosition(1, 0).widget()
    assert dir_label.text() == "저장 경로"
    assert dir_label.property("compactFormLabel") is True
    for field in (
        widget.calib_cols_spin,
        widget.calib_rows_spin,
        widget.calib_square_spin,
        widget.calib_min_images_spin,
        widget.calib_dir_edit,
    ):
        assert field.property("compactFormField") is True

    zoom_group.setChecked(False)
    app.processEvents()

    assert not widget.zoom_in_button.isVisible()
    assert zoom_group.maximumHeight() == 34

    zoom_group.setChecked(True)
    app.processEvents()

    assert widget.zoom_in_button.isVisible()
    assert zoom_group.maximumHeight() > 1000
    template_group = widget.findChild(QtWidgets.QGroupBox, "zoomCommandTemplateGroup")
    assert template_group is None
    assert not hasattr(widget, "zoom_in_template_edit")
    assert not hasattr(widget, "zoom_out_template_edit")
    assert not hasattr(widget, "zoom_stop_template_edit")
    assert not hasattr(widget, "zoom_set_template_edit")

    widget.deleteLater()
    _ = app


def test_checkerboard_preview_refresh_queues_detection_worker(monkeypatch) -> None:
    from PyQt5 import QtGui, QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget._stop_calibration_preview_worker()
    widget.tabs.setCurrentIndex(widget.calibration_tab_index)
    widget.calibration_tabs.setCurrentIndex(widget.checkerboard_calibration_tab_index)
    queued = []

    def queue_detection(camera, image):
        queued.append((camera, image.size()))
        return len(queued)

    monkeypatch.setattr(widget, "_queue_calibration_preview_detection", queue_detection)
    image = QtGui.QImage(80, 60, QtGui.QImage.Format_RGB888)
    image.fill(QtGui.QColor("black"))
    widget.latest_calibration_preview_images["left"] = image

    widget._refresh_calibration_preview("left")

    assert queued == [("left", image.size())]
    assert widget.calibration_left_preview.pixmap() is not None

    widget.deleteLater()
    _ = app


def test_calibration_capture_saves_only_detected_checkerboard_side(tmp_path, monkeypatch) -> None:
    from PyQt5 import QtGui, QtWidgets

    import wiris_stereo_distance.gui as gui
    from wiris_stereo_distance.calibration import CheckerboardDetection

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget._stop_calibration_preview_worker()
    widget.calib_dir_edit.setText(str(tmp_path / "calibration_captures"))
    image = QtGui.QImage(80, 60, QtGui.QImage.Format_RGB888)
    image.fill(QtGui.QColor("black"))
    widget.left_pane.latest_image = image
    widget.right_pane.latest_image = image

    monkeypatch.setattr(gui, "opencv_available", lambda: (True, None))

    def detect(path, _config):
        found = path.name.startswith("left_")
        return CheckerboardDetection(found=found, corner_count=104 if found else 0)

    monkeypatch.setattr(gui, "detect_checkerboard", detect)

    widget.capture_calibration_images()

    root = tmp_path / "calibration_captures"
    assert (root / "left" / "left_001.png").exists()
    assert not (root / "right" / "right_001.png").exists()
    assert widget.capture_calibration_button.text() == "캡처"
    assert "left=OK saved" in widget.calibration_status_label.text()
    assert "right=not found" in widget.calibration_status_label.text()

    widget.deleteLater()
    _ = app


def test_camera_parameters_tab_shows_sources_without_pixhawk() -> None:
    from PyQt5 import QtWidgets

    from wiris_stereo_distance.calibration import CameraIntrinsics, CheckerboardConfig, StereoIntrinsicsResult
    from wiris_stereo_distance.gui import WirisStereoDistancePanel

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = WirisStereoDistancePanel()
    widget.last_calibration_result = StereoIntrinsicsResult(
        left=CameraIntrinsics(
            width=1280,
            height=720,
            camera_matrix=[[900.0, 0.0, 640.0], [0.0, 901.0, 360.0], [0.0, 0.0, 1.0]],
            distortion_coefficients=[0.1, -0.2, 0.001, 0.002, 0.0],
            reprojection_error_px=0.42,
            image_count=18,
        ),
        right=CameraIntrinsics(
            width=1280,
            height=720,
            camera_matrix=[[910.0, 0.0, 641.0], [0.0, 911.0, 361.0], [0.0, 0.0, 1.0]],
            distortion_coefficients=[0.2, -0.1],
            reprojection_error_px=0.43,
            image_count=17,
        ),
        board=CheckerboardConfig(inner_cols=9, inner_rows=6, square_size_m=0.03),
    )
    widget.apply_intrinsics_to_distance()
    widget._refresh_camera_parameters_tab()

    assert "source: active calibration result" in widget.left_intrinsics_text.toPlainText()
    assert "source: none (Pixhawk disconnected or waiting)" in widget.left_extrinsics_text.toPlainText()
    assert "source: none (Pixhawk disconnected or waiting)" in widget.right_extrinsics_text.toPlainText()
    assert "intrinsics_source: active calibration result" in widget.stereo_geometry_text.toPlainText()
    assert "focal_source: calibrated intrinsics average fx" in widget.stereo_geometry_text.toPlainText()
    assert "baseline_source_detail: waiting for live GPS/RTK baseline" in widget.stereo_geometry_text.toPlainText()
    assert "distance_method: 3D ray fallback: Pixhawk 자세 대기 중" in widget.stereo_geometry_text.toPlainText()

    widget.deleteLater()
    _ = app


def test_pixhawk_rtk_can_drive_triangulation() -> None:
    from PyQt5 import QtWidgets

    from wiris_stereo_distance.calibration import CameraIntrinsics, CheckerboardConfig, StereoIntrinsicsResult
    from wiris_stereo_distance.gui import WirisStereoDistancePanel
    from wiris_stereo_distance.pixhawk_attitude import CameraExtrinsicState
    from wiris_stereo_distance.rtk_geometry import GpsPositionState, RtkBaselineEstimate

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = WirisStereoDistancePanel()
    intrinsics = CameraIntrinsics(
        width=1280,
        height=720,
        camera_matrix=[[900.0, 0.0, 640.0], [0.0, 900.0, 360.0], [0.0, 0.0, 1.0]],
        distortion_coefficients=[0.0],
        reprojection_error_px=0.2,
        image_count=20,
    )
    widget.last_calibration_result = StereoIntrinsicsResult(
        left=intrinsics,
        right=intrinsics,
        board=CheckerboardConfig(inner_cols=9, inner_rows=6, square_size_m=0.03),
    )
    widget.latest_rtk_baseline = RtkBaselineEstimate(
        baseline_m=0.3,
        north_m=0.0,
        east_m=0.3,
        down_m=0.0,
        left_age_s=0.0,
        right_age_s=0.0,
        fix_quality="GOOD",
    )
    widget.latest_gps["left"] = GpsPositionState("left", "/uav_0", 10.0, "gps", 37.0, 127.0, 10.0, fix_type=6)
    widget.latest_gps["right"] = GpsPositionState("right", "/uav_1", 10.0, "gps", 37.0, 127.000003, 10.0, fix_type=6)
    for camera in ("left", "right"):
        widget._update_extrinsic(
            camera,
            CameraExtrinsicState(
                camera=camera,
                namespace=f"/uav_{0 if camera == 'left' else 1}",
                roll_deg=0.0,
                pitch_deg=0.0,
                yaw_deg=0.0,
                timestamp_s=10.0,
                topic="mavlink/ATTITUDE",
                q=(1.0, 0.0, 0.0, 0.0),
            ),
        )
    widget.apply_intrinsics_to_distance()
    widget.distance_mode_combo.setCurrentIndex(widget.distance_mode_combo.findData("pixhawk"))
    widget._update_detection("left", detection("left", x1=690, y1=340, x2=710, y2=380, t=10.0))
    widget._update_detection("right", detection("right", x1=640, y1=340, x2=660, y2=380, t=10.0))

    assert "triangulated-pixhawk" in widget.distance_label.text()
    assert "baseline=gps-rtk:0.300m" in widget.distance_label.text()
    assert "gap:" in widget.triangulation_quality_gap_label.text()
    assert "mode: triangulated-pixhawk" in widget.triangulation_quality_mode_label.text()
    assert "Raw center 3D point:" in widget.triangulation_quality_raw_point_label.text()
    assert "3D ray 거리: Pixhawk 자세 + GPS/RTK baseline 사용" in widget.stereo_geometry_text.toPlainText()
    assert "위치/자세 출처=Pixhawk 자세 + GPS/RTK baseline" in widget.distance_parameters_label.text()

    widget.deleteLater()
    _ = app


def test_3d_mode_waits_instead_of_recording_disparity_fallback() -> None:
    from PyQt5 import QtWidgets

    from wiris_stereo_distance.calibration import CameraIntrinsics, CheckerboardConfig, StereoIntrinsicsResult
    from wiris_stereo_distance.gui import WirisStereoDistancePanel

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = WirisStereoDistancePanel()
    intrinsics = CameraIntrinsics(
        width=1280,
        height=720,
        camera_matrix=[[900.0, 0.0, 640.0], [0.0, 900.0, 360.0], [0.0, 0.0, 1.0]],
        distortion_coefficients=[0.0],
        reprojection_error_px=0.2,
        image_count=20,
    )
    widget.last_calibration_result = StereoIntrinsicsResult(
        left=intrinsics,
        right=intrinsics,
        board=CheckerboardConfig(inner_cols=9, inner_rows=6, square_size_m=0.03),
    )
    widget.apply_intrinsics_to_distance()
    widget.distance_mode_combo.setCurrentIndex(widget.distance_mode_combo.findData("pixhawk"))
    widget._update_detection("left", detection("left", x1=690, y1=340, x2=710, y2=380, t=10.0))
    widget._update_detection("right", detection("right", x1=640, y1=340, x2=660, y2=380, t=10.0))

    assert "거리: 3D ray 기하 대기 중" in widget.distance_label.text()
    assert "disparity" not in widget.distance_label.text()
    assert list(widget.distance_history) == []

    widget.deleteLater()
    _ = app


def test_no_detection_event_does_not_clear_recent_measurement_match(monkeypatch) -> None:
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")

    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    from wiris_stereo_distance.calibration import CameraIntrinsics, CheckerboardConfig, StereoIntrinsicsResult
    from wiris_stereo_distance.pixhawk_attitude import CameraExtrinsicState
    from wiris_stereo_distance.rtk_geometry import RtkBaselineEstimate

    intrinsics = CameraIntrinsics(
        width=1280,
        height=720,
        camera_matrix=[[900.0, 0.0, 640.0], [0.0, 900.0, 360.0], [0.0, 0.0, 1.0]],
        distortion_coefficients=[0.0],
        reprojection_error_px=0.2,
        image_count=20,
    )
    widget.last_calibration_result = StereoIntrinsicsResult(
        left=intrinsics,
        right=intrinsics,
        board=CheckerboardConfig(inner_cols=9, inner_rows=6, square_size_m=0.03),
    )
    widget.latest_rtk_baseline = RtkBaselineEstimate(
        baseline_m=0.3,
        north_m=0.0,
        east_m=0.3,
        down_m=0.0,
        left_age_s=0.0,
        right_age_s=0.0,
        fix_quality="GOOD",
    )
    for camera in ("left", "right"):
        widget._update_extrinsic(
            camera,
            CameraExtrinsicState(
                camera=camera,
                namespace=f"/uav_{0 if camera == 'left' else 1}",
                roll_deg=0.0,
                pitch_deg=0.0,
                yaw_deg=0.0,
                timestamp_s=10.01,
                topic="mavlink/ATTITUDE",
                q=(1.0, 0.0, 0.0, 0.0),
            ),
        )
    widget.apply_intrinsics_to_distance()
    widget.distance_mode_combo.setCurrentIndex(widget.distance_mode_combo.findData("pixhawk"))

    widget._update_detection("left", detection("left", x1=690, y1=340, x2=710, y2=380, t=10.0))
    widget._update_detection("left", None)
    widget._update_detection("right", detection("right", x1=640, y1=340, x2=660, y2=380, t=10.02))

    assert "거리:" in widget.distance_label.text()
    assert "m (" in widget.distance_label.text()
    assert len(widget.detection_buffers["left"]) == 1

    widget.deleteLater()
    _ = app


def test_removed_disparity_mode_is_not_available(monkeypatch) -> None:
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")

    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    assert widget.distance_mode_combo.findData("disparity") == -1

    widget._update_detection("left", detection("left", x1=690, y1=340, x2=710, y2=380, t=10.0))
    widget._update_detection("right", detection("right", x1=640, y1=340, x2=660, y2=380, t=10.0))

    assert "거리: 3D ray 기하 대기 중" in widget.distance_label.text()
    assert list(widget.distance_history) == []

    widget.deleteLater()
    _ = app


def test_measurement_settings_show_only_selected_parameter_source_controls(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget.show()
    widget.tabs.setCurrentIndex(widget.depth_tab_index)
    widget.measurement_tabs.setCurrentIndex(widget.stereo_settings_tab_index)
    app.processEvents()

    widget.intrinsics_source_combo.setCurrentIndex(widget.intrinsics_source_combo.findData("checkerboard"))
    widget.distance_mode_combo.setCurrentIndex(widget.distance_mode_combo.findData("pixhawk"))
    app.processEvents()

    assert not widget.focal_spin.isVisible()
    assert not widget.baseline_spin.isVisible()
    assert not hasattr(widget, "max_attitude_age_spin")
    assert "체커보드 보정값" in widget.intrinsics_status_label.text()

    widget.intrinsics_source_combo.setCurrentIndex(widget.intrinsics_source_combo.findData("manual"))
    widget.distance_mode_combo.setCurrentIndex(widget.distance_mode_combo.findData("pixhawk"))
    app.processEvents()

    assert widget.focal_spin.isVisible()
    assert not widget.baseline_spin.isVisible()
    assert not hasattr(widget, "max_attitude_age_spin")
    assert widget.manual_focal_checkbox.isChecked()

    widget.deleteLater()
    _ = app


def test_manual_gps_baseline_calculator_is_removed(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    from wiris_stereo_distance.gui import WirisStereoDistancePanel

    monkeypatch.setattr(WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = WirisStereoDistancePanel()

    assert not widget.baseline_spin.isVisible()
    assert widget.gps_baseline_spins == {}
    assert not hasattr(widget, "gps_baseline_result_label")
    assert not hasattr(widget, "calculate_gps_baseline")
    assert not hasattr(widget, "apply_gps_baseline_to_manual_input")

    widget.deleteLater()
    _ = app


def test_wiris_pc_time_uses_one_shared_stus_timestamp_for_both_cameras(monkeypatch) -> None:
    import threading

    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget.wiris_command_target_combo.setCurrentIndex(widget.wiris_command_target_combo.findData("both"))
    sent_commands = []
    lock = threading.Lock()

    def fake_send_tcp_line(command):
        with lock:
            sent_commands.append(command.command)
        return ""

    monkeypatch.setattr(gui, "send_tcp_line", fake_send_tcp_line)

    widget.set_selected_wiris_time_from_pc()

    assert len(sent_commands) == 2
    assert sent_commands[0] == sent_commands[1]
    assert sent_commands[0].startswith("STUS ")
    assert "명령 UTC 차이" in widget.wiris_command_status_label.text()
    assert "0.000 ms" in widget.wiris_command_status_label.text()

    widget.deleteLater()
    _ = app


def test_intrinsics_save_writes_zoom_specific_and_latest_alias(tmp_path, monkeypatch) -> None:
    from PyQt5 import QtWidgets

    from wiris_stereo_distance.calibration import CameraIntrinsics, CheckerboardConfig, StereoIntrinsicsResult, load_intrinsics_json
    import wiris_stereo_distance.gui as gui

    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    monkeypatch.setattr(gui, "save_settings", lambda settings: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget.calib_dir_edit.setText(str(tmp_path / "calibration_captures"))
    widget.zoom_level_spin.setValue(12)
    widget.last_calibration_result = StereoIntrinsicsResult(
        left=CameraIntrinsics(
            width=1280,
            height=720,
            camera_matrix=[[900.0, 0.0, 640.0], [0.0, 901.0, 360.0], [0.0, 0.0, 1.0]],
            distortion_coefficients=[0.1, -0.2, 0.001, 0.002, 0.0],
            reprojection_error_px=0.42,
            image_count=18,
        ),
        right=CameraIntrinsics(
            width=1280,
            height=720,
            camera_matrix=[[910.0, 0.0, 641.0], [0.0, 911.0, 361.0], [0.0, 0.0, 1.0]],
            distortion_coefficients=[0.2, -0.1],
            reprojection_error_px=0.43,
            image_count=17,
        ),
        board=CheckerboardConfig(inner_cols=9, inner_rows=6, square_size_m=0.03),
    )

    widget.save_intrinsics()

    root = tmp_path / "calibration_captures"
    zoom_path = root / "intrinsics_zoom_12.json"
    latest_path = root / "intrinsics.json"
    assert zoom_path.exists()
    assert latest_path.exists()
    assert "zoom 12" in widget.calibration_status_label.text().lower()

    zoom_intrinsics = load_intrinsics_json(zoom_path)
    latest_intrinsics = load_intrinsics_json(latest_path)
    assert zoom_intrinsics.left.camera_matrix[0][0] == 900.0
    assert latest_intrinsics.right.camera_matrix[0][0] == 910.0
    assert widget.zoom_intrinsics_status_label.text().startswith("줌 12")
    assert "source: zoom 12 file" in widget.zoom_left_intrinsics_text.toPlainText()
    assert widget.zoom_intrinsics_compare_list.count() == 1
    assert str(zoom_path) in widget.zoom_intrinsics_compare_list.item(0).text()
    assert "Left  1280x720 fx=900.000" in widget.zoom_intrinsics_compare_list.item(0).text()

    widget.deleteLater()
    _ = app


def test_zoom_intrinsics_comparison_list_add_remove_and_drag_mode(tmp_path, monkeypatch) -> None:
    from PyQt5 import QtWidgets

    from wiris_stereo_distance.calibration import CameraIntrinsics, CheckerboardConfig, StereoIntrinsicsResult, save_intrinsics_json
    import wiris_stereo_distance.gui as gui

    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    monkeypatch.setattr(gui, "save_settings", lambda settings: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    root = tmp_path / "calibration_captures"
    root.mkdir(parents=True, exist_ok=True)
    first = root / "intrinsics_zoom_12_old.json"
    second = root / "intrinsics_zoom_12_new.json"
    for path, fx in ((first, 880.0), (second, 900.0)):
        save_intrinsics_json(
            path,
            StereoIntrinsicsResult(
                left=CameraIntrinsics(
                    width=1280,
                    height=720,
                    camera_matrix=[[fx, 0.0, 640.0], [0.0, fx + 1.0, 360.0], [0.0, 0.0, 1.0]],
                    distortion_coefficients=[0.0],
                    reprojection_error_px=0.4,
                    image_count=10,
                ),
                right=CameraIntrinsics(
                    width=1280,
                    height=720,
                    camera_matrix=[[fx + 10.0, 0.0, 641.0], [0.0, fx + 11.0, 361.0], [0.0, 0.0, 1.0]],
                    distortion_coefficients=[0.0],
                    reprojection_error_px=0.5,
                    image_count=11,
                ),
                board=CheckerboardConfig(inner_cols=9, inner_rows=6, square_size_m=0.03),
            ),
            metadata={"zoom_level": 12, "timestamp_s": 1000.0},
        )

    monkeypatch.setattr(
        QtWidgets.QFileDialog,
        "getOpenFileNames",
        lambda *_args, **_kwargs: ([str(first), str(second)], "Intrinsics JSON (*.json)"),
    )

    widget.add_zoom_intrinsics_comparison_files()

    assert widget.zoom_intrinsics_compare_list.count() == 2
    assert widget.zoom_intrinsics_compare_list.dragDropMode() == QtWidgets.QAbstractItemView.InternalMove
    assert "fx=880.000" in widget.zoom_intrinsics_compare_list.item(0).text()
    assert "fx=900.000" in widget.zoom_intrinsics_compare_list.item(1).text()

    widget.zoom_intrinsics_compare_list.clearSelection()
    widget.zoom_intrinsics_compare_list.item(0).setSelected(True)
    widget.remove_selected_zoom_intrinsics_comparison_files()

    assert widget.zoom_intrinsics_compare_list.count() == 1
    assert "fx=900.000" in widget.zoom_intrinsics_compare_list.item(0).text()
    assert first.exists()

    widget.deleteLater()
    _ = app


def test_zoom_intrinsics_comparison_selection_applies_to_distance(tmp_path, monkeypatch) -> None:
    from PyQt5 import QtWidgets

    from wiris_stereo_distance.calibration import CameraIntrinsics, CheckerboardConfig, StereoIntrinsicsResult, save_intrinsics_json
    import wiris_stereo_distance.gui as gui

    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    monkeypatch.setattr(gui, "save_settings", lambda settings: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    root = tmp_path / "calibration_captures"
    root.mkdir(parents=True, exist_ok=True)
    selected_path = root / "intrinsics_zoom_12_candidate.json"
    save_intrinsics_json(
        selected_path,
        StereoIntrinsicsResult(
            left=CameraIntrinsics(
                width=1280,
                height=720,
                camera_matrix=[[1000.0, 0.0, 640.0], [0.0, 1001.0, 360.0], [0.0, 0.0, 1.0]],
                distortion_coefficients=[0.0],
                reprojection_error_px=0.3,
                image_count=20,
            ),
            right=CameraIntrinsics(
                width=1280,
                height=720,
                camera_matrix=[[1020.0, 0.0, 641.0], [0.0, 1021.0, 361.0], [0.0, 0.0, 1.0]],
                distortion_coefficients=[0.0],
                reprojection_error_px=0.31,
                image_count=20,
            ),
            board=CheckerboardConfig(inner_cols=9, inner_rows=6, square_size_m=0.03),
        ),
        metadata={"zoom_level": 12, "timestamp_s": 1000.0},
    )
    widget._add_zoom_intrinsics_comparison_path(selected_path)
    widget.zoom_intrinsics_compare_list.setCurrentRow(0)

    widget.apply_selected_zoom_intrinsics_comparison_file()

    assert widget.active_intrinsics is not None
    assert widget.active_intrinsics.left.camera_matrix[0][0] == 1000.0
    assert abs(widget.focal_spin.value() - 1010.0) < 1e-6
    assert widget.last_calibration_zoom_level == 12
    assert "Applied comparison intrinsics" in widget.calibration_status_label.text()

    widget.deleteLater()
    _ = app


def test_intrinsics_load_prefers_current_zoom_file(tmp_path, monkeypatch) -> None:
    from PyQt5 import QtWidgets

    from wiris_stereo_distance.calibration import CameraIntrinsics, CheckerboardConfig, StereoIntrinsicsResult, save_intrinsics_json
    import wiris_stereo_distance.gui as gui

    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    monkeypatch.setattr(gui, "save_settings", lambda settings: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget.calib_dir_edit.setText(str(tmp_path / "calibration_captures"))
    widget.zoom_level_spin.setValue(12)

    root = tmp_path / "calibration_captures"
    root.mkdir(parents=True, exist_ok=True)
    save_intrinsics_json(
        root / "intrinsics.json",
        StereoIntrinsicsResult(
            left=CameraIntrinsics(
                width=1280,
                height=720,
                camera_matrix=[[800.0, 0.0, 640.0], [0.0, 801.0, 360.0], [0.0, 0.0, 1.0]],
                distortion_coefficients=[0.0],
                reprojection_error_px=0.50,
                image_count=16,
            ),
            right=CameraIntrinsics(
                width=1280,
                height=720,
                camera_matrix=[[810.0, 0.0, 641.0], [0.0, 811.0, 361.0], [0.0, 0.0, 1.0]],
                distortion_coefficients=[0.0],
                reprojection_error_px=0.51,
                image_count=16,
            ),
            board=CheckerboardConfig(inner_cols=9, inner_rows=6, square_size_m=0.03),
        ),
    )
    save_intrinsics_json(
        root / "intrinsics_zoom_12.json",
        StereoIntrinsicsResult(
            left=CameraIntrinsics(
                width=1280,
                height=720,
                camera_matrix=[[900.0, 0.0, 640.0], [0.0, 901.0, 360.0], [0.0, 0.0, 1.0]],
                distortion_coefficients=[0.0],
                reprojection_error_px=0.42,
                image_count=18,
            ),
            right=CameraIntrinsics(
                width=1280,
                height=720,
                camera_matrix=[[910.0, 0.0, 641.0], [0.0, 911.0, 361.0], [0.0, 0.0, 1.0]],
                distortion_coefficients=[0.0],
                reprojection_error_px=0.43,
                image_count=18,
            ),
            board=CheckerboardConfig(inner_cols=9, inner_rows=6, square_size_m=0.03),
        ),
    )

    widget.load_intrinsics()

    assert abs(widget.focal_spin.value() - 905.0) < 1e-6
    assert "zoom 12" in widget.calibration_status_label.text().lower()
    assert widget.zoom_intrinsics_status_label.text().startswith("줌 12")
    assert "source: zoom 12 file" in widget.zoom_left_intrinsics_text.toPlainText()

    widget.deleteLater()
    _ = app


def test_start_yolo_switches_to_depth_tab_after_camera_parameters_tab(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    from wiris_stereo_distance.gui import WirisStereoDistancePanel
    import wiris_stereo_distance.gui as gui

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = WirisStereoDistancePanel()
    monkeypatch.setattr(gui, "interface_has_address", lambda *_args, **_kwargs: True)
    monkeypatch.setattr(widget.left_pane, "start_stream", lambda *args, **kwargs: None)
    monkeypatch.setattr(widget.right_pane, "start_stream", lambda *args, **kwargs: None)
    widget.left_url_edit.setText("rtsp://10.0.0.231:8554/visible")
    widget.right_url_edit.setText("rtsp://10.0.0.232:8554/visible")

    widget.start_yolo()

    assert widget.tabs.tabText(widget.depth_tab_index) == "측정"
    assert widget.tabs.currentIndex() == widget.depth_tab_index

    widget.deleteLater()
    _ = app


def test_start_yolo_blocks_identical_left_right_rtsp_urls(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    from wiris_stereo_distance.gui import WirisStereoDistancePanel
    import wiris_stereo_distance.gui as gui

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = WirisStereoDistancePanel()
    monkeypatch.setattr(gui, "interface_has_address", lambda *_args, **_kwargs: True)
    started = []
    monkeypatch.setattr(widget.left_pane, "start_stream", lambda *args, **kwargs: started.append("left"))
    monkeypatch.setattr(widget.right_pane, "start_stream", lambda *args, **kwargs: started.append("right"))
    widget.left_url_edit.setText("rtsp://10.0.0.232:8554/visible")
    widget.right_url_edit.setText("rtsp://10.0.0.232:8554/visible")

    widget.start_yolo()

    assert started == []
    assert widget.yolo_enabled is False
    assert "동일" in widget.distance_label.text()

    widget.deleteLater()
    _ = app


def test_measurement_tab_hides_rtp_rtcp_timing_readouts_but_keeps_record_metadata() -> None:
    import time

    from PyQt5 import QtGui, QtWidgets

    from wiris_stereo_distance.gui import WirisStereoDistancePanel

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = WirisStereoDistancePanel()
    image = QtGui.QImage(16, 9, QtGui.QImage.Format_RGB888)
    base_s = time.monotonic()

    widget._update_calibration_preview(
        "left",
        image,
        {
            "timestamp_s": base_s,
            "source_timestamp_s": 10.000,
            "timestamp_source": "gstreamer-pts",
            "rtp_timestamp": 90_000,
            "rtp_ssrc": 11,
            "rtcp_frame_ntp_unix_s": 1_700_000_000.100,
            "rtcp_sr_rtp_timestamp": 45_000,
            "rtcp_sr_age_s": 0.25,
        },
    )
    widget._update_calibration_preview(
        "right",
        image,
        {
            "timestamp_s": base_s + 0.015,
            "source_timestamp_s": 10.012,
            "timestamp_source": "gstreamer-pts",
            "rtp_timestamp": 91_800,
            "rtp_ssrc": 22,
            "rtcp_frame_ntp_unix_s": 1_700_000_000.118,
            "rtcp_sr_rtp_timestamp": 46_800,
            "rtcp_sr_age_s": 0.50,
        },
    )

    assert not hasattr(widget, "left_frame_rtp_label")
    assert not hasattr(widget, "right_frame_rtp_label")
    assert not hasattr(widget, "frame_rtp_delta_label")
    assert not hasattr(widget, "left_frame_timestamp_label")
    assert not hasattr(widget, "frame_timestamp_delta_label")

    row = widget._record_frame_timestamp_parameters()
    assert row["left_rtp_timestamp"] == "90000"
    assert row["left_rtp_ssrc"] == "11"
    assert row["left_frame_timestamp_reference"] == "rtcp-ntp"
    assert row["left_rtcp_sr_rtp_timestamp"] == "45000"
    assert row["right_rtp_timestamp"] == "91800"
    assert row["right_frame_timestamp_reference"] == "rtcp-ntp"
    assert row["rtcp_frame_ntp_delta_s"] == "0.018000"
    assert row["camera_rtc_frame_delta_s"] == "0.018000"
    assert row["left_camera_rtc_frame_unix_s"] == "1700000000.100000"
    assert row["right_camera_rtc_basis"] == "rtcp-sender-report"

    widget.deleteLater()
    _ = app


def test_measurement_tab_runtime_stats_tooltips_explain_yolo_hz() -> None:
    from PyQt5 import QtWidgets

    from wiris_stereo_distance.gui import WirisStereoDistancePanel

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = WirisStereoDistancePanel()

    tooltip = widget.left_runtime_stats_label.toolTip()

    assert "해상도/Hz" in tooltip
    assert "YOLO Hz" in tooltip
    assert "-- 는 아직 통계가 없고" in tooltip
    assert "0.0 Hz 는 통계는 있으나" in tooltip
    assert "시각" in tooltip
    assert "GNSS" in tooltip
    assert "camera RTC" in tooltip
    assert "source timestamp" in tooltip
    assert "missing" in tooltip

    widget.deleteLater()
    _ = app


def test_distance_history_plot_left_click_selects_and_right_click_deletes_point() -> None:
    from PyQt5 import QtCore, QtWidgets, QtTest

    from wiris_stereo_distance.measurement_widgets import DistanceHistoryPoint, DistanceHistoryPlot

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    plot = DistanceHistoryPlot()
    plot.resize(420, 140)
    plot.show()

    points = [
        DistanceHistoryPoint(10.0, 5.40, "GOOD", "stereo", class_name="person", sample_key=(1.0, 2.0, "stereo")),
        DistanceHistoryPoint(11.0, 5.80, "BAD", "stereo", class_name="car", sample_key=(1.5, 2.5, "stereo")),
    ]
    for point in points:
        plot.add_point(point)
    app.processEvents()

    target = points[0]
    target_pos = plot._point_position(target)
    assert target_pos is not None
    click_pos = QtCore.QPoint(int(round(target_pos.x())), int(round(target_pos.y())))

    QtTest.QTest.mouseClick(plot, QtCore.Qt.LeftButton, QtCore.Qt.NoModifier, click_pos)
    app.processEvents()
    assert plot._selected_point_text() == "1. YOLO range: 5.40 m | 클래스: person"

    second_pos = plot._point_position(points[1])
    assert second_pos is not None
    second_click_pos = QtCore.QPoint(int(round(second_pos.x())), int(round(second_pos.y())))
    QtTest.QTest.mouseClick(plot, QtCore.Qt.LeftButton, QtCore.Qt.NoModifier, second_click_pos)
    app.processEvents()
    assert (
        plot._selected_point_text()
        == "1. YOLO range: 5.40 m | 클래스: person\n"
        "2. YOLO range: 5.80 m | 클래스: car"
    )

    QtTest.QTest.mouseClick(plot, QtCore.Qt.LeftButton, QtCore.Qt.NoModifier, second_click_pos)
    app.processEvents()
    assert plot._selected_point_text() == "1. YOLO range: 5.40 m | 클래스: person"

    QtTest.QTest.mouseClick(plot, QtCore.Qt.LeftButton, QtCore.Qt.NoModifier, QtCore.QPoint(2, 2))
    app.processEvents()
    assert plot._selected_point_text() is None

    QtTest.QTest.mouseClick(plot, QtCore.Qt.LeftButton, QtCore.Qt.NoModifier, click_pos)
    app.processEvents()
    assert plot._selected_point_text() == "1. YOLO range: 5.40 m | 클래스: person"

    QtTest.QTest.mouseClick(plot, QtCore.Qt.RightButton, QtCore.Qt.NoModifier, click_pos)
    app.processEvents()
    assert len(plot.points) == 1
    assert plot.points[0].distance_m == 5.80
    assert plot._selected_point_text() is None

    plot.deleteLater()
    _ = app


def test_distance_history_plot_tracks_xyz_and_distance_panels(monkeypatch) -> None:
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")

    from PyQt5 import QtWidgets

    from wiris_stereo_distance.measurement_widgets import DistanceHistoryPoint, DistanceHistoryPlot

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    plot = DistanceHistoryPlot()
    plot.resize(640, 300)
    plot.show()
    point = DistanceHistoryPoint(
        10.0,
        5.40,
        "GOOD",
        "triangulated-pixhawk",
        x_m=1.0,
        y_m=5.2,
        z_m=-0.4,
        live_truth_range_m=6.4,
        live_range_error_m=-1.0,
        alignment_profile_id="live_profile",
        sample_key=(1.0, 2.0, "triangulated-pixhawk"),
    )
    plot.add_point(point)
    app.processEvents()

    assert set(plot._plot_rects()) == {"x_m", "y_m", "z_m", "distance_m"}
    assert plot._plot_metrics("x_m") is not None
    assert plot._plot_metrics("y_m") is not None
    assert plot._plot_metrics("z_m") is not None
    distance_metrics = plot._plot_metrics("distance_m")
    assert distance_metrics is not None
    assert distance_metrics[3] == 5.4
    assert distance_metrics[4] == 6.4

    plot._selected_point_keys.add(plot._point_key(point))
    selected_text = plot._selected_point_text() or ""
    assert "Right/Down/Forward: +1.00, +5.20, -0.40 m" in selected_text
    assert "Telemetry range: 6.40 m" in selected_text
    assert "YOLO-Telemetry error: -1.00 m" in selected_text
    assert "Telemetry Range" in plot._panel_series("distance_m", "Range", "m", "")[1][1]

    plot.deleteLater()
    _ = app


def test_distance_history_plot_delete_removes_recorded_sample_from_panel(monkeypatch) -> None:
    from types import SimpleNamespace

    from PyQt5 import QtCore, QtWidgets, QtTest

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget.resize(1190, 820)
    widget.show()

    widget.distance_recording_enabled = True
    widget.distance_recording_started_monotonic_s = time.monotonic() - 2.0
    widget._refresh_recording_status_label()
    estimate = SimpleNamespace(
        left=SimpleNamespace(timestamp_s=1.0, class_name="person"),
        right=SimpleNamespace(timestamp_s=1.1, class_name="person"),
        sync_quality="GOOD",
        disparity_px=42.0,
        time_delta_s=0.1,
        vertical_delta_px=2.0,
        warnings=[],
        left_attitude_age_s=None,
        right_attitude_age_s=None,
        attitude_pair_delta_s=None,
        relative_roll_deg=None,
        relative_pitch_deg=None,
        relative_yaw_deg=None,
    )

    widget._append_distance_history(
        estimate=estimate,
        distance_m=5.4,
        method="stereo",
        baseline_source="manual",
        baseline_m=0.3,
    )

    assert len(widget.distance_history) == 1
    assert len(widget.distance_recording_rows) == 1
    assert len(widget.distance_recording_keys) == 1
    assert float(widget.distance_recording_rows[0]["recording_elapsed_s"]) >= 2.0
    assert widget.distance_history[-1].recording_elapsed_s is not None
    assert widget.recording_sample_table.rowCount() == 1
    assert widget.recording_sample_table.item(0, 1).text() == "5.400000"

    point = widget.distance_history[-1]
    assert point.class_name == "person"
    point_pos = widget.distance_plot._point_position(point)
    assert point_pos is not None
    click_pos = QtCore.QPoint(int(round(point_pos.x())), int(round(point_pos.y())))

    QtTest.QTest.mouseClick(widget.distance_plot, QtCore.Qt.RightButton, QtCore.Qt.NoModifier, click_pos)
    app.processEvents()

    assert len(widget.distance_plot.points) == 0
    assert len(widget.distance_history) == 0
    assert len(widget.distance_recording_rows) == 0
    assert len(widget.distance_recording_keys) == 0
    assert widget.recording_sample_table.rowCount() == 0
    assert widget.distance_plot._selected_point_text() is None
    assert widget.recording_status_label.text() == "세션 기록: 진행 중, 거리 샘플=0, 외부 파라미터=0"

    widget.deleteLater()
    _ = app


def test_recording_sample_table_shows_live_csv_fields(monkeypatch) -> None:
    from types import SimpleNamespace

    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget.distance_recording_enabled = True

    estimate = SimpleNamespace(
        left=SimpleNamespace(timestamp_s=2.0, class_name="person"),
        right=SimpleNamespace(timestamp_s=2.05, class_name="person"),
        sync_quality="GOOD",
        disparity_px=40.0,
        time_delta_s=0.05,
        vertical_delta_px=1.5,
        warnings=["low baseline confidence"],
        left_attitude_age_s=None,
        right_attitude_age_s=None,
        attitude_pair_delta_s=None,
        relative_roll_deg=1.0,
        relative_pitch_deg=2.0,
        relative_yaw_deg=3.0,
    )

    assert widget._append_distance_history(
        estimate=estimate,
        distance_m=6.0,
        method="stereo",
        baseline_source="manual",
        baseline_m=0.25,
    )

    assert widget.recording_sample_table.rowCount() == 1
    assert widget.recording_sample_table.item(0, 1).text() == "6.000000"
    assert widget.recording_sample_table.item(0, 3).text() == "GOOD"
    assert widget.recording_sample_table.item(0, 11).text() == "3.000000"
    assert "distance_m: 6.000000" in widget.recording_sample_detail.toPlainText()
    assert "warnings: low baseline confidence" in widget.recording_sample_detail.toPlainText()

    widget.clear_distance_history()

    assert widget.recording_sample_table.rowCount() == 0
    assert widget.recording_sample_detail.toPlainText() == ""

    widget.deleteLater()
    _ = app


def test_distance_recording_saves_filtered_and_corrected_tracking_fields(monkeypatch) -> None:
    from types import SimpleNamespace

    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui
    from wiris_stereo_distance.tracking_filter import TrackingUpdateResult

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget.distance_recording_enabled = True

    estimate = SimpleNamespace(
        left=SimpleNamespace(timestamp_s=2.0, class_name="UAV"),
        right=SimpleNamespace(timestamp_s=2.05, class_name="UAV"),
        sync_quality="GOOD",
        disparity_px=111.0,
        time_delta_s=0.05,
        vertical_delta_px=1.5,
        warnings=[],
        left_attitude_age_s=None,
        right_attitude_age_s=None,
        attitude_pair_delta_s=None,
        relative_roll_deg=None,
        relative_pitch_deg=None,
        relative_yaw_deg=None,
    )
    tracking_result = TrackingUpdateResult(
        point_m=(1.0, 2.0, 9.0),
        velocity_mps=(0.1, 0.2, 0.3),
        covariance_diag_m2=(0.01, 0.02, 0.03),
        measurement_std_m=(0.4, 0.4, 6.0),
        accepted=True,
        initialized=True,
        reset=False,
        normalized_innovation=1.25,
        timestamp_s=2.05,
    )

    assert widget._append_distance_history(
        estimate=estimate,
        distance_m=9.273618,
        method="triangulated-pixhawk+kf",
        baseline_source="manual",
        baseline_m=25.0,
        point_m=tracking_result.point_m,
        raw_point_m=(4.0, 5.0, 12.0),
        corrected_point_m=(2.0, 3.0, 10.0),
        tracking_result=tracking_result,
    )

    row = widget.distance_recording_rows[-1]
    assert row["distance_m"] == "9.273618"
    assert row["filtered_forward_m"] == "9.000000"
    assert row["corrected_forward_m"] == "10.000000"
    assert row["raw_forward_m"] == "12.000000"
    assert row["tracking_filter_id"] == "constant_velocity_kalman_v1"
    assert row["tracking_accepted"] == "true"
    assert row["tracking_velocity_forward_mps"] == "0.300000"
    assert widget.distance_history[-1].z_m == 9.0

    widget.deleteLater()
    _ = app


def test_bias_profile_uses_raw_minus_drone_error_bins(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()

    profile = widget._bias_profile_from_rows(
        [
            {
                "raw_distance_m": "12.0",
                "raw_x_m": "0.0",
                "raw_y_m": "2.0",
                "raw_z_m": "6.0",
                "drone_x_m": "-1.0",
                "drone_y_m": "1.5",
                "drone_z_m": "5.0",
            },
            {
                "raw_distance_m": "18.0",
                "raw_x_m": "2.0",
                "raw_y_m": "3.0",
                "raw_z_m": "8.0",
                "drone_x_m": "1.0",
                "drone_y_m": "1.5",
                "drone_z_m": "5.0",
            },
        ],
        bin_size_m=20.0,
    )

    assert profile is not None
    assert len(profile.bins) == 1
    assert profile.correction(15.0) == (1.0, 1.0, 2.0)

    widget.deleteLater()
    _ = app


def test_bias_recording_saves_bbox_drone_raw_corrected_and_error_fields(monkeypatch) -> None:
    from types import SimpleNamespace

    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui
    from wiris_stereo_distance.uam_telemetry import UamAlignmentProfile

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget.distance_recording_enabled = True
    widget.latest_drone_position_m = (9.0, 1.5, 3.0)
    widget.uam_alignment_profile = UamAlignmentProfile(
        profile_id="live_alignment",
        rotation_matrix=((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
        origin_offset_m=(7.0, 1.5, 3.0),
        sample_count=4,
        rmse_m=0.0,
        max_error_m=0.0,
        created_unix_s=1.0,
    )
    widget.active_bias_profile = gui.BBoxBiasProfile(
        profile_id="test_profile",
        bins=(
            gui.BBoxBiasBin(
                min_m=0.0,
                max_m=100.0,
                bias_x_m=1.0,
                bias_y_m=0.5,
                bias_z_m=-2.0,
                sample_count=3,
            ),
        ),
        created_unix_s=1.0,
    )

    widget.latest_drone_position_timestamp_s = 3.02
    estimate = SimpleNamespace(
        left=detection(
            "left",
            x1=100.0,
            y1=200.0,
            x2=140.0,
            y2=260.0,
            t=3.0,
            cls=1,
            timestamp_metadata={"timestamp_reference": "rtcp-ntp", "rtcp_frame_ntp_unix_s": 1_800_000_000.0},
        ),
        right=detection(
            "right",
            x1=90.0,
            y1=201.0,
            x2=130.0,
            y2=261.0,
            t=3.05,
            cls=1,
            timestamp_metadata={"timestamp_reference": "rtcp-ntp", "rtcp_frame_ntp_unix_s": 1_800_000_000.05},
        ),
        sync_quality="GOOD",
        disparity_px=10.0,
        time_delta_s=0.05,
        vertical_delta_px=1.0,
        warnings=[],
        left_attitude_age_s=None,
        right_attitude_age_s=None,
        attitude_pair_delta_s=None,
        relative_roll_deg=None,
        relative_pitch_deg=None,
        relative_yaw_deg=None,
    )

    assert widget._append_distance_history(
        estimate=estimate,
        distance_m=9.25,
        method="triangulated-rtk",
        baseline_source="manual",
        baseline_m=0.3,
        point_m=(9.0, 2.0, 4.0),
        raw_point_m=(10.0, 2.5, 2.0),
        raw_point_left_m=(10.15, 2.5, 2.0),
        translation_right_in_left_m=(0.3, 0.0, 0.0),
        bias_correction_m=(1.0, 0.5, -2.0),
    )

    row = widget.distance_recording_rows[-1]
    assert row["measurement_frame"] == "stereo_center_opencv_optical"
    assert row["measurement_axis_contract"] == "x/right,y/down,z/forward"
    assert row["raw_x_m"] == "10.000000"
    assert row["raw_left_x_m"] == "10.150000"
    assert row["translation_right_in_left_x_m"] == "0.300000"
    assert row["corrected_z_m"] == "4.000000"
    assert row["live_truth_range_m"] == "2.000000"
    assert row["live_range_error_m"] == "7.250000"
    assert row["live_range_error_basis"] == "baseline_center"
    assert row["live_range_alignment_profile_id"] == "live_alignment"
    assert row["telemetry_range_m"] == "2.000000"
    assert row["telemetry_range_error_m"] == "7.250000"
    assert row["telemetry_range_basis"] == "baseline_center"
    assert row["telemetry_range_profile_id"] == "live_alignment"
    assert "telemetry_baseline_center_distance_m" not in row
    assert "telemetry_baseline_range_error_m" not in row
    assert "origin_baseline_center_distance_m" not in row
    assert "origin_baseline_range_error_m" not in row
    assert widget.distance_history[-1].live_truth_range_m == 2.0
    assert widget.distance_history[-1].live_range_error_m == 7.25
    assert row["drone_north_m"] == "9.000000"
    assert row["drone_y_m"] == "1.500000"
    assert row["error_x_m"] == "0.000000"
    assert row["error_frame"] == "mixed-stereo-center-vs-px4-local-ned"
    assert row["error_y_m"] == "0.500000"
    assert row["error_z_m"] == "1.000000"
    assert row["bias_profile_id"] == "test_profile"
    assert row["world_frame"] == "ned"
    assert row["body_frame"] == "px4_frd"
    assert row["camera_frame"] == "opencv_optical"
    assert row["camera_mount_profile"] == "front_camera_z_to_body_x_x_to_body_y"
    assert row["mount_assumption_model"] == "gps-rtk-baseline-only-v1"
    assert row["mount_assumption_status"] == "GPS/RTK baseline only"
    assert row["mount_assumption_detail"] == "live_gps_rtk_baseline_used_as_stereo_translation"
    assert "manual_baseline_setting_m" not in row
    assert row["left_bbox_center_x_px"] == "120.000"
    assert row["right_bbox_confidence"] == "0.900000"
    assert row["bbox_mid_timestamp_s"] == "3.025000"
    assert row["bbox_mid_utc_unix_s"] == "1800000000.025000"
    assert row["drone_timestamp_s"] == "3.020000"
    assert row["drone_age_s"] == "0.005000"

    widget.deleteLater()
    _ = app


def test_drone_position_watch_updates_latest_drone_position(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui
    from wiris_stereo_distance.pixhawk_attitude import CustomTopicSample

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()

    for key, value in (("drone_x", 1.0), ("drone_y", -2.0), ("drone_z", 3.5)):
        widget._update_custom_watch(
            CustomTopicSample(
                key=key,
                topic="/pose",
                field_path=key,
                value_text=str(value),
                numeric_value=value,
                timestamp_s=1.0,
            )
        )

    assert widget.latest_drone_position_m == (1.0, -2.0, 3.5)
    assert len(widget.uam_live_telemetry_samples) == 1
    assert widget.uam_live_telemetry_samples[-1].source == "live-ros2"
    assert widget.custom_watch_remove_combo.findData("drone_x") < 0
    assert "N=+1.000" in widget.bias_drone_status_label.text()
    assert widget.bias_drone_status_label.text().startswith("UAM 중심 위치:")

    widget.deleteLater()
    _ = app


def test_uam_mavlink_watch_updates_live_buffer(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui
    from wiris_stereo_distance.pixhawk_attitude import CustomTopicSample

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()

    for key, value in (("left:uam_x", 10.0), ("left:uam_y", -2.0), ("left:uam_z", -1.0)):
        widget._update_custom_watch(
            CustomTopicSample(
                key=key,
                topic="mavlink/LOCAL_POSITION_NED",
                field_path=key,
                value_text=str(value),
                numeric_value=value,
                timestamp_s=3.0,
            )
        )

    assert widget.latest_drone_position_m == (10.0, -2.0, -1.0)
    assert len(widget.uam_live_telemetry_samples) == 1
    assert widget.uam_live_telemetry_samples[-1].source == "live-mavlink:left"
    assert "source=live-mavlink:left" in widget.uam_latest_label.text()
    assert any(point.series == "truth_north_m" and point.value == 10.0 for point in widget.uam_raw_plot.points)

    widget.deleteLater()
    _ = app


def test_uam_usb_mavlink_uses_separate_subscriber(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    class FakeSignal:
        def __init__(self) -> None:
            self.slots = []

        def connect(self, slot) -> None:
            self.slots.append(slot)

    class FakeMavlinkSubscriber:
        instances = []

        def __init__(self, *, left_url, right_url, left_baud=57600, right_baud=57600, custom_watches=()) -> None:
            self.urls = {"left": left_url, "right": right_url}
            self.baudrates = {"left": left_baud, "right": right_baud}
            self.custom_watches = list(custom_watches)
            self.connections = {}
            self.gps_changed = FakeSignal()
            self.custom_watch_changed = FakeSignal()
            self.status_changed = FakeSignal()
            self.extrinsic_changed = FakeSignal()
            self.mag_sensor_changed = FakeSignal()
            self.gps_sensor_changed = FakeSignal()
            self.gnss_relative_changed = FakeSignal()
            self.shell_output = FakeSignal()
            self.started_camera = None
            self.instances.append(self)

        def start(self, camera=None) -> None:
            self.started_camera = camera
            if camera == "left" and self.urls["left"]:
                self.connections["left"] = object()

        def stop(self, camera=None) -> None:
            self.connections.clear()

        def add_custom_watch(self, spec) -> None:
            self.custom_watches.append(spec)

        def remove_custom_watch(self, key) -> None:
            self.custom_watches = [spec for spec in self.custom_watches if spec.get("key") != key]

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    monkeypatch.setattr(gui, "DualPixhawkMavlinkSubscriber", FakeMavlinkSubscriber)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget._set_combo_data(widget.uam_source_combo, "mavlink")
    widget.uam_mavlink_url_edit.setText("/dev/ttyACM7")
    widget.uam_mavlink_baud_spin.setValue(921600)

    widget.connect_uam_telemetry_from_ui()

    subscriber = widget.uam_mavlink_pixhawk
    assert subscriber is not None
    assert widget.mavlink_pixhawk is None
    assert subscriber.urls == {"left": "/dev/ttyACM7", "right": ""}
    assert subscriber.baudrates["left"] == 921600
    assert subscriber.started_camera == "left"
    assert all(spec["source"] == "uam_mavlink" for spec in subscriber.custom_watches)
    assert {spec["key"] for spec in subscriber.custom_watches} == {"uam_x", "uam_y", "uam_z"}
    assert "USB MAVLink watch 등록" in widget.uam_status_label.text()

    widget.deleteLater()
    _ = app


def test_uam_usb_mavlink_gps_converts_to_local_ned(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui
    from wiris_stereo_distance.rtk_geometry import GpsPositionState

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget._set_combo_data(widget.uam_source_combo, "mavlink_gps")

    widget._update_uam_live_telemetry_from_gps(
        "left",
        GpsPositionState(
            camera="left",
            namespace="uam_usb",
            timestamp_s=10.0,
            topic="mavlink/GPS_RAW_INT",
            latitude_deg=37.0,
            longitude_deg=127.0,
            altitude_m=100.0,
        ),
    )
    widget._update_uam_live_telemetry_from_gps(
        "left",
        GpsPositionState(
            camera="left",
            namespace="uam_usb",
            timestamp_s=11.0,
            topic="mavlink/GPS_RAW_INT",
            latitude_deg=37.00001,
            longitude_deg=127.00002,
            altitude_m=98.5,
        ),
    )

    assert widget.latest_drone_position_m is not None
    north_m, east_m, down_m = widget.latest_drone_position_m
    assert north_m > 0.0
    assert east_m > 0.0
    assert down_m == 1.5
    assert widget.latest_drone_truth_sample is not None
    assert widget.latest_drone_truth_sample.source_frame == "global_lla"
    assert widget.latest_drone_truth_sample.source_axis_contract == "lat_deg,lon_deg,alt_m"

    widget.deleteLater()
    _ = app


def test_disconnect_uam_telemetry_removes_uam_watches(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()

    for axis in ("x", "y", "z"):
        widget._add_custom_topic_watch(
            key=f"uam_{axis}",
            topic="/uav_0/fmu/out/vehicle_local_position",
            field=axis,
            source="ros2",
        )
    widget.latest_drone_position_m = (1.0, 2.0, 3.0)
    widget.latest_drone_position_timestamp_reference = "live-ros2"
    widget.latest_drone_position_utc_unix_s = 1_800_000_000.0

    widget.disconnect_uam_telemetry_from_ui()

    assert all(widget.custom_watch_remove_combo.findData(f"uam_{axis}") < 0 for axis in ("x", "y", "z"))
    assert not any(str(spec.get("key", "")).startswith("uam_") for spec in widget.custom_watch_specs)
    assert widget.latest_drone_position_m is None
    assert widget.latest_drone_position_timestamp_reference == "app-receive"
    assert widget.latest_drone_position_utc_unix_s is None
    assert "해제됨" in widget.uam_status_label.text()

    widget.deleteLater()
    _ = app


def test_uam_time_offset_estimate_applies_and_runs_analysis(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui
    from wiris_stereo_distance.uam_telemetry import UamTelemetrySample

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget.distance_recording_rows = [
        {"timestamp_s": "1.0", "x_m": "0.0", "y_m": "0.0", "z_m": "0.0", "distance_m": "0.0"},
        {"timestamp_s": "2.0", "x_m": "1.0", "y_m": "0.0", "z_m": "0.0", "distance_m": "1.0"},
        {"timestamp_s": "3.0", "x_m": "2.0", "y_m": "0.0", "z_m": "0.0", "distance_m": "2.0"},
    ]
    widget.uam_ulog_samples = [
        UamTelemetrySample(timestamp_s=11.0, x_m=0.0, y_m=0.0, z_m=0.0, source="ulog:test"),
        UamTelemetrySample(timestamp_s=12.0, x_m=1.0, y_m=0.0, z_m=0.0, source="ulog:test"),
        UamTelemetrySample(timestamp_s=13.0, x_m=2.0, y_m=0.0, z_m=0.0, source="ulog:test"),
    ]
    widget.uam_compare_max_dt_spin.setValue(0.2)
    widget.uam_time_offset_spin.setValue(0.0)

    widget.estimate_uam_time_offset_from_ui()

    assert widget.uam_time_offset_spin.value() == 10.0
    assert len(widget.uam_analysis_rows) == 3
    assert widget.uam_analysis_rows[0]["truth_source"] == "ulog:test"
    assert widget.uam_plot_stack.currentWidget() is widget.uam_error_plot
    assert "rows=3" in widget.uam_analysis_status_label.text()

    widget.deleteLater()
    _ = app


def test_uam_alignment_estimate_applies_ned_origin_correction(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui
    from wiris_stereo_distance.uam_telemetry import UamTelemetrySample

    def camera_to_ned(point: tuple[float, float, float]) -> tuple[float, float, float]:
        right, down, forward = point
        return (10.0 + forward, 20.0 + right, 30.0 + down)

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    camera_points = [
        (1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
        (0.0, 0.0, 1.0),
        (2.0, 1.0, 3.0),
    ]
    widget.distance_recording_rows = [
        {
            "timestamp_s": f"{index + 1}.0",
            "x_m": f"{point[0]}",
            "y_m": f"{point[1]}",
            "z_m": f"{point[2]}",
            "distance_m": "1.0",
        }
        for index, point in enumerate(camera_points)
    ]
    widget.uam_ulog_samples = [
        UamTelemetrySample(
            timestamp_s=float(index + 1),
            x_m=camera_to_ned(point)[0],
            y_m=camera_to_ned(point)[1],
            z_m=camera_to_ned(point)[2],
            source="ulog:test",
        )
        for index, point in enumerate(camera_points)
    ]
    widget.uam_compare_max_dt_spin.setValue(0.01)

    widget.estimate_uam_alignment_from_ui()

    assert widget.uam_alignment_profile is not None
    assert widget.uam_alignment_profile.sample_count == 4
    assert len(widget.uam_analysis_rows) == 4
    assert widget.uam_analysis_rows[0]["comparison_frame"] == "px4_local_ned"
    assert widget.uam_analysis_rows[0]["estimated_north_m"] == "10.000000"
    assert widget.uam_analysis_rows[0]["estimated_east_m"] == "21.000000"
    assert widget.uam_analysis_rows[0]["estimated_down_m"] == "30.000000"
    assert widget.uam_analysis_rows[0]["truth_rdf_right_m"] == "1.000000"
    assert widget.uam_analysis_rows[0]["truth_rdf_down_m"] == "0.000000"
    assert widget.uam_analysis_rows[0]["truth_rdf_forward_m"] == "0.000000"
    assert widget.uam_analysis_rows[0]["error_rdf_right_m"] == "0.000000"
    assert widget.uam_analysis_rows[0]["error_rdf_down_m"] == "0.000000"
    assert widget.uam_analysis_rows[0]["error_rdf_forward_m"] == "0.000000"
    assert widget.uam_analysis_rows[0]["measurement_frd_forward_m"] == "0.000000"
    assert widget.uam_analysis_rows[0]["measurement_frd_right_m"] == "1.000000"
    assert widget.uam_analysis_rows[0]["measurement_frd_down_m"] == "0.000000"
    assert widget.uam_analysis_rows[0]["truth_frd_forward_m"] == "0.000000"
    assert widget.uam_analysis_rows[0]["truth_frd_right_m"] == "1.000000"
    assert widget.uam_analysis_rows[0]["truth_frd_down_m"] == "0.000000"
    assert widget.uam_analysis_rows[0]["error_frd_forward_m"] == "0.000000"
    assert widget.uam_analysis_rows[0]["error_frd_right_m"] == "0.000000"
    assert widget.uam_analysis_rows[0]["error_frd_down_m"] == "0.000000"
    assert widget.uam_analysis_rows[0]["estimated_range_m"] == "1.000000"
    assert widget.uam_analysis_rows[0]["truth_range_m"] == "1.000000"
    assert "estimated_baseline_center_distance_m" not in widget.uam_analysis_rows[0]
    assert "truth_baseline_center_distance_m" not in widget.uam_analysis_rows[0]
    assert widget.uam_analysis_rows[0]["range_error_m"] == "0.000000"
    assert widget.uam_analysis_rows[0]["range_error_basis"] == "baseline_center"
    assert widget.uam_analysis_rows[0]["error_distance_kind"] == "position_error_m"
    assert "alignment=" in widget.uam_analysis_status_label.text()
    assert "rmse=" in widget.uam_alignment_status_label.text()
    assert any(point.series == "error_frd_forward_m" for point in widget.uam_error_plot.points)

    widget.deleteLater()
    _ = app


def test_uam_alignment_profile_save_and_load(monkeypatch, tmp_path) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui
    from wiris_stereo_distance.uam_telemetry import UamAlignmentProfile

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    profile = UamAlignmentProfile(
        profile_id="saved_alignment",
        rotation_matrix=((0.0, 0.0, 1.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)),
        origin_offset_m=(10.0, 20.0, 30.0),
        sample_count=4,
        rmse_m=0.01,
        max_error_m=0.02,
        created_unix_s=1_800_000_000.0,
    )
    path = tmp_path / "alignment.json"
    widget.uam_alignment_profile = profile

    monkeypatch.setattr(QtWidgets.QFileDialog, "getSaveFileName", lambda *args, **kwargs: (str(path), ""))
    widget.save_uam_alignment_profile()

    assert path.exists()
    assert widget.uam_alignment_profile_path == path

    widget.uam_alignment_profile = None
    widget.uam_alignment_profile_path = None
    monkeypatch.setattr(QtWidgets.QFileDialog, "getOpenFileName", lambda *args, **kwargs: (str(path), ""))
    widget.load_uam_alignment_profile()

    assert widget.uam_alignment_profile == profile
    assert widget.uam_alignment_profile_path == path
    assert "saved_alignment" in widget.uam_alignment_status_label.text()

    widget.deleteLater()
    _ = app


def test_uam_truth_plot_refreshes_raw_plot(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui
    from wiris_stereo_distance.uam_telemetry import UamTelemetrySample

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()

    widget._refresh_uam_truth_plot(
        [UamTelemetrySample(timestamp_s=5.0, x_m=4.0, y_m=-1.0, z_m=2.0, source="ulog:test")]
    )

    raw_points = list(widget.uam_raw_plot.points)
    assert [(point.series, point.value) for point in raw_points] == [
        ("truth_north_m", 4.0),
        ("truth_east_m", -1.0),
        ("truth_down_m", 2.0),
    ]
    assert widget.uam_raw_plot._series_names() == ["truth_north_m", "truth_east_m", "truth_down_m"]
    assert widget.uam_raw_plot._series_label("truth_north_m") == "N"
    assert widget.uam_raw_plot._series_label("truth_east_m") == "E"
    assert widget.uam_raw_plot._series_label("truth_down_m") == "D"
    assert widget.uam_raw_plot._time_bounds() == (5.0, 6.0)
    assert widget.uam_raw_plot._format_hover_text(raw_points[0]) == "N\nt=5.000s\nvalue=4.000 m"

    widget.deleteLater()
    _ = app


def test_uam_analysis_uses_recorded_truth_fallback(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget.distance_recording_rows = [
        {
            "timestamp_s": "5.0",
            "x_m": "11.0",
            "y_m": "0.5",
            "z_m": "-2.0",
            "distance_m": "11.191514",
            "drone_x_m": "10.0",
            "drone_y_m": "0.0",
            "drone_z_m": "-2.0",
            "drone_timestamp_s": "4.9",
        }
    ]

    widget.run_uam_analysis()

    assert len(widget.uam_analysis_rows) == 1
    assert widget.uam_analysis_table.rowCount() == 1
    assert widget.uam_analysis_rows[0]["truth_source"] == "live-recorded-row"
    assert widget.uam_analysis_rows[0]["error_x_m"] == "1.000000"
    assert "rows=1" in widget.uam_analysis_status_label.text()

    widget.deleteLater()
    _ = app


def test_uam_analysis_uses_live_telemetry_for_metrics(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui
    from wiris_stereo_distance.uam_telemetry import UamAlignmentProfile, UamTelemetrySample

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget.distance_recording_rows = [
        {"timestamp_s": "1.0", "x_m": "1.0", "y_m": "0.0", "z_m": "0.0", "distance_m": "1.0"},
        {"timestamp_s": "2.0", "x_m": "0.0", "y_m": "2.0", "z_m": "0.0", "distance_m": "2.0"},
    ]
    widget.uam_live_telemetry_samples = [
        UamTelemetrySample(timestamp_s=1.0, x_m=1.0, y_m=0.0, z_m=0.0, source="live:test"),
        UamTelemetrySample(timestamp_s=2.0, x_m=0.0, y_m=2.0, z_m=0.0, source="live:test"),
    ]
    widget.uam_alignment_profile = UamAlignmentProfile(
        profile_id="live_alignment",
        rotation_matrix=((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
        origin_offset_m=(0.0, 0.0, 0.0),
        sample_count=4,
        rmse_m=0.0,
        max_error_m=0.0,
        created_unix_s=1_800_000_000.0,
    )
    widget.uam_compare_max_dt_spin.setValue(0.01)

    widget.run_uam_analysis()

    assert len(widget.uam_analysis_rows) == 2
    assert widget.uam_analysis_rows[0]["truth_source"] == "live:test"
    assert widget.uam_analysis_rows[0]["error_distance_kind"] == "position_error_m"
    assert widget.uam_analysis_rows[0]["position_error_m"] == "0.000000"
    assert widget.uam_analysis_rows[0]["truth_rdf_right_m"] == "1.000000"
    assert widget.uam_analysis_rows[0]["error_rdf_right_m"] == "0.000000"
    assert widget.uam_analysis_rows[0]["truth_frd_right_m"] == "1.000000"
    assert widget.uam_analysis_rows[0]["error_frd_right_m"] == "0.000000"
    assert not any(key.endswith("_eligible") or key.endswith("_blocker") for key in widget.uam_analysis_rows[0])
    assert widget.uam_analysis_table.rowCount() == 2
    assert "source=live-buffer:2 samples" in widget.uam_analysis_status_label.text()
    assert "rmse=0.000 m" in widget.uam_analysis_status_label.text()

    widget.deleteLater()
    _ = app


def test_uam_analysis_does_not_mix_ulog_with_recorded_row_fallback(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui
    from wiris_stereo_distance.uam_telemetry import UamTelemetrySample

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget.distance_recording_rows = [
        {
            "timestamp_s": "1.0",
            "x_m": "1.0",
            "y_m": "0.0",
            "z_m": "0.0",
            "distance_m": "1.0",
            "drone_x_m": "100.0",
            "drone_y_m": "100.0",
            "drone_z_m": "100.0",
            "drone_timestamp_s": "1.0",
        },
        {
            "timestamp_s": "100.0",
            "x_m": "2.0",
            "y_m": "0.0",
            "z_m": "0.0",
            "distance_m": "2.0",
            "drone_x_m": "2.0",
            "drone_y_m": "0.0",
            "drone_z_m": "0.0",
            "drone_timestamp_s": "100.0",
        },
    ]
    widget.uam_ulog_samples = [
        UamTelemetrySample(timestamp_s=1.0, x_m=1.0, y_m=0.0, z_m=0.0, source="ulog:test")
    ]
    widget.uam_compare_max_dt_spin.setValue(0.01)

    widget.run_uam_analysis()

    assert len(widget.uam_analysis_rows) == 1
    assert widget.uam_analysis_rows[0]["truth_source"] == "ulog:test"
    assert "live-recorded-row" not in {row["truth_source"] for row in widget.uam_analysis_rows}
    assert "matched=1/2" in widget.uam_analysis_status_label.text()
    assert "unmatched=1" in widget.uam_analysis_status_label.text()

    widget.deleteLater()
    _ = app


def test_distance_label_shows_graph_reason_in_smaller_text(monkeypatch) -> None:
    from types import SimpleNamespace

    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()

    estimate = SimpleNamespace(
        left=SimpleNamespace(timestamp_s=1.0, class_name="person"),
        right=SimpleNamespace(timestamp_s=1.1, class_name="person"),
        sync_quality="GOOD",
        disparity_px=42.0,
        time_delta_s=0.1,
        vertical_delta_px=2.0,
        warnings=[],
    )

    assert widget._append_distance_history(
        estimate=estimate,
        distance_m=5.4,
        method="stereo",
        baseline_source="manual",
        baseline_m=0.3,
    )
    assert not widget._append_distance_history(
        estimate=estimate,
        distance_m=5.4,
        method="stereo",
        baseline_source="manual",
        baseline_m=0.3,
    )

    widget._set_distance_text(
        "거리: 5.40 m (triangulated-pixhawk)",
        quality="GOOD",
        detail="그래프 미표시: 직전과 동일한 샘플이라 중복 추가를 건너뛰었습니다.",
    )

    text = widget.distance_label.text()
    assert "거리: 5.40 m (triangulated-pixhawk)" in text
    assert "그래프 미표시: 직전과 동일한 샘플이라 중복 추가를 건너뛰었습니다." in text
    assert "font-size: 9px" in text
    assert "font-size: 14px" in widget.distance_label.styleSheet()

    widget.deleteLater()
    _ = app


def test_apply_rtsp_candidates_single_match_keeps_other_side(monkeypatch) -> None:
    from types import SimpleNamespace

    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget.check_cameras = lambda: None

    widget.left_url_edit.setText("rtsp://10.0.0.231:8554/visible")
    widget.right_url_edit.setText("rtsp://10.0.0.240:8554/visible")
    widget.left_zoom_host_edit.setText("10.0.0.231")
    widget.right_zoom_host_edit.setText("10.0.0.232")

    candidate = SimpleNamespace(
        url="rtsp://10.0.0.232:8554/visible",
        host="10.0.0.232",
        port=8554,
        interface=SimpleNamespace(name="enx588694fce07e", address="10.0.0.10/24"),
    )

    widget._apply_rtsp_candidates([candidate])

    assert widget.left_url_edit.text() == "rtsp://10.0.0.231:8554/visible"
    assert widget.left_zoom_host_edit.text() == "10.0.0.231"
    assert widget.right_url_edit.text() == "rtsp://10.0.0.232:8554/visible"
    assert widget.right_zoom_host_edit.text() == "10.0.0.232"

    widget.deleteLater()
    _ = app


def test_camera_detection_populates_manual_candidate_table_and_applies_rtsp(monkeypatch) -> None:
    from types import SimpleNamespace

    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget.check_cameras = lambda: None

    interface = SimpleNamespace(name="enx-new", address="10.0.0.10/24")
    widget._apply_camera_detection(
        {
            "mode": "rtsp",
            "host_interfaces": [SimpleNamespace(interface=interface, neighbor_hosts=("10.0.0.231",))],
            "rtsp": [
                SimpleNamespace(
                    url="rtsp://10.0.0.231:8554/visible",
                    host="10.0.0.231",
                    port=8554,
                    interface=interface,
                )
            ],
            "control": [],
            "verified_control": [],
            "arp_scan": [],
        }
    )

    assert widget.camera_candidate_table.rowCount() == 2
    widget.camera_candidate_table.selectRow(1)
    widget.apply_selected_camera_candidate("right")

    assert widget.right_url_edit.text() == "rtsp://10.0.0.231:8554/visible"
    assert widget.right_rtsp_ip_edit.text() == "10.0.0.231"
    assert widget.interface_edit.text() == "enx-new"
    assert widget.host_address_edit.text() == "10.0.0.10/24"

    widget.deleteLater()
    _ = app


def test_manual_control_candidate_applies_host_port_and_rtsp(monkeypatch) -> None:
    from types import SimpleNamespace

    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()

    interface = SimpleNamespace(name="enx-new", address="10.0.0.10/24")
    widget._set_camera_candidate_rows(
        [],
        [],
        [],
        [
            {
                "host": "10.0.0.232",
                "port": 4321,
                "interface": interface,
                "ok": True,
                "response": "OK",
            }
        ],
    )

    widget.camera_candidate_table.selectRow(0)
    widget.apply_selected_camera_candidate("left")

    assert widget.left_zoom_host_edit.text() == "10.0.0.232"
    assert widget.left_rtsp_ip_edit.text() == "10.0.0.232"
    assert widget.left_control_port_spin.value() == 4321
    assert widget.interface_edit.text() == "enx-new"

    widget.deleteLater()
    _ = app


def test_rtsp_apply_button_updates_url_without_changing_control_host(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()

    widget.left_url_edit.setText("rtsp://10.0.0.231:8554/visible")
    widget.left_rtsp_ip_edit.setText("10.0.0.250")
    widget.left_zoom_host_edit.setText("10.0.0.231")
    widget.right_zoom_host_edit.setText("10.0.0.232")

    widget.left_rtsp_apply_button.click()
    app.processEvents()

    assert widget.left_url_edit.text() == "rtsp://10.0.0.250:8554/visible"
    assert widget.left_zoom_host_edit.text() == "10.0.0.231"
    assert widget.right_zoom_host_edit.text() == "10.0.0.232"
    assert widget.camera_connection_label.text() == "camera RTSP: left=10.0.0.231->10.0.0.250"

    widget.deleteLater()
    _ = app


def test_control_apply_button_saves_host_port_and_syncs_rtsp(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    saved = []
    monkeypatch.setattr(widget, "_save_current_settings", lambda *args, **kwargs: saved.append(True))

    widget.left_zoom_host_edit.setText("10.0.0.231")
    widget.left_control_port_spin.setValue(2240)
    widget.left_url_edit.setText("rtsp://10.0.0.250:8554/visible")
    widget.left_rtsp_ip_edit.setText("10.0.0.250")

    widget.left_control_apply_button.click()
    app.processEvents()

    assert saved == [True]
    assert widget.left_zoom_host_edit.text() == "10.0.0.231"
    assert widget.left_control_port_spin.value() == 2240
    assert widget.left_url_edit.text() == "rtsp://10.0.0.231:8554/visible"
    assert widget.left_rtsp_ip_edit.text() == "10.0.0.231"
    assert widget.camera_connection_label.text() == "camera control: left=applied 10.0.0.231:2240"

    widget.deleteLater()
    _ = app


def test_format_runtime_camera_timestamp_prefers_gnss_utc_then_rtcp_then_source_then_missing() -> None:
    from datetime import datetime, timezone

    from wiris_stereo_distance.gui import format_runtime_camera_timestamp

    expected_utc = datetime.fromtimestamp(1_700_000_000.25, tz=timezone.utc).isoformat(timespec="microseconds")
    expected_utc = expected_utc.replace("+00:00", "Z")

    assert format_runtime_camera_timestamp({"rtcp_frame_ntp_unix_s": 1_700_000_000.25, "timestamp_reference": "gnss-utc"}) == (
        f"UTC {expected_utc}"
    )
    assert format_runtime_camera_timestamp({"rtcp_frame_ntp_unix_s": 1_700_000_000.25, "source_timestamp_s": 10.5}) == (
        f"Camera RTC {expected_utc}"
    )
    assert format_runtime_camera_timestamp(
        {"source_timestamp_s": 10.012345, "timestamp_source": "gstreamer-pts"}
    ) == "src 10.012345 s [gstreamer-pts]"
    assert format_runtime_camera_timestamp({}) == "missing"


def test_runtime_status_labels_show_utc_and_rtcp_fallback() -> None:
    from PyQt5 import QtGui, QtWidgets

    from wiris_stereo_distance.gui import WirisStereoDistancePanel

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = WirisStereoDistancePanel()
    image = QtGui.QImage(16, 9, QtGui.QImage.Format_RGB888)

    widget._update_calibration_preview(
        "left",
        image,
        {
            "timestamp_s": 1.0,
            "source_timestamp_s": 10.5,
            "timestamp_source": "gstreamer-pts",
            "rtcp_frame_ntp_unix_s": 1_700_000_000.25,
            "timestamp_reference": "gnss-utc",
        },
    )
    widget._update_calibration_preview(
        "right",
        image,
        {
            "timestamp_s": 1.0,
            "source_timestamp_s": 10.012345,
            "timestamp_source": "gstreamer-pts",
            "rtcp_frame_ntp_unix_s": 1_700_000_000.118,
        },
    )

    assert widget.left_runtime_stats_label.text().startswith("왼쪽: ")
    assert " | 시각: UTC " in widget.left_runtime_stats_label.text()
    assert widget.right_runtime_stats_label.text().startswith("오른쪽: ")
    assert " | 시각: Camera RTC " in widget.right_runtime_stats_label.text()

    widget.deleteLater()
    _ = app


def test_browse_yolo_weights_updates_path(monkeypatch, tmp_path) -> None:
    from PyQt5 import QtWidgets

    from wiris_stereo_distance.gui import WirisStereoDistancePanel

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = WirisStereoDistancePanel()
    selected = tmp_path / "custom.pt"
    selected.write_text("fake weights")
    monkeypatch.setattr(
        QtWidgets.QFileDialog,
        "getOpenFileName",
        lambda *_args, **_kwargs: (str(selected), "Model weights (*.pt)"),
    )

    widget.browse_yolo_weights()

    assert widget.weights_edit.text() == str(selected)
    assert "weights 파일 선택" in widget.camera_yolo_log.toPlainText()

    widget.deleteLater()
    _ = app
