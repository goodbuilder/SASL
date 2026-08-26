import csv
import struct
import time
from math import degrees
from pathlib import Path

import pytest

from wiris_stereo_distance.pixhawk_attitude import quaternion_to_euler_deg
from wiris_stereo_distance.stereo import (
    DetectionObservation,
    StereoCalibration,
    estimate_disparity_distance,
    estimate_distance,
    match_stereo_observations,
)


@pytest.fixture(autouse=True)
def _isolate_legacy_runtime_settings_writes(monkeypatch, tmp_path: Path) -> None:
    """Never let a GUI shutdown test rewrite the tracked operator settings.

    ``WirisStereoDistancePanel.close()`` intentionally persists the visible
    settings.  Hundreds of GUI tests use the real panel, so the default path
    must be redirected before any test can close a mutated widget.
    """

    from wiris_stereo_distance import legacy_gui
    from wiris_stereo_distance.settings import save_settings as write_settings

    destination = tmp_path / "stereo_distance_settings.json"
    monkeypatch.setattr(
        legacy_gui,
        "save_settings",
        lambda settings: write_settings(settings, destination),
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


def test_measurement_attempt_generation_identity_is_unique_with_same_session_id() -> None:
    from wiris_stereo_distance.gui import _new_measurement_attempt_generation_id

    session_id = "20260722_120000"

    first = _new_measurement_attempt_generation_id(session_id)
    second = _new_measurement_attempt_generation_id(session_id)

    assert first != second
    assert first.startswith(f"{session_id}-")
    assert second.startswith(f"{session_id}-")


def recording_sheet(widget, name: str):
    """Activate one 저장·분석 workbook sheet and return its populated table.

    Sheets refresh lazily, and only the visible one refreshes, so the tab has
    to be selected before its contents can be asserted on.
    """

    tabs = widget.recording_sheet_tabs
    for index in range(tabs.count()):
        if tabs.tabText(index) == name:
            tabs.setCurrentIndex(index)
            widget._refresh_active_recording_sheet()
            return widget.recording_sheet_tables[name]
    raise AssertionError(f"no workbook sheet named {name!r}")


def sheet_cell(table, row: int, column_name: str) -> str | None:
    """Read one cell by column name, since sheet columns are data-dependent."""

    for column in range(table.columnCount()):
        header = table.horizontalHeaderItem(column)
        if header is not None and header.text() == column_name:
            item = table.item(row, column)
            return None if item is None else item.text()
    raise AssertionError(f"no column named {column_name!r}")


def recording_attempt(widget, estimate, *, baseline_m: float = 0.3, translation=None):
    """Build the immutable selected-input fixture consumed by valid-row recording."""

    from wiris_stereo_distance.measurement_attempt import (
        AttitudeSnapshot,
        BaselineSnapshot,
        DetectionSnapshot,
        GeometryConfigSnapshot,
        GpsSnapshot,
        MeasurementAttempt,
        StereoIntrinsicsSnapshot,
        detection_pair_key,
    )
    def selected_detection(camera: str):
        candidate = getattr(estimate, camera)
        if not all(hasattr(candidate, name) for name in ("x1", "y1", "x2", "y2", "width", "height")):
            candidate = detection(
                camera,
                x1=0.0,
                y1=0.0,
                x2=20.0,
                y2=20.0,
                t=float(candidate.timestamp_s),
            )
        return candidate

    left_source = selected_detection("left")
    right_source = selected_detection("right")
    left_detection = DetectionSnapshot.from_observation(
        left_source, pairing_timestamp_basis="detection", pairing_timestamp_s=left_source.timestamp_s
    )
    right_detection = DetectionSnapshot.from_observation(
        right_source, pairing_timestamp_basis="detection", pairing_timestamp_s=right_source.timestamp_s
    )
    left_state = widget.latest_extrinsics.get("left")
    right_state = widget.latest_extrinsics.get("right")
    left_attitude = None if left_state is None else AttitudeSnapshot.from_state(left_state)
    right_attitude = None if right_state is None else AttitudeSnapshot.from_state(right_state)
    left_gps_state = widget.latest_gps.get("left")
    right_gps_state = widget.latest_gps.get("right")
    left_gps = None if left_gps_state is None else GpsSnapshot.from_state(left_gps_state)
    right_gps = None if right_gps_state is None else GpsSnapshot.from_state(right_gps_state)
    intrinsics = (
        None
        if widget.active_intrinsics is None
        else StereoIntrinsicsSnapshot.from_result(
            widget.active_intrinsics,
            active_zoom_level=widget.zoom_level_spin.value(),
            selected_profile_path="test-fixture.json",
        )
    )
    basis = None if translation is None or left_attitude is None else left_attitude.q
    baseline = BaselineSnapshot(
        source="gps-rtk",
        baseline_m=baseline_m,
        ned_right_from_left_m=(0.0, baseline_m, 0.0),
        translation_right_in_left_rdf_m=translation,
        left_basis_quaternion_wxyz=basis,
        fix_quality="GOOD",
        warnings=(),
        usable_for_translation=translation is not None,
        unusable_reason="" if translation is not None else "test fixture has no translation",
        gps_pair_delta_s=(
            None if left_gps is None or right_gps is None else abs(left_gps.timestamp_s - right_gps.timestamp_s)
        ),
        left_age_at_capture_s=0.1 if left_gps is not None else None,
        right_age_at_capture_s=0.2 if right_gps is not None else None,
        left_age_from_detection_midpoint_s=None,
        right_age_from_detection_midpoint_s=None,
        left_basis_attitude_monotonic_s=(
            None if left_attitude is None or translation is None else left_attitude.timestamp_s
        ),
    )
    key = detection_pair_key(
        triangulation_source="pixhawk",
        timestamp_basis="detection",
        left=left_source,
        right=right_source,
        left_pairing_timestamp_s=left_source.timestamp_s,
        right_pairing_timestamp_s=right_source.timestamp_s,
    )
    return MeasurementAttempt(
        attempt_id=1,
        attempt_session_id=widget.recording_session_id or "test",
        detection_pair_key=key,
        created_monotonic_s=max(left_source.timestamp_s, right_source.timestamp_s),
        created_wall_s=1000.0,
        detection_midpoint_monotonic_s=(left_source.timestamp_s + right_source.timestamp_s) / 2.0,
        timestamp_basis="detection",
        triangulation_source="pixhawk",
        left_detection=left_detection,
        right_detection=right_detection,
        left_attitude=left_attitude,
        right_attitude=right_attitude,
        left_gps=left_gps,
        right_gps=right_gps,
        intrinsics=intrinsics,
        baseline=baseline,
        geometry_config=GeometryConfigSnapshot(
            focal_length_px=widget.focal_spin.value(),
            max_time_delta_s=None,
            max_vertical_delta_px=None,
            require_same_class=False,
            timestamp_basis="detection",
            manual_focal_override=widget.manual_focal_checkbox.isChecked(),
        ),
        left_attitude_age_s=getattr(estimate, "left_attitude_age_s", None),
        right_attitude_age_s=getattr(estimate, "right_attitude_age_s", None),
        attitude_pair_delta_s=getattr(estimate, "attitude_pair_delta_s", None),
        relative_roll_deg=getattr(estimate, "relative_roll_deg", None),
        relative_pitch_deg=getattr(estimate, "relative_pitch_deg", None),
        relative_yaw_deg=getattr(estimate, "relative_yaw_deg", None),
        source_quality=estimate.sync_quality,
        warnings=tuple(estimate.warnings),
    )


def set_selected_gps_baseline(widget, *, timestamp_s: float = 10.0, baseline_m: float = 0.3) -> None:
    from wiris_stereo_distance.rtk_geometry import EARTH_RADIUS_M, GpsPositionState

    longitude_delta_deg = degrees(baseline_m / EARTH_RADIUS_M)
    widget.latest_gps["left"] = GpsPositionState(
        "left", "/uav_0", timestamp_s, "gps", 0.0, 127.0, 10.0, fix_type=6
    )
    widget.latest_gps["right"] = GpsPositionState(
        "right", "/uav_1", timestamp_s, "gps", 0.0, 127.0 + longitude_delta_deg, 10.0, fix_type=6
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


def test_ubx_nav_svin_parser_extracts_survey_in_state() -> None:
    import wiris_stereo_distance.gui as gui

    payload = struct.pack(
        "<B3sLLlllbbbBLLBBH",
        0,
        b"\x00\x00\x00",
        123000,
        321,
        1000,
        -2000,
        3000,
        1,
        -2,
        3,
        0,
        1234,
        456,
        1,
        0,
        0,
    )
    parser = gui.UbxStreamParser()

    messages = parser.feed(b"noise" + gui.ubx_frame(0x01, 0x3B, payload))

    assert len(messages) == 1
    message_class, message_id, parsed_payload = messages[0]
    assert (message_class, message_id) == (0x01, 0x3B)
    state = gui.parse_ubx_nav_svin(parsed_payload)
    assert state is not None
    assert state["duration_s"] == 321
    assert state["valid"] is True
    assert state["active"] is False
    assert state["observations"] == 456
    assert state["mean_accuracy_m"] == pytest.approx(0.1234)
    assert state["mean_x_m"] == pytest.approx(10.0001)
    assert state["mean_y_m"] == pytest.approx(-20.0002)
    assert state["mean_z_m"] == pytest.approx(30.0003)


def test_ubx_cfg_tmode3_frames_encode_survey_in_and_disabled_modes() -> None:
    import wiris_stereo_distance.gui as gui

    parser = gui.UbxStreamParser()
    messages = parser.feed(gui.ubx_cfg_tmode3_survey_in_frame(min_duration_s=600, accuracy_m=0.25))

    assert len(messages) == 1
    message_class, message_id, payload = messages[0]
    assert (message_class, message_id) == (0x06, 0x71)
    version, _reserved0, flags = struct.unpack_from("<BBH", payload, 0)
    assert version == 0
    assert flags == 1
    assert struct.unpack_from("<I", payload, 24)[0] == 600
    assert struct.unpack_from("<I", payload, 28)[0] == 2500

    disabled_messages = parser.feed(gui.ubx_cfg_tmode3_disabled_frame())
    assert len(disabled_messages) == 1
    _message_class, _message_id, disabled_payload = disabled_messages[0]
    assert struct.unpack_from("<H", disabled_payload, 2)[0] == 0

    poll_messages = parser.feed(gui.ubx_nav_svin_poll_frame())
    assert len(poll_messages) == 1
    poll_class, poll_id, poll_payload = poll_messages[0]
    assert (poll_class, poll_id) == (0x01, 0x3B)
    assert poll_payload == b""


def test_ubx_cfg_tmode3_fixed_frame_uses_ecef_and_fixed_flag() -> None:
    import wiris_stereo_distance.gui as gui

    parser = gui.UbxStreamParser()
    messages = parser.feed(
        gui.ubx_cfg_tmode3_fixed_frame(
            latitude_deg=37.0,
            longitude_deg=127.0,
            ellipsoid_height_m=52.0,
            fixed_position_accuracy_m=0.05,
        )
    )

    assert len(messages) == 1
    _message_class, _message_id, payload = messages[0]
    _version, _reserved0, flags = struct.unpack_from("<BBH", payload, 0)
    assert flags == 2
    x_cm, y_cm, z_cm = struct.unpack_from("<lll", payload, 4)
    assert (x_cm, y_cm, z_cm) != (0, 0, 0)
    assert struct.unpack_from("<I", payload, 28)[0] == 500


def test_ubx_nav_pvt_coordinate_parser_preserves_height_and_fix_contract() -> None:
    import wiris_stereo_distance.gui as gui
    from wiris_stereo_distance.gnss_operation import HeightType

    payload = bytearray(92)
    # u-blox NAV-PVT uses fixType=3 plus carrier solution FIXED (flags bits
    # 6..7 == 2).  PX4's MAVLink fix_type=6 is a different contract.
    payload[20] = 3
    payload[21] = 0x81
    struct.pack_into("<i", payload, 24, int(round(127.0 * 1e7)))
    struct.pack_into("<i", payload, 28, int(round(37.0 * 1e7)))
    struct.pack_into("<i", payload, 32, 52_000)
    struct.pack_into("<i", payload, 36, 48_000)

    sample = gui.parse_ubx_nav_pvt_coordinate_sample(
        0x01,
        0x07,
        bytes(payload),
        timestamp_s=10.0,
        height_type=HeightType.ELLIPSOID,
    )

    assert sample is not None
    assert sample.fixed is True
    assert sample.latitude_deg == pytest.approx(37.0)
    assert sample.longitude_deg == pytest.approx(127.0)
    assert sample.ellipsoid_height_m == pytest.approx(52.0)
    assert sample.msl_altitude_m == pytest.approx(48.0)
    assert sample.selected_height_m == pytest.approx(52.0)

    payload[20] = 6
    payload[21] = 0x81
    assert gui.parse_ubx_nav_pvt_coordinate_sample(
        0x01, 0x07, bytes(payload), timestamp_s=10.0, height_type=HeightType.ELLIPSOID
    ).fixed is False

    for fix_type, flags in ((3, 0x41), (3, 0x80), (5, 0x81)):
        payload[20] = fix_type
        payload[21] = flags
        sample = gui.parse_ubx_nav_pvt_coordinate_sample(
            0x01, 0x07, bytes(payload), timestamp_s=10.0, height_type=HeightType.ELLIPSOID
        )
        assert sample is not None
        assert sample.fixed is False
    assert gui.parse_ubx_nav_pvt_coordinate_sample(
        0x01, 0x07, b"\x00" * 47, timestamp_s=10.0, height_type=HeightType.ELLIPSOID
    ) is None


def test_ubx_base_satellite_count_uses_nav_pvt_or_nav_sat() -> None:
    import wiris_stereo_distance.gui as gui

    nav_pvt_payload = bytearray(92)
    nav_pvt_payload[23] = 17
    nav_sat_payload = bytearray(8)
    nav_sat_payload[5] = 19

    assert gui.parse_ubx_base_satellite_count(0x01, 0x07, bytes(nav_pvt_payload)) == 17
    assert gui.parse_ubx_base_satellite_count(0x01, 0x35, bytes(nav_sat_payload)) == 19
    assert gui.parse_ubx_base_satellite_count(0x01, 0x3B, b"\x00" * 40) is None


def test_ubx_nav_sat_quality_parser_summarizes_signal_and_constellations() -> None:
    import struct

    import wiris_stereo_distance.gui as gui

    payload = bytearray(8 + 3 * 12)
    payload[5] = 3
    records = (
        (0, 1, 42, 45, 0x01),
        (2, 11, 28, 10, 0x00),
        (3, 21, 35, 20, 0x01),
    )
    for index, (gnss_id, sv_id, cno, elevation, flags) in enumerate(records):
        offset = 8 + index * 12
        payload[offset] = gnss_id
        payload[offset + 1] = sv_id
        payload[offset + 2] = cno
        struct.pack_into("<b", payload, offset + 3, elevation)
        struct.pack_into("<I", payload, offset + 8, flags)

    quality = gui.parse_ubx_nav_sat_quality(0x01, 0x35, bytes(payload))

    assert quality == {
        "tracked": 3,
        "used": 2,
        "avg_cno_dbhz": pytest.approx(35.0),
        "low_cno": 1,
        "low_elevation": 1,
        "constellations": {"GPS": 1, "Galileo": 1, "BeiDou": 1},
    }
    assert gui.parse_ubx_nav_sat_quality(0x01, 0x07, bytes(payload)) is None


def test_rtcm3_stream_parser_extracts_complete_frames() -> None:
    import wiris_stereo_distance.gui as gui

    frame = b"\xd3\x00\x03abc\x00\x00\x00"
    parser = gui.Rtcm3StreamParser()

    assert parser.feed(b"noise" + frame[:4]) == []
    assert parser.feed(frame[4:] + b"tail") == [frame]


def test_rtcm3_message_type_parser_reads_payload_header() -> None:
    import wiris_stereo_distance.gui as gui

    message_type = 1077
    payload = bytes((message_type >> 4, (message_type & 0x0F) << 4, 0x00))
    frame = b"\xd3\x00\x03" + payload + b"\x00\x00\x00"

    assert gui.parse_rtcm3_message_type(frame) == 1077
    assert gui.parse_rtcm3_message_type(b"\xd3\x00\x01x\x00\x00\x00") is None


def test_mavlink_rtcm_data_chunks_to_gps_rtcm_packets() -> None:
    from wiris_stereo_distance.pixhawk_attitude import _send_mavlink_rtcm_data

    class FakeMav:
        def __init__(self) -> None:
            self.calls: list[tuple[int, int, list[int]]] = []

        def gps_rtcm_data_send(self, flags: int, length: int, data: list[int]) -> None:
            self.calls.append((flags, length, data))

    class FakeConnection:
        def __init__(self) -> None:
            self.mav = FakeMav()

    connection = FakeConnection()
    payload = bytes(range(256))[:200]

    assert _send_mavlink_rtcm_data(connection, payload) == 2
    assert [call[1] for call in connection.mav.calls] == [180, 20]
    assert connection.mav.calls[0][0] & 0x01
    assert connection.mav.calls[1][0] & 0x01
    assert bytes(connection.mav.calls[0][2][:180] + connection.mav.calls[1][2][:20]) == payload

    with pytest.raises(ValueError):
        _send_mavlink_rtcm_data(connection, b"x" * 721)


def test_gnss_base_rtcm_frame_routes_to_selected_rovers(monkeypatch) -> None:
    from PyQt5 import QtCore, QtWidgets

    import wiris_stereo_distance.gui as gui

    class FakeMavlink:
        def __init__(self) -> None:
            self.sent: list[tuple[str, bytes]] = []

        def send_rtcm_data(self, camera: str, payload: bytes) -> int:
            self.sent.append((camera, payload))
            return 1

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget.mavlink_pixhawk = FakeMavlink()
    widget._set_combo_data(widget.gnss_base_rtcm_target_combo, "both")
    frame = b"\xd3\x00\x03" + bytes((1005 >> 4, (1005 & 0x0F) << 4, 0x00)) + b"\x00\x00\x00"

    widget._handle_gnss_base_rtcm_frame(frame)

    assert widget.mavlink_pixhawk.sent == [("left", frame), ("right", frame)]
    assert "RTCM 9 bytes" in widget.gnss_base_status_labels["rtcm_injection"].text()
    assert "frames=1" in widget.gnss_base_status_labels["rtcm_receive"].text()
    assert "1005:1" in widget.gnss_base_status_labels["rtcm_messages"].text()
    assert "L frames=1 bytes=9 fail=0" in widget.gnss_base_status_labels["rtcm_injection_stats"].text()
    assert "R frames=1 bytes=9 fail=0" in widget.gnss_base_status_labels["rtcm_injection_stats"].text()
    assert "MAVLink packets=2" in widget.gnss_base_status_labels["rtcm_bandwidth"].text()

    widget.deleteLater()
    _ = app


def test_gnss_operation_mode_changes_visible_settings(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui
    from wiris_stereo_distance.gnss_operation import GnssOperationMode

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()

    widget._set_combo_data(widget.gnss_operation_mode_combo, GnssOperationMode.LOCAL_SURVEY_IN.value)
    assert not widget.gnss_base_config_group.isHidden()
    assert not widget.gnss_survey_config_group.isHidden()
    assert widget.gnss_network_config_group.isHidden()
    assert widget.gnss_fixed_config_group.isHidden()
    assert not widget.gnss_base_candidate_group.isHidden()
    assert not widget.gnss_survey_progress_group.isHidden()
    assert not widget.gnss_survey_quality_group.isHidden()
    assert not widget.gnss_base_survey_start_button.isHidden()
    assert widget.gnss_network_start_button.isHidden()

    widget._set_combo_data(widget.gnss_operation_mode_combo, GnssOperationMode.NETWORK_ASSISTED_FIXED_BASE.value)
    assert not widget.gnss_base_config_group.isHidden()
    assert widget.gnss_survey_config_group.isHidden()
    assert not widget.gnss_network_config_group.isHidden()
    assert not widget.gnss_fixed_config_group.isHidden()
    assert not widget.gnss_base_candidate_group.isHidden()
    assert widget.gnss_survey_progress_group.isHidden()
    assert widget.gnss_survey_quality_group.isHidden()
    assert widget.gnss_base_survey_start_button.isHidden()
    assert not widget.gnss_network_start_button.isHidden()

    widget._set_combo_data(widget.gnss_operation_mode_combo, GnssOperationMode.NETWORK_RTK_DIRECT.value)
    assert widget.gnss_base_config_group.isHidden()
    assert widget.gnss_survey_config_group.isHidden()
    assert not widget.gnss_network_config_group.isHidden()
    assert widget.gnss_fixed_config_group.isHidden()
    assert widget.gnss_base_candidate_group.isHidden()
    assert widget.gnss_survey_progress_group.isHidden()
    assert widget.gnss_survey_quality_group.isHidden()
    assert widget.gnss_base_connect_button.isHidden()
    assert not widget.gnss_network_start_button.isHidden()

    widget.close()
    widget.deleteLater()
    _ = app


def test_gnss_operation_mode_changes_status_label_visibility(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui
    from wiris_stereo_distance.gnss_operation import GnssOperationMode

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()

    widget._set_combo_data(widget.gnss_operation_mode_combo, GnssOperationMode.LOCAL_SURVEY_IN.value)
    assert not widget.gnss_base_fixed_progress_group.isHidden()
    assert not widget.gnss_base_status_labels["survey_in"].isHidden()
    assert not widget.gnss_base_status_title_labels["survey_in"].isHidden()

    widget._set_combo_data(widget.gnss_operation_mode_combo, GnssOperationMode.NETWORK_ASSISTED_FIXED_BASE.value)
    assert not widget.gnss_base_fixed_progress_group.isHidden()
    assert widget.gnss_base_status_labels["survey_in"].isHidden()
    assert widget.gnss_base_status_title_labels["survey_in"].isHidden()

    widget._set_combo_data(widget.gnss_operation_mode_combo, GnssOperationMode.NETWORK_RTK_DIRECT.value)
    assert widget.gnss_base_fixed_progress_group.isHidden()
    assert widget.gnss_base_status_labels["survey_in"].isHidden()
    assert widget.gnss_base_status_title_labels["survey_in"].isHidden()

    widget.close()
    widget.deleteLater()
    _ = app


def test_gnss_transition_routes_to_group_specific_label(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui
    from wiris_stereo_distance.gnss_operation import GnssOperationMode, GnssOperationState

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    # NETWORK_RTK_DIRECT has no Group B stage, so this pins the "operation"/
    # "base_fixed" untouched-by-Group-A assertion regardless of whatever
    # operation_mode the on-disk settings file last saved.
    widget._set_combo_data(widget.gnss_operation_mode_combo, GnssOperationMode.NETWORK_RTK_DIRECT.value)
    widget.gnss_operation_manager.transition(GnssOperationState.CONFIGURED, "test setup")
    widget.gnss_operation_manager.transition(GnssOperationState.INITIALIZING, "test setup")

    operation_before = widget.gnss_base_status_labels["operation"].text()
    base_fixed_before = widget.gnss_base_status_labels["base_fixed"].text()

    # Group A (NTRIP): the dedicated "network" label owns this, "operation"
    # and "base_fixed" must stay untouched in NETWORK_RTK_DIRECT (no Group B).
    widget._gnss_transition(GnssOperationState.NTRIP_CONNECTING, "caster 연결 중")
    assert widget.gnss_base_status_labels["operation"].text() == operation_before
    assert widget.gnss_base_status_labels["base_fixed"].text() == base_fixed_before

    widget.gnss_operation_manager.transition(GnssOperationState.NTRIP_ACTIVE, "test setup")

    # Group B (Base RTK-Fixed acquisition): only "base_fixed" should change.
    widget._gnss_transition(GnssOperationState.BASE_AS_ROVER_FLOAT, "Base가 NTRIP 보정 rover로 수렴 중")
    assert widget.gnss_base_status_labels["operation"].text() == operation_before
    assert "BASE_AS_ROVER_FLOAT" in widget.gnss_base_status_labels["base_fixed"].text()

    widget.deleteLater()
    _ = app


def test_gnss_transition_explains_base_fixed_wait_during_ntrip_handshake(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui
    from wiris_stereo_distance.gnss_operation import GnssOperationMode, GnssOperationState

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget._set_combo_data(widget.gnss_operation_mode_combo, GnssOperationMode.NETWORK_ASSISTED_FIXED_BASE.value)
    widget.gnss_operation_manager.transition(GnssOperationState.CONFIGURED, "test setup")
    widget.gnss_operation_manager.transition(GnssOperationState.INITIALIZING, "test setup")

    # Still waiting on the caster handshake: base_fixed should say so, not
    # just sit on the generic "대기 중" placeholder.
    widget._gnss_transition(GnssOperationState.NTRIP_CONNECTING, "caster 연결 중")
    assert "NTRIP 연결 대기 중" in widget.gnss_base_status_labels["base_fixed"].text()

    # Caster connected, but no Base position sample has arrived yet.
    widget._gnss_transition(GnssOperationState.NTRIP_ACTIVE, "caster 연결됨")
    assert "샘플 수신 대기" in widget.gnss_base_status_labels["base_fixed"].text()

    # Once a real sample arrives, _handle_gnss_base_position takes over.
    widget._gnss_transition(GnssOperationState.BASE_AS_ROVER_FLOAT, "Base가 NTRIP 보정 rover로 수렴 중")
    assert "BASE_AS_ROVER_FLOAT" in widget.gnss_base_status_labels["base_fixed"].text()

    widget.deleteLater()
    _ = app


def test_network_rtk_direct_routes_ntrip_rtcm_without_base_serial(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui
    from wiris_stereo_distance.gnss_operation import GnssOperationMode

    class FakeMavlink:
        def __init__(self) -> None:
            self.sent: list[tuple[str, bytes]] = []

        def send_rtcm_data(self, camera: str, payload: bytes) -> int:
            self.sent.append((camera, payload))
            return 1

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget.mavlink_pixhawk = FakeMavlink()
    widget._set_combo_data(widget.gnss_operation_mode_combo, GnssOperationMode.NETWORK_RTK_DIRECT.value)
    widget._set_combo_data(widget.gnss_base_rtcm_target_combo, "left")
    widget.gnss_active_operation_config = widget._build_gnss_operation_config()
    # Controls can only be changed programmatically while active.  The
    # in-flight route remains the latched direct-mode left endpoint.
    widget._set_combo_data(widget.gnss_operation_mode_combo, GnssOperationMode.NETWORK_ASSISTED_FIXED_BASE.value)
    widget._set_combo_data(widget.gnss_base_rtcm_target_combo, "right")
    frame = b"\xd3\x00\x03" + bytes((1077 >> 4, (1077 & 0x0F) << 4, 0x00)) + b"\x00\x00\x00"

    widget._handle_gnss_network_chunk(b"noise" + frame)

    assert widget.mavlink_pixhawk.sent == [("left", frame)]
    assert "NTRIP RTCM 1 frame" in widget.gnss_base_status_labels["network"].text()
    assert "frames=1" in widget.gnss_base_status_labels["rtcm_receive"].text()

    widget.deleteLater()
    _ = app


def test_legacy_ntrip_direct_password_is_runtime_only(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget.gnss_ntrip_username_edit.setText("field-user")
    widget.gnss_ntrip_password_edit.setText("field-secret")

    config = widget._build_gnss_operation_config()
    resolved = widget._resolve_ntrip_credentials(config.network)

    assert resolved.username == "field-user"
    assert resolved.password == "field-secret"
    assert resolved.source == "session"
    mapping = config.to_mapping()
    assert mapping["network"]["username"] == "field-user"
    assert "password" not in mapping["network"]

    widget.deleteLater()
    _ = app


def test_legacy_ntrip_worker_reuses_explicit_credentials(monkeypatch) -> None:
    import wiris_stereo_distance.gui as gui

    seen = []

    class FakeClient:
        def __init__(self, _config, *, username=None, password=None):
            seen.append((username, password))
            self.stats = None

        def connect(self, *, gga_sentence=None):
            raise RuntimeError("stop-after-first-attempt")

        def close(self):
            return None

        def wait_for_reconnect(self, stop_event):
            stop_event.set()
            return False

    monkeypatch.setattr(gui, "NtripClient", FakeClient)
    worker = gui.NtripNetworkWorker(
        config=gui.NetworkConfig(caster_host="caster", mountpoint="MOUNT"),
        username="field-user",
        password="field-secret",
    )

    worker.run()

    assert seen == [("field-user", "field-secret"), ("field-user", "field-secret")]


def test_network_assisted_fixed_base_promotes_repeated_fixed_coordinate(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui
    from wiris_stereo_distance.gnss_operation import (
        CorrectionSourceIdentity,
        GnssOperationMode,
        GnssOperationState,
        HeightType,
    )
    from wiris_stereo_distance.gnss_profile import GnssCoordinateSample

    class FakeWorker:
        def __init__(self) -> None:
            self.sent: list[tuple[str, bytes]] = []

        def send_bytes(self, label: str, payload: bytes) -> None:
            self.sent.append((label, payload))

    class FakeMavlink:
        def __init__(self) -> None:
            self.sent: list[tuple[str, bytes]] = []

        def send_rtcm_data(self, camera: str, payload: bytes) -> int:
            self.sent.append((camera, payload))
            return 1

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget._set_combo_data(widget.gnss_operation_mode_combo, GnssOperationMode.NETWORK_ASSISTED_FIXED_BASE.value)
    widget._set_combo_data(widget.gnss_base_rtcm_target_combo, "left")
    widget.gnss_fixed_height_combo.setCurrentIndex(0)
    widget.gnss_fixed_acquisition_duration_spin.setValue(1)
    widget.gnss_fixed_min_duration_spin.setValue(1)
    widget.gnss_base_serial_worker = FakeWorker()
    widget.gnss_base_serial_connected = True
    widget.gnss_network_connected = True
    widget.mavlink_pixhawk = FakeMavlink()
    config = widget._build_gnss_operation_config()
    widget._gnss_configure_manager(config)
    widget.gnss_operation_manager.set_source_identity(CorrectionSourceIdentity(source_kind="test-ntrip"))
    widget.gnss_operation_manager.transition(GnssOperationState.INITIALIZING, "test")
    widget.gnss_operation_manager.transition(GnssOperationState.NTRIP_CONNECTING, "test")
    widget.gnss_operation_manager.transition(GnssOperationState.NTRIP_ACTIVE, "test")

    for timestamp_s in (0.0, 1.0, 2.0):
        widget._handle_gnss_base_position(
            GnssCoordinateSample(
                latitude_deg=37.0,
                longitude_deg=127.0,
                ellipsoid_height_m=52.0,
                msl_altitude_m=48.0,
                height_type=HeightType.ELLIPSOID,
                timestamp_s=timestamp_s,
                fixed=True,
            )
        )

    labels = [label for label, _payload in widget.gnss_base_serial_worker.sent]
    assert "CFG-TMODE3 fixed coordinate" in labels
    assert widget.gnss_fixed_base_profile is None
    assert widget.gnss_fixed_base_command_sent is True
    assert widget.gnss_operation_manager.state == GnssOperationState.BASE_RECONFIGURING
    position_text = widget.gnss_base_status_labels["base_position"].text()
    assert "37.0000000" in position_text
    assert "127.0000000" in position_text
    assert "fix=True" in position_text

    recorded = list(widget.gnss_base_recording_rows)
    assert len(recorded) == 3
    assert recorded[-1]["latitude_deg"] == 37.0
    assert recorded[-1]["longitude_deg"] == 127.0
    assert recorded[-1]["ellipsoid_height_m"] == 52.0
    assert recorded[-1]["height_type"] == "ELLIPSOID"
    assert recorded[-1]["fixed"] is True
    assert recorded[-1]["side"] == "base"
    assert recorded[-1]["sample_type"] == "gnss_base_position"
    assert isinstance(recorded[-1]["host_receive_utc_ns"], int)

    fix_series = [p.value for p in widget.gnss_base_fixed_state_plot.points if p.series == "fix"]
    assert fix_series == [1.0, 1.0, 1.0]
    scatter_h_series = [p.value for p in widget.gnss_base_fixed_scatter_plot.points if p.series == "horizontal"]
    assert len(scatter_h_series) >= 1
    assert recorded[-1]["host_receive_utc_ns"] > 0

    widget._handle_gnss_base_ubx_message(0x05, 0x01, b"\x06\x71")
    labels = [label for label, _payload in widget.gnss_base_serial_worker.sent]
    assert "CFG-TMODE3 poll after fixed coordinate ACK" in labels
    fixed_frame = next(payload for label, payload in widget.gnss_base_serial_worker.sent if label == "CFG-TMODE3 fixed coordinate")
    _message_class, _message_id, fixed_payload = gui.UbxStreamParser().feed(fixed_frame)[0]
    widget._handle_gnss_base_ubx_message(0x06, 0x71, fixed_payload)
    assert widget.gnss_operation_manager.state == GnssOperationState.BASE_RECONFIGURING

    frame = b"\xd3\x00\x03" + bytes((1005 >> 4, (1005 & 0x0F) << 4, 0x00)) + b"\x00\x00\x00"
    widget._handle_gnss_base_rtcm_frame(frame)
    assert widget.mavlink_pixhawk.sent == [("left", frame)]
    assert widget.gnss_fixed_base_profile is not None
    assert widget.gnss_operation_manager.state == GnssOperationState.LOCAL_RTCM_ACTIVE

    widget.deleteLater()
    _ = app


def test_gnss_base_tab_shows_rover_position(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui
    from wiris_stereo_distance.rtk_geometry import GpsPositionState

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget.latest_gps["left"] = GpsPositionState(
        "left", "/uav_0", 10.0, "gps", 37.401234, 127.123456, 10.0,
        fix_type=6, altitude_ellipsoid_m=52.5,
    )

    widget._refresh_gnss_base_tab()

    text = widget.gnss_base_status_labels["left_rover"].text()
    assert "37.4012340" in text
    assert "127.1234560" in text
    assert "h=52.500m" in text

    widget.deleteLater()
    _ = app


def test_gnss_base_satellite_quality_updates_diagnostic_labels(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()

    widget._handle_gnss_base_satellite_quality(
        {
            "tracked": 19,
            "used": 12,
            "avg_cno_dbhz": 36.5,
            "low_cno": 2,
            "low_elevation": 3,
            "constellations": {"GPS": 6, "Galileo": 5, "BeiDou": 4, "GLONASS": 4},
        }
    )

    assert widget.gnss_base_quality_labels["satellites"].text() == "used=12 tracked=19"
    assert "avg C/N0=36.5 dBHz" in widget.gnss_base_quality_labels["signal"].text()
    assert "low<30=2" in widget.gnss_base_quality_labels["signal"].text()
    assert widget.gnss_base_quality_labels["elevation"].text() == "low<15deg=3"
    assert "GPS 6" in widget.gnss_base_quality_labels["constellations"].text()
    assert "GLONASS 4" in widget.gnss_base_quality_labels["constellations"].text()

    widget.deleteLater()
    _ = app


def test_nmea_gga_satellite_count_parser_reads_base_fallback() -> None:
    import wiris_stereo_distance.gui as gui

    line = "$GNGGA,005621.90,3658.07765,N,12752.45633,E,2,12,0.89,118.4,M,20.7,M,,0137*49"

    assert gui.parse_nmea_gga_satellite_count(line) == (12, 2)
    assert gui.parse_nmea_gga_satellite_count("$GNVTG,,T,,M,0.012,N,0.023,K,D*3A") is None
    assert gui.parse_nmea_gga_satellite_count(line.replace("*49", "*00")) is None


def test_ros2_attitude_sample_age_s_requires_a_fresh_received_sample(monkeypatch) -> None:
    import time
    from types import SimpleNamespace

    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()

    assert widget._ros2_attitude_sample_age_s("left") is None

    widget.pixhawk_ros2_subscribers["left"] = SimpleNamespace(latest_extrinsics={})
    assert widget._ros2_attitude_sample_age_s("left") is None

    now_s = time.monotonic()
    widget.pixhawk_ros2_subscribers["left"] = SimpleNamespace(
        latest_extrinsics={"left": SimpleNamespace(timestamp_s=now_s - 0.5)}
    )
    age_s = widget._ros2_attitude_sample_age_s("left")
    assert age_s is not None
    assert 0.0 <= age_s < 2.0

    widget.pixhawk_ros2_subscribers["left"] = SimpleNamespace(
        latest_extrinsics={"left": SimpleNamespace(timestamp_s=now_s - 10.0)}
    )
    assert widget._ros2_attitude_sample_age_s("left") is None

    widget.deleteLater()
    _ = app


def test_verify_ros2_pixhawk_publishers_distinguishes_publisher_from_live_data(monkeypatch) -> None:
    import time
    from types import SimpleNamespace

    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    monkeypatch.setattr(gui, "_ros2_topic_has_publisher", lambda *args, **kwargs: True)
    widget.pixhawk_ros2_subscribers["left"] = SimpleNamespace(latest_extrinsics={})

    widget._verify_ros2_pixhawk_publishers(("left",))
    assert "publisher만 확인됨" in widget.pixhawk_connection_label.text()
    assert "자세 데이터 미수신" in widget.pixhawk_connection_label.text()

    widget.pixhawk_ros2_subscribers["left"] = SimpleNamespace(
        latest_extrinsics={"left": SimpleNamespace(timestamp_s=time.monotonic())}
    )
    widget._verify_ros2_pixhawk_publishers(("left",))
    assert "자세 데이터 수신 확인됨" in widget.pixhawk_connection_label.text()

    widget.deleteLater()
    _ = app


def test_gnss_base_candidate_rows_apply_selected_port(monkeypatch) -> None:
    from PyQt5 import QtCore, QtWidgets

    import wiris_stereo_distance.gui as gui

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()

    widget._set_gnss_base_candidate_rows(
        [
            {
                "port": "/dev/serial/by-id/usb-F9P",
                "target": "/dev/ttyACM0",
                "baud": 460800,
                "status": "UBX detected",
                "ubx": True,
                "messages": "NAV-SVIN, NAV-PVT sats=17",
            }
        ]
    )
    widget.gnss_base_candidate_table.selectRow(0)
    widget.apply_selected_gnss_base_candidate()

    assert widget.gnss_base_serial_port_edit.text() == "/dev/serial/by-id/usb-F9P"
    assert widget.gnss_base_baud_spin.value() == 460800
    widget._save_current_settings = lambda: None
    widget.close()
    app.processEvents()


def test_workstation_system_settings_bridge_preserves_legacy_owner_contract(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui
    from wiris_stereo_distance.settings import AppSettings

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    monkeypatch.setattr(gui, "load_settings", lambda: AppSettings())
    saved: list[AppSettings] = []
    monkeypatch.setattr(gui, "save_settings", lambda settings: saved.append(settings))

    widget = gui.WirisStereoDistancePanel()
    operation = AppSettings().gnss_operation.to_mapping()
    operation["operation_mode"] = "NETWORK_RTK_DIRECT"
    operation["readiness_gate_enabled"] = False
    operation["network"] = {
        **operation["network"],
        "caster_host": "caster.example",
        "tls": True,
        "read_timeout_s": 22.0,
    }
    operation["readiness"] = {
        **operation["readiness"],
        "maximum_baseline_std_m": 0.1234,
    }
    operation["fixed_base"] = {
        **operation["fixed_base"],
        "coordinate_profile_path": "gnss/base.json",
    }
    widget.apply_workstation_system_settings(
        {
            "interface_name": "enx-workstation",
            "yolo_device": "cuda:0",
            "pixhawk_ethernet_mode": "mavlink",
            "intrinsics_path": "custom/intrinsics.json",
            "auto_load_intrinsics": False,
            "manual_focal_override": True,
            "use_triangulation": False,
            "measurement_display_mode": "preview",
            "use_rtk_baseline": False,
            "require_rtk_fixed": True,
            "gnss_operation": operation,
        }
    )

    assert widget.interface_edit.text() == "enx-workstation"
    assert widget.device_edit.text() == "cuda:0"
    assert widget.pixhawk_ethernet_mode_combo.currentData() == "mavlink"
    assert widget.gnss_operation_mode_combo.currentData() == "NETWORK_RTK_DIRECT"
    built_config = widget._build_gnss_operation_config()
    assert built_config.network.caster_host == "caster.example"
    assert built_config.network.tls is True
    assert built_config.readiness_gate_enabled is False

    widget._save_current_settings()
    assert saved
    settings = saved[-1]
    assert settings.intrinsics_path == "custom/intrinsics.json"
    assert settings.auto_load_intrinsics is False
    assert settings.manual_focal_override is True
    assert settings.use_triangulation is False
    assert settings.measurement_display_mode == "preview"
    assert settings.use_rtk_baseline is False
    assert settings.require_rtk_fixed is True
    assert settings.gnss_operation.network.tls is True
    assert settings.gnss_operation.network.read_timeout_s == 22.0
    assert settings.gnss_operation.fixed_base.coordinate_profile_path == "gnss/base.json"
    assert settings.gnss_operation.fixed_base.height_type.value == "UNKNOWN"

    widget.deleteLater()
    app.processEvents()


def test_gnss_base_survey_in_command_status_moves_past_waiting(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui
    from wiris_stereo_distance.settings import AppSettings

    class FakeWorker:
        def __init__(self) -> None:
            self.sent: list[tuple[str, bytes]] = []

        def send_bytes(self, label: str, payload: bytes) -> None:
            self.sent.append((label, payload))

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    # Keep this command-path characterization test independent of the operator's
    # machine-local JSON, which may intentionally select NETWORK_RTK_DIRECT.
    monkeypatch.setattr(gui, "load_settings", lambda: AppSettings())
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    worker = FakeWorker()
    widget.gnss_base_serial_worker = worker

    widget.start_gnss_base_survey_in()

    assert [label for label, _payload in worker.sent] == [
        "CFG-TMODE3 survey-in start",
        "NAV-SVIN poll after survey-in start",
    ]
    assert "전송 대기" in widget.gnss_base_status_labels["survey_in"].text()

    widget._handle_gnss_base_command_sent("CFG-TMODE3 survey-in start", len(worker.sent[0][1]))

    assert "전송 완료" in widget.gnss_base_status_labels["survey_in"].text()
    assert "NAV-SVIN 상태 수신 대기" in widget.gnss_base_status_labels["survey_in"].text()

    worker.sent.clear()
    widget.stop_gnss_base_survey_in()
    assert [label for label, _payload in worker.sent] == [
        "CFG-TMODE3 disabled",
        "NAV-SVIN poll after survey-in stop",
    ]

    widget._handle_gnss_base_command_sent("CFG-TMODE3 disabled", len(worker.sent[0][1]))
    assert "중지 명령 전송 완료" in widget.gnss_base_status_labels["survey_in"].text()

    widget.deleteLater()
    _ = app


def test_gnss_base_survey_in_visualization_updates_from_nav_svin(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    scroll_area = widget.findChild(QtWidgets.QScrollArea, "gnssBaseScrollArea")
    survey_group = widget.findChild(QtWidgets.QGroupBox, "gnssBaseSurveyProgressGroup")
    connection_group = widget.findChild(QtWidgets.QGroupBox, "gnssBaseConnectionGroup")
    assert scroll_area is not None
    assert scroll_area.widgetResizable()
    assert survey_group is not None
    assert connection_group is not None
    assert survey_group.minimumHeight() >= 320
    assert connection_group.minimumHeight() >= 190
    widget.gnss_base_survey_duration_spin.setValue(100)
    widget.gnss_base_survey_accuracy_spin.setValue(0.25)
    monotonic_times = iter((100.0, 160.0, 220.0))
    monkeypatch.setattr(gui.time, "monotonic", lambda: next(monotonic_times))

    widget._handle_gnss_base_survey_in(
        {
            "active": True,
            "valid": False,
            "duration_s": 25,
            "mean_accuracy_m": 0.5,
            "observations": 42,
            "mean_x_m": 1.0,
            "mean_y_m": 2.0,
            "mean_z_m": 3.0,
        }
    )

    assert widget.gnss_base_survey_progress_bar.value() == 25
    assert widget.gnss_base_survey_progress_bar.text() == "active | time 25% | accuracy pending"
    assert "수렴: collecting trend" in widget.gnss_base_survey_convergence_label.text()
    assert "current=0.5000m" in widget.gnss_base_survey_accuracy_label.text()
    assert "obs=42" in widget.gnss_base_survey_accuracy_label.text()
    assert any(point.series == "accuracy" and point.value == 0.5 for point in widget.gnss_base_survey_accuracy_plot.points)
    assert any(point.series == "target" and point.value == 0.25 for point in widget.gnss_base_survey_accuracy_plot.points)
    assert any(
        point.series == "observations" and point.value == 42.0
        for point in widget.gnss_base_survey_observation_plot.points
    )

    widget._handle_gnss_base_survey_in(
        {
            "active": True,
            "valid": False,
            "duration_s": 150,
            "mean_accuracy_m": 0.5,
            "observations": 60,
        }
    )

    assert widget.gnss_base_survey_progress_bar.value() == 100
    assert widget.gnss_base_survey_progress_bar.text() == "active | time ok | accuracy pending"
    assert "수렴: stagnant 60s" in widget.gnss_base_survey_convergence_label.text()

    widget._handle_gnss_base_survey_in(
        {
            "active": False,
            "valid": True,
            "duration_s": 150,
            "mean_accuracy_m": 0.2,
            "observations": 80,
        }
    )

    assert widget.gnss_base_survey_progress_bar.value() == 100
    assert widget.gnss_base_survey_progress_bar.text() == "valid | time ok | accuracy ok"
    assert "수렴: complete" in widget.gnss_base_survey_convergence_label.text()

    widget.deleteLater()
    _ = app


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


def test_camera_rtc_required_rejects_matches_without_rtcp_timestamp() -> None:
    left = detection("left", x1=500, y1=200, x2=540, y2=260, t=10.0)
    right = detection("right", x1=450, y1=202, x2=490, y2=262, t=10.008)
    calibration = StereoCalibration(
        focal_length_px=900,
        baseline_m=0.3,
        max_time_delta_s=0.05,
        timestamp_basis="camera_rtc_required",
    )

    assert match_stereo_observations(left, right, calibration) is None

    left = detection(
        "left",
        x1=500,
        y1=200,
        x2=540,
        y2=260,
        t=10.0,
        timestamp_metadata={"rtcp_frame_ntp_unix_s": 1_800_000_000.000},
    )
    right = detection(
        "right",
        x1=450,
        y1=202,
        x2=490,
        y2=262,
        t=10.008,
        timestamp_metadata={"rtcp_frame_ntp_unix_s": 1_800_000_000.008},
    )

    match = match_stereo_observations(left, right, calibration)

    assert match is not None
    assert abs(match.time_delta_s - 0.008) < 1e-6

    left = detection(
        "left",
        x1=500,
        y1=200,
        x2=540,
        y2=260,
        t=10.0,
        timestamp_metadata={
            "rtcp_frame_ntp_unix_s": 1_800_000_000.000,
            "rtcp_timing_invalid_reason": "sender-report-ntp-pc-offset",
        },
    )
    right = detection(
        "right",
        x1=450,
        y1=202,
        x2=490,
        y2=262,
        t=10.008,
        timestamp_metadata={"rtcp_frame_ntp_unix_s": 1_800_000_000.008},
    )

    # An absolute PC-UTC failure does not erase a usable relative RTCP pair.
    match = match_stereo_observations(left, right, calibration)

    assert match is not None
    assert abs(match.time_delta_s - 0.008) < 1e-6


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


def test_camera_pane_overlays_latest_detection_bbox_on_live_preview() -> None:
    from PyQt5 import QtGui, QtWidgets

    from wiris_stereo_distance.camera_pane import CameraPane
    from wiris_stereo_distance.stereo import DetectionObservation

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    pane = CameraPane("left")
    pane.resize(400, 300)
    image = QtGui.QImage(640, 480, QtGui.QImage.Format_RGB888)
    image.fill(QtGui.QColor("black"))
    pane.latest_image = image
    pane._refresh_pixmap()
    without_box = pane.image_label.pixmap().toImage().copy()

    pane.latest_detection = DetectionObservation(
        camera="left",
        timestamp_s=1.0,
        width=640,
        height=480,
        x1=100.0,
        y1=100.0,
        x2=300.0,
        y2=300.0,
        confidence=0.9,
        class_id=0,
        class_name="person",
    )
    pane._refresh_pixmap()
    with_box = pane.image_label.pixmap().toImage().copy()

    assert with_box != without_box

    pane.stop()
    pane.deleteLater()
    _ = app


def test_legacy_camera_preview_inbox_renders_only_latest_frame() -> None:
    from PyQt5 import QtGui, QtWidgets

    from wiris_stereo_distance.camera_pane import CameraPane

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    pane = CameraPane("left")
    pane.stream_generation = 3
    first = QtGui.QImage(8, 6, QtGui.QImage.Format_RGB888)
    first.fill(QtGui.QColor(10, 0, 0))
    latest = QtGui.QImage(8, 6, QtGui.QImage.Format_RGB888)
    latest.fill(QtGui.QColor(90, 0, 0))

    pane._submit_preview_frame_for_generation(3, first, {"sequence": 1})
    pane._submit_preview_frame_for_generation(3, latest, {"sequence": 2})
    pane._drain_preview_frame()

    assert pane.current_image().pixelColor(0, 0).red() == 90
    pane.deleteLater()
    _ = app


def test_camera_pane_does_not_overlay_bbox_while_reviewing_measurement_snapshot() -> None:
    from PyQt5 import QtGui, QtWidgets

    from wiris_stereo_distance.camera_pane import CameraPane
    from wiris_stereo_distance.stereo import DetectionObservation

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    pane = CameraPane("left")
    pane.resize(400, 300)
    snapshot = QtGui.QImage(640, 480, QtGui.QImage.Format_RGB888)
    snapshot.fill(QtGui.QColor("black"))
    stale_detection = DetectionObservation(
        camera="left",
        timestamp_s=1.0,
        width=640,
        height=480,
        x1=100.0,
        y1=100.0,
        x2=300.0,
        y2=300.0,
        confidence=0.9,
        class_id=0,
        class_name="person",
    )

    pane.latest_image = snapshot.copy()
    pane.latest_detection = None
    pane.measurement_pair_display_active = False
    pane._refresh_pixmap()
    plain_snapshot = pane.image_label.pixmap().toImage().copy()

    pane.latest_detection = stale_detection
    pane.measurement_pair_display_active = True
    pane._refresh_pixmap()
    reviewed_with_stale_detection_set = pane.image_label.pixmap().toImage().copy()

    assert reviewed_with_stale_detection_set == plain_snapshot

    pane.stop()
    pane.deleteLater()
    _ = app


def test_camera_pane_preview_stream_updates_after_measurement_pair_display() -> None:
    from PyQt5 import QtGui, QtWidgets

    from wiris_stereo_distance.camera_pane import CameraPane

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    pane = CameraPane("left")
    measurement_image = QtGui.QImage(16, 9, QtGui.QImage.Format_RGB888)
    preview_image = QtGui.QImage(16, 9, QtGui.QImage.Format_RGB888)
    measurement_image.fill(QtGui.QColor("green"))
    preview_image.fill(QtGui.QColor("blue"))

    pane.show_selected_measurement_frame(measurement_image)
    pane.detection_stream_active = False
    pane._show_frame(preview_image, {"timestamp_s": 1.0})

    assert not pane.measurement_pair_display_active
    assert QtGui.QColor(pane.current_image().pixel(0, 0)).name() == QtGui.QColor("blue").name()

    pane.deleteLater()
    _ = app


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


def test_mavlink_global_position_int_does_not_clobber_gps_fix_state() -> None:
    from types import SimpleNamespace

    from wiris_stereo_distance.pixhawk_attitude import DualPixhawkMavlinkSubscriber

    class FakeMavlinkMessage(SimpleNamespace):
        def get_type(self):
            return self._type

    subscriber = DualPixhawkMavlinkSubscriber(left_url="udpin:0.0.0.0:14550", right_url="")
    captured = []
    subscriber.gps_changed.connect(lambda camera, gps: captured.append((camera, gps)))
    subscriber._handle_mavlink_message(
        "left",
        FakeMavlinkMessage(
            _type="GPS_RAW_INT",
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
    subscriber._handle_mavlink_message(
        "left",
        FakeMavlinkMessage(
            _type="GLOBAL_POSITION_INT",
            lat=370000100,
            lon=1270000100,
            alt=123460,
            relative_alt=100000,
            vx=0,
            vy=0,
            vz=0,
            hdg=9020,
            time_boot_ms=1000,
        ),
    )

    assert len(captured) == 1
    _, gps = captured[0]
    assert gps.topic == "mavlink/GPS_RAW_INT"
    assert gps.fix_type == 6


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
    assert [(item[1], item[4], item[5]) for item in sent[:2]] == [
        (5, 0, ""),
        (1, 0, ""),
    ]
    assert {item[1] for item in sent[2:]} == {1}
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

    assert sent == ["", "", "listener vehicle_attitude\n"]


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
        "param set MAV_2_CONFIG 1000\n"
        "param set MAV_2_MODE 0\n"
        "param set MAV_2_BROADCAST 1\n"
        "param set MAV_2_REMOTE_PRT 14550\n"
        "param set MAV_2_UDP_PRT 14550\n"
        "param set MAV_2_RATE 2000000\n"
        "param save\n"
    )


def test_mavlink_duplicate_raw_packet_is_ignored() -> None:
    from types import SimpleNamespace

    from wiris_stereo_distance.pixhawk_attitude import DualPixhawkMavlinkSubscriber

    class Message:
        roll = 0.1
        pitch = 0.2
        yaw = 0.3
        time_boot_ms = 10

        def get_type(self):
            return "ATTITUDE"

        def get_msgbuf(self):
            return b"same-mavlink-packet"

    class Connection:
        def __init__(self):
            self.messages = [Message(), Message()]

        def recv_match(self, **_kwargs):
            return self.messages.pop(0) if self.messages else None

    subscriber = DualPixhawkMavlinkSubscriber(left_url="udp:fake", right_url="")
    subscriber.connections["left"] = Connection()
    attitudes = []
    subscriber.extrinsic_changed.connect(lambda _side, state: attitudes.append(state))

    subscriber.spin_once()

    assert len(attitudes) == 1


def test_legacy_param_read_defers_attitude_instead_of_dropping_it() -> None:
    from wiris_stereo_distance.pixhawk_attitude import DualPixhawkMavlinkSubscriber

    class Message:
        def __init__(self, message_type, **values):
            self._message_type = message_type
            self.__dict__.update(values)

        def get_type(self):
            return self._message_type

    class Mav:
        def param_request_read_send(self, *_args):
            return None

    class Connection:
        target_system = 1
        target_component = 1
        mav = Mav()

        def __init__(self):
            self.messages = [
                Message("ATTITUDE", roll=0.1, pitch=0.2, yaw=0.3, time_boot_ms=1),
                Message("PARAM_VALUE", param_id="TEST", param_value=3.5, param_type=9),
            ]

        def recv_match(self, **_kwargs):
            return self.messages.pop(0) if self.messages else None

    subscriber = DualPixhawkMavlinkSubscriber(left_url="udp:fake", right_url="")
    subscriber.connections["left"] = Connection()
    attitudes = []
    subscriber.extrinsic_changed.connect(lambda _side, state: attitudes.append(state))

    assert subscriber.read_param("left", "TEST", timeout_s=0.1) == pytest.approx(3.5)
    assert attitudes == []
    subscriber.spin_once()
    assert len(attitudes) == 1


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
        assert window.windowTitle() == gui.APP_WINDOW_TITLE == "StereoWIRIS"
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
    widget.ros2_pixhawk_candidate_table.selectRow(1)
    widget.apply_selected_pixhawk_candidate("left", source="ros2")

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
    assert widget.pixhawk_candidate_table.rowCount() == 2
    assert widget.pixhawk_candidate_table.item(0, 0).text() == "USB"
    assert widget.pixhawk_candidate_table.item(0, 1).text() == "/dev/ttyACM0"
    assert widget.pixhawk_candidate_table.item(0, 2).text() == "57600"
    assert widget.pixhawk_candidate_table.item(0, 3).text() == "MAVLink heartbeat"
    assert widget.pixhawk_candidate_table.item(1, 0).text() == "MAVLink"
    assert widget.pixhawk_candidate_table.item(1, 1).text() == "/dev/ttyACM0"
    assert widget.pixhawk_candidate_table.item(1, 2).text() == "57600"
    assert widget.pixhawk_candidate_table.item(1, 3).text() == "USB serial direct"
    widget.pixhawk_candidate_table.selectRow(0)
    assert not widget.apply_pixhawk_candidate_host_button.isEnabled()
    assert widget.apply_pixhawk_candidate_left_button.isEnabled()
    assert widget.apply_pixhawk_candidate_right_button.isEnabled()
    assert "USB 부트스트랩" in widget.apply_pixhawk_candidate_left_button.toolTip()
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

    assert widget.pixhawk_candidate_table.rowCount() == 6
    assert widget.pixhawk_candidate_table.item(0, 0).text() == "MAVLink"
    assert widget.pixhawk_candidate_table.item(0, 1).text() == "udpin:0.0.0.0:14550"
    assert widget.pixhawk_candidate_table.item(0, 3).text() == "Ethernet UDP listen"
    assert widget.pixhawk_candidate_table.item(2, 1).text() == "tcp:10.0.0.2:5760"
    assert widget.pixhawk_candidate_table.item(3, 1).text() == "udpout:10.0.0.2:14550"
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

    ros2_kinds = [
        widget.ros2_pixhawk_candidate_table.item(row, 0).text()
        for row in range(widget.ros2_pixhawk_candidate_table.rowCount())
    ]
    ros2_values = [
        widget.ros2_pixhawk_candidate_table.item(row, 1).text()
        for row in range(widget.ros2_pixhawk_candidate_table.rowCount())
    ]
    assert ros2_kinds == ["ROS2"]
    assert ros2_values == ["uav_0"]
    assert widget.mavlink_pixhawk_candidate_table.rowCount() == 0

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
    widget.ros2_pixhawk_candidate_table.selectRow(0)

    assert not widget.apply_pixhawk_candidate_host_button.isEnabled()
    assert not widget.apply_pixhawk_candidate_left_button.isEnabled()
    assert not widget.apply_pixhawk_candidate_right_button.isEnabled()
    assert widget.apply_pixhawk_ros2_candidate_left_button.isEnabled()
    assert widget.apply_pixhawk_ros2_candidate_right_button.isEnabled()

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

    assert widget.right_pixhawk_bootstrap_url_edit.text() == "/dev/ttyACM0"
    assert widget.right_pixhawk_bootstrap_baud_spin.value() == 921600
    assert widget.right_mavlink_url_edit.text() != "/dev/ttyACM0"
    assert widget.mavlink_pixhawk is None
    assert "Pixhawk 2 USB 부트스트랩 후보 적용됨" in widget.pixhawk_connection_label.text()

    widget.deleteLater()
    _ = app


def test_usb_pixhawk_direct_mavlink_candidate_applies_url_and_baud(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    monkeypatch.setattr(gui, "usb_mavlink_device_candidates", lambda: ["/dev/serial/by-id/usb-PX4-left"])
    monkeypatch.setattr(
        gui,
        "probe_usb_mavlink_devices",
        lambda candidates, timeout_s=0.8: [("/dev/serial/by-id/usb-PX4-left", 921600)],
    )
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()

    widget.discover_usb_pixhawk_connections()
    assert widget.pixhawk_candidate_table.rowCount() == 2
    assert widget.pixhawk_candidate_table.item(1, 0).text() == "MAVLink"
    assert widget.pixhawk_candidate_table.item(1, 1).text() == "/dev/serial/by-id/usb-PX4-left"
    assert widget.pixhawk_candidate_table.item(1, 2).text() == "921600"

    widget.pixhawk_candidate_table.selectRow(1)
    widget.apply_selected_pixhawk_candidate("left")

    assert widget.left_mavlink_url_edit.text() == "/dev/serial/by-id/usb-PX4-left"
    assert widget.left_mavlink_baud_spin.value() == 921600
    assert "Pixhawk 1 MAVLink 연결을 누르세요" in widget.pixhawk_connection_label.text()

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

    assert stopped == []
    assert widget.right_pixhawk_bootstrap_url_edit.text() == "/dev/ttyACM0"
    assert widget.right_mavlink_url_edit.text() != "/dev/ttyACM0"
    assert "USB 부트스트랩" in widget.pixhawk_connection_label.text()

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
    widget.pixhawk_ethernet_mode_combo.setCurrentIndex(
        widget.pixhawk_ethernet_mode_combo.findData("mavlink")
    )
    widget.pixhawk_host_address_edit.setText("192.168.0.254")
    widget.right_pixhawk_eth_ip_edit.setText("192.168.0.3")
    widget.right_mavlink_url_edit.setText("/dev/ttyACM0")

    command = widget._persistent_pixhawk_eth_command("right")
    extras_text = widget._persistent_pixhawk_extras_text("right")
    net_cfg_text = widget._persistent_pixhawk_net_cfg_text("right")

    assert "ifconfig eth0 192.168.0.3 netmask 255.255.255.0 up" in command
    assert "param set MAV_2_CONFIG 1000" in command
    assert "param set UXRCE_DDS_CFG 0" in command
    assert "uxrce_dds_client start" not in command
    assert "mavlink stop -u 14551" not in command
    assert "mavlink start -x -u 14551 -o 14551 -t 192.168.0.254 -r 2000000 -m normal" not in command
    assert "mavlink status" in command
    assert "echo 'ifup eth0' > /fs/microsd/etc/extras.txt" in command
    assert "echo 'ifconfig eth0 192.168.0.3 netmask 255.255.255.0 up' >> /fs/microsd/etc/extras.txt" in command
    assert "cat /fs/microsd/etc/extras.txt" in command
    assert "ifconfig eth0 192.168.0.3 netmask 255.255.255.0 up" in extras_text
    assert "uxrce_dds_client start" not in extras_text
    assert "mavlink start" not in extras_text
    assert "echo '" not in extras_text
    assert "IPADDR=192.168.0.3" in net_cfg_text
    assert "ROUTER=192.168.0.254" in net_cfg_text
    assert "BOOTPROTO=static" in net_cfg_text

    widget.deleteLater()
    _ = app


def test_persistent_pixhawk_eth_mavlink_only_mode_uses_px4_ethernet_config(monkeypatch) -> None:
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
    widget.pixhawk_ethernet_mode_combo.setCurrentIndex(
        widget.pixhawk_ethernet_mode_combo.findData("mavlink")
    )
    widget.left_pixhawk_eth_ip_edit.setText("10.0.0.2")
    widget.left_mavlink_url_edit.setText("udpin:0.0.0.0:14550")

    command = widget._persistent_pixhawk_eth_command("left")
    extras_text = widget._persistent_pixhawk_extras_text("left")

    assert "param set MAV_2_CONFIG 1000" in command
    assert "param set UXRCE_DDS_CFG 0" in command
    assert "uxrce_dds_client start" not in command
    assert "mavlink start" not in extras_text
    assert "ifconfig eth0 10.0.0.2 netmask 255.255.255.0 up" in extras_text

    widget.deleteLater()
    _ = app


def test_usb_bootstrap_starts_ethernet_mavlink_without_changing_udp_url(monkeypatch) -> None:
    from ipaddress import ip_interface
    from types import SimpleNamespace

    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    class FakeSignal:
        def connect(self, _callback) -> None:
            return None

    class FakeMavlink:
        status_changed = FakeSignal()

        def __init__(self, **kwargs) -> None:
            created.append(kwargs)
            self.connections = {}

        def start(self, camera=None) -> None:
            started.append(camera)
            self.connections[camera] = object()

        def send_shell_command_with_result(self, camera, command, *, timeout_s=2.0):
            sent.append((camera, command))
            return SimpleNamespace(output="", flag_label="respond+exclusive")

        def stop(self, camera=None) -> None:
            stopped.append(camera)

    created = []
    started = []
    sent = []
    stopped = []
    reconnected = []
    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    monkeypatch.setattr(gui, "DualPixhawkMavlinkSubscriber", FakeMavlink)
    monkeypatch.setattr(
        gui,
        "local_ipv4_interfaces",
        lambda **_kwargs: [SimpleNamespace(name="pixhawk0", address=ip_interface("10.0.0.10/24"))],
    )
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget._save_current_settings = lambda: None
    widget.connect_mavlink_pixhawk = lambda camera=None, force_setup=False: reconnected.append((camera, force_setup))
    widget.pixhawk_host_address_edit.setText("10.0.0.10")
    widget.left_pixhawk_eth_ip_edit.setText("10.0.0.2")
    widget.left_mavlink_url_edit.setText("udpin:0.0.0.0:14550")
    widget.left_pixhawk_bootstrap_url_edit.setText("/dev/serial/by-id/usb-PX4-left")
    widget.left_pixhawk_bootstrap_baud_spin.setValue(921600)

    widget.bootstrap_ethernet_mavlink_from_usb("left")

    assert created == [
        {
            "left_url": "/dev/serial/by-id/usb-PX4-left",
            "right_url": "",
            "left_baud": 921600,
            "right_baud": 921600,
        }
    ]
    assert started == ["left"]
    assert stopped == ["left"]
    assert reconnected == [("left", False)]
    assert widget.left_mavlink_url_edit.text() == "udpin:0.0.0.0:14550"
    assert sent and sent[0][0] == "left"
    command = sent[0][1]
    assert "ifconfig eth0 10.0.0.2 netmask 255.255.255.0 up" in command
    assert "param set MAV_2_CONFIG 1000" in command
    assert "param set MAV_2_REMOTE_PRT 14550" in command
    assert "mavlink stop" not in command
    assert "mavlink start" not in command
    assert command.endswith("reboot")

    widget.deleteLater()
    _ = app


def test_usb_bootstrap_fast_baud_and_onboard_mode_fallback(monkeypatch) -> None:
    from ipaddress import ip_interface
    from types import SimpleNamespace

    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    class FakeSignal:
        def connect(self, _callback) -> None:
            return None

    class FakeConnection:
        def __init__(self, ok: bool) -> None:
            self.ok = ok

        def wait_heartbeat(self, timeout=1.2):
            return object() if self.ok else None

    class FakeMavlink:
        status_changed = FakeSignal()

        def __init__(self, **kwargs) -> None:
            self.baud = kwargs["left_baud"]
            self.connections = {}
            created.append(kwargs)

        def start(self, camera=None) -> None:
            started.append((camera, self.baud))
            self.connections[camera] = FakeConnection(self.baud == 57600)

        def send_shell_command_with_result(self, camera, command, *, timeout_s=2.0):
            sent.append((camera, self.baud, command))
            if "-m normal" in command:
                return SimpleNamespace(output="ERROR [mavlink] invalid mode", flag_label="respond")
            return SimpleNamespace(output="INFO [mavlink] mode: Onboard", flag_label="respond")

        def stop(self, camera=None) -> None:
            stopped.append((camera, self.baud))

    created = []
    started = []
    sent = []
    stopped = []
    reconnected = []
    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    monkeypatch.setattr(gui, "DualPixhawkMavlinkSubscriber", FakeMavlink)
    monkeypatch.setattr(
        gui,
        "local_ipv4_interfaces",
        lambda **_kwargs: [SimpleNamespace(name="pixhawk0", address=ip_interface("10.0.0.10/24"))],
    )
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget._save_current_settings = lambda: None
    widget.connect_mavlink_pixhawk = lambda camera=None, force_setup=False: reconnected.append((camera, force_setup))
    widget.pixhawk_host_address_edit.setText("10.0.0.10")
    widget.left_pixhawk_eth_ip_edit.setText("10.0.0.2")
    widget.left_mavlink_url_edit.setText("udpin:0.0.0.0:14550")
    widget.left_pixhawk_bootstrap_url_edit.setText("/dev/serial/by-id/usb-PX4-left")
    widget.left_pixhawk_bootstrap_baud_spin.setValue(115200)

    widget.bootstrap_ethernet_mavlink_from_usb("left")

    assert [entry["left_baud"] for entry in created] == [115200, 57600]
    assert started == [("left", 115200), ("left", 57600)]
    assert stopped == [("left", 115200), ("left", 57600)]
    assert widget.left_pixhawk_bootstrap_baud_spin.value() == 57600
    assert reconnected == [("left", False)]
    assert len(sent) == 2
    assert "-m normal" in sent[0][2]
    assert "-m onboard -n eth0" in sent[1][2]

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
    assert widget.pc_pixhawk_candidate_table.rowCount() == 2
    assert widget.pc_pixhawk_candidate_table.item(0, 0).text() == "PC_IP"
    assert widget.pc_pixhawk_candidate_table.item(0, 1).text() == "10.0.0.10"
    assert widget.pc_pixhawk_candidate_table.item(0, 4).text() == "enp46s0"
    assert widget.pc_pixhawk_candidate_table.item(1, 1).text() == "192.168.0.38"
    assert "Wi-Fi" in widget.pc_pixhawk_candidate_table.item(1, 3).text()

    widget.pc_pixhawk_candidate_table.selectRow(0)
    assert widget.apply_pixhawk_candidate_host_button.isEnabled()
    assert not widget.apply_pixhawk_candidate_left_button.isEnabled()
    assert not widget.apply_pixhawk_candidate_right_button.isEnabled()
    widget.apply_selected_pixhawk_candidate("host", source="pc")

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
    assert widget.detect_pixhawk_host_ip_button.parentWidget() is widget.pixhawk_pc_candidate_group

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

    from PyQt5 import QtCore, QtWidgets

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
    assert widget.runtime_camera_status_group.title() == ""
    assert widget.runtime_camera_status_group.layout().count() == 1
    assert widget.runtime_camera_status_group.minimumHeight() == widget.runtime_camera_status_group.maximumHeight()
    assert widget.runtime_camera_settings_group.layout().count() == 1
    assert widget.clear_history_button.text() == "그래프/기록 초기화"
    assert widget.runtime_camera_summary_label.parentWidget().title() == ""
    assert widget.left_runtime_stats_label.minimumHeight() == widget.left_runtime_stats_label.maximumHeight()
    assert widget.right_runtime_stats_label.minimumHeight() == widget.right_runtime_stats_label.maximumHeight()

    connection_tab_index = [widget.tabs.tabText(index) for index in range(widget.tabs.count())].index("연결")
    widget.tabs.setCurrentIndex(connection_tab_index)
    widget.setup_tabs.setCurrentIndex(widget.pixhawk_connection_tab_index)
    app.processEvents()
    pixhawk_buttons = [
        widget.left_mavlink_button,
        widget.right_mavlink_button,
        widget.left_mavlink_bootstrap_button,
        widget.right_mavlink_bootstrap_button,
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
    assert widget.swap_pixhawk_button.text() == "P1/P2 매핑 교체"
    assert widget.detect_pixhawk_host_ip_button.parentWidget().title() == "PC / Host IP"
    assert widget.pixhawk_auto_detect_button.parentWidget().title() == "ROS2 Namespace"
    assert widget.pixhawk_usb_discover_button.parentWidget().title() == "MAVLink / USB 제어"
    assert widget.pixhawk_mavlink_url_candidate_button.parentWidget().title() == "MAVLink / USB 제어"
    assert widget.pixhawk_usb_discover_button.text() == "USB 연결 탐색"
    assert not hasattr(widget, "usb_setup_target_combo")
    pixhawk_candidate_group = widget.findChild(QtWidgets.QGroupBox, "pixhawkCandidateGroup")
    assert pixhawk_candidate_group is not None
    assert widget.pixhawk_pc_candidate_group.title() == "PC / Host IP"
    assert widget.pixhawk_mavlink_candidate_group.title() == "MAVLink / USB 제어"
    assert widget.pixhawk_ros2_candidate_group.title() == "ROS2 Namespace"
    assert widget.pixhawk_pc_candidate_group.geometry().bottom() < widget.pixhawk_mavlink_candidate_group.geometry().top()
    assert widget.pixhawk_mavlink_candidate_group.geometry().bottom() < widget.pixhawk_ros2_candidate_group.geometry().top()
    candidate_group_widths = [
        widget.pixhawk_pc_candidate_group.geometry().width(),
        widget.pixhawk_mavlink_candidate_group.geometry().width(),
        widget.pixhawk_ros2_candidate_group.geometry().width(),
    ]
    assert max(candidate_group_widths) - min(candidate_group_widths) <= 1
    candidate_tables = [
        widget.pc_pixhawk_candidate_table,
        widget.mavlink_pixhawk_candidate_table,
        widget.ros2_pixhawk_candidate_table,
    ]
    for table in candidate_tables:
        header = table.horizontalHeader()
        assert header.sectionResizeMode(0) == QtWidgets.QHeaderView.Stretch
        assert table.height() >= 240
        widths = [table.columnWidth(column) for column in range(table.columnCount())]
        assert len(widths) == 5
        assert max(widths) - min(widths) <= 1
    assert widget.pixhawk_common_setup_group.title() == "0 공통 네트워크"
    assert widget.pixhawk_mavlink_rtcm_group.title() == "2 Rover MAVLink / RTCM"
    assert widget.pixhawk_pair_settings_group.title() == "2 Rover MAVLink / RTCM"
    assert widget.pixhawk_ros2_telemetry_group.title() == "3 ROS2 Telemetry"
    assert widget.pixhawk_connection_actions_group.title() == "3 ROS2 Telemetry"
    assert widget.pixhawk_advanced_diagnostics_group.title() == "4 고급 / 진단"
    left_pixhawk_group = widget.findChild(QtWidgets.QGroupBox, "leftPixhawkMavlinkWorkflowGroup")
    right_pixhawk_group = widget.findChild(QtWidgets.QGroupBox, "rightPixhawkMavlinkWorkflowGroup")
    left_ros2_group = widget.findChild(QtWidgets.QGroupBox, "leftPixhawkRos2WorkflowGroup")
    right_ros2_group = widget.findChild(QtWidgets.QGroupBox, "rightPixhawkRos2WorkflowGroup")
    assert left_pixhawk_group is not None
    assert right_pixhawk_group is not None
    assert left_ros2_group is not None
    assert right_ros2_group is not None
    assert left_pixhawk_group.title() == "Pixhawk 1 MAVLink"
    assert right_pixhawk_group.title() == "Pixhawk 2 MAVLink"
    assert left_ros2_group.title() == "Pixhawk 1 ROS2"
    assert right_ros2_group.title() == "Pixhawk 2 ROS2"
    assert abs(left_pixhawk_group.geometry().width() - right_pixhawk_group.geometry().width()) <= 1
    assert abs(left_ros2_group.geometry().width() - right_ros2_group.geometry().width()) <= 1
    assert widget.pixhawk_host_address_edit.parentWidget().title() == "0 공통 네트워크"
    assert widget.detect_pixhawk_host_ip_button.parentWidget().title() == "PC / Host IP"
    assert widget.mavlink_stream_rate_spin.parentWidget().title() == "2 Rover MAVLink / RTCM"
    assert widget.mavlink_configure_streams_checkbox.parentWidget().title() == "2 Rover MAVLink / RTCM"
    assert widget.domain_spin.parentWidget().title() == "3 ROS2 Telemetry"
    assert widget.uxrce_port_spin.parentWidget().title() == "3 ROS2 Telemetry"
    assert widget.mavlink_configure_uxrce_agent_checkbox.parentWidget().title() == "3 ROS2 Telemetry"
    left_pixhawk_form = left_pixhawk_group.layout()
    right_pixhawk_form = right_pixhawk_group.layout()
    left_ros2_form = left_ros2_group.layout()
    right_ros2_form = right_ros2_group.layout()
    assert left_pixhawk_form.itemAtPosition(0, 1).widget() is widget.left_mavlink_url_edit
    assert right_pixhawk_form.itemAtPosition(0, 1).widget() is widget.right_mavlink_url_edit
    assert left_pixhawk_form.itemAtPosition(1, 1).widget() is widget.left_mavlink_baud_spin
    assert right_pixhawk_form.itemAtPosition(1, 1).widget() is widget.right_mavlink_baud_spin
    assert left_pixhawk_form.itemAtPosition(2, 1).widget() is widget.left_pixhawk_bootstrap_url_edit
    assert right_pixhawk_form.itemAtPosition(2, 1).widget() is widget.right_pixhawk_bootstrap_url_edit
    assert left_pixhawk_form.itemAtPosition(3, 1).widget() is widget.left_pixhawk_bootstrap_baud_spin
    assert right_pixhawk_form.itemAtPosition(3, 1).widget() is widget.right_pixhawk_bootstrap_baud_spin
    assert left_pixhawk_form.itemAtPosition(4, 1).widget() is widget.left_pixhawk_eth_ip_edit
    assert right_pixhawk_form.itemAtPosition(4, 1).widget() is widget.right_pixhawk_eth_ip_edit
    assert left_ros2_form.itemAtPosition(0, 1).widget() is widget.left_namespace_edit
    assert right_ros2_form.itemAtPosition(0, 1).widget() is widget.right_namespace_edit
    assert widget.left_px4_shell_button.parentWidget().title() == "4 고급 / 진단"
    assert widget.right_px4_shell_button.parentWidget().title() == "4 고급 / 진단"
    assert widget.left_persist_eth_button.parentWidget().title() == "4 고급 / 진단"
    assert widget.right_persist_eth_button.parentWidget().title() == "4 고급 / 진단"
    assert widget.left_reboot_pixhawk_button.parentWidget().title() == "4 고급 / 진단"
    assert widget.right_reboot_pixhawk_button.parentWidget().title() == "4 고급 / 진단"
    assert widget.left_mavlink_button.parentWidget().title() == "2 Rover MAVLink / RTCM"
    assert widget.right_mavlink_button.parentWidget().title() == "2 Rover MAVLink / RTCM"
    assert widget.left_mavlink_bootstrap_button.parentWidget().title() == "2 Rover MAVLink / RTCM"
    assert widget.right_mavlink_bootstrap_button.parentWidget().title() == "2 Rover MAVLink / RTCM"
    assert widget.swap_pixhawk_button.parentWidget().title() == "0 공통 네트워크"
    swap_button_rect = QtCore.QRect(
        widget.swap_pixhawk_button.mapTo(widget.pixhawk_common_setup_group, QtCore.QPoint(0, 0)),
        widget.swap_pixhawk_button.rect().size(),
    )
    assert widget.pixhawk_common_setup_group.rect().contains(swap_button_rect)
    assert widget.left_ros2_button.parentWidget().title() == "3 ROS2 Telemetry"
    assert widget.right_ros2_button.parentWidget().title() == "3 ROS2 Telemetry"
    assert widget.pixhawk_button.parentWidget() is None
    assert widget.left_uxrce_param_button.parentWidget().title() == "3 ROS2 Telemetry"
    assert widget.right_uxrce_param_button.parentWidget().title() == "3 ROS2 Telemetry"
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
    status_height = widget.runtime_camera_status_group.height()
    label_height = widget.left_runtime_stats_label.height()
    widget.left_runtime_stats_label.setText(
        "왼쪽: 1280x720 20.0 Hz | 15.0 Hz | 연결 | live: RTCP 00:00:00.000Z"
    )
    app.processEvents()
    assert widget.runtime_camera_status_group.height() == status_height
    assert widget.left_runtime_stats_label.height() == label_height

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

    widget.pixhawk_ros2_subscribers["left"] = object()
    widget._sync_legacy_pixhawk_ref()
    widget.mavlink_pixhawk = SimpleNamespace(connections={"left": object()})
    widget._refresh_session_toggle_buttons()
    assert widget.pixhawk_button.text() == "ROS2 전체 해제"
    assert widget.left_ros2_button.text() == "P1 ROS2 해제"
    assert widget.right_ros2_button.text() == "P2 ROS2 연결"
    assert widget.left_mavlink_button.text() == "P1 MAVLink 해제"
    assert widget.right_mavlink_button.text() == "P2 MAVLink 연결"

    widget.deleteLater()
    _ = app


def test_mavlink_toggle_disconnects_open_socket_without_heartbeat(monkeypatch) -> None:
    from types import SimpleNamespace

    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    stopped = []

    class FakeMavlink:
        def __init__(self) -> None:
            self.connections = {"left": object()}
            self._heartbeat_logged = set()

        def stop(self, camera=None) -> None:
            stopped.append(camera)
            if camera is None:
                self.connections.clear()
            else:
                self.connections.pop(camera, None)

    widget.mavlink_pixhawk = FakeMavlink()
    widget.connect_mavlink_pixhawk = lambda camera=None, force_setup=False: (_ for _ in ()).throw(
        AssertionError("open MAVLink socket must disconnect instead of reconnecting")
    )

    widget.toggle_mavlink_connection("left")

    assert stopped == ["left"]
    assert widget.mavlink_pixhawk is None
    assert widget.left_mavlink_button.text() == "P1 MAVLink 연결"
    assert "MAVLink 연결을 해제했습니다" in widget.pixhawk_log.toPlainText()

    widget.deleteLater()
    _ = app


def test_mavlink_heartbeat_status_refreshes_side_action_buttons(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()

    class FakeMavlink:
        def __init__(self) -> None:
            self.connections = {"left": object()}
            self._heartbeat_logged = set()

    widget.mavlink_pixhawk = FakeMavlink()
    widget._refresh_mavlink_side_actions()
    assert not widget.left_px4_shell_button.isEnabled()
    assert not widget.left_uxrce_param_button.isEnabled()
    assert not widget.left_reboot_pixhawk_button.isEnabled()

    widget.mavlink_pixhawk._heartbeat_logged.add("left")
    widget._handle_mavlink_status_changed("왼쪽 MAVLink heartbeat 수신")

    assert widget.left_px4_shell_button.isEnabled()
    assert widget.left_uxrce_param_button.isEnabled()
    assert widget.left_reboot_pixhawk_button.isEnabled()
    assert not widget.right_px4_shell_button.isEnabled()
    assert "왼쪽 MAVLink heartbeat 수신" in widget.pixhawk_log.toPlainText()

    widget.deleteLater()
    _ = app


def test_ros2_connections_are_split_by_pixhawk_side(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    class FakeSignal:
        def __init__(self) -> None:
            self.slots = []

        def connect(self, slot) -> None:
            self.slots.append(slot)

    class FakeRos2Subscriber:
        def __init__(self, **kwargs) -> None:
            self.kwargs = kwargs
            self.extrinsic_changed = FakeSignal()
            self.gps_changed = FakeSignal()
            self.mag_sensor_changed = FakeSignal()
            self.gps_sensor_changed = FakeSignal()
            self.gnss_relative_changed = FakeSignal()
            self.custom_watch_changed = FakeSignal()
            self.status_changed = FakeSignal()
            self.stopped = False

        def start(self) -> None:
            pass

        def stop(self) -> None:
            self.stopped = True

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_ensure_micro_xrce_agent", lambda self: None)
    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_verify_ros2_pixhawk_publishers", lambda self, sides=None: None)
    monkeypatch.setattr(gui, "DualPixhawkAttitudeSubscriber", FakeRos2Subscriber)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget.left_namespace_edit.setText("uav_0")
    widget.right_namespace_edit.setText("uav_1")

    assert widget.connect_pixhawk("left") is True
    left_subscriber = widget.pixhawk_ros2_subscribers["left"]
    assert left_subscriber is not None
    assert left_subscriber.kwargs["active_sides"] == ("left",)
    assert widget.pixhawk_ros2_subscribers["right"] is None
    assert widget.left_ros2_button.text() == "P1 ROS2 해제"
    assert widget.right_ros2_button.text() == "P2 ROS2 연결"

    assert widget.connect_pixhawk("right") is True
    right_subscriber = widget.pixhawk_ros2_subscribers["right"]
    assert right_subscriber is not None
    assert right_subscriber.kwargs["active_sides"] == ("right",)

    widget.disconnect_pixhawk("left")
    assert left_subscriber.stopped
    assert widget.pixhawk_ros2_subscribers["left"] is None
    assert widget.pixhawk_ros2_subscribers["right"] is right_subscriber
    assert widget.left_ros2_button.text() == "P1 ROS2 연결"
    assert widget.right_ros2_button.text() == "P2 ROS2 해제"

    widget.deleteLater()
    _ = app


def test_camera_pane_stop_keeps_worker_owned_until_thread_finishes() -> None:
    from PyQt5 import QtWidgets

    from wiris_stereo_distance.camera_pane import CameraPane

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    pane = CameraPane("left")
    calls = []

    class FakeWorker:
        def request_stop(self) -> None:
            calls.append("request_stop")

    class FakeThread:
        running = True

        def isRunning(self) -> bool:
            return self.running

    pane.worker = FakeWorker()
    pane.thread = FakeThread()
    owned_worker = pane.worker
    owned_thread = pane.thread

    started_s = time.monotonic()
    assert pane.stop() is False
    elapsed_s = time.monotonic() - started_s

    assert elapsed_s < 0.05
    assert calls == ["request_stop"]
    assert pane.worker is owned_worker
    assert pane.thread is owned_thread

    owned_thread.running = False
    assert pane.stop() is True
    assert pane.worker is None
    assert pane.thread is None

    pane.deleteLater()
    _ = app


def test_camera_pane_start_does_not_replace_a_stopping_worker(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    from wiris_stereo_distance import camera_pane
    from wiris_stereo_distance.camera_pane import CameraPane

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    pane = CameraPane("left")
    calls: list[str] = []

    class FakeWorker:
        def request_stop(self) -> None:
            calls.append("request_stop")

    class FakeThread:
        def isRunning(self) -> bool:
            return True

    pane.worker = FakeWorker()
    pane.thread = FakeThread()
    monkeypatch.setattr(
        camera_pane,
        "YoloStreamWorker",
        lambda **_kwargs: pytest.fail("replacement worker started before teardown"),
    )

    assert pane.start_stream(
        "rtsp://camera.invalid/visible",
        enable_detection=False,
    ) is False
    assert calls == ["request_stop"]

    pane.worker = None
    pane.thread = None
    pane.deleteLater()
    _ = app


def test_legacy_camera_mode_switch_restarts_only_after_workers_stop(
    monkeypatch,
    tmp_path,
) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui, "APP_ROOT", tmp_path)
    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    panel = gui.WirisStereoDistancePanel()
    callbacks: list[object] = []
    monkeypatch.setattr(
        gui.QtCore.QTimer,
        "singleShot",
        lambda _delay_ms, callback: callbacks.append(callback),
    )
    panel._ensure_camera_host_ip = lambda _reason: None
    stopped = {"value": False}
    starts: list[tuple[str, bool]] = []
    panel.left_pane.stop = lambda *, wait=False: stopped["value"]
    panel.right_pane.stop = lambda *, wait=False: stopped["value"]
    panel.left_pane.start_stream = (
        lambda _url, *, enable_detection, **_kwargs: starts.append(("left", enable_detection)) or True
    )
    panel.right_pane.start_stream = (
        lambda _url, *, enable_detection, **_kwargs: starts.append(("right", enable_detection)) or True
    )

    assert panel._start_camera_streams(
        enable_detection=True,
        switch_to_depth=True,
    ) is True
    assert panel._pending_camera_start == (True, True)
    assert starts == []
    assert len(callbacks) == 1

    stopped["value"] = True
    callbacks.pop(0)()

    assert panel._pending_camera_start is None
    assert starts == [("left", True), ("right", True)]
    assert panel.camera_connected is True
    assert panel.yolo_enabled is True

    panel.camera_connected = False
    panel.left_pane.worker = None
    panel.left_pane.thread = None
    panel.right_pane.worker = None
    panel.right_pane.thread = None
    panel.deleteLater()
    app.processEvents()


def test_recording_stop_waits_for_video_worker_restart_before_session_save(
    monkeypatch,
    tmp_path,
) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui, "APP_ROOT", tmp_path)
    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    panel = gui.WirisStereoDistancePanel()
    panel.distance_recording_enabled = True
    panel.video_recording_enabled = True
    panel.camera_connected = True
    callbacks: list[object] = []
    saves: list[dict[str, object] | None] = []
    panel._camera_time_sync_snapshot = lambda _reason: None
    panel._start_camera_streams = (
        lambda *, enable_detection, switch_to_depth, after_started=None: callbacks.append(after_started) or True
    )
    panel.save_distance_history_csv = (
        lambda *, final_clock_sync_snapshot=None: saves.append(final_clock_sync_snapshot)
    )

    panel._stop_session_recording()

    assert panel._session_save_waiting_for_video is True
    assert saves == []
    assert len(callbacks) == 1

    callbacks.pop()()

    assert panel._session_save_waiting_for_video is False
    assert saves == [None]

    panel.camera_connected = False
    panel.deleteLater()
    app.processEvents()


def test_session_save_rejects_zero_byte_video_after_camera_frames(
    monkeypatch,
    tmp_path,
) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui, "APP_ROOT", tmp_path)
    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    panel = gui.WirisStereoDistancePanel()
    panel.recording_session_id = "zero-video"
    panel._raw_video_validation_required = True
    left_path = tmp_path / "recordings" / "sessions" / "zero-video" / "video" / "raw_left.mp4"
    left_path.parent.mkdir(parents=True)
    left_path.write_bytes(b"")
    panel.video_recording_paths = {"left": left_path}
    panel.camera_recording_rows = {"left": [{}], "right": []}

    panel.save_distance_history_csv()

    assert panel._session_save_thread is None
    assert panel._session_save_retry_required is True
    assert "0바이트" in panel.recording_status_label.text()

    panel.deleteLater()
    app.processEvents()


def test_session_save_rejects_missing_expected_mp4_before_first_frame(
    monkeypatch,
    tmp_path,
) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui, "APP_ROOT", tmp_path)
    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    panel = gui.WirisStereoDistancePanel()
    panel.recording_session_id = "missing-before-frame"
    panel._raw_video_validation_required = True
    panel._raw_video_expected_cameras = ("left", "right")
    video_dir = (
        tmp_path
        / "recordings"
        / "sessions"
        / "missing-before-frame"
        / "video"
    )
    panel.video_recording_paths = {
        "left": video_dir / "raw_left.mp4",
        "right": video_dir / "raw_right.mp4",
    }
    panel.camera_recording_rows = {"left": [], "right": []}

    panel.save_distance_history_csv()

    assert panel._session_save_thread is None
    assert panel._session_save_retry_required is True
    assert "MP4 파일 없음" in panel.recording_status_label.text()
    assert "frames=0" in panel.recording_status_label.text()

    panel.deleteLater()
    app.processEvents()


def test_preview_disconnect_waits_for_video_workers_before_session_save(
    monkeypatch,
    tmp_path,
) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui, "APP_ROOT", tmp_path)
    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    panel = gui.WirisStereoDistancePanel()
    callbacks: list[object] = []
    monkeypatch.setattr(
        gui.QtCore.QTimer,
        "singleShot",
        lambda _delay_ms, callback: callbacks.append(callback),
    )
    stopped = {"value": False}
    panel.left_pane.stop = lambda *, wait=False: stopped["value"]
    panel.right_pane.stop = lambda *, wait=False: stopped["value"]
    panel.distance_recording_enabled = True
    panel.video_recording_enabled = True
    panel.camera_connected = True
    panel._camera_time_sync_snapshot = lambda _reason: None
    saves: list[object] = []
    panel.save_distance_history_csv = (
        lambda *, final_clock_sync_snapshot=None: saves.append(final_clock_sync_snapshot)
    )

    assert panel.stop_yolo_streams(wait=False) is False
    assert panel._session_save_waiting_for_video is True
    assert saves == []
    assert len(callbacks) == 1

    stopped["value"] = True
    callbacks.pop()()

    assert panel._session_save_waiting_for_video is False
    assert saves == [None]

    panel.camera_connected = False
    panel.left_pane.worker = None
    panel.left_pane.thread = None
    panel.right_pane.worker = None
    panel.right_pane.thread = None
    panel.deleteLater()
    app.processEvents()


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


def test_mavlink_only_runtime_never_scans_or_connects_ros2(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setenv("SASL_MAVLINK_ONLY", "1")
    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)

    def fail_ros2_scan(*_args, **_kwargs):
        raise AssertionError("ROS2 topic discovery must not run in MAVLink-only mode")

    monkeypatch.setattr(gui, "discover_ros2_topic_types", fail_ros2_scan)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()

    assert widget.mavlink_only_runtime is True
    assert widget.pixhawk_ros2_telemetry_group.isHidden()
    assert widget.pixhawk_ros2_candidate_group.isHidden()
    assert widget.baseline_ros2_group.isHidden()
    assert not widget.pixhawk_ethernet_mode_combo.isEnabled()
    assert not widget.mavlink_configure_uxrce_agent_checkbox.isEnabled()
    assert widget.connect_pixhawk("left") is False

    widget.refresh_baseline_field_browser()
    widget.auto_detect_pixhawks()
    widget.toggle_ros2_connection("left")
    assert widget.pixhawk_ros2_subscribers == {"left": None, "right": None}

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
            self._heartbeat_logged = set()
            self.shell_commands = []

        def start(self, camera=None) -> None:
            cameras = ("left", "right") if camera is None else (camera,)
            for side in cameras:
                if self.urls.get(side):
                    self.connections[side] = object()
                    self._heartbeat_logged.add(side)

        def send_shell_command(self, camera, command) -> None:
            self.shell_commands.append((camera, command))

        def read_param_int(self, camera, name, *, timeout_s=2.0):
            return None

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
    widget.pixhawk_ros2_subscribers["left"] = object()
    widget._sync_legacy_pixhawk_ref()
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
    widget.pixhawk_ros2_subscribers["left"] = object()
    widget._sync_legacy_pixhawk_ref()
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
    widget.pixhawk_ros2_subscribers["left"] = object()
    widget.pixhawk_ros2_subscribers["right"] = object()
    widget._sync_legacy_pixhawk_ref()
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
    widget.pixhawk_ros2_subscribers["left"] = object()
    widget.pixhawk_ros2_subscribers["right"] = object()
    widget._sync_legacy_pixhawk_ref()
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

    assert not hasattr(widget, "alignment_overview_labels")
    assert "ROS2 미게시" in widget.baseline_gps_card_labels["left"]["gps2"].text()
    assert "ROS2 raw MAG 미게시" in widget.alignment_mag_labels["left_CAN1"]["health"].text()
    assert widget.alignment_mag_labels["left_BOARD"]["device_type"].text() == "Board internal BMM150"
    assert widget.alignment_mag_labels["left_GPS1_MAG"]["device_type"].text() == "GPS1 integrated IST8310 compass"
    assert widget.alignment_mag_labels["left_GPS2_MAG"]["device_type"].text() == "GPS2 integrated IST8310 compass"
    assert widget.alignment_mag_labels["left_BOARD"]["vector_norm"].text() == "--"

    widget.deleteLater()
    _ = app


def test_alignment_mag_role_status_displays_vector_components_and_device_type(monkeypatch) -> None:
    from types import SimpleNamespace

    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui
    from wiris_stereo_distance.px4_mag_manager import (
        BOARD_INTERNAL_MAG_ID,
        GPS1_MAG_ID,
        GPS2_MAG_ID,
        RM3100_BACKUP_ID,
        RM3100_PRIMARY_ID,
    )

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    states = {
        "BOARD": (BOARD_INTERNAL_MAG_ID, 0.1),
        "CAN1": (RM3100_PRIMARY_ID, 1.1),
        "CAN2": (RM3100_BACKUP_ID, 2.1),
        "GPS1_MAG": (GPS1_MAG_ID, 3.1),
        "GPS2_MAG": (GPS2_MAG_ID, 4.1),
    }
    widget.latest_mag_sensors["left"] = {
        slot: SimpleNamespace(
            topic="/uav_0/fmu/out/sensor_mag",
            device_id=device_id,
            sensor_slot=slot,
            x=value,
            y=value + 0.1,
            z=value + 0.2,
            magnitude=value + 0.3,
        )
        for slot, (device_id, value) in states.items()
    }
    widget._refresh_alignment_magnetometer_ui()

    assert widget.alignment_mag_labels["left_BOARD"]["x"].text() == "0.100 G"
    assert widget.alignment_mag_labels["left_BOARD"]["y"].text() == "0.200 G"
    assert widget.alignment_mag_labels["left_BOARD"]["z"].text() == "0.300 G"
    assert widget.alignment_mag_labels["left_BOARD"]["vector_norm"].text() == "0.400 G"
    assert widget.alignment_mag_labels["left_BOARD"]["health"].text() == "정상"
    assert widget.alignment_mag_labels["left_GPS1_MAG"]["device_type"].text() == "GPS1 integrated IST8310 compass"
    assert widget.alignment_mag_labels["left_GPS2_MAG"]["device_type"].text() == "GPS2 integrated IST8310 compass"

    widget.deleteLater()
    _ = app


def test_magnetometer_difference_displays_milligauss_without_treating_it_as_gauss(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui
    from wiris_stereo_distance.pixhawk_attitude import MagSensorState

    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    states = {
        "CAN1": (100.0, "mavlink/SCALED_IMU"),
        "CAN2": (110.0, "mavlink/SCALED_IMU2"),
    }
    for slot, (magnitude, topic) in states.items():
        state = MagSensorState(
            camera="left",
            namespace="uav_0",
            sensor_slot=slot,
            timestamp_s=time.monotonic(),
            topic=topic,
            x=0.0,
            y=0.0,
            z=magnitude,
            magnitude=magnitude,
        )
        widget.latest_mag_sensors["left"][slot] = state
        widget.magnetometer_monitor.update("left", state)

    widget._refresh_alignment_magnetometer_ui()

    assert widget.alignment_mag_compare_labels["left_can_diff"].text() == "10.000 mG"

    widget.deleteLater()
    _ = app


def test_ros2_failure_starts_mavlink_recovery_and_fallback(monkeypatch) -> None:
    from ipaddress import ip_interface
    from types import SimpleNamespace

    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui
    from wiris_stereo_distance.settings import AppSettings

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
    # This transport fallback characterization uses the repository defaults,
    # not an operator's intentionally selected pixhawk_ethernet_mode.
    monkeypatch.setattr(gui, "load_settings", lambda: AppSettings())
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
    assert widget.mavlink_pixhawk.configure_calls == [
        {
            "host_ip": "10.41.10.1",
            "ports": {"left": 14550, "right": 14551},
            "sysids": {"left": 1, "right": 2},
            "rate": widget.mavlink_stream_rate_spin.value(),
            "cameras": {"left", "right"},
        }
    ]
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

    assert not hasattr(widget, "use_rtk_baseline_checkbox")
    assert not hasattr(widget, "require_rtk_fixed_checkbox")
    assert not hasattr(widget, "same_class_checkbox")
    toggles = [
        widget.manual_focal_checkbox,
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
    assert widget.distance_mode_combo.findData("checkerboard") >= 0
    assert widget.distance_mode_combo.itemText(widget.distance_mode_combo.findData("pixhawk")) == "Pixhawk 자세 + GPS/RTK baseline"
    assert widget.distance_mode_combo.itemText(widget.distance_mode_combo.findData("checkerboard")) == "체커보드 Stereo 외부 파라미터"
    assert not hasattr(widget, "require_attitude_checkbox")
    assert not hasattr(widget, "measurement_display_combo")
    assert widget._measurement_display_mode() == "detection"
    assert widget.yolo_gate_matching_checkbox.text() == "시간 기준"
    assert widget.yolo_gate_time_delta_checkbox.text() == "최대 Δt"
    assert widget.yolo_gate_vertical_delta_checkbox.text() == "vertical Δ"
    assert widget.yolo_gate_time_basis_combo.itemText(widget.yolo_gate_time_basis_combo.findData("camera_rtc_required")) == "RTCP/NTP 필수"
    assert widget.yolo_gate_time_basis_combo.itemText(widget.yolo_gate_time_basis_combo.findData("camera_rtcp")) == "RTCP/NTP 우선"
    assert widget.yolo_gate_time_basis_combo.itemText(widget.yolo_gate_time_basis_combo.findData("detection")) == "app 수신 시간"

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

    assert isinstance(widget.device_combo, QtWidgets.QComboBox)
    assert widget.device_combo.currentText() == "CUDA"
    widget.device_combo.setCurrentIndex(widget.device_combo.findText("CPU"))
    assert widget.device_combo.currentText() == "CPU"
    assert widget.device_edit.text() == gui.YOLO_CPU_DEVICE
    assert widget.yolo_kwargs()["device"] == gui.YOLO_CPU_DEVICE
    assert widget.yolo_kwargs()["input_size"] == expected_input_size
    assert widget.yolo_kwargs()["display_rate_hz"] == gui.DEFAULT_PREVIEW_DISPLAY_RATE_HZ
    assert widget.yolo_kwargs()["preview_max_width"] == expected_preview_width
    assert widget.yolo_kwargs()["event_queue_size"] == widget.yolo_event_queue_size_spin.value()
    assert widget.yolo_kwargs()["frame_queue_size"] == widget.yolo_frame_queue_size_spin.value()

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
        "RTP 지연(ms)": widget.rtsp_latency_spin,
        "Event 큐": widget.yolo_event_queue_size_spin,
        "Preview 큐": widget.yolo_frame_queue_size_spin,
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


def test_camera_sync_gate_fields_are_editable_and_loaded_from_settings(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    group = widget.findChild(QtWidgets.QGroupBox, "cameraSyncSettingsGroup")

    assert group is not None
    assert widget.tabs.indexOf(group.parentWidget()) == widget.time_tab_index
    assert widget.camera_sync_history_window_spin.value() == pytest.approx(1.0)
    assert widget.camera_sync_receive_pair_max_delta_spin.value() == pytest.approx(350.0)
    assert widget.camera_sync_rtcp_pair_max_delta_spin.value() == pytest.approx(50.0)
    assert widget.camera_sync_max_sr_age_spin.value() == pytest.approx(7.0)
    assert widget.camera_sync_max_pc_offset_spin.value() == pytest.approx(100.0)
    assert widget.camera_sync_clock_rate_error_spin.value() == pytest.approx(0.5)
    assert widget.camera_sync_required_sr_count_spin.value() == 2
    assert widget.camera_sync_warmup_timeout_spin.value() == pytest.approx(30.0)

    widget.camera_sync_receive_pair_max_delta_spin.setValue(420.0)
    assert widget._camera_sync_receive_pair_max_delta_s() == pytest.approx(0.420)
    widget.camera_sync_required_sr_count_spin.setValue(4)
    assert widget._camera_sync_required_sr_count() == 4
    widget.deleteLater()
    _ = app

    widget.deleteLater()
    _ = app


def test_yolo_and_rtcp_history_tooltips_explain_latency_and_time_basis(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()

    assert "letterbox/resize" in widget.input_size_spin.toolTip()
    assert "confidence" in widget.conf_spin.toolTip()
    assert "NMS" in widget.iou_spin.toolTip()
    assert "추론 입력 크기" in widget.input_size_spin.toolTip()
    assert "RTSP 수신" in widget.rtsp_latency_spin.toolTip()
    assert "RTCP/NTP 동기화 Gate" in widget.yolo_gate_time_delta_checkbox.toolTip()
    assert "RTCP/NTP 필수" in widget.yolo_gate_time_basis_combo.toolTip()

    plot_tooltip = widget.clock_offset_plot.toolTip()
    assert "transit delay" in plot_tooltip
    assert "transport delay" in plot_tooltip
    assert "Sender Report anchor age" in plot_tooltip
    assert "RTP 지연(ms)" in plot_tooltip
    assert "0 s가 최신 샘플" in plot_tooltip

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
    assert not hasattr(widget, "runtime_stream_settings_label")
    assert "카메라:" in widget.runtime_camera_summary_label.text()

    settings_group.setChecked(False)
    assert widget.runtime_camera_summary_label.isHidden()
    settings_group.setChecked(True)
    assert not widget.runtime_camera_summary_label.isHidden()

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
        "왼쪽=연결 1920x1080 수신 29.8 Hz"
    )
    assert format_camera_stream_stats("right", "connected", yolo_stats) == (
        "오른쪽=연결 1280x720 수신 28.2 Hz, YOLO 11.5 Hz"
    )


def test_runtime_camera_stats_format_uses_received_stats_only() -> None:
    from wiris_stereo_distance.gui import format_runtime_camera_stats

    stats = {
        "decoded_hz": 19.75,
        "inferred_hz": 7.25,
        "width": 1920,
        "height": 1080,
        "enable_detection": True,
        "yolo_actual_device": "cuda:0",
    }

    assert format_runtime_camera_stats("left", stats, fallback_hz=12.3, stale=False) == (
        "왼쪽: 1920x1080 19.8 Hz | 7.2 Hz (CUDA)"
    )
    assert format_runtime_camera_stats("right", {}, fallback_hz=12.34, stale=True) == (
        "오른쪽: -- | -- | 지연"
    )


def test_runtime_camera_stats_format_marks_cpu_fallback() -> None:
    from wiris_stereo_distance.gui import format_runtime_camera_stats

    stats = {
        "decoded_hz": 18.1,
        "inferred_hz": 5.0,
        "width": 1280,
        "height": 720,
        "enable_detection": True,
        "yolo_actual_device": "cpu",
        "yolo_cpu_fallback": True,
    }

    assert format_runtime_camera_stats("right", stats, fallback_hz=12.3, stale=False) == (
        "오른쪽: 1280x720 18.1 Hz | 5.0 Hz (CPU fallback)"
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
    assert "P1 RDF Right=0.241 m" in widget.rtk_alignment_detail_label.text()
    assert "P1 RDF 좌우 성분 |Right|=0.241 m" in widget.rtk_alignment_detail_label.text()

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


def test_live_gps_baseline_visualizes_and_is_required(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui
    from wiris_stereo_distance.rtk_geometry import GpsPositionState

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()

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
    assert "거리 계산 필수 입력" in widget.rtk_baseline_label.text()
    assert "left 3D_FIX" in widget.rtk_baseline_label.text()
    assert "right 3D_FIX" in widget.rtk_baseline_label.text()
    assert "RTK Fixed: NO" in widget.rtk_fixed_status_label.text()
    assert "GPS baseline 필수" in widget.rtk_fixed_status_label.text()
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
    widget.distance_mode_combo.setCurrentIndex(widget.distance_mode_combo.findData("pixhawk"))

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
    widget.latest_extrinsics["right"] = CameraExtrinsicState(
        camera="right",
        namespace="/uav_1",
        roll_deg=0.0,
        pitch_deg=0.0,
        yaw_deg=0.0,
        timestamp_s=now_s,
        topic="/uav_1/fmu/out/vehicle_attitude",
        q=(1.0, 0.0, 0.0, 0.0),
    )

    translation = widget._latest_translation_right_in_left_m_for_diagnostics()

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


def test_baseline_tab_reports_rdf_forward_angle(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui
    from wiris_stereo_distance.pixhawk_attitude import CameraExtrinsicState
    from wiris_stereo_distance.rtk_geometry import GpsPositionState

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()

    now_s = time.monotonic()
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
    widget.latest_extrinsics["right"] = CameraExtrinsicState(
        camera="right",
        namespace="/uav_1",
        roll_deg=0.0,
        pitch_deg=0.0,
        yaw_deg=0.0,
        timestamp_s=now_s,
        topic="/uav_1/fmu/out/vehicle_attitude",
        q=(1.0, 0.0, 0.0, 0.0),
    )
    widget._update_gps(
        "left",
        GpsPositionState("left", "/uav_0", now_s, "gps", 37.0, 127.0, 10.0, fix_type=3, satellites_used=8, eph_m=2.5),
    )
    widget._update_gps(
        "right",
        GpsPositionState("right", "/uav_1", now_s, "gps", 37.0, 127.000003, 10.0, fix_type=3, satellites_used=9, eph_m=2.2),
    )

    assert "90.00 deg" in widget.baseline_attitude_labels["rdf_forward_angle"].text()
    assert "+0.0000 m" in widget.baseline_attitude_labels["forward_component"].text()
    assert "90.00 deg" in widget.baseline_attitude_labels["right_rdf_forward_angle"].text()
    assert "+0.0000 m" in widget.baseline_attitude_labels["right_forward_component"].text()
    assert "P1 RDF Forward vs baseline angle=90.00" in widget.rtk_alignment_detail_label.text()
    assert "P2 RDF Forward vs baseline angle=90.00" in widget.rtk_alignment_detail_label.text()

    widget.deleteLater()
    _ = app


def test_non_rtk_fixed_gps_can_drive_translation(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui
    from wiris_stereo_distance.pixhawk_attitude import CameraExtrinsicState
    from wiris_stereo_distance.rtk_geometry import GpsPositionState

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget.distance_mode_combo.setCurrentIndex(widget.distance_mode_combo.findData("pixhawk"))

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

    assert widget._latest_translation_right_in_left_m_for_diagnostics() is not None
    assert "RTK Fixed: NO" in widget.rtk_fixed_status_label.text()
    assert "사용 가능" in widget.rtk_fixed_status_label.text()
    assert widget._active_baseline_m()[1] == "gps-rtk"

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
    widget.distance_mode_combo.setCurrentIndex(widget.distance_mode_combo.findData("pixhawk"))

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

    translation = widget._latest_translation_right_in_left_m_for_diagnostics()
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
    widget.distance_mode_combo.setCurrentIndex(widget.distance_mode_combo.findData("pixhawk"))

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

    translation = widget._latest_translation_right_in_left_m_for_diagnostics()

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
    widget.distance_mode_combo.setCurrentIndex(widget.distance_mode_combo.findData("pixhawk"))

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

    assert widget._latest_translation_right_in_left_m_for_diagnostics() is None

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

def test_attitude_history_updates_without_roll_pitch_yaw_ui() -> None:
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
    assert not hasattr(widget, "attitude_compare_labels")
    assert not hasattr(widget, "attitude_plots")

    widget.deleteLater()
    _ = app


def test_yaw_difference_labels_use_wrapped_angle_difference() -> None:
    from PyQt5 import QtWidgets

    from wiris_stereo_distance.gui import WirisStereoDistancePanel, angle_diff_deg
    from wiris_stereo_distance.pixhawk_attitude import CameraExtrinsicState

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = WirisStereoDistancePanel()

    assert angle_diff_deg(179.0, -179.0) == -2.0
    assert angle_diff_deg(-179.0, 179.0) == 2.0

    widget.latest_extrinsics["left"] = CameraExtrinsicState(
        camera="left",
        namespace="/uav_0",
        roll_deg=0.0,
        pitch_deg=0.0,
        yaw_deg=179.0,
        timestamp_s=0.0,
        topic="/uav_0/fmu/out/vehicle_attitude",
        q=(1.0, 0.0, 0.0, 0.0),
    )
    widget.latest_extrinsics["right"] = CameraExtrinsicState(
        camera="right",
        namespace="/uav_1",
        roll_deg=0.0,
        pitch_deg=0.0,
        yaw_deg=-179.0,
        timestamp_s=0.0,
        topic="/uav_1/fmu/out/vehicle_attitude",
        q=(1.0, 0.0, 0.0, 0.0),
    )

    widget._refresh_alignment_attitude_ui()
    widget._update_relative_attitude_status()

    assert widget.baseline_attitude_labels["yaw_diff"].text() == "-2.000 deg"

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


def test_mavlink_spin_once_bounds_busy_connection_drain() -> None:
    from types import SimpleNamespace

    from wiris_stereo_distance.pixhawk_attitude import (
        DualPixhawkMavlinkSubscriber,
        MAVLINK_MAX_MESSAGES_PER_TICK,
    )

    class FakeMessage(SimpleNamespace):
        def get_type(self):
            return "UNKNOWN"

    class BusyConnection:
        def __init__(self):
            self.calls = 0

        def recv_match(self, *, blocking):
            assert blocking is False
            self.calls += 1
            return FakeMessage()

    subscriber = DualPixhawkMavlinkSubscriber(left_url="udpin:0.0.0.0:14550", right_url="")
    connection = BusyConnection()
    subscriber.connections["left"] = connection

    subscriber.spin_once()

    assert connection.calls == MAVLINK_MAX_MESSAGES_PER_TICK


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
        assert not widget.px4_mag_apply_manual_buttons[side].isEnabled()
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
    assert "ROS2 vehicle_magnetometer" in widget.px4_mag_status_labels["left"].text()
    assert "CAL_MAG slot/priority: N/A" in widget.px4_mag_status_labels["left"].text()
    assert widget.px4_mag_tables["left"].rowCount() == 0
    assert widget.px4_mag_refresh_buttons["left"].isEnabled()
    assert not widget.px4_mag_apply_manual_buttons["left"].isEnabled()
    assert "Actual selected MAG: 222" not in widget.px4_mag_status_labels["left"].text()
    assert "Pixhawk 2" in widget.px4_mag_status_labels["right"].text()
    assert "Actual selected MAG: 222" in widget.px4_mag_status_labels["right"].text()
    assert widget.px4_mag_tables["right"].rowCount() == 0

    widget.deleteLater()
    _ = app


def test_px4_mag_telemetry_does_not_trigger_mavlink_param_readback(monkeypatch) -> None:
    from types import SimpleNamespace

    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    class FakeMavlink:
        def __init__(self) -> None:
            self.connections = {"left": object()}
            self._heartbeat_logged = {"left"}
            self.read_calls = 0

        def read_param_int(self, camera: str, name: str, *, timeout_s: float = 2.0) -> int | None:
            self.read_calls += 1
            return None

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget.mavlink_pixhawk = FakeMavlink()
    state = SimpleNamespace(
        topic="/uav_0/fmu/out/vehicle_magnetometer",
        device_id=111,
        sensor_slot="SELECTED",
        timestamp_s=time.monotonic(),
        x=None,
        y=None,
        z=None,
        magnitude=None,
    )

    widget._update_mag_sensor("left", state)
    widget._update_mag_sensor("left", state)

    assert widget.mavlink_pixhawk.read_calls == 0
    assert "새로고침" in widget.px4_mag_status_labels["left"].text()

    widget.deleteLater()
    _ = app


def test_px4_mag_manager_ros2_raw_only_reports_selected_mag_wait(monkeypatch) -> None:
    from types import SimpleNamespace

    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    now_s = time.monotonic()
    widget.pixhawk_ros2_subscribers["left"] = object()
    widget._sync_legacy_pixhawk_ref()

    widget._update_mag_sensor(
        "left",
        SimpleNamespace(
            topic="/uav_0/fmu/out/sensor_mag_4",
            device_id=8944395,
            sensor_slot="CAN2",
            timestamp_s=now_s,
            x=0.1,
            y=0.2,
            z=0.3,
            magnitude=0.374,
        ),
    )

    text = widget.px4_mag_status_labels["left"].text()
    assert "ROS2 raw MAG 수신 중: CAN2" in text
    assert "vehicle_magnetometer device_id 수신 대기" in text
    assert "CAL_MAG priority readback은 MAVLink PARAM 연결이 필요" in text
    assert widget.px4_mag_tables["left"].rowCount() == 0
    assert not widget.px4_mag_refresh_buttons["left"].isEnabled()

    widget.deleteLater()
    _ = app


def test_px4_mag_manager_applies_manual_role_priorities(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui
    from wiris_stereo_distance.px4_mag_manager import (
        BOARD_INTERNAL_MAG_ID,
        GPS1_MAG_ID,
        GPS2_MAG_ID,
        RM3100_BACKUP_ID,
        RM3100_PRIMARY_ID,
    )

    class FakeMavlink:
        def __init__(self) -> None:
            self.connections = {"left": object()}
            self._heartbeat_logged = {"left"}
            self.values = {
                "UAVCAN_ENABLE": 2,
                "UAVCAN_SUB_MAG": 1,
                "EKF2_MAG_TYPE": 0,
                "CAL_MAG0_ID": BOARD_INTERNAL_MAG_ID,
                "CAL_MAG0_PRIO": 75,
                "CAL_MAG0_ROT": 0,
                "CAL_MAG1_ID": RM3100_PRIMARY_ID,
                "CAL_MAG1_PRIO": 75,
                "CAL_MAG1_ROT": 0,
                "CAL_MAG2_ID": RM3100_BACKUP_ID,
                "CAL_MAG2_PRIO": 75,
                "CAL_MAG2_ROT": 0,
                "CAL_MAG3_ID": GPS1_MAG_ID,
                "CAL_MAG3_PRIO": 75,
                "CAL_MAG3_ROT": 0,
                "CAL_MAG4_ID": GPS2_MAG_ID,
                "CAL_MAG4_PRIO": 75,
                "CAL_MAG4_ROT": 0,
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
    widget.latest_vehicle_mag_device_ids["left"] = BOARD_INTERNAL_MAG_ID
    spins = widget.px4_mag_priority_spins["left"]
    spins["BOARD"].setValue(5)
    spins["CAN1"].setValue(88)
    spins["CAN2"].setValue(77)
    spins["GPS1_MAG"].setValue(0)
    spins["GPS2_MAG"].setValue(1)

    widget.apply_manual_mag_priorities_from_panel("left")

    assert ("left", "CAL_MAG0_PRIO", 5) in widget.mavlink_pixhawk.set_calls
    assert ("left", "CAL_MAG1_PRIO", 88) in widget.mavlink_pixhawk.set_calls
    assert ("left", "CAL_MAG2_PRIO", 77) in widget.mavlink_pixhawk.set_calls
    assert ("left", "CAL_MAG3_PRIO", 0) in widget.mavlink_pixhawk.set_calls
    assert ("left", "CAL_MAG4_PRIO", 1) in widget.mavlink_pixhawk.set_calls

    widget.deleteLater()
    _ = app


def test_session_save_omits_pixhawk_mag_csv(tmp_path, monkeypatch) -> None:
    import time
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui
    from wiris_stereo_distance.px4_mag_manager import RM3100_PRIMARY_ID

    monkeypatch.setattr(gui, "APP_ROOT", tmp_path)
    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget.calib_dir_edit.setText(str(tmp_path / "calibration_captures"))
    widget.recording_session_id = "20260605_153012"
    widget.recording_session_started_wall_s = 1_800_000_000.0
    widget.recording_session_started_monotonic_s = time.monotonic() - 3.0
    widget.latest_vehicle_mag_device_ids["left"] = RM3100_PRIMARY_ID
    widget.latest_vehicle_mag_device_ids["right"] = 396809
    widget.save_distance_history_csv()
    deadline = time.monotonic() + 5.0
    while widget._session_save_thread is not None and time.monotonic() < deadline:
        app.processEvents()
        time.sleep(0.01)

    session_dir = tmp_path / "recordings" / "sessions" / "20260605_153012"
    csv_path = session_dir / "20260605_153012_distance_measurements.csv"
    xlsx_path = session_dir / "20260605_153012.xlsx"
    mag_path = session_dir / "alignment" / "pixhawk_mag_status.csv"
    assert csv_path.exists()
    assert xlsx_path.exists()
    assert not mag_path.exists()
    assert sorted(path.name for path in session_dir.iterdir()) == [
        ".generations",
        "20260605_153012.xlsx",
        "20260605_153012_camera_left.csv",
        "20260605_153012_camera_left.parquet",
        "20260605_153012_camera_right.csv",
        "20260605_153012_camera_right.parquet",
        "20260605_153012_distance_measurements.csv",
        "20260605_153012_distance_measurements.parquet",
        "20260605_153012_gnss_base.csv",
        "20260605_153012_gnss_base.parquet",
        "20260605_153012_px4_left.csv",
        "20260605_153012_px4_left.parquet",
        "20260605_153012_px4_left.xlsx",
        "20260605_153012_px4_right.csv",
        "20260605_153012_px4_right.parquet",
        "20260605_153012_px4_right.xlsx",
        "20260605_153012_session_rtk.xlsx",
        "20260605_153012_session_static.json",
        "published",
        "session_manifest.json",
    ]

    widget.deleteLater()
    _ = app


def test_px4_mag_manager_accepts_alternate_rm3100_backup_id() -> None:
    from wiris_stereo_distance.px4_mag_manager import (
        BOARD_INTERNAL_MAG_ID,
        GPS1_MAG_ID,
        GPS2_MAG_ID,
        MockParameterStore,
        Px4MagManager,
        RM3100_ALTERNATE_BACKUP_ID,
        RM3100_PRIMARY_ID,
        build_mag_status,
        identify_mag_device,
    )

    info = identify_mag_device(RM3100_ALTERNATE_BACKUP_ID)
    assert info.is_rm3100 is True
    assert "RM3100 #2" in info.device_type

    params = {
        "UAVCAN_ENABLE": 2,
        "UAVCAN_SUB_MAG": 1,
        "EKF2_MAG_TYPE": 0,
        "CAL_MAG0_ID": BOARD_INTERNAL_MAG_ID,
        "CAL_MAG0_PRIO": 0,
        "CAL_MAG0_ROT": 0,
        "CAL_MAG1_ID": GPS1_MAG_ID,
        "CAL_MAG1_PRIO": 0,
        "CAL_MAG1_ROT": 0,
        "CAL_MAG2_ID": GPS2_MAG_ID,
        "CAL_MAG2_PRIO": 0,
        "CAL_MAG2_ROT": 0,
        "CAL_MAG3_ID": RM3100_PRIMARY_ID,
        "CAL_MAG3_PRIO": 75,
        "CAL_MAG3_ROT": 0,
        "CAL_MAG4_ID": RM3100_ALTERNATE_BACKUP_ID,
        "CAL_MAG4_PRIO": 75,
        "CAL_MAG4_ROT": 0,
    }
    status = build_mag_status(params, actual_selected_device_id=RM3100_ALTERNATE_BACKUP_ID)
    assert status.is_actual_rm3100 is True
    assert "Second RM3100 is not registered" not in status.warning_text
    assert next(slot for slot in status.slots if slot.slot == 4).is_rm3100 is True

    manager = Px4MagManager(MockParameterStore(params))
    manager.use_rm3100_for_yaw()
    assert manager.store.values["CAL_MAG3_PRIO"] == 100
    assert manager.store.values["CAL_MAG4_PRIO"] == 90


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


def test_enable_wiris_rtsp_does_not_apply_stale_encoder_profile_or_redundant_seth(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    assert widget.settings.wiris_visible_encoder_idr == 1
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
        ("10.0.0.232", gui.GET_ETHERNET_MODE_COMMAND, gui.LINE_TERMINATOR),
    ]
    assert not any(
        command.startswith(("SEBR", "SEQZ", "SEGS", "SEIR"))
        for _host, command, _terminator in sent
    )
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
        ("10.0.0.232", gui.GET_ETHERNET_MODE_COMMAND, gui.LINE_TERMINATOR),
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
        if command.command == "GEBR VISIBLE":
            return "0"
        if command.command == "GEQZ VISIBLE":
            return "1"
        if command.command == "GEGS VISIBLE":
            return "90"
        if command.command == "GEIR VISIBLE":
            return "1"
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
    assert not any(
        command.startswith(("SEBR", "SEQZ", "SEGS", "SEIR"))
        for _host, _port, command in sent
    )
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
        if command.command == "GEBR VISIBLE":
            return "0"
        if command.command == "GEQZ VISIBLE":
            return "1"
        if command.command == "GEGS VISIBLE":
            return "90"
        if command.command == "GEIR VISIBLE":
            return "1"
        return "TRUE" if command.command == gui.GET_ETHERNET_MODE_COMMAND else "OK"

    monkeypatch.setattr(gui, "send_tcp_line", slow_send)
    widget.check_cameras = lambda: None

    started_s = time.monotonic()
    widget.enable_wiris_rtsp()
    elapsed_s = time.monotonic() - started_s

    assert elapsed_s < 0.1
    assert widget.rtsp_enable_thread is not None
    assert process_events_until(app, lambda: widget.rtsp_enable_thread is None, timeout_s=5.0)

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
        lambda command: (
            "TRUE"
            if command.command == gui.GET_ETHERNET_MODE_COMMAND
            else {
                "GEBR VISIBLE": "0",
                "GEQZ VISIBLE": "1",
                "GEGS VISIBLE": "90",
                "GEIR VISIBLE": "1",
            }.get(command.command, "OK")
        ),
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
    from types import SimpleNamespace

    from PyQt5 import QtWidgets
    from ipaddress import ip_interface

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget.interface_edit.setText("enx-test")
    widget.host_address_edit.setText("10.41.10.1/24")

    apply_calls = []
    discovery_calls = []
    monkeypatch.setattr(
        gui,
        "discover_host_interface_candidates",
        lambda **kwargs: (
            discovery_calls.append(kwargs)
            or [
                SimpleNamespace(
                    interface=SimpleNamespace(name="enx-camera", address=ip_interface("10.0.0.10/24")),
                    neighbor_hosts=("10.0.0.231", "10.0.0.232"),
                )
            ]
        ),
    )
    monkeypatch.setattr(gui, "interface_link_exists", lambda *_args, **_kwargs: True)
    monkeypatch.setattr(
        gui,
        "resolve_nm_interface_address",
        lambda interface_name, host_hints: (
            interface_name == "enx-camera"
            and tuple(host_hints) == ("10.0.0.231", "10.0.0.232")
            and ip_interface("10.0.0.10/24")
        ),
    )
    monkeypatch.setattr(
        gui,
        "apply_interface_address",
        lambda interface_name, address_text: (
            apply_calls.append((interface_name, address_text)) or True,
            f"applied {address_text} to {interface_name}",
        ),
    )

    widget.discover_pc_camera_ip()

    assert discovery_calls == [{"host_address": "10.41.10.1/24", "include_unconfigured": True}]
    assert widget.interface_edit.text() == "enx-camera"
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

    assert [widget.alignment_tabs.tabText(index) for index in range(widget.alignment_tabs.count())] == [
        "센서 인식",
        "소스 / 품질",
        "GNSS Base",
        "MAG 관리",
        "Baseline 계산",
        "Baseline / 자세 정렬",
        "Optical 정렬",
    ]
    assert widget.alignment_tabs.tabText(widget.baseline_alignment_tab_index) == "Baseline 계산"
    assert widget.advanced_tabs.tabText(widget.advanced_baseline_field_tab_index) == "Baseline 필드"
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

    widget.baseline_samples.append((object(), object(), object()))
    widget.baseline_length_plot.add_point(gui.TelemetryHistoryPoint(now_s - 1.0, "length", 25.0))
    widget.start_baseline_sampling()
    assert widget.baseline_samples == []
    assert len(widget.baseline_length_plot.points) == 0

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


def test_baseline_input_selection_drives_live_rtk_baseline(monkeypatch) -> None:
    import time

    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui
    from wiris_stereo_distance.rtk_geometry import GpsPositionState, estimate_rtk_baseline

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget.distance_mode_combo.setCurrentIndex(widget.distance_mode_combo.findData("pixhawk"))
    now_s = time.monotonic()
    left_gps1 = GpsPositionState("left", "/uav_0", now_s, "gps1", 37.0, 127.0, 10.0, fix_type=6)
    right_gps1 = GpsPositionState("right", "/uav_1", now_s, "gps1", 37.0, 127.000050, 10.0, fix_type=6)
    left_gps2 = GpsPositionState("left_GPS2", "/uav_0", now_s, "gps2", 37.0, 127.0, 10.0, fix_type=6)
    right_gps2 = GpsPositionState("right_GPS2", "/uav_1", now_s, "gps2", 37.0, 127.000003, 10.0, fix_type=6)
    widget.latest_gps.update(
        {
            "left": left_gps1,
            "right": right_gps1,
            "left_GPS2": left_gps2,
            "right_GPS2": right_gps2,
        }
    )

    widget._refresh_rtk_baseline()
    gps1_expected = estimate_rtk_baseline(left_gps1, right_gps1, now_s=now_s, require_fixed=True)
    assert gps1_expected is not None
    assert widget.latest_rtk_baseline is not None
    assert abs(widget.latest_rtk_baseline.baseline_m - gps1_expected.baseline_m) < 0.001

    widget.baseline_left_gps_combo.setCurrentIndex(widget.baseline_left_gps_combo.findData("GPS2"))
    widget.baseline_right_gps_combo.setCurrentIndex(widget.baseline_right_gps_combo.findData("GPS2"))

    gps2_expected = estimate_rtk_baseline(left_gps2, right_gps2, now_s=now_s, require_fixed=True)
    assert gps2_expected is not None
    assert widget.latest_rtk_baseline is not None
    assert abs(widget.latest_rtk_baseline.baseline_m - gps2_expected.baseline_m) < 0.001
    assert widget.latest_rtk_baseline.baseline_m < gps1_expected.baseline_m
    assert widget._active_baseline_m()[1] == "gps-rtk"

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


def test_ros2_alternate_rm3100_backup_id_maps_to_can2_mag_sensor() -> None:
    from types import SimpleNamespace

    from wiris_stereo_distance.pixhawk_attitude import DualPixhawkAttitudeSubscriber
    from wiris_stereo_distance.px4_mag_manager import RM3100_ALTERNATE_BACKUP_ID

    subscriber = DualPixhawkAttitudeSubscriber(domain_id=0, left_namespace="uav_0", right_namespace="uav_1")
    mags = []
    subscriber.mag_sensor_changed.connect(lambda camera, mag: mags.append((camera, mag)))

    subscriber._handle_mag_message(
        "right",
        "uav_1",
        SimpleNamespace(
            device_id=RM3100_ALTERNATE_BACKUP_ID,
            magnetometer_ga=(0.0, 3.0, 4.0),
            timestamp=123456,
        ),
        topic="/uav_1/fmu/out/sensor_mag_4",
    )

    assert mags[0][0] == "right"
    assert mags[0][1].sensor_slot == "CAN2"
    assert mags[0][1].device_id == RM3100_ALTERNATE_BACKUP_ID
    assert mags[0][1].topic == "/uav_1/fmu/out/sensor_mag_4"
    assert mags[0][1].magnitude == 5.0


def test_camera_parameters_tab_shows_intrinsics_and_live_extrinsics() -> None:
    import time

    from PyQt5 import QtCore, QtWidgets

    from wiris_stereo_distance.calibration import CameraIntrinsics, CheckerboardConfig, StereoExtrinsics, StereoIntrinsicsResult
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
        extrinsics=StereoExtrinsics(
            rotation_left_to_right=[[1.0, 0.0, 0.0], [0.0, 0.999, -0.001], [0.0, 0.001, 0.999]],
            translation_left_to_right_m=[0.300, 0.001, -0.002],
            reprojection_error_px=0.25,
            pair_count=12,
        ),
    )
    widget.apply_intrinsics_to_distance()
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
    widget.tabs.setCurrentIndex(widget.parameter_tab_index)
    widget.parameter_tabs.setCurrentIndex(widget.stereo_settings_tab_index)
    widget.resize(1190, 820)
    widget.show()
    app.processEvents()

    tab_names = [widget.tabs.tabText(index) for index in range(widget.tabs.count())]
    assert "진단" not in tab_names
    assert tab_names == [
        "연결",
        "시간",
        "이미지",
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
    ]
    assert [widget.calibration_tabs.tabText(index) for index in range(widget.calibration_tabs.count())] == [
        "체커보드",
        "체커보드 검증/계산",
        "줌 내부 파라미터",
    ]
    assert [widget.alignment_tabs.tabText(index) for index in range(widget.alignment_tabs.count())] == [
        "센서 인식",
        "소스 / 품질",
        "GNSS Base",
        "MAG 관리",
        "Baseline 계산",
        "Baseline / 자세 정렬",
        "Optical 정렬",
    ]
    assert [widget.pixhawk_tabs.tabText(index) for index in range(widget.pixhawk_tabs.count())] == [
        "요약",
        "픽스호크 1",
        "픽스호크 2",
    ]
    assert [widget.advanced_tabs.tabText(index) for index in range(widget.advanced_tabs.count())] == [
        "WIRIS SDK",
        "필드 감시",
        "Baseline 필드",
        "PX4 MAG",
    ]
    assert [widget.parameter_tabs.tabText(index) for index in range(widget.parameter_tabs.count())] == [
        "내부",
        "외부/기하",
        "계산 설정",
    ]
    assert [widget.measurement_tabs.tabText(index) for index in range(widget.measurement_tabs.count())] == [
        "실시간",
        "측정 품질",
    ]
    assert not hasattr(widget, "save_tabs")
    save_tab = widget.tabs.widget(widget.results_tab_index)
    assert save_tab.isAncestorOf(widget.recording_status_label)
    assert save_tab.isAncestorOf(widget.save_history_button)
    assert save_tab.isAncestorOf(widget.session_summary_label)
    assert save_tab.isAncestorOf(widget.recording_table_group)
    assert widget.advanced_tabs.tabText(widget.wiris_command_tab_index) == "WIRIS SDK"
    assert widget.advanced_tabs.tabText(widget.advanced_baseline_field_tab_index) == "Baseline 필드"
    assert widget.setup_tabs.tabText(widget.pixhawk_connection_tab_index) == "Pixhawk"
    assert widget.tabs.tabText(widget.camera_parameters_tab_index) == "파라미터"
    assert widget.parameter_tabs.tabText(widget.intrinsic_parameters_tab_index) == "내부"
    assert widget.parameter_tabs.tabText(widget.extrinsic_parameters_tab_index) == "외부/기하"
    assert widget.parameter_tabs.tabText(widget.stereo_settings_tab_index) == "계산 설정"
    assert widget.calibration_tabs.tabText(widget.checkerboard_calibration_tab_index) == "체커보드"
    assert widget.calibration_tabs.tabText(widget.checkerboard_validation_tab_index) == "체커보드 검증/계산"
    assert widget.calibration_tabs.tabText(widget.zoom_intrinsics_tab_index) == "줌 내부 파라미터"
    assert widget.alignment_tabs.tabText(widget.optical_alignment_tab_index) == "Optical 정렬"
    assert widget.intrinsics_settings_group.isCheckable()
    assert widget.intrinsics_settings_group.isChecked()
    assert widget.extrinsics_settings_group.title() == "기본: 외부 파라미터 / Baseline"
    assert widget.extrinsics_settings_group.isVisible()
    assert isinstance(widget.distance_mode_combo, QtWidgets.QComboBox)
    assert widget.distance_mode_combo.currentText() == "Pixhawk 자세 + GPS/RTK baseline"
    assert not hasattr(widget, "measurement_display_combo")
    assert widget._measurement_display_mode() == "detection"
    assert widget.stereo_parameter_splitter.orientation() == QtCore.Qt.Horizontal
    assert widget.stereo_parameter_splitter.count() == 2
    sizes = widget.stereo_parameter_splitter.sizes()
    assert len(sizes) == 2
    assert sizes[0] > 0 and sizes[1] > 0
    assert abs(sizes[0] - sizes[1]) < max(sizes)
    assert widget.distance_mode_combo.itemText(widget.distance_mode_combo.findData("pixhawk")) == "Pixhawk 자세 + GPS/RTK baseline"
    assert widget.distance_mode_combo.itemText(widget.distance_mode_combo.findData("checkerboard")) == "체커보드 Stereo 외부 파라미터"
    assert widget.save_intrinsics_button.parentWidget().title() == "저장 / 적용"
    assert widget.apply_intrinsics_button.parentWidget().title() == "저장 / 적용"
    assert widget.apply_intrinsics_button.text() == "계산에 적용"
    assert "source: active calibration result" in widget.left_intrinsics_text.toPlainText()
    assert "fx: 900.000000" in widget.left_intrinsics_text.toPlainText()
    assert "distortion_coefficients" in widget.right_intrinsics_text.toPlainText()
    assert "source: MAVLink (mavlink/ATTITUDE)" in widget.left_extrinsics_text.toPlainText()
    assert "roll:  1.000000 deg" in widget.left_extrinsics_text.toPlainText()
    assert "intrinsics_source: active calibration result" in widget.stereo_geometry_text.toPlainText()
    assert "focal_source: calibrated intrinsics average fx" in widget.stereo_geometry_text.toPlainText()
    assert "baseline_source_detail: waiting for live GPS/RTK baseline" in widget.stereo_geometry_text.toPlainText()
    assert "checkerboard_stereo_extrinsics: computed" in widget.stereo_geometry_text.toPlainText()
    assert "checkerboard_frame_contract: opencv_optical:RDF" in widget.stereo_geometry_text.toPlainText()
    assert "checkerboard_translation_left_to_right_m: 0.300000000, 0.001000000, -0.002000000" in widget.stereo_geometry_text.toPlainText()
    assert "distance_method: 3D ray fallback: Pixhawk 자세 대기 중" in widget.stereo_geometry_text.toPlainText()
    assert "use_3d_distance: true" in widget.stereo_geometry_text.toPlainText()
    assert "계산 파라미터:" in widget.distance_parameters_label.text()
    assert "baseline_used" in widget.stereo_geometry_text.toPlainText()
    assert "source: active calibration result for zoom" in widget.zoom_left_intrinsics_text.toPlainText()
    assert "source: active calibration result for zoom" in widget.zoom_right_intrinsics_text.toPlainText()

    widget.distance_mode_combo.setCurrentIndex(widget.distance_mode_combo.findData("checkerboard"))
    left_effective_extrinsic_text = widget.left_extrinsics_text.toPlainText()
    right_effective_extrinsic_text = widget.right_extrinsics_text.toPlainText()
    assert widget._distance_mode() == "checkerboard"
    assert widget._triangulation_extrinsics_source_label() == "체커보드 Stereo 외부 파라미터"
    assert widget._active_baseline_m()[1] == "checkerboard"
    assert widget._measurement_path_label() == "3D ray 거리: 체커보드 Stereo 외부 파라미터 사용"
    assert widget.mount_assumption_label.text().startswith("장착 가정: 체커보드")
    assert "Left Effective Extrinsic" in left_effective_extrinsic_text
    assert "Right Effective Extrinsic" in right_effective_extrinsic_text
    assert "mode: checkerboard stereo R/T" in left_effective_extrinsic_text
    assert "mode: checkerboard stereo R/T" in right_effective_extrinsic_text
    assert "source: applied checkerboard stereo extrinsics" in left_effective_extrinsic_text
    assert "source: applied checkerboard stereo extrinsics" in right_effective_extrinsic_text
    assert "used_for_triangulation: true" in left_effective_extrinsic_text
    assert "used_for_triangulation: true" in right_effective_extrinsic_text
    assert "origin_in_left_rdf_m: 0.000000000, 0.000000000, 0.000000000" in left_effective_extrinsic_text
    assert "translation_right_in_left_m: -0.300000000, -0.000997000, 0.001999000" in right_effective_extrinsic_text
    assert "Live Pixhawk diagnostic only" in left_effective_extrinsic_text
    assert "source: MAVLink (mavlink/ATTITUDE)" in left_effective_extrinsic_text
    assert "used_for_triangulation: false" in left_effective_extrinsic_text
    session_metadata = widget._recording_session_metadata()
    assert session_metadata["schema_version"] == "stereowiris-paired-frame-v2"
    assert session_metadata["calculation"]["triangulation_extrinsics_source"] == "checkerboard"
    assert session_metadata["calculation"]["baseline_source"] == "checkerboard"
    assert session_metadata["calculation"]["measurement_frame"] == "baseline_center_frd"
    assert session_metadata["calculation"]["legacy_xyz_alias_contract"] == "not_emitted"
    assert session_metadata["calculation"]["baseline_frame_scope"] == "measurement_attempt"
    assert session_metadata["calculation"]["baseline_frame_origin_model"] == (
        "checkerboard_stereo_rt_optical_centers"
    )
    checkerboard_metadata = session_metadata["checkerboard_stereo_extrinsics"]
    assert checkerboard_metadata["active"] is True
    assert checkerboard_metadata["pair_count"] == 12
    assert checkerboard_metadata["translation_left_to_right_m"] == [0.3, 0.001, -0.002]
    assert checkerboard_metadata["right_camera_origin_in_left_rdf_m"] is not None

    widget.deleteLater()
    _ = app


def test_optical_alignment_tab_reports_live_bbox_residuals(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    from wiris_stereo_distance.calibration import (
        CameraIntrinsics,
        CheckerboardConfig,
        StereoExtrinsics,
        StereoIntrinsicsResult,
    )
    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui, "load_settings", lambda: gui.AppSettings(auto_load_intrinsics=False))
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget.last_calibration_result = StereoIntrinsicsResult(
        left=CameraIntrinsics(
            width=1280,
            height=720,
            camera_matrix=[[900.0, 0.0, 640.0], [0.0, 900.0, 360.0], [0.0, 0.0, 1.0]],
            distortion_coefficients=[0.0],
            reprojection_error_px=0.2,
            image_count=20,
        ),
        right=CameraIntrinsics(
            width=1280,
            height=720,
            camera_matrix=[[900.0, 0.0, 640.0], [0.0, 900.0, 365.0], [0.0, 0.0, 1.0]],
            distortion_coefficients=[0.0],
            reprojection_error_px=0.2,
            image_count=20,
        ),
        board=CheckerboardConfig(inner_cols=9, inner_rows=6, square_size_m=0.03),
        extrinsics=StereoExtrinsics(
            rotation_left_to_right=[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            translation_left_to_right_m=[-0.3, 0.0, 0.0],
            reprojection_error_px=0.2,
            pair_count=20,
        ),
    )
    widget.apply_intrinsics_to_distance()
    widget.distance_mode_combo.setCurrentIndex(widget.distance_mode_combo.findData("checkerboard"))
    widget.optical_expected_dy_combo.setCurrentIndex(widget.optical_expected_dy_combo.findData("raw_intrinsics"))
    widget.optical_target_distance_spin.setValue(10.0)

    widget._update_detection("left", detection("left", x1=630, y1=340, x2=650, y2=360, t=10.0))
    widget._update_detection("right", detection("right", x1=576, y1=347, x2=596, y2=367, t=10.0))

    assert widget.alignment_tabs.tabText(widget.optical_alignment_tab_index) == "Optical 정렬"
    assert "Optical alignment:" in widget.optical_alignment_status_label.text()
    assert "x=640.0, y=350.0" in widget.optical_alignment_labels["left_center"].text()
    assert "x=586.0, y=357.0" in widget.optical_alignment_labels["right_center"].text()
    assert "dx=+54.000 px, dy=-7.000 px" in widget.optical_alignment_labels["bbox_delta"].text()
    assert "-5.000 px" in widget.optical_alignment_labels["expected_dy"].text()
    assert "-2.000 px" in widget.optical_alignment_labels["dy_residual"].text()
    assert "target=10.00 m" in widget.optical_alignment_labels["expected_disparity"].text()
    assert "checkerboard:" in widget.optical_alignment_labels["baseline"].text()
    assert widget.optical_alignment_labels["range"].text() != "대기 중"
    assert widget.optical_alignment_note_label.text().startswith("Alignment preview only")

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
    calibration_group = next(group for group in widget.findChildren(QtWidgets.QGroupBox) if group.title() == "체커보드 캡처")

    assert zoom_group is not None
    assert zoom_group.isCheckable()
    assert zoom_group.isChecked()
    assert zoom_group.geometry().top() < calibration_group.geometry().top()
    assert widget.zoom_in_button.isVisible()
    assert widget.calib_cols_spin.value() == 13
    assert widget.calib_rows_spin.value() == 8
    assert widget.calib_square_spin.value() == 40.0
    assert widget.calib_min_images_spin.value() == 20
    assert widget.calibration_tabs.tabText(widget.checkerboard_validation_tab_index) == "체커보드 검증/계산"

    calib_form = calibration_group.layout()
    assert isinstance(calib_form, QtWidgets.QGridLayout)

    def form_position(control: QtWidgets.QWidget) -> tuple[int, int, int, int]:
        index = calib_form.indexOf(control)
        assert index >= 0
        return calib_form.getItemPosition(index)

    assert form_position(widget.calib_cols_spin) == (0, 1, 1, 1)
    assert form_position(widget.calib_rows_spin) == (0, 3, 1, 1)
    assert form_position(widget.calib_square_spin) == (0, 5, 1, 1)
    assert form_position(widget.calib_dir_edit) == (1, 1, 1, 5)
    assert form_position(widget.capture_calibration_button) == (1, 6, 1, 1)
    assert calib_form.indexOf(widget.run_calibration_button) == -1
    assert widget.calibration_status_label.objectName() == "statusStrip"
    assert calib_form.indexOf(widget.calibration_status_label) == -1
    assert widget.calibration_status_label.geometry().top() > calibration_group.geometry().bottom()

    for column, label_text in ((0, "내부 열"), (2, "내부 행"), (4, "격자 크기")):
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
        widget.calib_dir_edit,
    ):
        assert field.property("compactFormField") is True
    assert widget.calib_min_images_spin.parentWidget().title() == "계산 / 검증"
    assert widget.run_calibration_button.text() == "전체 사진으로 계산"
    assert widget.checkerboard_validation_save_button.text() == "계산 결과 저장"
    assert widget.checkerboard_validation_apply_button.text() == "계산에 적용"

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
    widget.camera_frame_metadata["left"] = {"rtcp_frame_ntp_unix_s": 1_800_000_000.000}
    widget.camera_frame_metadata["right"] = {"rtcp_frame_ntp_unix_s": 1_800_000_000.020}

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


def test_calibration_capture_rejects_stereo_pair_without_rtcp_timestamps(tmp_path, monkeypatch) -> None:
    from PyQt5 import QtGui, QtWidgets

    import wiris_stereo_distance.gui as gui

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

    widget.capture_calibration_images()

    root = tmp_path / "calibration_captures"
    assert not (root / "left" / "left_001.png").exists()
    assert not (root / "right" / "right_001.png").exists()
    assert "pair=rejected" in widget.calibration_status_label.text()
    assert "missing RTCP/NTP camera timestamp" in widget.calibration_status_label.text()

    widget.deleteLater()
    _ = app


def test_calibration_capture_rejects_stereo_pair_when_rtcp_delta_is_large(tmp_path, monkeypatch) -> None:
    from PyQt5 import QtGui, QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget._stop_calibration_preview_worker()
    widget.calib_dir_edit.setText(str(tmp_path / "calibration_captures"))
    image = QtGui.QImage(80, 60, QtGui.QImage.Format_RGB888)
    image.fill(QtGui.QColor("black"))
    widget.left_pane.latest_image = image
    widget.right_pane.latest_image = image
    widget.camera_frame_metadata["left"] = {"rtcp_frame_ntp_unix_s": 1_800_000_000.000}
    widget.camera_frame_metadata["right"] = {"rtcp_frame_ntp_unix_s": 1_800_000_000.100}

    monkeypatch.setattr(gui, "opencv_available", lambda: (True, None))

    widget.capture_calibration_images()

    root = tmp_path / "calibration_captures"
    assert not (root / "left" / "left_001.png").exists()
    assert not (root / "right" / "right_001.png").exists()
    assert "pair=rejected" in widget.calibration_status_label.text()
    assert "exceeds 50.0 ms" in widget.calibration_status_label.text()

    widget.deleteLater()
    _ = app


def test_checkerboard_validation_pixhawk_mode_uses_intrinsics_only_and_applies_mode(tmp_path, monkeypatch) -> None:
    from PyQt5 import QtWidgets

    from wiris_stereo_distance.calibration import CameraIntrinsics, StereoExtrinsics
    import wiris_stereo_distance.gui as gui

    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    monkeypatch.setattr(gui, "save_settings", lambda settings: None)
    monkeypatch.setattr(gui, "opencv_available", lambda: (True, None))
    root = tmp_path / "calibration_captures"
    for side in ("left", "right"):
        directory = root / side
        directory.mkdir(parents=True)
        for index in range(1, 5):
            (directory / f"{side}_{index:03d}.png").write_text("fake")

    stereo_calls = []

    def fake_calibrate(paths, _config):
        first = paths[0].name
        fx = 900.0 if first.startswith("left_") else 910.0
        return CameraIntrinsics(
            width=1280,
            height=720,
            camera_matrix=[[fx, 0.0, 640.0], [0.0, fx + 1.0, 360.0], [0.0, 0.0, 1.0]],
            distortion_coefficients=[0.0],
            reprojection_error_px=0.4,
            image_count=len(paths),
        )

    def fake_stereo(*args, **kwargs):
        stereo_calls.append((args, kwargs))
        return StereoExtrinsics(
            rotation_left_to_right=[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            translation_left_to_right_m=[-0.3, 0.0, 0.0],
            reprojection_error_px=0.2,
            pair_count=4,
        )

    monkeypatch.setattr(gui, "calibrate_camera_from_images", fake_calibrate)
    monkeypatch.setattr(gui, "calibrate_stereo_extrinsics_from_image_pairs", fake_stereo)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget.calib_dir_edit.setText(str(root))
    widget.checkerboard_validation_root_edit.setText(str(root))
    widget.checkerboard_validation_mode_combo.setCurrentIndex(
        widget.checkerboard_validation_mode_combo.findData("pixhawk")
    )

    widget.run_intrinsic_calibration()

    assert widget.checkerboard_validation_result is not None
    assert widget.checkerboard_validation_result.extrinsics is None
    assert stereo_calls == []
    assert "Extrinsics: not computed" in widget.calibration_result_text.toPlainText()

    widget.save_checkerboard_validation_result()
    assert (root / "intrinsics_zoom_10.json").exists()
    widget.apply_checkerboard_validation_result()

    assert widget.active_intrinsics is widget.checkerboard_validation_result
    assert widget.distance_mode_combo.currentData() == "pixhawk"

    widget.deleteLater()
    _ = app


def test_checkerboard_validation_checkerboard_mode_uses_stereo_pairs_and_random_validation(tmp_path, monkeypatch) -> None:
    from PyQt5 import QtWidgets

    from wiris_stereo_distance.calibration import CameraIntrinsics, StereoExtrinsics
    import wiris_stereo_distance.gui as gui

    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    monkeypatch.setattr(gui, "save_settings", lambda settings: None)
    monkeypatch.setattr(gui, "opencv_available", lambda: (True, None))
    root = tmp_path / "calibration_captures"
    (root / "left").mkdir(parents=True)
    (root / "right").mkdir(parents=True)
    for index in range(1, 7):
        (root / "left" / f"left_{index:03d}.png").write_text("fake")
        if index <= 5:
            (root / "right" / f"right_{index:03d}.png").write_text("fake")
    calls = {"calibrate": [], "stereo": []}

    def fake_calibrate(paths, _config):
        calls["calibrate"].append([path.name for path in paths])
        first = paths[0].name
        fx = 900.0 if first.startswith("left_") else 910.0
        return CameraIntrinsics(
            width=1280,
            height=720,
            camera_matrix=[[fx + len(paths), 0.0, 640.0], [0.0, fx + len(paths), 360.0], [0.0, 0.0, 1.0]],
            distortion_coefficients=[0.0],
            reprojection_error_px=0.3 + len(paths) * 0.01,
            image_count=len(paths),
        )

    def fake_stereo(left_paths, right_paths, _config, _left, _right):
        calls["stereo"].append(([path.name for path in left_paths], [path.name for path in right_paths]))
        return StereoExtrinsics(
            rotation_left_to_right=[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            translation_left_to_right_m=[-0.3 - len(left_paths) * 0.001, 0.0, 0.0],
            reprojection_error_px=0.2 + len(left_paths) * 0.01,
            pair_count=len(left_paths),
        )

    monkeypatch.setattr(gui, "calibrate_camera_from_images", fake_calibrate)
    monkeypatch.setattr(gui, "calibrate_stereo_extrinsics_from_image_pairs", fake_stereo)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget.calib_dir_edit.setText(str(root))
    widget.checkerboard_validation_root_edit.setText(str(root))
    widget.checkerboard_validation_mode_combo.setCurrentIndex(
        widget.checkerboard_validation_mode_combo.findData("checkerboard")
    )
    widget.checkerboard_validation_random_runs_spin.setValue(3)
    widget.checkerboard_validation_random_pair_count_spin.setValue(3)
    widget.checkerboard_validation_seed_spin.setValue(123)

    widget.scan_checkerboard_validation_files()
    widget.run_intrinsic_calibration()
    widget.run_checkerboard_random_validation()

    assert "stereo_pair=5" in widget.checkerboard_validation_scan_label.text()
    assert widget.checkerboard_validation_result is not None
    assert widget.checkerboard_validation_result.extrinsics is not None
    assert widget.checkerboard_validation_result.extrinsics.pair_count == 5
    assert all("left_006.png" not in names for names in calls["calibrate"])
    assert len(calls["stereo"]) == 4
    assert "Random stereo-pair validation" in widget.calibration_result_text.toPlainText()
    assert "Random 1" in widget.calibration_result_text.toPlainText()

    widget.apply_checkerboard_validation_result()

    assert widget.active_intrinsics is widget.checkerboard_validation_result
    assert widget.distance_mode_combo.currentData() == "checkerboard"

    widget.deleteLater()
    _ = app


def test_camera_parameters_tab_shows_sources_without_pixhawk(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    from wiris_stereo_distance.calibration import CameraIntrinsics, CheckerboardConfig, StereoIntrinsicsResult
    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui, "load_settings", lambda: gui.AppSettings(auto_load_intrinsics=False))
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
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


def test_pixhawk_rtk_can_drive_triangulation(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    from wiris_stereo_distance.calibration import CameraIntrinsics, CheckerboardConfig, StereoIntrinsicsResult
    import wiris_stereo_distance.gui as gui
    from wiris_stereo_distance.pixhawk_attitude import CameraExtrinsicState
    from wiris_stereo_distance.rtk_geometry import GpsPositionState, RtkBaselineEstimate

    monkeypatch.setattr(gui, "load_settings", lambda: gui.AppSettings(auto_load_intrinsics=False))
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
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
    set_selected_gps_baseline(widget, timestamp_s=10.0, baseline_m=0.3)
    for camera in ("left", "right"):
        widget._update_extrinsic(
            camera,
            CameraExtrinsicState(
                camera=camera,
                namespace=f"/uav_{0 if camera == 'left' else 1}",
                roll_deg=0.0,
                pitch_deg=0.0,
                yaw_deg=0.0,
                # Deliberately old relative to the t=10 detection pair.  The
                # UI must expose this without blocking ray triangulation.
                timestamp_s=-1.0,
                topic="mavlink/ATTITUDE",
                q=(1.0, 0.0, 0.0, 0.0),
            ),
        )
    widget.apply_intrinsics_to_distance()
    widget.distance_mode_combo.setCurrentIndex(widget.distance_mode_combo.findData("pixhawk"))
    widget.distance_recording_enabled = True
    widget._update_detection(
        "left",
        detection(
            "left",
            x1=690,
            y1=340,
            x2=710,
            y2=380,
            t=10.0,
            timestamp_metadata={"rtcp_frame_ntp_unix_s": 1_800_000_000.000},
        ),
    )
    widget._update_detection(
        "right",
        detection(
            "right",
            x1=640,
            y1=340,
            x2=660,
            y2=380,
            t=10.0,
            timestamp_metadata={"rtcp_frame_ntp_unix_s": 1_800_000_000.000},
        ),
    )

    assert "two_ray_closest_midpoint-pixhawk" in widget.distance_label.text()
    assert "baseline=gps-rtk:0.300m" in widget.distance_label.text()
    assert "ATT:VERY_STALE" in widget.distance_label.text()
    assert "quality=DEGRADED" in widget.distance_label.text()
    assert "telemetryRange" not in widget.distance_label.text()
    assert "gap:" in widget.triangulation_quality_gap_label.text()
    assert "mode: two_ray_closest_midpoint-pixhawk" in widget.triangulation_quality_mode_label.text()
    assert "Attitude timing [VERY_STALE]" in widget.triangulation_quality_attitude_label.text()
    assert "Left age 11.000 s" in widget.triangulation_quality_attitude_label.text()
    assert widget.triangulation_quality_attitude_label.property("healthLevel") == "bad"
    assert "Raw baseline_center_frd point [F/R/D]:" in widget.triangulation_quality_raw_point_label.text()
    assert "source left_camera_rdf [R/D/F]:" in widget.triangulation_quality_raw_point_label.text()
    assert "3D ray 거리: Pixhawk 자세 + GPS/RTK baseline 사용" in widget.stereo_geometry_text.toPlainText()
    assert "위치/자세 출처=Pixhawk 자세 + GPS/RTK baseline" in widget.distance_parameters_label.text()
    row = widget.distance_recording_rows[-1]
    assert row["method"] == "two_ray_closest_midpoint-pixhawk"
    assert row["schema_version"] == "stereowiris-paired-frame-v2"
    assert row["measurement_frame"] == "baseline_center_frd"
    assert row["baseline_frame_scope"] == "measurement_attempt"
    assert row["baseline_frame_origin_model"] == "gps_rtk_antenna_baseline_proxy_no_lever_arm"
    assert row["baseline_center_forward_m"] == row["raw_baseline_center_forward_m"]
    assert row["baseline_center_right_m"] == row["raw_baseline_center_right_m"]
    assert row["baseline_center_down_m"] == row["raw_baseline_center_down_m"]
    assert row["baseline_center_range_m"] == row["range_m"]
    assert "left_camera_right_m" not in row
    assert "stereo_center_right_m" not in row
    assert row["attitude_timing_status"] == "VERY_STALE"
    assert row["attitude_timing_policy"] == "display_only_non_blocking"
    assert row["attitude_timing_blocks_measurement"] == "false"
    assert row["attitude_stale_age_threshold_s"] == "2.000"
    assert row["attitude_very_stale_age_threshold_s"] == "10.000"
    assert row["attitude_stale_pair_delta_threshold_s"] == "1.000"
    assert row["attitude_very_stale_pair_delta_threshold_s"] == "5.000"
    assert "telemetry_range_m" not in row
    assert "live_truth_range_m" not in row
    assert "corrected_range_m" not in row
    assert "filtered_range_m" not in row

    widget.deleteLater()
    _ = app


def test_measurement_attempt_freezes_gui_geometry_and_valid_row_provenance(monkeypatch, tmp_path) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui
    from wiris_stereo_distance.calibration import CameraIntrinsics, CheckerboardConfig, StereoIntrinsicsResult
    from wiris_stereo_distance.measurement_controller import calculate_measurement_geometry
    from wiris_stereo_distance.pixhawk_attitude import CameraExtrinsicState
    from wiris_stereo_distance.recording_session import write_dynamic_csv
    from wiris_stereo_distance.rtk_geometry import GpsPositionState, RtkBaselineEstimate

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    monkeypatch.setattr(gui, "load_settings", lambda: gui.AppSettings(auto_load_intrinsics=False))
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget.distance_mode_combo.setCurrentIndex(widget.distance_mode_combo.findData("pixhawk"))
    intrinsics_a = CameraIntrinsics(
        1280,
        720,
        [[900.0, 0.0, 640.0], [0.0, 900.0, 360.0], [0.0, 0.0, 1.0]],
        [0.0],
        0.2,
        20,
    )
    widget.last_calibration_result = StereoIntrinsicsResult(
        intrinsics_a, intrinsics_a, CheckerboardConfig(9, 6, 0.03)
    )
    widget.apply_intrinsics_to_distance()
    set_selected_gps_baseline(widget, timestamp_s=10.0, baseline_m=0.3)
    widget.latest_rtk_baseline = RtkBaselineEstimate(0.3, 0.0, 0.3, 0.0, 0.0, 0.0, "GOOD")
    for camera in ("left", "right"):
        widget._update_extrinsic(
            camera,
            CameraExtrinsicState(
                camera,
                camera,
                0.0,
                0.0,
                0.0,
                10.0,
                f"/{camera}/attitude-a",
                (1.0, 0.0, 0.0, 0.0),
                9.0,
            ),
        )
    left = detection("left", x1=630, y1=350, x2=650, y2=370, t=10.0)
    right = detection("right", x1=603, y1=350, x2=623, y2=370, t=10.0)
    widget.detection_buffers["left"].add(left)
    widget.detection_buffers["right"].add(right)
    pair = widget._matched_detection_pair()
    assert pair is not None
    attempt_a = widget._build_measurement_attempt(pair)
    geometry_a = calculate_measurement_geometry(attempt_a)
    assert geometry_a.triangulated is not None
    hash_a = attempt_a.canonical_sha256()

    intrinsics_b = CameraIntrinsics(
        1280,
        720,
        [[1200.0, 0.0, 620.0], [0.0, 1200.0, 350.0], [0.0, 0.0, 1.0]],
        [0.1, 0.0, 0.0, 0.0, 0.0],
        9.0,
        1,
    )
    widget.active_intrinsics = StereoIntrinsicsResult(
        intrinsics_b, intrinsics_b, CheckerboardConfig(9, 6, 0.03)
    )
    widget.zoom_level_spin.blockSignals(True)
    widget.zoom_level_spin.setValue(3)
    widget.zoom_level_spin.blockSignals(False)
    widget.attitude_buffers["left"].clear()
    widget.attitude_buffers["right"].clear()
    for camera in ("left", "right"):
        state_b = CameraExtrinsicState(
            camera,
            camera,
            0.0,
            0.0,
            180.0,
            10.0,
            f"/{camera}/attitude-b",
            (0.0, 0.0, 0.0, 1.0),
            99.0,
        )
        widget.latest_extrinsics[camera] = state_b
        widget.attitude_buffers[camera].add(state_b)
    widget.latest_gps["left"] = GpsPositionState(
        "left", "left", 20.0, "/left/gps-b", 1.0, 1.0, 1.0, fix_type=3
    )
    widget.latest_gps["right"] = GpsPositionState(
        "right", "right", 21.0, "/right/gps-b", 2.0, 2.0, 2.0, fix_type=3
    )
    widget.latest_rtk_baseline = RtkBaselineEstimate(99.0, 99.0, 0.0, 0.0, 0.0, 0.0, "DEGRADED")

    assert widget._build_measurement_attempt(pair) is attempt_a
    assert widget._translation_right_in_left_m(attempt_a) == attempt_a.translation_right_in_left_m
    geometry_after = calculate_measurement_geometry(attempt_a)
    assert geometry_after.triangulated is not None
    assert geometry_after.triangulated.point_left_rdf_m == pytest.approx(
        geometry_a.triangulated.point_left_rdf_m, abs=1e-9
    )
    assert geometry_after.triangulated.range_m == pytest.approx(geometry_a.triangulated.range_m, abs=1e-9)
    assert attempt_a.canonical_sha256() == hash_a

    widget.distance_recording_enabled = True
    widget._refresh_distance()
    row = widget.distance_recording_rows[-1]
    assert row["measurement_attempt_id"] == str(attempt_a.attempt_id)
    assert row["measurement_attempt_sha256"] == hash_a
    assert row["measurement_attempt_content_sha256"] == attempt_a.content_sha256()
    assert row["measurement_attempt_instance_sha256"] == attempt_a.instance_sha256()
    assert list(row)[:5] == [
        "session_id",
        "measurement_attempt_id",
        "measurement_attempt_session_id",
        "measurement_attempt_sha256",
        "measurement_detection_pair_key",
    ]
    assert list(row)[-2:] == [
        "measurement_attempt_content_sha256",
        "measurement_attempt_instance_sha256",
    ]
    assert row["left_pixhawk_q"] == "[1.0,0.0,0.0,0.0]"
    assert row["left_pixhawk_topic"] == "/left/attitude-a"
    assert row["left_gps_lon_deg"] == "127.000000000"
    assert row["left_intr_fx"] == "900.000000"
    assert row["intrinsics_zoom_level"] != "3"
    assert row["baseline_left_camera_rdf_right_m"] == "0.300000"
    assert row["rtk_baseline_east_m"] == "0.300000"
    assert row["measurement_frame"] == "baseline_center_frd"

    output = tmp_path / "distance_history.csv"
    write_dynamic_csv(output, [row])
    with output.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        persisted = next(reader)
        expected_fieldnames = [
            "sample_index",
            "session_elapsed_s",
            *(key for key in row if key not in {"sample_index", "session_elapsed_s"}),
        ]
        assert reader.fieldnames == expected_fieldnames
    assert list(persisted) == expected_fieldnames
    assert persisted["measurement_attempt_sha256"] == hash_a
    assert persisted["measurement_attempt_content_sha256"] == attempt_a.content_sha256()
    assert persisted["measurement_attempt_instance_sha256"] == attempt_a.instance_sha256()

    widget._stop_calibration_preview_worker()
    widget.deleteLater()
    app.processEvents()


def test_measurement_attempt_identity_is_monotonic_across_failed_geometry(monkeypatch) -> None:
    from types import SimpleNamespace

    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui
    from wiris_stereo_distance.measurement_controller import calculate_measurement_geometry

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    monkeypatch.setattr(gui, "load_settings", lambda: gui.AppSettings(auto_load_intrinsics=False))
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget.distance_mode_combo.setCurrentIndex(widget.distance_mode_combo.findData("pixhawk"))
    first_pair = SimpleNamespace(
        left=detection("left", x1=630, y1=350, x2=650, y2=370, t=1.0),
        right=detection("right", x1=600, y1=350, x2=620, y2=370, t=1.0),
    )
    first = widget._build_measurement_attempt(first_pair)
    assert widget._build_measurement_attempt(first_pair) is first
    assert calculate_measurement_geometry(first).triangulated is None

    second_pair = SimpleNamespace(
        left=detection("left", x1=630, y1=350, x2=650, y2=370, t=2.0),
        right=detection("right", x1=600, y1=350, x2=620, y2=370, t=2.0),
    )
    second = widget._build_measurement_attempt(second_pair)
    assert second.attempt_id == first.attempt_id + 1

    widget._stop_calibration_preview_worker()
    widget.deleteLater()
    app.processEvents()


@pytest.mark.parametrize(
    ("left_class", "right_class", "left_candidates", "right_candidates", "expected_reason"),
    (
        (1, 2, (1,), (2,), "correspondence_class_mismatch"),
        (1, 1, (1, 1), (1,), "ambiguous_left_candidates"),
        (1, 1, (1,), (1, 1), "ambiguous_right_candidates"),
    ),
)
def test_measurement_correspondence_rejects_class_mismatch_and_ambiguity(
    monkeypatch,
    left_class,
    right_class,
    left_candidates,
    right_candidates,
    expected_reason,
) -> None:
    from dataclasses import replace

    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    monkeypatch.setattr(gui, "load_settings", lambda: gui.AppSettings(auto_load_intrinsics=False))
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    left = replace(
        detection("left", x1=630, y1=350, x2=650, y2=370, t=10.0),
        class_id=left_class,
        candidate_class_ids=left_candidates,
    )
    right = replace(
        detection("right", x1=600, y1=350, x2=620, y2=370, t=10.0),
        class_id=right_class,
        candidate_class_ids=right_candidates,
    )
    widget.detection_buffers["left"].add(left)
    widget.detection_buffers["right"].add(right)

    assert widget._matched_detection_pair() is None
    assert widget._last_correspondence_rejection_reason == expected_reason
    widget._refresh_distance()
    assert len(widget.distance_history) == 0
    assert len(widget.distance_recording_rows) == 0

    widget._stop_calibration_preview_worker()
    widget.deleteLater()
    app.processEvents()


def test_measurement_attempt_recovers_same_pair_when_required_inputs_arrive(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui
    from wiris_stereo_distance.calibration import CameraIntrinsics, CheckerboardConfig, StereoIntrinsicsResult
    from wiris_stereo_distance.measurement_controller import calculate_measurement_geometry
    from wiris_stereo_distance.pixhawk_attitude import CameraExtrinsicState

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    monkeypatch.setattr(gui, "load_settings", lambda: gui.AppSettings(auto_load_intrinsics=False))
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget.distance_mode_combo.setCurrentIndex(widget.distance_mode_combo.findData("pixhawk"))
    left = detection("left", x1=630, y1=350, x2=650, y2=370, t=10.0)
    right = detection("right", x1=603, y1=350, x2=623, y2=370, t=10.0)
    widget.detection_buffers["left"].add(left)
    widget.detection_buffers["right"].add(right)
    pair = widget._matched_detection_pair()
    assert pair is not None

    incomplete = widget._build_measurement_attempt(pair)
    assert not incomplete.is_geometry_ready
    assert widget._build_measurement_attempt(pair) is incomplete

    intrinsics = CameraIntrinsics(
        1280,
        720,
        [[900.0, 0.0, 640.0], [0.0, 900.0, 360.0], [0.0, 0.0, 1.0]],
        [0.0],
        0.2,
        20,
    )
    widget.active_intrinsics = StereoIntrinsicsResult(
        intrinsics, intrinsics, CheckerboardConfig(9, 6, 0.03)
    )
    set_selected_gps_baseline(widget, timestamp_s=10.0, baseline_m=0.3)
    for camera in ("left", "right"):
        state = CameraExtrinsicState(
            camera,
            camera,
            0.0,
            0.0,
            0.0,
            10.0,
            f"/{camera}/attitude-a",
            (1.0, 0.0, 0.0, 0.0),
            9.0,
        )
        widget.latest_extrinsics[camera] = state
        widget.attitude_buffers[camera].add(state)

    recovered = widget._build_measurement_attempt(pair)
    assert recovered is not incomplete
    assert recovered.attempt_id == incomplete.attempt_id + 1
    assert recovered.is_geometry_ready
    assert calculate_measurement_geometry(recovered).triangulated is not None
    assert widget._build_measurement_attempt(pair) is recovered

    widget._stop_calibration_preview_worker()
    widget.deleteLater()
    app.processEvents()


def test_checkerboard_triangulation_marks_pixhawk_attitude_not_used(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    from wiris_stereo_distance.calibration import (
        CameraIntrinsics,
        CheckerboardConfig,
        StereoExtrinsics,
        StereoIntrinsicsResult,
    )
    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui, "load_settings", lambda: gui.AppSettings(auto_load_intrinsics=False))
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
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
        extrinsics=StereoExtrinsics(
            rotation_left_to_right=[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            translation_left_to_right_m=[-0.3, 0.0, 0.0],
            reprojection_error_px=0.2,
            pair_count=20,
        ),
    )
    widget.apply_intrinsics_to_distance()
    widget.distance_mode_combo.setCurrentIndex(widget.distance_mode_combo.findData("checkerboard"))
    widget.distance_recording_enabled = True
    timestamp_metadata = {"rtcp_frame_ntp_unix_s": 1_800_000_000.000}
    widget._update_detection(
        "left",
        detection("left", x1=630, y1=350, x2=650, y2=370, t=10.0, timestamp_metadata=timestamp_metadata),
    )
    widget._update_detection(
        "right",
        detection("right", x1=576, y1=350, x2=596, y2=370, t=10.0, timestamp_metadata=timestamp_metadata),
    )

    assert "two_ray_closest_midpoint-checkerboard" in widget.distance_label.text()
    assert "ATT:N/A" in widget.distance_label.text()
    assert "Attitude timing [N/A]" in widget.triangulation_quality_attitude_label.text()
    assert "PIXHAWK ATTITUDE NOT USED" in widget.triangulation_quality_attitude_label.text()
    assert widget.triangulation_quality_attitude_label.property("healthLevel") == "idle"
    row = widget.distance_recording_rows[-1]
    assert row["attitude_used_for_triangulation"] == "false"
    assert row["attitude_timing_status"] == "NOT_USED"
    assert row["left_attitude_age_s"] == ""
    assert row["right_attitude_age_s"] == ""
    pair = widget._matched_detection_pair()
    attempt = widget._build_measurement_attempt(pair)
    assert attempt.left_attitude is None and attempt.right_attitude is None
    assert attempt.left_gps is None and attempt.right_gps is None
    assert attempt.measurement_frame == "baseline_center_frd"

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
    widget._update_detection(
        "left",
        detection(
            "left",
            x1=690,
            y1=340,
            x2=710,
            y2=380,
            t=10.0,
            timestamp_metadata={"rtcp_frame_ntp_unix_s": 1_800_000_000.000},
        ),
    )
    widget._update_detection(
        "right",
        detection(
            "right",
            x1=640,
            y1=340,
            x2=660,
            y2=380,
            t=10.0,
            timestamp_metadata={"rtcp_frame_ntp_unix_s": 1_800_000_000.000},
        ),
    )

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
    set_selected_gps_baseline(widget, timestamp_s=10.01, baseline_m=0.3)
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

    widget._update_detection(
        "left",
        detection(
            "left",
            x1=690,
            y1=340,
            x2=710,
            y2=380,
            t=10.0,
            timestamp_metadata={"rtcp_frame_ntp_unix_s": 1_800_000_000.000},
        ),
    )
    widget._update_detection("left", None)
    widget._update_detection(
        "right",
        detection(
            "right",
            x1=640,
            y1=340,
            x2=660,
            y2=380,
            t=10.02,
            timestamp_metadata={"rtcp_frame_ntp_unix_s": 1_800_000_000.020},
        ),
    )

    assert "거리:" in widget.distance_label.text()
    assert "m (" in widget.distance_label.text()
    assert len(widget.detection_buffers["left"]) == 1

    widget.deleteLater()
    _ = app


def test_yolo_matching_gate_limits_time_and_vertical_delta(monkeypatch) -> None:
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")

    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui
    from wiris_stereo_distance.calibration import CameraIntrinsics, CheckerboardConfig, StereoIntrinsicsResult
    from wiris_stereo_distance.pixhawk_attitude import CameraExtrinsicState
    from wiris_stereo_distance.rtk_geometry import RtkBaselineEstimate

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    monkeypatch.setattr(gui, "load_settings", lambda: gui.AppSettings(auto_load_intrinsics=False))
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
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
    set_selected_gps_baseline(widget, timestamp_s=10.05, baseline_m=0.3)
    for camera in ("left", "right"):
        widget._update_extrinsic(
            camera,
            CameraExtrinsicState(
                camera=camera,
                namespace=f"/uav_{0 if camera == 'left' else 1}",
                roll_deg=0.0,
                pitch_deg=0.0,
                yaw_deg=0.0,
                timestamp_s=10.05,
                topic="mavlink/ATTITUDE",
                q=(1.0, 0.0, 0.0, 0.0),
            ),
        )
    widget.apply_intrinsics_to_distance()
    widget.distance_mode_combo.setCurrentIndex(widget.distance_mode_combo.findData("pixhawk"))
    widget.yolo_gate_matching_checkbox.setChecked(True)
    widget.yolo_gate_time_delta_checkbox.setChecked(True)
    widget.yolo_gate_vertical_delta_checkbox.setChecked(True)
    widget.yolo_gate_time_delta_spin.setValue(0.05)
    widget.yolo_gate_vertical_delta_spin.setValue(100.0)
    widget.yolo_gate_time_basis_combo.setCurrentIndex(widget.yolo_gate_time_basis_combo.findData("detection"))

    widget._update_detection("left", detection("left", x1=690, y1=340, x2=710, y2=380, t=10.0))
    widget._update_detection("right", detection("right", x1=640, y1=340, x2=660, y2=380, t=10.1))

    assert "timestamp가 맞는 좌/우 검출 대기 중" in widget.distance_label.text()

    widget.detection_buffers["left"].clear()
    widget.detection_buffers["right"].clear()
    widget.yolo_gate_time_delta_spin.setValue(0.2)
    widget.yolo_gate_vertical_delta_spin.setValue(10.0)
    widget._update_detection("left", detection("left", x1=690, y1=340, x2=710, y2=380, t=10.0))
    widget._update_detection("right", detection("right", x1=640, y1=400, x2=660, y2=440, t=10.1))

    assert "거리: 유효한 스테레오 기하 대기 중" in widget.distance_label.text()

    widget.yolo_gate_vertical_delta_spin.setValue(100.0)
    widget._update_detection("left", detection("left", x1=690, y1=340, x2=710, y2=380, t=11.0))
    widget._update_detection("right", detection("right", x1=640, y1=400, x2=660, y2=440, t=11.1))

    assert "거리:" in widget.distance_label.text()
    assert "m (" in widget.distance_label.text()

    widget.deleteLater()
    _ = app


def test_yolo_matching_gate_prefers_camera_rtcp_time(monkeypatch) -> None:
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")

    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui
    from wiris_stereo_distance.calibration import CameraIntrinsics, CheckerboardConfig, StereoIntrinsicsResult
    from wiris_stereo_distance.pixhawk_attitude import CameraExtrinsicState
    from wiris_stereo_distance.rtk_geometry import RtkBaselineEstimate

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
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
    set_selected_gps_baseline(widget, timestamp_s=10.05, baseline_m=0.3)
    for camera in ("left", "right"):
        widget._update_extrinsic(
            camera,
            CameraExtrinsicState(
                camera=camera,
                namespace=f"/uav_{0 if camera == 'left' else 1}",
                roll_deg=0.0,
                pitch_deg=0.0,
                yaw_deg=0.0,
                timestamp_s=10.05,
                topic="mavlink/ATTITUDE",
                q=(1.0, 0.0, 0.0, 0.0),
            ),
        )
    widget.apply_intrinsics_to_distance()
    widget.distance_mode_combo.setCurrentIndex(widget.distance_mode_combo.findData("pixhawk"))
    widget.yolo_gate_matching_checkbox.setChecked(True)
    widget.yolo_gate_time_delta_checkbox.setChecked(True)
    widget.yolo_gate_time_delta_spin.setValue(0.05)
    widget.yolo_gate_vertical_delta_spin.setValue(100.0)
    widget.yolo_gate_time_basis_combo.setCurrentIndex(widget.yolo_gate_time_basis_combo.findData("camera_rtcp"))

    widget._update_detection(
        "left",
        detection(
            "left",
            x1=690,
            y1=340,
            x2=710,
            y2=380,
            t=10.0,
            timestamp_metadata={"rtcp_frame_ntp_unix_s": 1_800_000_000.000},
        ),
    )
    widget._update_detection(
        "right",
        detection(
            "right",
            x1=640,
            y1=340,
            x2=660,
            y2=380,
            t=10.1,
            timestamp_metadata={"rtcp_frame_ntp_unix_s": 1_800_000_000.020},
        ),
    )

    assert "거리:" in widget.distance_label.text()
    assert "m (" in widget.distance_label.text()
    assert widget.latest_left is not None
    assert widget.latest_right is not None
    assert abs(widget._yolo_match_timestamp_s(widget.latest_right) - widget._yolo_match_timestamp_s(widget.latest_left)) < 0.021

    widget.yolo_gate_time_basis_combo.setCurrentIndex(widget.yolo_gate_time_basis_combo.findData("detection"))
    widget._refresh_distance()

    assert "timestamp가 맞는 좌/우 검출 대기 중" in widget.distance_label.text()

    widget.deleteLater()
    _ = app


def test_yolo_matching_gate_falls_back_to_app_receive_time_when_rtcp_pair_drifts(monkeypatch) -> None:
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")

    from PyQt5 import QtWidgets
    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    now_s = time.monotonic()
    widget.yolo_gate_matching_checkbox.setChecked(True)
    widget.yolo_gate_time_delta_checkbox.setChecked(True)
    widget.yolo_gate_time_delta_spin.setValue(0.05)
    widget.yolo_gate_time_basis_combo.setCurrentIndex(widget.yolo_gate_time_basis_combo.findData("camera_rtcp"))

    left = detection(
        "left", x1=690, y1=340, x2=710, y2=380, t=now_s,
        timestamp_metadata={"timestamp_s": now_s, "rtcp_frame_ntp_unix_s": 1_800_000_000.000},
    )
    right = detection(
        "right", x1=640, y1=340, x2=660, y2=380, t=now_s + 0.010,
        timestamp_metadata={"timestamp_s": now_s + 0.010, "rtcp_frame_ntp_unix_s": 1_800_000_000.300},
    )
    widget._update_detection("left", left)
    widget._update_detection("right", right)

    pair = widget._matched_detection_pair()

    assert pair is not None
    assert widget._runtime_pair_uses_app_fallback is True
    assert abs(pair.delta_s - 0.010) < 1e-6
    assert widget._selected_pair_sync_summary(pair.left, pair.right) == ("app fallback", None)

    widget.deleteLater()
    _ = app


def test_camera_sync_status_does_not_gate_live_distance_pair(monkeypatch) -> None:
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")

    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget.camera_connected = True
    widget.camera_sync_status = gui.CAMERA_SYNC_STATUS_FAILED
    widget.yolo_gate_matching_checkbox.setChecked(False)
    now_s = time.monotonic()

    widget._update_detection(
        "left",
        detection("left", x1=690, y1=340, x2=710, y2=380, t=now_s - 0.010),
    )
    widget._update_detection(
        "right",
        detection("right", x1=640, y1=340, x2=660, y2=380, t=now_s),
    )

    pair = widget._matched_detection_pair()

    assert pair is not None
    assert widget._last_correspondence_rejection_reason == ""
    assert widget._yolo_time_basis() == "detection"
    assert widget._effective_yolo_time_basis() == "detection"

    widget.deleteLater()
    _ = app


def test_yolo_matching_gate_requires_camera_rtc_when_selected(monkeypatch) -> None:
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")

    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui
    from wiris_stereo_distance.calibration import CameraIntrinsics, CheckerboardConfig, StereoIntrinsicsResult
    from wiris_stereo_distance.pixhawk_attitude import CameraExtrinsicState
    from wiris_stereo_distance.rtk_geometry import RtkBaselineEstimate

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
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
                timestamp_s=10.05,
                topic="mavlink/ATTITUDE",
                q=(1.0, 0.0, 0.0, 0.0),
            ),
        )
    widget.apply_intrinsics_to_distance()
    widget.distance_mode_combo.setCurrentIndex(widget.distance_mode_combo.findData("pixhawk"))
    widget.yolo_gate_matching_checkbox.setChecked(True)
    widget.yolo_gate_time_delta_spin.setValue(0.05)
    widget.yolo_gate_vertical_delta_spin.setValue(100.0)
    widget.yolo_gate_time_basis_combo.setCurrentIndex(widget.yolo_gate_time_basis_combo.findData("camera_rtc_required"))

    widget._update_detection("left", detection("left", x1=690, y1=340, x2=710, y2=380, t=10.0))
    widget._update_detection("right", detection("right", x1=640, y1=340, x2=660, y2=380, t=10.008))

    assert "RTCP/NTP 필수 모드" in widget.distance_label.text()
    assert "timestamp가 맞는 좌/우 검출 대기 중" in widget.distance_label.text()

    widget.deleteLater()
    _ = app


def test_yolo_matching_gate_toggles_time_and_vertical_limits_independently(monkeypatch) -> None:
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")

    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget.yolo_gate_matching_checkbox.setChecked(True)
    widget.yolo_gate_time_delta_checkbox.setChecked(False)
    widget.yolo_gate_vertical_delta_checkbox.setChecked(False)
    widget.yolo_gate_time_delta_spin.setValue(0.05)
    widget.yolo_gate_vertical_delta_spin.setValue(10.0)
    widget.yolo_gate_time_basis_combo.setCurrentIndex(widget.yolo_gate_time_basis_combo.findData("camera_rtc_required"))
    widget.detection_buffers["left"].add(
        detection(
            "left",
            x1=690,
            y1=340,
            x2=710,
            y2=380,
            t=10.0,
            timestamp_metadata={"rtcp_frame_ntp_unix_s": 1_800_000_000.000},
        )
    )
    widget.detection_buffers["right"].add(
        detection(
            "right",
            x1=640,
            y1=400,
            x2=660,
            y2=440,
            t=11.0,
            timestamp_metadata={"rtcp_frame_ntp_unix_s": 1_800_000_001.000},
        )
    )

    pair = widget._matched_detection_pair()

    assert pair is not None
    assert abs(pair.delta_s - 1.0) < 1e-6
    assert widget._yolo_vertical_max_delta_px() == float("inf")
    assert not widget.yolo_gate_time_delta_spin.isEnabled()
    assert not widget.yolo_gate_vertical_delta_spin.isEnabled()
    assert widget.yolo_gate_time_basis_combo.isEnabled()

    widget.deleteLater()
    _ = app


def test_yolo_gate_off_wait_status_does_not_blame_delta_t(monkeypatch) -> None:
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")

    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    assert not hasattr(widget, "yolo_gate_min_bbox_checkbox")
    assert not hasattr(widget, "yolo_gate_min_width_spin")
    assert not hasattr(widget, "yolo_gate_min_height_spin")
    assert not hasattr(widget, "yolo_gate_min_area_spin")
    widget.yolo_gate_class_checkbox.setChecked(False)
    widget.yolo_gate_matching_checkbox.setChecked(False)
    widget.yolo_gate_time_delta_checkbox.setChecked(False)
    widget.yolo_gate_vertical_delta_checkbox.setChecked(False)

    widget._refresh_distance()

    assert "좌/우 검출 쌍 대기 중" in widget.distance_label.text()
    assert "timestamp가 맞는" not in widget.distance_label.text()
    assert "최대 Δt" not in widget.distance_label.text()
    assert "최대 Δt" not in widget.distance_plot.empty_message

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

    widget._update_detection(
        "left",
        detection(
            "left",
            x1=690,
            y1=340,
            x2=710,
            y2=380,
            t=10.0,
            timestamp_metadata={"rtcp_frame_ntp_unix_s": 1_800_000_000.000},
        ),
    )
    widget._update_detection(
        "right",
        detection(
            "right",
            x1=640,
            y1=340,
            x2=660,
            y2=380,
            t=10.0,
            timestamp_metadata={"rtcp_frame_ntp_unix_s": 1_800_000_000.000},
        ),
    )

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
    widget.tabs.setCurrentIndex(widget.parameter_tab_index)
    widget.parameter_tabs.setCurrentIndex(widget.stereo_settings_tab_index)
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


def test_wiris_pc_time_uses_camera_rtt_compensation_for_both_cameras(monkeypatch) -> None:
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
        return "OK"

    monkeypatch.setattr(gui, "send_tcp_line", fake_send_tcp_line)

    widget.set_selected_wiris_time_from_pc()

    assert len(sent_commands) == 8
    assert sent_commands.count("HIWS") == 6
    stus_commands = [command for command in sent_commands if command.startswith("STUS ")]
    assert len(stus_commands) == 2
    assert all(value is not None for value in widget._last_wiris_pc_utc_sync["stus_rtt_by_camera"].values())
    assert all(
        value >= 0.002
        for value in widget._last_wiris_pc_utc_sync["stus_compensation_by_camera"].values()
    )
    assert widget._last_wiris_pc_utc_sync["result"] == "success"
    assert "STUS RTT 보정" in widget.wiris_command_status_label.text()
    assert "명령 UTC 차이" in widget.wiris_command_status_label.text()

    widget.deleteLater()
    _ = app


def test_wiris_stus_sends_right_after_left_ack_with_late_target_time(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    target_hosts = {label: host for label, host in widget.zoom_hosts("both")}
    sent_stus = []
    clock_s = 1_800_000_000.0

    def fake_time() -> float:
        nonlocal clock_s
        value = clock_s
        clock_s += 0.500
        return value

    def fake_rtt(self, **_kwargs):
        return {
            "command": "HIWS",
            "samples_s": [0.010, 0.010, 0.010],
            "median_s": 0.010,
            "errors": [],
        }

    def fake_send_tcp_line(command):
        if command.command.startswith("STUS "):
            sent_stus.append((command.host, command.command))
        return "OK"

    monkeypatch.setattr(gui.time, "time", fake_time)
    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_measure_wiris_control_rtt", fake_rtt)
    monkeypatch.setattr(gui, "send_tcp_line", fake_send_tcp_line)

    widget.set_selected_wiris_time_from_pc(target="both")

    evidence = widget._last_wiris_pc_utc_sync
    assert [host for host, _command in sent_stus] == [target_hosts["left"], target_hosts["right"]]
    assert evidence["stus_send_order"] == ["left", "right"]
    assert evidence["stus_left_ack_received"] is True
    assert evidence["requested_utc_by_camera"]["right"] > evidence["requested_utc_by_camera"]["left"]
    assert "right waits for left STUS OK" in widget.clock_sync_summary_text.toPlainText()

    widget.deleteLater()
    _ = app


def test_wiris_stus_does_not_send_right_when_left_ack_fails(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    target_hosts = {label: host for label, host in widget.zoom_hosts("both")}
    sent_stus = []

    monkeypatch.setattr(
        gui.WirisStereoDistancePanel,
        "_measure_wiris_control_rtt",
        lambda self, **_kwargs: {
            "command": "HIWS",
            "samples_s": [0.010],
            "median_s": 0.010,
            "errors": [],
        },
    )

    def fake_send_tcp_line(command):
        if command.command.startswith("STUS "):
            sent_stus.append(command.host)
            return "ERR" if command.host == target_hosts["left"] else "OK"
        return "OK"

    monkeypatch.setattr(gui, "send_tcp_line", fake_send_tcp_line)

    widget.set_selected_wiris_time_from_pc(target="both")

    evidence = widget._last_wiris_pc_utc_sync
    assert sent_stus == [target_hosts["left"]]
    assert evidence["stus_send_order"] == ["left"]
    assert evidence["stus_left_ack_received"] is False
    assert evidence["result"] == "failed"
    assert "right" not in evidence["requested_utc_by_camera"]
    assert "오른쪽" in widget.wiris_command_status_label.text()
    assert "미전송" in widget.wiris_command_status_label.text()

    widget.deleteLater()
    _ = app


def test_time_tab_pc_time_button_uses_selected_wiris_target(monkeypatch) -> None:
    import threading

    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget.clock_wiris_time_target_combo.setCurrentIndex(widget.clock_wiris_time_target_combo.findData("both"))
    sent_commands = []
    lock = threading.Lock()

    def fake_send_tcp_line(command):
        with lock:
            sent_commands.append(command.command)
        return "OK"

    monkeypatch.setattr(gui, "send_tcp_line", fake_send_tcp_line)

    widget.wiris_set_pc_time_button.click()

    assert len(sent_commands) == 8
    assert sent_commands.count("HIWS") == 6
    stus_commands = [command for command in sent_commands if command.startswith("STUS ")]
    assert len(stus_commands) == 2
    assert "PC UTC 설정 전송" in widget.clock_sync_status_label.text()
    assert "STUS camera-specific target applied" in widget.clock_sync_summary_text.toPlainText()
    assert widget._last_wiris_pc_utc_sync["command"] == "STUS"
    assert widget._last_wiris_pc_utc_sync["result"] == "success"
    assert set(widget._last_wiris_pc_utc_sync["successful_targets"]) == {"left", "right"}
    assert set(widget._last_wiris_pc_utc_sync["requested_utc_by_camera"]) == {"left", "right"}
    widget.deleteLater()
    _ = app


def test_wiris_gdti_parser_accepts_sdk_datetime_text() -> None:
    from wiris_stereo_distance.gui import WirisStereoDistancePanel

    panel = WirisStereoDistancePanel.__new__(WirisStereoDistancePanel)

    parsed = panel._parse_wiris_gdti_unix_s("OK 2026/08/01-12:34:56.123456")

    assert parsed is not None
    assert parsed == pytest.approx(1785587696.123456, abs=1e-6)


def test_wiris_stus_compensation_rejects_invalid_rtt() -> None:
    from wiris_stereo_distance.gui import WirisStereoDistancePanel

    assert WirisStereoDistancePanel._stus_one_way_delay_s(0.012) == pytest.approx(0.006)
    assert WirisStereoDistancePanel._stus_compensation_s(0.012) == pytest.approx(0.008)
    assert WirisStereoDistancePanel._stus_compensation_s(None) == 0.0
    assert WirisStereoDistancePanel._stus_compensation_s(float("nan")) == 0.0
    assert WirisStereoDistancePanel._stus_compensation_s(0.251) == 0.0


def test_wiris_gdti_verification_uses_query_midpoint_and_integer_resolution() -> None:
    from wiris_stereo_distance.gui import WirisStereoDistancePanel

    panel = WirisStereoDistancePanel.__new__(WirisStereoDistancePanel)
    panel._camera_sync_requested_utc_s = 1_800_000_000.0
    panel._last_clock_command_timing = {
        "left": {"started_utc_s": 1_800_000_010.2, "finished_utc_s": 1_800_000_010.4},
        "right": {"started_utc_s": 1_800_000_010.2, "finished_utc_s": 1_800_000_010.4},
    }
    panel._send_clock_wiris_command_parallel = lambda _command: [
        ("left", "OK 2027/01/15-08:00:10"),
        ("right", "OK 2027/01/15-08:00:10"),
    ]

    values = panel._read_camera_gdti_after_stus()

    assert values["left"]["clock_valid"] is True
    assert values["left"]["query_offset_ms"] == pytest.approx(-300.0)
    assert values["left"]["offset_tolerance_ms"] == pytest.approx(600.0)
    assert values["left"]["offset_ms"] > 5_000.0


def test_camera_sync_readiness_requires_fresh_rtcp_and_gdti() -> None:
    from wiris_stereo_distance.gui import (
        CAMERA_SYNC_STATUS_WARMING,
        WirisStereoDistancePanel,
    )

    panel = WirisStereoDistancePanel.__new__(WirisStereoDistancePanel)
    panel.camera_connected = True
    panel.camera_sync_status = CAMERA_SYNC_STATUS_WARMING
    panel._camera_sync_requested_utc_s = 1_800_000_000.0
    panel._camera_sync_gdti = {
        "left": {"unix_s": 1_800_000_000.01},
        "right": {"unix_s": 1_800_000_000.02},
    }
    metadata = {
        "rtcp_sr_received_count": 2,
        "rtcp_sr_age_s": 0.2,
        "rtcp_camera_ntp_rate": 1.0001,
        "rtcp_camera_media_rate": 0.9999,
        "rtcp_frame_ntp_unix_s": 1_800_000_000.0,
        "rtcp_frame_receive_delay_s": 0.05,
        "rtcp_sr_pc_offset_s": 0.02,
        "timestamp_s": 10.0,
    }
    panel.camera_frame_metadata = {"left": dict(metadata), "right": dict(metadata)}
    panel._camera_name_ko = lambda camera: camera
    panel._camera_time_sync_metrics = lambda *_args, **_kwargs: {
        "pair_delta_s": 0.01,
        "receive_pair_delta_s": 0.01,
        "max_sr_pc_offset_ms": 20.0,
        "max_pc_offset_ms": 20.0,
    }

    ready, reason = panel._camera_sync_readiness()

    assert ready is True
    assert "검증 완료" in reason


def test_camera_sync_separates_frame_history_window_from_sr_stale_budget() -> None:
    import wiris_stereo_distance.gui as gui

    assert gui.CAMERA_SYNC_FRAME_HISTORY_WINDOW_S == 1.0
    assert gui.CAMERA_SYNC_HISTORY_MAX_AGE_S == 1.0
    assert gui.CAMERA_SYNC_MAX_SR_AGE_S == 7.0


def test_time_tab_stream_sync_status_reports_rtcp_pc_offsets(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    monkeypatch.setattr(gui.time, "time", lambda: 1_800_000_010.0)
    monkeypatch.setattr(gui.time, "monotonic", lambda: 110.0)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget.camera_frame_metadata["left"] = {
        "timestamp_s": 109.5,
        "rtcp_frame_ntp_unix_s": 1_800_000_009.510,
        "rtcp_sr_received_count": 4,
        "timestamp_reference": "rtcp-ntp",
    }
    widget.camera_frame_metadata["right"] = {
        "timestamp_s": 109.5,
        "rtcp_frame_ntp_unix_s": 1_800_000_009.530,
        "rtcp_sr_received_count": 5,
        "timestamp_reference": "rtcp-ntp",
    }

    widget.refresh_clock_sync_status()

    status = widget.clock_sync_status_label.text()
    detail = widget.clock_sync_summary_text.toPlainText()
    assert "좌우 RTCP/NTPΔ=+20.0 ms" in status
    assert "PC UTC 최대 offset=30.0 ms" in status
    assert "RTCP SR 수신: L=4 R=5" in status
    assert "RTCP SR 수신: L=4 R=5" in detail
    assert "왼쪽: offset=+10.0 ms" in detail
    assert "오른쪽: offset=+30.0 ms" in detail
    assert "불량" not in status
    assert "주의" not in detail
    assert "time_basis: RTCP Sender Report" in detail

    widget.deleteLater()
    _ = app


def test_camera_time_sync_metrics_separate_relative_pair_from_absolute_pc_quality(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    left = {"rtcp_frame_ntp_unix_s": 1_800_000_009.510}
    right = {"rtcp_frame_ntp_unix_s": 1_800_000_009.530}

    within = widget._camera_time_sync_metrics(
        left,
        right,
        left_receive_mono_s=109.5,
        right_receive_mono_s=109.5,
        now_wall_s=1_800_000_010.0,
        now_mono_s=110.0,
    )
    assert within["status"] == "within-threshold-unverified"
    assert within["pair_delta_s"] == pytest.approx(0.020)
    assert within["max_pc_offset_ms"] == pytest.approx(30.0)

    bad_absolute = widget._camera_time_sync_metrics(
        {"rtcp_frame_ntp_unix_s": 1_800_000_008.510},
        {"rtcp_frame_ntp_unix_s": 1_800_000_008.530},
        left_receive_mono_s=109.5,
        right_receive_mono_s=109.5,
        now_wall_s=1_800_000_010.0,
        now_mono_s=110.0,
    )
    assert bad_absolute["status"] == "pc-offset-failed"

    bad_relative = widget._camera_time_sync_metrics(
        left,
        {"rtcp_frame_ntp_unix_s": 1_800_000_009.610},
        left_receive_mono_s=109.5,
        right_receive_mono_s=109.5,
        now_wall_s=1_800_000_010.0,
        now_mono_s=110.0,
    )
    assert bad_relative["status"] == "pair-delta-failed"

    absolute_invalid = widget._camera_time_sync_metrics(
        {
            "timestamp_s": 109.5,
            "rtcp_frame_ntp_unix_s": 1_800_000_009.510,
            "rtcp_timing_invalid_reason": "sender-report-ntp-pc-offset",
        },
        {
            "timestamp_s": 109.5,
            "rtcp_frame_ntp_unix_s": 1_800_000_009.530,
            "rtcp_timing_invalid_reason": "sender-report-ntp-pc-offset",
        },
        now_wall_s=1_800_000_010.0,
        now_mono_s=110.0,
    )
    assert absolute_invalid["status"] == "absolute-rtcp-invalid"
    assert absolute_invalid["pair_delta_s"] == pytest.approx(0.020)
    assert absolute_invalid["max_pc_offset_ms"] is None
    assert "절대 UTC" in absolute_invalid["warning"]

    widget.deleteLater()
    _ = app


def test_clock_sync_status_selects_corresponding_history_frames(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget.camera_frame_metadata["left"] = {
        "timestamp_s": 109.99,
        "rtcp_frame_ntp_unix_s": 1_800_000_104.000,
    }
    widget.camera_frame_metadata["right"] = {
        "timestamp_s": 109.99,
        "rtcp_frame_ntp_unix_s": 1_800_000_108.000,
    }
    widget.camera_frame_metadata_history["left"].extend(
        [
            {"timestamp_s": 109.90, "rtcp_frame_ntp_unix_s": 1_800_000_100.000},
            {"timestamp_s": 109.99, "rtcp_frame_ntp_unix_s": 1_800_000_104.000},
        ]
    )
    widget.camera_frame_metadata_history["right"].extend(
        [
            {"timestamp_s": 109.92, "rtcp_frame_ntp_unix_s": 1_800_000_100.018},
            {"timestamp_s": 109.99, "rtcp_frame_ntp_unix_s": 1_800_000_108.000},
        ]
    )

    left, right = widget._clock_sync_pair_metadata(now_mono_s=110.0)

    assert left["rtcp_frame_ntp_unix_s"] == 1_800_000_100.000
    assert right["rtcp_frame_ntp_unix_s"] == 1_800_000_100.018

    widget.deleteLater()
    _ = app


def test_clock_sync_pair_metadata_rejects_history_frames_older_than_one_second(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    assert gui.CAMERA_SYNC_HISTORY_MAX_AGE_S == 1.0

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget.camera_frame_metadata["left"] = {
        "timestamp_s": 109.9,
        "rtcp_frame_ntp_unix_s": 1_800_000_200.000,
    }
    widget.camera_frame_metadata["right"] = {
        "timestamp_s": 109.9,
        "rtcp_frame_ntp_unix_s": 1_800_000_200.050,
    }

    def set_history(age_s: float) -> None:
        for camera, ntp_s in (("left", 1_800_000_100.000), ("right", 1_800_000_100.010)):
            widget.camera_frame_metadata_history[camera].clear()
            widget.camera_frame_metadata_history[camera].append(
                {"timestamp_s": 110.0 - age_s, "rtcp_frame_ntp_unix_s": ntp_s}
            )

    # 0.5 s old: inside the window, so the history pair is selected.
    set_history(0.5)
    left, right = widget._clock_sync_pair_metadata(now_mono_s=110.0)
    assert left["rtcp_frame_ntp_unix_s"] == 1_800_000_100.000
    assert right["rtcp_frame_ntp_unix_s"] == 1_800_000_100.010

    # 1.5 s old: outside the window (was inside under the previous 2.0 s
    # limit), so it falls back to the latest frame instead.
    set_history(1.5)
    left, right = widget._clock_sync_pair_metadata(now_mono_s=110.0)
    assert left["rtcp_frame_ntp_unix_s"] == 1_800_000_200.000
    assert right["rtcp_frame_ntp_unix_s"] == 1_800_000_200.050

    widget.deleteLater()
    _ = app


def test_clock_sync_pair_metadata_prefers_timestamp_pair_across_receive_skew(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    assert gui.CAMERA_SYNC_RECEIVE_PAIR_MAX_DELTA_S == 0.350
    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget.camera_frame_metadata["left"] = {
        "timestamp_s": 109.95,
        "rtcp_frame_ntp_unix_s": 1_800_000_300.000,
    }
    widget.camera_frame_metadata["right"] = {
        "timestamp_s": 109.95,
        "rtcp_frame_ntp_unix_s": 1_800_000_300.250,
    }
    widget.camera_frame_metadata_history["left"].append(
        {"timestamp_s": 109.50, "rtcp_frame_ntp_unix_s": 1_800_000_100.000}
    )
    widget.camera_frame_metadata_history["right"].append(
        {"timestamp_s": 109.75, "rtcp_frame_ntp_unix_s": 1_800_000_100.020}
    )

    left, right = widget._clock_sync_pair_metadata(now_mono_s=110.0)

    assert left["rtcp_frame_ntp_unix_s"] == 1_800_000_100.000
    assert right["rtcp_frame_ntp_unix_s"] == 1_800_000_100.020
    assert abs(float(right["timestamp_s"]) - float(left["timestamp_s"])) == pytest.approx(0.25)

    widget.deleteLater()
    _ = app


def test_camera_time_sync_metrics_reports_frame_transit_separately(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    left = {
        "rtcp_frame_ntp_unix_s": 1_800_000_009.510,
        "rtcp_frame_receive_delay_s": 1.200,
    }
    right = {
        "rtcp_frame_ntp_unix_s": 1_800_000_009.530,
        "rtcp_frame_receive_delay_s": 0.900,
    }

    metrics = widget._camera_time_sync_metrics(left, right)
    assert metrics["status"] == "within-threshold-unverified"
    assert metrics["pair_delta_s"] == pytest.approx(0.020)
    assert metrics["max_frame_transit_delay_ms"] == pytest.approx(1200.0)
    assert metrics["left_frame_transit_delay_ms"] == pytest.approx(1200.0)
    assert metrics["right_frame_transit_delay_ms"] == pytest.approx(900.0)
    assert metrics["max_pc_offset_ms"] is None

    widget.camera_frame_metadata["left"] = left
    widget.camera_frame_metadata["right"] = right
    widget.refresh_clock_sync_status()
    assert "RTCP frame transit max=1200.0 ms" in widget.clock_sync_status_label.text()
    assert "transit_delay=+1200.0 ms" in widget.clock_sync_summary_text.toPlainText()

    widget.deleteLater()
    _ = app


def test_recorded_distance_row_keeps_camera_time_sync_contract(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    left = detection(
        "left",
        x1=500,
        y1=200,
        x2=540,
        y2=260,
        t=109.5,
        timestamp_metadata={"rtcp_frame_ntp_unix_s": 1_800_000_009.510},
    )
    right = detection(
        "right",
        x1=450,
        y1=202,
        x2=490,
        y2=262,
        t=109.5,
        timestamp_metadata={"rtcp_frame_ntp_unix_s": 1_800_000_009.530},
    )
    estimate = type("Estimate", (), {"left": left, "right": right})()

    row = widget._record_measurement_sync_timestamp_parameters(estimate)

    assert row["camera_time_sync_status"] in {"within-threshold-unverified", "pc-offset-failed"}
    assert row["camera_time_sync_basis"] == "rtcp-sender-report-ntp"
    assert row["rtcp_pair_delta_s"] == "0.020000"
    assert "GDTI" in row["camera_time_sync_warning"] or row["camera_time_sync_status"] == "within-threshold-unverified"

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


def test_measurement_tab_shows_camera_ntp_in_summary_and_keeps_record_metadata() -> None:
    import time
    from types import SimpleNamespace

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
    summary = widget.runtime_camera_summary_label.text()
    assert "L" in summary and "R" in summary
    assert "NTP:22:13:20.100Z" in summary
    assert "NTP:22:13:20.118Z" in summary
    assert "Pair" in summary

    estimate = SimpleNamespace(
        left=detection(
            "left", x1=100, y1=100, x2=120, y2=120, t=base_s,
            timestamp_metadata=widget.camera_frame_metadata["left"],
        ),
        right=detection(
            "right", x1=90, y1=100, x2=110, y2=120, t=base_s + 0.015,
            timestamp_metadata=widget.camera_frame_metadata["right"],
        ),
        sync_quality="GOOD",
        warnings=(),
    )
    attempt = recording_attempt(widget, estimate)
    row = widget._record_frame_timestamp_parameters(attempt)
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


def test_time_tab_keeps_only_stus_sync_and_image_tab_takes_capture_and_exif() -> None:
    from PyQt5 import QtWidgets

    from wiris_stereo_distance.gui import WirisStereoDistancePanel

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = WirisStereoDistancePanel()
    time_tab = widget.tabs.widget(widget.time_tab_index)
    image_tab = widget.tabs.widget(widget.image_tab_index)

    assert widget.tabs.tabText(widget.image_tab_index) == "이미지"
    # STUS sync + its visualization stay on 시간.
    for widget_name in ("wiris_set_pc_time_button", "clock_sync_status_label", "clock_offset_plot"):
        assert time_tab.isAncestorOf(getattr(widget, widget_name)), widget_name
    # Capture / FTP / EXIF move to 이미지.
    for widget_name in (
        "clock_cptr_button",
        "clock_workflow_button",
        "clock_download_latest_button",
        "clock_analyze_pair_button",
        "clock_result_table",
        "clock_summary_text",
    ):
        assert image_tab.isAncestorOf(getattr(widget, widget_name)), widget_name
        assert not time_tab.isAncestorOf(getattr(widget, widget_name)), widget_name

    widget.deleteLater()
    _ = app


def test_clock_offset_samples_are_recorded_regardless_of_visible_tab() -> None:
    from PyQt5 import QtWidgets

    from wiris_stereo_distance.gui import WirisStereoDistancePanel

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = WirisStereoDistancePanel()
    widget.tabs.setCurrentIndex(widget.depth_tab_index)
    widget.camera_frame_metadata["left"] = {
        "rtcp_frame_receive_delay_s": 1.25,
        "rtcp_sr_age_s": 42.0,
    }
    widget.camera_frame_metadata["right"] = {
        "rtcp_frame_receive_delay_s": 0.12,
        "rtcp_sr_age_s": 1.5,
    }

    widget._record_clock_offset_sample(100.0)

    assert len(widget.clock_offset_plot.points) == 1
    point = widget.clock_offset_plot.points[-1]
    assert point.left_transit_delay_ms == 1250.0
    assert point.right_transit_delay_ms == 120.0
    assert point.left_sr_age_s == 42.0

    # Rate limited, so a sample right behind the previous one is dropped.
    widget._record_clock_offset_sample(100.1)
    assert len(widget.clock_offset_plot.points) == 1
    widget._record_clock_offset_sample(101.0)
    assert len(widget.clock_offset_plot.points) == 2

    widget.deleteLater()
    _ = app


def test_clock_offset_history_resets_when_streams_restart(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget.left_url_edit.setText("rtsp://left-camera/live")
    widget.right_url_edit.setText("rtsp://right-camera/live")
    monkeypatch.setattr(widget.left_pane, "start_stream", lambda *args, **kwargs: None)
    monkeypatch.setattr(widget.right_pane, "start_stream", lambda *args, **kwargs: None)

    widget.camera_frame_metadata["left"] = {"rtcp_frame_receive_delay_s": 12.0}
    # 1 s apart, so none are dropped by the sampler's rate limit.
    for timestamp_s in (1.0, 2.0, 3.0, 4.0):
        widget._record_clock_offset_sample(timestamp_s)
    assert len(widget.clock_offset_plot.points) == 4

    widget._start_camera_streams(enable_detection=False, switch_to_depth=False)

    # A restart re-anchors the Sender Report, so pre-restart samples describe a
    # different series and must not hold the y axis at the old scale.
    assert len(widget.clock_offset_plot.points) == 0

    widget.deleteLater()
    _ = app


def test_clock_offset_plot_time_axis_ticks_are_seconds_ago() -> None:
    from PyQt5 import QtWidgets

    from wiris_stereo_distance.measurement_widgets import ClockOffsetHistoryPlot, ClockOffsetPoint

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    plot = ClockOffsetHistoryPlot()
    plot.resize(640, 260)

    assert plot.time_axis_ticks() == []

    for index in range(41):
        plot.add_point(
            ClockOffsetPoint(timestamp_s=index * 1.0, left_transit_delay_ms=1000.0 + index)
        )

    # 40 s of samples, newest at 0 s ago.
    assert plot.time_axis_ticks() == [-40.0, -30.0, -20.0, -10.0, 0.0]
    assert plot.time_axis_ticks(tick_count=2) == [-40.0, -20.0, 0.0]

    plot.grab()

    plot.deleteLater()
    _ = app


def test_clock_offset_plot_spaces_points_by_time_not_sample_index() -> None:
    from PyQt5 import QtGui, QtWidgets

    from wiris_stereo_distance.measurement_widgets import ClockOffsetHistoryPlot, ClockOffsetPoint

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    evenly_spaced = ClockOffsetHistoryPlot()
    with_gap = ClockOffsetHistoryPlot()
    for plot in (evenly_spaced, with_gap):
        plot.resize(640, 260)

    values = [1000.0, 1100.0, 1200.0, 1300.0]
    for index, value in enumerate(values):
        evenly_spaced.add_point(ClockOffsetPoint(timestamp_s=index * 1.0, left_transit_delay_ms=value))
    # Same samples, but a stalled stream between the 2nd and 3rd.
    for timestamp_s, value in zip((0.0, 1.0, 31.0, 32.0), values):
        with_gap.add_point(ClockOffsetPoint(timestamp_s=timestamp_s, left_transit_delay_ms=value))

    assert evenly_spaced.grab().toImage() != with_gap.grab().toImage()

    for plot in (evenly_spaced, with_gap):
        plot.deleteLater()
    _ = app


def test_clock_offset_plot_ignores_missing_rtcp_fields() -> None:
    from PyQt5 import QtWidgets

    from wiris_stereo_distance.gui import WirisStereoDistancePanel

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = WirisStereoDistancePanel()
    widget.camera_frame_metadata["left"] = {"timestamp_s": 1.0}
    widget.camera_frame_metadata["right"] = {}

    widget._record_clock_offset_sample(100.0)

    point = widget.clock_offset_plot.points[-1]
    assert point.left_transit_delay_ms is None
    assert point.right_transit_delay_ms is None
    assert point.left_sr_age_s is None
    widget.clock_offset_plot.resize(400, 200)
    widget.clock_offset_plot.grab()  # must not raise with all-None series

    widget.deleteLater()
    _ = app


def test_clock_sync_display_only_auto_refreshes_while_time_tab_is_visible() -> None:
    from PyQt5 import QtGui, QtWidgets

    from wiris_stereo_distance.gui import WirisStereoDistancePanel

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = WirisStereoDistancePanel()
    image = QtGui.QImage(16, 9, QtGui.QImage.Format_RGB888)

    widget.tabs.setCurrentIndex(widget.depth_tab_index)
    assert widget._is_clock_sync_tab_visible() is False
    placeholder_text = widget.clock_sync_status_label.text()

    frame_metadata = {
        "timestamp_s": 100.0,
        "rtcp_frame_ntp_unix_s": 1_700_000_000.0,
        "timestamp_reference": "rtcp-ntp",
    }
    widget._update_calibration_preview("left", image, dict(frame_metadata))
    widget._update_calibration_preview("right", image, dict(frame_metadata))
    assert widget.clock_sync_status_label.text() == placeholder_text

    widget.tabs.setCurrentIndex(widget.time_tab_index)
    assert widget._is_clock_sync_tab_visible() is True
    widget._update_calibration_preview("left", image, dict(frame_metadata))
    widget._update_calibration_preview("right", image, dict(frame_metadata))
    assert widget.clock_sync_status_label.text() != placeholder_text
    assert "스트림 RTCP/NTP 상태" in widget.clock_sync_status_label.text()

    widget.deleteLater()
    _ = app


def test_runtime_camera_summary_tooltip_shows_per_camera_ntp_pc_utc_and_deltas() -> None:
    from PyQt5 import QtWidgets

    from wiris_stereo_distance.gui import WirisStereoDistancePanel

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = WirisStereoDistancePanel()
    widget.camera_frame_metadata["left"] = {
        "rtcp_frame_ntp_unix_s": 1_700_000_000.100,
        "rtcp_frame_received_wall_s": 1_700_000_000.350,
        "rtcp_frame_receive_delay_s": 0.250,
    }
    widget.camera_frame_metadata["right"] = {
        "rtcp_frame_ntp_unix_s": 1_700_000_000.130,
        "rtcp_frame_received_wall_s": 1_700_000_000.360,
        "rtcp_frame_receive_delay_s": 0.230,
    }

    _summary, tooltip = widget._format_runtime_camera_summary()

    assert "L NTP: 22:13:20.100Z | PC UTC: 22:13:20.350Z | Δt(PC-NTP): +250.0 ms" in tooltip
    assert "R NTP: 22:13:20.130Z | PC UTC: 22:13:20.360Z | Δt(PC-NTP): +230.0 ms" in tooltip
    assert "NTP Δt(R-L): +30.0 ms" in tooltip

    widget.deleteLater()
    _ = app


def test_runtime_camera_summary_tooltip_handles_missing_rtcp_data() -> None:
    from PyQt5 import QtWidgets

    from wiris_stereo_distance.gui import WirisStereoDistancePanel

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = WirisStereoDistancePanel()

    _summary, tooltip = widget._format_runtime_camera_summary()

    assert "L NTP: -- | PC UTC: -- | Δt(PC-NTP): --" in tooltip
    assert "R NTP: -- | PC UTC: -- | Δt(PC-NTP): --" in tooltip
    assert "NTP Δt(R-L): --" in tooltip

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
    assert "RTCP/NTP" in tooltip
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


def test_triangulation_3d_scene_points_places_cameras_symmetrically_on_right_axis() -> None:
    from sasl_common.stereo_geometry import BASELINE_CENTER_FRD_FRAME, RayTriangulationResult
    from wiris_stereo_distance.measurement_widgets import triangulation_3d_scene_points

    result = RayTriangulationResult(
        point_left_rdf_m=(0.1, 0.2, 5.0),
        range_m=5.0,
        closest_gap_m=0.01,
        left_depth_m=5.0,
        right_depth_m=5.1,
        ray_angle_deg=1.2,
        baseline_m=0.3,
        output_frame=BASELINE_CENTER_FRD_FRAME,
        output_point_m=(5.0, 0.05, 0.1),
        output_point_on_left_ray_m=(4.98, 0.0, 0.1),
        output_point_on_right_ray_m=(5.02, 0.1, 0.1),
        output_midpoint_m=(5.0, 0.05, 0.1),
    )

    scene = triangulation_3d_scene_points(result, baseline_m=0.3)

    assert scene is not None
    assert scene["cam_l"] == (0.0, -0.15, 0.0)
    assert scene["cam_r"] == (0.0, 0.15, 0.0)
    assert scene["point_on_left_ray"] == (4.98, 0.0, 0.1)
    assert scene["point_on_right_ray"] == (5.02, 0.1, 0.1)
    assert scene["detected_point"] == (5.0, 0.05, 0.1)


def test_triangulation_3d_scene_points_returns_none_outside_baseline_center_frd() -> None:
    from sasl_common.stereo_geometry import LEFT_CAMERA_RDF_FRAME, RayTriangulationResult
    from wiris_stereo_distance.measurement_widgets import triangulation_3d_scene_points

    result = RayTriangulationResult(
        point_left_rdf_m=(0.1, 0.2, 5.0),
        range_m=5.0,
        closest_gap_m=0.01,
        left_depth_m=5.0,
        right_depth_m=5.1,
        ray_angle_deg=1.2,
        baseline_m=0.3,
        output_frame=LEFT_CAMERA_RDF_FRAME,
    )

    assert triangulation_3d_scene_points(result, baseline_m=0.3) is None
    assert triangulation_3d_scene_points(None, baseline_m=0.3) is None


def test_triangulation_3d_view_falls_back_to_placeholder_without_gl(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.measurement_widgets as measurement_widgets_module

    monkeypatch.setattr(measurement_widgets_module.Triangulation3DView, "_build_gl_view", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = measurement_widgets_module.Triangulation3DView()
    widget.show()
    app.processEvents()

    assert widget._gl_view is None
    widget.set_quality(
        {
            "closest_gap_m": 0.01,
            "distance_m": 5.0,
            "method": "test",
            "scene": {"cam_l": (0.0, -0.15, 0.0)},
        }
    )
    assert widget._placeholder_label.isVisible()
    widget.set_quality(None)
    assert widget._placeholder_label.isVisible()
    widget.deleteLater()
    _ = app


def test_distance_history_plot_tracks_frd_and_range_panels(monkeypatch) -> None:
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
        forward_m=1.0,
        right_m=5.2,
        down_m=-0.4,
        sample_key=(1.0, 2.0, "triangulated-pixhawk"),
    )
    plot.add_point(point)
    app.processEvents()

    assert set(plot._plot_rects()) == {"forward_m", "right_m", "down_m", "distance_m"}
    assert plot._plot_metrics("forward_m") is not None
    assert plot._plot_metrics("right_m") is not None
    assert plot._plot_metrics("down_m") is not None
    distance_metrics = plot._plot_metrics("distance_m")
    assert distance_metrics is not None
    assert distance_metrics[3] < 5.4 < distance_metrics[4]

    plot._selected_point_keys.add(plot._point_key(point))
    selected_text = plot._selected_point_text() or ""
    assert "baseline_center_frd Forward/Right/Down: +1.00, +5.20, -0.40 m" in selected_text
    assert plot._panel_series("distance_m", "Range", "m", "") == (
        ("distance_m", "Range", "m", ""),
    )

    plot.deleteLater()
    _ = app


def test_distance_history_plot_does_not_fill_panels_with_previous_point_brush(monkeypatch) -> None:
    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")

    from PyQt5 import QtGui, QtWidgets

    from wiris_stereo_distance.measurement_widgets import DistanceHistoryPoint, DistanceHistoryPlot
    from wiris_stereo_distance.theme import CHART_BG, SUCCESS

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    plot = DistanceHistoryPlot()
    plot.resize(640, 300)
    plot.show()
    for index, value in enumerate((5.4, 5.8, 6.1)):
        plot.add_point(
            DistanceHistoryPoint(
                10.0 + index,
                value,
                "GOOD",
                "triangulated-pixhawk",
                forward_m=1.0 + index * 0.1,
                right_m=2.0 + index * 0.1,
                down_m=4.0 + index * 0.2,
            )
        )
    app.processEvents()

    image = plot.grab().toImage()
    down_rect = plot._plot_rects()["down_m"]
    background_pixel = image.pixelColor(down_rect.left() + 20, down_rect.top() + 30)

    assert background_pixel == QtGui.QColor(CHART_BG)
    assert background_pixel != QtGui.QColor(SUCCESS)

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

    attempt = recording_attempt(widget, estimate, baseline_m=0.3)
    widget._append_distance_history(
        estimate=estimate,
        distance_m=5.4,
        method="stereo",
        baseline_source="manual",
        baseline_m=0.3,
        measurement_attempt=attempt,
    )

    assert len(widget.distance_history) == 1
    assert len(widget.distance_recording_rows) == 1
    assert len(widget.distance_recording_keys) == 1
    assert float(widget.distance_recording_rows[0]["recording_elapsed_s"]) >= 2.0
    assert widget.distance_history[-1].recording_elapsed_s is not None
    measurements = recording_sheet(widget, "DistanceMeasurements")
    assert measurements.rowCount() == 1
    assert sheet_cell(measurements, 0, "distance_m") == "5.4"

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
    assert recording_sheet(widget, "DistanceMeasurements").rowCount() == 0
    assert widget.distance_plot._selected_point_text() is None
    assert widget.recording_status_label.text() == "세션 기록: 진행 중, 매칭 프레임쌍=0"

    widget.deleteLater()
    _ = app


def test_recording_sheet_tabs_mirror_the_session_workbook_sheets() -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui
    from wiris_stereo_distance.recording_session import session_sheet_previews

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()

    tab_names = [
        widget.recording_sheet_tabs.tabText(index)
        for index in range(widget.recording_sheet_tabs.count())
    ]

    assert tab_names == [preview.name for preview in session_sheet_previews({}, [])]
    assert tab_names == [
        "Summary",
        "DistanceMeasurements",
        "CameraLeft",
        "CameraRight",
        "Px4Left",
        "Px4Right",
        "CalibrationStatic",
    ]
    assert not hasattr(widget, "recording_sample_table")

    widget.deleteLater()
    _ = app


def test_recording_sheet_columns_follow_the_recorded_rows() -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget.distance_recording_rows = [
        {
            "sample_index": 0,
            "session_elapsed_s": 0.0,
            "distance_m": 5.0,
            "gps_latitude_deg": 37.1,
            "baseline_length_m": 0.3,
        }
    ]

    measurements = recording_sheet(widget, "DistanceMeasurements")
    assert sheet_cell(measurements, 0, "distance_m") == "5.0"
    assert sheet_cell(measurements, 0, "baseline_m") == "0.3"
    with pytest.raises(AssertionError):
        sheet_cell(measurements, 0, "gps_latitude_deg")

    summary = recording_sheet(widget, "Summary")
    assert [summary.horizontalHeaderItem(c).text() for c in range(summary.columnCount())] == [
        "key",
        "value",
    ]
    assert summary.rowCount() > 0

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

    attempt = recording_attempt(widget, estimate, baseline_m=0.25)
    assert widget._append_distance_history(
        estimate=estimate,
        distance_m=6.0,
        method="stereo",
        baseline_source="manual",
        baseline_m=0.25,
        measurement_attempt=attempt,
    )

    measurements = recording_sheet(widget, "DistanceMeasurements")
    assert measurements.rowCount() == 1
    assert sheet_cell(measurements, 0, "distance_m") == "6.0"
    assert sheet_cell(measurements, 0, "quality") == "GOOD"
    with pytest.raises(AssertionError):
        sheet_cell(measurements, 0, "relative_yaw_deg")
    measurements = recording_sheet(widget, "DistanceMeasurements")
    measurements.selectRow(0)
    assert "distance_m: 6.000000" in widget.recording_sample_detail.toPlainText()
    assert "warnings: low baseline confidence" in widget.recording_sample_detail.toPlainText()

    widget.clear_distance_history()

    assert recording_sheet(widget, "DistanceMeasurements").rowCount() == 0
    assert widget.recording_sample_detail.toPlainText() == ""

    widget.deleteLater()
    _ = app


def test_matched_pair_row_is_upserted_when_geometry_becomes_valid(monkeypatch) -> None:
    from types import SimpleNamespace

    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget.recording_session_id = "paired-session"
    widget.recording_session_started_monotonic_ns = 2_000_000_000
    widget.recording_session_started_monotonic_s = 2.0
    widget.recording_session_started_unix_ns = 1_800_000_000_000_000_000
    widget.recording_session_started_wall_s = 1_800_000_000.0
    widget.distance_recording_enabled = True

    estimate = SimpleNamespace(
        left=detection("left", x1=100, y1=200, x2=140, y2=260, t=3.0),
        right=detection("right", x1=90, y1=201, x2=130, y2=261, t=3.05),
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
    attempt = recording_attempt(widget, estimate, baseline_m=0.3)

    widget._record_unresolved_pair(
        attempt,
        status="matched_triangulation_pending",
        warnings=("waiting for geometry",),
    )
    assert len(widget.distance_recording_rows) == 1
    assert widget.distance_recording_rows[0]["pair_status"] == "matched_triangulation_pending"
    sample_index = widget.distance_recording_rows[0]["sample_index"]

    assert widget._append_distance_history(
        estimate=estimate,
        distance_m=6.0,
        method="triangulated-pixhawk",
        baseline_source="gps-rtk",
        baseline_m=0.3,
        point_m=(6.0, 0.0, 0.0),
        raw_point_m=(6.0, 0.0, 0.0),
        measurement_attempt=attempt,
    )

    assert len(widget.distance_recording_rows) == 1
    assert len(widget.distance_recording_keys) == 1
    row = widget.distance_recording_rows[0]
    assert row["sample_index"] == sample_index
    assert row["pair_status"] == "measured"
    assert row["measurement_valid"] == "true"
    assert row["session_elapsed_s"] == pytest.approx(1.025)

    second_estimate = SimpleNamespace(
        **{
            **vars(estimate),
            "left": detection("left", x1=101, y1=200, x2=141, y2=260, t=3.0),
            "right": detection("right", x1=91, y1=201, x2=131, y2=261, t=3.05),
        }
    )
    second_attempt = recording_attempt(widget, second_estimate, baseline_m=0.3)
    assert widget._append_distance_history(
        estimate=second_estimate,
        distance_m=6.1,
        method="triangulated-pixhawk",
        baseline_source="gps-rtk",
        baseline_m=0.3,
        point_m=(6.1, 0.0, 0.0),
        raw_point_m=(6.1, 0.0, 0.0),
        measurement_attempt=second_attempt,
    )
    assert len(widget.distance_recording_rows) == 2
    assert [item["sample_index"] for item in widget.distance_recording_rows] == [0, 1]

    widget.deleteLater()
    _ = app


def test_stop_session_writes_pair_workbook_and_device_timeline_csvs(tmp_path, monkeypatch) -> None:
    import json
    import time
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    session_id = "20260724_120000"
    monkeypatch.setattr(
        widget,
        "_recording_session_dir",
        lambda stamp: tmp_path / "sessions" / stamp,
    )
    widget.recording_session_id = session_id
    widget.recording_session_started_unix_ns = 1_800_000_000_000_000_000
    widget.recording_session_started_monotonic_ns = 2_000_000_000
    widget.recording_session_started_wall_s = 1_800_000_000.0
    widget.recording_session_started_monotonic_s = 2.0
    widget.distance_recording_enabled = True
    widget.video_recording_enabled = True
    now_wall_s = time.time()
    widget.camera_frame_metadata["left"] = {
        "timestamp_s": time.monotonic(),
        "rtcp_frame_ntp_unix_s": now_wall_s,
    }
    widget.camera_frame_metadata["right"] = {
        "timestamp_s": time.monotonic(),
        "rtcp_frame_ntp_unix_s": now_wall_s + 0.020,
    }
    widget._capture_recording_clock_sync_start_observation()
    widget.camera_frame_metadata["right"]["rtcp_frame_ntp_unix_s"] = now_wall_s + 0.120
    widget.camera_recording_rows["left"].append(
        {
            "sequence": 0,
            "rtcp_frame_ntp_unix_s": now_wall_s,
            "rtcp_frame_receive_delay_s": 0.055,
            "rtcp_frame_transport_delay_s": 0.012,
            "rtcp_clock_rate_hz": 90_000,
            "rtcp_camera_ntp_rate": 0.999,
            "rtcp_camera_media_rate": 0.979,
            "rtp_timestamp": 90_000,
        }
    )
    widget.distance_recording_rows = [
        {"sample_index": 0, "session_elapsed_s": 0.0, "pair_status": "matched"}
    ]

    widget._stop_session_recording()

    deadline = time.monotonic() + 5.0
    while widget._session_save_thread is not None and time.monotonic() < deadline:
        app.processEvents()
        time.sleep(0.01)

    output_dir = tmp_path / "sessions" / session_id
    assert widget._session_save_thread is None
    assert widget.distance_recording_enabled is False
    assert widget.video_recording_enabled is False
    assert (output_dir / f"{session_id}_distance_measurements.csv").is_file()
    assert (output_dir / f"{session_id}.xlsx").is_file()
    static_metadata = json.loads(
        (output_dir / f"{session_id}_session_static.json").read_text(encoding="utf-8")
    )
    session_metadata = static_metadata["metadata"]
    assert session_metadata["camera_time_sync_observations"]["end"]["scope"] == (
        "last_observed_before_session_stop"
    )
    assert session_metadata["camera_time_sync_observations"]["end"]["metrics"]["pair_delta_s"] == pytest.approx(0.120)
    assert session_metadata["camera_time_sync_drift"]["status"] == "measured"
    left_timing = session_metadata["camera_timing_diagnostics"]["left"]
    assert left_timing["rtcp_valid_frame_count"] == 1
    assert left_timing["rtcp_receive_delay_ms"]["max"] == pytest.approx(55.0)
    assert left_timing["rtcp_transport_delay_ms"]["median"] == pytest.approx(12.0)
    assert left_timing["estimated_rtp_clock_rate_hz"]["median"] == pytest.approx(88_110.0)
    assert sorted(path.name for path in output_dir.iterdir()) == [
        ".generations",
        f"{session_id}.xlsx",
        f"{session_id}_camera_left.csv",
        f"{session_id}_camera_left.parquet",
        f"{session_id}_camera_right.csv",
        f"{session_id}_camera_right.parquet",
        f"{session_id}_distance_measurements.csv",
        f"{session_id}_distance_measurements.parquet",
        f"{session_id}_gnss_base.csv",
        f"{session_id}_gnss_base.parquet",
        f"{session_id}_px4_left.csv",
        f"{session_id}_px4_left.parquet",
        f"{session_id}_px4_left.xlsx",
        f"{session_id}_px4_right.csv",
        f"{session_id}_px4_right.parquet",
        f"{session_id}_px4_right.xlsx",
        f"{session_id}_session_rtk.xlsx",
        f"{session_id}_session_static.json",
        "published",
        "session_manifest.json",
    ]

    widget.deleteLater()
    _ = app


def test_stop_save_is_async_and_spooled_device_rows_are_not_lost(tmp_path, monkeypatch) -> None:
    import csv
    import time
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui
    from wiris_stereo_distance.recording_session import SpoolRows

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    original_save = gui.save_session_files

    def slow_save(*args, **kwargs):
        time.sleep(0.20)
        return original_save(*args, **kwargs)

    monkeypatch.setattr(gui, "save_session_files", slow_save)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    session_id = "async-spool-session"
    output_dir = tmp_path / "sessions" / session_id
    monkeypatch.setattr(widget, "_recording_session_dir", lambda stamp: tmp_path / "sessions" / stamp)
    widget.recording_session_id = session_id
    widget.recording_session_started_unix_ns = 1_800_000_000_000_000_000
    widget.recording_session_started_monotonic_ns = 2_000_000_000
    widget.recording_session_started_wall_s = 1_800_000_000.0
    widget.recording_session_started_monotonic_s = 2.0
    widget.distance_recording_enabled = True
    widget.camera_recording_rows["left"] = SpoolRows(preview_limit=8)
    widget.camera_recording_rows["left"].bind(output_dir / ".spool" / "camera_left.jsonl")
    for sequence in range(50):
        widget.camera_recording_rows["left"].append(
            {"sequence": sequence, "session_id": session_id, "side": "left"}
        )

    started = time.monotonic()
    widget._stop_session_recording()
    elapsed = time.monotonic() - started

    assert elapsed < 0.10
    assert widget.camera_recording_rows["left"].preview_count == 8
    widget.clear_distance_history()
    assert len(widget.camera_recording_rows["left"]) == 50
    assert "저장 중" in widget.recording_status_label.text()
    deadline = time.monotonic() + 5.0
    while widget._session_save_thread is not None and time.monotonic() < deadline:
        app.processEvents()
        time.sleep(0.01)
    assert widget._session_save_thread is None
    with (output_dir / f"{session_id}_camera_left.csv").open(newline="", encoding="utf-8") as handle:
        assert [int(row["sequence"]) for row in csv.DictReader(handle)] == list(range(50))
    assert not (output_dir / ".spool").exists()

    widget.deleteLater()
    _ = app


def test_panel_close_is_deferred_while_session_save_is_running(monkeypatch) -> None:
    from PyQt5 import QtCore, QtGui, QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    monkeypatch.setattr(widget, "_save_current_settings", lambda: None)
    monkeypatch.setattr(widget, "_stop_calibration_preview_worker", lambda *_args: None)
    monkeypatch.setattr(widget, "disconnect_gnss_base_serial", lambda: None)
    monkeypatch.setattr(widget, "stop_all", lambda *args, **kwargs: None)
    monkeypatch.setattr(widget, "_wait_for_session_save", lambda *_args: False)
    scheduled: list[tuple[int, object]] = []
    monkeypatch.setattr(
        QtCore.QTimer,
        "singleShot",
        lambda delay, callback: scheduled.append((delay, callback)),
    )
    event = QtGui.QCloseEvent()

    widget.closeEvent(event)

    assert event.isAccepted() is False
    assert scheduled and scheduled[0][0] == 250
    widget.deleteLater()
    _ = app


def test_pair_spool_and_record_table_stay_bounded_for_long_session(tmp_path, monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui
    from wiris_stereo_distance.recording_session import SpoolRows

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    rows = SpoolRows(preview_limit=gui.RECORDING_PREVIEW_LIMIT, supports_updates=True)
    rows.bind(tmp_path / "pair_rows.jsonl")
    widget.distance_recording_rows = rows
    for sample_index in range(1000):
        row = {
            "sample_index": sample_index,
            "session_elapsed_s": sample_index * 0.01,
            "distance_m": float(sample_index),
        }
        rows.append(row)
        widget._append_recording_table_row(row)

    assert len(rows) == 1000
    assert rows.preview_count == gui.RECORDING_PREVIEW_LIMIT
    measurements = recording_sheet(widget, "DistanceMeasurements")
    assert measurements.rowCount() == gui.RECORDING_PREVIEW_LIMIT
    assert int(sheet_cell(measurements, 0, "sample_index") or 0) >= 0
    rows.close()
    recovered = SpoolRows(preview_limit=gui.RECORDING_PREVIEW_LIMIT, supports_updates=True)
    recovered.open_existing(tmp_path / "pair_rows.jsonl")
    assert len(recovered) == 1000
    assert recovered[0]["sample_index"] == 0
    assert recovered[-1]["sample_index"] == 999
    widget.deleteLater()
    _ = app


def test_live_save_worker_finishes_before_panel_close_is_accepted(tmp_path, monkeypatch) -> None:
    import time
    from PyQt5 import QtGui, QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    original_save = gui.save_session_files

    def slow_save(*args, **kwargs):
        time.sleep(0.15)
        return original_save(*args, **kwargs)

    monkeypatch.setattr(gui, "save_session_files", slow_save)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    session_id = "close-live-save"
    monkeypatch.setattr(widget, "_recording_session_dir", lambda stamp: tmp_path / stamp)
    widget.recording_session_id = session_id
    widget.recording_session_started_unix_ns = 1_800_000_000_000_000_000
    widget.recording_session_started_monotonic_ns = 2_000_000_000
    widget.recording_session_started_wall_s = 1_800_000_000.0
    widget.recording_session_started_monotonic_s = 2.0
    widget.distance_recording_enabled = True
    widget.distance_recording_rows = [{"sample_index": 0, "session_elapsed_s": 0.0}]
    widget._stop_session_recording()
    monkeypatch.setattr(widget, "_save_current_settings", lambda: None)
    monkeypatch.setattr(widget, "_stop_calibration_preview_worker", lambda *_args: None)
    monkeypatch.setattr(widget, "disconnect_gnss_base_serial", lambda: None)
    monkeypatch.setattr(widget, "stop_all", lambda *args, **kwargs: None)
    event = QtGui.QCloseEvent()

    widget.closeEvent(event)

    assert event.isAccepted() is True
    assert (tmp_path / session_id / f"{session_id}_distance_measurements.csv").is_file()
    deadline = time.monotonic() + 2.0
    while widget._session_save_thread is not None and time.monotonic() < deadline:
        app.processEvents()
    assert widget._session_save_thread is None
    widget.deleteLater()
    _ = app


def test_startup_does_not_invoke_spool_recovery_or_intrinsics_auto_load(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    calls: list[str] = []
    monkeypatch.setattr(
        gui.WirisStereoDistancePanel,
        "_recover_retryable_recording_session",
        lambda self: calls.append("spool") or False,
    )
    monkeypatch.setattr(
        gui.WirisStereoDistancePanel,
        "_auto_load_intrinsics",
        lambda self: calls.append("intrinsics"),
    )
    monkeypatch.setattr(gui, "interface_addresses", lambda _name: "test-interface")
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()

    for _ in range(5):
        app.processEvents()

    assert calls == []
    assert widget.recording_session_id == ""
    assert widget.active_intrinsics is None
    widget.deleteLater()
    _ = app


def test_startup_ignores_retryable_spool_and_new_recording_preserves_it(tmp_path, monkeypatch) -> None:
    import json
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui
    from wiris_stereo_distance.recording_session import SpoolRows, write_recovery_metadata

    old_output = tmp_path / "recordings" / "sessions" / "old-retryable-session"
    rows = SpoolRows(preview_limit=2, supports_updates=True)
    rows.bind(old_output / ".spool" / "pair_rows.jsonl")
    rows.append({"sample_index": 0, "session_elapsed_s": 0.0})
    rows.close()
    metadata_path = write_recovery_metadata(
        old_output,
        {"session_id": "old-retryable-session", "session_started_unix_ns": 1},
    )
    (old_output / "session_manifest.json").write_text(
        json.dumps({"status": "failed", "retryable": True}), encoding="utf-8"
    )
    original_metadata = metadata_path.read_bytes()
    monkeypatch.setattr(gui, "APP_ROOT", tmp_path)
    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_auto_load_intrinsics", lambda self: None)
    monkeypatch.setattr(gui, "interface_addresses", lambda _name: "test-interface")
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    for _ in range(5):
        app.processEvents()

    assert widget.recording_session_id == ""
    assert widget._recovered_session_metadata is None
    assert sorted(path.name for path in (tmp_path / "recordings" / "sessions").iterdir()) == [
        "old-retryable-session"
    ]

    monkeypatch.setattr(widget, "_start_camera_streams", lambda **_kwargs: True)
    widget._start_session_recording()

    assert widget.recording_session_id != "old-retryable-session"
    assert (tmp_path / "recordings" / "sessions" / widget.recording_session_id / ".spool").is_dir()
    assert metadata_path.read_bytes() == original_metadata
    assert (old_output / ".spool").is_dir()
    widget.distance_recording_enabled = False
    widget.video_recording_enabled = False
    for row_store in (
        widget.distance_recording_rows,
        *widget.camera_recording_rows.values(),
        *widget.px4_recording_rows.values(),
        widget.gnss_base_recording_rows,
    ):
        if hasattr(row_store, "close"):
            row_store.close()
    widget.deleteLater()
    _ = app


def test_gui_discovers_and_loads_retryable_pair_spool_after_restart(tmp_path, monkeypatch) -> None:
    import json
    import time
    from zipfile import ZipFile
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui
    from wiris_stereo_distance.recording_session import SpoolRows, write_recovery_metadata

    output_dir = tmp_path / "recordings" / "sessions" / "restart-session"
    rows = SpoolRows(preview_limit=2, supports_updates=True)
    rows.bind(output_dir / ".spool" / "pair_rows.jsonl")
    for sample_index in range(6):
        rows.append({"sample_index": sample_index, "session_elapsed_s": sample_index * 0.1})
    rows.close()
    write_recovery_metadata(
        output_dir,
        {
            "session_id": "restart-session",
            "session_started_unix_ns": 1_800_000_000_000_000_000,
            "session_started_monotonic_ns": 2_000_000_000,
            "provenance_marker": "ORIGINAL_CAPTURE_MARKER",
            "stream": {"left_rtsp_url": "rtsp://original-camera/live"},
            "left_video_path": "/capture/original/raw_left.mp4",
            "right_video_path": "/capture/original/raw_right.mp4",
        },
    )
    (output_dir / "session_manifest.json").write_text(
        json.dumps({"status": "failed", "retryable": True}), encoding="utf-8"
    )
    monkeypatch.setattr(gui, "APP_ROOT", tmp_path)
    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    original_save = gui.save_session_files
    captured_metadata: dict[str, object] = {}

    def capture_save(*args, **kwargs):
        captured_metadata.update(kwargs["metadata"])
        return original_save(*args, **kwargs)

    monkeypatch.setattr(gui, "save_session_files", capture_save)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()

    assert widget._recover_retryable_recording_session() is True
    assert widget.recording_session_id == "restart-session"
    assert len(widget.distance_recording_rows) == 6
    assert recording_sheet(widget, "DistanceMeasurements").rowCount() == 6
    assert widget.record_button.isEnabled() is True
    assert widget.save_history_button.isEnabled() is True
    widget.left_url_edit.setText("rtsp://changed-after-restart/live")
    widget.save_distance_history_csv()
    deadline = time.monotonic() + 5.0
    while widget._session_save_thread is not None and time.monotonic() < deadline:
        app.processEvents()
        time.sleep(0.01)
    assert widget._session_save_thread is None
    assert captured_metadata["provenance_marker"] == "ORIGINAL_CAPTURE_MARKER"
    assert captured_metadata["stream"] == {"left_rtsp_url": "rtsp://original-camera/live"}
    assert captured_metadata["left_video_path"] == "/capture/original/raw_left.mp4"
    assert captured_metadata["matched_frame_pair_count"] == 6
    assert captured_metadata["finalization"]["recovered_retry"] is True
    with ZipFile(output_dir / "restart-session.xlsx") as workbook:
        shared_strings = workbook.read("xl/sharedStrings.xml").decode("utf-8")
    assert "ORIGINAL_CAPTURE_MARKER" in shared_strings
    assert "rtsp://original-camera/live" in shared_strings
    assert "/capture/original/raw_left.mp4" in shared_strings
    assert "rtsp://changed-after-restart/live" not in shared_strings
    widget.deleteLater()
    _ = app


def test_gui_starts_new_recording_after_discarding_recovered_session(tmp_path, monkeypatch) -> None:
    import json
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui
    from wiris_stereo_distance.recording_session import SpoolRows, write_recovery_metadata

    output_dir = tmp_path / "recordings" / "sessions" / "discard-before-new-session"
    rows = SpoolRows(preview_limit=2, supports_updates=True)
    rows.bind(output_dir / ".spool" / "pair_rows.jsonl")
    rows.append({"sample_index": 0, "session_elapsed_s": 0.0})
    rows.close()
    write_recovery_metadata(
        output_dir,
        {"session_id": "discard-before-new-session", "session_started_unix_ns": 1},
    )
    (output_dir / "session_manifest.json").write_text(
        json.dumps({"status": "failed", "retryable": True}), encoding="utf-8"
    )
    monkeypatch.setattr(gui, "APP_ROOT", tmp_path)
    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()

    assert widget._recover_retryable_recording_session() is True
    assert widget.record_button.isEnabled() is True
    monkeypatch.setattr(widget, "_start_camera_streams", lambda **_kwargs: True)
    widget._start_session_recording()

    assert widget.recording_session_id != "discard-before-new-session"
    assert widget._session_recording_active() is True
    assert not (output_dir / ".spool").exists()
    manifest = json.loads((output_dir / "session_manifest.json").read_text(encoding="utf-8"))
    assert manifest["status"] == "discarded"
    assert manifest["discard_reason"] == "operator_started_new_session"
    for row_store in (
        widget.distance_recording_rows,
        *widget.camera_recording_rows.values(),
        *widget.px4_recording_rows.values(),
        widget.gnss_base_recording_rows,
    ):
        if hasattr(row_store, "close"):
            row_store.close()
    widget.deleteLater()
    _ = app


def test_session_start_recovery_metadata_includes_prepared_video_paths(tmp_path, monkeypatch) -> None:
    import json
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui, "APP_ROOT", tmp_path)
    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()

    monkeypatch.setattr(widget, "_start_camera_streams", lambda **_kwargs: True)
    widget._start_session_recording()

    output_dir = tmp_path / "recordings" / "sessions" / widget.recording_session_id
    metadata = json.loads(
        (output_dir / ".spool" / "session_metadata.json").read_text(encoding="utf-8")
    )
    assert metadata["left_video_path"].endswith("/video/raw_left.mp4")
    assert metadata["right_video_path"].endswith("/video/raw_right.mp4")
    assert metadata["left_video_path"] == str(widget.video_recording_paths["left"])
    assert widget.video_recording_paths["left"].parent.is_dir()
    assert widget.video_recording_paths["right"].parent.is_dir()
    widget.distance_recording_enabled = False
    for rows in (
        widget.distance_recording_rows,
        *widget.camera_recording_rows.values(),
        *widget.px4_recording_rows.values(),
    ):
        if hasattr(rows, "close"):
            rows.close()
    widget.deleteLater()
    _ = app


def test_session_start_without_preview_immediately_requests_recording_streams(
    tmp_path,
    monkeypatch,
) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui, "APP_ROOT", tmp_path)
    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget.camera_connected = False
    widget.yolo_enabled = False
    starts: list[tuple[bool, bool]] = []
    widget._start_camera_streams = lambda **kwargs: starts.append(
        (bool(kwargs["enable_detection"]), bool(kwargs["switch_to_depth"]))
    ) or True

    widget._start_session_recording()

    assert starts == [(False, False)]
    assert widget._session_recording_active() is True
    assert widget._raw_video_validation_required is True
    assert widget._raw_video_expected_cameras == ("left", "right")
    assert widget.video_recording_paths["left"].name == "raw_left.mp4"
    assert widget.video_recording_paths["right"].name == "raw_right.mp4"

    widget.distance_recording_enabled = False
    widget.video_recording_enabled = False
    for rows in (
        widget.distance_recording_rows,
        *widget.camera_recording_rows.values(),
        *widget.px4_recording_rows.values(),
        widget.gnss_base_recording_rows,
    ):
        if hasattr(rows, "close"):
            rows.close()
    widget.deleteLater()
    _ = app


def test_session_start_rolls_back_visible_recording_state_when_camera_start_rejects(
    tmp_path,
    monkeypatch,
) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui, "APP_ROOT", tmp_path)
    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget._start_camera_streams = lambda **_kwargs: False

    widget._start_session_recording()

    assert widget._session_recording_active() is False
    assert widget._session_save_retry_required is True
    assert widget.record_button.text() == "세션 기록 시작"
    assert "시작 실패" in widget.recording_status_label.text()

    for rows in (
        widget.distance_recording_rows,
        *widget.camera_recording_rows.values(),
        *widget.px4_recording_rows.values(),
        widget.gnss_base_recording_rows,
    ):
        if hasattr(rows, "close"):
            rows.close()
    widget.deleteLater()
    _ = app


def test_session_start_never_reads_px4_mag_params_on_gui_thread(tmp_path, monkeypatch) -> None:
    import json
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui, "APP_ROOT", tmp_path)
    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget._px4_mag_manager_has_data_source = lambda _side: True
    widget._sync_px4_mag_manager_store = lambda _side: True

    def unexpected_readback():
        raise AssertionError("session start attempted blocking MAG PARAM readback")

    widget.px4_mag_managers["left"].refresh = unexpected_readback
    widget.px4_mag_managers["right"].refresh = unexpected_readback

    monkeypatch.setattr(widget, "_start_camera_streams", lambda **_kwargs: True)
    widget._start_session_recording()

    metadata_path = (
        tmp_path
        / "recordings"
        / "sessions"
        / widget.recording_session_id
        / ".spool"
        / "session_metadata.json"
    )
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    assert metadata["mag_param_snapshot_status"] == {
        "left": {"status": "unavailable", "captured_monotonic_s": None},
        "right": {"status": "unavailable", "captured_monotonic_s": None},
    }

    widget.distance_recording_enabled = False
    widget.video_recording_enabled = False
    for rows in (
        widget.distance_recording_rows,
        *widget.camera_recording_rows.values(),
        *widget.px4_recording_rows.values(),
        widget.gnss_base_recording_rows,
    ):
        if hasattr(rows, "close"):
            rows.close()
    widget.deleteLater()
    _ = app


def test_decoder_fallback_ends_active_recording_before_software_restart(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    stopped = []
    widget.distance_recording_enabled = True
    widget.video_recording_enabled = True
    widget._stop_session_recording = lambda: stopped.append(True)

    widget._handle_runtime_decoder_fallback(
        "left",
        "DECODER_FALLBACK_REQUIRED: nvh264dec decoded-frame stall",
    )

    assert widget._runtime_decoder_override == "software"
    assert stopped == [True]
    widget.distance_recording_enabled = False
    widget.video_recording_enabled = False
    widget.deleteLater()
    _ = app


def test_same_process_save_retry_keeps_first_capture_metadata(tmp_path, monkeypatch) -> None:
    import time
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui
    import wiris_stereo_distance.recording_session as recording_session

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    monkeypatch.setattr(widget, "_recording_session_dir", lambda stamp: tmp_path / stamp)
    widget.recording_session_id = "same-process-retry"
    widget.recording_session_started_unix_ns = 1_800_000_000_000_000_000
    widget.recording_session_started_monotonic_ns = 2_000_000_000
    widget.recording_session_started_wall_s = 1_800_000_000.0
    widget.recording_session_started_monotonic_s = 2.0
    widget.distance_recording_rows = [{"sample_index": 0, "session_elapsed_s": 0.0}]
    widget.left_url_edit.setText("rtsp://first-capture/live")
    captured: list[dict[str, object]] = []
    original_save = gui.save_session_files

    def capture_save(*args, **kwargs):
        captured.append(kwargs["metadata"])
        return original_save(*args, **kwargs)

    monkeypatch.setattr(gui, "save_session_files", capture_save)
    original_xlsx = recording_session._write_xlsx
    monkeypatch.setattr(
        recording_session,
        "_write_xlsx",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(OSError("first XLSX failure")),
    )
    widget.save_distance_history_csv()
    deadline = time.monotonic() + 5.0
    while widget._session_save_thread is not None and time.monotonic() < deadline:
        app.processEvents()
        time.sleep(0.01)
    assert widget._session_save_retry_required is True

    widget.left_url_edit.setText("rtsp://changed-before-retry/live")
    monkeypatch.setattr(recording_session, "_write_xlsx", original_xlsx)
    widget.save_distance_history_csv()
    deadline = time.monotonic() + 5.0
    while widget._session_save_thread is not None and time.monotonic() < deadline:
        app.processEvents()
        time.sleep(0.01)

    assert len(captured) == 2
    assert captured[0]["stream"]["left_rtsp_url"] == "rtsp://first-capture/live"
    assert captured[1]["stream"]["left_rtsp_url"] == "rtsp://first-capture/live"
    assert captured[1]["finalization"]["recovered_retry"] is True
    widget.deleteLater()
    _ = app


def test_camera_timeline_records_only_frames_delivered_to_gui_during_session(monkeypatch) -> None:
    from PyQt5 import QtGui, QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget.recording_session_id = "gui-frame-session"
    widget.recording_session_started_monotonic_ns = 2_000_000_000
    widget.recording_session_started_monotonic_s = 2.0
    widget.recording_session_started_unix_ns = 1_800_000_000_000_000_000
    widget.recording_session_started_wall_s = 1_800_000_000.0
    widget.distance_recording_started_monotonic_s = 2.0
    widget.distance_recording_enabled = True
    image = QtGui.QImage(640, 480, QtGui.QImage.Format_RGB888)

    widget._update_calibration_preview(
        "left",
        image,
        {
            "timestamp_s": 1.9,
            "timestamp_source": "gstreamer-pts",
            "rtcp_frame_ntp_unix_s": 1_799_999_999.9,
        },
    )
    assert widget.camera_recording_rows["left"] == []

    widget._update_calibration_preview(
        "left",
        image,
        {
            "timestamp_s": 2.25,
            "timestamp_source": "gstreamer-pts",
            "timestamp_reference": "rtcp-ntp",
            "rtcp_frame_ntp_unix_s": 1_800_000_000.3,
        },
    )

    assert len(widget.camera_recording_rows["left"]) == 1
    row = widget.camera_recording_rows["left"][0]
    assert row["sample_type"] == "gui_camera_frame"
    assert row["frame_stage"] == "gui_delivered_preview"
    assert row["mp4_frame_index_contract"] == "not_available"
    assert row["sample_monotonic_ns"] == 2_250_000_000
    assert row["session_elapsed_s"] == "0.250000000"
    assert row["session_utc_ns"] == 1_800_000_000_250_000_000
    assert row["source_utc_ns"] == 1_800_000_000_300_000_000
    assert row["source_utc_method"] == "rtcp_ntp"
    assert row["image_width_px"] == 640
    assert row["image_height_px"] == 480
    assert widget.camera_recording_rows["right"] == []

    widget.distance_recording_enabled = False
    widget._update_calibration_preview("left", image, {"timestamp_s": 2.5})
    assert len(widget.camera_recording_rows["left"]) == 1

    widget.deleteLater()
    _ = app


def test_px4_timeline_keeps_monotonic_session_utc_and_source_gps_utc(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget.recording_session_id = "px4-session"
    widget.recording_session_started_monotonic_ns = 2_000_000_000
    widget.recording_session_started_monotonic_s = 2.0
    widget.recording_session_started_unix_ns = 1_800_000_000_000_000_000
    widget.recording_session_started_wall_s = 1_800_000_000.0
    widget.distance_recording_started_monotonic_s = 2.0
    widget.distance_recording_enabled = True

    widget._record_external_timeline_row(
        "right",
        "gps_rtk",
        {
            "sample_app_timestamp_s": "2.500000",
            "px4_timestamp_us": 12_000_000,
            "px4_timestamp_sample_us": 11_999_000,
            "gps_utc_us": 1_800_000_010_000_000,
            "time_source": "px4_ros2",
            "source_utc_method": "px4_vehicle_gps_position_time_utc_usec",
        },
    )

    assert widget.px4_recording_rows["left"] == []
    assert len(widget.px4_recording_rows["right"]) == 1
    row = widget.px4_recording_rows["right"][0]
    assert row["sample_monotonic_ns"] == 2_500_000_000
    assert row["session_elapsed_s"] == "0.500000000"
    assert row["session_utc_ns"] == 1_800_000_000_500_000_000
    assert row["source_utc_ns"] == 1_800_000_010_000_000_000
    assert row["source_utc_method"] == "px4_vehicle_gps_position_time_utc_usec"
    assert row["px4_timestamp_sample_us"] == 11_999_000

    widget.deleteLater()
    _ = app


def test_px4_timeline_records_mag_samples_and_slot_status(monkeypatch) -> None:
    from types import SimpleNamespace

    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui
    from wiris_stereo_distance.px4_mag_manager import RM3100_PRIMARY_ID

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget.recording_session_id = "px4-mag-session"
    widget.recording_session_started_monotonic_ns = 2_000_000_000
    widget.recording_session_started_monotonic_s = 2.0
    widget.recording_session_started_unix_ns = 1_800_000_000_000_000_000
    widget.recording_session_started_wall_s = 1_800_000_000.0
    widget.distance_recording_started_monotonic_s = 2.0
    widget.distance_recording_enabled = True
    monkeypatch.setattr(widget, "_sync_px4_mag_manager_store", lambda _camera: True)
    widget.latest_vehicle_mag_device_ids["left"] = RM3100_PRIMARY_ID
    widget.refresh_px4_mag_manager("left", read_params=True)

    widget._update_mag_sensor(
        "left",
        SimpleNamespace(
            namespace="uav_0",
            topic="/uav_0/fmu/out/vehicle_magnetometer",
            device_id=RM3100_PRIMARY_ID,
            sensor_slot="SELECTED",
            timestamp_s=2.5,
            x=0.1,
            y=0.2,
            z=0.3,
            magnitude=0.3741657387,
            temperature_c=23.5,
            error_count=4,
            px4_timestamp_s=12.0,
        ),
    )
    widget._update_mag_sensor(
        "left",
        SimpleNamespace(
            namespace="uav_0",
            topic="mavlink/SCALED_IMU",
            device_id=None,
            sensor_slot="CAN1",
            timestamp_s=2.6,
            x=100.0,
            y=200.0,
            z=300.0,
            magnitude=374.1657387,
            temperature_c=24.0,
            error_count=None,
            px4_timestamp_s=None,
        ),
    )

    rows = list(widget.px4_recording_rows["left"])
    samples = [row for row in rows if row["sample_type"] == "magnetometer"]
    statuses = [row for row in rows if row["sample_type"] == "mag_slot_status"]
    assert len(samples) == 2
    assert samples[0]["mag_x"] == "0.100000000"
    assert samples[0]["mag_y"] == "0.200000000"
    assert samples[0]["mag_z"] == "0.300000000"
    assert samples[0]["mag_vector_norm"] == "0.374165739"
    assert "RM3100" in samples[0]["mag_device_type"]
    assert samples[0]["mag_temperature_c"] == "23.500000"
    assert samples[0]["mag_error_count"] == 4
    assert samples[1]["mag_value_units"] == "milligauss"
    assert statuses
    assert any(row["mag_device_id"] == str(RM3100_PRIMARY_ID) for row in statuses)
    assert any("mag_priority" in row for row in statuses)

    widget.deleteLater()
    _ = app


def test_pair_row_records_session_start_attitude_gps_and_baseline_changes(monkeypatch) -> None:
    import math
    from types import SimpleNamespace

    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui
    from wiris_stereo_distance.measurement_attempt import AttitudeSnapshot, GpsSnapshot
    from wiris_stereo_distance.pixhawk_attitude import CameraExtrinsicState
    from wiris_stereo_distance.rtk_geometry import GpsPositionState, ned_delta_m

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    half_degree_rad = math.radians(0.5)
    initial_attitudes = {
        camera: CameraExtrinsicState(
            camera,
            f"uav_{index}",
            0.0,
            0.0,
            0.0,
            2.0,
            f"/{camera}/attitude",
            (1.0, 0.0, 0.0, 0.0),
        )
        for index, camera in enumerate(("left", "right"))
    }
    initial_gps = {
        "left": GpsPositionState(
            "left", "uav_0", 2.0, "/left/gps", 37.0, 127.0, 100.0, fix_type=6
        ),
        "right": GpsPositionState(
            "right", "uav_1", 2.0, "/right/gps", 37.0, 127.000001, 100.0, fix_type=6
        ),
    }
    widget.recording_reference_attitudes = {
        camera: AttitudeSnapshot.from_state(state)
        for camera, state in initial_attitudes.items()
    }
    widget.recording_reference_gps = {
        camera: GpsSnapshot.from_state(state) for camera, state in initial_gps.items()
    }
    widget.recording_origin_gps = widget.recording_reference_gps["left"]
    widget.recording_reference_baseline_ned_m = ned_delta_m(
        widget.recording_reference_gps["left"], widget.recording_reference_gps["right"]
    )
    widget.latest_extrinsics = {
        camera: CameraExtrinsicState(
            camera,
            f"uav_{index}",
            0.0,
            0.0,
            1.0,
            3.0,
            f"/{camera}/attitude",
            (math.cos(half_degree_rad), 0.0, 0.0, math.sin(half_degree_rad)),
        )
        for index, camera in enumerate(("left", "right"))
    }
    widget.latest_gps = {
        "left": GpsPositionState(
            "left", "uav_0", 3.0, "/left/gps", 37.0, 127.000001, 100.0, fix_type=6
        ),
        "right": GpsPositionState(
            "right", "uav_1", 3.0, "/right/gps", 37.0, 127.000003, 100.0, fix_type=6
        ),
    }
    estimate = SimpleNamespace(
        left=detection("left", x1=100, y1=200, x2=140, y2=260, t=3.0),
        right=detection("right", x1=90, y1=201, x2=130, y2=261, t=3.05),
        sync_quality="GOOD",
        warnings=[],
        left_attitude_age_s=0.0,
        right_attitude_age_s=0.0,
        attitude_pair_delta_s=0.0,
        relative_roll_deg=0.0,
        relative_pitch_deg=0.0,
        relative_yaw_deg=0.0,
    )
    attempt = recording_attempt(widget, estimate, baseline_m=0.3, translation=(0.3, 0.0, 0.0))

    row = widget._record_reference_delta_parameters(attempt)

    assert float(row["left_pixhawk_delta_rotation_angle_deg"]) == pytest.approx(1.0)
    assert float(row["right_pixhawk_delta_rotation_angle_deg"]) == pytest.approx(1.0)
    assert float(row["left_gps_antenna_delta_east_m"]) > 0.0
    assert float(row["baseline_reference_length_m"]) > 0.0
    assert float(row["baseline_length_change_m"]) > 0.0
    assert row["gps_position_frame"] == "session_left_start_gps_antenna_ned"
    assert row["attitude_orientation_contract"] == "px4_frd_to_ned; quaternion_wxyz"

    widget.deleteLater()
    _ = app


def test_raw_recording_saves_bbox_geometry_and_pixhawk_status_fields(monkeypatch) -> None:
    from types import SimpleNamespace

    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui
    from wiris_stereo_distance.pixhawk_attitude import CameraExtrinsicState
    from wiris_stereo_distance.rtk_geometry import GpsPositionState

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget.distance_mode_combo.setCurrentIndex(widget.distance_mode_combo.findData("pixhawk"))
    widget.distance_recording_enabled = True
    now_s = time.monotonic()
    widget.latest_extrinsics["left"] = CameraExtrinsicState(
        "left",
        "uav_0",
        roll_deg=1.0,
        pitch_deg=-2.0,
        yaw_deg=3.0,
        timestamp_s=now_s - 0.10,
        topic="/uav_0/fmu/out/vehicle_attitude",
        q=(1.0, 0.0, 0.0, 0.0),
        px4_timestamp_us=9_900_000,
        px4_timestamp_sample_us=9_899_500,
        delta_q_reset=(1.0, 0.0, 0.0, 0.0),
        quat_reset_counter=2,
    )
    widget.latest_extrinsics["right"] = CameraExtrinsicState(
        "right",
        "uav_1",
        roll_deg=4.0,
        pitch_deg=-5.0,
        yaw_deg=6.0,
        timestamp_s=now_s - 0.20,
        topic="/uav_1/fmu/out/vehicle_attitude",
        q=(1.0, 0.0, 0.0, 0.0),
        px4_timestamp_us=9_800_000,
        px4_timestamp_sample_us=9_799_500,
    )
    widget.latest_gps["left"] = GpsPositionState(
        "left",
        "uav_0",
        now_s - 0.30,
        "/uav_0/fmu/out/vehicle_gps_position",
        fix_type=6,
        satellites_used=18,
        eph_m=0.12,
        epv_m=0.34,
        px4_timestamp_us=9_700_000,
        px4_timestamp_sample_us=9_699_500,
        gps_utc_us=1_800_000_000_100_000,
        gps_timestamp_time_relative_us=-100,
        velocity_n_m_s=1.0,
        velocity_e_m_s=2.0,
        velocity_d_m_s=3.0,
        velocity_ned_valid=True,
    )
    widget.latest_gps["right"] = GpsPositionState(
        "right",
        "uav_1",
        now_s - 0.40,
        "/uav_1/fmu/out/vehicle_gps_position",
        fix_type=5,
        satellites_used=17,
        eph_m=0.22,
        epv_m=0.44,
        px4_timestamp_us=9_600_000,
        px4_timestamp_sample_us=9_599_500,
        gps_utc_us=1_800_000_000_200_000,
    )
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

    attempt = recording_attempt(widget, estimate, baseline_m=0.3, translation=(0.3, 0.0, 0.0))
    assert widget._append_distance_history(
        estimate=estimate,
        distance_m=9.25,
        method="triangulated-rtk",
        baseline_source="manual",
        baseline_m=0.3,
        point_m=(9.0, 2.0, 4.0),
        raw_point_m=(10.0, 2.5, 2.0),
        raw_point_left_m=(10.15, 2.5, 2.0),
        measurement_attempt=attempt,
    )

    row = widget.distance_recording_rows[-1]
    assert row["schema_version"] == "stereowiris-paired-frame-v2"
    assert row["measurement_frame"] == "baseline_center_frd"
    assert row["measurement_axis_contract"] == "baseline_center_frd:forward_right_down"
    assert row["public_vector_contract"] == "baseline_center_frd:[forward,right,down,range]"
    assert row["legacy_xyz_alias_contract"] == "not_emitted"
    assert row["baseline_frame_scope"] == "measurement_attempt"
    assert row["baseline_frame_instance_id"] == attempt.instance_sha256()
    assert row["baseline_frame_origin_model"] == "gps_rtk_antenna_baseline_proxy_no_lever_arm"
    assert row["measurement_method"] == "two_ray_closest_midpoint"
    assert row["baseline_center_forward_m"] == "9.000000"
    assert row["baseline_center_right_m"] == "2.000000"
    assert row["baseline_center_down_m"] == "4.000000"
    assert row["baseline_center_range_m"] == "9.250000"
    assert row["raw_baseline_center_forward_m"] == "10.000000"
    assert row["raw_baseline_center_right_m"] == "2.500000"
    assert row["raw_baseline_center_down_m"] == "2.000000"
    assert row["raw_baseline_center_range_m"] == "10.500000"
    assert row["source_left_camera_rdf_right_m"] == "10.150000"
    assert row["source_left_camera_rdf_down_m"] == "2.500000"
    assert row["source_left_camera_rdf_forward_m"] == "2.000000"
    assert "estimated_right_m" not in row
    assert "estimated_down_m" not in row
    assert "estimated_forward_m" not in row
    assert "estimated_range_m" not in row
    assert row["baseline_left_camera_rdf_right_m"] == "0.300000"
    assert "x_m" not in row
    assert "raw_x_m" not in row
    assert "left_camera_right_m" not in row
    assert "stereo_center_right_m" not in row
    assert "corrected_z_m" not in row
    assert "filtered_distance_m" not in row
    assert "tracking_filter_id" not in row
    assert "live_truth_range_m" not in row
    assert "telemetry_range_m" not in row
    assert "drone_north_m" not in row
    assert "error_x_m" not in row
    assert "bias_profile_id" not in row
    assert row["world_frame"] == "ned"
    assert row["body_frame"] == "px4_frd"
    assert row["camera_frame"] == "camera_optical_rdf"
    assert row["attitude_frame"] == "px4_frd"
    assert row["attitude_units"] == "deg"
    assert row["left_roll_deg"] == "1.000000"
    assert row["left_pitch_deg"] == "-2.000000"
    assert row["left_yaw_deg"] == "3.000000"
    assert row["right_roll_deg"] == "4.000000"
    assert row["right_pitch_deg"] == "-5.000000"
    assert row["right_yaw_deg"] == "6.000000"
    assert row["left_gps_satellites_used"] == "18"
    assert row["left_gps_eph_m"] == "0.120000"
    assert row["left_gps_epv_m"] == "0.340000"
    assert row["right_gps_satellites_used"] == "17"
    assert row["right_gps_eph_m"] == "0.220000"
    assert row["right_gps_epv_m"] == "0.440000"
    assert row["left_gps_fix_type"] == "6"
    assert row["right_gps_fix_type"] == "5"
    assert row["left_pixhawk_px4_timestamp_us"] == 9_900_000
    assert row["left_pixhawk_px4_timestamp_sample_us"] == 9_899_500
    assert row["left_pixhawk_quat_reset_counter"] == 2
    assert row["left_gps_px4_timestamp_us"] == 9_700_000
    assert row["left_gps_px4_timestamp_sample_us"] == 9_699_500
    assert row["left_gps_gps_utc_us"] == 1_800_000_000_100_000
    assert row["left_gps_velocity_n_m_s"] == "1.000000"
    assert row["left_attitude_alignment"] == "nearest_app_receive_monotonic"
    assert row["left_gps_alignment"] == "latest_selected_app_receive_monotonic"
    assert row["camera_mount_profile"] == "front_camera_z_to_body_x_x_to_body_y"
    assert row["mount_assumption_model"] == "gps-rtk-baseline-only-v1"
    assert row["mount_assumption_status"] == "GPS/RTK baseline only"
    assert row["mount_assumption_detail"] == "live_gps_rtk_baseline_used_as_stereo_translation"
    assert "manual_baseline_setting_m" not in row
    assert (
        row["left_bbox_x1_px"],
        row["left_bbox_y1_px"],
        row["left_bbox_x2_px"],
        row["left_bbox_y2_px"],
    ) == ("100.000", "200.000", "140.000", "260.000")
    assert (
        row["right_bbox_x1_px"],
        row["right_bbox_y1_px"],
        row["right_bbox_x2_px"],
        row["right_bbox_y2_px"],
    ) == ("90.000", "201.000", "130.000", "261.000")
    assert row["left_bbox_center_x_px"] == "120.000"
    assert row["right_bbox_confidence"] == "0.900000"
    assert row["bbox_mid_timestamp_s"] == "3.025000"
    assert row["bbox_mid_utc_unix_s"] == "1800000000.025000"
    assert row["left_rtcp_frame_ntp_unix_ns"] == 1_800_000_000_000_000_000
    assert row["right_rtcp_frame_ntp_unix_ns"] == 1_800_000_000_050_000_000
    assert row["pair_mid_utc_ns"] == 1_800_000_000_025_000_000
    assert "drone_timestamp_s" not in row

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

    attempt = recording_attempt(widget, estimate, baseline_m=0.3)
    assert widget._append_distance_history(
        estimate=estimate,
        distance_m=5.4,
        method="stereo",
        baseline_source="manual",
        baseline_m=0.3,
        measurement_attempt=attempt,
    )
    assert not widget._append_distance_history(
        estimate=estimate,
        distance_m=5.4,
        method="stereo",
        baseline_source="manual",
        baseline_m=0.3,
        measurement_attempt=attempt,
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
        f"RTCP NTP {expected_utc}"
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
    assert " | live: UTC " in widget.left_runtime_stats_label.text()
    assert widget.right_runtime_stats_label.text().startswith("오른쪽: ")
    assert " | live: RTCP " in widget.right_runtime_stats_label.text()
    assert widget.runtime_pair_status_label.text() == "Pair: 대기 | pair: missing"

    widget.deleteLater()
    _ = app


def test_runtime_preview_frames_do_not_replace_measurement_pair_display(monkeypatch) -> None:
    from PyQt5 import QtGui, QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    left_image = QtGui.QImage(16, 9, QtGui.QImage.Format_RGB888)
    right_image = QtGui.QImage(16, 9, QtGui.QImage.Format_RGB888)
    left_image.fill(QtGui.QColor("red"))
    right_image.fill(QtGui.QColor("blue"))
    now_s = time.monotonic()
    widget._update_calibration_preview(
        "left",
        left_image,
        {
            "timestamp_s": now_s,
            "source_timestamp_s": 1.0,
            "timestamp_source": "gstreamer-pts",
            "rtcp_frame_ntp_unix_s": 1_700_000_000.000,
        },
    )
    widget._update_calibration_preview(
        "right",
        right_image,
        {
            "timestamp_s": now_s,
            "source_timestamp_s": 2.0,
            "timestamp_source": "gstreamer-pts",
            "rtcp_frame_ntp_unix_s": 1_700_000_000.020,
        },
    )

    assert widget.selected_runtime_frame_pair_delta_s is None
    assert not widget.left_pane.measurement_pair_display_active
    assert not widget.right_pane.measurement_pair_display_active

    widget.deleteLater()
    _ = app


def test_runtime_preview_frame_clears_stale_measurement_pair_timestamp(monkeypatch) -> None:
    from PyQt5 import QtGui, QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget.yolo_enabled = False
    widget.selected_runtime_frame_metadata["left"] = {
        "timestamp_s": time.monotonic() - 1.0,
        "rtcp_frame_ntp_unix_s": 1_700_000_000.000,
    }
    widget.selected_runtime_frame_metadata["right"] = {
        "timestamp_s": time.monotonic() - 1.0,
        "rtcp_frame_ntp_unix_s": 1_700_000_000.010,
    }
    widget.selected_runtime_frame_pair_delta_s = 0.010
    widget.left_pane.measurement_pair_display_active = True
    widget.right_pane.measurement_pair_display_active = True
    preview_image = QtGui.QImage(16, 9, QtGui.QImage.Format_RGB888)
    preview_image.fill(QtGui.QColor("blue"))

    widget._update_calibration_preview(
        "left",
        preview_image,
        {
            "timestamp_s": time.monotonic(),
            "rtcp_frame_ntp_unix_s": 1_700_000_001.000,
        },
    )

    assert widget.selected_runtime_frame_metadata["left"] == {}
    assert widget.selected_runtime_frame_metadata["right"] == {}
    assert widget.selected_runtime_frame_pair_delta_s is None
    assert not widget.left_pane.measurement_pair_display_active
    assert not widget.right_pane.measurement_pair_display_active
    assert "선택Δ=" not in widget.left_runtime_stats_label.text()
    assert "live: RTCP 22:13:21.000Z" in widget.left_runtime_stats_label.text()
    assert "live 전체 시각: RTCP NTP 2023-11-14T22:13:21.000000Z" in widget.left_runtime_stats_label.toolTip()

    widget.deleteLater()
    _ = app


def test_measurement_pair_display_uses_detection_frames_and_timestamps(monkeypatch) -> None:
    from PyQt5 import QtGui, QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    now_s = time.monotonic()
    left_detection = detection(
        "left",
        x1=500,
        y1=200,
        x2=540,
        y2=260,
        t=now_s,
        timestamp_metadata={"timestamp_s": now_s, "rtcp_frame_ntp_unix_s": 1_700_000_000.000},
    )
    right_detection = detection(
        "right",
        x1=450,
        y1=202,
        x2=490,
        y2=262,
        t=now_s + 0.010,
        timestamp_metadata={"timestamp_s": now_s + 0.010, "rtcp_frame_ntp_unix_s": 1_700_000_000.020},
    )
    left_image = QtGui.QImage(16, 9, QtGui.QImage.Format_RGB888)
    right_image = QtGui.QImage(16, 9, QtGui.QImage.Format_RGB888)
    left_image.fill(QtGui.QColor("green"))
    right_image.fill(QtGui.QColor("blue"))
    widget.left_pane._store_measurement_frame(now_s, left_image)
    widget.right_pane._store_measurement_frame(now_s + 0.010, right_image)
    widget.camera_frame_metadata["left"] = {"timestamp_s": now_s + 1.0, "rtcp_frame_ntp_unix_s": 1_700_000_001.000}
    widget.camera_frame_metadata["right"] = {"timestamp_s": now_s + 1.0, "rtcp_frame_ntp_unix_s": 1_700_000_001.020}
    widget.yolo_gate_matching_checkbox.setChecked(True)
    widget.yolo_gate_time_basis_combo.setCurrentIndex(widget.yolo_gate_time_basis_combo.findData("camera_rtcp"))

    widget._show_measurement_pair(left_detection, right_detection)

    assert abs(widget.selected_runtime_frame_pair_delta_s - 0.020) < 1e-6
    assert widget.selected_runtime_frame_metadata["left"]["rtcp_frame_ntp_unix_s"] == 1_700_000_000.000
    assert widget.selected_runtime_frame_metadata["right"]["rtcp_frame_ntp_unix_s"] == 1_700_000_000.020
    assert "선택Δ" not in widget.left_runtime_stats_label.text()
    assert "RTCΔ=" not in widget.left_runtime_stats_label.text()
    assert "화면Δ=" not in widget.left_runtime_stats_label.text()
    assert "age=" not in widget.left_runtime_stats_label.text()
    assert "live: RTCP 22:13:21.000Z" in widget.left_runtime_stats_label.text()
    assert "pair L RTCP 22:13:20.000Z" in widget.runtime_pair_status_label.text()
    assert "R RTCP 22:13:20.020Z" in widget.runtime_pair_status_label.text()
    assert "Δ 20.0 ms" in widget.runtime_pair_status_label.text()
    assert "live 전체 시각: RTCP NTP 2023-11-14T22:13:21.000000Z" in widget.left_runtime_stats_label.toolTip()
    assert "left pair 전체 시각: RTCP NTP 2023-11-14T22:13:20.000000Z" in widget.runtime_pair_status_label.toolTip()
    assert QtGui.QColor(widget.left_pane.current_image().pixel(0, 0)).name() == QtGui.QColor("green").name()
    assert QtGui.QColor(widget.right_pane.current_image().pixel(0, 0)).name() == QtGui.QColor("blue").name()

    widget.deleteLater()
    _ = app


def test_measurement_pair_display_shows_app_fallback_sync_basis(monkeypatch) -> None:
    from PyQt5 import QtGui, QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    now_s = time.monotonic()
    left_detection = detection(
        "left",
        x1=500,
        y1=200,
        x2=540,
        y2=260,
        t=now_s,
        timestamp_metadata={"timestamp_s": now_s},
    )
    right_detection = detection(
        "right",
        x1=450,
        y1=202,
        x2=490,
        y2=262,
        t=now_s + 0.008,
        timestamp_metadata={"timestamp_s": now_s + 0.008},
    )
    image = QtGui.QImage(16, 9, QtGui.QImage.Format_RGB888)
    image.fill(QtGui.QColor("green"))
    widget.left_pane._store_measurement_frame(now_s, image)
    widget.right_pane._store_measurement_frame(now_s + 0.008, image)
    widget.yolo_gate_matching_checkbox.setChecked(True)
    widget.yolo_gate_time_basis_combo.setCurrentIndex(widget.yolo_gate_time_basis_combo.findData("camera_rtcp"))

    widget._show_measurement_pair(left_detection, right_detection)

    assert widget.selected_runtime_frame_pair_basis == "app fallback"
    assert widget.selected_runtime_frame_pair_camera_rtc_delta_s is None
    assert "선택Δ" not in widget.left_runtime_stats_label.text()
    assert "RTCΔ=" not in widget.left_runtime_stats_label.text()

    widget.deleteLater()
    _ = app


def test_measurement_pair_display_reports_frame_lookup_delta(monkeypatch) -> None:
    from PyQt5 import QtGui, QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    now_s = time.monotonic()
    image = QtGui.QImage(16, 9, QtGui.QImage.Format_RGB888)
    image.fill(QtGui.QColor("green"))
    left_detection = detection(
        "left",
        x1=500,
        y1=200,
        x2=540,
        y2=260,
        t=now_s,
        timestamp_metadata={"timestamp_s": now_s, "rtcp_frame_ntp_unix_s": 1_700_000_000.000},
    )
    right_detection = detection(
        "right",
        x1=450,
        y1=202,
        x2=490,
        y2=262,
        t=now_s + 0.010,
        timestamp_metadata={"timestamp_s": now_s + 0.010, "rtcp_frame_ntp_unix_s": 1_700_000_000.010},
    )
    widget.left_pane._store_measurement_frame(now_s + 0.0005, image)
    widget.right_pane._store_measurement_frame(now_s + 0.0105, image)

    widget._show_measurement_pair(left_detection, right_detection)

    assert widget.left_pane.selected_measurement_frame_lookup_delta_s == pytest.approx(0.0005)
    assert widget.right_pane.selected_measurement_frame_lookup_delta_s == pytest.approx(0.0005)
    assert "화면Δ=" not in widget.left_runtime_stats_label.text()

    widget.deleteLater()
    _ = app


def test_measurement_pair_display_clears_when_no_rtc_pair_matches(monkeypatch) -> None:
    from PyQt5 import QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    widget.yolo_gate_matching_checkbox.setChecked(True)
    widget.yolo_gate_time_delta_checkbox.setChecked(True)
    widget.yolo_gate_time_delta_spin.setValue(0.05)
    widget.yolo_gate_time_basis_combo.setCurrentIndex(widget.yolo_gate_time_basis_combo.findData("camera_rtc_required"))
    widget.selected_runtime_frame_metadata["left"] = {
        "timestamp_s": time.monotonic() - 10.0,
        "rtcp_frame_ntp_unix_s": 1_700_000_000.000,
    }
    widget.selected_runtime_frame_metadata["right"] = {
        "timestamp_s": time.monotonic() - 10.0,
        "rtcp_frame_ntp_unix_s": 1_700_000_000.010,
    }
    widget.selected_runtime_frame_pair_delta_s = 0.010
    widget.selected_runtime_frame_pair_basis = "Camera RTC required"
    widget.left_pane.measurement_pair_display_active = True
    widget.right_pane.measurement_pair_display_active = True
    widget.detection_buffers["left"].add(
        detection(
            "left",
            x1=690,
            y1=340,
            x2=710,
            y2=380,
            t=time.monotonic(),
            timestamp_metadata={"timestamp_s": time.monotonic(), "rtcp_frame_ntp_unix_s": 1_800_000_000.000},
        )
    )
    widget.detection_buffers["right"].add(
        detection(
            "right",
            x1=640,
            y1=340,
            x2=660,
            y2=380,
            t=time.monotonic(),
            timestamp_metadata={"timestamp_s": time.monotonic(), "rtcp_frame_ntp_unix_s": 1_800_000_000.250},
        )
    )

    widget._refresh_distance()

    assert widget.selected_runtime_frame_pair_delta_s is None
    assert widget.selected_runtime_frame_metadata["left"] == {}
    assert widget.selected_runtime_frame_metadata["right"] == {}
    assert not widget.left_pane.measurement_pair_display_active
    assert not widget.right_pane.measurement_pair_display_active
    assert "선택Δ=" not in widget.left_runtime_stats_label.text()
    assert "timestamp가 맞는 좌/우 검출 대기 중" in widget.distance_label.text()

    widget.deleteLater()
    _ = app


def test_runtime_synced_preview_does_not_overwrite_recent_detection_pair(monkeypatch) -> None:
    from PyQt5 import QtGui, QtWidgets

    import wiris_stereo_distance.gui as gui

    monkeypatch.setattr(gui.WirisStereoDistancePanel, "_initial_status", lambda self: None)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    widget = gui.WirisStereoDistancePanel()
    now_s = time.monotonic()
    left_detection = detection(
        "left",
        x1=500,
        y1=200,
        x2=540,
        y2=260,
        t=now_s,
        timestamp_metadata={"rtcp_frame_ntp_unix_s": 1_700_000_000.000},
    )
    right_detection = detection(
        "right",
        x1=450,
        y1=202,
        x2=490,
        y2=262,
        t=now_s + 0.010,
        timestamp_metadata={"rtcp_frame_ntp_unix_s": 1_700_000_000.010},
    )
    widget.detection_buffers["left"].add(left_detection)
    widget.detection_buffers["right"].add(right_detection)
    detection_image = QtGui.QImage(16, 9, QtGui.QImage.Format_RGB888)
    detection_image.fill(QtGui.QColor("green"))
    preview_image = QtGui.QImage(16, 9, QtGui.QImage.Format_RGB888)
    preview_image.fill(QtGui.QColor("blue"))
    widget.left_pane.show_selected_measurement_frame(detection_image)
    widget.right_pane.show_selected_measurement_frame(detection_image)

    widget._update_calibration_preview(
        "left",
        preview_image,
        {"timestamp_s": now_s + 0.100, "rtcp_frame_ntp_unix_s": 1_700_000_000.100},
    )
    widget._update_calibration_preview(
        "right",
        preview_image,
        {"timestamp_s": now_s + 0.110, "rtcp_frame_ntp_unix_s": 1_700_000_000.110},
    )

    assert QtGui.QColor(widget.left_pane.current_image().pixel(0, 0)).name() == QtGui.QColor("green").name()
    assert QtGui.QColor(widget.right_pane.current_image().pixel(0, 0)).name() == QtGui.QColor("green").name()

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
