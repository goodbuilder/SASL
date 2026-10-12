"""Compatibility and independent frame/clock oracles for the Pixhawk leaves."""

from __future__ import annotations

import base64
from dataclasses import FrozenInstanceError, asdict, fields
import json
import math
from pathlib import Path
import pickle
import subprocess
import sys
import textwrap
from types import SimpleNamespace

import pytest

from Workstation.workstation_core import pixhawk_attitude as legacy
from Workstation.workstation_core.pixhawk_link import decode, timebase, types


APPS_ROOT = Path(__file__).resolve().parents[2]
FIXTURE = Path(__file__).with_name("fixtures") / "pixhawk_link_legacy_types.json"
LEGACY_SAMPLES = json.loads(FIXTURE.read_text())["samples"]


def _fresh_interpreter(script: str, *, stdin: str = "") -> subprocess.CompletedProcess:
    return subprocess.run(
        [sys.executable, "-I", "-B", "-c", textwrap.dedent(script), str(APPS_ROOT)],
        input=stdin,
        text=True,
        capture_output=True,
        timeout=30,
        check=True,
    )


@pytest.mark.parametrize("sample", LEGACY_SAMPLES, ids=lambda row: row["class_name"])
def test_moved_values_preserve_legacy_class_schema_and_pickle(sample: dict) -> None:
    cls = getattr(types, sample["class_name"])
    assert getattr(legacy, sample["class_name"]) is cls
    assert cls.__module__ == "Workstation.workstation_core.pixhawk_attitude"
    assert [field.name for field in fields(cls)] == sample["field_names"]
    payload = base64.b64decode(sample["pickle_protocol4_base64"])
    restored = pickle.loads(payload)
    assert type(restored) is cls
    assert json.loads(json.dumps(asdict(restored))) == sample["expected_values"]
    assert pickle.dumps(restored, protocol=4) == payload
    with pytest.raises(FrozenInstanceError):
        setattr(restored, sample["field_names"][0], None)


def test_legacy_pickles_load_in_a_fresh_interpreter() -> None:
    result = _fresh_interpreter(
        """
        import base64, json, pickle, sys
        from pathlib import Path
        apps = Path(sys.argv[1])
        sys.path[:0] = [str(apps), str(apps.parent / 'common/python')]
        from Workstation.workstation_core.pixhawk_link import types
        for sample in json.load(sys.stdin):
            value = pickle.loads(base64.b64decode(sample['pickle_protocol4_base64']))
            assert type(value) is getattr(types, sample['class_name'])
        print('legacy values restored')
        """,
        stdin=json.dumps(LEGACY_SAMPLES),
    )
    assert result.stdout.strip() == "legacy values restored"


def test_data_decode_and_clock_leaves_import_without_qt_or_device_runtime() -> None:
    result = _fresh_interpreter(
        """
        import importlib.abc, sys
        from pathlib import Path
        apps = Path(sys.argv[1])
        sys.path[:0] = [str(apps), str(apps.parent / 'common/python')]
        forbidden = ('PyQt5', 'pymavlink', 'Workstation.workstation_core.pixhawk_attitude')
        class DenyRuntime(importlib.abc.MetaPathFinder):
            def find_spec(self, fullname, path=None, target=None):
                if any(fullname == p or fullname.startswith(p + '.') for p in forbidden):
                    raise AssertionError('data leaf imported runtime: ' + fullname)
                return None
        sys.meta_path.insert(0, DenyRuntime())
        from Workstation.workstation_core.pixhawk_link import types, decode, timebase, transport
        from Workstation.workstation_core.alignment_managers import MagnetometerMonitor
        monitor = MagnetometerMonitor()
        monitor.update('left', types.MagSensorState(
            camera='left', namespace='fixture', sensor_slot='CAN1', timestamp_s=1.0,
            topic='mavlink/SCALED_IMU', magnitude=500.0))
        assert monitor.field_strength('left', 'CAN1') == 500.0
        assert decode.quaternion_to_euler_deg((1, 0, 0, 0)).yaw_deg == 0.0
        assert timebase._MavlinkBootTimeTracker().observe(10, received_at_s=1.0).reboot is None
        noop = lambda *args, **kwargs: None
        callbacks = transport.TransportCallbacks(
            status=noop, message=noop, reset_sources=noop, reset_stream_requests=noop,
            advance_stream_policy=noop, apply_pending_setup=noop,
            start_timer=noop, stop_timer=noop)
        link = transport.MavlinkTransport(left_url='', right_url='', callbacks=callbacks)
        assert link.resource_inventory() == {}
        assert not any(p in sys.modules for p in forbidden)
        print('headless leaves usable')
        """
    )
    assert result.stdout.strip() == "headless leaves usable"


@pytest.mark.parametrize(
    ("quaternion", "expected_deg"),
    [
        ((1.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
        ((math.sqrt(0.5), math.sqrt(0.5), 0.0, 0.0), (90.0, 0.0, 0.0)),
        ((math.cos(math.pi / 8), 0.0, -math.sin(math.pi / 8), 0.0), (0.0, -45.0, 0.0)),
        ((math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5)), (0.0, 0.0, 90.0)),
    ],
)
def test_wxyz_body_frd_to_ned_axes_have_independent_known_euler_values(quaternion: tuple, expected_deg: tuple) -> None:
    attitude = decode.quaternion_to_euler_deg(quaternion)
    assert (attitude.roll_deg, attitude.pitch_deg, attitude.yaw_deg) == pytest.approx(expected_deg)


def test_pitch_singularity_preserves_orientation_without_requiring_unique_roll_yaw() -> None:
    attitude = decode.quaternion_to_euler_deg((math.sqrt(0.5), 0.0, -math.sqrt(0.5), 0.0))
    assert attitude.pitch_deg == pytest.approx(-90.0)
    # At gimbal lock, (roll, yaw) are not individually identifiable. Check
    # the resulting body basis in NED using the independent Rz * Ry * Rx
    # definition, rather than forcing a particular Euler representation.
    r, p, y = (math.radians(value) for value in (attitude.roll_deg, attitude.pitch_deg, attitude.yaw_deg))
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    body_forward_ned = (cy * cp, sy * cp, -sp)
    body_right_ned = (cy * sp * sr - sy * cr, sy * sp * sr + cy * cr, cp * sr)
    body_down_ned = (cy * sp * cr + sy * sr, sy * sp * cr - cy * sr, cp * cr)
    assert body_forward_ned == pytest.approx((0.0, 0.0, 1.0))
    assert body_right_ned == pytest.approx((0.0, 1.0, 0.0))
    assert body_down_ned == pytest.approx((-1.0, 0.0, 0.0))


def test_attitude_factory_retains_radians_degrees_and_receipt_clock() -> None:
    state = decode.camera_extrinsic_from_euler_rad(
        camera="left",
        namespace=" rig/P1 ",
        roll_rad=0.0,
        pitch_rad=0.0,
        yaw_rad=math.pi / 2,
        timestamp_s=17.25,
    )
    assert state.q == pytest.approx((math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5)))
    assert (state.roll_deg, state.pitch_deg, state.yaw_deg) == pytest.approx((0.0, 0.0, 90.0))
    assert state.namespace == "rig/P1"
    assert state.timestamp_s == 17.25
    assert state.px4_timestamp_s is None
    assert isinstance(state, legacy.CameraExtrinsicState)


def test_local_position_timeline_keeps_ned_alias_order_and_provenance() -> None:
    message = SimpleNamespace(
        x=11.0,
        y=-23.0,
        z=37.0,
        vx=-0.5,
        vy=1.25,
        vz=-2.75,
        time_boot_ms=1234,
        srcSystem=1,
        srcComponent=2,
        seq=77,
        msgid=32,
    )
    row = decode._mavlink_optional_timeline_row("LOCAL_POSITION_NED", message, 17.25)
    assert row == {
        "sample_type": "mavlink_local_position_ned",
        "topic": "mavlink/LOCAL_POSITION_NED",
        "sample_app_timestamp_s": 17.25,
        "mavlink_message_name": "LOCAL_POSITION_NED",
        "mavlink_source_system": 1,
        "mavlink_source_component": 2,
        "mavlink_sequence": 77,
        "mavlink_message_id": 32,
        "mavlink_local_x_m": 11.0,
        "mavlink_local_y_m": -23.0,
        "mavlink_local_z_m": 37.0,
        "mavlink_local_velocity_x_m_s": -0.5,
        "mavlink_local_velocity_y_m_s": 1.25,
        "mavlink_local_velocity_z_m_s": -2.75,
        "mavlink_time_boot_ms": 1234,
    }


@pytest.mark.parametrize("coords_type", [0, 1])
def test_rtk_baseline_heading_is_ned_only_and_length_is_metres(coords_type: int) -> None:
    message = SimpleNamespace(
        baseline_a_mm=3000,
        baseline_b_mm=4000,
        baseline_c_mm=12000,
        baseline_coords_type=coords_type,
        accuracy=25,
        rtk_receiver_id=2,
    )
    state = decode._gnss_relative_state_from_mavlink_rtk("right", "rig/P2", message, "mavlink/GPS2_RTK", 17.25)
    assert state.position_length_m == pytest.approx(13.0)
    assert state.accuracy_length_m == pytest.approx(0.025)
    if coords_type == 1:
        assert state.heading_rad == pytest.approx(0.9272952180016122)
        assert state.heading_valid is True
    else:
        assert state.heading_rad is None
        assert state.heading_valid is False


def test_gps_quality_scales_and_timestamp_sources_stay_distinct() -> None:
    message = SimpleNamespace(
        fix_type=6,
        satellites_visible=27,
        h_acc=120,
        v_acc=340,
        eph=56,
        epv=78,
        yaw=9000,
        hdg_acc=125000,
        time_usec=1_700_000_000_123_456,
    )
    gps = decode._gps_sensor_state_from_mavlink("left", "rig/P1", message, "mavlink/GPS_RAW_INT", 17.25)
    heading = decode._gnss_relative_state_from_mavlink_gps_raw("left", "rig/P1", message, "mavlink/GPS_RAW_INT", 17.25)
    assert (gps.eph_m, gps.epv_m, gps.hdop, gps.vdop) == pytest.approx((0.12, 0.34, 0.56, 0.78))
    assert gps.gps_utc_us == 1_700_000_000_123_456
    assert gps.timestamp_s == 17.25
    assert gps.px4_timestamp_s is None
    assert heading.heading_rad == pytest.approx(math.pi / 2)
    assert heading.heading_accuracy_rad == pytest.approx(math.pi / 144)
    message.time_usec = 123456
    assert (
        decode._gps_sensor_state_from_mavlink("left", "rig/P1", message, "mavlink/GPS_RAW_INT", 17.25).gps_utc_us
        is None
    )


@pytest.mark.parametrize(("topic", "scale"), [("SCALED_IMU", 1000.0), ("SASL_CAN_MAG", 1.0)])
def test_raw_mag_axes_and_topic_units_survive_separation(topic: str, scale: float) -> None:
    message = SimpleNamespace(xmag=0.3 * scale, ymag=-0.4 * scale, zmag=0.0, can_port=1)
    decoder = (
        decode._mag_sensor_state_from_mavlink if topic == "SCALED_IMU" else decode._can_mag_sensor_state_from_mavlink
    )
    state = decoder("left", "rig/P1", message, topic, 17.25)
    assert (state.x, state.y, state.z) == pytest.approx((0.3 * scale, -0.4 * scale, 0.0))
    assert decode.magnetometer_value_to_gauss(state.magnitude, state.topic) == pytest.approx(0.5)


def test_nonfinite_source_reason_is_preserved_instead_of_becoming_valid_zero() -> None:
    message = SimpleNamespace(x=11.0, y=float("nan"), z=float("inf"), time_boot_ms=1234)
    row = decode._mavlink_optional_timeline_row("LOCAL_POSITION_NED", message, 17.25)
    assert row["mavlink_local_y_m"] is None
    assert row["mavlink_local_z_m"] is None
    assert row["nonfinite_source_reasons"] == {
        "LOCAL_POSITION_NED.y": "nan",
        "LOCAL_POSITION_NED.z": "positive_infinity",
    }
    assert (
        decode._mag_sensor_state_from_mavlink(
            "left", "rig/P1", SimpleNamespace(xmag=1.0, ymag=float("nan"), zmag=2.0), "SCALED_IMU", 17.25
        )
        is None
    )


def test_boot_tracker_handles_wrap_reordering_and_reboot_without_a_clock() -> None:
    tracker = timebase._MavlinkBootTimeTracker()
    for boot_ms, received_at_s in [((1 << 32) - 16, 10.0), (25, 10.05), (58, 10.1), (40, 10.15)]:
        result = tracker.observe(boot_ms, received_at_s=received_at_s)
        assert result.reboot is None
        assert result.accept_for_downstream
    tracker.reset()
    assert tracker.observe(100000, received_at_s=20.0).reboot is None
    assert tracker.observe(100, received_at_s=20.05).reboot is None
    reboot = tracker.observe(133, received_at_s=20.1).reboot
    assert (reboot.previous_time_boot_ms, reboot.current_time_boot_ms) == (100000, 133)
    assert not tracker.observe(100033, received_at_s=20.15).accept_for_downstream
    assert tracker.observe(166, received_at_s=20.2).accept_for_downstream
    assert legacy._MavlinkBootTimeTracker is timebase._MavlinkBootTimeTracker


def _transport_callbacks(**overrides):
    from Workstation.workstation_core.pixhawk_link.transport import TransportCallbacks

    callbacks = {field.name: (lambda *args, **kwargs: None) for field in fields(TransportCallbacks)}
    callbacks.update(overrides)
    return TransportCallbacks(**callbacks)


def test_transport_rejects_missing_mandatory_callbacks() -> None:
    with pytest.raises(TypeError, match="Missing mandatory transport callback: message"):
        _transport_callbacks(message=None)


def test_subscriber_compatibility_fields_share_one_real_resource_owner() -> None:
    subscriber = legacy.DualPixhawkMavlinkSubscriber(left_url="left", right_url="right")
    link = subscriber._transport
    assert subscriber.connections is link.connections
    assert subscriber._command_state_lock is link._command_state_lock
    assert subscriber._mavlink_send_locks is link._mavlink_send_locks
    assert subscriber._deferred_command_messages is link._deferred_command_messages
    assert subscriber.timer.interval() == 20
    assert "connections" not in vars(subscriber)
    assert "_command_state_lock" not in vars(subscriber)
    events = []
    connection = SimpleNamespace(close=lambda: events.append("closed"))
    subscriber.connections = {"left": connection}
    assert link.connections["left"] is connection
    assert subscriber.resource_inventory() == {"pixhawk.left.connection": connection}
    try:
        subscriber._mavlink_stream_requests_sent.add("left")
    finally:
        subscriber.stop("left")
    assert events == ["closed"]
    assert not subscriber._mavlink_stream_requests_sent
    assert subscriber.resource_inventory() == {}


def test_transport_command_failure_releases_reservation_without_losing_receive_time() -> None:
    from Workstation.workstation_core.pixhawk_link import transport

    received = []
    link = transport.MavlinkTransport(
        left_url="left",
        right_url="",
        callbacks=_transport_callbacks(message=lambda *args, **kwargs: received.append((args, kwargs))),
    )
    polls = []
    connection = SimpleNamespace(recv_match=lambda **kwargs: polls.append(kwargs), close=lambda: None)
    link.connections["left"] = connection
    packet = SimpleNamespace(get_type=lambda: "ATTITUDE", roll=0.0, pitch=0.0, yaw=0.0)
    try:
        with pytest.raises(RuntimeError, match="command failed"):
            with link.exclusive_command("left", hold_send_lock=False):
                link._deferred_command_messages["left"].append((17.25, packet))
                link.spin_once()
                assert polls == []
                assert received[0][1] == {"received_at_s": 17.25, "allow_heartbeat_actions": False}
                raise RuntimeError("command failed")
        assert not link._active_command_cameras
        with link.exclusive_command("left", hold_send_lock=False):
            assert link._active_command_cameras == {"left"}
    finally:
        link.stop()
    assert not link.connections


def test_transport_partial_connect_closes_only_the_failed_endpoint(monkeypatch) -> None:
    from pymavlink import mavutil
    from Workstation.workstation_core.pixhawk_link import endpoints, transport

    events = []
    connections = {}

    def connect(url, **kwargs):
        handle = SimpleNamespace(close=lambda: events.append(("closed", url)))
        connections[url] = handle
        return handle

    def harden(handle, url):
        if url == "right":
            raise OSError("right endpoint setup failed")

    monkeypatch.setattr(mavutil, "mavlink_connection", connect)
    monkeypatch.setattr(endpoints, "_assert_mavlink_udpin_available", lambda url: None)
    monkeypatch.setattr(endpoints, "_disable_mavlink_udpin_reuse", harden)
    monkeypatch.setattr(endpoints, "_seed_mavlink_udpin_peer", lambda *args: None)
    link = transport.MavlinkTransport(
        left_url="left",
        right_url="right",
        callbacks=_transport_callbacks(
            start_timer=lambda: events.append("timer-start"),
            reset_sources=lambda side: events.append(("reset", side)),
        ),
    )
    try:
        link.start()
        assert link.connections == {"left": connections["left"]}
        assert "right endpoint setup failed" in link.last_connect_errors["right"]
        assert events == [("closed", "right"), "timer-start"]
    finally:
        link.stop()
    assert events.count(("closed", "left")) == 1
    assert events.count(("closed", "right")) == 1
    assert ("reset", "left") in events
    assert link.resource_inventory() == {}
