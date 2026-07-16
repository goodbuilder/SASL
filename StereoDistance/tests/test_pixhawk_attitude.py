from __future__ import annotations

from types import SimpleNamespace

from wiris_stereo_distance.pixhawk_attitude import DualPixhawkAttitudeSubscriber, _send_mavlink_shell_command


def test_ros2_mag_slots_use_discovery_order_without_known_ids() -> None:
    subscriber = DualPixhawkAttitudeSubscriber.__new__(DualPixhawkAttitudeSubscriber)
    subscriber.mag_device_slots = {"left": {}, "right": {}}

    assert subscriber._mag_slot_for_device("left", 0x00431021) == "MAG1"
    assert subscriber._mag_slot_for_device("left", 0x00431029) == "MAG2"
    assert subscriber._mag_slot_for_device("left", 0x00431021) == "MAG1"
    assert subscriber._mag_slot_for_device("right", 0x00431021) == "MAG1"


def test_mavlink_shell_command_falls_back_to_respond_without_exclusive() -> None:
    class FakeMavlink:
        SERIAL_CONTROL_DEV_SHELL = 10
        SERIAL_CONTROL_FLAG_RESPOND = 2
        SERIAL_CONTROL_FLAG_EXCLUSIVE = 4

    class FakeMav:
        def __init__(self) -> None:
            self.calls = []
            self.current_flags = 0

        def serial_control_send(self, device, flags, timeout, baudrate, count, data) -> None:
            self.calls.append((device, flags, count, bytes(data[:count])))
            self.current_flags = flags

    class FakeMessage:
        count = 8
        data = list(b"nsh> OK\n") + [0] * 62

    class FakeConnection:
        mavlink = FakeMavlink()

        def __init__(self) -> None:
            self.mav = FakeMav()
            self.respond_output_sent = False

        def recv_match(self, type=None, blocking=True, timeout=0.05):
            if self.mav.current_flags == 2 and not self.respond_output_sent:
                self.respond_output_sent = True
                return FakeMessage()
            return None

    connection = FakeConnection()

    result = _send_mavlink_shell_command(connection, "mavlink status", timeout_s=0.01)

    probe_flags = [call[1] for call in connection.mav.calls if call[2] == 0]
    payload_flags = [call[1] for call in connection.mav.calls if call[2] > 0]
    assert probe_flags == [6, 2]
    assert payload_flags == [2]
    assert result.flag_label == "respond"
    assert "nsh> OK" in result.output


def test_ros2_global_position_is_kept_outside_active_sensor_gps_slot() -> None:
    subscriber = DualPixhawkAttitudeSubscriber(domain_id=0, left_namespace="uav_0", right_namespace="")
    states = []
    subscriber.gps_changed.connect(lambda camera, state: states.append((camera, state)))

    subscriber._handle_global_position_message(
        "left",
        "uav_0",
        SimpleNamespace(
            timestamp=2_000_000,
            timestamp_sample=1_900_000,
            lat=37.0,
            lon=127.0,
            alt=10.0,
            lat_lon_valid=True,
            alt_valid=True,
            eph=0.4,
            epv=0.6,
        ),
    )

    assert states[0][0] == "left"
    assert states[0][1].source == "global_position"
    assert states[0][1].topic.endswith("vehicle_global_position")
    assert subscriber.latest_gps_by_source["global_position"]["left"].source == "global_position"
    assert "left" not in subscriber.latest_gps
