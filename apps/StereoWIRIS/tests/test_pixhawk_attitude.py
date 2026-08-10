from __future__ import annotations

from wiris_stereo_distance.pixhawk_attitude import DualPixhawkAttitudeSubscriber, _send_mavlink_shell_command
from wiris_stereo_distance.px4_mag_manager import (
    RM3100_ALTERNATE_PRIMARY_ID,
    RM3100_BACKUP_ID,
    RM3100_PRIMARY_ID,
)


def test_ros2_mag_slots_prioritize_known_rm3100_ids_for_can_labels() -> None:
    subscriber = DualPixhawkAttitudeSubscriber.__new__(DualPixhawkAttitudeSubscriber)
    subscriber.mag_device_slots = {"left": {}, "right": {}}

    assert subscriber._mag_slot_for_device("left", 4395041) == "BOARD"
    assert subscriber._mag_slot_for_device("left", 396809) == "GPS1_MAG"
    assert subscriber._mag_slot_for_device("left", 396817) == "GPS2_MAG"
    assert subscriber._mag_slot_for_device("left", RM3100_PRIMARY_ID) == "CAN1"
    assert subscriber._mag_slot_for_device("left", RM3100_ALTERNATE_PRIMARY_ID) == "CAN1"
    assert subscriber._mag_slot_for_device("left", RM3100_BACKUP_ID) == "CAN2"


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
