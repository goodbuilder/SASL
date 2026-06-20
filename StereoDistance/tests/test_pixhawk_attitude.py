from __future__ import annotations

from wiris_stereo_distance.pixhawk_attitude import DualPixhawkAttitudeSubscriber
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
