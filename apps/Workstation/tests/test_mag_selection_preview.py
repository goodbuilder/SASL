from __future__ import annotations

from Workstation.workstation_core.px4_mag_manager import (
    BOARD_INTERNAL_MAG_ID,
    GPS1_MAG_ID,
    GPS2_MAG_ID,
    RM3100_BACKUP_ID,
    RM3100_PRIMARY_ID,
    build_mag_selection_preview,
)


def _params(*, slot2_id: int, slot3_id: int = RM3100_BACKUP_ID) -> dict[str, int]:
    return {
        "CAL_MAG0_ID": BOARD_INTERNAL_MAG_ID,
        "CAL_MAG0_PRIO": 1,
        "CAL_MAG1_ID": GPS1_MAG_ID,
        "CAL_MAG1_PRIO": 1,
        "CAL_MAG2_ID": slot2_id,
        "CAL_MAG2_PRIO": 1,
        "CAL_MAG3_ID": slot3_id,
        "CAL_MAG3_PRIO": 100,
    }


def test_five_instance_topology_keeps_can2_visible_but_predicts_can1() -> None:
    preview = build_mag_selection_preview(
        _params(slot2_id=GPS2_MAG_ID),
        {
            0: BOARD_INTERNAL_MAG_ID,
            1: GPS1_MAG_ID,
            2: GPS2_MAG_ID,
            3: RM3100_PRIMARY_ID,
            4: RM3100_BACKUP_ID,
        },
        requested_priorities={"CAN2": 100},
        actual_selected_device_id=RM3100_PRIMARY_ID,
    )

    assert preview.predicted_device_id == RM3100_PRIMARY_ID
    assert preview.predicted_instance == 3
    assert preview.out_of_range_device_ids == (RM3100_BACKUP_ID,)
    assert preview.actual_device_id == RM3100_PRIMARY_ID


def test_four_instance_topology_predicts_can2_at_instance_three() -> None:
    preview = build_mag_selection_preview(
        _params(slot2_id=RM3100_PRIMARY_ID),
        {
            0: BOARD_INTERNAL_MAG_ID,
            1: GPS1_MAG_ID,
            2: RM3100_PRIMARY_ID,
            3: RM3100_BACKUP_ID,
        },
        requested_priorities={"CAN2": 100},
    )

    assert preview.predicted_device_id == RM3100_BACKUP_ID
    assert preview.predicted_role == "CAN2"
    assert preview.predicted_instance == 3
    assert preview.out_of_range_device_ids == ()


def test_preview_keeps_requested_priority_edit_separate_from_actual_device() -> None:
    preview = build_mag_selection_preview(
        _params(slot2_id=GPS2_MAG_ID),
        {0: BOARD_INTERNAL_MAG_ID, 1: GPS1_MAG_ID, 2: GPS2_MAG_ID, 3: RM3100_BACKUP_ID},
        requested_priorities={"CAN2": 100, "GPS2_MAG": 75},
        actual_selected_device_id=GPS2_MAG_ID,
    )

    assert preview.predicted_device_id == RM3100_BACKUP_ID
    assert preview.actual_device_id == GPS2_MAG_ID
    assert preview.actual_role == "GPS2_MAG"
