from __future__ import annotations

from pymavlink.dialects.v20 import common as mavlink_common

from Workstation.workstation_core.px4_attitude_cal_manager import (
    BOARD_ROTATION_LABELS,
    board_rotation_label,
)


def test_board_rotation_labels_match_mavlink_sensor_rotation_enum() -> None:
    # BOARD_ROTATION_LABELS is a hand-written subset of MAV_SENSOR_ROTATION_*;
    # the codes are not evenly spaced, so a copy/paste slip silently maps a
    # code to the wrong neighboring label instead of failing loudly. Check
    # every entry against the actual enum instead of pinning expected values.
    for code, label in BOARD_ROTATION_LABELS.items():
        expected_name = f"MAV_SENSOR_{label}"
        assert hasattr(mavlink_common, expected_name), f"no such MAVLink rotation enum: {expected_name}"
        assert getattr(mavlink_common, expected_name) == code, (
            f"BOARD_ROTATION_LABELS[{code}] = {label!r} but MAVLink defines "
            f"{expected_name} = {getattr(mavlink_common, expected_name)}"
        )


def test_board_rotation_label_reports_none_and_unknown_codes() -> None:
    assert board_rotation_label(None) == "N/A"
    assert board_rotation_label(9999) == "ROTATION_CODE_9999"
    assert board_rotation_label(0) == "ROTATION_NONE"
