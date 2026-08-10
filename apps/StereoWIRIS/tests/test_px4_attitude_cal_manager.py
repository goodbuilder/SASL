from wiris_stereo_distance.px4_attitude_cal_manager import (
    MockParameterStore,
    Px4AttitudeCalManager,
    build_attitude_cal_status,
    current_pixhawk_6x_attitude_cal_params,
)


def test_uncalibrated_pixhawk_reports_accel_and_gyro_warnings() -> None:
    status = build_attitude_cal_status(current_pixhawk_6x_attitude_cal_params())

    assert not any(slot.calibrated for slot in status.accel_slots)
    assert not any(slot.calibrated for slot in status.gyro_slots)
    assert "가속도계가 보정되지 않았습니다" in status.warning_text
    assert "자이로가 보정되지 않았습니다" in status.warning_text


def test_calibrated_accel_and_gyro_slot_clears_warning_and_reports_float_offsets() -> None:
    params = current_pixhawk_6x_attitude_cal_params()
    params.update(
        {
            "CAL_ACC0_ID": 1376268,
            "CAL_ACC0_PRIO": 75,
            "CAL_ACC0_XOFF": 0.0123,
            "CAL_ACC0_YOFF": -0.045,
            "CAL_ACC0_ZOFF": 0.09,
            "CAL_ACC0_XSCALE": 1.001,
            "CAL_ACC0_YSCALE": 0.998,
            "CAL_ACC0_ZSCALE": 1.002,
            "CAL_GYRO0_ID": 1376268,
            "CAL_GYRO0_PRIO": 75,
            "CAL_GYRO0_XOFF": 0.0011,
            "CAL_GYRO0_YOFF": -0.0009,
            "CAL_GYRO0_ZOFF": 0.0002,
        }
    )

    status = build_attitude_cal_status(params)

    assert status.accel_slots[0].calibrated
    assert status.accel_slots[0].offset_m_s2 == (0.0123, -0.045, 0.09)
    assert status.accel_slots[0].scale == (1.001, 0.998, 1.002)
    assert status.gyro_slots[0].calibrated
    assert status.gyro_slots[0].offset_rad_s == (0.0011, -0.0009, 0.0002)
    assert "가속도계가 보정되지 않았습니다" not in status.warning_text
    assert "자이로가 보정되지 않았습니다" not in status.warning_text


def test_board_rotation_and_level_horizon_offsets_are_surfaced() -> None:
    params = current_pixhawk_6x_attitude_cal_params()
    params.update(
        {
            "SENS_BOARD_ROT": 4,
            "SENS_BOARD_X_OFF": 0.5,
            "SENS_BOARD_Y_OFF": -0.3,
            "SENS_BOARD_Z_OFF": 0.0,
        }
    )

    status = build_attitude_cal_status(params)

    assert status.board_rotation.rotation_label == "ROTATION_YAW_180"
    assert status.board_rotation.level_offset_deg == (0.5, -0.3, 0.0)


def test_unknown_board_rotation_code_falls_back_to_generic_label() -> None:
    params = current_pixhawk_6x_attitude_cal_params()
    params["SENS_BOARD_ROT"] = 99

    status = build_attitude_cal_status(params)

    assert status.board_rotation.rotation_label == "ROTATION_CODE_99"


def test_manager_refresh_reads_through_store() -> None:
    params = current_pixhawk_6x_attitude_cal_params()
    params["CAL_ACC0_ID"] = 1376268
    params["CAL_ACC0_PRIO"] = 75
    store = MockParameterStore(params)
    manager = Px4AttitudeCalManager(store)

    status = manager.refresh()

    assert status.accel_slots[0].device_id == 1376268
    assert status.accel_slots[0].enabled
