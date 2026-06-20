from wiris_stereo_distance.px4_mag_manager import (
    RM3100_ALTERNATE_PRIMARY_ID,
    RM3100_BACKUP_ID,
    RM3100_PRIMARY_ID,
    MockParameterStore,
    Px4MagManager,
    build_mag_status,
    current_pixhawk_6x_mag_params,
    identify_mag_device,
)


def test_identifies_known_rm3100_devices() -> None:
    primary = identify_mag_device(RM3100_PRIMARY_ID)
    alternate_primary = identify_mag_device(RM3100_ALTERNATE_PRIMARY_ID)
    backup = identify_mag_device(RM3100_BACKUP_ID)

    assert primary.is_rm3100
    assert "0x7D" in primary.device_type
    assert alternate_primary.is_rm3100
    assert "0x7C" in alternate_primary.device_type
    assert backup.is_rm3100
    assert "0x7B" in backup.device_type


def test_pixhawk2_rm3100_primary_variant_is_registered_as_rm3100() -> None:
    params = current_pixhawk_6x_mag_params()
    params["CAL_MAG3_ID"] = RM3100_ALTERNATE_PRIMARY_ID

    status = build_mag_status(params, actual_selected_device_id=RM3100_ALTERNATE_PRIMARY_ID)

    assert status.slots[3].is_rm3100
    assert status.is_actual_rm3100
    assert "PX4 is using RM3100" in status.warning_text


def test_current_state_marks_cal_mag3_as_rm3100_but_expected_tie_uses_first_highest_priority() -> None:
    status = build_mag_status(current_pixhawk_6x_mag_params(), actual_selected_device_id=396809)

    cal_mag3 = status.slots[3]
    assert cal_mag3.device_id == RM3100_PRIMARY_ID
    assert cal_mag3.is_rm3100
    assert status.expected_selected_device_id == 396809
    assert "PX4 is still using I2C/internal magnetometer" in status.warning_text


def test_use_rm3100_for_yaw_with_one_rm3100_disables_internal_compasses() -> None:
    store = MockParameterStore(current_pixhawk_6x_mag_params())
    manager = Px4MagManager(store)

    status = manager.use_rm3100_for_yaw()

    assert store.get_param("CAL_MAG3_PRIO") == 100
    assert store.get_param("CAL_MAG1_PRIO") == 0
    assert store.get_param("CAL_MAG2_PRIO") == 0
    assert store.get_param("CAL_MAG0_PRIO") == 0
    assert status.expected_selected_device_id == RM3100_PRIMARY_ID


def test_use_rm3100_for_yaw_with_two_rm3100_sets_primary_and_backup() -> None:
    params = current_pixhawk_6x_mag_params()
    params["CAL_MAG4_ID"] = RM3100_BACKUP_ID
    params["CAL_MAG4_PRIO"] = 75
    params["CAL_MAG4_ROT"] = None
    store = MockParameterStore(params)
    manager = Px4MagManager(store)

    status = manager.use_rm3100_for_yaw()

    assert store.get_param("CAL_MAG3_PRIO") == 100
    assert store.get_param("CAL_MAG4_PRIO") == 90
    assert store.get_param("CAL_MAG0_PRIO") == 0
    assert store.get_param("CAL_MAG1_PRIO") == 0
    assert store.get_param("CAL_MAG2_PRIO") == 0
    assert status.expected_selected_device_id == RM3100_PRIMARY_ID


def test_armed_state_blocks_parameter_changes() -> None:
    manager = Px4MagManager(MockParameterStore(current_pixhawk_6x_mag_params()))
    manager.armed = True

    try:
        manager.use_rm3100_for_yaw()
    except RuntimeError as exc:
        assert "Disarm required" in str(exc)
    else:
        raise AssertionError("Expected armed state to block parameter changes")
