from wiris_stereo_distance.px4_mag_manager import (
    MockParameterStore,
    Px4MagManager,
    build_mag_status,
    current_pixhawk_6x_mag_params,
    identify_mag_device,
    px4_device_id_bus_type,
    px4_device_id_interface,
)


def test_decodes_interface_from_px4_device_id_without_known_id() -> None:
    dronecan_mag_id = 0x00887A03
    i2c_mag_id = 0x00431021

    assert px4_device_id_bus_type(dronecan_mag_id) == 3
    assert px4_device_id_interface(dronecan_mag_id) == "DroneCAN"
    assert identify_mag_device(dronecan_mag_id).interface == "DroneCAN"
    assert identify_mag_device(dronecan_mag_id).device_type == "PX4 MAG"
    assert identify_mag_device(dronecan_mag_id).is_rm3100 is False
    assert px4_device_id_interface(i2c_mag_id) == "I2C"


def test_build_mag_status_uses_slots_interfaces_and_priorities_without_known_ids() -> None:
    params = current_pixhawk_6x_mag_params()
    params.update(
        {
            "UAVCAN_ENABLE": 2,
            "UAVCAN_SUB_MAG": 1,
            "CAL_MAG0_ID": 0x00431021,
            "CAL_MAG0_PRIO": 0,
            "CAL_MAG1_ID": 0x00431029,
            "CAL_MAG1_PRIO": 0,
            "CAL_MAG2_ID": 0x00431031,
            "CAL_MAG2_PRIO": 0,
            "CAL_MAG3_ID": 0x00887A03,
            "CAL_MAG3_PRIO": 100,
        }
    )

    status = build_mag_status(params, actual_selected_device_id=0x00887A03)

    assert status.expected_selected_device_id == 0x00887A03
    assert status.is_actual_rm3100 is False
    assert status.slots[3].interface == "DroneCAN"
    assert status.slots[3].status == "Actual selected"
    assert "PX4 selected MAG interface: DroneCAN." in status.warning_text


def test_use_rm3100_for_yaw_uses_dronecan_slots_without_known_ids() -> None:
    params = current_pixhawk_6x_mag_params()
    params.update(
        {
            "UAVCAN_ENABLE": 2,
            "UAVCAN_SUB_MAG": 1,
            "CAL_MAG0_ID": 0x00431021,
            "CAL_MAG0_PRIO": 75,
            "CAL_MAG1_ID": 0x00431029,
            "CAL_MAG1_PRIO": 75,
            "CAL_MAG2_ID": 0x00887A03,
            "CAL_MAG2_PRIO": 75,
            "CAL_MAG3_ID": 0x00887B03,
            "CAL_MAG3_PRIO": 75,
        }
    )
    store = MockParameterStore(params)
    manager = Px4MagManager(store)

    status = manager.use_rm3100_for_yaw()

    assert store.get_param("CAL_MAG2_PRIO") == 100
    assert store.get_param("CAL_MAG3_PRIO") == 90
    assert store.get_param("CAL_MAG0_PRIO") == 0
    assert store.get_param("CAL_MAG1_PRIO") == 0
    assert status.expected_selected_device_id == 0x00887A03


def test_armed_state_blocks_parameter_changes() -> None:
    manager = Px4MagManager(MockParameterStore(current_pixhawk_6x_mag_params()))
    manager.armed = True

    try:
        manager.use_rm3100_for_yaw()
    except RuntimeError as exc:
        assert "Disarm required" in str(exc)
    else:
        raise AssertionError("Expected armed state to block parameter changes")
