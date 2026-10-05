"""Pure contracts for the Pixhawk magnetometer yaw readout."""

from __future__ import annotations

import math

import pytest

from Workstation.workstation_core.magnetometer_yaw import (
    calibrated_magnetometer_yaw_deg,
    project_magnetometer_yaw,
    raw_magnetometer_yaw_deg,
    wrap_signed_yaw_deg,
)


def test_raw_magnetometer_yaw_uses_px4_signed_sensor_axis_convention() -> None:
    assert raw_magnetometer_yaw_deg(1.0, -1.0) == pytest.approx(45.0)
    assert raw_magnetometer_yaw_deg(0.0, 1.0) == pytest.approx(-90.0)
    assert raw_magnetometer_yaw_deg(0.0, 0.0) is None
    assert raw_magnetometer_yaw_deg(math.nan, 1.0) is None


def test_calibrated_magnetometer_yaw_applies_px4_offset_scale_and_rotation() -> None:
    params = {
        "CAL_MAG2_XOFF": 1.0,
        "CAL_MAG2_YOFF": 0.0,
        "CAL_MAG2_ZOFF": 0.0,
        "CAL_MAG2_XSCALE": 1.0,
        "CAL_MAG2_YSCALE": 1.0,
        "CAL_MAG2_ZSCALE": 1.0,
        "CAL_MAG2_XODIAG": 0.0,
        "CAL_MAG2_YODIAG": 0.0,
        "CAL_MAG2_ZODIAG": 0.0,
        "CAL_MAG2_XCOMP": 0.0,
        "CAL_MAG2_YCOMP": 0.0,
        "CAL_MAG2_ZCOMP": 0.0,
        "CAL_MAG2_ROT": 2.0,
        "CAL_MAG2_ROLL": 0.0,
        "CAL_MAG2_PITCH": 0.0,
        "CAL_MAG2_YAW": 0.0,
    }
    result = calibrated_magnetometer_yaw_deg(
        x=2.0,
        y=0.0,
        z=0.0,
        calibration_params=params,
        calibration_slot=2,
    )

    assert result["corrected_yaw_state"] == "CALIBRATED"
    assert result["corrected_yaw_deg"] == pytest.approx(-90.0)


def test_calibrated_magnetometer_yaw_waits_for_parameter_readback() -> None:
    result = calibrated_magnetometer_yaw_deg(
        x=1.0,
        y=0.0,
        z=0.0,
        calibration_params=None,
        calibration_slot=None,
    )

    assert result["corrected_yaw_deg"] is None
    assert result["corrected_yaw_state"] == "PARAM WAITING"


def test_yaw_wrap_is_half_open_and_delta_is_px4_minus_raw() -> None:
    assert wrap_signed_yaw_deg(180.0) == pytest.approx(-180.0)
    assert wrap_signed_yaw_deg(540.0) == pytest.approx(-180.0)
    projection = project_magnetometer_yaw(
        x=1.0,
        y=0.0,
        sensor_device_id=7,
        active_device_id=7,
        px4_yaw_deg=-170.0,
        px4_age_s=0.1,
    )
    assert projection.px4_yaw_state == "SELECTED"
    assert projection.raw_yaw_deg == pytest.approx(0.0)
    assert projection.px4_yaw_deg == pytest.approx(-170.0)
    assert projection.delta_yaw_deg == pytest.approx(-170.0)


@pytest.mark.parametrize(
    ("sensor_device_id", "active_device_id", "px4_age_s", "state"),
    [
        (7, None, 0.1, "ACTIVE ID WAITING"),
        (None, 7, 0.1, "IDENTITY UNKNOWN"),
        (8, 7, 0.1, "NOT SELECTED"),
        (7, 7, 2.1, "STALE"),
    ],
)
def test_px4_reference_requires_fresh_exact_active_identity(
    sensor_device_id: int | None,
    active_device_id: int | None,
    px4_age_s: float,
    state: str,
) -> None:
    projection = project_magnetometer_yaw(
        x=1.0,
        y=0.0,
        sensor_device_id=sensor_device_id,
        active_device_id=active_device_id,
        px4_yaw_deg=20.0,
        px4_age_s=px4_age_s,
    )
    assert projection.px4_yaw_state == state
    assert projection.px4_yaw_deg is None
    assert projection.delta_yaw_deg is None


def test_px4_reference_keeps_raw_diagnostic_when_attitude_is_missing() -> None:
    projection = project_magnetometer_yaw(
        x=0.0,
        y=-1.0,
        sensor_device_id=7,
        active_device_id=7,
        px4_yaw_deg=None,
        px4_age_s=None,
    )
    assert projection.raw_yaw_deg == pytest.approx(90.0)
    assert projection.px4_yaw_state == "WAITING"
    assert projection.px4_yaw_deg is None
