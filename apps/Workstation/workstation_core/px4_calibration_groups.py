"""PX4 calibration parameter ownership and reset contracts.

The reset operation deliberately delegates the numeric default to PX4's own
``param reset`` command.  That keeps the Workstation aligned with the
connected firmware instead of copying a version-specific default table into
the host application.
"""

from __future__ import annotations

from dataclasses import dataclass

from .px4_attitude_cal_manager import (
    ACCEL_CORRECTION_PARAM_NAMES,
    ATTITUDE_CAL_PARAM_NAMES,
    GYRO_CORRECTION_PARAM_NAMES,
)
from .px4_mag_manager import MAG_CAL_CORRECTION_PARAM_NAMES, MAG_CAL_PARAM_NAMES


@dataclass(frozen=True)
class Px4CalibrationGroup:
    """One calibration-owned parameter set shown in the Pixhawk page."""

    key: str
    label: str
    parameter_names: tuple[str, ...]
    reset_parameter_names: tuple[str, ...]
    contract: str
    description: str

    @property
    def resettable(self) -> bool:
        return bool(self.reset_parameter_names)


_ACCELEROMETER_PARAM_NAMES = tuple(
    name for name in ATTITUDE_CAL_PARAM_NAMES if name.startswith("CAL_ACC")
)
_GYROSCOPE_PARAM_NAMES = tuple(
    name for name in ATTITUDE_CAL_PARAM_NAMES if name.startswith("CAL_GYRO")
)


PX4_CALIBRATION_GROUPS = (
    Px4CalibrationGroup(
        key="accelerometer",
        label="Accelerometer calibration",
        parameter_names=_ACCELEROMETER_PARAM_NAMES,
        reset_parameter_names=ACCEL_CORRECTION_PARAM_NAMES,
        contract="PX4 accelerometer sensor axes; no NED/ENU/camera conversion",
        description=(
            "CAL_ACC* identity, priority, rotation, offsets, and scale factors. "
            "Reset only zeroes offsets and sets scale to 1 (raw sensor passthrough) -- "
            "_ID/_PRIO/_ROT are left alone, like level's SENS_BOARD_ROT, so the slot "
            "stays bound to its device and armable without a QGC re-run."
        ),
    ),
    Px4CalibrationGroup(
        key="gyroscope",
        label="Gyroscope calibration",
        parameter_names=_GYROSCOPE_PARAM_NAMES,
        reset_parameter_names=GYRO_CORRECTION_PARAM_NAMES,
        contract="PX4 gyroscope sensor axes; no NED/ENU/camera conversion",
        description=(
            "CAL_GYRO* identity, priority, rotation, and bias offsets. "
            "Reset only zeroes the bias offsets (raw sensor passthrough) -- "
            "_ID/_PRIO/_ROT are left alone, like level's SENS_BOARD_ROT, so the slot "
            "stays bound to its device and armable without a QGC re-run."
        ),
    ),
    Px4CalibrationGroup(
        key="compass",
        label="Compass / magnetometer calibration",
        parameter_names=tuple(MAG_CAL_PARAM_NAMES),
        reset_parameter_names=MAG_CAL_CORRECTION_PARAM_NAMES,
        contract="PX4 magnetometer sensor axes; no NED/ENU/camera conversion",
        description=(
            "CAL_MAG* identity, priority, rotation, hard/soft-iron, and compensation values. "
            "Reset only zeroes hard-iron/off-diagonal/compensation and sets scale to 1 (raw "
            "sensor passthrough) -- _ID/_PRIO/_ROT/_ROLL/_PITCH/_YAW are left alone, like "
            "level's SENS_BOARD_ROT, so the slot stays bound to its device and armable "
            "without a QGC re-run."
        ),
    ),
    Px4CalibrationGroup(
        key="level",
        label="Level horizon calibration",
        parameter_names=("SENS_BOARD_X_OFF", "SENS_BOARD_Y_OFF"),
        reset_parameter_names=("SENS_BOARD_X_OFF", "SENS_BOARD_Y_OFF"),
        contract="PX4 board roll/pitch offsets; no NED/ENU/camera conversion",
        description="The level-horizon calibration-owned board roll and pitch offsets.",
    ),
    Px4CalibrationGroup(
        key="board",
        label="Board orientation (manual)",
        parameter_names=("SENS_BOARD_ROT", "SENS_BOARD_Z_OFF"),
        reset_parameter_names=(),
        contract="PX4 board orientation parameters; no NED/ENU/camera conversion",
        description="Manual mounting orientation and yaw offset; not changed by level calibration.",
    ),
)

PX4_CALIBRATION_GROUPS_BY_KEY = {group.key: group for group in PX4_CALIBRATION_GROUPS}


def px4_calibration_group(key: str) -> Px4CalibrationGroup:
    """Return a validated group definition for a reset/readback command."""

    try:
        return PX4_CALIBRATION_GROUPS_BY_KEY[str(key)]
    except KeyError as exc:
        raise ValueError(f"Unknown PX4 calibration group: {key}") from exc
