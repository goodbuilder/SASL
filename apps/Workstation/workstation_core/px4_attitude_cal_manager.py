from __future__ import annotations

import json
from dataclasses import dataclass
from typing import Protocol

ACC_SLOTS = (0, 1, 2)
GYRO_SLOTS = (0, 1, 2)

ACCEL_AXIS_SUFFIXES = ("ROT", "XOFF", "YOFF", "ZOFF", "XSCALE", "YSCALE", "ZSCALE")
GYRO_AXIS_SUFFIXES = ("ROT", "XOFF", "YOFF", "ZOFF")
# PX4's level-horizon calibration owns the board roll/pitch fine-tuning
# offsets.  SENS_BOARD_Z_OFF remains a manual mounting parameter.
LEVEL_HORIZON_RESET_PARAM_NAMES = (
    "SENS_BOARD_X_OFF",
    "SENS_BOARD_Y_OFF",
)

ATTITUDE_CAL_PARAM_NAMES = (
    *(f"CAL_ACC{slot}_ID" for slot in ACC_SLOTS),
    *(f"CAL_ACC{slot}_PRIO" for slot in ACC_SLOTS),
    *(f"CAL_ACC{slot}_{suffix}" for slot in ACC_SLOTS for suffix in ACCEL_AXIS_SUFFIXES),
    *(f"CAL_GYRO{slot}_ID" for slot in GYRO_SLOTS),
    *(f"CAL_GYRO{slot}_PRIO" for slot in GYRO_SLOTS),
    *(f"CAL_GYRO{slot}_{suffix}" for slot in GYRO_SLOTS for suffix in GYRO_AXIS_SUFFIXES),
    "SENS_BOARD_ROT",
    "SENS_BOARD_X_OFF",
    "SENS_BOARD_Y_OFF",
    "SENS_BOARD_Z_OFF",
)

# PX4/MAVLink Rotation enum (MAV_SENSOR_ROTATION_*, src/lib/conversion/rotation.h),
# only the commonly used subset. Values verified against pymavlink's
# dialects.v20.common.MAV_SENSOR_ROTATION_* -- do not reorder without
# re-checking against that source, the codes are not evenly spaced.
BOARD_ROTATION_LABELS = {
    0: "ROTATION_NONE",
    1: "ROTATION_YAW_45",
    2: "ROTATION_YAW_90",
    3: "ROTATION_YAW_135",
    4: "ROTATION_YAW_180",
    5: "ROTATION_YAW_225",
    6: "ROTATION_YAW_270",
    7: "ROTATION_YAW_315",
    8: "ROTATION_ROLL_180",
    12: "ROTATION_PITCH_180",
    16: "ROTATION_ROLL_90",
    20: "ROTATION_ROLL_270",
    24: "ROTATION_PITCH_90",
    25: "ROTATION_PITCH_270",
    28: "ROTATION_ROLL_90_PITCH_90",
    32: "ROTATION_ROLL_270_PITCH_180",
}


class ParameterStore(Protocol):
    def get_param(self, name: str) -> int | float | None:
        ...


@dataclass(frozen=True)
class AccelSlotStatus:
    slot: int
    device_id: int | None
    priority: int | None
    enabled: bool
    calibrated: bool
    offset_m_s2: tuple[float | None, float | None, float | None]
    scale: tuple[float | None, float | None, float | None]


@dataclass(frozen=True)
class GyroSlotStatus:
    slot: int
    device_id: int | None
    priority: int | None
    enabled: bool
    calibrated: bool
    offset_rad_s: tuple[float | None, float | None, float | None]


@dataclass(frozen=True)
class BoardRotationStatus:
    rotation_code: int | None
    rotation_label: str
    level_offset_deg: tuple[float | None, float | None, float | None]


@dataclass(frozen=True)
class Px4AttitudeCalStatus:
    params: dict[str, int | float | None]
    accel_slots: tuple[AccelSlotStatus, ...]
    gyro_slots: tuple[GyroSlotStatus, ...]
    board_rotation: BoardRotationStatus
    warning_text: str

    def to_dict(self) -> dict[str, object]:
        return {
            "accel_slots": [slot.__dict__ for slot in self.accel_slots],
            "gyro_slots": [slot.__dict__ for slot in self.gyro_slots],
            "board_rotation": self.board_rotation.__dict__,
            "warning_text": self.warning_text,
        }

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), ensure_ascii=False, sort_keys=True)


class MockParameterStore:
    def __init__(self, values: dict[str, int | float | None] | None = None) -> None:
        self.values = dict(values or {})

    def get_param(self, name: str) -> int | float | None:
        return self.values.get(name)

    def set_param(self, name: str, value: int | float) -> None:
        if name not in self.values:
            raise KeyError(f"Parameter is not registered: {name}")
        self.values[name] = value


def current_pixhawk_6x_attitude_cal_params() -> dict[str, int | float | None]:
    return {name: None for name in ATTITUDE_CAL_PARAM_NAMES}


def read_attitude_cal_params(store: ParameterStore) -> dict[str, int | float | None]:
    return {name: store.get_param(name) for name in ATTITUDE_CAL_PARAM_NAMES}


def _param_int(params: dict[str, int | float | None], name: str) -> int | None:
    value = params.get(name)
    return None if value is None else int(value)


def _param_float(params: dict[str, int | float | None], name: str) -> float | None:
    value = params.get(name)
    return None if value is None else float(value)


def board_rotation_label(rotation_code: int | None) -> str:
    if rotation_code is None:
        return "N/A"
    return BOARD_ROTATION_LABELS.get(rotation_code, f"ROTATION_CODE_{rotation_code}")


def build_attitude_cal_status(params: dict[str, int | float | None]) -> Px4AttitudeCalStatus:
    warnings: list[str] = []

    accel_slots: list[AccelSlotStatus] = []
    for slot in ACC_SLOTS:
        device_id = _param_int(params, f"CAL_ACC{slot}_ID")
        priority = _param_int(params, f"CAL_ACC{slot}_PRIO")
        offset = tuple(_param_float(params, f"CAL_ACC{slot}_{axis}OFF") for axis in "XYZ")
        scale = tuple(_param_float(params, f"CAL_ACC{slot}_{axis}SCALE") for axis in "XYZ")
        calibrated = device_id is not None and device_id != 0
        enabled = priority is not None and priority > 0
        accel_slots.append(
            AccelSlotStatus(
                slot=slot,
                device_id=device_id,
                priority=priority,
                enabled=enabled,
                calibrated=calibrated,
                offset_m_s2=offset,
                scale=scale,
            )
        )
    if params and not any(slot.calibrated for slot in accel_slots):
        warnings.append("가속도계가 보정되지 않았습니다. QGC Sensor Setup에서 Accelerometer Calibration을 실행하세요.")

    gyro_slots: list[GyroSlotStatus] = []
    for slot in GYRO_SLOTS:
        device_id = _param_int(params, f"CAL_GYRO{slot}_ID")
        priority = _param_int(params, f"CAL_GYRO{slot}_PRIO")
        offset = tuple(_param_float(params, f"CAL_GYRO{slot}_{axis}OFF") for axis in "XYZ")
        calibrated = device_id is not None and device_id != 0
        enabled = priority is not None and priority > 0
        gyro_slots.append(
            GyroSlotStatus(
                slot=slot,
                device_id=device_id,
                priority=priority,
                enabled=enabled,
                calibrated=calibrated,
                offset_rad_s=offset,
            )
        )
    if params and not any(slot.calibrated for slot in gyro_slots):
        warnings.append("자이로가 보정되지 않았습니다. QGC Sensor Setup에서 Gyroscope Calibration을 실행하세요.")

    rotation_code = _param_int(params, "SENS_BOARD_ROT")
    board_rotation = BoardRotationStatus(
        rotation_code=rotation_code,
        rotation_label=board_rotation_label(rotation_code),
        level_offset_deg=(
            _param_float(params, "SENS_BOARD_X_OFF"),
            _param_float(params, "SENS_BOARD_Y_OFF"),
            _param_float(params, "SENS_BOARD_Z_OFF"),
        ),
    )

    return Px4AttitudeCalStatus(
        params=params,
        accel_slots=tuple(accel_slots),
        gyro_slots=tuple(gyro_slots),
        board_rotation=board_rotation,
        warning_text="\n".join(warnings),
    )


class Px4AttitudeCalManager:
    def __init__(self, store: ParameterStore) -> None:
        self.store = store

    def refresh(self) -> Px4AttitudeCalStatus:
        return build_attitude_cal_status(read_attitude_cal_params(self.store))
