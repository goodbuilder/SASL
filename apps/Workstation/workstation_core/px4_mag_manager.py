from __future__ import annotations

import json
from dataclasses import dataclass
from typing import Protocol


RM3100_PRIMARY_ID = 8944899
RM3100_ALTERNATE_PRIMARY_ID = 8944643
RM3100_PRIMARY_IDS = (RM3100_PRIMARY_ID, RM3100_ALTERNATE_PRIMARY_ID)
RM3100_BACKUP_ID = 8944387
RM3100_ALTERNATE_BACKUP_ID = 8944395
RM3100_BACKUP_IDS = (RM3100_BACKUP_ID, RM3100_ALTERNATE_BACKUP_ID)
RM3100_IDS = (*RM3100_PRIMARY_IDS, *RM3100_BACKUP_IDS)
BOARD_INTERNAL_MAG_ID = 4395041
GPS1_MAG_ID = 396809
GPS2_MAG_ID = 396817
GPS_MAG_IDS = (GPS1_MAG_ID, GPS2_MAG_ID)
INTERNAL_MAG_IDS = (BOARD_INTERNAL_MAG_ID, *GPS_MAG_IDS)
# PX4 v1.17's sensors module declares four calibration instances, exposed as
# CAL_MAG0_* through CAL_MAG3_*. CAL_MAG4_* is not part of the stock firmware
# parameter surface and waiting for it turns an otherwise read-only refresh
# into eighteen sequential timeouts. Explicit callers may still request an
# arbitrary parameter through the generic MAVLink parameter API.
MAG_SLOTS = tuple(range(4))
MAG_PRIORITY_LEVELS = (
    (-1, "Uninitialized"),
    (0, "Disabled"),
    (1, "Min"),
    (25, "Low"),
    (50, "Medium"),
    (75, "High"),
    (100, "Max"),
)
MAG_PRIORITY_LABELS = dict(MAG_PRIORITY_LEVELS)


def mag_priority_label(value: int | float | None) -> str:
    """Return the PX4 semantic label while preserving non-standard raw values."""

    if value is None:
        return "N/A"
    raw = int(value)
    label = MAG_PRIORITY_LABELS.get(raw)
    return label if label is not None else f"Custom ({raw})"


def mag_priority_display(value: int | float | None) -> str:
    """Return a lossless operator-facing value such as ``High (75)``."""

    if value is None:
        return "N/A"
    raw = int(value)
    label = MAG_PRIORITY_LABELS.get(raw)
    return f"{label} ({raw})" if label is not None else f"Custom ({raw})"


# Hard-iron/soft-iron fit coefficients only -- the calibration *output*,
# as opposed to _ID/_PRIO (which slot/device this is) and _ROT/_ROLL/_PITCH/
# _YAW (external compass mounting orientation, a manual descriptor like
# SENS_BOARD_ROT). Resetting only these to their firmware defaults
# (0 offset, 1 scale, 0 off-diagonal/compensation) makes PX4 apply an
# identity transform -- i.e. pass the raw sensor reading through -- without
# forgetting which physical device this slot belongs to.
MAG_CAL_CORRECTION_SUFFIXES = (
    "XOFF",
    "YOFF",
    "ZOFF",
    "XSCALE",
    "YSCALE",
    "ZSCALE",
    "XODIAG",
    "YODIAG",
    "ZODIAG",
    "XCOMP",
    "YCOMP",
    "ZCOMP",
)
MAG_CAL_CORRECTION_PARAM_NAMES = tuple(
    f"CAL_MAG{slot}_{suffix}" for slot in MAG_SLOTS for suffix in MAG_CAL_CORRECTION_SUFFIXES
)
MAG_CAL_PARAM_NAMES = (
    *(f"CAL_MAG{slot}_ID" for slot in MAG_SLOTS),
    *(f"CAL_MAG{slot}_PRIO" for slot in MAG_SLOTS),
    *(f"CAL_MAG{slot}_{suffix}" for slot in MAG_SLOTS for suffix in ("ROT", "ROLL", "PITCH", "YAW")),
    *MAG_CAL_CORRECTION_PARAM_NAMES,
)
MAG_PARAM_NAMES = (
    "UAVCAN_ENABLE",
    "UAVCAN_SUB_MAG",
    "EKF2_MAG_TYPE",
    *MAG_CAL_PARAM_NAMES,
)


class ParameterStore(Protocol):
    def get_param(self, name: str) -> int | float | None:
        ...

    def set_param(self, name: str, value: int | float) -> None:
        ...


@dataclass(frozen=True)
class MagDeviceInfo:
    device_id: int | None
    device_type: str
    interface: str
    is_rm3100: bool


@dataclass(frozen=True)
class MagSlotStatus:
    slot: int
    device_id: int | None
    priority: int | None
    rotation: int | None
    enabled: bool
    expected_selected: bool
    actual_selected: bool
    device_type: str
    interface: str
    is_rm3100: bool
    status: str


@dataclass(frozen=True)
class Px4MagStatus:
    params: dict[str, int | float | None]
    slots: tuple[MagSlotStatus, ...]
    rm3100_primary_id: int
    rm3100_backup_id: int
    expected_selected_device_id: int | None
    actual_selected_device_id: int | None
    is_actual_rm3100: bool
    warning_text: str
    armed: bool

    def to_dict(self) -> dict[str, object]:
        data: dict[str, object] = {
            "rm3100_primary_id": self.rm3100_primary_id,
            "rm3100_backup_id": self.rm3100_backup_id,
            "expected_selected_device_id": self.expected_selected_device_id,
            "actual_selected_device_id": self.actual_selected_device_id,
            "is_actual_rm3100": self.is_actual_rm3100,
            "warning_text": self.warning_text,
            "armed": self.armed,
            "slots": [slot.__dict__ for slot in self.slots],
        }
        for slot in MAG_SLOTS:
            data[f"CAL_MAG{slot}_ID"] = self.params.get(f"CAL_MAG{slot}_ID")
            data[f"CAL_MAG{slot}_PRIO"] = self.params.get(f"CAL_MAG{slot}_PRIO")
            for suffix in (
                "ROT",
                "ROLL",
                "PITCH",
                "YAW",
                "XOFF",
                "YOFF",
                "ZOFF",
                "XSCALE",
                "YSCALE",
                "ZSCALE",
                "XODIAG",
                "YODIAG",
                "ZODIAG",
                "XCOMP",
                "YCOMP",
                "ZCOMP",
            ):
                data[f"CAL_MAG{slot}_{suffix}"] = self.params.get(f"CAL_MAG{slot}_{suffix}")
        return data

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


def current_pixhawk_6x_mag_params() -> dict[str, int | float | None]:
    values: dict[str, int | float | None] = {
        "UAVCAN_ENABLE": 2,
        "UAVCAN_SUB_MAG": 1,
        "EKF2_MAG_TYPE": None,
        "CAL_MAG0_ID": 4395041,
        "CAL_MAG0_PRIO": 0,
        "CAL_MAG0_ROT": None,
        "CAL_MAG1_ID": 396809,
        "CAL_MAG1_PRIO": 75,
        "CAL_MAG1_ROT": None,
        "CAL_MAG2_ID": 396817,
        "CAL_MAG2_PRIO": 75,
        "CAL_MAG2_ROT": None,
        "CAL_MAG3_ID": RM3100_PRIMARY_ID,
        "CAL_MAG3_PRIO": 75,
        "CAL_MAG3_ROT": None,
    }
    return values


def identify_mag_device(device_id: int | None) -> MagDeviceInfo:
    if device_id == RM3100_PRIMARY_ID:
        return MagDeviceInfo(device_id, "RM3100 #1 / UAVCAN node 0x7D", "DroneCAN", True)
    if device_id == RM3100_ALTERNATE_PRIMARY_ID:
        return MagDeviceInfo(device_id, "RM3100 #1 / UAVCAN node 0x7C", "DroneCAN", True)
    if device_id in RM3100_BACKUP_IDS:
        return MagDeviceInfo(device_id, "RM3100 #2 / UAVCAN node 0x7B", "DroneCAN", True)
    if device_id == BOARD_INTERNAL_MAG_ID:
        return MagDeviceInfo(device_id, "Board internal BMM150", "Pixhawk internal I2C", False)
    if device_id == GPS1_MAG_ID:
        return MagDeviceInfo(device_id, "GPS1 integrated IST8310 compass", "GPS1 I2C compass", False)
    if device_id == GPS2_MAG_ID:
        return MagDeviceInfo(device_id, "GPS2 integrated IST8310 compass", "GPS2 I2C compass", False)
    if device_id is None:
        return MagDeviceInfo(device_id, "N/A", "N/A", False)
    return MagDeviceInfo(device_id, "Unknown MAG", "Unknown", False)


def read_mag_params(store: ParameterStore) -> dict[str, int | float | None]:
    return {name: store.get_param(name) for name in MAG_PARAM_NAMES}


def _param_int(params: dict[str, int | float | None], name: str) -> int | None:
    value = params.get(name)
    return None if value is None else int(value)


def expected_selected_slot(params: dict[str, int | float | None]) -> int | None:
    best_slot: int | None = None
    best_priority = 0
    for slot in MAG_SLOTS:
        priority = _param_int(params, f"CAL_MAG{slot}_PRIO")
        device_id = _param_int(params, f"CAL_MAG{slot}_ID")
        if priority is None or priority <= 0 or device_id is None:
            continue
        if best_slot is None or priority > best_priority:
            best_slot = slot
            best_priority = priority
    return best_slot


def build_mag_status(
    params: dict[str, int | float | None],
    *,
    actual_selected_device_id: int | None = None,
    armed: bool = False,
) -> Px4MagStatus:
    expected_slot = expected_selected_slot(params)
    expected_device_id = None if expected_slot is None else _param_int(params, f"CAL_MAG{expected_slot}_ID")
    warnings: list[str] = []
    if params.get("UAVCAN_ENABLE") != 2:
        warnings.append("DroneCAN is not configured for sensor auto config.")
    if params.get("UAVCAN_SUB_MAG") != 1:
        warnings.append("DroneCAN magnetometer subscription is disabled.")

    registered_rm3100 = [
        _param_int(params, f"CAL_MAG{slot}_ID")
        for slot in MAG_SLOTS
        if _param_int(params, f"CAL_MAG{slot}_ID") in RM3100_IDS
    ]
    if not registered_rm3100:
        warnings.append("No RM3100 registered in CAL_MAG. Run Compass Calibration with RM3100 connected.")
    elif any(device_id in RM3100_PRIMARY_IDS for device_id in registered_rm3100) and not any(
        device_id in RM3100_BACKUP_IDS for device_id in registered_rm3100
    ):
        warnings.append("Second RM3100 is not registered in CAL_MAG. Run Compass Calibration again with both RM3100 connected.")

    if actual_selected_device_id == BOARD_INTERNAL_MAG_ID:
        warnings.append("PX4 is still using the board internal magnetometer, not RM3100.")
    elif actual_selected_device_id in GPS_MAG_IDS:
        warnings.append("PX4 is still using a GPS integrated magnetometer, not RM3100.")
    elif actual_selected_device_id in RM3100_IDS:
        warnings.append("PX4 is using RM3100 as selected magnetometer.")
    elif actual_selected_device_id is None:
        warnings.append("Actual selected MAG ID is not available via the current MAVLink telemetry path.")

    slots: list[MagSlotStatus] = []
    for slot in MAG_SLOTS:
        device_id = _param_int(params, f"CAL_MAG{slot}_ID")
        priority = _param_int(params, f"CAL_MAG{slot}_PRIO")
        rotation = _param_int(params, f"CAL_MAG{slot}_ROT")
        info = identify_mag_device(device_id)
        enabled = priority is not None and priority > 0
        is_expected = slot == expected_slot
        is_actual = actual_selected_device_id is not None and device_id == actual_selected_device_id
        if device_id is None:
            status = "N/A"
        elif is_actual:
            status = "Actual selected"
        elif is_expected:
            status = "Expected selected"
        elif not enabled:
            status = "Disabled"
        elif info.is_rm3100:
            status = "RM3100 candidate"
        else:
            status = "Enabled non-RM3100"
        slots.append(
            MagSlotStatus(
                slot=slot,
                device_id=device_id,
                priority=priority,
                rotation=rotation,
                enabled=enabled,
                expected_selected=is_expected,
                actual_selected=is_actual,
                device_type=info.device_type,
                interface=info.interface,
                is_rm3100=info.is_rm3100,
                status=status,
            )
        )

    return Px4MagStatus(
        params=params,
        slots=tuple(slots),
        rm3100_primary_id=RM3100_PRIMARY_ID,
        rm3100_backup_id=RM3100_BACKUP_ID,
        expected_selected_device_id=expected_device_id,
        actual_selected_device_id=actual_selected_device_id,
        is_actual_rm3100=actual_selected_device_id in RM3100_IDS,
        warning_text="\n".join(warnings),
        armed=armed,
    )


class Px4MagManager:
    def __init__(self, store: ParameterStore) -> None:
        self.store = store
        self.actual_selected_device_id: int | None = None
        self.armed = False
        self.last_change_log: list[str] = []

    def refresh(self) -> Px4MagStatus:
        return build_mag_status(
            read_mag_params(self.store),
            actual_selected_device_id=self.actual_selected_device_id,
            armed=self.armed,
        )

    def use_rm3100_for_yaw(self) -> Px4MagStatus:
        if self.armed:
            raise RuntimeError("Disarm required before changing PX4 magnetometer parameters.")
        params = read_mag_params(self.store)
        rm_slots = [
            slot
            for slot in MAG_SLOTS
            if _param_int(params, f"CAL_MAG{slot}_ID") in RM3100_IDS and params.get(f"CAL_MAG{slot}_PRIO") is not None
        ]
        if not rm_slots:
            raise RuntimeError("No RM3100 registered in CAL_MAG. Run Compass Calibration with RM3100 connected.")
        before = {f"CAL_MAG{slot}_PRIO": params.get(f"CAL_MAG{slot}_PRIO") for slot in MAG_SLOTS}
        target_priorities = {slot: 0 for slot in MAG_SLOTS}
        target_priorities[rm_slots[0]] = 100
        if len(rm_slots) >= 2:
            target_priorities[rm_slots[1]] = 90
        for slot in MAG_SLOTS:
            name = f"CAL_MAG{slot}_PRIO"
            if params.get(name) is None:
                continue
            self.store.set_param(name, target_priorities[slot])
        after = {f"CAL_MAG{slot}_PRIO": self.store.get_param(f"CAL_MAG{slot}_PRIO") for slot in MAG_SLOTS}
        self.last_change_log = [
            "Use RM3100 for Yaw",
            f"before={before}",
            f"after={after}",
            "GPS1/GPS2 integrated compass priority is set to 0.",
            "Run param save and reboot PX4 if PX4 does not persist changes automatically.",
        ]
        return self.refresh()
