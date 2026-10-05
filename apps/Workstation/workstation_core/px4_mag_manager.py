from __future__ import annotations

import json
from dataclasses import dataclass
from typing import Mapping, Protocol


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
MAG_PRIORITY_KEYS = tuple(f"CAL_MAG{slot}" for slot in MAG_SLOTS)
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


def mag_priority_slot_key(slot: int) -> str:
    """Return the operator-facing key for one ``CAL_MAGn_PRIO`` value."""

    slot = int(slot)
    if slot not in MAG_SLOTS:
        raise ValueError(f"Unsupported PX4 CAL_MAG slot: {slot}")
    return f"CAL_MAG{slot}"


def mag_priority_slot_from_key(key: object) -> int | None:
    """Parse ``CAL_MAGn``/``CAL_MAGn_PRIO`` without accepting another slot."""

    text = str(key or "").strip().upper()
    if not text.startswith("CAL_MAG"):
        return None
    suffix = text[len("CAL_MAG") :]
    if suffix.endswith("_PRIO"):
        suffix = suffix[: -len("_PRIO")]
    if not suffix.isdigit():
        return None
    slot = int(suffix)
    return slot if slot in MAG_SLOTS else None


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
MAG_PRIORITY_PARAM_NAMES = (
    "UAVCAN_ENABLE",
    "UAVCAN_SUB_MAG",
    *(f"CAL_MAG{slot}_ID" for slot in MAG_SLOTS),
    *(f"CAL_MAG{slot}_PRIO" for slot in MAG_SLOTS),
)


def missing_mag_status_params(params: Mapping[str, object] | None) -> tuple[str, ...]:
    """Return the CAL_MAG PARAM names a slot table/corrected yaw still needs.

    A radio PARAM readback is a sequence of single-shot requests, so one lost
    reply leaves ``None`` in the middle of an otherwise valid status.  Every
    slot needs its ``_ID``/``_PRIO`` to be placed in the table at all; a slot
    that names a device also needs its rotation and correction coefficients
    before a CAL_MAG-corrected yaw can be shown for that row.
    """

    values = params if isinstance(params, Mapping) else {}
    missing: list[str] = []
    for slot in MAG_SLOTS:
        for suffix in ("ID", "PRIO"):
            name = f"CAL_MAG{slot}_{suffix}"
            if values.get(name) is None:
                missing.append(name)
        device_id = values.get(f"CAL_MAG{slot}_ID")
        try:
            configured = device_id is not None and int(device_id) != 0
        except (TypeError, ValueError):
            configured = False
        if not configured:
            continue
        for suffix in ("ROT", *MAG_CAL_CORRECTION_SUFFIXES):
            name = f"CAL_MAG{slot}_{suffix}"
            if values.get(name) is None:
                missing.append(name)
    return tuple(missing)


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
class MagSelectionCandidate:
    """One live ``sensor_mag`` instance considered by the stock selector preview."""

    instance: int
    device_id: int
    role: str
    priority: int
    cal_slot: int | None
    stock_selector_range: bool


@dataclass(frozen=True)
class MagSelectionPreview:
    """Read-only prediction; it is not evidence that PX4 actually selected the candidate."""

    instance_map: tuple[tuple[int, int], ...]
    candidates: tuple[MagSelectionCandidate, ...]
    predicted_instance: int | None
    predicted_device_id: int | None
    predicted_role: str | None
    predicted_priority: int | None
    actual_device_id: int | None
    actual_role: str | None
    out_of_range_device_ids: tuple[int, ...]
    priority_tie: bool


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


def mag_role_for_device_id(device_id: int | None) -> str | None:
    """Return the stable operator role for a known magnetometer identity."""

    if device_id == BOARD_INTERNAL_MAG_ID:
        return "BOARD"
    if device_id in RM3100_PRIMARY_IDS:
        return "CAN1"
    if device_id in RM3100_BACKUP_IDS:
        return "CAN2"
    if device_id == GPS1_MAG_ID:
        return "GPS1_MAG"
    if device_id == GPS2_MAG_ID:
        return "GPS2_MAG"
    return None


def _default_mag_priority(device_id: int) -> int:
    """PX4's default priority for an identity with no matching CAL_MAG slot."""

    return 50 if device_id == BOARD_INTERNAL_MAG_ID else 75


def build_mag_selection_preview(
    params: Mapping[str, int | float | None],
    instance_device_ids: Mapping[int, int | None],
    *,
    requested_priorities: Mapping[str, int] | None = None,
    actual_selected_device_id: int | None = None,
) -> MagSelectionPreview:
    """Predict the stock PX4 MAG candidate from live instances and priorities.

    This intentionally does not claim estimator success.  Stock PX4's
    ``VehicleMagnetometer`` consumes only instances 0..3; instance 4 remains
    visible in the topology map but is excluded from the candidate preview.
    Health/confidence tie-breaking is left to the actual active-device readback.
    """

    role_overrides: dict[str, int] = {}
    slot_overrides: dict[int, int] = {}
    for key, value in dict(requested_priorities or {}).items():
        slot = mag_priority_slot_from_key(key)
        if slot is not None:
            slot_overrides[slot] = int(value)
        else:
            # Keep the previous role-based contract for callers outside the
            # visible CAL_MAG UI. New UI payloads use CAL_MAGn keys.
            role_overrides[str(key)] = int(value)
    slot_by_device: dict[int, int] = {}
    priority_by_slot: dict[int, int] = {}
    for slot in MAG_SLOTS:
        device_id = _param_int(dict(params), f"CAL_MAG{slot}_ID")
        if device_id is None:
            continue
        slot_by_device[device_id] = slot
        priority_by_slot[slot] = _param_int(dict(params), f"CAL_MAG{slot}_PRIO") or 0

    normalized_instances = tuple(
        sorted(
            (int(instance), int(device_id))
            for instance, device_id in dict(instance_device_ids).items()
            if device_id is not None
        )
    )
    candidates: list[MagSelectionCandidate] = []
    for instance, device_id in normalized_instances:
        role = mag_role_for_device_id(device_id) or "UNKNOWN"
        slot = slot_by_device.get(device_id)
        if slot in slot_overrides:
            priority = int(slot_overrides[slot])
        elif role in role_overrides:
            priority = int(role_overrides[role])
        elif slot is not None:
            priority = int(priority_by_slot.get(slot, 0))
        else:
            priority = _default_mag_priority(device_id)
        candidates.append(
            MagSelectionCandidate(
                instance=instance,
                device_id=device_id,
                role=role,
                priority=priority,
                cal_slot=slot,
                stock_selector_range=instance < 4,
            )
        )

    eligible = [candidate for candidate in candidates if candidate.stock_selector_range and candidate.priority > 0]
    best_priority = max((candidate.priority for candidate in eligible), default=None)
    best = [candidate for candidate in eligible if candidate.priority == best_priority]
    selected = min(best, key=lambda candidate: candidate.instance) if best else None
    active_role = mag_role_for_device_id(actual_selected_device_id)
    return MagSelectionPreview(
        instance_map=normalized_instances,
        candidates=tuple(candidates),
        predicted_instance=None if selected is None else selected.instance,
        predicted_device_id=None if selected is None else selected.device_id,
        predicted_role=None if selected is None else selected.role,
        predicted_priority=None if selected is None else selected.priority,
        actual_device_id=actual_selected_device_id,
        actual_role=active_role,
        out_of_range_device_ids=tuple(
            candidate.device_id for candidate in candidates if not candidate.stock_selector_range
        ),
        priority_tie=len(best) > 1,
    )


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
