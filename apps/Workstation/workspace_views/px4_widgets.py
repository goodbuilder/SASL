"""PX4 priority controls shared by device and advanced workspaces."""

from __future__ import annotations

from PyQt5 import QtWidgets
from Workstation.workstation_core.px4_mag_manager import MAG_PRIORITY_LEVELS, mag_priority_display


def _set_px4_priority_combo_value(combo: QtWidgets.QComboBox, value: int | float | None) -> None:
    if value is None:
        return
    raw = int(value)
    index = combo.findData(raw)
    if index < 0:
        combo.addItem(f"Custom ({raw})", raw)
        index = combo.count() - 1
    combo.setCurrentIndex(index)
    combo.setToolTip(f"PX4 priority: {mag_priority_display(raw)}")


def _build_px4_priority_combo(value: int) -> QtWidgets.QComboBox:
    combo = QtWidgets.QComboBox()
    for raw, label in MAG_PRIORITY_LEVELS:
        combo.addItem(label, raw)
    _set_px4_priority_combo_value(combo, value)
    combo.currentIndexChanged.connect(
        lambda _index, combo=combo: combo.setToolTip(
            f"PX4 priority: {mag_priority_display(combo.currentData())}"
        )
    )
    return combo


def _px4_priority_combo_value(combo: QtWidgets.QComboBox) -> int:
    value = combo.currentData()
    return 0 if value is None else int(value)
