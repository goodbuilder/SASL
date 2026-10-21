"""Compose Designer slots with the existing Pixhawk display widgets."""
from __future__ import annotations

import json
from PyQt5 import QtWidgets

from Workstation.ui.components import EngineeringValue, StatusIndicator
from Workstation.ui.designer import load_form
from Workstation.ui.telemetry_plots import BaselineYawAlignmentPlot


def load_pixhawk_form(name: str) -> QtWidgets.QWidget:
    form = load_form(name, QtWidgets.QWidget())
    # These are presentation slots only; no runtime/backend is passed to them.
    for slot in form.findChildren(QtWidgets.QWidget):
        label = slot.property("engineeringLabel")
        kind = slot.property("customKind")
        value = None
        if label is not None:
            value = EngineeringValue(str(label), slot)
        elif kind == "StatusIndicator":
            value = StatusIndicator(str(slot.property("customLabel")), slot)
        elif kind == "BaselineYawAlignmentPlot":
            value = BaselineYawAlignmentPlot()
        if value is not None:
            slot.layout().addWidget(value)
            setattr(form, slot.objectName(), value)
        data = slot.property("comboData")
        if data is not None:
            for index, item in enumerate(json.loads(str(data))):
                slot.setItemData(index, item)
        legacy = slot.property("legacyObjectName")
        if legacy:
            slot.setObjectName(str(legacy))
    return form
