"""Overview of runtime state and public measurement frame evidence."""

from __future__ import annotations

from collections.abc import Mapping
from Workstation.ui.designer import load_form
from PyQt5 import QtCore, QtWidgets
from Workstation.workstation_core.coordinate_contract import MEASUREMENT_AXIS_CONTRACT, MEASUREMENT_FRAME
from Workstation.workstation_core.measurement_methods import MEASUREMENT_METHOD_IDS, MEASUREMENT_METHOD_LABELS
from Workstation.ui.components import EngineeringValue, StatusIndicator
from Workstation.application.status_model import RuntimeStatus, StatusModel
from .shared import _compact_text


class OverviewWorkspace(QtWidgets.QWidget):
    open_context = QtCore.pyqtSignal(str)

    def __init__(self, backend: object, status_model: StatusModel, parent: QtWidgets.QWidget | None = None) -> None:
        super().__init__(parent)
        self.backend = backend
        self.status_model = status_model
        load_form("overview.ui", self)
        for widget in self.findChildren(QtWidgets.QWidget):
            if widget.property("styleObjectName"):
                widget.setObjectName(widget.property("styleObjectName"))
        self.indicators: dict[str, StatusIndicator] = {}
        for key in ("camera.left", "camera.right", "pixhawk.left", "pixhawk.right",
                    "gnss.base", "detection", "measurement", "recording"):
            slot = getattr(self, "indicators_" + key.replace(".", "_") + "Slot")
            indicator = StatusIndicator(slot.property("runtimeLabel"))
            slot.layout().addWidget(indicator)
            self.indicators[key] = indicator
        for name in ("session_value", "measurement_value", "contract_value"):
            slot = getattr(self, name + "Slot")
            value = EngineeringValue(slot.property("runtimeLabel"))
            slot.layout().addWidget(value)
            setattr(self, name, value)
        self.measurement_method_cards = {
            method_id: getattr(self, "measurement_method_cards_" + method_id)
            for method_id in MEASUREMENT_METHOD_IDS
        }
        for card in self.measurement_method_cards.values():
            card.clicked.connect(
                lambda _checked=False: self.open_context.emit("processing.measurement")
            )
        for button in self.findChildren(QtWidgets.QPushButton):
            context = button.property("contextId")
            if context:
                button.clicked.connect(
                    lambda _checked=False, target=context: self.open_context.emit(target)
                )

    def refresh(self, snapshot: dict[str, RuntimeStatus]) -> None:
        for key, indicator in self.indicators.items():
            indicator.set_status(snapshot.get(key))
        panel = getattr(self.backend, "panel", self.backend)
        storage_reader = getattr(panel, "data_storage_snapshot", None)
        try:
            storage = storage_reader() if callable(storage_reader) else {}
        except Exception:
            storage = {}
        storage_state = str(storage.get("state") or "UNAVAILABLE")
        self.session_value.set_value(
            storage_state,
            source=str(storage.get("root") or "measurement_data_root"),
            valid=True if storage_state == "OK" else False if storage_state == "UNAVAILABLE" else None,
        )
        measurement_status = snapshot.get("measurement")
        measurement_is_current_valid = bool(
            measurement_status and measurement_status.state == "VALID"
        )
        self.measurement_value.set_value(
            str(getattr(panel, "last_distance_status", "Waiting for measurement") or ""),
            source="MeasurementAttempt/controller result",
            frame=MEASUREMENT_FRAME,
            age_s=measurement_status.age_s if measurement_status else None,
            valid=(
                True
                if measurement_is_current_valid
                else False
                if measurement_status
                and measurement_status.state in {"ERROR", "REJECTED"}
                else None
            ),
        )
        self.contract_value.set_value(
            f"{MEASUREMENT_AXIS_CONTRACT} · metres",
            frame=MEASUREMENT_FRAME,
            source="coordinate_contract.py",
        )
        reader = getattr(self.backend, "measurement_methods_snapshot", None)
        try:
            method_snapshot = reader() if callable(reader) else {}
        except Exception:
            method_snapshot = {}
        catalog = method_snapshot.get("method_catalog") if isinstance(method_snapshot, Mapping) else None
        catalog_rows = {
            str(row.get("method_id") or ""): dict(row)
            for row in catalog or ()
            if isinstance(row, Mapping)
        }
        for method_id, card in self.measurement_method_cards.items():
            row = catalog_rows.get(method_id, {})
            state = str(row.get("runtime_status") or "BLOCKED")
            counters = row.get("result_counters")
            counters = counters if isinstance(counters, Mapping) else {}
            blocker = str(row.get("primary_blocker") or "")
            extra = int(row.get("additional_blocker_count") or 0)
            blocker_text = blocker + (f" · +{extra} more" if extra else "")
            primary = "PRIMARY · " if method_id == "pixhawk_rtk_live" else ""
            card.setText(
                f"{primary}{MEASUREMENT_METHOD_LABELS[method_id]}\n"
                f"{state} · V{int(counters.get('VALID', 0) or 0)} / "
                f"B{int(counters.get('BLOCKED', 0) or 0)} / "
                f"F{int(counters.get('FAILED', 0) or 0)}"
                + (f"\n{_compact_text(blocker_text, limit=72)}" if blocker_text else "")
            )
            reasons = row.get("runtime_reasons")
            reasons = (
                [str(reason) for reason in reasons if str(reason)]
                if isinstance(reasons, (list, tuple))
                else ([blocker] if blocker else [])
            )
            required_inputs = row.get("required_inputs")
            required_inputs = (
                [str(item) for item in required_inputs if str(item)]
                if isinstance(required_inputs, (list, tuple))
                else []
            )
            tooltip_lines = [f"{MEASUREMENT_METHOD_LABELS[method_id]} · {state}"]
            if reasons:
                tooltip_lines.append("Reasons:")
                tooltip_lines.extend(f"  - {reason}" for reason in reasons)
            else:
                tooltip_lines.append("Reasons: none (ready)")
            if required_inputs:
                tooltip_lines.append("Required inputs:")
                tooltip_lines.extend(f"  - {item}" for item in required_inputs)
            tooltip_lines.append("Click to open read-only Measurement details.")
            card.setToolTip("\n".join(tooltip_lines))
