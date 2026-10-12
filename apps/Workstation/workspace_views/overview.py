"""Overview of runtime state and public measurement frame evidence."""

from __future__ import annotations

from collections.abc import Mapping
from PyQt5 import QtCore, QtWidgets
from Workstation.workstation_core.coordinate_contract import MEASUREMENT_AXIS_CONTRACT, MEASUREMENT_FRAME
from Workstation.workstation_core.measurement_methods import MEASUREMENT_METHOD_IDS, MEASUREMENT_METHOD_LABELS
from Workstation.ui.components import EngineeringValue, StatusIndicator, WorkspaceHeader
from Workstation.application.status_model import RuntimeStatus, StatusModel
from .shared import _compact_text


class OverviewWorkspace(QtWidgets.QWidget):
    open_context = QtCore.pyqtSignal(str)

    def __init__(self, backend: object, status_model: StatusModel, parent: QtWidgets.QWidget | None = None) -> None:
        super().__init__(parent)
        self.backend = backend
        self.status_model = status_model
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)
        layout.addWidget(WorkspaceHeader("Overview", "Connect → verify → measure → auto-save"))

        status_group = QtWidgets.QGroupBox("Status")
        status_layout = QtWidgets.QGridLayout(status_group)
        status_layout.setContentsMargins(6, 6, 6, 6)
        status_layout.setSpacing(5)
        self.indicators: dict[str, StatusIndicator] = {}
        for index, (key, label) in enumerate(
            (
                ("camera.left", "CAM L"),
                ("camera.right", "CAM R"),
                ("pixhawk.left", "P1"),
                ("pixhawk.right", "P2"),
                ("gnss.base", "BASE"),
                ("detection", "DET"),
                ("measurement", "MEAS"),
                ("recording", "REC"),
            )
        ):
            indicator = StatusIndicator(label)
            self.indicators[key] = indicator
            status_layout.addWidget(indicator, index // 2, index % 2)
        layout.addWidget(status_group)

        evidence_group = QtWidgets.QGroupBox("Latest result")
        evidence_layout = QtWidgets.QGridLayout(evidence_group)
        evidence_layout.setContentsMargins(8, 8, 8, 8)
        self.session_value = EngineeringValue("Storage")
        self.measurement_value = EngineeringValue("Result")
        self.contract_value = EngineeringValue("Output")
        evidence_layout.addWidget(self.session_value, 0, 0)
        evidence_layout.addWidget(self.measurement_value, 1, 0)
        evidence_layout.addWidget(self.contract_value, 2, 0)
        evidence_layout.setColumnStretch(0, 1)
        layout.addWidget(evidence_group)

        methods_group = QtWidgets.QGroupBox("Measurement methods · read-only")
        methods_layout = QtWidgets.QGridLayout(methods_group)
        methods_layout.setContentsMargins(6, 6, 6, 6)
        methods_layout.setSpacing(5)
        self.measurement_method_cards: dict[str, QtWidgets.QPushButton] = {}
        for index, method_id in enumerate(MEASUREMENT_METHOD_IDS):
            card = QtWidgets.QPushButton(
                f"{MEASUREMENT_METHOD_LABELS[method_id]}\nBLOCKED · V0 / B0 / F0"
            )
            card.setObjectName(f"overviewMethodCard_{method_id}")
            card.setProperty("navigationOnly", True)
            card.setMinimumHeight(54)
            card.setStyleSheet("text-align:left; padding:6px;")
            card.clicked.connect(
                lambda _checked=False: self.open_context.emit(
                    "processing.measurement"
                )
            )
            self.measurement_method_cards[method_id] = card
            methods_layout.addWidget(card, index // 2, index % 2)
        layout.addWidget(methods_group)

        actions = QtWidgets.QHBoxLayout()
        camera_button = QtWidgets.QPushButton("Connect cameras")
        camera_button.clicked.connect(lambda: self.open_context.emit("devices.cameras"))
        measurement_button = QtWidgets.QPushButton("Measure")
        measurement_button.clicked.connect(lambda: self.open_context.emit("processing.measurement"))
        diagnostics_button = QtWidgets.QPushButton("Diagnostics")
        diagnostics_button.clicked.connect(lambda: self.open_context.emit("diagnostics.system"))
        actions.addWidget(camera_button)
        actions.addWidget(measurement_button)
        actions.addWidget(diagnostics_button)
        actions.addStretch(1)
        layout.addLayout(actions)

        layout.addStretch(1)

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
