"""Shared presentation helpers; no workspace or device ownership."""

from __future__ import annotations

from typing import Callable
from PyQt5 import QtCore, QtGui, QtWidgets
from Workstation.application.evidence import EvidenceAdapter
from Workstation.application.status_model import RuntimeStatus, StatusModel


def _status(model: StatusModel, key: str) -> RuntimeStatus | None:
    return model.record(key)


def _evidence(backend: object) -> EvidenceAdapter:
    adapter = getattr(backend, "evidence", None)
    if isinstance(adapter, EvidenceAdapter):
        return adapter
    return EvidenceAdapter(getattr(backend, "panel", backend))


def _display_number(value: object, *, digits: int = 3, suffix: str = "") -> str:
    if value in (None, ""):
        return "--"
    try:
        return f"{float(value):.{digits}f}{suffix}"
    except (TypeError, ValueError):
        return f"{value}{suffix}"


def _compact_text(value: object, *, limit: int = 128) -> str:
    text = " ".join(str(value or "").split())
    if len(text) <= limit:
        return text
    return text[: max(1, limit - 1)].rstrip() + "…"


def _display_class(target: dict[str, object]) -> str:
    class_name = target.get("class_name")
    if class_name not in (None, ""):
        return str(class_name)
    class_id = target.get("class_id")
    return "--" if class_id is None else str(class_id)


def _table_model(parent: QtCore.QObject, headers: tuple[str, ...]) -> QtGui.QStandardItemModel:
    model = QtGui.QStandardItemModel(0, len(headers), parent)
    model.setHorizontalHeaderLabels(list(headers))
    return model


def _append_table_row(model: QtGui.QStandardItemModel, values: tuple[object, ...]) -> None:
    items = []
    for value in values:
        item = QtGui.QStandardItem(str(value))
        item.setEditable(False)
        items.append(item)
    model.appendRow(items)


def _sync_table_rows(
    model: QtGui.QStandardItemModel,
    rows: tuple[tuple[object, ...], ...],
) -> None:
    """Update table cells in place so polling does not discard selection."""

    if model.rowCount() != len(rows):
        model.setRowCount(len(rows))
    for row_index, values in enumerate(rows):
        for column_index, value in enumerate(values):
            text = str(value)
            item = model.item(row_index, column_index)
            if item is None:
                item = QtGui.QStandardItem(text)
                item.setEditable(False)
                model.setItem(row_index, column_index, item)
            elif item.text() != text:
                item.setText(text)


def _command_button(
    text: str,
    command_id: str,
    invoke_command: Callable[[str], bool] | None,
) -> QtWidgets.QPushButton:
    button = QtWidgets.QPushButton(text)
    button.setProperty("commandId", command_id)
    if invoke_command is None:
        button.setEnabled(False)
    else:
        button.clicked.connect(lambda _checked=False, command=command_id: invoke_command(command))
    return button


def _live_inspection(
    snapshot: dict[str, object],
    resolver: Callable[[], dict[str, object]],
) -> dict[str, object]:
    snapshot["_resolver"] = resolver
    return snapshot
