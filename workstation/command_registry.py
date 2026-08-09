from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Callable, Any

from PyQt5 import QtCore, QtWidgets


@dataclass(frozen=True)
class CommandSpec:
    command_id: str
    text: str
    tooltip: str
    category: str
    handler: Callable[[], Any]
    shortcut: str = ""
    checkable: bool = False
    deferred_completion: bool = False


class CommandRegistry(QtCore.QObject):
    """One QAction path shared by menu, toolbar, and contextual buttons."""

    invoked = QtCore.pyqtSignal(str)
    accepted = QtCore.pyqtSignal(str, object)
    completed = QtCore.pyqtSignal(str, object)
    failed = QtCore.pyqtSignal(str, str)

    def __init__(self, parent: QtCore.QObject | None = None) -> None:
        super().__init__(parent)
        self._specs: dict[str, CommandSpec] = {}
        self._actions: dict[str, QtWidgets.QAction] = {}

    def register(self, spec: CommandSpec) -> QtWidgets.QAction:
        if spec.command_id in self._specs:
            raise ValueError(f"duplicate command id: {spec.command_id}")
        action = QtWidgets.QAction(spec.text, self)
        action.setToolTip(spec.tooltip)
        action.setStatusTip(spec.tooltip)
        action.setCheckable(spec.checkable)
        if spec.shortcut:
            action.setShortcut(spec.shortcut)
        action.triggered.connect(lambda _checked=False, command_id=spec.command_id: self.invoke(command_id))
        self._specs[spec.command_id] = spec
        self._actions[spec.command_id] = action
        return action

    def action(self, command_id: str) -> QtWidgets.QAction:
        return self._actions[command_id]

    def set_enabled(self, command_id: str, enabled: bool) -> None:
        action = self._actions.get(command_id)
        if action is not None:
            action.setEnabled(bool(enabled))

    def set_text(self, command_id: str, text: str) -> None:
        action = self._actions.get(command_id)
        if action is not None:
            action.setText(str(text))

    def set_deferred_completion(self, command_id: str, deferred: bool = True) -> None:
        spec = self._specs.get(command_id)
        if spec is not None:
            self._specs[command_id] = replace(spec, deferred_completion=bool(deferred))

    def invoke(self, command_id: str) -> bool:
        spec = self._specs.get(command_id)
        if spec is None:
            self.failed.emit(command_id, "unknown command")
            return False
        self.invoked.emit(command_id)
        try:
            result = spec.handler()
        except Exception as exc:  # pragma: no cover - defensive UI boundary.
            self.failed.emit(command_id, str(exc))
            return False
        if result is False:
            self.failed.emit(command_id, "handler rejected command")
            return False
        # Accepted means the existing owner returned without an exception;
        # runtime status remains the postcondition evidence for asynchronous
        # camera, telemetry, detection, recording, and save transitions.
        self.accepted.emit(command_id, result)
        if not spec.deferred_completion:
            self.completed.emit(command_id, result)
        return True
