"""Widget registry for calibration command presentation, never domain admission."""
from PyQt5 import QtWidgets


class CalibrationCommandWidgets:
    def __init__(self):
        self.primary: dict[str, QtWidgets.QPushButton] = {}
        self.instances: dict[str, list[QtWidgets.QPushButton]] = {}

    def buttons(self, command_id):
        instances = tuple(self.instances.get(command_id, ()))
        primary = self.primary.get(command_id)
        return instances or (() if primary is None else (primary,))
