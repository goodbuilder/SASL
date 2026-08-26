from __future__ import annotations

from PyQt5 import QtCore, QtGui, QtWidgets


_CONTEXT_ALIASES = {
    # Keep saved UI state and external callers compatible while the visible
    # tree moves to paired contexts and a single Data surface.
    "devices.cameras.left": "devices.cameras",
    "devices.cameras.right": "devices.cameras",
    "devices.cameras.stereo": "devices.cameras",
    "devices.pixhawk.p1": "devices.pixhawk",
    "devices.pixhawk.p2": "devices.pixhawk",
    "processing.detection": "devices.cameras",
    "processing.synchronization": "devices.time",
    # Legacy session/recording/review contexts remain valid deep links into
    # the sessionless Data workspace; they no longer expose a lifecycle.
    "tasks.session": "tasks.data",
    "tasks.session_details": "tasks.data",
    "tasks.recording": "tasks.data",
    "tasks.playback": "tasks.data",
    "tasks.export": "tasks.data",
}


def canonical_context_id(context_id: str) -> str:
    """Return the visible navigation context for a legacy/old saved ID."""

    current = str(context_id or "")
    seen: set[str] = set()
    while current in _CONTEXT_ALIASES and current not in seen:
        seen.add(current)
        current = _CONTEXT_ALIASES[current]
    return current


class NavigationTree(QtWidgets.QTreeView):
    context_changed = QtCore.pyqtSignal(str)

    def __init__(self, parent: QtWidgets.QWidget | None = None) -> None:
        super().__init__(parent)
        self.setHeaderHidden(True)
        self.setUniformRowHeights(True)
        self.setMinimumWidth(180)
        self.setIndentation(16)
        self.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.model = QtGui.QStandardItemModel(self)
        self.setModel(self.model)
        self.selectionModel().currentChanged.connect(self._on_current_changed)
        self._build_model()

    def _item(self, text: str, context_id: str, *, selectable: bool = True) -> QtGui.QStandardItem:
        item = QtGui.QStandardItem(text)
        item.setData(context_id, QtCore.Qt.UserRole)
        item.setEditable(False)
        if not selectable:
            item.setFlags(item.flags() & ~QtCore.Qt.ItemIsSelectable)
        return item

    def _build_model(self) -> None:
        root = self._item("Workstation", "root", selectable=False)
        self.model.appendRow(root)
        root.appendRow(self._item("Overview", "overview"))
        root.appendRow(self._item("Protocol", "processing.experiment"))

        devices = self._item("Devices", "devices", selectable=False)
        devices.appendRow(self._item("Cameras", "devices.cameras"))
        devices.appendRow(self._item("Pixhawk", "devices.pixhawk"))
        devices.appendRow(self._item("Telemetry", "devices.telemetry"))
        devices.appendRow(self._item("Base GNSS", "devices.gnss"))
        devices.appendRow(self._item("Time / Sync", "devices.time"))
        root.appendRow(devices)

        root.appendRow(self._item("Data", "tasks.data"))
        root.appendRow(self._item("Calibration", "tasks.calibration"))
        root.appendRow(self._item("Alignment", "processing.alignment"))
        root.appendRow(self._item("Measurement", "processing.measurement"))
        root.appendRow(self._item("Advanced", "tasks.advanced"))
        root.appendRow(self._item("Settings", "tasks.settings"))

        diagnostics = self._item("Diagnostics", "diagnostics", selectable=False)
        diagnostics.appendRow(self._item("System", "diagnostics.system"))
        diagnostics.appendRow(self._item("Camera", "diagnostics.camera"))
        diagnostics.appendRow(self._item("Pixhawk / MAVLink", "diagnostics.pixhawk"))
        diagnostics.appendRow(self._item("GNSS / RTK", "diagnostics.gnss"))
        diagnostics.appendRow(self._item("Logs", "diagnostics.logs"))
        diagnostics.appendRow(self._item("FTP", "diagnostics.ftp"))
        diagnostics.appendRow(self._item("Measurement", "diagnostics.measurement"))
        diagnostics.appendRow(self._item("Performance", "diagnostics.performance"))
        root.appendRow(diagnostics)
        self.expand(root.index())
        self.expand(diagnostics.index())
        self.setCurrentIndex(root.child(0).index())

    def _on_current_changed(self, current: QtCore.QModelIndex, _previous: QtCore.QModelIndex) -> None:
        context_id = current.data(QtCore.Qt.UserRole)
        if context_id:
            self.context_changed.emit(str(context_id))

    def select_context(self, context_id: str) -> bool:
        context_id = canonical_context_id(context_id)
        matches = self.model.match(
            self.model.index(0, 0),
            QtCore.Qt.UserRole,
            context_id,
            -1,
            QtCore.Qt.MatchRecursive,
        )
        if not matches:
            return False
        self.setCurrentIndex(matches[0])
        return True
