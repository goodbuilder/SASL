from __future__ import annotations

from PyQt5 import QtCore, QtGui, QtWidgets


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
        root = self._item("StereoWIRIS", "root", selectable=False)
        self.model.appendRow(root)
        root.appendRow(self._item("Overview", "overview"))

        devices = self._item("Devices", "devices", selectable=False)
        cameras = self._item("Cameras", "devices.cameras", selectable=True)
        cameras.appendRow(self._item("Left Camera", "devices.cameras.left"))
        cameras.appendRow(self._item("Right Camera", "devices.cameras.right"))
        cameras.appendRow(self._item("Stereo Pair", "devices.cameras.stereo"))
        devices.appendRow(cameras)
        devices.appendRow(self._item("Time / Sync", "devices.time"))
        pixhawk = self._item("Pixhawk", "devices.pixhawk", selectable=True)
        pixhawk.appendRow(self._item("Pixhawk 1", "devices.pixhawk.p1"))
        pixhawk.appendRow(self._item("Pixhawk 2", "devices.pixhawk.p2"))
        devices.appendRow(pixhawk)
        devices.appendRow(self._item("GNSS / RTK", "devices.gnss"))
        root.appendRow(devices)

        processing = self._item("Processing", "processing", selectable=False)
        processing.appendRow(self._item("Detection", "processing.detection"))
        processing.appendRow(self._item("Measurement", "processing.measurement"))
        processing.appendRow(self._item("Synchronization", "processing.synchronization"))
        processing.appendRow(self._item("Alignment", "processing.alignment"))
        root.appendRow(processing)

        tasks = self._item("Tasks", "tasks", selectable=False)
        tasks.appendRow(self._item("Calibration", "tasks.calibration"))
        tasks.appendRow(self._item("Recording", "tasks.recording"))
        tasks.appendRow(self._item("Session Review", "tasks.playback"))
        tasks.appendRow(self._item("Artifacts", "tasks.export"))
        tasks.appendRow(self._item("Advanced owner tools", "tasks.advanced"))
        tasks.appendRow(self._item("All settings", "tasks.settings"))
        root.appendRow(tasks)

        diagnostics = self._item("Diagnostics", "diagnostics", selectable=False)
        diagnostics.appendRow(self._item("System", "diagnostics.system"))
        diagnostics.appendRow(self._item("Camera", "diagnostics.camera"))
        diagnostics.appendRow(self._item("Pixhawk / MAVLink", "diagnostics.pixhawk"))
        diagnostics.appendRow(self._item("GNSS / RTK", "diagnostics.gnss"))
        diagnostics.appendRow(self._item("Logs", "diagnostics.logs"))
        diagnostics.appendRow(self._item("Measurement", "diagnostics.measurement"))
        diagnostics.appendRow(self._item("Performance", "diagnostics.performance"))
        root.appendRow(diagnostics)
        self.expand(root.index())
        self.expand(cameras.index())
        self.expand(pixhawk.index())
        self.expand(processing.index())
        self.expand(tasks.index())
        self.expand(diagnostics.index())
        self.setCurrentIndex(root.child(0).index())

    def _on_current_changed(self, current: QtCore.QModelIndex, _previous: QtCore.QModelIndex) -> None:
        context_id = current.data(QtCore.Qt.UserRole)
        if context_id:
            self.context_changed.emit(str(context_id))

    def select_context(self, context_id: str) -> bool:
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
