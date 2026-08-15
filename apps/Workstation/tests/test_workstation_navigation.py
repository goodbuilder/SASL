from __future__ import annotations

from PyQt5 import QtCore, QtWidgets

from Workstation.navigation import NavigationTree


def test_navigation_uses_paired_contexts_without_protocol_page() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    tree = NavigationTree()
    root = tree.model.item(0, 0)

    assert [root.child(row).text() for row in range(root.rowCount())] == [
        "Overview",
        "Devices",
        "Calibration",
        "Alignment",
        "Measurement",
        "Recording / Sessions",
        "Advanced",
        "Settings",
        "Diagnostics",
    ]
    devices = root.child(1)
    assert [devices.child(row).text() for row in range(devices.rowCount())] == [
        "Cameras",
        "Pixhawk",
        "Telemetry",
        "Base GNSS",
        "Time / Sync",
    ]
    assert all(devices.child(row).rowCount() == 0 for row in range(devices.rowCount()))

    for legacy_id, canonical_id in {
        "devices.cameras.left": "devices.cameras",
        "devices.cameras.stereo": "devices.cameras",
        "devices.pixhawk.p1": "devices.pixhawk",
        "processing.detection": "devices.cameras",
        "processing.synchronization": "devices.time",
        "tasks.playback": "tasks.recording",
        "tasks.export": "tasks.recording",
        "processing.experiment": "overview",
    }.items():
        assert tree.select_context(legacy_id)
        assert tree.currentIndex().data(QtCore.Qt.UserRole) == canonical_id

    tree.deleteLater()
    app.processEvents()
