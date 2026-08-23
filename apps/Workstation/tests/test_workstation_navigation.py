from __future__ import annotations

from PyQt5 import QtCore, QtWidgets

from Workstation.navigation import NavigationTree


def test_navigation_exposes_one_canonical_sessions_workspace() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    tree = NavigationTree()
    root = tree.model.item(0, 0)

    assert [root.child(row).text() for row in range(root.rowCount())] == [
        "Overview",
        "Protocol",
        "Devices",
        "Sessions",
        "Calibration",
        "Alignment",
        "Measurement",
        "Advanced",
        "Settings",
        "Diagnostics",
    ]
    devices = root.child(2)
    assert [devices.child(row).text() for row in range(devices.rowCount())] == [
        "Cameras",
        "Pixhawk",
        "Telemetry",
        "Base GNSS",
        "Time / Sync",
    ]
    assert all(devices.child(row).rowCount() == 0 for row in range(devices.rowCount()))

    diagnostics = root.child(9)
    assert [diagnostics.child(row).text() for row in range(diagnostics.rowCount())] == [
        "System",
        "Camera",
        "Pixhawk / MAVLink",
        "GNSS / RTK",
        "Logs",
        "FTP",
        "Measurement",
        "Performance",
    ]

    for legacy_id, canonical_id in {
        "devices.cameras.left": "devices.cameras",
        "devices.cameras.stereo": "devices.cameras",
        "devices.pixhawk.p1": "devices.pixhawk",
        "processing.detection": "devices.cameras",
        "processing.synchronization": "devices.time",
        "tasks.session_details": "tasks.session",
        "tasks.recording": "tasks.session",
        "tasks.playback": "tasks.session",
        "tasks.export": "tasks.session",
        "processing.experiment": "processing.experiment",
    }.items():
        assert tree.select_context(legacy_id)
        assert tree.currentIndex().data(QtCore.Qt.UserRole) == canonical_id

    tree.deleteLater()
    app.processEvents()
