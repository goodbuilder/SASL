from __future__ import annotations

from PyQt5 import QtWidgets


def _transport_values(combo: QtWidgets.QComboBox) -> list[str]:
    return [str(combo.itemData(index)) for index in range(combo.count())]


def test_pixhawk_board_transport_offers_usb_and_writes_the_usb_endpoint() -> None:
    from Workstation.main_window import WorkstationMainWindow
    from Workstation.tests.workstation_test_support import _FakeBackend

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    backend = _FakeBackend()
    applied: list[tuple[str, str, int]] = []
    backend.set_pixhawk_transport = lambda role, transport: applied.append(
        ("transport", role, transport)
    )
    backend.configure_pixhawk_usb = lambda role, device, baud: applied.append(
        ("usb", role, device, baud)
    )
    backend.configure_pixhawk_radio = lambda role, device, baud: applied.append(
        ("radio", role, device, baud)
    )
    window = WorkstationMainWindow(backend=backend)
    window.navigation.select_context("devices.pixhawk")
    app.processEvents()

    pixhawk = window.pixhawk
    for side in ("left", "right"):
        assert _transport_values(pixhawk.transport_combos[side]) == [
            "radio",
            "ethernet",
            "usb",
        ]

    combo = pixhawk.transport_combos["left"]
    combo.setCurrentIndex(combo.findData("usb"))
    app.processEvents()
    assert ("transport", "left", "usb") in applied
    # USB is a serial transport, so the device/baud row stays editable.
    assert pixhawk.radio_device_edits["left"].isEnabled()
    assert pixhawk.radio_baud_spins["left"].isEnabled()

    pixhawk.radio_device_edits["left"].setText("/dev/ttyACM0")
    pixhawk.radio_baud_spins["left"].setValue(115_200)
    pixhawk._apply_radio_endpoint(
        "left",
        pixhawk.radio_device_edits["left"],
        pixhawk.radio_baud_spins["left"],
    )
    assert ("usb", "left", "/dev/ttyACM0", 115_200) in applied
    assert not any(entry[0] == "radio" for entry in applied)

    # Ethernet has no serial endpoint, so the same row goes read-only.
    combo.setCurrentIndex(combo.findData("ethernet"))
    app.processEvents()
    assert not pixhawk.radio_device_edits["left"].isEnabled()

    window.close()
    app.processEvents()
    window.deleteLater()
    app.processEvents()


def test_telemetry_owns_the_drone_transport_selector_including_usb() -> None:
    from Workstation.main_window import WorkstationMainWindow
    from Workstation.tests.workstation_test_support import _FakeBackend

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    backend = _FakeBackend()
    window = WorkstationMainWindow(backend=backend)
    window.navigation.select_context("devices.telemetry")
    app.processEvents()

    telemetry = window.telemetry
    assert _transport_values(telemetry.transport_combo) == ["radio", "ethernet", "usb"]

    telemetry.transport_combo.setCurrentIndex(telemetry.transport_combo.findData("usb"))
    app.processEvents()
    telemetry.serial_device_edit.setText("/dev/ttyACM2")
    payload = telemetry.payload()
    assert payload["drone_pixhawk_transport"] == "usb"
    assert payload["drone_pixhawk_serial_device"] == "/dev/ttyACM2"
    # A serial transport edits the device path, not the MAVLink URL.
    assert telemetry.serial_device_edit.isEnabled()
    assert not telemetry.endpoint_edit.isEnabled()

    telemetry.transport_combo.setCurrentIndex(
        telemetry.transport_combo.findData("ethernet")
    )
    app.processEvents()
    assert telemetry.endpoint_edit.isEnabled()
    assert not telemetry.serial_device_edit.isEnabled()
    assert telemetry.payload()["drone_pixhawk_transport"] == "ethernet"

    window.close()
    app.processEvents()
    window.deleteLater()
    app.processEvents()
