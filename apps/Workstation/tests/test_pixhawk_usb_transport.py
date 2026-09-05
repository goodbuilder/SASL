from __future__ import annotations

from types import SimpleNamespace

import pytest

from Workstation.device_runtime_controller import DeviceRuntimeController
from Workstation.workstation_core.pixhawk_attitude import (
    _is_serial_transport,
    _normalize_transport,
)
from Workstation.workstation_core.settings import (
    PIXHAWK_SERIAL_TRANSPORTS,
    PIXHAWK_TRANSPORT_USB,
    PIXHAWK_TRANSPORTS,
    normalize_pixhawk_transport,
)


class _Edit:
    def __init__(self, value: str = "") -> None:
        self.value = str(value)

    def text(self) -> str:
        return self.value

    def setText(self, value: str) -> None:
        self.value = str(value)


class _Spin:
    def __init__(self, value: int = 57_600) -> None:
        self._value = int(value)

    def value(self) -> int:
        return self._value

    def setValue(self, value: int) -> None:
        self._value = int(value)


def _controller(tmp_path):
    logs: list[str] = []
    saves: list[int] = []
    panel = SimpleNamespace(
        left_pixhawk_transport="usb",
        right_pixhawk_transport="radio",
        drone_pixhawk_transport="ethernet",
        left_pixhawk_radio_device="/dev/d30",
        right_pixhawk_radio_device="/dev/du",
        left_pixhawk_radio_baud=57_600,
        right_pixhawk_radio_baud=57_600,
        left_pixhawk_bootstrap_url_edit=_Edit("/dev/ttyACM0"),
        right_pixhawk_bootstrap_url_edit=_Edit(""),
        drone_pixhawk_bootstrap_url_edit=_Edit(""),
        left_pixhawk_bootstrap_baud_spin=_Spin(115_200),
        right_pixhawk_bootstrap_baud_spin=_Spin(57_600),
        drone_pixhawk_bootstrap_baud_spin=_Spin(57_600),
        drone_telemetry_url="udpin:0.0.0.0:14552",
        drone_telemetry_baud=57_600,
        mavlink_pixhawk=None,
        _mavlink_open_sides=lambda: (),
        _log=logs.append,
        _save_current_settings=lambda: saves.append(1),
        _px4_mag_camera_label=lambda side: side,
    )
    controller = DeviceRuntimeController(panel)
    return controller, panel, logs, saves


def test_usb_is_a_persisted_transport_that_counts_as_serial() -> None:
    assert PIXHAWK_TRANSPORT_USB in PIXHAWK_TRANSPORTS
    assert PIXHAWK_SERIAL_TRANSPORTS == ("radio", "usb")
    assert normalize_pixhawk_transport("usb") == "usb"
    assert normalize_pixhawk_transport("USB") == "usb"
    # An unknown value still falls back to the explicit Radio default.
    assert normalize_pixhawk_transport("modem") == "radio"

    assert _normalize_transport("usb") == "usb"
    assert _is_serial_transport("usb")
    assert _is_serial_transport("radio")
    assert not _is_serial_transport("ethernet")


def test_usb_transport_resolves_the_bootstrap_serial_endpoint(tmp_path) -> None:
    controller, _panel, _logs, _saves = _controller(tmp_path)

    assert controller._pixhawk_selected_transport("left") == "usb"
    assert controller._pixhawk_transport_is_serial("left")
    assert controller._pixhawk_selected_endpoint("left") == "/dev/ttyACM0"
    assert controller._pixhawk_selected_baud("left") == 115_200

    # Radio keeps its own saved endpoint; the two never overwrite each other.
    assert controller._pixhawk_selected_endpoint("right") == "/dev/du"
    assert controller._pixhawk_selected_baud("right") == 57_600
    # Ethernet still resolves through the Drone Telemetry URL.
    assert not controller._pixhawk_transport_is_serial("drone")
    assert controller._pixhawk_selected_endpoint("drone") == "udpin:0.0.0.0:14552"


def test_set_pixhawk_transport_accepts_usb_and_rejects_unknown(tmp_path) -> None:
    controller, panel, _logs, _saves = _controller(tmp_path)

    assert controller.set_pixhawk_transport("right", "usb") is True
    assert panel.right_pixhawk_transport == "usb"
    # Re-selecting the current transport is not a change.
    assert controller.set_pixhawk_transport("right", "usb") is False

    with pytest.raises(ValueError):
        controller.set_pixhawk_transport("left", "modem")


def test_configure_pixhawk_usb_writes_bootstrap_without_touching_radio(tmp_path) -> None:
    controller, panel, _logs, saves = _controller(tmp_path)

    assert controller.configure_pixhawk_usb("left", "/dev/ttyACM3", 921_600) is True
    assert panel.left_pixhawk_bootstrap_url_edit.text() == "/dev/ttyACM3"
    assert panel.left_pixhawk_bootstrap_baud_spin.value() == 921_600
    # The Radio endpoint for the same board is untouched.
    assert panel.left_pixhawk_radio_device == "/dev/d30"
    assert panel.left_pixhawk_radio_baud == 57_600
    assert saves

    # An unchanged write is a no-op rather than a redundant save.
    assert controller.configure_pixhawk_usb("left", "/dev/ttyACM3", 921_600) is False

    with pytest.raises(ValueError):
        controller.configure_pixhawk_usb("left", "/dev/ttyACM3", 0)
    with pytest.raises(ValueError):
        controller.configure_pixhawk_usb("nose", "/dev/ttyACM3", 57_600)


def test_usb_preflight_reports_usb_identity_and_missing_device(tmp_path) -> None:
    controller, panel, _logs, _saves = _controller(tmp_path)

    missing = controller._pixhawk_serial_preflight("left")
    assert missing.ok is False
    assert missing.state == "USB DEVICE MISSING"

    device = tmp_path / "ttyACM9"
    device.write_text("", encoding="utf-8")
    panel.left_pixhawk_bootstrap_url_edit.setText(str(device))
    ready = controller._pixhawk_serial_preflight("left")
    assert ready.ok is True
    assert ready.state == "USB READY"

    # The same check keeps its Radio wording for a Radio board.
    panel.right_pixhawk_radio_device = ""
    radio_missing = controller._pixhawk_serial_preflight("right")
    assert radio_missing.state == "RADIO DEVICE MISSING"


def test_usb_endpoint_is_exclusive_across_roles(tmp_path) -> None:
    controller, panel, _logs, _saves = _controller(tmp_path)
    device = tmp_path / "ttyACM9"
    device.write_text("", encoding="utf-8")
    panel.left_pixhawk_bootstrap_url_edit.setText(str(device))
    panel.mavlink_pixhawk = SimpleNamespace(
        urls={"right": str(device)},
        connections={"right": object()},
    )

    busy = controller._pixhawk_serial_preflight("left")
    assert busy.ok is False
    assert busy.state == "USB DEVICE BUSY"
