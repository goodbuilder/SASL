"""Regression coverage for the Measurement tab camera-pane resize jitter.

`_update_runtime_camera_settings_layout_mode` toggles `runtime_header_layout`
between LeftToRight and TopToBottom based on the available width. Toggling
that direction changes the header's own sizeHint, which changes how much of
`runtime_layout`'s remaining stretch space the camera panes (`video_row`)
get. With a single breakpoint and no hysteresis, resizing near that
breakpoint could flip the direction back and forth every layout pass,
visibly jittering the camera preview size. A hysteresis band fixes that.
"""

from __future__ import annotations

from types import SimpleNamespace

from PyQt5 import QtWidgets

from Workstation import standalone_gui

_update = standalone_gui.WirisStereoDistancePanel._update_runtime_camera_settings_layout_mode
_BREAKPOINT = standalone_gui.RUNTIME_CAMERA_LAYOUT_BREAKPOINT_PX
_HYSTERESIS = standalone_gui.RUNTIME_CAMERA_LAYOUT_HYSTERESIS_PX


def _panel_with_layout(direction) -> SimpleNamespace:
    return SimpleNamespace(
        runtime_header_layout=QtWidgets.QBoxLayout(direction),
        runtime_camera_settings_group=object(),
    )


def test_width_inside_hysteresis_band_does_not_flip_from_horizontal() -> None:
    panel = _panel_with_layout(QtWidgets.QBoxLayout.LeftToRight)

    _update(panel, _BREAKPOINT - 1)

    assert panel.runtime_header_layout.direction() == QtWidgets.QBoxLayout.LeftToRight


def test_width_inside_hysteresis_band_does_not_flip_from_vertical() -> None:
    panel = _panel_with_layout(QtWidgets.QBoxLayout.TopToBottom)

    _update(panel, _BREAKPOINT)

    assert panel.runtime_header_layout.direction() == QtWidgets.QBoxLayout.TopToBottom


def test_flips_to_stacked_once_clearly_below_breakpoint() -> None:
    panel = _panel_with_layout(QtWidgets.QBoxLayout.LeftToRight)

    _update(panel, _BREAKPOINT - _HYSTERESIS - 1)

    assert panel.runtime_header_layout.direction() == QtWidgets.QBoxLayout.TopToBottom


def test_flips_back_to_horizontal_once_clearly_above_breakpoint() -> None:
    panel = _panel_with_layout(QtWidgets.QBoxLayout.TopToBottom)

    _update(panel, _BREAKPOINT + _HYSTERESIS)

    assert panel.runtime_header_layout.direction() == QtWidgets.QBoxLayout.LeftToRight


def test_repeated_updates_at_the_bare_breakpoint_never_change_direction() -> None:
    """This is the exact scenario that used to jitter: hovering right at the
    single old breakpoint value on every resize event."""

    panel = _panel_with_layout(QtWidgets.QBoxLayout.LeftToRight)

    for _ in range(10):
        _update(panel, _BREAKPOINT)
        _update(panel, _BREAKPOINT - 1)
        _update(panel, _BREAKPOINT + 1)

    assert panel.runtime_header_layout.direction() == QtWidgets.QBoxLayout.LeftToRight


def test_noop_without_required_attributes() -> None:
    panel = SimpleNamespace()

    _update(panel, 500)  # must not raise
