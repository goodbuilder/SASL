"""Regression coverage for the panel delegate-binding swap in `standalone_gui.py`.

Before this refactor, 13 of the 14 hand-written delegate blocks guarded the
controller lookup with `getattr(panel, accessor_name, None)` and fell back to
constructing the controller directly when that accessor was absent (e.g. a
bare test double standing in for the panel). The 14th block —
`camera_time_sync` — skipped the guard and called `panel._camera_time_sync_controller()`
unconditionally, which raises `AttributeError` on any panel-like object that
doesn't define that accessor. `bind_controller_delegates` applies the same
guarded pattern to all 14 controllers, closing that gap.
"""

from __future__ import annotations

from types import SimpleNamespace

from Workstation import standalone_gui
from Workstation.camera_time_sync_controller import CameraTimeSyncController


class _PanelWithCachedAccessor:
    """Mimics the real class's `_camera_time_sync_controller` accessor."""

    def _camera_time_sync_controller(self):
        owner = getattr(self, "_owner", None)
        if owner is None:
            owner = CameraTimeSyncController(self)
            self._owner = owner
        return owner


def test_camera_time_sync_delegate_uses_cached_accessor_when_present():
    panel = _PanelWithCachedAccessor()

    result = standalone_gui.WirisStereoDistancePanel._camera_sync_widget_value(panel, "missing_attr", 2.5)

    assert result == 2.5
    assert panel._owner is panel._camera_time_sync_controller()


def test_camera_time_sync_delegate_falls_back_without_accessor():
    """This is the case that used to raise AttributeError before the fix."""

    panel = SimpleNamespace()

    result = standalone_gui.WirisStereoDistancePanel._camera_sync_widget_value(panel, "missing_attr", 3.5)

    assert result == 3.5
