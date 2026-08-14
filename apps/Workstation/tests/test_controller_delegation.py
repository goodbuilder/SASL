from __future__ import annotations

from Workstation.controller_delegation import ControllerDelegateBinding, bind_controller_delegates


class _FakeController:
    def __init__(self, panel):
        self.panel = panel

    def greet(self, suffix=""):
        return f"hello:{self.panel.name}{suffix}"


class _CachingPanel:
    name = "panel-a"

    def _fake_controller(self):
        owner = getattr(self, "_fake_controller_owner", None)
        if owner is None:
            owner = _FakeController(self)
            self._fake_controller_owner = owner
        return owner


class _FallbackPanel:
    """Deliberately has no ``_fake_controller`` accessor at all."""

    name = "panel-b"


def test_bind_controller_delegates_uses_cached_accessor():
    binding = ControllerDelegateBinding(_FakeController, "_fake_controller", ["greet"])
    bind_controller_delegates(_CachingPanel, binding)

    panel = _CachingPanel()
    assert panel.greet("!") == "hello:panel-a!"

    owner_first = panel._fake_controller()
    panel.greet()
    owner_second = panel._fake_controller()
    assert owner_first is owner_second


def test_bind_controller_delegates_falls_back_to_uncached_construction():
    binding = ControllerDelegateBinding(_FakeController, "_fake_controller", ["greet"])
    bind_controller_delegates(_FallbackPanel, binding)

    panel = _FallbackPanel()
    assert panel.greet("?") == "hello:panel-b?"


def test_bind_controller_delegates_binds_every_requested_method_name():
    binding = ControllerDelegateBinding(_FakeController, "_fake_controller", ["greet"])
    bind_controller_delegates(_CachingPanel, binding)

    assert hasattr(_CachingPanel, "greet")
    assert getattr(_CachingPanel, "greet").__name__ == "greet"
