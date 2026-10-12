"""Finite dependency ports and state/view ownership for the Workstation panel.

Features have no panel proxy and cannot resolve undeclared names. Existing named
attributes remain compatibility properties; their backing values have one owner.
"""
from __future__ import annotations

from dataclasses import dataclass
from types import SimpleNamespace
from typing import Callable
from PyQt5 import QtCore


@dataclass(frozen=True)
class FeaturePort:
    read: Callable[[], object]
    write: Callable[[object], None] | None = None


class OwnedField:
    def __init__(self, name: str, *, view: bool = False):
        self.name = name
        self.bundle = "views" if view else "state"

    def __get__(self, instance, owner=None):
        if instance is None:
            return self
        return getattr(getattr(instance, self.bundle), self.name)

    def __set__(self, instance, value):
        setattr(getattr(instance, self.bundle), self.name, value)

    def __delete__(self, instance):
        delattr(getattr(instance, self.bundle), self.name)


class DependencyField:
    def __init__(self, name: str):
        self.name = name

    def __get__(self, instance, owner=None):
        if instance is None:
            return self
        return instance.ports[self.name].read()

    def __set__(self, instance, value):
        writer = instance.ports[self.name].write
        if writer is None:
            raise AttributeError(f"{self.name} is a read-only feature dependency")
        writer(value)


class PanelFeature(QtCore.QObject):
    """One feature's mutable state, widget references, behavior and bounded ports."""
    feature_name: str
    dependency_names: tuple[str, ...] = ()
    writable_dependencies: frozenset[str] = frozenset()

    def __init__(self, widget, *, state_factory: Callable[[QtCore.QObject], object] | None = None):
        super().__init__(widget)
        # QWidget is used only as the Qt parent/context. All domain access uses
        # the finite names declared on the concrete feature class.
        self.widget = widget
        # A feature can select typed, single-owner state during composition.
        # It must not replace the authoritative store after initialization.
        self.state = SimpleNamespace() if state_factory is None else state_factory(self)
        self.views = SimpleNamespace()
        self.ports: dict[str, FeaturePort] = {}

    def resource_inventory(self) -> dict[str, object]:
        """Expose real owned handles, including grouped thread/process owners."""
        return {
            name: value
            for name, value in {**vars(self.state), **vars(self.views)}.items()
            if name.endswith(("_thread", "_threads", "_worker", "_workers", "_timer", "_timers", "_process", "_socket"))
        }
