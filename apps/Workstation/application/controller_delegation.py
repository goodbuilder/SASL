"""Generic factory for binding controller methods onto the panel class.

Replaces the 14 hand-duplicated delegate-factory + ``setattr`` loop pairs that
used to live at the end of ``standalone_gui.py``. Every one of those blocks
implemented the same guarded lookup-then-fallback pattern; this module
implements it once so new controllers don't need a bespoke copy.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Sequence


@dataclass(frozen=True)
class ControllerDelegateBinding:
    """Describes how one controller's methods should be exposed on a panel.

    ``accessor_name`` must name a method already defined on the panel class
    that lazily constructs and caches the controller instance (the existing
    ``_recording_controller``-style accessors). ``controller_cls`` is used
    only as an uncached fallback when that accessor is absent, matching the
    behavior every existing delegate block already relies on.
    """

    controller_cls: type
    accessor_name: str
    method_names: Sequence[str]


def _make_delegate(binding: ControllerDelegateBinding, method_name: str):
    def delegated(panel: Any, *args: Any, **kwargs: Any) -> Any:
        owner_factory = getattr(panel, binding.accessor_name, None)
        owner = owner_factory() if callable(owner_factory) else binding.controller_cls(panel)
        return getattr(owner, method_name)(*args, **kwargs)

    delegated.__name__ = method_name
    return delegated


def bind_controller_delegates(panel_cls: type, binding: ControllerDelegateBinding) -> None:
    """Attach every name in ``binding.method_names`` onto ``panel_cls``.

    Each attached method, when called on a panel instance, forwards to the
    controller instance returned by ``binding.accessor_name`` (or a freshly
    constructed, uncached one if that accessor isn't present).
    """

    for method_name in binding.method_names:
        setattr(panel_cls, method_name, _make_delegate(binding, method_name))
