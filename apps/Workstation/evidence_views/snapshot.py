"""Detached read-only values at the source-to-evidence boundary."""

from __future__ import annotations

from collections.abc import Mapping
from copy import deepcopy
from types import MappingProxyType


class FrozenList(tuple):
    """Immutable capture retaining the producer's list-vs-tuple schema."""


def thaw_snapshot(value: object) -> object:
    """Return detached public output with its original container shapes."""
    if isinstance(value, Mapping):
        return {key: thaw_snapshot(item) for key, item in value.items()}
    if isinstance(value, FrozenList):
        return [thaw_snapshot(item) for item in value]
    if isinstance(value, tuple):
        return tuple(thaw_snapshot(item) for item in value)
    return deepcopy(value)


def _freeze(value: object) -> object:
    # Numerical source arrays/scalars become immutable native values; a copied
    # ndarray would still permit mutation through the captured evidence input.
    if type(value).__module__.split(".")[0] == "numpy" and callable(getattr(value, "tolist", None)):
        return _freeze(value.tolist())
    if isinstance(value, bytearray):
        return bytes(value)
    if isinstance(value, Mapping):
        return MappingProxyType({key: _freeze(item) for key, item in value.items()})
    if isinstance(value, FrozenList):
        return FrozenList(_freeze(item) for item in value)
    if isinstance(value, list):
        return FrozenList(_freeze(item) for item in value)
    if isinstance(value, tuple):
        return tuple(_freeze(item) for item in value)
    return deepcopy(value)


def snapshot_attributes(source: object | None, names: tuple[str, ...]) -> Mapping[str, object] | None:
    if source is None:
        return None
    missing = object()
    values = {}
    for name in names:
        value = getattr(source, name, missing)
        if value is not missing:
            values[name] = _freeze(value)
    return MappingProxyType(values)


def snapshot_mapping(values: Mapping[str, object], names: tuple[str, ...] | None = None) -> Mapping[str, object]:
    return MappingProxyType({key: _freeze(value) for key, value in values.items() if names is None or key in names})
