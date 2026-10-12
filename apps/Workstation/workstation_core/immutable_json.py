"""Owned immutable JSON values at asynchronous measurement boundaries.

Arrays are tuples internally. Public exports are detached dict/list trees,
so the wire schema, numeric values, axis order and canonical JSON stay intact.
Unlike a bare mappingproxy, FrozenJsonObject also crosses spawn/pickle boundaries.
"""

from __future__ import annotations

import math
from collections.abc import Iterator, Mapping
from types import MappingProxyType


class FrozenJsonObject(Mapping[str, object]):
    __slots__ = ("_values",)

    def __init__(self, values: Mapping[str, object]) -> None:
        owned = {}
        for key, value in values.items():
            if not isinstance(key, str):
                raise TypeError("JSON object keys must be strings")
            owned[key] = freeze_json(value)
        object.__setattr__(self, "_values", MappingProxyType(owned))

    def __setattr__(self, _name: str, _value: object) -> None:
        raise TypeError("frozen JSON object cannot be modified")

    def __delattr__(self, _name: str) -> None:
        raise TypeError("frozen JSON object cannot be modified")

    def __getitem__(self, key: str) -> object:
        return self._values[key]

    def __iter__(self) -> Iterator[str]:
        return iter(self._values)

    def __len__(self) -> int:
        return len(self._values)

    def __eq__(self, other: object) -> bool:
        return isinstance(other, Mapping) and thaw_json(self) == thaw_json(other)

    def __repr__(self) -> str:
        return f"FrozenJsonObject({thaw_json(self)!r})"

    def __deepcopy__(self, _memo: dict) -> FrozenJsonObject:
        return self

    def __reduce__(self):
        return (type(self), (thaw_json(self),))


def freeze_json(value: object) -> object:
    """Detach mutable aliases and reject values outside finite JSON."""
    if isinstance(value, FrozenJsonObject):
        return value
    if isinstance(value, Mapping):
        return FrozenJsonObject(value)
    if isinstance(value, (list, tuple)):
        return tuple(freeze_json(item) for item in value)
    if value is None or isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ValueError("frozen JSON numbers must be finite")
        return value
    raise TypeError(f"unsupported frozen JSON value: {type(value).__name__}")


def thaw_json(value: object):
    """Return a fresh JSON tree; callers may edit it without changing its owner."""
    if isinstance(value, Mapping):
        return {key: thaw_json(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [thaw_json(item) for item in value]
    return value
