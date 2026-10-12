"""Finite external value ports. Only the outer adapter knows the Qt panel."""

from __future__ import annotations
from dataclasses import dataclass
from typing import Any, Callable
from Workstation.measurement.state import UNSET, read


@dataclass(frozen=True)
class ValueInput:
    reader: Callable[[Any], Any]
    writer: Callable[[Any], None] | None = None

    def get(self, default: Any = UNSET) -> Any:
        return read(self.reader(UNSET), default)

    def exists(self) -> bool:
        return self.reader(UNSET) is not UNSET

    def set(self, value: Any) -> None:
        if self.writer is None:
            raise TypeError("read-only Measurement input")
        self.writer(value)
