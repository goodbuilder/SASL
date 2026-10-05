"""Small, UI-neutral lifecycle primitives shared by SASL application shells."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable


@dataclass(frozen=True)
class LifecycleStep:
    """One ordered shutdown/finalization operation."""

    name: str
    operation: Callable[[], object]


class LifecycleCoordinator:
    """Run an ordered, idempotent, retryable lifecycle sequence.

    A step returns ``False`` when it has requested an asynchronous stop but
    has not completed yet.  The next retry resumes at that same step.
    ``None`` and ``True`` mean that the step is complete.  The coordinator is
    intentionally independent of Qt, Legacy, Workstation, or any resource
    implementation so each UI remains free to own its resources separately.
    """

    def __init__(self, steps: list[LifecycleStep]) -> None:
        self.steps = list(steps)
        self.index = 0
        self.complete = False

    def run(self) -> bool:
        if self.complete:
            return True
        while self.index < len(self.steps):
            result = self.steps[self.index].operation()
            if result is False:
                return False
            self.index += 1
        self.complete = True
        return True
