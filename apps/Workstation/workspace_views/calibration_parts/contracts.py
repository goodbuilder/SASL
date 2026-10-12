"""Finite calibration display inputs; no application or panel reference."""

from dataclasses import dataclass
from collections.abc import Callable


@dataclass(frozen=True)
class PreviewChannel:
    side: str
    frame_changed: object = None
    detection_changed: object = None
    stats_changed: object = None


@dataclass(frozen=True)
class ValidationActions:
    """Optional actions preserve read-only/legacy backend availability."""

    configure: Callable | None = None
    load_reference: Callable | None = None
    run: Callable | None = None
    snapshot: Callable | None = None
    save: Callable | None = None
    apply: Callable | None = None
