"""Qt-free owner of transient Measurement state and observation generations.

Device and storage adapters report facts or perform effects; neither owns Run
admission. Clocks are supplied in UTC/host-monotonic nanoseconds. Metadata is
opaque finite JSON and retains its producer's coordinate/time contracts.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field, replace

from Workstation.workstation_core.immutable_json import freeze_json, thaw_json


MEASUREMENT_RUN_STATES = frozenset({"IDLE", "QUEUED", "STARTING", "RUNNING", "STOPPING", "ERROR"})
MEASUREMENT_SOURCE_STATES = frozenset({"PENDING", "WAITING_RESOURCE", "LIVE", "UNAVAILABLE", "LEFT"})
MEASUREMENT_SOURCE_IDS = (
    "camera.left", "camera.right", "recording.left", "recording.right",
    "pixhawk.left", "pixhawk.right", "gnss.base", "ntrip", "target",
)


@dataclass(frozen=True)
class MeasurementRunSnapshot:
    state: str = "IDLE"
    detail: str = "Ready for Measurement"
    error: str = ""
    generation: str = ""
    last_generation: str = ""
    queued_reason: str = ""
    metadata: Mapping[str, object] = field(default_factory=dict)
    started_utc_ns: int | None = None
    started_monotonic_ns: int | None = None
    active_sides: tuple[str, ...] = ()
    sources: Mapping[str, object] = field(default_factory=dict)
    last_sources: Mapping[str, object] = field(default_factory=dict)

    def __post_init__(self) -> None:
        for name in ("metadata", "sources", "last_sources"):
            object.__setattr__(self, name, freeze_json(getattr(self, name)))

    def to_mapping(self) -> dict[str, object]:
        return {
            "schema": "workstation-measurement-run-v1",
            "state": self.state, "detail": self.detail, "error": self.error,
            "queued_reason": self.queued_reason, "generation": self.generation,
            "started_utc_ns": self.started_utc_ns,
            "started_monotonic_ns": self.started_monotonic_ns,
            "active_sides": list(self.active_sides),
            "sources": thaw_json(self.sources),
        }


@dataclass(frozen=True)
class SourceTransition:
    accepted: bool
    event: Mapping[str, object] | None = None


class MeasurementRunService:
    def __init__(self) -> None:
        self._snapshot = MeasurementRunSnapshot()

    @property
    def snapshot(self) -> MeasurementRunSnapshot:
        return self._snapshot

    def set_state(self, state: str, detail: str = "", error: str = "") -> None:
        normalized = str(state).upper()
        if normalized not in MEASUREMENT_RUN_STATES:
            raise ValueError(f"unsupported Measurement run state: {state}")
        self._snapshot = replace(self._snapshot, state=normalized, detail=str(detail), error=str(error))

    @staticmethod
    def initial_sources() -> dict[str, dict[str, object]]:
        return {
            source: dict(state="PENDING", detail="awaiting source evidence",
                         joined_utc_ns=None, joined_monotonic_ns=None,
                         left_utc_ns=None, left_monotonic_ns=None)
            for source in MEASUREMENT_SOURCE_IDS
        }

    def queue(self, reason: str) -> bool:
        if self._snapshot.state in {"STARTING", "RUNNING", "STOPPING"}:
            return False
        self._snapshot = replace(self._snapshot, state="QUEUED", queued_reason=str(reason),
                                 detail=f"Measurement start queued: {reason}", error="")
        return True

    def begin(self, generation: str, utc_ns: int, monotonic_ns: int, metadata: Mapping[str, object]) -> bool:
        if self._snapshot.state in {"STARTING", "RUNNING", "STOPPING"}:
            return False
        if not generation or utc_ns < 0 or monotonic_ns < 0:
            raise ValueError("Measurement generation and non-negative clock values are required")
        self._snapshot = replace(
            self._snapshot, state="STARTING", detail="Starting Measurement", error="",
            generation=str(generation), queued_reason="", metadata=metadata,
            started_utc_ns=int(utc_ns), started_monotonic_ns=int(monotonic_ns),
            active_sides=(), sources=self.initial_sources(),
        )
        return True

    def source_transition(self, source_id: str, state: str, detail: str,
                          utc_ns: int, monotonic_ns: int) -> SourceTransition:
        current = self._snapshot
        if not current.generation or current.state not in {"RUNNING", "STOPPING"}:
            return SourceTransition(False)
        source, normalized = str(source_id).strip(), str(state).strip().upper()
        if source not in MEASUREMENT_SOURCE_IDS:
            raise ValueError(f"unsupported Measurement source_id: {source}")
        if normalized not in MEASUREMENT_SOURCE_STATES:
            raise ValueError(f"unsupported Measurement source state: {state}")
        sources = thaw_json(current.sources)
        row = dict(sources[source])
        if row["state"] == normalized and row["detail"] == str(detail):
            return SourceTransition(True)
        if normalized == "LIVE" and row["state"] != "LIVE":
            row.update(joined_utc_ns=utc_ns, joined_monotonic_ns=monotonic_ns,
                       left_utc_ns=None, left_monotonic_ns=None)
        elif normalized in {"UNAVAILABLE", "LEFT"}:
            row.update(left_utc_ns=utc_ns, left_monotonic_ns=monotonic_ns)
        row.update(state=normalized, detail=str(detail))
        sources[source] = row
        active = tuple(side for side in ("left", "right") if sources[f"camera.{side}"]["state"] == "LIVE")
        live_count = sum(value["state"] == "LIVE" for value in sources.values())
        self._snapshot = replace(
            current, sources=sources, active_sides=active,
            detail=f"Measurement observing; live sources={live_count}" if current.state == "RUNNING" else current.detail,
        )
        return SourceTransition(True, freeze_json({
            "event_type": "measurement_source_transition",
            "runtime_generation": current.generation,
            "source_id": source, "source_state": normalized, "detail": str(detail),
            "observed_utc_ns": utc_ns, "observed_monotonic_ns": monotonic_ns,
        }))

    def mark_stopping(self) -> bool:
        if self._snapshot.state not in {"STARTING", "RUNNING", "ERROR"}:
            return False
        self.set_state("STOPPING", "Releasing camera/Detection/video owners")
        return True

    def finish(self, expected_generation: str) -> MeasurementRunSnapshot | None:
        current = self._snapshot
        if not current.generation or expected_generation != current.generation:
            return None
        self._snapshot = replace(
            current, state="IDLE", detail="Ready for next Measurement", error="",
            last_generation=current.generation, last_sources=current.sources,
            generation="", active_sides=(), sources={},
        )
        return current

    def cancel(self, reason: str) -> bool:
        if self._snapshot.state not in {"QUEUED", "STARTING"}:
            return False
        self._snapshot = replace(self._snapshot, state="IDLE", detail=str(reason), error="",
                                 queued_reason="", generation="", active_sides=(), sources={})
        return True
