"""Current-generation frame admission and one-side live integrity recovery.

Image buffers are never copied here. Native raster metadata and source clocks
pass through unchanged; the existing pane owns stream generations/resources.
"""

from collections.abc import Mapping
from dataclasses import dataclass
from typing import Callable


RECOVERY_FIELDS = (
    "_camera_integrity_unavailable_sides",
    "_camera_integrity_failed_generations",
    "_measurement_camera_rejoin_expected_generations",
)


@dataclass(frozen=True)
class SideStreamState:
    generation: int
    active: bool
    detection_active: bool


@dataclass(frozen=True)
class FrameAdmission:
    metadata: dict
    generation: int
    integrity_recovered: bool
    manual_rejoin_ready: bool


@dataclass(frozen=True)
class RecoveryPorts:
    read_side: Callable
    stop_side: Callable
    clear_side: Callable
    update_availability: Callable
    publish_source: Callable
    show_unavailable: Callable
    show_recovered: Callable
    recording_side_active: Callable
    refresh_view: Callable

    def __post_init__(self):
        if not all(callable(value) for value in vars(self).values()):
            raise TypeError("Recovery requires explicit side/source/presentation handoffs")


class StreamRecovery:
    def __init__(self, publish_legacy=None):
        self._camera_integrity_unavailable_sides = set()
        self._camera_integrity_failed_generations = {}
        self._measurement_camera_rejoin_expected_generations = {}
        self._publish_legacy = publish_legacy

    def _publish(self):
        if self._publish_legacy is not None:
            self._publish_legacy(self)

    def note_failure(self, side, generation):
        unavailable = set(self._camera_integrity_unavailable_sides or ())
        unavailable.add(side)
        self._camera_integrity_unavailable_sides = unavailable
        if not isinstance(self._camera_integrity_failed_generations, dict):
            self._camera_integrity_failed_generations = {}
        self._camera_integrity_failed_generations[side] = int(generation)
        self._publish()

    def admit_frame(self, side, metadata, *, current_generation):
        frame_metadata = dict(metadata) if isinstance(metadata, Mapping) else {}
        try:
            observed_generation = int(frame_metadata.get("stream_generation"))
        except (TypeError, ValueError):
            return None
        if observed_generation != int(current_generation):
            return None
        pending_rejoins = self._measurement_camera_rejoin_expected_generations
        manual_rejoin_ready = bool(
            isinstance(pending_rejoins, dict) and pending_rejoins.get(side) == observed_generation
        )
        if manual_rejoin_ready:
            pending_rejoins.pop(side, None)
        unavailable = set(self._camera_integrity_unavailable_sides or ())
        recovered = False
        if side in unavailable:
            failed = self._camera_integrity_failed_generations
            try:
                failed_generation = int(dict(failed or {})[side])
            except (KeyError, TypeError, ValueError):
                return None
            if observed_generation <= failed_generation:
                return None
            unavailable.discard(side)
            self._camera_integrity_unavailable_sides = unavailable
            if isinstance(failed, dict):
                failed.pop(side, None)
            recovered = True
        self._publish()
        return FrameAdmission(frame_metadata, observed_generation, recovered, manual_rejoin_ready)

    def active_sides(self, ports):
        return tuple(
            side
            for side in ("left", "right")
            if side not in set(self._camera_integrity_unavailable_sides or ()) and ports.read_side(side).active
        )

    def expect_manual_rejoin(self, side, generation, *, read_side):
        """Bind the restarted pane generation; keep an integrity-failed side out."""
        if not isinstance(self._measurement_camera_rejoin_expected_generations, dict):
            self._measurement_camera_rejoin_expected_generations = {}
        self._measurement_camera_rejoin_expected_generations[side] = int(generation)
        self._publish()
        unavailable = set(self._camera_integrity_unavailable_sides or ())
        if side not in unavailable:
            return None
        states = {key: read_side(key) for key in ("left", "right") if key != side and key not in unavailable}
        active = tuple(key for key, state in states.items() if state.active)
        return active, bool(active and any(states[key].detection_active for key in active))

    def unavailable(self, side, reason, ports: RecoveryPorts):
        if side not in {"left", "right"} or side in set(self._camera_integrity_unavailable_sides or ()):
            return
        self.note_failure(side, ports.read_side(side).generation)
        ports.stop_side(side)
        ports.clear_side(side)
        active = self.active_sides(ports)
        ports.update_availability(active, bool(active and any(ports.read_side(key).detection_active for key in active)))
        ports.publish_source(f"camera.{side}", "UNAVAILABLE", str(reason))
        ports.publish_source(f"recording.{side}", "UNAVAILABLE", str(reason))
        ports.show_unavailable(side, active, reason)

    def recovered(self, side, ports: RecoveryPorts):
        active = self.active_sides(ports)
        ports.update_availability(active, bool(active and any(ports.read_side(key).detection_active for key in active)))
        ports.show_recovered(side, active)
        ports.publish_source(f"camera.{side}", "LIVE", "newer-generation current frame confirmed after manual recovery")
        if ports.recording_side_active(side):
            ports.publish_source(f"recording.{side}", "LIVE", "new video segment joined after manual camera recovery")
        ports.refresh_view()
