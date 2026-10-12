"""Compatibility composition root for cohesive recording owners.

Recording schemas retain camera RDF, measurement-center FRD and GNSS NED
contracts. Mutable recording state lives in recording.state, never the facade.
"""
from __future__ import annotations
import time
from Workstation.controllers.ulog_controller import ULogController
from Workstation.workstation_core.session_storage.recovery import discover_retryable_sessions, load_retryable_session
from Workstation.recording.contracts import RECORDING_CONTROLLER_METHODS, PairRecordingCommitResult, DistanceHistoryCommitResult, new_measurement_attempt_generation_id
from Workstation.recording.state import RECORDING_STATE_FIELDS, RecordingState
from Workstation.recording.binding import RecordingPanelBinding, recording_state_for
from Workstation.recording.composition import compose_recording
from Workstation.recording.compatibility import METHOD_OWNERS, QOBJECT_RECEIVERS, STATIC_METHODS, bind_recording_compatibility


class RecordingController:
    def __init__(self, panel) -> None:
        self.recording_state = recording_state_for(panel)
        self._binding = RecordingPanelBinding(panel, self.recording_state)
        self.owners = compose_recording(self._binding, self.recording_state)
        self.owners.ulog.controller_type = lambda: ULogController
        self.owners.recovery.discover_sessions = lambda root: discover_retryable_sessions(root)
        self.owners.recovery.load_session = lambda candidate: load_retryable_session(candidate)
        # Small legacy embeddings may omit the optional timing projection.
        # The blocked row still retains its canonical frozen timing fallback.
        if not callable(getattr(panel, "_record_measurement_sync_timestamp_parameters", None)):
            self.owners.measurement_rows.blocked_timing_writer = None
        for name, owner_name in METHOD_OWNERS.items():
            if name in QOBJECT_RECEIVERS or name in STATIC_METHODS:
                continue
            override = self._binding.fixture_override(name)
            if override is not None:
                setattr(getattr(self.owners, owner_name), name, override)
        video = self.recording_state.video_state
        if not hasattr(video, "_measurement_video_segments"):
            video._measurement_video_segments = {"left": [], "right": []}
        if not isinstance(getattr(video, "_measurement_video_segments_by_generation", None), dict):
            video._measurement_video_segments_by_generation = {}
        if not isinstance(getattr(video, "_recording_presence_state_by_generation", None), dict):
            video._recording_presence_state_by_generation = {}
        if not hasattr(video, "_last_recording_presence_state"):
            video._last_recording_presence_state = "IDLE"
        generation = str(
            getattr(self.recording_state.session, "recording_session_id", "")
            or getattr(panel, "_measurement_run_generation", "") or ""
        )
        if generation:
            video._measurement_video_segments_by_generation.setdefault(generation, video._measurement_video_segments)
        self._binding.publish_fixture_fields()

    def _call_recording(self, owner_name: str, method_name: str, *args, **kwargs):
        with self._binding.call_scope():
            return getattr(getattr(self.owners, owner_name), method_name)(*args, **kwargs)

    def resource_inventory(self) -> dict[str, object]:
        return self.recording_state.resource_inventory()


bind_recording_compatibility(RecordingController)
