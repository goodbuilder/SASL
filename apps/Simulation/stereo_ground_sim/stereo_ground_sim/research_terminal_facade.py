"""Research terminal state ownership and submission facade."""

import math

from stereo_ground_sim.research_terminal_coordinator import ResearchTerminalCoordinator
from stereo_ground_sim.research_terminal_record_builder import build_terminal_record

RESEARCH_TERMINAL_FACADE_METHODS = [
    "_research_recording_enabled",
    "_ensure_research_terminal_coordinator_locked",
    "_sync_research_terminal_state",
    "_all_finite",
    "_submit_research_terminal_locked"
]


class ResearchTerminalFacade:
    def __init__(self, node, *, camera_oracle_unset):
        self.node = node
        self.camera_oracle_unset = camera_oracle_unset

    def _research_recording_enabled(self):
        coordinator = getattr(self.node, "research_terminal_coordinator", None)
        if coordinator is not None:
            return coordinator.enabled
        return getattr(self.node, "research_recorder", None) is not None

    def _ensure_research_terminal_coordinator_locked(self):
        coordinator = getattr(self.node, "research_terminal_coordinator", None)
        recorder = getattr(self.node, "research_recorder", None)
        if coordinator is None:
            coordinator = ResearchTerminalCoordinator(recorder)
            coordinator.terminal_pair_keys = getattr(
                self, "research_terminal_pair_keys", set()
            )
            coordinator.terminal_record_count = getattr(
                self, "research_terminal_record_count", 0
            )
            coordinator.recording_overrun_invalid = getattr(
                self, "research_recording_overrun_invalid", False
            )
            coordinator.recording_error_count = getattr(
                self, "research_recording_error_count", 0
            )
            coordinator.first_error_type = getattr(
                self, "research_recording_first_error_type", ""
            )
            coordinator.first_error_message = getattr(
                self, "research_recording_first_error_message", ""
            )
            self.node.research_terminal_coordinator = coordinator
        elif coordinator.recorder is None and recorder is not None:
            coordinator.attach(recorder)
        self.node.research_terminal_pair_keys = coordinator.terminal_pair_keys
        return coordinator

    def _sync_research_terminal_state(self, coordinator):
        self.node.research_terminal_record_count = coordinator.terminal_record_count
        self.node.research_recording_overrun_invalid = coordinator.recording_overrun_invalid
        self.node.research_recording_error_count = coordinator.recording_error_count
        self.node.research_recording_first_error_type = coordinator.first_error_type
        self.node.research_recording_first_error_message = coordinator.first_error_message

    @staticmethod
    def _all_finite(values):
        return all(value is not None and math.isfinite(float(value)) for value in values)

    def _submit_research_terminal_locked(
        self,
        pair,
        *,
        terminal_status,
        terminal_reason,
        measurement_status,
        evaluation_status,
        metrics=None,
        alignment=None,
    ):
        """Submit one immutable terminal snapshot while holding the node lock.

        The recorder performs only validation plus ``put_nowait`` here.  File
        I/O, JSONL validation, CSV export, hashes, and final summary all run
        outside ``yolo_condition``.
        """

        coordinator = self.node._ensure_research_terminal_coordinator_locked()
        recorder = coordinator.recorder
        if recorder is None or pair is None:
            return False
        if not coordinator.claim(pair.epoch, pair.pair_id):
            return False

        record = build_terminal_record(
            self.node,
            pair,
            terminal_status=terminal_status,
            terminal_reason=terminal_reason,
            measurement_status=measurement_status,
            evaluation_status=evaluation_status,
            metrics=metrics,
            alignment=alignment,
            recorder=recorder,
            camera_oracle_unset=self.camera_oracle_unset,
        )
        submitted = coordinator.submit_claimed(record)
        self.node._sync_research_terminal_state(coordinator)
        return submitted
