"""Lifecycle and fault containment for immutable research terminal records.

The coordinator treats record payloads as opaque schema values. Frame and axis
contracts remain the responsibility of the record builder and recorder schema.
"""


class ResearchTerminalCoordinator:
    def __init__(self, recorder=None):
        self.recorder = recorder
        self.terminal_pair_keys = set()
        self.terminal_record_count = 0
        self.recording_overrun_invalid = False
        self.recording_error_count = 0
        self.first_error_type = ""
        self.first_error_message = ""

    @property
    def enabled(self):
        return self.recorder is not None

    def attach(self, recorder):
        self.recorder = recorder

    def claim(self, epoch, pair_id):
        if not self.enabled:
            return False
        key = (int(epoch), int(pair_id))
        if key in self.terminal_pair_keys:
            return False
        self.terminal_pair_keys.add(key)
        return True

    def submit_claimed(self, record):
        """Submit an already claimed record and contain recorder failure state."""

        if not self.enabled:
            return False
        submitted = self.recorder.submit(record)
        if submitted:
            self.terminal_record_count += 1
            return True
        self.recording_error_count += 1
        self.recording_overrun_invalid = self.recorder.overrun_invalid
        if not self.first_error_type:
            self.first_error_type = self.recorder.first_error_type
            self.first_error_message = self.recorder.first_error_message
        return False

    def close(self, final_state, *, error=lambda _message: None):
        if not self.enabled:
            return None
        try:
            summary = self.recorder.close(dict(final_state))
            self.recording_overrun_invalid = self.recorder.overrun_invalid
            self.recording_error_count = self.recorder.error_count
            self.first_error_type = self.recorder.first_error_type
            self.first_error_message = self.recorder.first_error_message
            return summary
        except Exception as exc:
            self.recording_error_count += 1
            if not self.first_error_type:
                self.first_error_type = type(exc).__name__
                self.first_error_message = str(exc)
            error(
                "Research recorder finalization failed; RUN_INCOMPLETE retained: "
                f"{type(exc).__name__}: {exc}"
            )
            return None
