"""Format numeric recording fields and convert the declared session clock anchor.

Only the UTC/monotonic host anchor is used; source camera/GPS clocks are not aliased."""
from __future__ import annotations
import time
from .state import (RecordingSessionState)


class Formatting:
    def __init__(self, *,
        session: RecordingSessionState,
    ) -> None:
        self.session = session

    def _fmt_record_float(self, value: float | None, precision: int = 6) -> float | None:
        """Round a recorded number to ``precision`` decimals, keeping it numeric.

        Rows used to carry every number as a fixed-precision string, which
        made JSONL/CSV consumers parse text and turned workbook cells into
        text. ``None`` stays ``None`` (JSON ``null``, empty CSV cell).
        """

        if value is None:
            return None
        number = float(value)
        if number != number or number in (float("inf"), float("-inf")):
            return None
        return round(number, int(precision))

    def _fmt_record_bool(self, value: bool | None) -> str:
        return "" if value is None else ("true" if value else "false")

    def _recording_elapsed_s(self, timestamp_s: float | None = None) -> float | None:
        started_s = self.session.distance_recording_started_monotonic_s or self.session.recording_session_started_monotonic_s
        if started_s is None:
            return None
        if timestamp_s is None:
            timestamp_s = time.monotonic()
        return max(0.0, float(timestamp_s) - started_s)

    def _session_utc_ns_for_monotonic_ns(self, monotonic_ns: int) -> int | None:
        if (
            self.session.recording_session_started_unix_ns is None
            or self.session.recording_session_started_monotonic_ns is None
        ):
            return None
        return (
            self.session.recording_session_started_unix_ns
            + monotonic_ns
            - self.session.recording_session_started_monotonic_ns
        )

    def _recording_side(self, camera: str) -> str | None:
        normalized = str(camera).lower()
        if normalized.startswith("left"):
            return "left"
        if normalized.startswith("right"):
            return "right"
        if normalized.startswith("drone"):
            return "drone"
        return None
