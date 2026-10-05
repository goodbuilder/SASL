"""Severity classification for operator log lines.

``MainWindow._log`` receives plain Korean-prefixed strings with no logging
level, so the persisted ``application_log`` stream could not be filtered for
warnings. This classifier infers a level from the text so the stream carries a
``level`` field (``INFO``/``WARNING``/``ERROR``) without touching the hundreds
of call sites. It is deliberately conservative: only explicit failure or
warning vocabulary escalates a line.
"""

from __future__ import annotations

import re

_ERROR_PATTERN = re.compile(
    r"(실패|오류|에러|예외|FAILED|ERROR|Traceback|Exception|CRITICAL|손상|거부됨|rejected)",
    re.IGNORECASE,
)
_WARNING_PATTERN = re.compile(
    r"(경고|주의|STALE|timeout|타임아웃|skipped|건너뜀|불완전|없음|missing|unavailable|drop|밀림|지연|backpressure|retry|재시도|WARN)",
    re.IGNORECASE,
)


def classify_log_level(text: object) -> str:
    """Return ``ERROR``, ``WARNING`` or ``INFO`` for one operator log line."""

    value = str(text or "")
    if _ERROR_PATTERN.search(value):
        return "ERROR"
    if _WARNING_PATTERN.search(value):
        return "WARNING"
    return "INFO"


__all__ = ["classify_log_level"]
