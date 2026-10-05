"""Shared throttle for repeated error status/log emission."""

from __future__ import annotations

DEFAULT_ERROR_LOG_THROTTLE_S = 1.0


def should_emit_throttled_error(
    state: dict[str, tuple[str, float]],
    key: str,
    message: str,
    now_s: float,
    interval_s: float = DEFAULT_ERROR_LOG_THROTTLE_S,
) -> bool:
    """True once per distinct ``message`` per ``key``, then at most every ``interval_s``.

    ``state`` is mutated in place so callers can reuse one dict across
    repeated calls (keyed per camera, worker, etc).
    """

    previous = state.get(key)
    if (
        previous is not None
        and previous[0] == message
        and now_s - previous[1] < interval_s
    ):
        return False
    state[key] = (message, now_s)
    return True


__all__ = ["DEFAULT_ERROR_LOG_THROTTLE_S", "should_emit_throttled_error"]
