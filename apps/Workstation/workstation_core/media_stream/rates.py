"""rates owner extracted from the Workstation media runtime.

Existing queues, locks, clocks, source-frame identity and policies are preserved.
Native pixels use image u/v and RGB/BGR/NV12; no world/body transform is introduced.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from typing import Any
from .policy import (
    CPU_DECODE_HZ_CRITICAL_GUARD,
    CPU_DECODE_HZ_FLOOR,
    CPU_DECODE_HZ_GUARD,
    CPU_DECODE_HZ_RECOVERY,
    CPU_DECODE_HZ_SOFT_FLOOR,
    CPU_DECODE_HZ_STRONG_RECOVERY,
    DEFAULT_CPU_YOLO_ADAPTIVE_MAX_RATE_HZ,
    DEFAULT_CPU_YOLO_INITIAL_RATE_HZ,
    DEFAULT_CPU_YOLO_MAX_RATE_HZ,
    DEFAULT_CPU_YOLO_MIN_RATE_HZ,
    DEFAULT_DECODE_RATE_WINDOW_S,
)


@dataclass
class RollingFrameRateEstimator:
    """Estimate a decoded frame rate without one-second boundary jitter."""

    window_s: float = DEFAULT_DECODE_RATE_WINDOW_S
    _timestamps: deque[float] = field(default_factory=deque)

    def __post_init__(self) -> None:
        try:
            window_s = float(self.window_s)
        except (TypeError, ValueError):
            window_s = DEFAULT_DECODE_RATE_WINDOW_S
        self.window_s = max(1.0, window_s) if math.isfinite(window_s) else DEFAULT_DECODE_RATE_WINDOW_S

    def _trim(self, now_s: float) -> None:
        cutoff_s = now_s - self.window_s
        while self._timestamps and self._timestamps[0] < cutoff_s:
            self._timestamps.popleft()

    def add(self, timestamp_s: float) -> None:
        try:
            timestamp_s = float(timestamp_s)
        except (TypeError, ValueError):
            return
        if not math.isfinite(timestamp_s):
            return
        if self._timestamps and timestamp_s < self._timestamps[-1]:
            return
        self._timestamps.append(timestamp_s)
        self._trim(timestamp_s)

    def rate(self, now_s: float) -> float:
        try:
            now_s = float(now_s)
        except (TypeError, ValueError):
            return 0.0
        if not math.isfinite(now_s):
            return 0.0
        self._trim(now_s)
        if len(self._timestamps) < 2:
            return 0.0
        elapsed_s = min(self.window_s, max(0.0, now_s - self._timestamps[0]))
        if elapsed_s <= 0.0:
            return 0.0
        return (len(self._timestamps) - 1) / elapsed_s


@dataclass
class RollingCounterRateEstimator:
    """Estimate a rate from a monotonic cumulative counter."""

    window_s: float = DEFAULT_DECODE_RATE_WINDOW_S
    _samples: deque[tuple[float, int]] = field(default_factory=deque)

    def __post_init__(self) -> None:
        try:
            window_s = float(self.window_s)
        except (TypeError, ValueError):
            window_s = DEFAULT_DECODE_RATE_WINDOW_S
        self.window_s = max(1.0, window_s) if math.isfinite(window_s) else DEFAULT_DECODE_RATE_WINDOW_S

    def _trim(self, now_s: float) -> None:
        cutoff_s = now_s - self.window_s
        while self._samples and self._samples[0][0] < cutoff_s:
            self._samples.popleft()

    def add(self, timestamp_s: float, total_count: int) -> None:
        try:
            timestamp_s = float(timestamp_s)
            total_count = int(total_count)
        except (TypeError, ValueError):
            return
        if not math.isfinite(timestamp_s) or total_count < 0:
            return
        if self._samples and timestamp_s < self._samples[-1][0]:
            return
        if self._samples and total_count < self._samples[-1][1]:
            # A worker reconnect or counter reset starts a fresh rate window.
            self._samples.clear()
        self._samples.append((timestamp_s, total_count))
        self._trim(timestamp_s)

    def rate(self, now_s: float) -> float:
        try:
            now_s = float(now_s)
        except (TypeError, ValueError):
            return 0.0
        if not math.isfinite(now_s):
            return 0.0
        self._trim(now_s)
        if len(self._samples) < 2:
            return 0.0
        first_s, first_count = self._samples[0]
        _last_s, last_count = self._samples[-1]
        elapsed_s = min(self.window_s, max(0.0, now_s - first_s))
        if elapsed_s <= 0.0:
            return 0.0
        return max(0, last_count - first_count) / elapsed_s


@dataclass
class AdaptiveYoloRateLimiter:
    enabled: bool
    target_rate_hz: float
    max_rate_hz: float
    min_rate_hz: float = DEFAULT_CPU_YOLO_MIN_RATE_HZ

    @classmethod
    def from_config(cls, config: dict[str, Any], *, enable_detection: bool) -> "AdaptiveYoloRateLimiter":
        max_rate_hz = max(0.0, float(config.get("max_rate_hz", 0.0)))
        enabled = bool(config.get("adaptive_rate", False)) and enable_detection and max_rate_hz > 0.0
        if not enabled:
            return cls(False, max_rate_hz, max_rate_hz, 0.0)
        try:
            adaptive_max_rate_hz = float(config.get("adaptive_max_rate_hz", DEFAULT_CPU_YOLO_ADAPTIVE_MAX_RATE_HZ))
        except (TypeError, ValueError):
            adaptive_max_rate_hz = DEFAULT_CPU_YOLO_ADAPTIVE_MAX_RATE_HZ
        max_rate_hz = min(max_rate_hz, max(DEFAULT_CPU_YOLO_MIN_RATE_HZ, adaptive_max_rate_hz))
        min_rate_hz = min(DEFAULT_CPU_YOLO_MIN_RATE_HZ, max_rate_hz)
        initial_rate_hz = min(max(DEFAULT_CPU_YOLO_INITIAL_RATE_HZ, min_rate_hz), max_rate_hz)
        return cls(True, initial_rate_hz, max_rate_hz, min_rate_hz)

    @property
    def interval_s(self) -> float:
        if self.target_rate_hz <= 0.0:
            return 0.0
        return 1.0 / self.target_rate_hz

    def update(self, *, decoded_hz: float, inferred_hz: float) -> bool:
        if not self.enabled:
            return False
        if inferred_hz <= 0.0:
            return False
        previous_rate_hz = self.target_rate_hz
        if decoded_hz <= CPU_DECODE_HZ_FLOOR:
            self.target_rate_hz = self.min_rate_hz
        elif decoded_hz < CPU_DECODE_HZ_CRITICAL_GUARD:
            self.target_rate_hz = self.min_rate_hz
        elif decoded_hz < CPU_DECODE_HZ_SOFT_FLOOR:
            self.target_rate_hz = max(self.min_rate_hz, self.target_rate_hz - 3.0)
        elif decoded_hz < CPU_DECODE_HZ_GUARD:
            self.target_rate_hz = max(self.min_rate_hz, self.target_rate_hz - 1.5)
        elif decoded_hz >= CPU_DECODE_HZ_STRONG_RECOVERY:
            self.target_rate_hz = min(self.max_rate_hz, self.target_rate_hz + 0.3)
        elif decoded_hz >= CPU_DECODE_HZ_RECOVERY:
            self.target_rate_hz = min(self.max_rate_hz, self.target_rate_hz + 0.15)
        return abs(self.target_rate_hz - previous_rate_hz) >= 0.05


def _activate_cpu_fallback_rate_limiter(
    rate_limiter: AdaptiveYoloRateLimiter,
    config: dict[str, Any],
) -> None:
    """Enable the CPU limiter when a requested CUDA device falls back at runtime."""

    if rate_limiter.enabled:
        return
    try:
        configured_max_rate_hz = float(config.get("max_rate_hz", 0.0) or 0.0)
    except (TypeError, ValueError):
        configured_max_rate_hz = 0.0
    max_rate_hz = min(
        configured_max_rate_hz if configured_max_rate_hz > 0.0 else DEFAULT_CPU_YOLO_MAX_RATE_HZ,
        DEFAULT_CPU_YOLO_ADAPTIVE_MAX_RATE_HZ,
    )
    min_rate_hz = min(DEFAULT_CPU_YOLO_MIN_RATE_HZ, max_rate_hz)
    rate_limiter.enabled = True
    rate_limiter.max_rate_hz = max_rate_hz
    rate_limiter.min_rate_hz = min_rate_hz
    rate_limiter.target_rate_hz = min(max(DEFAULT_CPU_YOLO_INITIAL_RATE_HZ, min_rate_hz), max_rate_hz)
    config["device"] = "cpu"
    config["adaptive_rate"] = True
    config["adaptive_max_rate_hz"] = max_rate_hz
    config["max_rate_hz"] = max_rate_hz


# Retain legacy pickle names without importing the facade.
RollingFrameRateEstimator.__module__ = "Workstation.workstation_core.yolo_stream"
RollingCounterRateEstimator.__module__ = "Workstation.workstation_core.yolo_stream"
AdaptiveYoloRateLimiter.__module__ = "Workstation.workstation_core.yolo_stream"
