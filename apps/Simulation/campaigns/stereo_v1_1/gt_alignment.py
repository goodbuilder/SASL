"""Image/control-time Gazebo GT alignment in world ENU with XYZW SLERP.

Position is world ENU [east,north,up] metres. Orientation maps the vehicle
reference link into world ENU and is stored as XYZW. Interpolation never
crosses a simulation epoch or extrapolates beyond source GT samples.
"""

from __future__ import annotations

import bisect
import math
from dataclasses import dataclass
from typing import Iterable


def _unit_xyzw(values) -> tuple[float, float, float, float]:
    if len(values) != 4:
        raise ValueError("GT quaternion must contain XYZW")
    q = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in q):
        raise ValueError("GT quaternion must be finite")
    norm = math.sqrt(sum(value * value for value in q))
    if norm <= 0:
        raise ValueError("GT quaternion must have nonzero norm")
    return tuple(value / norm for value in q)


def shortest_path_slerp_xyzw(left, right, alpha: float) -> tuple[float, float, float, float]:
    """Interpolate world-from-reference rotations along the shorter SO(3) arc."""
    t = float(alpha)
    if not 0 <= t <= 1 or not math.isfinite(t):
        raise ValueError("SLERP alpha must be in [0,1]")
    q0, q1 = _unit_xyzw(left), _unit_xyzw(right)
    dot = sum(a * b for a, b in zip(q0, q1))
    if dot < 0:
        q1 = tuple(-value for value in q1)
        dot = -dot
    dot = min(1.0, max(-1.0, dot))
    if dot > 0.9995:
        return _unit_xyzw([(1 - t) * a + t * b for a, b in zip(q0, q1)])
    theta = math.acos(dot)
    denominator = math.sin(theta)
    w0 = math.sin((1 - t) * theta) / denominator
    w1 = math.sin(t * theta) / denominator
    return _unit_xyzw([w0 * a + w1 * b for a, b in zip(q0, q1)])


@dataclass(frozen=True)
class AlignedGt:
    status: str
    position_world_enu_m: tuple[float, float, float] | None
    orientation_world_from_reference_xyzw: tuple[float, float, float, float] | None
    before_timestamp_ns: int | None
    after_timestamp_ns: int | None
    interpolation_alpha: float | None
    bracketing_gap_ns: int | None


class GroundTruthSeries:
    def __init__(self, rows: Iterable[dict], max_bracketing_gap_ns: int = 40_000_000):
        if max_bracketing_gap_ns <= 0:
            raise ValueError("maximum GT bracketing gap must be positive")
        self.max_bracketing_gap_ns = int(max_bracketing_gap_ns)
        by_epoch: dict[int, list[dict]] = {}
        for row in rows:
            epoch = int(row["simulation_epoch"])
            stamp = int(row["source_timestamp_ns"])
            position = tuple(float(value) for value in row["position_world_enu_m"])
            if len(position) != 3 or not all(math.isfinite(value) for value in position):
                raise ValueError("GT world ENU position must contain three finite metres")
            orientation = _unit_xyzw(row["orientation_world_from_reference_xyzw"])
            by_epoch.setdefault(epoch, []).append({"stamp": stamp, "position": position, "orientation": orientation})
        for epoch, values in by_epoch.items():
            values.sort(key=lambda item: item["stamp"])
            if any(a["stamp"] == b["stamp"] for a, b in zip(values, values[1:])):
                raise ValueError(f"duplicate GT stamp within simulation epoch {epoch}")
        self.by_epoch = by_epoch
        self.times = {epoch: [row["stamp"] for row in values] for epoch, values in by_epoch.items()}

    def align(self, simulation_epoch: int, timestamp_ns: int) -> AlignedGt:
        epoch = int(simulation_epoch)
        stamp = int(timestamp_ns)
        rows = self.by_epoch.get(epoch)
        if not rows:
            return AlignedGt("UNAVAILABLE", None, None, None, None, None, None)
        times = self.times[epoch]
        index = bisect.bisect_left(times, stamp)
        if index < len(times) and times[index] == stamp:
            row = rows[index]
            return AlignedGt("EXACT", row["position"], row["orientation"], stamp, stamp, 0.0, 0)
        if index == 0 or index == len(times):
            before = times[index - 1] if index > 0 else None
            after = times[index] if index < len(times) else None
            return AlignedGt("EXTRAPOLATION", None, None, before, after, None, None)
        before, after = rows[index - 1], rows[index]
        gap = after["stamp"] - before["stamp"]
        alpha = (stamp - before["stamp"]) / gap
        position = tuple(
            (1 - alpha) * before["position"][axis] + alpha * after["position"][axis]
            for axis in range(3)
        )
        orientation = shortest_path_slerp_xyzw(before["orientation"], after["orientation"], alpha)
        return AlignedGt(
            "GAP_GE_40MS" if gap >= self.max_bracketing_gap_ns else "INTERPOLATED",
            position, orientation, before["stamp"], after["stamp"], alpha, gap,
        )
