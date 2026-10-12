"""Pure v1.1 world-ENU reference to PX4-local-NED contracts.

Reference JSON arrays are world ENU [east, north, up]. MAVLink local-NED
position and velocity arrays are [north, east, down]. Every position
conversion requires a runtime-observed origin; configured model spawn is not
an acceptable implicit default.
"""

from __future__ import annotations

import bisect
import math
from dataclasses import dataclass
from typing import Mapping, Sequence


def _finite(value: float, label: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{label} must be finite")
    return result


@dataclass(frozen=True)
class WorldEnu:
    east_m: float
    north_m: float
    up_m: float

    @classmethod
    def from_reference_xyz(cls, values: Sequence[float]) -> "WorldEnu":
        """Read a protocol world position array [east, north, up], metres."""
        if len(values) != 3:
            raise ValueError("world ENU position must have three values")
        return cls(*(_finite(value, "world ENU position") for value in values))

    def as_xyz(self) -> tuple[float, float, float]:
        return self.east_m, self.north_m, self.up_m


@dataclass(frozen=True)
class LocalNed:
    north_m: float
    east_m: float
    down_m: float

    @classmethod
    def from_px4_xyz(cls, values: Sequence[float]) -> "LocalNed":
        """Read PX4 local `x/y/z` as [north, east, down], metres."""
        if len(values) != 3:
            raise ValueError("PX4 local NED position must have three values")
        return cls(*(_finite(value, "PX4 local NED position") for value in values))

    def as_xyz(self) -> tuple[float, float, float]:
        return self.north_m, self.east_m, self.down_m


@dataclass(frozen=True)
class LocalNedVelocity:
    north_mps: float
    east_mps: float
    down_mps: float

    def as_xyz(self) -> tuple[float, float, float]:
        return self.north_mps, self.east_mps, self.down_mps


def observed_origin_world_enu(gt_world: WorldEnu, px4_local: LocalNed) -> WorldEnu:
    """Infer the PX4 local origin from synchronized GT and PX4 positions.

    Input world position is Gazebo reference-link ENU metres; input PX4
    position is local NED metres. The returned ENU translation is a sample,
    not a calibration: caller must check time alignment, estimator validity,
    consistency and reset counters before freezing it for a flight.
    """
    return WorldEnu(
        east_m=gt_world.east_m - px4_local.east_m,
        north_m=gt_world.north_m - px4_local.north_m,
        up_m=gt_world.up_m + px4_local.down_m,
    )


def world_position_to_px4_local_ned(position: WorldEnu, origin: WorldEnu) -> LocalNed:
    """Convert world ENU metres to PX4 local NED metres about observed origin."""
    return LocalNed(
        north_m=position.north_m - origin.north_m,
        east_m=position.east_m - origin.east_m,
        down_m=origin.up_m - position.up_m,
    )


def world_velocity_to_px4_local_ned(values: Sequence[float]) -> LocalNedVelocity:
    """Convert world ENU [east,north,up] m/s to local NED [north,east,down] m/s."""
    if len(values) != 3:
        raise ValueError("world ENU velocity must have three values")
    east, north, up = (_finite(value, "world ENU velocity") for value in values)
    return LocalNedVelocity(north, east, -up)


def heading_clockwise_from_north_to_px4_yaw_rad(degrees: float) -> float:
    """Return NED yaw radians from protocol north→east-positive heading."""
    radians = math.radians(_finite(degrees, "heading"))
    return math.atan2(math.sin(radians), math.cos(radians))


def interpolate_reference(samples: Sequence[Mapping[str, object]], elapsed_s: float) -> dict:
    """Linearly sample the 50 Hz time reference inside its science window.

    Outputs retain the protocol arrays: world ENU position [east,north,up] m,
    velocity [east,north,up] m/s and acceleration [east,north,up] m/s².
    Science time outside the recorded endpoints is rejected; it is never
    silently extrapolated into takeoff or post-science phases.
    """
    t = _finite(elapsed_s, "elapsed science time")
    if not samples:
        raise ValueError("reference has no samples")
    times = [float(row["t_science_s"]) for row in samples]
    if not all(math.isfinite(value) for value in times):
        raise ValueError("reference time must be finite")
    if any(right <= left for left, right in zip(times, times[1:])):
        raise ValueError("reference times must increase strictly")
    if t < times[0] or t > times[-1]:
        raise ValueError("science time is outside reference")
    index = bisect.bisect_left(times, t)
    if index == 0 or times[index] == t:
        return dict(samples[index])
    before, after = samples[index - 1], samples[index]
    alpha = (t - times[index - 1]) / (times[index] - times[index - 1])
    output = {"t_science_s": t}
    for name in (
        "position_world_enu_m",
        "velocity_world_enu_mps",
        "acceleration_world_enu_mps2",
    ):
        left = before[name]
        right = after[name]
        if not isinstance(left, (list, tuple)) or not isinstance(right, (list, tuple)) or len(left) != 3 or len(right) != 3:
            raise ValueError(f"{name} must contain three ENU components")
        output[name] = [
            (1 - alpha) * _finite(left[axis], name) + alpha * _finite(right[axis], name)
            for axis in range(3)
        ]
    output["heading_deg_clockwise_from_north"] = (
        (1 - alpha) * _finite(before["heading_deg_clockwise_from_north"], "heading")
        + alpha * _finite(after["heading_deg_clockwise_from_north"], "heading")
    )
    return output
