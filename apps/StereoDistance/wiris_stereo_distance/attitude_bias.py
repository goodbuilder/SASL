from __future__ import annotations

from dataclasses import dataclass
from math import atan2, asin, copysign, degrees, pi, sqrt
from typing import Iterable


Quaternion = tuple[float, float, float, float]


@dataclass(frozen=True)
class RelativeAttitude:
    roll_deg: float
    pitch_deg: float
    yaw_deg: float


@dataclass(frozen=True)
class RelativeAttitudeBias:
    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    sample_count: int
    captured_timestamp_s: float


def normalize_quaternion(q: Iterable[float]) -> Quaternion:
    w, x, y, z = (float(value) for value in q)
    norm = sqrt(w * w + x * x + y * y + z * z)
    if norm <= 0.0:
        raise ValueError("zero-length quaternion")
    return (w / norm, x / norm, y / norm, z / norm)


def quaternion_conjugate(q: Quaternion) -> Quaternion:
    w, x, y, z = q
    return (w, -x, -y, -z)


def quaternion_multiply(a: Quaternion, b: Quaternion) -> Quaternion:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return normalize_quaternion(
        (
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        )
    )


def quaternion_to_euler_deg(q: Iterable[float]) -> RelativeAttitude:
    w, x, y, z = normalize_quaternion(q)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = degrees(atan2(sinr_cosp, cosr_cosp))

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = degrees(copysign(pi / 2.0, sinp))
    else:
        pitch = degrees(asin(sinp))

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = degrees(atan2(siny_cosp, cosy_cosp))
    return RelativeAttitude(roll, pitch, yaw)


def relative_attitude(left_q: Iterable[float], right_q: Iterable[float]) -> RelativeAttitude:
    left = normalize_quaternion(left_q)
    right = normalize_quaternion(right_q)
    relative_q = quaternion_multiply(right, quaternion_conjugate(left))
    return quaternion_to_euler_deg(relative_q)


def apply_relative_bias(relative: RelativeAttitude, bias: RelativeAttitudeBias | None) -> RelativeAttitude:
    if bias is None:
        return relative
    return RelativeAttitude(
        roll_deg=relative.roll_deg - bias.roll_deg,
        pitch_deg=relative.pitch_deg - bias.pitch_deg,
        yaw_deg=relative.yaw_deg - bias.yaw_deg,
    )


class RelativeAttitudeBiasCapture:
    def __init__(self, *, start_timestamp_s: float, duration_s: float) -> None:
        self.start_timestamp_s = float(start_timestamp_s)
        self.duration_s = float(duration_s)
        self.samples: list[RelativeAttitude] = []

    @property
    def deadline_s(self) -> float:
        return self.start_timestamp_s + self.duration_s

    def add(self, left_q: Iterable[float], right_q: Iterable[float]) -> None:
        self.samples.append(relative_attitude(left_q, right_q))

    def ready(self, now_s: float) -> bool:
        return now_s >= self.deadline_s

    def finish(self, *, captured_timestamp_s: float) -> RelativeAttitudeBias | None:
        if not self.samples:
            return None
        count = len(self.samples)
        return RelativeAttitudeBias(
            roll_deg=sum(sample.roll_deg for sample in self.samples) / count,
            pitch_deg=sum(sample.pitch_deg for sample in self.samples) / count,
            yaw_deg=sum(sample.yaw_deg for sample in self.samples) / count,
            sample_count=count,
            captured_timestamp_s=captured_timestamp_s,
        )
