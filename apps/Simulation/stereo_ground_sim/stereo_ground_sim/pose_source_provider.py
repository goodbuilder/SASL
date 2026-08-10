"""Validated PX4 attitude/GPS source cache with immutable pair snapshots.

Attitude samples are Hamilton WXYZ rotations from body FRD to earth NED.
GPS fixes remain geodetic latitude/longitude/MSL altitude. No values are
converted to world ENU, optical RDF, or baseline-center FRD in this module.
"""

from dataclasses import dataclass
import math

from stereo_ground_sim.gps_baseline import RawGpsFix


@dataclass(frozen=True)
class Px4AttitudeSample:
    quaternion_wxyz: tuple[float, float, float, float] | None
    timestamp_us: int | None
    simulation_timestamp_ns: int | None
    reset_counter: int | None
    simulation_epoch: int
    received_monotonic_ns: int
    valid: bool
    failure_reason: str


class PoseSourceProvider:
    def __init__(self):
        self.attitudes = {"left": None, "right": None}
        self.raw_gps = {"left": None, "right": None, "uam": None}
        self.raw_gps_failures = {"left": None, "right": None, "uam": None}
        self.global_positions = {"left": None, "right": None}

    def store_attitude(self, label, message, *, epoch, simulation_timestamp_ns, received_ns):
        quaternion = None
        timestamp_us = None
        reset_counter = None
        failure = f"invalid_{label}_quaternion"
        try:
            quaternion = tuple(float(value) for value in message.q)
            if len(quaternion) != 4 or not all(math.isfinite(v) for v in quaternion):
                raise ValueError
            magnitude = math.sqrt(sum(value * value for value in quaternion))
            if magnitude <= 1.0e-15:
                raise ValueError
            quaternion = tuple(value / magnitude for value in quaternion)
            timestamp_us = int(getattr(message, "timestamp_sample", 0)) or int(
                getattr(message, "timestamp", 0)
            )
            if timestamp_us <= 0:
                failure = f"invalid_{label}_attitude_timestamp"
                raise ValueError
            reset_counter = int(getattr(message, "quat_reset_counter", 0))
            valid = True
            failure = ""
        except (AttributeError, OverflowError, TypeError, ValueError):
            if not (
                isinstance(quaternion, tuple)
                and len(quaternion) == 4
                and all(math.isfinite(v) for v in quaternion)
                and math.sqrt(sum(value * value for value in quaternion)) > 1.0e-15
            ):
                quaternion = None
            valid = False
        sample = Px4AttitudeSample(
            quaternion, timestamp_us, simulation_timestamp_ns, reset_counter,
            int(epoch), int(received_ns), valid, failure,
        )
        self.attitudes[label] = sample
        return sample

    @staticmethod
    def _fix(message, *, fused):
        if fused and (not bool(message.lat_lon_valid) or not bool(message.alt_valid)):
            raise ValueError("invalid fused global position")
        fix = RawGpsFix(
            timestamp_us=int(message.timestamp_sample or message.timestamp),
            latitude_deg=float(message.lat if fused else message.latitude_deg),
            longitude_deg=float(message.lon if fused else message.longitude_deg),
            altitude_msl_m=float(message.alt if fused else message.altitude_msl_m),
            eph_m=float(message.eph), epv_m=float(message.epv),
        )
        if fix.timestamp_us <= 0 or not all(
            math.isfinite(value)
            for value in (
                fix.latitude_deg, fix.longitude_deg, fix.altitude_msl_m,
                fix.eph_m, fix.epv_m,
            )
        ):
            raise ValueError("non-finite or unstamped position fix")
        return fix

    def store_raw_gps(self, label, message, *, epoch, simulation_timestamp_ns, received_ns):
        try:
            fix = self._fix(message, fused=False)
        except (AttributeError, OverflowError, TypeError, ValueError) as exc:
            self.raw_gps[label] = None
            self.raw_gps_failures[label] = (
                f"invalid_{label}_gps_fix:{exc}", int(epoch), int(received_ns)
            )
            return None
        entry = (fix, int(received_ns), int(epoch), simulation_timestamp_ns)
        self.raw_gps[label] = entry
        self.raw_gps_failures[label] = None
        return entry

    def store_global_position(self, label, message, *, epoch, simulation_timestamp_ns, received_ns):
        try:
            fix = self._fix(message, fused=True)
        except (AttributeError, OverflowError, TypeError, ValueError):
            return None
        entry = (fix, int(received_ns), int(epoch), simulation_timestamp_ns)
        self.global_positions[label] = entry
        return entry

    def snapshot(self, *, simulation_timestamp_ns, monotonic_ns, epoch):
        return {
            "captured_simulation_timestamp_ns": simulation_timestamp_ns,
            "captured_monotonic_ns": int(monotonic_ns),
            "simulation_epoch": int(epoch),
            "px4_attitudes": dict(self.attitudes),
            "gps_baseline_fixes": dict(self.raw_gps),
            "global_baseline_fixes": dict(self.global_positions),
        }
