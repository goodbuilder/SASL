"""GGA policy and skip-state owner with explicit selected GNSS observations.

Latitude/longitude are WGS84 degrees, GGA altitude is MSL metres. Existing
single-source helpers preserve ellipsoid-minus-MSL geoid separation; the
system-center hint retains its original geodetic mean and blank geoid field.
Sample age/skew uses host monotonic time; the NMEA encoder uses wall UTC.
No NED/ENU/body/camera conversion or runtime ground truth is introduced.
"""

from __future__ import annotations
import time
from math import isfinite
from Workstation.workstation_core.gnss_network import format_nmea_gga, gga_from_coordinate_sample, gga_from_position
from Workstation.workstation_core.gnss_operation import GgaPolicy

SYSTEM_CENTER_GGA_MAXIMUM_GPS_SAMPLE_AGE_S = 10.0
SYSTEM_CENTER_GGA_MAXIMUM_LEFT_RIGHT_TIMESTAMP_DELTA_MS = 3000.0
GGA_STATE_FIELDS = ("_gnss_last_gga_skip_reason", "_gnss_last_gga_skip_monotonic_s", "_gnss_gga_skip_log_throttle")


class GgaSentenceOwner:
    def __init__(self, publish_legacy=None):
        self._gnss_last_gga_skip_reason = ""
        self._gnss_last_gga_skip_monotonic_s = None
        self._gnss_gga_skip_log_throttle = None
        self._publish_legacy = publish_legacy

    def _publish(self):
        if self._publish_legacy is not None:
            self._publish_legacy(self)

    def clear_skip(self):
        self._gnss_last_gga_skip_reason = ""
        self._gnss_last_gga_skip_monotonic_s = None
        self._publish()

    def log_skip(self, key: str, message: str, log):
        now_s = time.monotonic()
        self._gnss_last_gga_skip_reason = message
        self._gnss_last_gga_skip_monotonic_s = now_s
        throttle = self._gnss_gga_skip_log_throttle
        if not isinstance(throttle, dict):
            throttle = {}
            self._gnss_gga_skip_log_throttle = throttle
        self._publish()
        previous_s = float(throttle.get(key, float("-inf")))
        if now_s - previous_s < 30.0:
            return
        throttle[key] = now_s
        log(message)

    def resolve(self, active_config, *, base_sample=None, left=None, right=None, log):
        policy = active_config.network.gga_policy

        def log_skip(key, message):
            self.log_skip(key, message, log)

        if policy == GgaPolicy.BASE_GNSS:
            sample = base_sample
            if sample is None:
                log_skip(
                    "base-missing",
                    "[GNSS][NTRIP] GGA skipped: no Base GNSS sample received yet (policy=BASE_GNSS)",
                )
                return None
            age_s = max(0.0, time.monotonic() - sample.timestamp_s)
            if age_s > 15.0:
                log_skip(
                    "base-stale",
                    f"[GNSS][NTRIP] GGA skipped: Base GNSS sample is stale "
                    f"(age={age_s:.1f}s > 15.0s, policy=BASE_GNSS)",
                )
                return None
            return gga_from_coordinate_sample(sample)
        if policy == GgaPolicy.MANUAL:
            log_skip(
                "manual",
                "[GNSS][NTRIP] GGA skipped: GGA policy is MANUAL; GGA is intentionally never sent",
            )
            return None
        now_s = time.monotonic()
        maximum_age_s = (
            SYSTEM_CENTER_GGA_MAXIMUM_GPS_SAMPLE_AGE_S
            if policy == GgaPolicy.SYSTEM_CENTER
            else float(active_config.readiness.maximum_gps_sample_age_s)
        )

        def fresh(position: object, side: str, *, log: bool = True) -> object | None:
            def skip(key: str, message: str) -> None:
                if log:
                    log_skip(key, message)

            if position is None:
                skip(
                    f"{side}-missing",
                    f"[GNSS][NTRIP] GGA skipped: {side} GNSS has no message (policy={policy.value})",
                )
                return None
            if not bool(getattr(position, "has_position", False)):
                skip(
                    f"{side}-incomplete",
                    f"[GNSS][NTRIP] GGA skipped: {side} GNSS position is incomplete (policy={policy.value})",
                )
                return None
            try:
                timestamp_s = float(getattr(position, "timestamp_s"))
                age_s = now_s - timestamp_s
            except (AttributeError, TypeError, ValueError):
                age_s = float("nan")
            if not isfinite(age_s) or age_s < 0.0:
                skip(
                    f"{side}-timestamp-invalid",
                    f"[GNSS][NTRIP] GGA skipped: {side} GNSS timestamp is invalid (policy={policy.value})",
                )
                return None
            if age_s > maximum_age_s:
                skip(
                    f"{side}-stale",
                    f"[GNSS][NTRIP] GGA skipped: {side} GNSS sample is stale "
                    f"(age={age_s:.3f}s > {maximum_age_s:.3f}s, policy={policy.value})",
                )
                return None
            return position

        if policy == GgaPolicy.LEFT_GNSS:
            selected = fresh(left, "left")
            return None if selected is None else gga_from_position(selected)
        if policy == GgaPolicy.PER_ROVER_SESSION:
            # P1 and P2 may only be connectable one at a time.  A VRS caster
            # only needs the rover placed within its service area, and the two
            # antennas sit metres apart, so the session sends whichever rover
            # is currently delivering instead of requiring one fixed side.
            # Left wins when both deliver, keeping the both-connected result
            # identical to the previous behaviour.
            for side, position in (("left", left), ("right", right)):
                selected = fresh(position, side, log=False)
                if selected is not None:
                    return gga_from_position(selected)

            def describe(position: object) -> str:
                if position is None:
                    return "no message received"
                if not bool(getattr(position, "has_position", False)):
                    return "message received but lat/lon/alt incomplete"
                return "stale or invalid timestamp"

            log_skip(
                "per-rover-session-unavailable",
                "[GNSS][NTRIP] GGA skipped: no connected rover has a usable position "
                f"(policy={policy.value}, left={describe(left)}, right={describe(right)})",
            )
            return None
        # SYSTEM_CENTER: average WGS84 geodetic values for the two antenna hints.
        fresh_left = fresh(left, "left")
        fresh_right = fresh(right, "right")
        if fresh_left is None or fresh_right is None:

            def describe(position: object) -> str:
                if position is None:
                    return "no message received"
                if not bool(getattr(position, "has_position", False)):
                    return "message received but lat/lon/alt incomplete"
                return "stale or invalid timestamp"

            log_skip(
                "system-center-unavailable",
                "[GNSS][NTRIP] GGA skipped: no left/right GNSS position available "
                f"(policy={policy.value}, left={describe(left)}, right={describe(right)})",
            )
            return None
        timestamp_delta_ms = (
            abs(float(getattr(fresh_right, "timestamp_s")) - float(getattr(fresh_left, "timestamp_s"))) * 1000.0
        )
        maximum_delta_ms = SYSTEM_CENTER_GGA_MAXIMUM_LEFT_RIGHT_TIMESTAMP_DELTA_MS
        if timestamp_delta_ms > maximum_delta_ms:
            log_skip(
                "system-center-skew",
                "[GNSS][NTRIP] GGA skipped: left/right GNSS timestamp delta "
                f"{timestamp_delta_ms:.3f}ms > {maximum_delta_ms:.3f}ms",
            )
            return None
        positions = (fresh_left, fresh_right)
        lat = sum(float(position.latitude_deg) for position in positions) / len(positions)
        lon = sum(float(position.longitude_deg) for position in positions) / len(positions)
        alt = sum(float(position.altitude_m) for position in positions) / len(positions)
        satellites_values = [
            int(value)
            for value in (getattr(position, "satellites_used", None) for position in positions)
            if value is not None
        ]
        accuracy_values = [
            float(value)
            for value in (getattr(position, "eph_m", None) for position in positions)
            if value is not None and isfinite(float(value))
        ]
        return format_nmea_gga(
            lat,
            lon,
            alt,
            fix_quality=1,
            satellites=min(satellites_values) if satellites_values else 12,
            hdop=max(accuracy_values) if accuracy_values else 1.0,
        )
