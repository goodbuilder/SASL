"""Read-only live stereo transforms for Alignment, using Measurement's math.

GNSS is P1 -> P2 world NED metres. Attitude rotates body FRD -> NED.
Output is OpenCV RDF: X_right = R * X_left + t. No candidates, measurements,
device connections, or persistence state are created by this query.
"""
from __future__ import annotations

import math
import time
from collections.abc import Mapping

from Workstation.workstation_core.attitude_geometry import relative_attitude
from Workstation.workstation_core.baseline_yaw import quaternion_from_roll_pitch_yaw_deg, select_baseline_yaw
from Workstation.workstation_core.rtk_geometry import ned_delta_m
from Workstation.workstation_core.triangulation import camera_translation_right_in_left_camera, pixhawk_canonical_transform

LIVE_RT_METHODS = (("pixhawk_rtk_live", "Attitude + GNSS"),
                   ("pixhawk_rp_gps_baseline_yaw", "GNSS Yaw90"))


def _finite(value):
    result = float(value)
    if not math.isfinite(result):
        raise ValueError("non-finite sensor value")
    return result


def build_live_rt_rows(*, attitudes, gps, reverse=False, now_s, maximum_age_s=2.0):
    """Project two independent live rows; retain finite stale values with ages."""
    now = _finite(now_s)
    timestamps = tuple(getattr(sample, "timestamp_s", None) for sample in (*attitudes, *gps))
    ages = tuple(now - float(t) if isinstance(t, (float, int)) and math.isfinite(t) else None
                 for t in timestamps)
    known_times = [float(t) for t in timestamps if isinstance(t, (float, int)) and math.isfinite(t)]
    rows = []
    for method, label in LIVE_RT_METHODS:
        row = {"method_id": method, "label": label, "state": "WAITING", "reason": "",
               "oldest_sample_s": min(known_times) if known_times else None,
               "input_ages_s": ages, "age_s": max((a for a in ages if a is not None), default=None),
               "rotation_right_from_left_rdf": None, "translation_right_from_left_rdf_m": None,
               "right_origin_in_left_rdf_m": None, "relative_rpy_deg": None,
               "equation": "X_right = R_right_from_left_rdf * X_left + t_right_from_left_rdf_m",
               "frame": "OpenCV optical RDF [Right,Down,Forward]; t metres",
               "warnings": []}
        try:
            if any(sample is None for sample in attitudes):
                raise ValueError("P1/P2 attitude missing")
            if any(sample is None for sample in gps):
                raise ValueError("selected P1/P2 GNSS missing")
            baseline = ned_delta_m(*gps)
            if baseline is None or not all(math.isfinite(v) for v in baseline):
                raise ValueError("GNSS baseline unavailable")
            if math.sqrt(sum(v*v for v in baseline)) <= 1e-9:
                raise ValueError("GNSS baseline length is zero")
            if method == "pixhawk_rtk_live":
                quaternions = tuple(tuple(getattr(a, "q", ())) for a in attitudes)
            else:
                selection = select_baseline_yaw(north_m=baseline[0], east_m=baseline[1],
                    p1_yaw_deg=getattr(attitudes[0], "yaw_deg", None),
                    p2_yaw_deg=getattr(attitudes[1], "yaw_deg", None), reverse=bool(reverse))
                row["yaw_selection"] = selection.to_mapping()
                if not selection.ready:
                    raise ValueError(selection.reason)
                quaternions = tuple(quaternion_from_roll_pitch_yaw_deg(
                    _finite(a.roll_deg), _finite(a.pitch_deg), selection.selected_yaw_deg) for a in attitudes)
                if selection.low_margin:
                    row["warnings"].append("Yaw90 branch has low selection margin")
            origin = camera_translation_right_in_left_camera(
                baseline_right_in_left_world_m=baseline, left_body_to_world_q=quaternions[0])
            if origin is None:
                raise ValueError("invalid P1 attitude quaternion")
            rotation, translation = pixhawk_canonical_transform(
                left_body_to_world_q=quaternions[0], right_body_to_world_q=quaternions[1],
                translation_right_in_left_m=origin)
            relative = relative_attitude(*quaternions)
            row.update(rotation_right_from_left_rdf=rotation,
                       translation_right_from_left_rdf_m=translation,
                       right_origin_in_left_rdf_m=origin,
                       relative_rpy_deg=(relative.roll_deg, relative.pitch_deg, relative.yaw_deg),
                       baseline_ned_m=baseline,
                       state="LIVE" if all(a is not None and 0 <= a <= maximum_age_s for a in ages) else "STALE")
            if any(getattr(sample, "fix_type", None) != 6 for sample in gps):
                row["warnings"].append("GNSS is not RTK_FIXED on both sides")
            if row["state"] == "LIVE" and row["warnings"]:
                row["state"] = "WARNING"
            row["reason"] = "; ".join(row["warnings"])
        except (AttributeError, TypeError, ValueError, OverflowError) as exc:
            row["reason"] = str(exc)
        rows.append(row)
    return {"schema": "alignment-live-rt-v1", "observed_monotonic_s": now, "rows": rows}


def alignment_live_rt_snapshot(panel, *, now_s=None):
    """Peek at selected receiver and attitude owners without warming caches."""
    now = time.monotonic() if now_s is None else now_s
    latest = getattr(panel, "latest_extrinsics", {}) or {}
    attitudes = tuple(latest.get(side) for side in ("left", "right"))
    selector = getattr(panel, "_selected_baseline_gps_states", None)
    if callable(selector):
        gps = tuple(selector())
    else:
        latest_gps = getattr(panel, "latest_gps", {}) or {}
        settings = getattr(panel, "settings", None)
        gps = tuple(latest_gps.get(f"{side}_GPS2" if
            getattr(settings, f"pixhawk_{side}_gnss_source", "GPS1") == "GPS2" else side)
            for side in ("left", "right"))
    settings = getattr(panel, "settings", None)
    reverse = getattr(settings, "measurement_baseline_yaw_reverse", False)
    reader = getattr(panel, "_workstation_setting", None)
    if callable(reader):
        reverse = reader("measurement_baseline_yaw_reverse", reverse)
    else:
        overrides = getattr(panel, "_workstation_setting_overrides", {})
        if isinstance(overrides, Mapping):
            reverse = overrides.get("measurement_baseline_yaw_reverse", reverse)
    return build_live_rt_rows(attitudes=attitudes, gps=gps, reverse=reverse, now_s=now)
