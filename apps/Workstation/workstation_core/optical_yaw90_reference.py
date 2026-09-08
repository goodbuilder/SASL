"""Frozen request-time Yaw90 reference; NED positions and optical RDF rotations."""
from __future__ import annotations

import hashlib
import json
import math
from types import SimpleNamespace

from .baseline_yaw import select_baseline_yaw
from .optical_pose import CameraPoseNed
from .rtk_geometry import ned_delta_m


def canonical_evidence(value) -> str:
    return json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":"), allow_nan=False)


def _clean(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {k: _clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_clean(v) for v in value]
    return value


def reference_poses(reference):
    if reference.get("schema") != "optical-yaw90-reference-v1":
        raise ValueError("unsupported Yaw90 reference schema")
    if reference.get("sha256"):
        digest = hashlib.sha256(canonical_evidence({k:v for k,v in reference.items() if k != "sha256"}).encode()).hexdigest()
        if digest != reference["sha256"]:
            raise ValueError("Yaw90 reference checksum mismatch")
    if reference.get("state") != "READY":
        raise RuntimeError("Yaw90 capture reference unavailable: " + "; ".join(reference.get("blocking_reasons", ())))
    roll, pitch = reference["p1_roll_pitch_deg"]
    yaw = reference["selection"]["selected_yaw_deg"]
    return tuple(CameraPoseNed.from_virtual_frd_rpy_deg(
        center_ned_m=center, roll_deg=roll, pitch_deg=pitch, yaw_deg=yaw,
    ) for center in ((0., 0., 0.), reference["baseline_ned_m"]))


def build_reference(*, gps, attitudes, reverse, requested_monotonic_s, requested_utc_ns, sources):
    """Build from copied inputs only; aged finite samples warn, absent geometry blocks."""
    gps, attitudes = _clean(gps), _clean(attitudes)
    result = dict(schema="optical-yaw90-reference-v1", source="capture_request",
                  requested_monotonic_s=requested_monotonic_s, requested_utc_ns=requested_utc_ns,
                  gps=gps, attitudes=attitudes, gps_sources=sources, reverse=bool(reverse),
                  assumptions=["P1 body FRD aligned with camera virtual FRD",
                               "GNSS antenna centers approximate camera optical centers",
                               "camera yaw perpendicular to horizontal P1-P2 baseline",
                               "initial relative rotation is identity"],
                  state="BLOCKED", blocking_reasons=[], warnings=[],
                  frame="world NED; camera optical RDF; RPY virtual FRD degrees")
    try:
        positions = []
        for side in ("left", "right"):
            row = gps.get(side) or {}
            vals = [row.get(k) for k in ("latitude_deg", "longitude_deg", "altitude_m")]
            if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in vals):
                raise ValueError(f"{side} capture GNSS position missing or non-finite")
            if abs(vals[0]) > 90 or abs(vals[1]) > 180:
                raise ValueError(f"{side} capture GNSS position out of bounds")
            positions.append(SimpleNamespace(latitude_deg=vals[0], longitude_deg=vals[1], altitude_m=vals[2], has_position=True))
            if row.get("fix_type") != 6:
                result["warnings"].append(f"{side} capture GNSS fix_type={row.get('fix_type')}")
        baseline = ned_delta_m(*positions)
        rp = [(attitudes.get("left") or {}).get(k) for k in ("roll_deg", "pitch_deg")]
        if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in rp):
            raise ValueError("P1 capture Roll/Pitch missing or non-finite")
        selection = select_baseline_yaw(north_m=baseline[0], east_m=baseline[1],
            p1_yaw_deg=(attitudes.get("left") or {}).get("yaw_deg"),
            p2_yaw_deg=(attitudes.get("right") or {}).get("yaw_deg"), reverse=reverse)
        result.update(baseline_ned_m=list(baseline), p1_roll_pitch_deg=rp, selection=selection.to_mapping())
        if not selection.ready:
            raise ValueError(selection.reason)
        if selection.consensus is False or selection.low_margin:
            result["warnings"].append("Yaw90 branch disagreement or low selection margin")
        for kind, rows in (("gps", gps), ("attitude", attitudes)):
            for side, row in rows.items():
                stamp = (row or {}).get("timestamp_s")
                age = None if stamp is None else requested_monotonic_s - stamp
                if row is not None:
                    row["age_at_request_s"] = age
                if age is None or age < 0 or age > 2:
                    result["warnings"].append(f"{side} {kind} age at request={age}")
        result["state"] = "READY"
        reference_poses(result)  # Validate the optical rotations as well.
    except (TypeError, ValueError, KeyError) as exc:
        result["state"] = "BLOCKED"
        result["blocking_reasons"].append(str(exc))
    result["sha256"] = hashlib.sha256(canonical_evidence(result).encode()).hexdigest()
    return result
