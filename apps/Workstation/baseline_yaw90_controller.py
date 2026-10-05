"""Read-only installation evidence and the existing Reverse setting bridge.

Positions are NED metres; vertex angles use horizontal N/E only. Raw Pixhawk
R/P remains body-FRD orientation in NED. No measurement geometry is written.
"""
from __future__ import annotations

import math
import time

from Workstation.evidence import EvidenceAdapter
from Workstation.workstation_core.pixhawk_attitude import MAVLINK_ATTITUDE_STALE_S
from Workstation.workstation_core.baseline_yaw import baseline_azimuth_deg, select_baseline_yaw
from Workstation.workstation_core.rtk_geometry import ned_delta_m


def reverse_editable(panel: object) -> bool:
    """Use the live run boundary, not the legacy per-method DOF fill lock."""
    state = str(getattr(panel, "_measurement_run_state", "IDLE")).upper()
    pending = getattr(panel, "_pending_camera_start", None)
    pending_detection = isinstance(pending, tuple) and bool(pending) and bool(pending[0])
    return (
        state in {"IDLE", "ERROR"}
        and not bool(getattr(panel, "yolo_enabled", False))
        and not pending_detection
    )


def baseline_yaw90_snapshot(panel: object) -> dict[str, object]:
    now = time.monotonic()
    evidence = EvidenceAdapter(panel)
    baseline = evidence.baseline_yaw_alignment(now_s=now)
    # Installation follows the selected receivers now, even before the runtime
    # refreshes its derived baseline cache after a receiver change.
    selector = getattr(panel, "_selected_baseline_gps_states", None)
    if callable(selector):
        left, right = selector()
    else:
        gps = getattr(panel, "latest_gps", {}) or {}
        left_key = "left_GPS2" if baseline.get("p1_gps_source") == "GPS2" else "left"
        right_key = "right_GPS2" if baseline.get("p2_gps_source") == "GPS2" else "right"
        left, right = gps.get(left_key), gps.get(right_key)
    delta = None
    try:
        delta = ned_delta_m(left, right) if left is not None and right is not None else None
        azimuth = baseline_azimuth_deg(delta[0], delta[1]) if delta is not None else None
    except (TypeError, ValueError):
        azimuth = None
    baseline["baseline_azimuth_deg"] = azimuth
    def source_age(sample):
        timestamp = getattr(sample, "timestamp_s", None)
        return now - timestamp if isinstance(timestamp, (int, float)) and math.isfinite(timestamp) else None

    ages = (source_age(left), source_age(right))
    baseline["p1_gps_age_s"], baseline["p2_gps_age_s"] = ages
    baseline["baseline_age_s"] = max((age for age in ages if age is not None), default=None)
    baseline["baseline_state"] = (
        "WAITING" if azimuth is None else "CURRENT"
        if all(isinstance(age, (int, float)) and 0 <= age <= baseline["maximum_gps_age_s"] for age in ages)
        else "STALE"
    )
    frozen = []
    for side, source_key in (("left", "p1_gps_source"), ("right", "p2_gps_source")):
        gps_evidence = evidence.gps(side, source=baseline.get(source_key, "GPS1"), now_s=now)
        if gps_evidence.get("position_frozen"):
            frozen.append(gps_evidence.get("position_frozen_cause", side))
    if frozen and azimuth is not None:
        baseline["baseline_state"] = "FROZEN"
        baseline["reason"] = "; ".join(str(reason) for reason in frozen)
    try:
        drone = evidence.drone_pixhawk_geometry(now_s=now)
    except (TypeError, ValueError, RuntimeError) as exc:
        drone = {"state": "WAITING", "reason": str(exc)}
    if frozen:
        drone = {**drone, "installation_position_frozen": True, "reason": baseline["reason"]}
    attitudes = []
    for side, label in (("left", "P1"), ("right", "P2")):
        row = evidence.attitude(side, now_s=now)
        values = (row.get("roll_deg"), row.get("pitch_deg"))
        finite = all(isinstance(v, (int, float)) and math.isfinite(v) for v in values)
        timestamp = row.get("app_timestamp_s")
        age = now - timestamp if isinstance(timestamp, (int, float)) else None
        row["age_s"] = age
        row["state"] = (
            "WAITING" if not finite else "CURRENT"
            if isinstance(age, (int, float)) and 0 <= age <= MAVLINK_ATTITUDE_STALE_S
            else "STALE"
        )
        attitudes.append({**row, "label": label})
    settings = evidence.measurement_settings()
    selection = None
    reference_ages = (None, None)
    branch_error = ""
    if delta is not None:
        # Peek at valid references using the existing validation/selection rule.
        # Do not warm Measurement caches or initialize its selection state just
        # because the operator opened an installation view.
        from Workstation.measurement_view_controller import MeasurementViewController

        live = getattr(panel, "latest_extrinsics", {}) or {}
        cached = getattr(panel, "_measurement_latest_valid_attitude", {}) or {}
        references = []
        for side in ("left", "right"):
            candidates = [sample for sample in (live.get(side), cached.get(side))
                          if MeasurementViewController._attitude_state_is_valid_for_measurement_cache(sample)]
            references.append(max(candidates, key=lambda sample: float(sample.timestamp_s)) if candidates else None)
        try:
            selection = select_baseline_yaw(
                north_m=delta[0], east_m=delta[1],
                p1_yaw_deg=getattr(references[0], "yaw_deg", None),
                p2_yaw_deg=getattr(references[1], "yaw_deg", None),
                reverse=bool(settings.get("measurement_baseline_yaw_reverse", False)),
            ).to_mapping()
            reference_ages = tuple(None if sample is None else max(0.0, now - sample.timestamp_s) for sample in references)
        except (TypeError, ValueError, RuntimeError, AttributeError) as exc:
            branch_error = str(exc)
    telemetry_reader = getattr(panel, "drone_telemetry_snapshot", None)
    try:
        telemetry = telemetry_reader() if callable(telemetry_reader) else {}
    except (TypeError, ValueError, RuntimeError, AttributeError) as exc:
        telemetry = {"gnss_baseline_center_frd": {"state": "WAITING", "reason": str(exc)}}
    return {
        "attitudes": attitudes,
        "baseline": baseline,
        "drone_geometry": drone,
        "midpoint": dict(telemetry or {}).get("gnss_baseline_center_frd", {}),
        "branch_preview": selection,
        "branch_error": branch_error,
        "reference_ages_s": reference_ages,
        "reverse": bool(settings.get("measurement_baseline_yaw_reverse", False)),
        "reverse_editable": reverse_editable(panel),
        "run_state": str(getattr(panel, "_measurement_run_state", "IDLE")),
    }
