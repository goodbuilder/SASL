"""Source-backed quality decisions shared by validation and profile application."""
from __future__ import annotations
from dataclasses import dataclass, replace
from Workstation.workstation_core.calibration import StereoIntrinsicsResult


@dataclass(frozen=True)
class QualityDecision:
    allowed: bool
    message: str = ""
    severity: str = ""
    acknowledge_warning: bool = False


@dataclass(frozen=True)
class MinimumPoseAssessment:
    result: StereoIntrinsicsResult
    below_minimum: bool


class CalibrationQuality:
    @staticmethod
    def assess_minimum_effective_poses(result: StereoIntrinsicsResult, minimum: int) -> MinimumPoseAssessment:
        """Apply the configured per-camera effective-pose gate to solver evidence.

        Counts are the solver's retained native-image observations. This gate
        changes quality evidence only; K,D, optical RDF R,t and image counts
        remain exactly the values returned by the solver.
        """
        below_minimum = result.left.image_count < minimum or result.right.image_count < minimum
        if not below_minimum:
            return MinimumPoseAssessment(result, False)
        quality = dict(result.calibration_quality or {})
        reason_codes = list(quality.get("reason_codes", ()) or ())
        reasons = list(quality.get("reasons", ()) or ())
        code = "configured_minimum_effective_poses_not_met"
        if code not in reason_codes:
            reason_codes.append(code)
            reasons.append({
                "severity": "FAIL",
                "code": code,
                "message": (
                    f"effective poses left={result.left.image_count}, "
                    f"right={result.right.image_count}; configured minimum={minimum}"
                ),
            })
        quality.update(state="FAIL", reason_codes=reason_codes, reasons=reasons)
        return MinimumPoseAssessment(replace(result, calibration_quality=quality), True)

    @staticmethod
    def approved_for_profile_save(state: str, *, capture_zoom_known: bool = True) -> bool:
        """Warnings remain attempt-only even when an operator approved Apply."""
        return state in {"PASS", "UNASSESSED"} and capture_zoom_known

    @staticmethod
    def _calibration_quality_mapping(value: object) -> dict[str, object] | None:
        if value is None:
            return None
        if isinstance(value, dict):
            return dict(value)
        to_mapping = getattr(value, "to_mapping", None)
        if callable(to_mapping):
            try:
                payload = to_mapping()
            except Exception:
                payload = None
            if isinstance(payload, dict):
                return dict(payload)
        state = str(getattr(value, "state", "") or "").strip().upper()
        if not state:
            return None
        return {
            "state": state,
            "reason_codes": tuple(getattr(value, "reason_codes", ()) or ()),
        }


    def _calibration_quality_state(
        self,
        result: StereoIntrinsicsResult,
        legacy_quality=(),
    ) -> tuple[str, tuple[str, ...]]:
        """Read quality output without turning missing assessment into success."""

        containers: list[object] = [result, getattr(result, "calibration_quality", None)]
        result_has_quality = any(
            value is not None
            for value in (
                getattr(result, "calibration_quality", None),
                getattr(result, "left_quality", None),
                getattr(result, "right_quality", None),
            )
        )
        # A result-owned snapshot is authoritative. Panel-level attributes are
        # compatibility fallbacks for older integrations and may refer to a
        # previously displayed result.
        if not result_has_quality:
            containers.extend(value for value in legacy_quality if value is not None)

        side_states: dict[str, str] = {}
        overall_states: list[str] = []
        reason_codes: list[str] = []
        for container in containers:
            container_mapping = self._calibration_quality_mapping(container)
            if container_mapping is not None:
                state = str(container_mapping.get("state") or "").strip().upper()
                if state in {"PASS", "WARNING", "FAIL"}:
                    overall_states.append(state)
                reason_codes.extend(
                    str(code)
                    for code in container_mapping.get("reason_codes", ()) or ()
                    if str(code)
                )
            for side in ("left", "right"):
                key = f"{side}_quality"
                value = container.get(key) if isinstance(container, dict) else getattr(container, key, None)
                mapping = self._calibration_quality_mapping(value)
                if mapping is None:
                    continue
                state = str(mapping.get("state") or "").strip().upper()
                if state in {"PASS", "WARNING", "FAIL"}:
                    side_states[side] = state
                reason_codes.extend(
                    str(code)
                    for code in mapping.get("reason_codes", ()) or ()
                    if str(code)
                )

        states = [*overall_states, *side_states.values()]
        if "FAIL" in states:
            state = "FAIL"
        elif "WARNING" in states:
            state = "WARNING"
        elif overall_states or set(side_states) == {"left", "right"}:
            state = "PASS"
        else:
            state = "UNASSESSED"
            reason_codes.append("quality_snapshot_unavailable")
        return state, tuple(dict.fromkeys(reason_codes))



    @staticmethod
    def decide(action, state, reasons, *, warning_confirmed=False):
        reason_text = ", ".join(reasons) or "no reason supplied"
        if state == "FAIL":
            return QualityDecision(False, f"{action} 차단: calibration quality FAIL ({reason_text})", "bad")
        if state == "WARNING" and not warning_confirmed:
            return QualityDecision(False, f"{action} 대기: WARNING 사유를 검토하고 명시적으로 승인하세요 ({reason_text})", "warn")
        return QualityDecision(True, acknowledge_warning=state == "WARNING")
