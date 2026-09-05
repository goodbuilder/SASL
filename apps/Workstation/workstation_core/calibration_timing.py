"""Calibration-pair timing evidence without conflating host proxies and exposure."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping


CALIBRATION_TIMING_EVIDENCE_SCHEMA = "workstation-calibration-timing-evidence-v1"
EXPOSURE_TIMESTAMP_VERIFIED = "EXPOSURE_TIMESTAMP_VERIFIED"
CAMERA_TIMESTAMP_VERIFIED = "CAMERA_TIMESTAMP_VERIFIED"
HOST_DISPATCH_PROXY = "HOST_DISPATCH_PROXY"
HOST_RECEIVE_PROXY = "HOST_RECEIVE_PROXY"
UNKNOWN_TIMING_EVIDENCE = "UNKNOWN"
CALIBRATION_TIMING_EVIDENCE_CLASSES = frozenset(
    {
        EXPOSURE_TIMESTAMP_VERIFIED,
        CAMERA_TIMESTAMP_VERIFIED,
        HOST_DISPATCH_PROXY,
        HOST_RECEIVE_PROXY,
        UNKNOWN_TIMING_EVIDENCE,
    }
)
STEREO_PAIR_ELIGIBLE = "STEREO_ELIGIBLE"
INTRINSICS_ONLY = "INTRINSICS_ONLY"
LEGACY_UNVERIFIED = "LEGACY_UNVERIFIED"
INVALID_METADATA = "INVALID_METADATA"
CALIBRATION_PAIR_USAGE_STATES = frozenset(
    {
        STEREO_PAIR_ELIGIBLE,
        INTRINSICS_ONLY,
        LEGACY_UNVERIFIED,
        INVALID_METADATA,
    }
)


@dataclass(frozen=True)
class CalibrationPairUsage:
    """Fail-closed stereo eligibility while preserving legacy photo sets."""

    state: str
    stereo_eligible: bool
    detail: str
    pair_delta_s: float | None = None
    maximum_delta_s: float | None = None

    def __post_init__(self) -> None:
        if self.state not in CALIBRATION_PAIR_USAGE_STATES:
            raise ValueError("unsupported calibration pair usage state")
        if self.state in {INTRINSICS_ONLY, INVALID_METADATA} and self.stereo_eligible:
            raise ValueError("rejected calibration pair usage cannot be stereo eligible")
        if self.state in {STEREO_PAIR_ELIGIBLE, LEGACY_UNVERIFIED} and not self.stereo_eligible:
            raise ValueError("accepted calibration pair usage must be stereo eligible")


def _optional_finite_float(value: object) -> float | None:
    if value is None:
        return None
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return result if math.isfinite(result) else None


def classify_calibration_pair_usage(
    payload: Mapping[str, object] | None,
) -> CalibrationPairUsage:
    """Classify whether one persisted capture may be used for stereo R,t.

    Missing timing fields remain eligible for backward compatibility.  Any
    explicit rejection wins over an allow field so a selection sidecar cannot
    promote an intrinsics-only capture back into the stereo solver.
    """

    if payload is None:
        return CalibrationPairUsage(
            LEGACY_UNVERIFIED,
            True,
            "capture timing metadata is absent; legacy compatibility",
        )
    if not isinstance(payload, Mapping):
        return CalibrationPairUsage(
            INVALID_METADATA,
            False,
            "capture metadata must contain a JSON object",
        )

    evidence = payload.get("calibration_timing_evidence")
    if evidence is not None and not isinstance(evidence, Mapping):
        return CalibrationPairUsage(
            INVALID_METADATA,
            False,
            "calibration_timing_evidence must contain an object",
        )
    evidence = evidence if isinstance(evidence, Mapping) else {}
    decisions: list[tuple[str, bool]] = []
    for owner, field_payload, field in (
        ("capture", payload, "stereo_pair_saved"),
        ("capture", payload, "pair_persistence_allowed"),
        ("timing evidence", evidence, "pair_persistence_allowed"),
    ):
        if field not in field_payload:
            continue
        value = field_payload[field]
        if type(value) is not bool:
            return CalibrationPairUsage(
                INVALID_METADATA,
                False,
                f"{owner} {field} must be a JSON boolean",
            )
        decisions.append((f"{owner}.{field}", value))

    pair_delta_s = _optional_finite_float(
        evidence.get("pair_delta_s", payload.get("pair_delta_s"))
    )
    maximum_delta_s = _optional_finite_float(
        evidence.get("maximum_delta_s", payload.get("pair_max_delta_s"))
    )
    detail = str(
        evidence.get("detail")
        or payload.get("pair_timing_status")
        or ""
    ).strip()
    rejected = [source for source, allowed in decisions if not allowed]
    allowed = [source for source, decision in decisions if decision]
    if rejected:
        conflict = f"; conflicting allow fields: {', '.join(allowed)}" if allowed else ""
        decision_detail = f"stereo rejected by {', '.join(rejected)}{conflict}"
        return CalibrationPairUsage(
            INTRINSICS_ONLY,
            False,
            f"{detail}; {decision_detail}" if detail else decision_detail,
            pair_delta_s,
            maximum_delta_s,
        )
    if allowed:
        return CalibrationPairUsage(
            STEREO_PAIR_ELIGIBLE,
            True,
            detail or f"stereo allowed by {', '.join(allowed)}",
            pair_delta_s,
            maximum_delta_s,
        )
    return CalibrationPairUsage(
        LEGACY_UNVERIFIED,
        True,
        "timing decision fields are absent; legacy compatibility",
        pair_delta_s,
        maximum_delta_s,
    )


def load_calibration_pair_usage(metadata_path: str | Path) -> CalibrationPairUsage:
    """Load pair usage from one capture metadata file."""

    path = Path(metadata_path)
    if not path.is_file():
        return classify_calibration_pair_usage(None)
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, TypeError, ValueError) as exc:
        return CalibrationPairUsage(
            INVALID_METADATA,
            False,
            f"cannot parse capture metadata: {exc}",
        )
    return classify_calibration_pair_usage(payload)


@dataclass(frozen=True)
class CalibrationTimingEvidence:
    evidence_class: str
    pair_persistence_allowed: bool
    precision_qualified: bool
    actual_exposure_verified: bool
    timestamp_reference: str
    pair_delta_s: float | None
    maximum_delta_s: float
    detail: str
    schema: str = CALIBRATION_TIMING_EVIDENCE_SCHEMA

    def __post_init__(self) -> None:
        if self.schema != CALIBRATION_TIMING_EVIDENCE_SCHEMA:
            raise ValueError("unsupported calibration timing evidence schema")
        if self.evidence_class not in CALIBRATION_TIMING_EVIDENCE_CLASSES:
            raise ValueError("unsupported calibration timing evidence class")
        if not math.isfinite(float(self.maximum_delta_s)) or self.maximum_delta_s < 0.0:
            raise ValueError("maximum_delta_s must be finite and non-negative")
        if self.pair_delta_s is not None and not math.isfinite(float(self.pair_delta_s)):
            raise ValueError("pair_delta_s must be finite when present")
        if self.actual_exposure_verified and self.evidence_class != EXPOSURE_TIMESTAMP_VERIFIED:
            raise ValueError("actual exposure verification requires exposure timestamp evidence")
        if self.evidence_class in {HOST_DISPATCH_PROXY, HOST_RECEIVE_PROXY} and (
            self.precision_qualified or self.actual_exposure_verified
        ):
            raise ValueError("host timing proxy cannot qualify or verify camera exposure")
        if self.precision_qualified and not self.pair_persistence_allowed:
            raise ValueError("precision timing cannot qualify a rejected pair")

    def to_mapping(self) -> dict[str, object]:
        return {
            "schema": self.schema,
            "evidence_class": self.evidence_class,
            "pair_persistence_allowed": self.pair_persistence_allowed,
            "precision_qualified": self.precision_qualified,
            "actual_exposure_verified": self.actual_exposure_verified,
            "timestamp_reference": self.timestamp_reference,
            "pair_delta_s": self.pair_delta_s,
            "maximum_delta_s": self.maximum_delta_s,
            "detail": self.detail,
            "pair_timing_ok_compatibility_contract": (
                "pair persistence gate only; never proof of simultaneous exposure"
            ),
        }

    @classmethod
    def from_mapping(cls, payload: Mapping[str, object]) -> "CalibrationTimingEvidence":
        return cls(
            schema=str(payload.get("schema") or ""),
            evidence_class=str(payload.get("evidence_class") or ""),
            pair_persistence_allowed=bool(payload.get("pair_persistence_allowed")),
            precision_qualified=bool(payload.get("precision_qualified")),
            actual_exposure_verified=bool(payload.get("actual_exposure_verified")),
            timestamp_reference=str(payload.get("timestamp_reference") or ""),
            pair_delta_s=(
                None
                if payload.get("pair_delta_s") is None
                else float(payload["pair_delta_s"])
            ),
            maximum_delta_s=float(payload.get("maximum_delta_s") or 0.0),
            detail=str(payload.get("detail") or ""),
        )


__all__ = [
    "CALIBRATION_PAIR_USAGE_STATES",
    "CALIBRATION_TIMING_EVIDENCE_CLASSES",
    "CALIBRATION_TIMING_EVIDENCE_SCHEMA",
    "CAMERA_TIMESTAMP_VERIFIED",
    "CalibrationPairUsage",
    "CalibrationTimingEvidence",
    "EXPOSURE_TIMESTAMP_VERIFIED",
    "HOST_DISPATCH_PROXY",
    "HOST_RECEIVE_PROXY",
    "INTRINSICS_ONLY",
    "INVALID_METADATA",
    "LEGACY_UNVERIFIED",
    "STEREO_PAIR_ELIGIBLE",
    "UNKNOWN_TIMING_EVIDENCE",
    "classify_calibration_pair_usage",
    "load_calibration_pair_usage",
]
