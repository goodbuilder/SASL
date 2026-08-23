"""Calibration-pair timing evidence without conflating host dispatch and exposure."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Mapping


CALIBRATION_TIMING_EVIDENCE_SCHEMA = "workstation-calibration-timing-evidence-v1"
EXPOSURE_TIMESTAMP_VERIFIED = "EXPOSURE_TIMESTAMP_VERIFIED"
CAMERA_TIMESTAMP_VERIFIED = "CAMERA_TIMESTAMP_VERIFIED"
HOST_DISPATCH_PROXY = "HOST_DISPATCH_PROXY"
UNKNOWN_TIMING_EVIDENCE = "UNKNOWN"
CALIBRATION_TIMING_EVIDENCE_CLASSES = frozenset(
    {
        EXPOSURE_TIMESTAMP_VERIFIED,
        CAMERA_TIMESTAMP_VERIFIED,
        HOST_DISPATCH_PROXY,
        UNKNOWN_TIMING_EVIDENCE,
    }
)


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
        if self.evidence_class == HOST_DISPATCH_PROXY and (
            self.precision_qualified or self.actual_exposure_verified
        ):
            raise ValueError("host dispatch proxy cannot qualify or verify camera exposure")
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
    "CALIBRATION_TIMING_EVIDENCE_CLASSES",
    "CALIBRATION_TIMING_EVIDENCE_SCHEMA",
    "CAMERA_TIMESTAMP_VERIFIED",
    "CalibrationTimingEvidence",
    "EXPOSURE_TIMESTAMP_VERIFIED",
    "HOST_DISPATCH_PROXY",
    "UNKNOWN_TIMING_EVIDENCE",
]
