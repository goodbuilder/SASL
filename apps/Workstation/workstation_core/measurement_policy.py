"""Single Workstation field-measurement validity policy.

Hard reasons mean geometry cannot be computed without inventing an input or
violating an identity/frame contract. Advisory reasons preserve computed
geometry while making its reduced confidence explicit in evidence and UI.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any


FIELD_MEASUREMENT_POLICY_ID = "field_measurement_v1"
MEASUREMENT_VALIDITY_VALUES = ("VALID", "ESTIMATED", "INVALID")


@dataclass(frozen=True)
class MeasurementGateDecision:
    hard_block_reasons: tuple[str, ...] = ()
    advisory_reasons: tuple[str, ...] = ()
    policy_id: str = FIELD_MEASUREMENT_POLICY_ID

    def __post_init__(self) -> None:
        if self.policy_id != FIELD_MEASUREMENT_POLICY_ID:
            raise ValueError(f"policy_id must be {FIELD_MEASUREMENT_POLICY_ID}")
        object.__setattr__(
            self,
            "hard_block_reasons",
            tuple(dict.fromkeys(str(value) for value in self.hard_block_reasons if str(value))),
        )
        object.__setattr__(
            self,
            "advisory_reasons",
            tuple(dict.fromkeys(str(value) for value in self.advisory_reasons if str(value))),
        )

    @property
    def calculable(self) -> bool:
        return not self.hard_block_reasons

    @property
    def validity(self) -> str:
        if self.hard_block_reasons:
            return "INVALID"
        return "ESTIMATED" if self.advisory_reasons else "VALID"

    @property
    def valid(self) -> bool:
        return self.validity in {"VALID", "ESTIMATED"}

    def to_mapping(self) -> dict[str, Any]:
        return {
            "policy_id": self.policy_id,
            "calculable": self.calculable,
            "valid": self.valid,
            "validity": self.validity,
            "hard_block_reasons": list(self.hard_block_reasons),
            "advisory_reasons": list(self.advisory_reasons),
        }


def field_measurement_decision(
    *,
    hard_block_reasons: tuple[str, ...] = (),
    advisory_reasons: tuple[str, ...] = (),
) -> MeasurementGateDecision:
    return MeasurementGateDecision(
        hard_block_reasons=hard_block_reasons,
        advisory_reasons=advisory_reasons,
    )
