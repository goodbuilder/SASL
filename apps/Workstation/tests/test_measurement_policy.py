from Workstation.workstation_core.measurement_policy import (
    FIELD_MEASUREMENT_POLICY_ID,
    field_measurement_decision,
)


def test_field_decision_valid_estimated_invalid_contract() -> None:
    valid = field_measurement_decision()
    estimated = field_measurement_decision(advisory_reasons=("stale attitude",))
    invalid = field_measurement_decision(
        hard_block_reasons=("missing R,t",),
        advisory_reasons=("sync unavailable",),
    )

    assert valid.to_mapping() == {
        "policy_id": FIELD_MEASUREMENT_POLICY_ID,
        "calculable": True,
        "valid": True,
        "validity": "VALID",
        "hard_block_reasons": [],
        "advisory_reasons": [],
    }
    assert estimated.validity == "ESTIMATED"
    assert estimated.valid is True
    assert invalid.validity == "INVALID"
    assert invalid.calculable is False


def test_field_decision_deduplicates_reasons_in_order() -> None:
    decision = field_measurement_decision(
        hard_block_reasons=("missing K,D", "missing K,D"),
        advisory_reasons=("stale", "stale", "low fix"),
    )

    assert decision.hard_block_reasons == ("missing K,D",)
    assert decision.advisory_reasons == ("stale", "low fix")
