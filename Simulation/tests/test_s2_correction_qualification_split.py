from __future__ import annotations

from collections import Counter
from pathlib import Path
import sys


PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "stereo_ground_sim"
sys.path.insert(0, str(PACKAGE_ROOT))

from stereo_ground_sim.correction_qualification import (
    DATASET_ROLES,
    QUALIFICATION_STATES,
    QualificationSpec,
    build_qualification_bundle,
    prepare_plans,
    profile_key,
    spatial_group_key,
)


SHA = "a" * 40
BRANCH = "sim/s2-r4a-test"


def _spec() -> QualificationSpec:
    return QualificationSpec(
        qualification_id="qualification-test",
        repository_name="goodbuilder/SASL",
        branch=BRANCH,
        commit_sha=SHA,
    )


def test_dataset_roles_states_and_explicit_role_counts() -> None:
    assert DATASET_ROLES == ("calibration", "validation", "sealed_evaluation")
    assert QUALIFICATION_STATES == (
        "PLANS_PREPARED",
        "CALIBRATION_VALIDATION_READY",
        "PROFILE_FROZEN",
        "SEALED_OPENED",
        "SEALED_EVALUATED",
        "INVALID",
    )
    bundle = build_qualification_bundle(_spec())
    assert bundle["qualification_plan"]["role_case_counts"] == {
        "calibration": 450,
        "validation": 225,
        "sealed_evaluation": 225,
    }


def test_split_is_by_disjoint_spatial_group_not_pair_or_sample() -> None:
    roles = build_qualification_bundle(_spec())["roles"]
    expected_ranges = {
        "calibration": {300.0, 1000.0},
        "validation": {500.0},
        "sealed_evaluation": {1852.0},
    }
    spatial_sets = {}
    case_sets = {}
    for role, expected in expected_ranges.items():
        cases = roles[role]["cases"]
        assert {case["payload"]["spatial"]["range_m"]["value"] for case in cases} == expected
        spatial_sets[role] = {spatial_group_key(case["payload"]) for case in cases}
        case_sets[role] = {case["case_id"] for case in cases}
    for index, left in enumerate(DATASET_ROLES):
        for right in DATASET_ROLES[index + 1 :]:
            assert spatial_sets[left].isdisjoint(spatial_sets[right])
            assert case_sets[left].isdisjoint(case_sets[right])


def test_same_twenty_five_profile_keys_have_exact_role_support() -> None:
    roles = build_qualification_bundle(_spec())["roles"]
    expected_support = {
        "calibration": 18,
        "validation": 9,
        "sealed_evaluation": 9,
    }
    profile_sets = []
    for role in DATASET_ROLES:
        counts = Counter(profile_key(case["payload"]) for case in roles[role]["cases"])
        assert len(counts) == 25
        assert set(counts.values()) == {expected_support[role]}
        profile_sets.append(set(counts))
    assert profile_sets[0] == profile_sets[1] == profile_sets[2]


def test_profile_key_uses_only_canonical_source_attitude_tuple() -> None:
    case = build_qualification_bundle(_spec())["roles"]["calibration"]["cases"][17]
    payload = case["payload"]
    altered = dict(payload)
    altered.update(
        {
            "range_m": 999999,
            "hostname": "transient-host",
            "created_utc": "never-part-of-profile-key",
            "case_id": "different",
        }
    )
    assert profile_key(payload) == profile_key(altered)


def test_prepare_plans_is_byte_deterministic_and_sealed_is_plan_only(tmp_path: Path) -> None:
    first = tmp_path / "first"
    second = tmp_path / "second"
    prepare_plans(first, _spec())
    prepare_plans(second, _spec())
    relative_files = (
        "qualification_plan.json",
        "calibration_plan/sweep_plan.json",
        "calibration_plan/sweep_cases.jsonl",
        "validation_plan/sweep_plan.json",
        "validation_plan/sweep_cases.jsonl",
        "sealed_plan/sealed_sweep_plan.json",
        "sealed_plan/sealed_sweep_cases.jsonl",
        "sealed_evaluation_commitment.json",
        "candidate_search_plan.json",
    )
    for relative in relative_files:
        assert (first / relative).read_bytes() == (second / relative).read_bytes()
    assert not (first / "sealed_plan" / "cases").exists()
    assert not list((first / "sealed_plan").rglob("attempt-*"))
    assert not list((first / "sealed_plan").rglob("pair_records.jsonl"))
