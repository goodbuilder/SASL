from __future__ import annotations

from decimal import Decimal
from pathlib import Path
import sys

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "stereo_ground_sim"
sys.path.insert(0, str(PACKAGE_ROOT))

from stereo_ground_sim.correction_sweep import (
    ATTITUDE_AXES,
    CameraContract,
    build_attitude_cases,
    build_sweep_plan,
    parse_attitude_axis_range,
    parse_numeric_values,
    write_plan,
)


def _plan(**changes):
    values = {
        "sweep_id": "sweep",
        "commit_sha": "a" * 40,
        "branch": "sim/test",
        "repository_name": "goodbuilder/SASL",
        "attitude_mode": "ofat",
        "axis_ranges": [parse_attitude_axis_range("common_optical_roll_error_deg:0.1:0.1:0.1")],
        "range_values": parse_numeric_values("500", name="range"),
        "bearing_values": parse_numeric_values("-1,1", name="bearing"),
        "elevation_values": parse_numeric_values("-0.5,0.5", name="elevation"),
        "repetitions": 1,
        "seed": 20260722,
        "case_order": "canonical",
        "invalid_spatial_policy": "fail",
        "max_cases": 16,
        "contract": CameraContract(),
    }
    values.update(changes)
    return build_sweep_plan(**values)


def test_exact_twelve_attitude_axes_and_unknown_axis_rejected() -> None:
    assert len(ATTITUDE_AXES) == 12
    assert len(set(ATTITUDE_AXES)) == 12
    with pytest.raises(ValueError, match="unknown attitude axis"):
        parse_attitude_axis_range("bogus:0:1:1")


def test_decimal_range_endpoint_dedup_and_negative_zero() -> None:
    assert tuple(map(str, parse_numeric_values("0.1:0.3:0.1", name="x"))) == (
        "0.1",
        "0.2",
        "0.3",
    )
    assert parse_numeric_values("-0,0,1,1.0", name="x") == (
        Decimal("0"),
        Decimal("1"),
    )
    with pytest.raises(ValueError, match="exactly reachable"):
        parse_numeric_values("0:1:0.3", name="x")


@pytest.mark.parametrize("spec", ("0:1:0", "1:0:0.1", "NaN", "Inf"))
def test_decimal_invalid_ranges_rejected(spec: str) -> None:
    with pytest.raises(ValueError):
        parse_numeric_values(spec, name="x")


def test_ofat_has_one_baseline_and_one_nonzero_source_per_case() -> None:
    cases = build_attitude_cases(
        (
            parse_attitude_axis_range("left_roll_error_deg:-0.1:0.1:0.1"),
            parse_attitude_axis_range("right_pitch_error_deg:0:0.2:0.2"),
        ),
        mode="ofat",
    )
    assert sum(not any(item.values()) for item in cases) == 1
    assert all(sum(value != 0 for value in item.values()) <= 1 for item in cases)
    assert len(cases) == 4


def test_cartesian_product_includes_baseline_once() -> None:
    cases = build_attitude_cases(
        (
            ("left_roll_error_deg", (Decimal("0.1"), Decimal("0.2"))),
            ("right_yaw_error_deg", (Decimal("-0.1"), Decimal("0.1"))),
        ),
        mode="cartesian",
    )
    assert len(cases) == 5
    assert sum(not any(item.values()) for item in cases) == 1
    assert all(item["common_optical_yaw_error_deg"] == 0 for item in cases)


def test_duplicate_axis_and_max_case_guard_rejected() -> None:
    duplicate = parse_attitude_axis_range("left_roll_error_deg:0:0:1")
    with pytest.raises(ValueError, match="duplicate"):
        build_attitude_cases((duplicate, duplicate), mode="ofat")
    with pytest.raises(ValueError, match="exceeds max_cases"):
        _plan(max_cases=7)


def test_qualification_plan_counts_ids_and_hash_are_stable() -> None:
    first_plan, first_cases = _plan()
    second_plan, second_cases = _plan()
    assert first_plan == second_plan
    assert first_plan["attitude_case_count"] == 2
    assert first_plan["spatial_cell_count"] == 4
    assert first_plan["total_runnable"] == 8
    assert [case["case_id"] for case in first_cases] == [case["case_id"] for case in second_cases]
    assert len({case["case_id"] for case in first_cases}) == 8


def test_repetition_and_seeded_order_change_execution_only() -> None:
    canonical_plan, canonical_cases = _plan(repetitions=2, max_cases=32)
    seeded_plan, seeded_cases = _plan(
        repetitions=2, max_cases=32, case_order="seeded"
    )
    assert {case["case_id"] for case in canonical_cases} == {
        case["case_id"] for case in seeded_cases
    }
    assert [case["case_id"] for case in canonical_cases] != seeded_plan[
        "case_ids_in_execution_order"
    ]
    assert sorted(case["repetition_index"] for case in (item["payload"] for item in canonical_cases)) == [0] * 8 + [1] * 8
    again, _ = _plan(repetitions=2, max_cases=32, case_order="seeded")
    assert seeded_plan["case_ids_in_execution_order"] == again["case_ids_in_execution_order"]


def test_plan_files_are_byte_identical_in_different_directories(tmp_path: Path) -> None:
    plan, cases = _plan()
    first = tmp_path / "first"
    second = tmp_path / "second"
    write_plan(first, plan, cases)
    write_plan(second, plan, cases)
    assert (first / "sweep_plan.json").read_bytes() == (second / "sweep_plan.json").read_bytes()
    assert (first / "sweep_cases.jsonl").read_bytes() == (second / "sweep_cases.jsonl").read_bytes()
