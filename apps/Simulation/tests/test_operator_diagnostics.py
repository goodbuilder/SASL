import math
from types import SimpleNamespace

import pytest

from stereo_ground_sim.operator_diagnostics import (
    OperatorPairDiagnostics,
    build_pair_diagnostics_from_terminal_record,
)


def pair(**changes):
    values = {"epoch": 0, "pair_id": 1, "pair_midpoint_stamp_ns": 20}
    values.update(changes)
    return SimpleNamespace(**values)


def record(**changes):
    values = {
        "epoch": 0, "pair_id": 1, "pair_midpoint_stamp_ns": 20,
        "terminal_status": "evaluated", "measurement_status": "processed",
        "evaluation_status": "evaluated", "output_frame": "baseline_center_frd",
        "position_unit": "m", "angle_unit": "deg",
        "raw_range_error_m": 1, "raw_forward_error_m": 0.0,
    }
    values.update(changes)
    return values


def test_runtime_shaped_record_and_identity_binding():
    snapshot = build_pair_diagnostics_from_terminal_record(pair=pair(), terminal_record=record())
    assert snapshot is not None
    assert snapshot.pair_id == 1
    assert snapshot.values == (("raw_forward_error_m", 0.0), ("raw_range_error_m", 1.0))
    assert build_pair_diagnostics_from_terminal_record(pair=pair(pair_id=2), terminal_record=record()) is None
    assert build_pair_diagnostics_from_terminal_record(pair=pair(), terminal_record=record(epoch=1)) is None
    assert build_pair_diagnostics_from_terminal_record(pair=pair(), terminal_record=record(pair_midpoint_stamp_ns=21)) is None


@pytest.mark.parametrize("changes", [
    {"pair_id": 0}, {"pair_id": -1}, {"pair_id": True}, {"pair_id": "1"},
    {"epoch": -1}, {"epoch": True}, {"pair_midpoint_stamp_ns": -1},
    {"pair_midpoint_stamp_ns": False},
])
def test_pair_identity_is_positive_integer_contract(changes):
    assert build_pair_diagnostics_from_terminal_record(pair=pair(**changes), terminal_record=record()) is None


@pytest.mark.parametrize("changes", [
    {"terminal_status": "valid"}, {"terminal_status": "gt_unavailable"},
    {"terminal_status": "evaluation_error"}, {"measurement_status": "discarded"},
    {"measurement_status": "unprocessed"}, {"evaluation_status": "unavailable"},
    {"evaluation_status": "not_evaluated"}, {"evaluation_status": None},
])
def test_only_evaluated_terminal_status_is_accepted(changes):
    assert build_pair_diagnostics_from_terminal_record(pair=pair(), terminal_record=record(**changes)) is None


@pytest.mark.parametrize("changes", [
    {"output_frame": "gazebo_world_enu"}, {"position_unit": "cm"},
    {"angle_unit": "rad"}, {"output_frame": None},
])
def test_coordinate_contract_is_exact(changes):
    assert build_pair_diagnostics_from_terminal_record(pair=pair(), terminal_record=record(**changes)) is None


def test_individual_invalid_metrics_are_omitted_but_zero_is_kept():
    snapshot = build_pair_diagnostics_from_terminal_record(
        pair=pair(), terminal_record=record(
            raw_range_error_m=None, raw_forward_error_m=0,
            corrected_forward_error_m=math.nan, corrected_right_error_m=math.inf,
            corrected_down_error_m=True, raw_down_error_m="bad",
        )
    )
    assert snapshot is not None
    assert snapshot.values == (("raw_forward_error_m", 0.0),)
    assert build_pair_diagnostics_from_terminal_record(
        pair=pair(), terminal_record=record(raw_range_error_m=None, raw_forward_error_m=None)
    ) is None


def test_snapshot_is_deeply_immutable_and_ordered():
    source = record(raw_range_error_m=2, corrected_position_error_m=3)
    snapshot = build_pair_diagnostics_from_terminal_record(pair=pair(), terminal_record=source)
    source["raw_range_error_m"] = 99
    assert snapshot.values == (
        ("corrected_position_error_m", 3.0),
        ("raw_forward_error_m", 0.0),
        ("raw_range_error_m", 2.0),
    )
    copied = snapshot.value_map()
    copied["raw_range_error_m"] = 0
    assert snapshot.value_map()["raw_range_error_m"] == 2.0
    with pytest.raises(TypeError):
        snapshot.values[0] = ("raw_position_error_m", 1.0)


@pytest.mark.parametrize("values", [
    (), [], [("raw_range_error_m", 1.0)], (("raw_range_error_m", 1.0, 2.0),),
    (("unknown", 1.0),), (("raw_range_error_m", 1.0), ("raw_range_error_m", 2.0)),
    (("raw_range_error_m", 1),), (("raw_range_error_m", True),),
    (("raw_range_error_m", math.nan),),
])
def test_direct_constructor_is_fail_closed(values):
    with pytest.raises((ValueError, TypeError)):
        OperatorPairDiagnostics(0, 1, 20, "baseline_center_frd", "m", "deg", values)


def test_direct_constructor_accepts_only_allowlist_ordered_nonempty_subset():
    snapshot = OperatorPairDiagnostics(
        0, 1, 20, "baseline_center_frd", "m", "deg",
        (("raw_forward_error_m", 0.0), ("raw_range_error_m", 1.0)),
    )
    assert snapshot.value_map() == {"raw_forward_error_m": 0.0, "raw_range_error_m": 1.0}
