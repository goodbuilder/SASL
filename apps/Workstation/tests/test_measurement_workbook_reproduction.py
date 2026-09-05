"""The workbook alone must reproduce Forward/Right/Down/Range."""

from __future__ import annotations

from datetime import timedelta
from pathlib import Path

from Workstation.workstation_core.measurement_methods import MEASUREMENT_METHOD_IDS
from Workstation.workstation_core.measurement_record import MeasurementRegistry
from Workstation.workstation_core.measurement_reproduce import (
    reproduce_workbook,
    reproduce_workbook_exact,
)
from Workstation.workstation_core.measurement_workbook import write_measurement_workbook
from Workstation.tests.measurement_fixtures import (
    MEASUREMENT_ID,
    STARTED_LOCAL,
    checkerboard_attempt,
    frozen_snapshot,
    outcome_for,
    pixhawk_attempt,
)


def _workbook(tmp_path: Path) -> Path:
    clock = iter(STARTED_LOCAL + timedelta(seconds=index) for index in range(100))
    registry = MeasurementRegistry(data_root_provider=lambda: tmp_path, clock_local=lambda: next(clock))
    record = registry.begin(MEASUREMENT_ID, started_utc_ns=1, started_monotonic_ns=1_000_000)
    attempts = [
        checkerboard_attempt(1),
        checkerboard_attempt(2, left_x=300.0, right_x=340.0, timestamp_s=2.0),  # negative depth
        checkerboard_attempt(3, left_x=350.0, right_x=250.0, timestamp_s=3.0),
        pixhawk_attempt(4, timestamp_s=4.0),
    ]
    for attempt in attempts:
        snapshot = frozen_snapshot(attempt)
        record.register_pair(
            snapshot.canonical_payload(),
            submitted_method_ids=(attempt.measurement_method_id,),
            not_attempted={m: "not ready" for m in MEASUREMENT_METHOD_IDS if m != attempt.measurement_method_id},
        )
        registry.route_outcome(outcome_for(attempt, snapshot))
    registry.stop(MEASUREMENT_ID, stopped_utc_ns=9, stopped_monotonic_ns=9_000_000)
    return write_measurement_workbook(record.snapshot(), record.workbook_path)


def test_exact_sheet_reproduces_every_result_bit_for_bit(tmp_path: Path) -> None:
    results = reproduce_workbook_exact(_workbook(tmp_path))

    assert {(item.pair_id[-1], item.method_id) for item in results} == {
        ("1", "checkerboard_stereo"),
        ("2", "checkerboard_stereo"),
        ("3", "checkerboard_stereo"),
        ("4", "pixhawk_rtk_live"),
    }
    for item in results:
        assert item.reason == "ok"
        assert all(value is not None for value in item.stored)
        assert item.bit_equal, (item.method_id, item.stored, item.reproduced)
    negative = next(item for item in results if item.pair_id.endswith("2"))
    assert negative.reproduced[3] == negative.stored[3]


def test_decimal_cells_reproduce_within_numerical_tolerance(tmp_path: Path) -> None:
    results = reproduce_workbook(_workbook(tmp_path))

    assert len(results) == 4
    for item in results:
        assert item.reason == "ok"
        assert item.close(rel_tol=1e-9, abs_tol=1e-9), (item.method_id, item.stored, item.reproduced)
