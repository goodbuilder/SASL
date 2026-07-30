from __future__ import annotations

import json
import math
import os
import sys
from pathlib import Path

import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "stereo_ground_sim"
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

pytest.importorskip("python_qt_binding")

from stereo_ground_sim import operator_panel  # noqa: E402


@pytest.fixture(scope="module")
def qt_app():
    from python_qt_binding.QtWidgets import QApplication

    app = QApplication.instance() or QApplication([])
    return app


def _valid_ray(**overrides):
    ray = {
        "valid": True,
        "computation_status": "computed",
        "failure_reason": "",
        "forward_m": 900.0,
        "right_m": 3.0,
        "down_m": -2.0,
        "range_m": 900.011,
        "gap_m": 0.000123,
        "intersection_angle_deg": 1.234,
    }
    ray.update(overrides)
    return ray


def test_ray_state_reports_valid_missing_and_bucketed_failures():
    assert operator_panel._ray_state(None) == (
        "MISSING",
        "No ray in the latest /stereo/ray_metrics snapshot.",
    )
    assert operator_panel._ray_state(_valid_ray())[0] == "VALID"

    stale = {"valid": False, "computation_status": "stale_left_attitude", "failure_reason": "stale_left_attitude: age 0.5s"}
    assert operator_panel._ray_state(stale) == ("STALE", "stale_left_attitude: age 0.5s")

    skew = {"valid": False, "computation_status": "attitude_pair_skew_exceeded", "failure_reason": ""}
    assert operator_panel._ray_state(skew)[0] == "SKEW"

    epoch = {"valid": False, "computation_status": "attitude_epoch_mismatch", "failure_reason": ""}
    assert operator_panel._ray_state(epoch)[0] == "EPOCH_MISMATCH"

    quat = {"valid": False, "computation_status": "invalid_px4_attitude", "failure_reason": ""}
    assert operator_panel._ray_state(quat)[0] == "INVALID_QUATERNION"

    gps = {"valid": False, "computation_status": "missing_left_gps_fix", "failure_reason": ""}
    assert operator_panel._ray_state(gps)[0] == "INVALID_BASELINE"

    other = {"valid": False, "computation_status": "frame_mismatch", "failure_reason": ""}
    assert operator_panel._ray_state(other) == ("FRAME_MISMATCH", "frame_mismatch")


def test_fmt_num_and_fmt_int_treat_nan_inf_and_none_as_not_available():
    assert operator_panel._fmt_num(None) == "—"
    assert operator_panel._fmt_num(math.nan) == "—"
    assert operator_panel._fmt_num(math.inf) == "—"
    assert operator_panel._fmt_num(-math.inf) == "—"
    assert operator_panel._fmt_num("not-a-number") == "—"
    assert operator_panel._fmt_num(0.0) == "0.000"
    assert operator_panel._fmt_num(0.0) != "—"
    assert operator_panel._fmt_num(1.23456, precision=6) == "1.234560"

    assert operator_panel._fmt_int(None) == "—"
    assert operator_panel._fmt_int("not-an-int") == "—"
    assert operator_panel._fmt_int(0) == "0"
    assert operator_panel._fmt_int(7) == "7"

    assert operator_panel._fmt_bool(None) == "—"
    assert operator_panel._fmt_bool(False) == "no"
    assert operator_panel._fmt_bool(True) == "yes"


def _cell(table, row, column):
    item = table.item(row, column)
    return "" if item is None else item.text()


def test_panel_renders_all_eight_ray_rows_with_correct_labels_and_order(qt_app):
    panel = operator_panel.RayDiagnosticsPanel()

    labels = [_cell(panel._geometry_table, row, 0) for row in range(panel._geometry_table.rowCount())]

    assert labels == ["Raw", "Truth", "Ideal", "GPS", "Global", "Attitude", "Baseline", "Full"]


def test_panel_shows_valid_ray_numbers_and_not_available_for_invalid_ray(qt_app):
    panel = operator_panel.RayDiagnosticsPanel()
    payload = {
        "pair_id": 11,
        "nominal": _valid_ray(),
        "baseline": {
            "valid": False,
            "computation_status": "missing_left_gps_fix",
            "failure_reason": "missing_left_gps_fix",
        },
    }

    panel.update_ray_metrics(json.dumps(payload))

    raw_row = 0
    assert _cell(panel._geometry_table, raw_row, 1) == "VALID"
    assert _cell(panel._geometry_table, raw_row, 2) == "900.000"
    assert _cell(panel._geometry_table, raw_row, 3) == "3.000"
    assert _cell(panel._geometry_table, raw_row, 4) == "-2.000"
    assert _cell(panel._geometry_table, raw_row, 6) == "0.000123"

    baseline_row = 6
    assert _cell(panel._geometry_table, baseline_row, 1) == "INVALID_BASELINE"
    assert _cell(panel._geometry_table, baseline_row, 2) == "—"
    assert _cell(panel._geometry_table, baseline_row, 2) != "0.000"


def test_panel_ignores_malformed_payload_without_raising(qt_app):
    panel = operator_panel.RayDiagnosticsPanel()

    panel.update_ray_metrics("not json")
    panel.update_ray_metrics(json.dumps([1, 2, 3]))
    panel.update_ray_metrics(
        json.dumps({"full": {"valid": True, "forward_m": None, "gap_m": math.nan}})
    )

    full_row = 7
    assert _cell(panel._geometry_table, full_row, 2) == "—"
    assert _cell(panel._geometry_table, full_row, 6) == "—"


def test_panel_clears_stale_numbers_when_next_pair_is_invalid(qt_app):
    panel = operator_panel.RayDiagnosticsPanel()
    panel.update_ray_metrics(
        json.dumps({"pair_id": 5, "full": _valid_ray(forward_m=500.0)})
    )
    full_row = 7
    assert _cell(panel._geometry_table, full_row, 2) == "500.000"
    assert _cell(panel._geometry_table, full_row, 1) == "VALID"

    panel.update_ray_metrics(
        json.dumps(
            {
                "pair_id": 6,
                "full": {
                    "valid": False,
                    "computation_status": "invalid_px4_attitude",
                    "failure_reason": "invalid_px4_attitude",
                },
            }
        )
    )

    assert _cell(panel._geometry_table, full_row, 2) == "—"
    assert _cell(panel._geometry_table, full_row, 1) == "INVALID_QUATERNION"


def test_panel_separates_truth_delta_gt_error_and_interaction(qt_app):
    panel = operator_panel.RayDiagnosticsPanel()
    payload = {
        "pair_id": 9,
        "attitude": {
            "valid": True,
            "computation_status": "computed",
            "delta_valid": True,
            "delta_forward_m": 0.25,
            "delta_right_m": 0.0,
            "delta_down_m": 0.0,
            "delta_range_m": 0.25,
            "position_difference_m": 0.25,
            "delta_failure_reason": "",
            "gt_valid": True,
            "gt_forward_error_m": 0.05,
            "gt_right_error_m": 0.0,
            "gt_down_error_m": 0.0,
            "gt_range_error_m": 0.05,
            "gt_position_error_m": 0.05,
            "gt_failure_reason": "",
        },
        "interaction": {
            "valid": True,
            "forward_m": 0.01,
            "right_m": 0.0,
            "down_m": 0.0,
            "position_m": 0.01,
            "failure_reason": "",
        },
        "gt_source": "gazebo_vehicle_reference_link_world_pose",
        "gt_fallback_used": False,
    }

    panel.update_ray_metrics(json.dumps(payload))

    assert _cell(panel._delta_table, 0, 0) == "Attitude"
    assert _cell(panel._delta_table, 0, 2) == "0.250"
    assert _cell(panel._gt_table, 0, 2) == "0.050"
    assert panel._interaction_labels["Magnitude [m]"].text() == "0.010"
    assert "not a Ray" in panel._interaction_box.title()
    assert panel._gt_source_label.text().startswith("GT source: gazebo_vehicle_reference_link_world_pose")


def test_panel_frame_and_unit_labels_are_present(qt_app):
    panel = operator_panel.RayDiagnosticsPanel()

    text = panel._frame_label.text()

    assert "baseline_center_frd" in text
    assert "Forward" in text
    assert "Right" in text
    assert "Down" in text
    assert " m " in text or text.endswith(" m") or "m " in text
    assert "deg" in text


def test_panel_pair_label_updates_from_payload(qt_app):
    panel = operator_panel.RayDiagnosticsPanel()

    panel.update_ray_metrics(json.dumps({"pair_id": 123, "nominal": _valid_ray()}))

    assert "Pair: 123" in panel._frame_label.text()
