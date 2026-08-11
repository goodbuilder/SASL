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

    assert labels == [
        "Nominal Pose", "Optical Pose", "Ideal", "GPS Baseline", "Global Baseline",
        "Attitude", "Operational", "Attitude (True North)",
    ]


def test_panel_shows_valid_ray_numbers_and_not_available_for_invalid_ray(qt_app):
    panel = operator_panel.RayDiagnosticsPanel()
    payload = {
        "pair_id": 11,
        "nominal_pose": _valid_ray(),
        "gps_baseline": {
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

    baseline_row = 3
    assert _cell(panel._geometry_table, baseline_row, 1) == "INVALID_BASELINE"
    assert _cell(panel._geometry_table, baseline_row, 2) == "—"
    assert _cell(panel._geometry_table, baseline_row, 2) != "0.000"


def test_panel_ignores_malformed_payload_without_raising(qt_app):
    panel = operator_panel.RayDiagnosticsPanel()

    panel.update_ray_metrics("not json")
    panel.update_ray_metrics(json.dumps([1, 2, 3]))
    panel.update_ray_metrics(
        json.dumps({"operational": {"valid": True, "forward_m": None, "gap_m": math.nan}})
    )

    full_row = 6
    assert _cell(panel._geometry_table, full_row, 2) == "—"
    assert _cell(panel._geometry_table, full_row, 6) == "—"


def test_panel_clears_stale_numbers_when_next_pair_is_invalid(qt_app):
    panel = operator_panel.RayDiagnosticsPanel()
    panel.update_ray_metrics(
        json.dumps({"pair_id": 5, "operational": _valid_ray(forward_m=500.0)})
    )
    full_row = 6
    assert _cell(panel._geometry_table, full_row, 2) == "500.000"
    assert _cell(panel._geometry_table, full_row, 1) == "VALID"

    panel.update_ray_metrics(
        json.dumps(
            {
                "pair_id": 6,
                "operational": {
                    "valid": False,
                    "computation_status": "invalid_px4_attitude",
                    "failure_reason": "invalid_px4_attitude",
                },
            }
        )
    )

    assert _cell(panel._geometry_table, full_row, 2) == "—"
    assert _cell(panel._geometry_table, full_row, 1) == "INVALID_QUATERNION"


def test_panel_has_no_optical_pose_delta_table(qt_app):
    # The "Optical Pose Δ" tab (Truth-ray-relative delta) has been removed;
    # only Geometry, GT Error, and Sensors remain.
    panel = operator_panel.RayDiagnosticsPanel()

    assert not hasattr(panel, "_delta_table")


def test_panel_gt_error_covers_all_seven_rays(qt_app):
    panel = operator_panel.RayDiagnosticsPanel()
    payload = {
        "pair_id": 9,
        "attitude": {
            "valid": True,
            "computation_status": "computed",
            "gt_valid": True,
            "gt_forward_error_m": 0.05,
            "gt_right_error_m": 0.0,
            "gt_down_error_m": 0.0,
            "gt_range_error_m": 0.05,
            "gt_position_error_m": 0.05,
            "gt_failure_reason": "",
        },
        "gps_baseline": {
            "computation_status": "computed",
            "gt_valid": True,
            "gt_forward_error_m": 0.12,
            "gt_right_error_m": 0.0,
            "gt_down_error_m": 0.0,
            "gt_range_error_m": 0.12,
            "gt_position_error_m": 0.12,
            "gt_failure_reason": "",
        },
        "gt_source": "gazebo_vehicle_reference_link_world_pose",
        "gt_fallback_used": False,
    }

    panel.update_ray_metrics(json.dumps(payload))

    gps_baseline_row = 3
    attitude_row = 5
    assert _cell(panel._gt_table, gps_baseline_row, 0) == "GPS Baseline"
    assert _cell(panel._gt_table, gps_baseline_row, 2) == "0.120"
    assert _cell(panel._gt_table, attitude_row, 0) == "Attitude"
    assert _cell(panel._gt_table, attitude_row, 2) == "0.050"
    assert panel._gt_source_label.text().startswith("GT source: gazebo_vehicle_reference_link_world_pose")


def test_panel_frame_and_unit_summary_is_removed_from_ray_diagnostics(qt_app):
    panel = operator_panel.RayDiagnosticsPanel()

    assert not hasattr(panel, "_frame_label")


def test_panel_pair_id_payload_still_refreshes_geometry_without_summary_label(qt_app):
    panel = operator_panel.RayDiagnosticsPanel()

    panel.update_ray_metrics(json.dumps({"pair_id": 123, "nominal_pose": _valid_ray()}))

    assert _cell(panel._geometry_table, 0, 1) == "VALID"


def test_px4_attitude_env_values_read_from_environment(monkeypatch):
    monkeypatch.setenv("CAM_L_PX4_ATTITUDE_ROLL_DEG", "0.0")
    monkeypatch.setenv("CAM_L_PX4_ATTITUDE_PITCH_DEG", "0.0")
    monkeypatch.setenv("CAM_L_PX4_ATTITUDE_YAW_DEG", "0.0")
    monkeypatch.setenv("CAM_R_PX4_ATTITUDE_ROLL_DEG", "0.0")
    monkeypatch.setenv("CAM_R_PX4_ATTITUDE_PITCH_DEG", "0.0")
    monkeypatch.setenv("CAM_R_PX4_ATTITUDE_YAW_DEG", "0.05")

    values = operator_panel._px4_attitude_env_values()

    assert values["left"] == ("0.0", "0.0", "0.0")
    assert values["right"] == ("0.0", "0.0", "0.05")


def test_px4_attitude_env_values_default_to_zero(monkeypatch):
    for name in (
        "CAM_L_PX4_ATTITUDE_ROLL_DEG",
        "CAM_L_PX4_ATTITUDE_PITCH_DEG",
        "CAM_L_PX4_ATTITUDE_YAW_DEG",
        "CAM_R_PX4_ATTITUDE_ROLL_DEG",
        "CAM_R_PX4_ATTITUDE_PITCH_DEG",
        "CAM_R_PX4_ATTITUDE_YAW_DEG",
    ):
        monkeypatch.delenv(name, raising=False)

    values = operator_panel._px4_attitude_env_values()

    assert values["left"] == ("0.0", "0.0", "0.0")
    assert values["right"] == ("0.0", "0.0", "0.0")


def test_setup_group_shows_px4_attitude_values(qt_app, monkeypatch):
    monkeypatch.setenv("CAM_R_PX4_ATTITUDE_YAW_DEG", "0.05")

    panel = operator_panel.OperatorPanel.__new__(operator_panel.OperatorPanel)
    group = operator_panel.OperatorPanel._build_px4_attitude_group(panel)

    assert group.title() == "PX4 Attitude"
    assert "independent of the camera optical pose" in group.toolTip()
    assert "body frd" in group.toolTip().lower()


def test_sensors_tab_shows_residual_within_tolerance_without_warning(qt_app, monkeypatch):
    monkeypatch.setenv("CAM_L_PX4_ATTITUDE_ROLL_DEG", "0.0")
    monkeypatch.setenv("CAM_L_PX4_ATTITUDE_PITCH_DEG", "0.0")
    monkeypatch.setenv("CAM_L_PX4_ATTITUDE_YAW_DEG", "0.0")
    panel = operator_panel.RayDiagnosticsPanel()

    panel.update_ray_metrics(
        json.dumps(
            {
                "attitude": {
                    "valid": True,
                    "computation_status": "computed",
                    "left_attitude_truth_residual_deg": 1.0e-7,
                    "right_attitude_truth_residual_deg": 1.0e-7,
                }
            }
        )
    )

    assert panel._attitude_fields["Left residual [deg]"].text() != "—"
    assert panel._attitude_fields["Left residual [deg]"].toolTip() == ""


def test_sensors_tab_flags_reference_mismatch_when_error_is_zero_but_residual_is_large(
    qt_app, monkeypatch
):
    monkeypatch.setenv("CAM_L_PX4_ATTITUDE_ROLL_DEG", "0.0")
    monkeypatch.setenv("CAM_L_PX4_ATTITUDE_PITCH_DEG", "0.0")
    monkeypatch.setenv("CAM_L_PX4_ATTITUDE_YAW_DEG", "0.0")
    panel = operator_panel.RayDiagnosticsPanel()

    panel.update_ray_metrics(
        json.dumps(
            {
                "attitude": {
                    "valid": True,
                    "computation_status": "computed",
                    "left_attitude_truth_residual_deg": 1.5,
                    "right_attitude_truth_residual_deg": 0.0,
                }
            }
        )
    )

    assert "REFERENCE_MISMATCH" in panel._attitude_fields["Left residual [deg]"].toolTip()
    # The runtime computation_status is untouched by this UI-only flag.
    assert panel._attitude_fields["Overall status"].text() == "VALID"


def test_sensors_tab_does_not_flag_reference_mismatch_when_error_is_injected(
    qt_app, monkeypatch
):
    monkeypatch.setenv("CAM_R_PX4_ATTITUDE_YAW_DEG", "1.5")
    panel = operator_panel.RayDiagnosticsPanel()

    panel.update_ray_metrics(
        json.dumps(
            {
                "attitude": {
                    "valid": True,
                    "computation_status": "computed",
                    "left_attitude_truth_residual_deg": 0.0,
                    "right_attitude_truth_residual_deg": 1.5,
                }
            }
        )
    )

    assert panel._attitude_fields["Right residual [deg]"].toolTip() == ""


def test_ray_diagnostics_tabs_do_not_force_a_tall_minimum_window(qt_app):
    """Regression test: QTabWidget sizes for the union of every page (Qt's
    internal QStackedWidget considers all pages, not just the current one),
    so the Sensors tab's many-row PX4/GPS health grids used to inflate this
    whole panel's minimumSizeHint to ~860px tall even while showing the
    small Geometry tab. That minimum propagated to the top-level operator
    window and, once it exceeded the screen height, several window managers
    could no longer maximize it (there was no smaller "restored" state left
    to grow from). Each tab is now wrapped in a resizable QScrollArea, whose
    own minimumSizeHint stays small regardless of the wrapped content."""

    panel = operator_panel.RayDiagnosticsPanel()
    panel.show()
    qt_app.processEvents()

    assert panel.minimumSizeHint().height() < 400


def test_ray_diagnostics_group_has_no_collapse_toggle(qt_app):
    panel = operator_panel.RayDiagnosticsPanel()
    panel.show()
    qt_app.processEvents()

    from python_qt_binding.QtWidgets import QGroupBox

    group = panel.findChild(QGroupBox)
    assert group is not None
    assert group.isCheckable() is False
    assert panel.minimumSizeHint().height() < 400


def test_ray_diagnostics_renders_at_10_hz_with_fixed_horizontal_spacing(qt_app):
    panel = operator_panel.RayDiagnosticsPanel()

    assert operator_panel.RAY_DIAGNOSTICS_RENDER_HZ == 10.0
    assert panel._render_timer.interval() == 100
    assert operator_panel.RAY_DIAGNOSTICS_HORIZONTAL_SPACING == 8


def test_analysis_tab_description_does_not_force_a_wide_minimum_window(qt_app):
    """Regression test: an unwrapped single-line QLabel's sizeHint is as wide
    as its text needs to render on one line -- the Analysis tab's Ray-list
    description is long enough (~3260px unwrapped) to push the whole
    top-level window's minimum width past most screens, the same
    union-of-pages-sizeHint failure mode as
    test_ray_diagnostics_tabs_do_not_force_a_tall_minimum_window, just on the
    width axis instead of height."""

    panel = operator_panel.OperatorPanel.__new__(operator_panel.OperatorPanel)
    panel._plot_stack = operator_panel.FrdPlotStack()
    tab = operator_panel.OperatorPanel._build_analysis_tab(panel)
    tab.show()
    qt_app.processEvents()

    assert tab.minimumSizeHint().width() < 1000


def test_analysis_graph_legend_matches_the_saved_ray_names():
    # These four series ARE recorded Rays; their legend text must exactly
    # match RAY_DIAGNOSTICS_ROWS / the CSV-XLSX Ray display names.
    ray_display_names = dict(operator_panel.RAY_DIAGNOSTICS_ROWS)
    assert operator_panel.SERIES_LABELS["nominal_pose"] == ray_display_names["nominal_pose"]
    assert operator_panel.SERIES_LABELS["optical_pose"] == ray_display_names["optical_pose"]
    assert operator_panel.SERIES_LABELS["ideal"] == ray_display_names["ideal"]
    assert operator_panel.SERIES_LABELS["gps_baseline"] == ray_display_names["gps_baseline"]


def test_analysis_graph_legend_does_not_collide_gt_with_the_truth_ray():
    # "truth" subscribes to /ground_truth_position (the literal Gazebo
    # target GT), which is not one of the eight recorded Rays and must not
    # render as a near-duplicate of the "optical" series' "Truth" Ray label.
    assert operator_panel.SERIES_LABELS["truth"] != "truth"
    assert operator_panel.SERIES_LABELS["truth"].lower() != "truth"
    assert operator_panel.SERIES_LABELS["truth"] != operator_panel.SERIES_LABELS["optical_pose"]


def test_analysis_tab_series_covers_all_eight_recorded_rays():
    # RAY_DIAGNOSTICS_ROWS is the canonical list of the eight recorded Rays.
    # The Analysis tab's SERIES must plot every one of them, under the same
    # display name it uses elsewhere (the Ray Diagnostics panel, CSV/XLSX).
    ray_display_names = dict(operator_panel.RAY_DIAGNOSTICS_ROWS)
    series_key_by_ray_key = {
        "nominal_pose": "nominal_pose",
        "optical_pose": "optical_pose",
        "ideal": "ideal",
        "gps_baseline": "gps_baseline",
        "global_baseline": "global_baseline",
        "attitude": "attitude",
        "operational": "operational",
        "attitude_true_north": "attitude_true_north",
    }
    series_names = {name for name, _topic, _color in operator_panel.SERIES}
    assert set(series_key_by_ray_key.values()) <= series_names
    for ray_key, display_name in ray_display_names.items():
        series_key = series_key_by_ray_key[ray_key]
        assert operator_panel.SERIES_LABELS[series_key] == display_name


def test_analysis_tab_series_without_a_dedicated_topic_are_skipped_in_ros_init():
    # gps_global/attitude/baseline/full have no PointStamped/Float32MultiArray
    # position topic of their own -- _init_ros's per-topic subscription loop
    # must skip them (topic is None) rather than crash trying to subscribe to
    # a None topic name; they are fed from /stereo/ray_metrics instead.
    for series_name in ("global_baseline", "attitude", "operational"):
        matches = [entry for entry in operator_panel.SERIES if entry[0] == series_name]
        assert len(matches) == 1
        assert matches[0][1] is None


def test_plot_stack_accepts_ray_metrics_only_series(qt_app):
    stack = operator_panel.FrdPlotStack()
    for series_name in ("global_baseline", "attitude", "operational"):
        stack.append(series_name, [900.0, 3.0, -2.0, 900.01])
        axis_name, _axis_index = operator_panel.AXES[0]
        assert stack.samples[axis_name][series_name][-1][1] == pytest.approx(900.0)


def test_analysis_series_checkboxes_control_plot_visibility(qt_app):
    stack = operator_panel.FrdPlotStack()
    assert stack.is_series_visible("truth") is True

    stack.set_series_visible("truth", False)
    assert stack.is_series_visible("truth") is False
    assert stack.is_series_visible("nominal_pose") is True

    stack.set_series_visible("unknown", True)
    assert stack.is_series_visible("unknown") is False


def test_analysis_tab_builds_checked_series_selectors(qt_app):
    panel = operator_panel.OperatorPanel.__new__(operator_panel.OperatorPanel)
    panel._plot_stack = operator_panel.FrdPlotStack()
    tab = operator_panel.OperatorPanel._build_analysis_tab(panel)

    assert set(panel._plot_series_checks) == {
        series_name for series_name, _topic, _color in operator_panel.SERIES
    }
    assert all(checkbox.isChecked() for checkbox in panel._plot_series_checks.values())
    panel._plot_series_checks["truth"].setChecked(False)
    assert panel._plot_stack.is_series_visible("truth") is False
    tab.deleteLater()


def test_handle_ray_metrics_feeds_the_plot_stack_for_the_four_json_only_rays(qt_app, monkeypatch):
    panel = operator_panel.OperatorPanel.__new__(operator_panel.OperatorPanel)
    panel._plot_stack = operator_panel.FrdPlotStack()
    panel._facts = None
    panel._record_preview = None
    panel._ray_diagnostics = None
    monkeypatch.setattr(panel, "_facts", type("_F", (), {"update_sample": lambda *a, **k: None})())
    monkeypatch.setattr(
        panel,
        "_record_preview",
        type(
            "_R",
            (),
            {
                "update_live_ray_metrics": lambda *a, **k: None,
                "update_live_sample": lambda *a, **k: None,
            },
        )(),
    )
    monkeypatch.setattr(
        panel, "_ray_diagnostics", type("_D", (), {"update_ray_metrics": lambda *a, **k: None})()
    )
    payload = json.dumps(
        {
            "global_baseline": _valid_ray(forward_m=1.0),
            "attitude": _valid_ray(forward_m=2.0),
            "operational": _valid_ray(forward_m=4.0),
        }
    )
    operator_panel.OperatorPanel._handle_ray_metrics(panel, payload)
    axis_name, _axis_index = operator_panel.AXES[0]
    assert panel._plot_stack.samples[axis_name]["global_baseline"][-1][1] == pytest.approx(1.0)
    assert panel._plot_stack.samples[axis_name]["attitude"][-1][1] == pytest.approx(2.0)
    assert panel._plot_stack.samples[axis_name]["operational"][-1][1] == pytest.approx(4.0)


def test_handle_ray_metrics_does_not_crash_on_null_valued_fields(qt_app, monkeypatch):
    # Regression test: an invalid/failed ray still reports its forward_m/
    # right_m/down_m/range_m keys, just with a JSON null (-> None) rather
    # than omitting them. dict.get(key, math.nan) only substitutes when the
    # key is *absent*, not when it's present and None, so a naive .get(key,
    # math.nan) let None reach math.isfinite() in _handle_sample and crashed
    # the whole panel (and, transitively, run_operator_view.sh's process
    # supervisor) the first time a live ray actually went invalid.
    panel = operator_panel.OperatorPanel.__new__(operator_panel.OperatorPanel)
    panel._plot_stack = operator_panel.FrdPlotStack()
    panel._facts = None
    panel._record_preview = None
    panel._ray_diagnostics = None
    monkeypatch.setattr(panel, "_facts", type("_F", (), {"update_sample": lambda *a, **k: None})())
    monkeypatch.setattr(
        panel,
        "_record_preview",
        type(
            "_R",
            (),
            {
                "update_live_ray_metrics": lambda *a, **k: None,
                "update_live_sample": lambda *a, **k: None,
            },
        )(),
    )
    monkeypatch.setattr(
        panel, "_ray_diagnostics", type("_D", (), {"update_ray_metrics": lambda *a, **k: None})()
    )
    payload = json.dumps(
        {
            "global_baseline": _valid_ray(
                forward_m=None, right_m=None, down_m=None, range_m=None
            ),
            "attitude": _valid_ray(forward_m=5.0),
            "operational": _valid_ray(forward_m=None, range_m=None),
        }
    )
    operator_panel.OperatorPanel._handle_ray_metrics(panel, payload)  # must not raise
    axis_name, _axis_index = operator_panel.AXES[0]
    assert math.isnan(panel._plot_stack.samples[axis_name]["global_baseline"][-1][1])
    assert panel._plot_stack.samples[axis_name]["attitude"][-1][1] == pytest.approx(5.0)
    assert math.isnan(panel._plot_stack.samples[axis_name]["operational"][-1][1])
