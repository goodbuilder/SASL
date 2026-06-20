import csv
import json
from pathlib import Path

from wiris_stereo_distance.simulation_origin import (
    OriginBboxProfile,
    aggregate_summary_files,
    build_origin_bbox_reprojection_profile_from_telemetry,
    build_origin_bbox_profile_from_telemetry,
    build_profile_from_telemetry,
    evaluate_origin_bbox_reprojection_profile_rows,
    evaluate_origin_bbox_profile_rows,
    evaluate_profile_rows,
    generate_random_origin_waypoints,
    generate_origin_waypoints,
    save_profile,
    simulation_comparison_rows,
)


def test_generate_origin_waypoints_respects_full_overlap_margin() -> None:
    waypoints = generate_origin_waypoints(
        drone_env={"DRONE_SPAWN_Y_M": "2000.0"},
        camera_env={
            "CAMERA_BASELINE_M": "25.0",
            "CAMERA_HEIGHT_M": "10.0",
            "CAMERA_HORIZONTAL_FOV_RAD": "0.154997710698",
        },
    )

    assert len(waypoints) == 6
    assert waypoints[-1]["range_m"] == 320.0
    assert abs(waypoints[-1]["east"]) <= waypoints[-1]["max_full_overlap_center_m"]
    assert {round(waypoint["altitude"], 1) for waypoint in waypoints} >= {8.0, 10.0, 12.0}


def test_generate_random_origin_waypoints_are_seeded_and_overlap_safe() -> None:
    common = {
        "drone_env": {"DRONE_SPAWN_Y_M": "1500.0"},
        "camera_env": {
            "CAMERA_BASELINE_M": "25.0",
            "CAMERA_HEIGHT_M": "10.0",
            "CAMERA_HORIZONTAL_FOV_RAD": "0.154997710698",
        },
        "range_min_m": 320.0,
        "range_max_m": 1100.0,
        "waypoint_count": 6,
    }

    first = generate_random_origin_waypoints(seed=42, **common)
    repeated = generate_random_origin_waypoints(seed=42, **common)
    second = generate_random_origin_waypoints(seed=43, **common)

    assert first == repeated
    assert first != second
    assert [waypoint["range_m"] for waypoint in first] == sorted(
        (waypoint["range_m"] for waypoint in first),
        reverse=True,
    )
    assert any(waypoint["planned_right_m"] > 0.0 for waypoint in first)
    assert any(waypoint["planned_right_m"] < 0.0 for waypoint in first)
    for waypoint in first:
        assert 320.0 <= waypoint["range_m"] <= 1100.0
        assert abs(waypoint["planned_right_m"]) <= waypoint["max_full_overlap_center_m"]


def test_simulation_origin_profile_and_evaluation_use_right_down_forward(tmp_path: Path) -> None:
    telemetry = tmp_path / "telemetry.csv"
    rows = [
        {
            "time_sec": "1.0",
            "estimated_right_m": "1.0",
            "estimated_down_m": "0.0",
            "estimated_forward_m": "0.0",
            "estimated_range_m": "1.0",
            "corrected_right_m": "11.0",
            "corrected_down_m": "20.0",
            "corrected_forward_m": "30.0",
            "corrected_range_m": "37.749172",
            "drone_right_m": "11.0",
            "drone_down_m": "20.0",
            "drone_forward_m": "30.0",
            "ground_truth_m": "37.749172",
            "left_cx": "600.0",
            "left_cy": "350.0",
            "right_cx": "610.0",
            "right_cy": "350.0",
        },
        {
            "time_sec": "2.0",
            "estimated_right_m": "0.0",
            "estimated_down_m": "1.0",
            "estimated_forward_m": "0.0",
            "estimated_range_m": "1.0",
            "corrected_right_m": "10.0",
            "corrected_down_m": "21.0",
            "corrected_forward_m": "30.0",
            "corrected_range_m": "38.013156",
            "drone_right_m": "10.0",
            "drone_down_m": "21.0",
            "drone_forward_m": "30.0",
            "ground_truth_m": "38.013156",
            "left_cx": "620.0",
            "left_cy": "360.0",
            "right_cx": "630.0",
            "right_cy": "360.0",
        },
        {
            "time_sec": "3.0",
            "estimated_right_m": "0.0",
            "estimated_down_m": "0.0",
            "estimated_forward_m": "1.0",
            "estimated_range_m": "1.0",
            "corrected_right_m": "10.0",
            "corrected_down_m": "20.0",
            "corrected_forward_m": "31.0",
            "corrected_range_m": "38.431758",
            "drone_right_m": "10.0",
            "drone_down_m": "20.0",
            "drone_forward_m": "31.0",
            "ground_truth_m": "38.431758",
            "left_cx": "640.0",
            "left_cy": "370.0",
            "right_cx": "650.0",
            "right_cy": "370.0",
        },
    ]
    with telemetry.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    profile, comparison_rows = build_profile_from_telemetry(
        telemetry,
        profile_id="simulation_test",
        min_samples=3,
    )
    evaluated, summary = evaluate_profile_rows(comparison_rows, profile)
    profile_path = tmp_path / "origin_profile.json"
    save_profile(profile_path, profile)

    payload = json.loads(profile_path.read_text(encoding="utf-8"))
    assert payload["measurement_frame"] == "stereo_center_opencv_optical"
    assert payload["truth_frame"] == "stereo_center_opencv_optical"
    assert comparison_rows[0]["stereo_center_right_m"] == "1.000000"
    assert comparison_rows[0]["measurement_right_m"] == "1.000000"
    assert comparison_rows[0]["measurement_frd_forward_m"] == "0.000000"
    assert comparison_rows[0]["measurement_frd_right_m"] == "1.000000"
    assert comparison_rows[0]["measurement_frd_down_m"] == "0.000000"
    assert comparison_rows[0]["truth_right_m"] == "11.000000"
    assert comparison_rows[0]["truth_frd_forward_m"] == "30.000000"
    assert comparison_rows[0]["truth_frd_right_m"] == "11.000000"
    assert comparison_rows[0]["truth_frd_down_m"] == "20.000000"
    assert comparison_rows[0]["estimated_range_m"] == "1.000000"
    assert comparison_rows[0]["raw_range_error_m"] == "-36.749172"
    assert comparison_rows[0]["raw_position_error_m"] == "37.416574"
    assert comparison_rows[0]["corrected_frd_forward_m"] == "30.000000"
    assert comparison_rows[0]["corrected_frd_right_m"] == "11.000000"
    assert comparison_rows[0]["corrected_frd_down_m"] == "20.000000"
    assert comparison_rows[0]["corrected_range_error_m"] == "0.000000"
    assert comparison_rows[0]["corrected_position_error_m"] == "0.000000"
    assert comparison_rows[0]["bbox_available"] == "true"
    assert comparison_rows[0]["bbox_corrected_available"] == "true"
    assert evaluated[0]["profile_frd_forward_m"] == "30.000000"
    assert evaluated[0]["profile_frd_right_m"] == "11.000000"
    assert evaluated[0]["profile_frd_down_m"] == "20.000000"
    assert evaluated[0]["error_frd_forward_m"] == "0.000000"
    assert evaluated[0]["profile_right_m"] == "11.000000"
    assert summary["raw_position_error_rmse_m"] > 30.0
    assert summary["corrected_position_error_rmse_m"] < 1e-9
    assert summary["corrected_range_error_rmse_m"] < 1e-6
    assert summary["position_error_rmse_m"] < 1e-9


def test_origin_bbox_profile_learns_screen_cell_residuals(tmp_path: Path) -> None:
    telemetry = tmp_path / "telemetry.csv"
    rows = []
    for index in range(6):
        raw_right = 10.0 + index
        raw_down = 2.0
        raw_forward = 500.0 + index
        truth_right = raw_right + 3.0
        truth_down = raw_down - 1.0
        truth_forward = raw_forward + 5.0
        rows.append(
            {
                "time_sec": f"{index + 1}.0",
                "estimated_right_m": f"{raw_right}",
                "estimated_down_m": f"{raw_down}",
                "estimated_forward_m": f"{raw_forward}",
                "estimated_range_m": f"{(raw_right**2 + raw_down**2 + raw_forward**2) ** 0.5}",
                "drone_right_m": f"{truth_right}",
                "drone_down_m": f"{truth_down}",
                "drone_forward_m": f"{truth_forward}",
                "ground_truth_m": f"{(truth_right**2 + truth_down**2 + truth_forward**2) ** 0.5}",
                "left_cx": "640.0",
                "left_cy": "360.0",
                "right_cx": "620.0",
                "right_cy": "360.0",
            }
        )
    with telemetry.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    profile, comparison_rows = build_origin_bbox_profile_from_telemetry(
        telemetry,
        profile_id="origin_bbox_test",
        min_samples=3,
        min_cell_samples=1,
        residual_gain=1.0,
    )
    evaluated, summary = evaluate_origin_bbox_profile_rows(comparison_rows, profile)

    assert profile.method == "origin-bbox-grid-v1"
    assert len(profile.cells) >= 1
    assert summary["raw_position_error_rmse_m"] > 5.0
    assert summary["origin_bbox_position_error_rmse_m"] < 1e-9
    assert summary["origin_bbox_range_error_rmse_m"] < 0.1
    assert summary["origin_bbox_position_improvement_ratio"] > 0.99
    assert evaluated[0]["origin_bbox_fallback_level"] == "exact"
    assert evaluated[0]["origin_bbox_residual_right_m"] == "3.000000"
    assert evaluated[0]["origin_bbox_residual_down_m"] == "-1.000000"
    assert evaluated[0]["origin_bbox_residual_forward_m"] == "5.000000"
    assert evaluated[0]["origin_bbox_range_residual_m"] != "0.000000"


def test_origin_bbox_profile_deadbands_weak_corrections(tmp_path: Path) -> None:
    telemetry = tmp_path / "telemetry.csv"
    rows = []
    for index in range(6):
        raw_right = 10.0 + index
        raw_down = 2.0
        raw_forward = 500.0 + index
        truth_right = raw_right + 0.1
        truth_down = raw_down
        truth_forward = raw_forward + 0.1
        rows.append(
            {
                "time_sec": f"{index + 1}.0",
                "estimated_right_m": f"{raw_right}",
                "estimated_down_m": f"{raw_down}",
                "estimated_forward_m": f"{raw_forward}",
                "estimated_range_m": f"{(raw_right**2 + raw_down**2 + raw_forward**2) ** 0.5}",
                "drone_right_m": f"{truth_right}",
                "drone_down_m": f"{truth_down}",
                "drone_forward_m": f"{truth_forward}",
                "ground_truth_m": f"{(truth_right**2 + truth_down**2 + truth_forward**2) ** 0.5}",
                "left_cx": "640.0",
                "left_cy": "360.0",
                "right_cx": "620.0",
                "right_cy": "360.0",
            }
        )
    with telemetry.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    profile, comparison_rows = build_origin_bbox_profile_from_telemetry(
        telemetry,
        profile_id="origin_bbox_deadband_test",
        min_samples=3,
        min_cell_samples=1,
        residual_gain=1.0,
        min_correction_norm_m=0.25,
        min_range_correction_m=0.25,
    )
    evaluated, summary = evaluate_origin_bbox_profile_rows(comparison_rows, profile)

    assert evaluated[0]["origin_bbox_residual_right_m"] == "0.000000"
    assert evaluated[0]["origin_bbox_residual_forward_m"] == "0.000000"
    assert evaluated[0]["origin_bbox_range_residual_m"] == "0.000000"
    assert evaluated[0]["origin_bbox_fallback_level"] == "exact_deadband"
    assert abs(summary["origin_bbox_range_rmse_delta_m"]) < 1e-6


def test_origin_bbox_axis_corrections_are_exact_cell_only() -> None:
    profile = OriginBboxProfile(
        profile_id="origin_bbox_axis_exact_only_test",
        image_width_px=900.0,
        image_height_px=900.0,
        u_bins=(0.0, 1.0 / 3.0, 2.0 / 3.0, 1.0),
        v_bins=(0.0, 1.0 / 3.0, 2.0 / 3.0, 1.0),
        range_bins_m=(0.0, 1000.0),
        cells={
            "r0:u1:v1": {
                "range_bin": 0,
                "u_bin": 1,
                "v_bin": 1,
                "count": 20,
                "residual_m": [100.0, 100.0, 100.0],
                "range_residual_m": 0.0,
                "axis_gains": [1.0, 1.0, 1.0],
                "axis_models": {},
            }
        },
        global_residual_m=(0.0, 0.0, 0.0),
        global_range_residual_m=0.0,
        sample_count=20,
        min_cell_samples=1,
        residual_gain=1.0,
        min_correction_norm_m=0.0,
        min_range_correction_m=0.0,
        training_range_min_m=0.0,
        training_range_max_m=1000.0,
        created_unix_s=0.0,
    )
    rows = simulation_comparison_rows(
        [
            {
                "time_sec": "1.0",
                "estimated_right_m": "0.0",
                "estimated_down_m": "0.0",
                "estimated_forward_m": "500.0",
                "estimated_range_m": "500.0",
                "drone_right_m": "0.0",
                "drone_down_m": "0.0",
                "drone_forward_m": "500.0",
                "ground_truth_m": "500.0",
                "left_cx": "100.0",
                "left_cy": "450.0",
                "right_cx": "100.0",
                "right_cy": "450.0",
            }
        ]
    )

    evaluated, summary = evaluate_origin_bbox_profile_rows(rows, profile)

    assert evaluated[0]["origin_bbox_fallback_level"] == "same_range_nearest_axis_noop"
    assert evaluated[0]["origin_bbox_residual_right_m"] == "0.000000"
    assert evaluated[0]["origin_bbox_residual_down_m"] == "0.000000"
    assert evaluated[0]["origin_bbox_residual_forward_m"] == "0.000000"
    assert summary["origin_bbox_position_error_rmse_m"] < 1e-9


def test_origin_bbox_profile_can_disable_axis_models(tmp_path: Path) -> None:
    telemetry = tmp_path / "telemetry.csv"
    rows = []
    for index in range(8):
        raw_right = 10.0 + index
        raw_down = 2.0
        raw_forward = 500.0 + index
        truth_right = raw_right + 2.0
        truth_down = raw_down - 1.0
        truth_forward = raw_forward + 4.0
        rows.append(
            {
                "time_sec": f"{index + 1}.0",
                "estimated_right_m": f"{raw_right}",
                "estimated_down_m": f"{raw_down}",
                "estimated_forward_m": f"{raw_forward}",
                "estimated_range_m": f"{(raw_right**2 + raw_down**2 + raw_forward**2) ** 0.5}",
                "drone_right_m": f"{truth_right}",
                "drone_down_m": f"{truth_down}",
                "drone_forward_m": f"{truth_forward}",
                "ground_truth_m": f"{(truth_right**2 + truth_down**2 + truth_forward**2) ** 0.5}",
                "left_cx": "640.0",
                "left_cy": "360.0",
                "right_cx": "620.0",
                "right_cy": "360.0",
            }
        )
    with telemetry.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    profile, _ = build_origin_bbox_profile_from_telemetry(
        telemetry,
        profile_id="origin_bbox_gain_only_test",
        min_samples=3,
        min_cell_samples=1,
        validation_telemetry_csv=telemetry,
        axis_model_kinds=(),
        min_validation_cell_samples=1,
    )

    assert profile.cells
    assert all(cell.get("axis_models") == {} for cell in profile.cells.values())
    assert any(any(float(gain) > 0.0 for gain in cell.get("axis_gains", [])) for cell in profile.cells.values())


def test_origin_bbox_profile_can_disable_specific_axis(tmp_path: Path) -> None:
    telemetry = tmp_path / "telemetry.csv"
    rows = []
    for index in range(8):
        raw_right = 10.0 + index
        raw_down = 2.0
        raw_forward = 500.0 + index
        truth_right = raw_right + 2.0
        truth_down = raw_down - 1.0
        truth_forward = raw_forward + 4.0
        rows.append(
            {
                "time_sec": f"{index + 1}.0",
                "estimated_right_m": f"{raw_right}",
                "estimated_down_m": f"{raw_down}",
                "estimated_forward_m": f"{raw_forward}",
                "estimated_range_m": f"{(raw_right**2 + raw_down**2 + raw_forward**2) ** 0.5}",
                "drone_right_m": f"{truth_right}",
                "drone_down_m": f"{truth_down}",
                "drone_forward_m": f"{truth_forward}",
                "ground_truth_m": f"{(truth_right**2 + truth_down**2 + truth_forward**2) ** 0.5}",
                "left_cx": "640.0",
                "left_cy": "360.0",
                "right_cx": "620.0",
                "right_cy": "360.0",
            }
        )
    with telemetry.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    profile, comparison_rows = build_origin_bbox_profile_from_telemetry(
        telemetry,
        profile_id="origin_bbox_disable_forward_test",
        min_samples=3,
        min_cell_samples=1,
        validation_telemetry_csv=telemetry,
        axis_model_kinds=(),
        disabled_axes=("forward",),
        min_validation_cell_samples=1,
    )
    evaluated, _ = evaluate_origin_bbox_profile_rows(comparison_rows, profile)

    assert profile.cells
    assert all(cell.get("axis_disabled") == ["forward"] for cell in profile.cells.values())
    assert all(float(cell.get("axis_gains", [0.0, 0.0, 1.0])[2]) == 0.0 for cell in profile.cells.values())
    assert evaluated[0]["origin_bbox_residual_forward_m"] == "0.000000"


def test_origin_bbox_reprojection_range_only_policy_preserves_raw_lateral(tmp_path: Path) -> None:
    telemetry = tmp_path / "telemetry.csv"
    rows = []
    for index in range(6):
        rows.append(
            {
                "time_sec": f"{index + 1}.0",
                "estimated_right_m": "1.0",
                "estimated_down_m": "2.0",
                "estimated_forward_m": "30.0",
                "estimated_range_m": "30.083218",
                "drone_right_m": "4.0",
                "drone_down_m": "6.0",
                "drone_forward_m": "20.0",
                "ground_truth_m": "21.260292",
                "left_cx": "655.0",
                "left_cy": "360.0",
                "right_cx": "625.0",
                "right_cy": "360.0",
                "raw_disparity_px": "30.0",
            }
        )
    with telemetry.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    profile, comparison_rows = build_origin_bbox_reprojection_profile_from_telemetry(
        telemetry,
        profile_id="range_only_reprojection_test",
        min_samples=3,
        u_bins=1,
        v_bins=1,
        depth_bins=1,
        min_cell_samples=1,
        correction_mode="full",
        output_policy="range_only_forward_fusion",
    )
    evaluated, summary = evaluate_origin_bbox_reprojection_profile_rows(comparison_rows, profile)

    assert profile.output_policy == "range_only_forward_fusion"
    assert evaluated[0]["origin_bbox_reprojection_corrected_right_m"] == "1.000000"
    assert evaluated[0]["origin_bbox_reprojection_corrected_down_m"] == "2.000000"
    assert evaluated[0]["origin_bbox_reprojection_corrected_forward_m"] != "30.000000"
    assert summary["profile_output_policy"] == "range_only_forward_fusion"
    assert summary["origin_bbox_reprojection_right_error_rmse_m"] == summary["raw_right_error_rmse_m"]
    assert summary["origin_bbox_reprojection_down_error_rmse_m"] == summary["raw_down_error_rmse_m"]
    assert summary["origin_bbox_reprojection_forward_error_rmse_m"] < summary["raw_forward_error_rmse_m"]
    assert summary["origin_bbox_reprojection_position_improvement_ratio"] > 0.0


def test_aggregate_summary_files_includes_reprojection_metrics(tmp_path: Path) -> None:
    paths = []
    for index, forward_rmse in enumerate((20.0, 18.0), start=1):
        path = tmp_path / f"summary_{index}.json"
        path.write_text(
            json.dumps(
                {
                    "raw_forward_error_rmse_m": 30.0,
                    "origin_bbox_reprojection_forward_error_rmse_m": forward_rmse,
                    "origin_bbox_reprojection_forward_improvement_ratio": (30.0 - forward_rmse) / 30.0,
                    "raw_position_error_rmse_m": 31.0,
                    "origin_bbox_reprojection_position_error_rmse_m": forward_rmse + 1.0,
                    "origin_bbox_reprojection_position_improvement_ratio": (31.0 - forward_rmse - 1.0)
                    / 31.0,
                }
            ),
            encoding="utf-8",
        )
        paths.append(path)

    summary = aggregate_summary_files(paths)

    assert summary["origin_bbox_reprojection_forward_error_rmse_mean_m"] == 19.0
    assert summary["origin_bbox_reprojection_forward_error_rmse_max_m"] == 20.0
    assert summary["origin_bbox_reprojection_forward_improvement_ratio_min"] > 0.3


def test_simulation_comparison_rows_accepts_legacy_xyz_aliases() -> None:
    rows = [
        {
            "t_s": "1.0",
            "estimated_x_m": "2.0",
            "estimated_y_m": "3.0",
            "estimated_z_m": "4.0",
            "drone_x_m": "2.5",
            "drone_y_m": "3.5",
            "drone_z_m": "4.5",
        }
    ]

    comparison_rows = simulation_comparison_rows(rows)

    assert comparison_rows[0]["measurement_right_m"] == "2.000000"
    assert comparison_rows[0]["stereo_center_right_m"] == "2.000000"
    assert comparison_rows[0]["measurement_down_m"] == "3.000000"
    assert comparison_rows[0]["measurement_forward_m"] == "4.000000"
    assert comparison_rows[0]["measurement_frd_forward_m"] == "4.000000"
    assert comparison_rows[0]["measurement_frd_right_m"] == "2.000000"
    assert comparison_rows[0]["measurement_frd_down_m"] == "3.000000"
    assert comparison_rows[0]["truth_right_m"] == "2.500000"


def test_simulation_comparison_rows_accepts_frd_axis_contract_legacy_xyz() -> None:
    rows = [
        {
            "axis_contract": "stereo_center_frd:forward_right_down",
            "t_s": "1.0",
            "estimated_x_m": "4.0",
            "estimated_y_m": "2.0",
            "estimated_z_m": "3.0",
            "drone_x_m": "4.5",
            "drone_y_m": "2.5",
            "drone_z_m": "3.5",
        }
    ]

    comparison_rows = simulation_comparison_rows(rows)

    assert comparison_rows[0]["measurement_forward_m"] == "4.000000"
    assert comparison_rows[0]["measurement_right_m"] == "2.000000"
    assert comparison_rows[0]["measurement_down_m"] == "3.000000"
    assert comparison_rows[0]["truth_forward_m"] == "4.500000"
    assert comparison_rows[0]["truth_right_m"] == "2.500000"
    assert comparison_rows[0]["truth_down_m"] == "3.500000"
