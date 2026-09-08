from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

import pyarrow.parquet as pq
import pytest

import Workstation.measurement_view_controller as measurement_module
from Workstation.measurement_view_controller import MeasurementViewController
from Workstation.recording_controller import RecordingController
from Workstation.workstation_core.calibration import (
    CameraIntrinsics,
    CheckerboardConfig,
    StereoIntrinsicsResult,
)
from Workstation.workstation_core.optical_pose import CameraPoseNed
from Workstation.workstation_core.session_parquet import finalize_geometry_comparison


def _intrinsics() -> StereoIntrinsicsResult:
    camera = CameraIntrinsics(
        width=640,
        height=480,
        camera_matrix=[
            [800.0, 0.0, 320.0],
            [0.0, 800.0, 240.0],
            [0.0, 0.0, 1.0],
        ],
        distortion_coefficients=[0.0, 0.0, 0.0, 0.0, 0.0],
        reprojection_error_px=0.2,
        image_count=20,
    )
    return StereoIntrinsicsResult(
        left=camera,
        right=camera,
        board=CheckerboardConfig(9, 6, 0.03),
    )


def _candidate_row(profile_hash: str) -> dict[str, object]:
    return {
        "candidate_id": "candidate-b",
        "method": "gnss-constrained-mountain-uv",
        "left_pose_ned": CameraPoseNed.from_virtual_frd_rpy_deg(
            center_ned_m=(0.0, 0.0, 0.0),
            roll_deg=0.0,
            pitch_deg=0.0,
            yaw_deg=0.0,
        ),
        "right_pose_ned": CameraPoseNed.from_virtual_frd_rpy_deg(
            center_ned_m=(0.0, 1.0, 0.0),
            roll_deg=0.0,
            pitch_deg=0.0,
            yaw_deg=0.0,
        ),
        "optimized_dofs": (
            "roll_deg",
            "pitch_deg",
            "yaw_deg",
            "north_m",
            "east_m",
            "down_m",
        ),
        "candidate_profile_hash": "c" * 64,
        "candidate_intrinsics_hash": profile_hash,
        "candidate_quality": {"reprojection_error_px": 0.25},
        "candidate_warnings": [],
        "candidate_applied": False,
    }


def _panel(active_hash: str, candidate_hash: str) -> SimpleNamespace:
    owner = SimpleNamespace(
        _intrinsics_snapshot=lambda **_kwargs: {
            "applicability": {"profile_sha256": active_hash}
        }
    )
    return SimpleNamespace(
        active_intrinsics=_intrinsics(),
        latest_extrinsics={},
        latest_rtk_baseline=None,
        checkerboard_extrinsics_profile=None,
        optical_pose_extrinsics_profile=None,
        _optical_pose_controller=lambda: owner,
        _gps_baseline_usable_for_translation=lambda _baseline: (False, "not used"),
        workstation_optical_pose_candidate_poses=lambda: (
            _candidate_row(candidate_hash),
        ),
    )


def _pair() -> SimpleNamespace:
    return SimpleNamespace(
        left=SimpleNamespace(center=(350.0, 240.0)),
        right=SimpleNamespace(center=(290.0, 240.0)),
    )


@pytest.mark.parametrize("loaded_calibration", [False, True])
def test_nominal_operating_intrinsics_refresh_comparison(loaded_calibration: bool) -> None:
    from Workstation.workstation_core.operating_intrinsics import (
        build_nominal_operating_intrinsics_profile,
    )

    profile = build_nominal_operating_intrinsics_profile(
        zoom_index=1, left_width_px=640, left_height_px=480,
        right_width_px=640, right_height_px=480,
    )
    panel = _panel("a" * 64, "a" * 64)
    if not loaded_calibration:
        panel.active_intrinsics = None
    owner = panel._optical_pose_controller()
    owner.operating_intrinsics_profile_for_geometry = lambda: profile
    controller = MeasurementViewController(panel)
    pair = _pair()
    submitted = []
    panel._rtsp_urls_are_same_stream = lambda: False
    panel._matched_detection_pair = lambda: pair
    panel._geometry_comparison_rows = controller._geometry_comparison_rows
    panel._measurement_display_mode = lambda: "detection"
    panel._show_measurement_pair = lambda *_args: None
    controller._multi_method_measurement_active = lambda: True
    controller._select_measurement_method_for_attempt = lambda: None
    controller._submit_method_evaluations = lambda value: submitted.append(value)
    controller._refresh_distance()
    first = panel.geometry_comparison_rows
    pair.right.center = (320.0, 240.0)
    controller._refresh_distance()
    second = panel.geometry_comparison_rows
    assert len(submitted) == 2

    # Parallel cameras, one metre baseline: depth = focal_px / disparity_px.
    focal = float(profile.left.camera_matrix[0][0])
    assert first[-1]["forward_m"] == pytest.approx(focal / 60.0)
    assert second[-1]["forward_m"] == pytest.approx(focal / 30.0)
    cx = float(profile.left.camera_matrix[0][2])
    cy = float(profile.left.camera_matrix[1][2])
    fy = float(profile.left.camera_matrix[1][1])
    assert first[-1]["right_m"] == pytest.approx((350.0 - cx) / 60.0 - 0.5)
    assert first[-1]["down_m"] == pytest.approx((240.0 - cy) * focal / (60.0 * fy))
    assert first[-1]["not_measurement_result"] is True
    assert "NOMINAL" in first[-1]["status"]


def test_comparison_does_not_reuse_loaded_calibration_when_operating_kd_blocked() -> None:
    panel = _panel("a" * 64, "a" * 64)
    panel._optical_pose_controller().operating_intrinsics_profile_for_geometry = lambda: None
    assert MeasurementViewController(panel)._geometry_comparison_rows(_pair()) == ()


def test_matching_unapplied_candidate_is_analysis_only_with_complete_identity(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    profile_hash = "a" * 64
    calls: list[dict[str, object]] = []

    def fake_triangulate(**kwargs):
        calls.append(dict(kwargs))
        return SimpleNamespace(
            result=SimpleNamespace(point_m=(12.0, 0.3, -0.1), range_m=12.004),
            reason="ok",
        )

    monkeypatch.setattr(
        measurement_module,
        "triangulate_pixels_with_relative_pose_attempt",
        fake_triangulate,
    )

    rows = MeasurementViewController(
        _panel(profile_hash, profile_hash)
    )._geometry_comparison_rows(_pair())
    candidate = next(row for row in rows if row["source_state"] == "candidate_unapplied")

    assert len(calls) == 1
    assert candidate["result_class"] == "analysis_only"
    assert candidate["not_measurement_result"] is True
    assert candidate["evidence_namespace"] == "analysis/geometry_comparison"
    assert candidate["candidate_profile_id"] == "candidate-b"
    assert candidate["candidate_profile_hash"] == "c" * 64
    assert candidate["candidate_intrinsics_hash"] == profile_hash
    assert candidate["active_intrinsics_hash"] == profile_hash
    assert candidate["candidate_applied"] is False
    assert json.loads(candidate["candidate_applicability"])["applicable"] is True
    assert json.loads(candidate["candidate_quality"])["reprojection_error_px"] == 0.25
    assert candidate["range_m"] == pytest.approx(12.004)


def test_stale_candidate_is_not_triangulated_even_as_analysis(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(
        measurement_module,
        "triangulate_pixels_with_relative_pose_attempt",
        lambda **_kwargs: pytest.fail("stale candidate must not be triangulated"),
    )

    rows = MeasurementViewController(
        _panel("a" * 64, "b" * 64)
    )._geometry_comparison_rows(_pair())
    candidate = next(row for row in rows if row["source_state"] == "candidate_unapplied")

    assert candidate["forward_m"] is None
    assert candidate["range_m"] is None
    assert "analysis blocked" in candidate["status"]
    applicability = json.loads(candidate["candidate_applicability"])
    assert applicability == {
        "applicable": False,
        "reasons": ["candidate K,D differs from active K,D"],
    }


def test_recorded_candidate_comparison_retains_analysis_namespace_and_parquet_columns(
    tmp_path: Path,
) -> None:
    comparison = {
        "schema_version": "workstation-geometry-comparison-analysis-v1",
        "evidence_namespace": "analysis/geometry_comparison",
        "result_class": "analysis_only",
        "not_measurement_result": True,
        "source_state": "candidate_unapplied",
        "candidate_profile_id": "candidate-b",
        "candidate_profile_hash": "c" * 64,
        "candidate_intrinsics_hash": "a" * 64,
        "active_intrinsics_hash": "a" * 64,
        "candidate_applied": False,
        "candidate_applicability": '{"applicable":true,"reasons":[]}',
        "candidate_quality": '{"reprojection_error_px":0.25}',
        "method": "PnP candidate: gnss-constrained-mountain-uv",
        "forward_m": 12.0,
        "right_m": 0.3,
        "down_m": -0.1,
        "range_m": 12.004,
        "status": "ok",
    }
    panel = SimpleNamespace(
        distance_recording_enabled=True,
        geometry_comparison_rows=(comparison,),
        extrinsics_comparison_recording_rows=[],
        _fmt_record_float=lambda value, precision=6: (
            "" if value is None else f"{float(value):.{precision}f}"
        ),
    )

    RecordingController(panel)._append_extrinsics_comparison_snapshot(7, 10.5)

    assert len(panel.extrinsics_comparison_recording_rows) == 1
    recorded = panel.extrinsics_comparison_recording_rows[0]
    assert recorded["pair_sequence"] == 7
    assert recorded["result_class"] == "analysis_only"
    assert recorded["not_measurement_result"] is True
    assert recorded["candidate_profile_id"] == "candidate-b"
    output = tmp_path / "geometry_comparison.parquet"
    metadata = finalize_geometry_comparison(output, [recorded])
    table = pq.read_table(output)
    values = table.to_pylist()[0]
    assert metadata.row_count == 1
    assert pq.read_schema(output).metadata[b"workstation.schema_id"] == (
        b"workstation-geometry-comparison-parquet-v2"
    )
    assert values["evidence_namespace"] == "analysis/geometry_comparison"
    assert values["not_measurement_result"] is True
    assert values["candidate_profile_hash"] == "c" * 64
    assert json.loads(values["row_json"])["candidate_applied"] is False
