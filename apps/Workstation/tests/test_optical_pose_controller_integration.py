from __future__ import annotations

import time
from pathlib import Path
from types import SimpleNamespace

import pytest
from PyQt5 import QtGui

from Workstation.optical_pose_controller import OpticalPoseController
from Workstation.workstation_core.calibration import (
    CameraIntrinsics,
    CheckerboardConfig,
    StereoExtrinsics,
    StereoExtrinsicsProfile,
    StereoIntrinsicsResult,
)
from Workstation.workstation_core.operating_intrinsics import OperatingIntrinsicsProfile
from Workstation.workstation_core.optical_pose import (
    CameraPoseNed,
    OpticalPoseCandidate,
    PoseDofMask,
)
from Workstation.workstation_core.terrain import TerrainMetadata


def _image() -> QtGui.QImage:
    image = QtGui.QImage(640, 480, QtGui.QImage.Format_RGB888)
    image.fill(QtGui.QColor(20, 30, 40))
    return image


class _CameraPane:
    def __init__(self, side: str) -> None:
        self.side = side

    def latest_native_measurement_frame(self):
        return (
            100.0,
            _image(),
            {
                "timestamp_s": 100.0,
                "timestamp_reference": "app-receive",
                "native_width": 640,
                "native_height": 480,
                "overlay_applied": False,
                "camera_serial": f"SERIAL-{self.side.upper()}",
                "stabilization_state": "disabled",
            },
        )


def _intrinsics() -> StereoIntrinsicsResult:
    camera = lambda: CameraIntrinsics(
        width=640,
        height=480,
        camera_matrix=[[800.0, 0.0, 320.0], [0.0, 800.0, 240.0], [0.0, 0.0, 1.0]],
        distortion_coefficients=[0.0, 0.0, 0.0, 0.0, 0.0],
        reprojection_error_px=0.2,
        image_count=20,
    )
    return StereoIntrinsicsResult(
        left=camera(),
        right=camera(),
        board=CheckerboardConfig(9, 6, 0.03),
        extrinsics=StereoExtrinsics(
            rotation_left_to_right=[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            translation_left_to_right_m=[-25.0, 0.0, 0.0],
            reprojection_error_px=0.3,
            pair_count=30,
        ),
    )


def _panel(tmp_path: Path):
    active = _intrinsics()
    base = StereoExtrinsicsProfile(
        extrinsics=active.extrinsics,
        source_zoom_level=1,
        method="checkerboard-1x",
        profile_id="base-1x",
    )
    now = time.monotonic()
    zoom = {
        side: {
            "index": 8,
            "ratio": 8.0,
            "updated_monotonic_s": now,
        }
        for side in ("left", "right")
    }
    frame_metadata = {
        side: {
            "native_width": 640,
            "native_height": 480,
            "camera_serial": f"SERIAL-{side.upper()}",
            "stabilization_state": "disabled",
        }
        for side in ("left", "right")
    }
    events: list[tuple[str, dict[str, object]]] = []
    panel = SimpleNamespace(
        recording_session_id="session-001",
        left_pane=_CameraPane("left"),
        right_pane=_CameraPane("right"),
        actual_visible_zoom=zoom,
        camera_frame_metadata=frame_metadata,
        active_intrinsics=active,
        selected_extrinsics_profile=base,
        _active_intrinsics_source_zoom_level=8,
        _active_intrinsics_profile_hash="a" * 64,
        _active_intrinsics_profile_path="calibration/zoom-08.json",
        distance_recording_rows=[{"pair_sequence": 0}, {"pair_sequence": 1}],
        events=events,
    )
    panel.operating_intrinsics_profile = OperatingIntrinsicsProfile.from_stereo_result(
        active,
        source_zoom_index=8,
        left_camera_serial="SERIAL-LEFT",
        right_camera_serial="SERIAL-RIGHT",
        left_stabilization_state="disabled",
        right_stabilization_state="disabled",
        profile_id="exact-8x",
        provenance={"test": "explicit exact operating K,D"},
    )
    panel._recording_session_dir = lambda session_id: tmp_path / session_id

    def ensure(event_type, payload, **_kwargs):
        events.append((str(event_type), dict(payload)))
        return {}

    def append(event_type, payload, **_kwargs):
        events.append((str(event_type), dict(payload)))
        return {}

    panel.ensure_workstation_run_evidence = ensure
    panel.append_workstation_run_event = append
    return panel


def _annotations(snapshot: dict[str, object], *, horizon: bool = True):
    result = {}
    for side, frozen in snapshot["frozen_frames"].items():
        row = {
            "frame_id": frozen["frame_id"],
            "image_sha256": frozen["image_sha256"],
            "source_width_px": 640,
            "source_height_px": 480,
            "horizon_line": [],
            "ridge_polyline": [],
            "target_point": [],
        }
        if horizon:
            row["horizon_line"] = [
                {"u_px": 20.0, "v_px": 240.0},
                {"u_px": 620.0, "v_px": 240.0},
            ]
        result[side] = row
    return result


def test_capture_horizon_and_apply_create_next_pair_immutable_profile(tmp_path: Path) -> None:
    panel = _panel(tmp_path)
    controller = OpticalPoseController(panel)
    base_profile = panel.selected_extrinsics_profile
    base_active_intrinsics = panel.active_intrinsics

    frozen = controller.capture_workstation_optical_pose_burst({})

    assert set(frozen["frozen_frames"]) == {"left", "right"}
    assert len(list((tmp_path / "session-001" / "frames" / "optical-pose-burst").glob("*.png"))) == 2
    assert frozen["intrinsics_applicability"]["applicability"]["state"] == "READY"

    solved = controller.solve_workstation_optical_pose_horizon(
        {"annotations": _annotations(frozen)}
    )
    candidate = solved["candidates"][-1]
    assert candidate["method"] == "flat-horizon-roll-pitch"
    assert candidate["observable_dof"]["yaw_deg"] is False

    with pytest.raises(ValueError, match="not optimized/observable"):
        controller.apply_workstation_optical_pose_candidate(
            {"candidate_id": candidate["candidate_id"], "dof": {"yaw": True}}
        )

    applied = controller.apply_workstation_optical_pose_candidate(
        {
            "candidate_id": candidate["candidate_id"],
            "dof": {"roll": True, "pitch": True},
        }
    )

    assert applied["effective_from_pair_sequence"] == 2
    assert panel.selected_extrinsics_profile is not base_profile
    assert base_profile.profile_id == "base-1x"
    assert panel.selected_extrinsics_profile.profile_id == applied["selection_id"]
    assert panel.active_intrinsics.extrinsics.translation_left_to_right_m == pytest.approx(
        [-25.0, 0.0, 0.0], abs=1e-9
    )
    assert panel.selected_extrinsics_profile.metadata["rotation_observation_zoom_index"] == 8
    assert panel.selected_extrinsics_profile.metadata["translation_source_zoom_index"] == 1
    assert panel.selected_extrinsics_profile.source_zoom_level is None

    controller.revert_workstation_optical_pose({})
    assert panel.selected_extrinsics_profile is base_profile
    assert panel.active_intrinsics is base_active_intrinsics


def test_combined_pose_requires_explicit_candidate_ids(tmp_path: Path) -> None:
    controller = OpticalPoseController(_panel(tmp_path))

    with pytest.raises(ValueError, match="explicitly selected compatible candidate IDs"):
        controller.solve_workstation_optical_pose_combined({})


def test_terrain_pnp_fails_clearly_without_explicit_3d_to_ridge_matches(tmp_path: Path) -> None:
    panel = _panel(tmp_path)
    controller = OpticalPoseController(panel)
    frozen = controller.capture_workstation_optical_pose_burst({})
    annotations = _annotations(frozen, horizon=False)
    for row in annotations.values():
        row["ridge_polyline"] = [
            {"u_px": 50.0, "v_px": 220.0},
            {"u_px": 180.0, "v_px": 200.0},
            {"u_px": 360.0, "v_px": 210.0},
            {"u_px": 590.0, "v_px": 190.0},
        ]
    controller._terrain_metadata = TerrainMetadata(
        path="/not/exposed/in/snapshot.tif",
        sha256="b" * 64,
        width=100,
        height=100,
        band_count=1,
        dtype="float32",
        crs_wkt="LOCAL_CS[test]",
        affine_gdal=(0.0, 1.0, 0.0, 0.0, 0.0, -1.0),
        bounds=(0.0, 0.0, 100.0, 100.0),
        nodata=None,
        horizontal_datum="WGS84",
        vertical_datum="ellipsoid",
        transformer_definition="test",
        transformer_accuracy_m=0.1,
    )

    with pytest.raises(ValueError, match="object_points_ned_m matched one-to-one"):
        controller.solve_workstation_optical_pose_terrain(
            {"annotations": annotations, "correspondences": {"left": {}}}
        )


def test_drone_yaw_requires_explicit_los_and_preserves_other_dof(tmp_path: Path) -> None:
    panel = _panel(tmp_path)
    controller = OpticalPoseController(panel)
    controller.capture_workstation_optical_pose_burst({})

    snapshot = controller.solve_workstation_optical_pose_drone_yaw(
        {
            "drone_yaw": {
                "left": {
                    "observed_ray_camera_rdf": [0.0, 0.0, 1.0],
                    "expected_los_ned": [0.0, 1.0, 0.0],
                    "timestamp_alignment_verified": True,
                }
            }
        }
    )

    candidate = snapshot["candidates"][-1]
    assert candidate["method"] == "drone-rtk-target-yaw"
    assert candidate["left_delta_rpy_deg"] == pytest.approx([0.0, 0.0, 90.0])
    assert candidate["right_delta_rpy_deg"] == pytest.approx([0.0, 0.0, 0.0])


def test_gnss_mountain_uses_mountain_rotation_and_gnss_metric_baseline(tmp_path: Path) -> None:
    panel = _panel(tmp_path)
    controller = OpticalPoseController(panel)
    frozen = controller.capture_workstation_optical_pose_burst({})
    annotations = _annotations(frozen, horizon=False)
    annotations["left"]["ridge_polyline"] = [
        {"u_px": 80.0, "v_px": 180.0},
        {"u_px": 200.0, "v_px": 210.0},
        {"u_px": 350.0, "v_px": 160.0},
        {"u_px": 520.0, "v_px": 230.0},
    ]
    annotations["right"]["ridge_polyline"] = [
        {"u_px": 90.0, "v_px": 175.0},
        {"u_px": 210.0, "v_px": 205.0},
        {"u_px": 360.0, "v_px": 155.0},
        {"u_px": 530.0, "v_px": 225.0},
    ]

    snapshot = controller.solve_workstation_optical_pose_gnss_mountain(
        {
            "annotations": annotations,
            "gnss_baseline_ned_m": [0.0, 2.0, 0.25],
            "gnss_fix_quality": "DEGRADED",
        }
    )

    candidate = snapshot["candidates"][-1]
    assert candidate["method"] == "gnss-constrained-mountain-uv"
    assert candidate["input_hashes"]["gnss_baseline_sha256"]
    assert candidate["input_hashes"]["mountain_uv_pairs_sha256"]
    assert "terrain_geotiff_sha256" not in candidate["input_hashes"]
    assert candidate["quality_metrics"]["mountain_uv_pair_count"] == pytest.approx(4.0)
    assert candidate["quality_metrics"]["gnss_baseline_m"] == pytest.approx(2.015564437)
    assert "camera optical center equals GNSS antenna" in " ".join(candidate["warnings"])
    assert "no GeoTIFF" in " ".join(candidate["warnings"])
    assert "fix quality=DEGRADED" in " ".join(candidate["warnings"])
    assert candidate["right_pose"]["center_ned_m"] == pytest.approx([0.0, 2.0, 0.25])


def test_registration_rejects_candidate_as_its_own_implicit_prior() -> None:
    panel = SimpleNamespace(
        actual_visible_zoom={},
        camera_frame_metadata={},
        active_intrinsics=None,
        selected_extrinsics_profile=None,
    )
    controller = OpticalPoseController(panel)
    pose = CameraPoseNed.from_virtual_frd_rpy_deg(
        center_ned_m=(0.0, 0.0, 0.0),
        roll_deg=0.0,
        pitch_deg=0.0,
        yaw_deg=0.0,
    )
    candidate = OpticalPoseCandidate(
        candidate_id="no-prior",
        method="test",
        left_pose_ned=pose,
        right_pose_ned=pose,
        optimized_dofs=PoseDofMask(),
        observability=tuple((name, True) for name in (
            "roll_deg", "pitch_deg", "yaw_deg", "north_m", "east_m", "down_m"
        )),
    )

    with pytest.raises(RuntimeError, match="explicit prior"):
        controller.register_workstation_optical_pose_candidate(candidate)
