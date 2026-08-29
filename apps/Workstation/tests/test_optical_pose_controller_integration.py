from __future__ import annotations

import copy
import json
import hashlib
import math
import time
from pathlib import Path
from types import SimpleNamespace

import pytest
from PyQt5 import QtGui, QtWidgets

import Workstation.optical_pose_controller as optical_pose_module
from Workstation.optical_pose_controller import OpticalPoseController
from Workstation.measurement_view_controller import MeasurementViewController
from Workstation.workspaces import OpticalPosePane
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
from Workstation.workstation_core.rtk_geometry import GpsPositionState, ned_delta_m
from Workstation.workstation_core.terrain import TerrainMetadata
from Workstation.workstation_core.measurement_methods import HORIZON_HYBRID
from Workstation.workstation_core.measurement_attempt import AttitudeSnapshot, GpsSnapshot
from Workstation.workstation_core.optical_pose_auto_select import (
    AutoSelectionResult,
    MountainAutoSelection,
)


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


class _TimedCameraPane:
    """Like ``_CameraPane`` but with an explicit per-side frame timestamp.

    Used only by the Drone RTK capture-observation clock-domain tests, which
    need independent control over each side's ``timestamp_s`` /
    ``timestamp_reference`` -- something the fixed ``_CameraPane`` above
    cannot express.
    """

    def __init__(self, side: str, *, timestamp_s: float, timestamp_reference: str = "app-receive") -> None:
        self.side = side
        self._timestamp_s = timestamp_s
        self._timestamp_reference = timestamp_reference

    def latest_native_measurement_frame(self):
        return (
            self._timestamp_s,
            _image(),
            {
                "timestamp_s": self._timestamp_s,
                "timestamp_reference": self._timestamp_reference,
                "native_width": 640,
                "native_height": 480,
                "overlay_applied": False,
                "camera_serial": f"SERIAL-{self.side.upper()}",
                "stabilization_state": "disabled",
            },
        )


def _gps_state(
    camera: str,
    *,
    timestamp_s: float,
    latitude_deg: float | None = 37.123456789,
    longitude_deg: float | None = 127.987654321,
    altitude_m: float | None = 183.25,
    altitude_ellipsoid_m: float | None = 205.42,
    fix_type: int | None = 6,
    satellites_used: int | None = 24,
) -> GpsPositionState:
    return GpsPositionState(
        camera=camera,
        namespace=f"{camera}-pixhawk",
        timestamp_s=timestamp_s,
        topic="mavlink/GPS_RAW_INT",
        latitude_deg=latitude_deg,
        longitude_deg=longitude_deg,
        altitude_m=altitude_m,
        altitude_ellipsoid_m=altitude_ellipsoid_m,
        fix_type=fix_type,
        satellites_used=satellites_used,
        eph_m=0.02,
        epv_m=0.03,
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
        source_path=panel._active_intrinsics_profile_path,
        source_content_sha256=panel._active_intrinsics_profile_hash,
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


def _active_geometry_state(panel: object, controller: OpticalPoseController) -> dict[str, object]:
    names = (
        "selected_extrinsics_profile",
        "active_extrinsics_profile",
        "optical_pose_extrinsics_profile",
        "active_intrinsics",
        "_selected_extrinsics_profile_path",
        "_selected_extrinsics_profile_hash",
        "_active_extrinsics_profile_path",
        "_active_extrinsics_profile_hash",
        "_active_extrinsics_source_zoom_level",
        "_active_calibration_selection_mode",
        "_active_optical_pose_profile_hash",
        "_active_optical_pose_selection_id",
        "_active_optical_pose_effective_from_pair_sequence",
        "_active_alignment_snapshot_path",
        "_active_alignment_snapshot_sha256",
        "_last_measurement_attempt_key",
        "_last_measurement_attempt",
        "_last_distance_history_key",
    )
    return copy.deepcopy(
        {
            "panel": {
                name: (hasattr(panel, name), getattr(panel, name, None))
                for name in names
            },
            "active_selection": controller._active_selection,
            "active_poses": controller._active_poses,
            "selection_history": tuple(controller._selection_history),
        }
    )


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


def test_horizon_candidate_is_blocked_until_explicit_apply(
    tmp_path: Path,
) -> None:
    panel = _panel(tmp_path)
    pose = OpticalPoseController(panel)
    frozen = pose.capture_workstation_optical_pose_burst({})
    pose.solve_workstation_optical_pose_horizon({"annotations": _annotations(frozen)})
    now = time.monotonic()
    panel.latest_extrinsics = {
        side: SimpleNamespace(
            camera=side,
            roll_deg=1.0,
            pitch_deg=2.0,
            yaw_deg=3.0,
            timestamp_s=now,
            q=(1.0, 0.0, 0.0, 0.0),
            namespace=f"{side}-pixhawk",
            topic="mavlink/ATTITUDE_QUATERNION",
            px4_timestamp_s=None,
        )
        for side in ("left", "right")
    }
    panel.latest_rtk_baseline = SimpleNamespace(
        north_m=0.0,
        east_m=2.0,
        down_m=0.1,
        baseline_m=math.sqrt(4.01),
        warnings=(),
    )
    live_left_gps = _gps_state("left", timestamp_s=now)
    live_right_gps = _gps_state(
        "right", timestamp_s=now, longitude_deg=127.987676321
    )
    panel._selected_baseline_gps_states = lambda: (live_left_gps, live_right_gps)
    panel.gnss_operation_gate_active = False
    panel._gps_baseline_usable_for_translation = lambda _baseline: (True, "")
    panel.settings = SimpleNamespace(
        measurement_preferred_method=HORIZON_HYBRID,
        measurement_method_fallback_order=(HORIZON_HYBRID,),
    )
    panel._workstation_setting_overrides = {}
    panel._workstation_setting = lambda _name, default=None: default
    panel._optical_pose_controller = lambda: pose
    panel.checkerboard_extrinsics_profile = panel.selected_extrinsics_profile
    panel.optical_pose_extrinsics_profile = None

    measurement = MeasurementViewController(panel)
    status = next(
        row for row in measurement._measurement_method_statuses()
        if row.method_id == HORIZON_HYBRID
    )

    assert status.ready is False
    assert "optical pose candidate not applied" in status.reasons
    applied = pose.apply_workstation_optical_pose_candidate(
        {"candidate_id": status.candidate_id}
    )
    status = next(
        row
        for row in measurement._measurement_method_statuses()
        if row.profile_id == applied["selection_id"]
    )
    assert status.ready is True
    assert all(
        source.startswith("profile:")
        for source in dict(status.component_sources).values()
    )
    left_gps = GpsSnapshot.from_state(_gps_state("left", timestamp_s=now))
    right_gps = GpsSnapshot.from_state(
        _gps_state("right", timestamp_s=now, longitude_deg=127.987676321)
    )
    frozen_left_attitude = AttitudeSnapshot.from_state(panel.latest_extrinsics["left"])
    frozen_right_attitude = AttitudeSnapshot.from_state(panel.latest_extrinsics["right"])
    result = measurement._measurement_intrinsics_result_for_method(
        status,
        left_attitude=frozen_left_attitude,
        right_attitude=frozen_right_attitude,
        left_gps=left_gps,
        right_gps=right_gps,
    )
    assert result is not None and result.extrinsics is not None
    assert result.extrinsics.pair_count >= 1
    panel.latest_extrinsics["left"].yaw_deg = 90.0
    panel.latest_rtk_baseline.east_m = 99.0
    repeated = measurement._measurement_intrinsics_result_for_method(
        status,
        left_attitude=frozen_left_attitude,
        right_attitude=frozen_right_attitude,
        left_gps=left_gps,
        right_gps=right_gps,
    )
    assert repeated is not None
    assert repeated.extrinsics == result.extrinsics


def test_optical_apply_missing_active_intrinsics_preserves_every_active_field(
    tmp_path: Path,
) -> None:
    panel = _panel(tmp_path)
    controller = OpticalPoseController(panel)
    frozen = controller.capture_workstation_optical_pose_burst({})
    solved = controller.solve_workstation_optical_pose_horizon(
        {"annotations": _annotations(frozen)}
    )
    candidate_id = solved["candidates"][-1]["candidate_id"]
    panel.active_intrinsics = None
    panel._last_measurement_attempt_key = ("old",)
    panel._last_measurement_attempt = {"state": "old"}
    panel._last_distance_history_key = ("old-distance",)
    before = _active_geometry_state(panel, controller)

    with pytest.raises(RuntimeError, match="active exact K,D is required"):
        controller.apply_workstation_optical_pose_candidate(
            {"candidate_id": candidate_id}
        )

    assert _active_geometry_state(panel, controller) == before
    assert candidate_id in controller._candidates
    assert controller._profile_restore_stack == []


def test_optical_apply_persistence_failure_rolls_back_complete_active_identity(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    panel = _panel(tmp_path)
    controller = OpticalPoseController(panel)
    frozen = controller.capture_workstation_optical_pose_burst({})
    solved = controller.solve_workstation_optical_pose_horizon(
        {"annotations": _annotations(frozen)}
    )
    candidate_id = solved["candidates"][-1]["candidate_id"]
    panel._last_measurement_attempt_key = ("old",)
    panel._last_measurement_attempt = {"state": "old"}
    panel._last_distance_history_key = ("old-distance",)
    before = _active_geometry_state(panel, controller)
    monkeypatch.setattr(
        controller,
        "_persist_session_alignment_snapshot",
        lambda: (_ for _ in ()).throw(OSError("forced alignment persistence failure")),
    )

    with pytest.raises(OSError, match="forced alignment persistence failure"):
        controller.apply_workstation_optical_pose_candidate(
            {"candidate_id": candidate_id}
        )

    assert _active_geometry_state(panel, controller) == before
    assert candidate_id in controller._candidates
    assert controller._profile_restore_stack == []


def test_optical_apply_changed_kd_identity_preserves_previous_active_state(
    tmp_path: Path,
) -> None:
    panel = _panel(tmp_path)
    controller = OpticalPoseController(panel)
    frozen = controller.capture_workstation_optical_pose_burst({})
    solved = controller.solve_workstation_optical_pose_horizon(
        {"annotations": _annotations(frozen)}
    )
    candidate_id = solved["candidates"][-1]["candidate_id"]
    panel._active_intrinsics_profile_path = "calibration/changed-zoom-08.json"
    panel._active_intrinsics_profile_hash = "b" * 64
    panel.operating_intrinsics_profile = OperatingIntrinsicsProfile.from_stereo_result(
        panel.active_intrinsics,
        source_zoom_index=8,
        left_camera_serial="SERIAL-LEFT",
        right_camera_serial="SERIAL-RIGHT",
        left_stabilization_state="disabled",
        right_stabilization_state="disabled",
        profile_id="changed-exact-8x",
        source_path=panel._active_intrinsics_profile_path,
        source_content_sha256=panel._active_intrinsics_profile_hash,
    )
    before = _active_geometry_state(panel, controller)

    with pytest.raises(RuntimeError, match="operating K,D changed"):
        controller.apply_workstation_optical_pose_candidate(
            {"candidate_id": candidate_id}
        )

    assert _active_geometry_state(panel, controller) == before
    assert candidate_id in controller._candidates
    assert controller._profile_restore_stack == []


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

    with pytest.raises(ValueError, match="owner-derived"):
        controller.apply_workstation_optical_pose_candidate(
            {"candidate_id": candidate["candidate_id"], "dof": {"yaw": True}}
        )

    applied = controller.apply_workstation_optical_pose_candidate(
        {"candidate_id": candidate["candidate_id"]}
    )

    assert applied["selected_dof"] == {
        "roll": True,
        "pitch": True,
        "yaw": False,
        "north": False,
        "east": False,
        "down": False,
    }

    assert applied["effective_from_pair_sequence"] == 2
    assert panel.selected_extrinsics_profile is not base_profile
    assert base_profile.profile_id == "base-1x"
    assert panel.selected_extrinsics_profile.profile_id == applied["selection_id"]
    assert panel.active_extrinsics_profile is panel.selected_extrinsics_profile
    assert panel.optical_pose_extrinsics_profile is panel.selected_extrinsics_profile
    assert panel._active_extrinsics_profile_path == (
        f"memory:{applied['selection_id']}"
    )
    assert panel._active_extrinsics_profile_hash == (
        panel.selected_extrinsics_profile.content_sha256
    )
    assert panel._active_optical_pose_profile_hash == (
        panel.selected_extrinsics_profile.content_sha256
    )
    assert panel._active_optical_pose_selection_id == applied["selection_id"]
    assert controller._active_selection.selection_id == applied["selection_id"]
    assert panel.active_intrinsics.extrinsics.translation_left_to_right_m == pytest.approx(
        [-25.0, 0.0, 0.0], abs=1e-9
    )
    assert panel.selected_extrinsics_profile.metadata["rotation_observation_zoom_index"] == 8
    assert panel.selected_extrinsics_profile.metadata["translation_source_zoom_index"] == 1
    assert panel.selected_extrinsics_profile.source_zoom_level is None
    alignment_path = tmp_path / "session-001" / "alignment" / "active_alignment.json"
    assert alignment_path.is_file()
    alignment_payload = json.loads(alignment_path.read_text(encoding="utf-8"))
    assert alignment_payload["session_id"] == "session-001"
    assert alignment_payload["alignment"]["active_selection_id"] == applied["selection_id"]
    applied_version = tmp_path / "session-001" / "alignment" / alignment_payload[
        "selection_relative_path"
    ]
    assert applied_version.is_file()
    assert panel._active_alignment_snapshot_path == str(applied_version)
    assert hashlib.sha256(applied_version.read_bytes()).hexdigest() == alignment_payload[
        "selection_sha256"
    ]

    controller.revert_workstation_optical_pose({})
    reverted_payload = json.loads(alignment_path.read_text(encoding="utf-8"))
    assert reverted_payload["alignment"]["active_selection_id"] == ""
    reverted_version = tmp_path / "session-001" / "alignment" / reverted_payload[
        "selection_relative_path"
    ]
    assert reverted_version.is_file()
    assert reverted_version != applied_version
    assert applied_version.is_file()
    assert panel.selected_extrinsics_profile is base_profile
    assert panel.active_intrinsics is base_active_intrinsics


def test_apply_rejects_candidate_when_any_owned_dof_is_unobservable(tmp_path: Path) -> None:
    panel = _panel(tmp_path)
    controller = OpticalPoseController(panel)
    controller.capture_workstation_optical_pose_burst({})
    prior = controller._resolve_prior_poses({}, prefer_frozen=True)
    all_names = (
        "roll_deg",
        "pitch_deg",
        "yaw_deg",
        "north_m",
        "east_m",
        "down_m",
    )
    candidate = OpticalPoseCandidate(
        candidate_id="rank-deficient-full-pose",
        method="drone-rtk-multiview-r-t",
        left_pose_ned=prior[0],
        right_pose_ned=prior[1],
        optimized_dofs=PoseDofMask.from_names(all_names),
        observability=tuple((name, name != "down_m") for name in all_names),
        input_hashes=(
            (
                "operating_intrinsics_profile_sha256",
                panel.operating_intrinsics_profile.profile_sha256,
            ),
        ),
    )
    controller.register_workstation_optical_pose_candidate(candidate, prior_poses=prior)

    with pytest.raises(ValueError, match="not observable on both camera sides: down_m"):
        controller.apply_workstation_optical_pose_candidate(
            {"candidate_id": candidate.candidate_id}
        )

    assert controller._active_selection is None
    assert controller._active_poses is None


def test_auto_select_job_keeps_owner_state_until_ui_commit_and_rejects_stale_result(
    tmp_path: Path,
) -> None:
    panel = _panel(tmp_path)
    controller = OpticalPoseController(panel)
    controller.capture_workstation_optical_pose_burst({})

    job = controller.prepare_workstation_optical_pose_auto_selection("horizon", {})
    suggestion = job.run()
    assert controller.workstation_optical_pose_snapshot()["auto_suggestions"] == []

    committed = controller.commit_workstation_optical_pose_auto_selection(suggestion)
    assert len(committed["auto_suggestions"]) == 1

    stale_job = controller.prepare_workstation_optical_pose_auto_selection("horizon", {})
    controller.capture_workstation_optical_pose_burst({})
    stale_suggestion = stale_job.run()
    with pytest.raises(RuntimeError, match="stale"):
        controller.commit_workstation_optical_pose_auto_selection(stale_suggestion)


def test_horizon_auto_suggestion_separates_quality_from_copy_structure(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    panel = _panel(tmp_path)
    controller = OpticalPoseController(panel)
    controller.capture_workstation_optical_pose_burst({})
    results = iter(
        (
            AutoSelectionResult(
                mode="horizon",
                state="GOOD",
                detection_method="fixture",
                points=((10.0, 20.0), (620.0, 22.0)),
                inlier_count=20,
                confidence=0.95,
                eligible=True,
            ),
            AutoSelectionResult(
                mode="horizon",
                state="LOW_CONFIDENCE",
                detection_method="fixture",
                points=((12.0, 30.0), (618.0, 33.0)),
                inlier_count=2,
                confidence=0.20,
                eligible=False,
                failure_reason="right horizon support is insufficient",
            ),
        )
    )
    monkeypatch.setattr(
        optical_pose_module,
        "auto_select_horizon",
        lambda *_args, **_kwargs: next(results),
    )

    suggestion = controller._compute_auto_select_horizon({})

    assert suggestion["state"] == "LOW_CONFIDENCE"
    assert suggestion["eligible"] is False
    assert suggestion["copy_eligible"] is True
    assert suggestion["copy_block_reason"] == ""
    assert suggestion["sides"]["left"]["eligible"] is True
    assert suggestion["sides"]["right"]["eligible"] is False


def test_mountain_auto_suggestion_separates_quality_from_copy_structure(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    controller = OpticalPoseController(_panel(tmp_path))
    controller.capture_workstation_optical_pose_burst({})
    left_points = (
        (10.0, 20.0),
        (120.0, 210.0),
        (260.0, 55.0),
        (430.0, 300.0),
    )
    right_points = tuple((u + 5.0, v - 2.0) for u, v in left_points)
    side_results = tuple(
        AutoSelectionResult(
            mode="mountain",
            state="LOW_CONFIDENCE",
            detection_method="fixture",
            points=points,
            inlier_count=4,
            confidence=0.20,
            spatial_rank=2,
            eligible=False,
            failure_reason="match_confidence_below_acceptance_threshold",
        )
        for points in (left_points, right_points)
    )
    result = MountainAutoSelection(
        left=side_results[0],
        right=side_results[1],
        state="LOW_CONFIDENCE",
        detection_method="fixture",
        match_count=4,
        inlier_count=4,
        confidence=0.20,
        eligible=False,
        failure_reason="match_confidence_below_acceptance_threshold",
    )
    monkeypatch.setattr(
        optical_pose_module,
        "auto_select_mountain",
        lambda *_args, **_kwargs: result,
    )

    suggestion = controller._compute_auto_select_mountain({})

    assert suggestion["state"] == "LOW_CONFIDENCE"
    assert suggestion["eligible"] is False
    assert suggestion["copy_eligible"] is True
    assert suggestion["copy_block_reason"] == ""


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


def test_terrain_import_rejects_structured_persistence_failure(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    metadata = TerrainMetadata(
        path=str(tmp_path / "terrain.tif"),
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
    monkeypatch.setattr(optical_pose_module, "inspect_geotiff", lambda *_args, **_kwargs: metadata)
    panel = _panel(tmp_path)
    panel.import_workstation_session_artifact = lambda *_args, **_kwargs: {
        "schema": "workstation-persistence-failure-v1",
        "status": "failed",
        "kind": "artifact_import",
        "error": "terrain-disk-sentinel",
    }
    controller = OpticalPoseController(panel)

    with pytest.raises(OSError, match="terrain-disk-sentinel"):
        controller.import_workstation_optical_pose_terrain(
            {"path": str(tmp_path / "terrain.tif"), "vertical_datum": "ellipsoid"}
        )

    assert controller._terrain_metadata is None


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
    assert candidate["observable_dof"]["yaw_deg"] is False
    assert candidate["observable_dof_by_side"]["left"]["yaw_deg"] is True
    assert candidate["observable_dof_by_side"]["right"]["yaw_deg"] is False
    retained = controller._candidate_drone_rotation_context[candidate["candidate_id"]]["left"]
    assert retained["mode"] == "yaw_only"
    assert retained["observed_ray_camera_rdf"] == pytest.approx((0.0, 0.0, 1.0))
    assert retained["expected_los_ned"] == pytest.approx((0.0, 1.0, 0.0))


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
    assert "lever arm UNMODELED" in " ".join(candidate["warnings"])
    assert "no GeoTIFF" in " ".join(candidate["warnings"])
    assert "fix quality=DEGRADED" in " ".join(candidate["warnings"])
    assert candidate["right_pose"]["center_ned_m"] == pytest.approx([0.0, 2.0, 0.25])
    assert candidate["input_hashes"]["lever_arm_status"] == "UNMODELED"
    assert candidate["input_hashes"]["left_anchor_pose_sha256"]
    assert "reprojection_error_px" not in candidate["quality_metrics"]


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


def test_capture_preserves_exact_drone_rtk_gnss_snapshot(tmp_path: Path) -> None:
    panel = _panel(tmp_path)
    panel.latest_gps = {
        "left": _gps_state("left", timestamp_s=100.0, latitude_deg=37.1, longitude_deg=127.1, altitude_m=50.0),
        "right": _gps_state("right", timestamp_s=100.0, latitude_deg=37.2, longitude_deg=127.2, altitude_m=51.0),
        "drone": _gps_state(
            "drone",
            timestamp_s=100.008,
            latitude_deg=37.123456789,
            longitude_deg=127.987654321,
            altitude_m=183.25,
            altitude_ellipsoid_m=205.42,
            fix_type=6,
            satellites_used=24,
        ),
    }
    controller = OpticalPoseController(panel)

    snapshot = controller.capture_workstation_optical_pose_burst({})

    observations = snapshot["drone_rtk_capture_observations"]
    assert len(observations) == 1
    drone = observations[0]["gnss"]["drone"]
    assert drone["latitude_deg"] == 37.123456789
    assert drone["longitude_deg"] == 127.987654321
    assert drone["altitude_m"] == 183.25
    assert drone["altitude_ellipsoid_m"] == 205.42
    assert drone["fix_type"] == 6
    assert drone["rtk_label"] == "RTK_FIXED"
    assert drone["satellites_used"] == 24
    assert observations[0]["state"] == "EVIDENCE_READY"


def test_drone_rtk_observation_history_is_immutable_across_captures(tmp_path: Path) -> None:
    panel = _panel(tmp_path)
    panel.latest_gps = {
        "left": _gps_state("left", timestamp_s=100.0),
        "right": _gps_state("right", timestamp_s=100.0),
        "drone": _gps_state("drone", timestamp_s=100.0, latitude_deg=10.0, longitude_deg=20.0, altitude_m=30.0),
    }
    controller = OpticalPoseController(panel)
    controller.capture_workstation_optical_pose_burst({})

    panel.latest_gps = {
        "left": _gps_state("left", timestamp_s=200.0),
        "right": _gps_state("right", timestamp_s=200.0),
        "drone": _gps_state("drone", timestamp_s=200.0, latitude_deg=99.0, longitude_deg=88.0, altitude_m=77.0),
    }
    snapshot = controller.capture_workstation_optical_pose_burst({})

    observations = snapshot["drone_rtk_capture_observations"]
    assert len(observations) == 2
    assert observations[0]["sequence"] == 0
    assert observations[1]["sequence"] == 1
    assert observations[0]["observation_id"] != observations[1]["observation_id"]
    assert observations[0]["gnss"]["drone"]["latitude_deg"] == 10.0
    assert observations[1]["gnss"]["drone"]["latitude_deg"] == 99.0


def test_capture_succeeds_and_marks_incomplete_when_drone_gnss_missing(tmp_path: Path) -> None:
    panel = _panel(tmp_path)
    panel.latest_gps = {
        "left": _gps_state("left", timestamp_s=100.0),
        "right": _gps_state("right", timestamp_s=100.0),
    }
    controller = OpticalPoseController(panel)

    snapshot = controller.capture_workstation_optical_pose_burst({})

    assert snapshot["frozen_frame_artifacts"]["left"]["frame_id"]
    observation = snapshot["drone_rtk_capture_observations"][0]
    assert observation["state"] == "INCOMPLETE"
    assert observation["gnss"]["drone"] is None
    assert observation["gnss"]["left"] is not None


def test_capture_succeeds_without_any_gnss_state_at_all(tmp_path: Path) -> None:
    """Horizon/Mountain/Target Capture must keep working with no GNSS wiring."""

    panel = _panel(tmp_path)
    controller = OpticalPoseController(panel)

    snapshot = controller.capture_workstation_optical_pose_burst({})

    observation = snapshot["drone_rtk_capture_observations"][0]
    assert observation["state"] == "INCOMPLETE"
    assert observation["gnss"]["left"] is None
    assert observation["gnss"]["right"] is None
    assert observation["gnss"]["drone"] is None


def test_capture_reports_rtk_float_without_falsely_showing_fixed(tmp_path: Path) -> None:
    panel = _panel(tmp_path)
    panel.latest_gps = {
        "left": _gps_state("left", timestamp_s=100.0),
        "right": _gps_state("right", timestamp_s=100.0),
        "drone": _gps_state("drone", timestamp_s=100.0, fix_type=5),
    }
    controller = OpticalPoseController(panel)

    snapshot = controller.capture_workstation_optical_pose_burst({})

    observation = snapshot["drone_rtk_capture_observations"][0]
    assert observation["gnss"]["drone"]["fix_type"] == 5
    assert observation["gnss"]["drone"]["rtk_label"] == "RTK_FLOAT"
    assert observation["gnss"]["drone"]["rtk_label"] != "RTK_FIXED"
    assert observation["state"] == "WARNING"


def test_drone_rtk_sync_delta_computed_only_for_confirmed_shared_clock(tmp_path: Path) -> None:
    panel = _panel(tmp_path)
    panel.left_pane = _TimedCameraPane("left", timestamp_s=100.000, timestamp_reference="app-receive")
    panel.right_pane = _TimedCameraPane("right", timestamp_s=100.004, timestamp_reference="app-receive")
    panel.latest_gps = {
        "left": _gps_state("left", timestamp_s=100.000),
        "right": _gps_state("right", timestamp_s=100.004),
        "drone": _gps_state("drone", timestamp_s=100.008),
    }
    controller = OpticalPoseController(panel)

    snapshot = controller.capture_workstation_optical_pose_burst({})

    sync = snapshot["drone_rtk_capture_observations"][0]["synchronization"]
    assert sync["state"] == "SYNC_OK"
    assert sync["left_frame_to_drone_gnss_s"] == pytest.approx(0.008, abs=1e-12)
    assert sync["right_frame_to_drone_gnss_s"] == pytest.approx(0.004, abs=1e-12)


def test_drone_rtk_sync_is_unverified_when_clock_domain_is_not_confirmed(tmp_path: Path) -> None:
    panel = _panel(tmp_path)
    # The left frame carries an NTP/source-derived timestamp instead of the
    # host-monotonic "app-receive" stamp, so it is not provably on the same
    # clock as the Drone GNSS sample even though the numbers look adjacent.
    panel.left_pane = _TimedCameraPane("left", timestamp_s=100.000, timestamp_reference="rtcp-ntp")
    panel.right_pane = _TimedCameraPane("right", timestamp_s=100.004, timestamp_reference="app-receive")
    panel.latest_gps = {
        "left": _gps_state("left", timestamp_s=100.000),
        "right": _gps_state("right", timestamp_s=100.004),
        "drone": _gps_state("drone", timestamp_s=100.008),
    }
    controller = OpticalPoseController(panel)

    snapshot = controller.capture_workstation_optical_pose_burst({})

    sync = snapshot["drone_rtk_capture_observations"][0]["synchronization"]
    assert sync["state"] == "SYNC_UNVERIFIED"
    assert sync["left_frame_to_drone_gnss_s"] is None
    assert sync["right_frame_to_drone_gnss_s"] is None
    assert sync["reason"]


@pytest.mark.parametrize(
    ("left_timestamp_s", "right_timestamp_s", "drone_timestamp_s", "reason_text"),
    (
        (100.0, 101.0, 100.1, "left/right frame delta"),
        (100.0, 100.004, 101.0, "frame/Drone GNSS delta"),
        (float("nan"), 100.004, 100.008, "not finite"),
    ),
)
def test_drone_rtk_sync_rejects_nonfinite_or_out_of_window_samples(
    tmp_path: Path,
    left_timestamp_s: float,
    right_timestamp_s: float,
    drone_timestamp_s: float,
    reason_text: str,
) -> None:
    panel = _panel(tmp_path)
    panel.left_pane = _TimedCameraPane(
        "left",
        timestamp_s=left_timestamp_s,
        timestamp_reference="app-receive",
    )
    panel.right_pane = _TimedCameraPane(
        "right",
        timestamp_s=right_timestamp_s,
        timestamp_reference="app-receive",
    )
    panel.latest_gps = {
        "left": _gps_state("left", timestamp_s=100.0),
        "right": _gps_state("right", timestamp_s=100.004),
        "drone": _gps_state("drone", timestamp_s=drone_timestamp_s),
    }
    controller = OpticalPoseController(panel)

    snapshot = controller.capture_workstation_optical_pose_burst({})

    observation = snapshot["drone_rtk_capture_observations"][0]
    sync = observation["synchronization"]
    assert sync["state"] == "SYNC_UNVERIFIED"
    assert sync["left_frame_to_drone_gnss_s"] is None
    assert sync["right_frame_to_drone_gnss_s"] is None
    assert reason_text in sync["reason"]
    assert observation["state"] == "WARNING"


def test_import_burst_does_not_attach_current_gnss_as_capture_time_gnss(tmp_path: Path) -> None:
    left_path = tmp_path / "left.png"
    right_path = tmp_path / "right.png"
    _image().save(str(left_path), "PNG")
    _image().save(str(right_path), "PNG")
    panel = _panel(tmp_path)
    panel.latest_gps = {
        "left": _gps_state("left", timestamp_s=100.0),
        "right": _gps_state("right", timestamp_s=100.0),
        "drone": _gps_state("drone", timestamp_s=100.0, latitude_deg=1.0, longitude_deg=2.0, altitude_m=3.0),
    }
    controller = OpticalPoseController(panel)

    snapshot = controller.import_workstation_optical_pose_burst(
        {"sources": {"left": {"path": str(left_path)}, "right": {"path": str(right_path)}}}
    )

    assert snapshot["frozen_frame_artifacts"]["left"]["frame_id"]
    assert snapshot["drone_rtk_capture_observations"] == []


def test_ftp_import_with_unknown_capture_time_never_claims_app_receive_sync(
    tmp_path: Path,
) -> None:
    left_path = tmp_path / "ftp-left.png"
    right_path = tmp_path / "ftp-right.png"
    _image().save(str(left_path), "PNG")
    _image().save(str(right_path), "PNG")
    panel = _panel(tmp_path)
    # These live RTSP values are deliberately plausible but unrelated to the
    # downloaded stills. They must not leak into imported evidence.
    for side in ("left", "right"):
        panel.camera_frame_metadata[side].update(
            {
                "timestamp_s": 1_787_000_000.0,
                "timestamp_reference": "app-receive",
                "source_timestamp_s": 42.0,
            }
        )
    panel.latest_gps = {
        "left": _gps_state("left", timestamp_s=100.0),
        "right": _gps_state("right", timestamp_s=100.004),
        "drone": _gps_state("drone", timestamp_s=100.008),
    }
    controller = OpticalPoseController(panel)
    before_import_s = time.monotonic()

    snapshot = controller.import_workstation_optical_pose_burst(
        {
            "sources": {
                "left": {"path": str(left_path)},
                "right": {"path": str(right_path)},
            },
            "_ftp_request_gnss_snapshot": panel.latest_gps,
        }
    )
    after_import_s = time.monotonic()

    for side in ("left", "right"):
        artifact = snapshot["frozen_frame_artifacts"][side]
        metadata = snapshot["frozen_native_frames"][side]["metadata"]
        assert artifact["timestamp_reference"] == "import-receive"
        assert before_import_s <= artifact["timestamp_s"] <= after_import_s
        assert metadata["capture_timestamp_known"] is False
        assert metadata["capture_zoom_index"] is None
        assert metadata["capture_zoom_known"] is False
        assert "source_timestamp_s" not in metadata
    sync = snapshot["drone_rtk_capture_observations"][0]["synchronization"]
    assert sync["state"] == "SYNC_UNVERIFIED"
    assert sync["left_frame_to_drone_gnss_s"] is None
    assert sync["right_frame_to_drone_gnss_s"] is None


def test_drone_rtk_observation_capture_has_no_solver_side_effects(tmp_path: Path) -> None:
    panel = _panel(tmp_path)
    panel.latest_gps = {
        "left": _gps_state("left", timestamp_s=100.0),
        "right": _gps_state("right", timestamp_s=100.0),
        "drone": _gps_state("drone", timestamp_s=100.0),
    }
    controller = OpticalPoseController(panel)

    snapshot = controller.capture_workstation_optical_pose_burst({})

    assert snapshot["drone_rtk_capture_observations"][0]["state"] == "EVIDENCE_READY"
    assert snapshot["candidates"] == []
    assert snapshot["active_profile_id"] == ""
    assert snapshot["active_candidate_id"] == ""
    assert controller._active_poses is None
    assert controller._active_selection is None


def _mountain_annotations(snapshot: dict[str, object]):
    annotations = _annotations(snapshot, horizon=False)
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
    return annotations


def _mountain_auto_suggestion(
    frozen: dict[str, object],
    *,
    suggestion_id: str,
    state: str = "LOW_CONFIDENCE",
) -> dict[str, object]:
    annotations = _mountain_annotations(frozen)
    annotations["left"]["ridge_polyline"].append({"u_px": 600.0, "v_px": 80.0})
    annotations["right"]["ridge_polyline"].append({"u_px": 610.0, "v_px": 75.0})
    return {
        "schema": "optical-pose-auto-suggestion-v1",
        "suggestion_id": suggestion_id,
        "mode": "mountain",
        "state": state,
        "provenance_state": "AUTO_PROPOSED",
        "detection_method": "fixture",
        "confidence": 0.20,
        "eligible": False,
        "copy_eligible": True,
        "failure_reason": "match_confidence_below_acceptance_threshold",
        "warnings": [],
        "sides": {
            side: {
                **annotations[side],
                "points": list(annotations[side]["ridge_polyline"]),
                "point_confidences": [0.9, 0.8, 0.7, 0.6, 0.5],
            }
            for side in ("left", "right")
        },
    }


def test_low_confidence_mountain_suggestion_accepts_filtered_structural_points(
    tmp_path: Path,
) -> None:
    panel = _panel(tmp_path)
    controller = OpticalPoseController(panel)
    frozen = controller.capture_workstation_optical_pose_burst({})
    suggestion = _mountain_auto_suggestion(
        frozen, suggestion_id="auto-mountain-low-confidence"
    )
    controller._record_auto_suggestion(suggestion)

    snapshot = controller.accept_workstation_optical_pose_suggestion(
        {
            "suggestion_id": "auto-mountain-low-confidence",
            "excluded_point_indices": [1],
        }
    )

    accepted = snapshot["accepted_auto_suggestion"]
    assert accepted["state"] == "LOW_CONFIDENCE"
    assert accepted["eligible"] is False
    assert accepted["copy_eligible"] is True
    assert accepted["excluded_point_indices"] == [1]
    assert accepted["original_point_count"] == 5
    assert accepted["retained_point_count"] == 4
    assert len(accepted["filtered_input_sha256"]) == 64
    assert accepted["sides"]["left"]["points"] == [
        suggestion["sides"]["left"]["points"][index]
        for index in (0, 2, 3, 4)
    ]
    assert snapshot["annotation_review"]["annotations"]["right"]["ridge_polyline"] == [
        suggestion["sides"]["right"]["points"][index]
        for index in (0, 2, 3, 4)
    ]
    assert "LOW_CONFIDENCE" in " ".join(accepted["warnings"])
    event_name, event_payload = panel.events[-1]
    assert event_name == "optical_pose_auto_suggestion_accepted"
    assert event_payload["suggestion"]["excluded_point_indices"] == [1]
    assert event_payload["suggestion"]["filtered_input_sha256"] == accepted[
        "filtered_input_sha256"
    ]

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    pane = OpticalPosePane()
    pane.apply_snapshot(snapshot)
    assert len(pane.annotation_views["left"].annotation_points("ridge")) == 4
    assert len(pane.annotation_views["right"].annotation_points("ridge")) == 4
    assert "LOW_CONFIDENCE" in pane.auto_suggestion_status_label.text()
    assert "AUTO_ACCEPTED" in pane.auto_suggestion_status_label.text()
    assert not pane.use_suggestion_button.isEnabled()
    pane.deleteLater()
    app.processEvents()


def test_mountain_suggestion_acceptance_rejects_invalid_exclusion(
    tmp_path: Path,
) -> None:
    controller = OpticalPoseController(_panel(tmp_path))
    frozen = controller.capture_workstation_optical_pose_burst({})
    controller._record_auto_suggestion(
        _mountain_auto_suggestion(frozen, suggestion_id="auto-mountain-invalid-exclusion")
    )

    with pytest.raises(ValueError, match="out of range"):
        controller.accept_workstation_optical_pose_suggestion(
            {
                "suggestion_id": "auto-mountain-invalid-exclusion",
                "excluded_point_indices": [99],
            }
        )


def test_mountain_suggestion_acceptance_rejects_stale_frozen_identity(
    tmp_path: Path,
) -> None:
    controller = OpticalPoseController(_panel(tmp_path))
    frozen = controller.capture_workstation_optical_pose_burst({})
    suggestion = _mountain_auto_suggestion(
        frozen, suggestion_id="auto-mountain-stale-copy"
    )
    controller.capture_workstation_optical_pose_burst({})
    # Adversarially restore the stale proposal to prove acceptance does not
    # rely only on Capture's normal suggestion-clear behavior.
    controller._auto_suggestions[suggestion["suggestion_id"]] = suggestion

    with pytest.raises(ValueError, match="frame_id does not match frozen evidence"):
        controller.accept_workstation_optical_pose_suggestion(
            {"suggestion_id": "auto-mountain-stale-copy"}
        )


def test_low_confidence_horizon_suggestion_accepts_structural_endpoints(
    tmp_path: Path,
) -> None:
    panel = _panel(tmp_path)
    controller = OpticalPoseController(panel)
    frozen = controller.capture_workstation_optical_pose_burst({})
    annotations = _annotations(frozen)
    suggestion = {
        "suggestion_id": "auto-horizon-low-confidence",
        "mode": "horizon",
        "state": "LOW_CONFIDENCE",
        "eligible": False,
        "confidence": 0.20,
        "failure_reason": "skyline_line_support_below_acceptance_threshold",
        "warnings": [],
        "sides": {
            side: {
                **annotations[side],
                "points": list(annotations[side]["horizon_line"]),
            }
            for side in ("left", "right")
        },
    }
    controller._record_auto_suggestion(suggestion)

    snapshot = controller.accept_workstation_optical_pose_suggestion(
        {"suggestion_id": "auto-horizon-low-confidence"}
    )

    accepted = snapshot["accepted_auto_suggestion"]
    assert accepted["state"] == "LOW_CONFIDENCE"
    assert accepted["eligible"] is False
    assert accepted["copy_eligible"] is True
    assert len(accepted["validated_input_sha256"]) == 64
    assert "LOW_CONFIDENCE" in " ".join(accepted["warnings"])
    assert len(snapshot["annotation_review"]["annotations"]["left"]["horizon_line"]) == 2
    event_name, event_payload = panel.events[-1]
    assert event_name == "optical_pose_auto_suggestion_accepted"
    assert event_payload["suggestion"]["validated_input_sha256"] == accepted[
        "validated_input_sha256"
    ]

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    pane = OpticalPosePane()
    pane.apply_snapshot(snapshot)
    assert len(pane.annotation_views["left"].annotation_points("horizon")) == 2
    assert len(pane.annotation_views["right"].annotation_points("horizon")) == 2
    assert "LOW_CONFIDENCE" in pane.auto_suggestion_status_label.text()
    assert "AUTO_ACCEPTED" in pane.auto_suggestion_status_label.text()
    pane.deleteLater()
    app.processEvents()


def test_horizon_suggestion_acceptance_rejects_short_or_stale_endpoints(
    tmp_path: Path,
) -> None:
    controller = OpticalPoseController(_panel(tmp_path))
    frozen = controller.capture_workstation_optical_pose_burst({})
    annotations = _annotations(frozen)
    suggestion = {
        "suggestion_id": "auto-horizon-structural-block",
        "mode": "horizon",
        "state": "LOW_CONFIDENCE",
        "eligible": False,
        "sides": {
            side: {
                **annotations[side],
                "points": [
                    {"u_px": 20.0, "v_px": 210.0},
                    {"u_px": 100.0, "v_px": 212.0},
                ],
            }
            for side in ("left", "right")
        },
    }
    controller._record_auto_suggestion(suggestion)
    with pytest.raises(ValueError, match="25% of image width"):
        controller.accept_workstation_optical_pose_suggestion(
            {"suggestion_id": "auto-horizon-structural-block"}
        )

    stale = copy.deepcopy(suggestion)
    stale["suggestion_id"] = "auto-horizon-stale-copy"
    for side in ("left", "right"):
        stale["sides"][side]["points"] = list(annotations[side]["horizon_line"])
    controller.capture_workstation_optical_pose_burst({})
    controller._auto_suggestions[stale["suggestion_id"]] = stale
    with pytest.raises(ValueError, match="frame_id does not match frozen evidence"):
        controller.accept_workstation_optical_pose_suggestion(
            {"suggestion_id": "auto-horizon-stale-copy"}
        )


def test_gnss_mountain_uses_capture_time_baseline_not_live_gps(tmp_path: Path) -> None:
    panel = _panel(tmp_path)
    panel.latest_gps = {
        "left": _gps_state("left", timestamp_s=100.0, latitude_deg=37.0, longitude_deg=127.0, altitude_m=50.0),
        "right": _gps_state("right", timestamp_s=100.0, latitude_deg=37.00002, longitude_deg=127.0, altitude_m=50.2),
        "drone": _gps_state("drone", timestamp_s=100.0, latitude_deg=37.00001, longitude_deg=127.00001, altitude_m=50.1),
    }
    controller = OpticalPoseController(panel)
    frozen = controller.capture_workstation_optical_pose_burst({})

    expected_baseline = ned_delta_m(panel.latest_gps["left"], panel.latest_gps["right"])

    # Live GPS changes after the Capture; the frozen burst's GNSS evidence
    # must not be affected, and Mountain must not read this live state.
    panel.latest_gps = {
        "left": _gps_state("left", timestamp_s=200.0, latitude_deg=10.0, longitude_deg=10.0, altitude_m=0.0),
        "right": _gps_state("right", timestamp_s=200.0, latitude_deg=20.0, longitude_deg=20.0, altitude_m=0.0),
        "drone": _gps_state("drone", timestamp_s=200.0, latitude_deg=15.0, longitude_deg=15.0, altitude_m=0.0),
    }

    snapshot = controller.solve_workstation_optical_pose_gnss_mountain(
        {"annotations": _mountain_annotations(frozen)}
    )

    candidate = snapshot["candidates"][-1]
    right_center = candidate["right_pose"]["center_ned_m"]
    # Left prior center is (0,0,0); lever arm is UNMODELED (zero), so the
    # right optical center is exactly the resolved GNSS baseline expressed
    # in NED after the left->NED rotation. The magnitude must match the
    # captured baseline, never the live-changed one (which is ~1500km away).
    baseline_m = math.sqrt(sum(v * v for v in right_center))
    expected_m = math.sqrt(sum(v * v for v in expected_baseline))
    assert baseline_m == pytest.approx(expected_m, rel=1.0e-6)
    assert baseline_m < 100.0  # metres; a live-GPS leak would be ~1500 km


def test_gnss_mountain_fails_closed_without_capture_baseline_or_override(tmp_path: Path) -> None:
    panel = _panel(tmp_path)
    controller = OpticalPoseController(panel)
    frozen = controller.capture_workstation_optical_pose_burst({})

    with pytest.raises(RuntimeError, match="no capture-associated GNSS baseline evidence"):
        controller.solve_workstation_optical_pose_gnss_mountain(
            {"annotations": _mountain_annotations(frozen)}
        )


def test_gnss_mountain_left_anchor_change_changes_provenance_hash(tmp_path: Path) -> None:
    panel = _panel(tmp_path)
    controller = OpticalPoseController(panel)
    frozen = controller.capture_workstation_optical_pose_burst({})
    annotations = _mountain_annotations(frozen)
    payload_base = {
        "annotations": annotations,
        "gnss_baseline_ned_m": [0.0, 2.0, 0.25],
    }

    first = controller.solve_workstation_optical_pose_gnss_mountain(dict(payload_base))
    first_hash = first["candidates"][-1]["input_hashes"]["left_anchor_pose_sha256"]

    rotated_left = CameraPoseNed.from_virtual_frd_rpy_deg(
        center_ned_m=(0.0, 0.0, 0.0), roll_deg=0.0, pitch_deg=0.0, yaw_deg=15.0
    )
    rotated_right = CameraPoseNed.from_virtual_frd_rpy_deg(
        center_ned_m=(0.0, -25.0, 0.0), roll_deg=0.0, pitch_deg=0.0, yaw_deg=15.0
    )
    payload_rotated = dict(payload_base)
    payload_rotated["prior_poses"] = {
        "left": {
            "rotation_ned_from_camera_rdf": [list(row) for row in rotated_left.rotation_ned_from_camera_rdf],
            "center_ned_m": list(rotated_left.center_ned_m),
        },
        "right": {
            "rotation_ned_from_camera_rdf": [list(row) for row in rotated_right.rotation_ned_from_camera_rdf],
            "center_ned_m": list(rotated_right.center_ned_m),
        },
    }
    second = controller.solve_workstation_optical_pose_gnss_mountain(payload_rotated)
    second_hash = second["candidates"][-1]["input_hashes"]["left_anchor_pose_sha256"]

    assert first_hash != second_hash


def test_gnss_mountain_lever_arm_defaults_unmodeled_and_applies_explicit_correction(
    tmp_path: Path,
) -> None:
    panel = _panel(tmp_path)
    controller = OpticalPoseController(panel)
    frozen = controller.capture_workstation_optical_pose_burst({})
    annotations = _mountain_annotations(frozen)
    payload = {"annotations": annotations, "gnss_baseline_ned_m": [0.0, 2.0, 0.25]}

    uncorrected = controller.solve_workstation_optical_pose_gnss_mountain(dict(payload))
    uncorrected_candidate = uncorrected["candidates"][-1]
    assert uncorrected_candidate["input_hashes"]["lever_arm_status"] == "UNMODELED"
    assert "lever arm UNMODELED" in " ".join(uncorrected_candidate["warnings"])
    uncorrected_center = uncorrected_candidate["right_pose"]["center_ned_m"]

    corrected_payload = dict(payload)
    corrected_payload["lever_arm_baseline_correction_left_rdf_m"] = [0.05, 0.0, -0.02]
    corrected = controller.solve_workstation_optical_pose_gnss_mountain(corrected_payload)
    corrected_candidate = corrected["candidates"][-1]
    assert corrected_candidate["input_hashes"]["lever_arm_status"] == "CALIBRATED"
    corrected_center = corrected_candidate["right_pose"]["center_ned_m"]

    delta = [corrected_center[i] - uncorrected_center[i] for i in range(3)]
    delta_norm = math.sqrt(sum(v * v for v in delta))
    expected_norm = math.sqrt(0.05 ** 2 + 0.0 ** 2 + 0.02 ** 2)
    assert delta_norm == pytest.approx(expected_norm, rel=1.0e-6)
    assert delta_norm > 1.0e-6


def test_horizon_spherical_model_requires_explicit_camera_height(tmp_path: Path) -> None:
    panel = _panel(tmp_path)
    controller = OpticalPoseController(panel)
    frozen = controller.capture_workstation_optical_pose_burst({})

    with pytest.raises(ValueError, match="camera_height_above_reference_surface_m"):
        controller.solve_workstation_optical_pose_horizon(
            {
                "annotations": _annotations(frozen),
                "horizon_model": "spherical_geometric",
            }
        )


def test_horizon_reports_endpoint_click_sensitivity_and_rejects_short_baseline(
    tmp_path: Path,
) -> None:
    panel = _panel(tmp_path)
    controller = OpticalPoseController(panel)
    frozen = controller.capture_workstation_optical_pose_burst({})
    annotations = _annotations(frozen)

    solved = controller.solve_workstation_optical_pose_horizon({"annotations": annotations})
    candidate = solved["candidates"][-1]
    metrics = candidate["quality_metrics"]
    assert "left_roll_sensitivity_1px_deg" in metrics
    assert "left_pitch_sensitivity_1px_deg" in metrics
    assert metrics["left_roll_sensitivity_1px_deg"] >= 0.0
    assert "reprojection_error_px" not in metrics

    short_annotations = _annotations(frozen)
    for row in short_annotations.values():
        row["horizon_line"] = [
            {"u_px": 300.0, "v_px": 240.0},
            {"u_px": 340.0, "v_px": 240.0},
        ]
    with pytest.raises(ValueError, match="horizon endpoints are only"):
        controller.solve_workstation_optical_pose_horizon({"annotations": short_annotations})


def test_horizon_short_endpoint_baseline_has_higher_sensitivity_than_long_one(
    tmp_path: Path,
) -> None:
    panel = _panel(tmp_path)
    controller = OpticalPoseController(panel)
    frozen = controller.capture_workstation_optical_pose_burst({})

    long_annotations = _annotations(frozen)
    long_solved = controller.solve_workstation_optical_pose_horizon({"annotations": long_annotations})
    long_metrics = long_solved["candidates"][-1]["quality_metrics"]

    medium_annotations = _annotations(frozen)
    for row in medium_annotations.values():
        row["horizon_line"] = [
            {"u_px": 250.0, "v_px": 240.0},
            {"u_px": 490.0, "v_px": 240.0},
        ]
    medium_solved = controller.solve_workstation_optical_pose_horizon({"annotations": medium_annotations})
    medium_metrics = medium_solved["candidates"][-1]["quality_metrics"]

    assert medium_metrics["left_pitch_sensitivity_1px_deg"] > long_metrics["left_pitch_sensitivity_1px_deg"]
