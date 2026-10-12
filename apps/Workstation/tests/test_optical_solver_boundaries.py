"""Optical candidate calculations do not require a live application owner."""

from __future__ import annotations

from pathlib import Path
import subprocess
import sys
import textwrap
from types import SimpleNamespace

import pytest

from Workstation.optical_pose.horizon import HorizonPoseSolver
from Workstation.optical_pose.terrain_solver import solve_terrain_candidate
from Workstation.workstation_core.operating_intrinsics import (
    OperatingCameraIntrinsics,
    OperatingIntrinsicsProfile,
)
from Workstation.workstation_core.optical_pose import CameraPoseNed, pose_parameters_from_camera_pose
from Workstation.workstation_core.terrain import TerrainMetadata


def _inputs():
    def camera(side):
        return OperatingCameraIntrinsics(
            side=side,
            width_px=640,
            height_px=480,
            camera_matrix=((800.0, 0.0, 320.0), (0.0, 800.0, 240.0), (0.0, 0.0, 1.0)),
            distortion_coefficients=(0.0,) * 5,
            source_zoom_index=1,
        )

    profile = OperatingIntrinsicsProfile(camera("left"), camera("right"))
    priors = tuple(
        CameraPoseNed.from_virtual_frd_rpy_deg(
            center_ned_m=center,
            roll_deg=0.0,
            pitch_deg=0.0,
            yaw_deg=0.0,
        )
        for center in ((1.0, 2.0, 3.0), (1.0, 4.0, 3.0))
    )
    artifact = SimpleNamespace(frame_id="frozen-left", sha256="a" * 64, width=640, height=480)
    annotation = dict(
        frame_id=artifact.frame_id, image_sha256=artifact.sha256, source_width_px=640, source_height_px=480
    )
    return profile, priors, artifact, annotation


def test_horizon_leaf_preserves_unobserved_world_pose_without_controller():
    profile, priors, artifact, annotation = _inputs()
    annotation["horizon_line"] = ((40.0, 240.0), (600.0, 240.0))
    candidate = HorizonPoseSolver().solve(
        {},
        profile=profile,
        priors=priors,
        annotations={"left": annotation},
        artifacts={"left": artifact},
    )
    parameters = pose_parameters_from_camera_pose(candidate.left_pose_ned)
    assert (parameters.roll_deg, parameters.pitch_deg, parameters.yaw_deg) == pytest.approx((0.0, 0.0, 0.0), abs=1e-8)
    assert candidate.left_pose_ned.center_ned_m == (1.0, 2.0, 3.0)
    assert candidate.right_pose_ned is priors[1]
    assert dict(candidate.observability_by_side)["right"] == tuple(
        (name, False) for name, _value in candidate.observability
    )


def test_terrain_leaf_recovers_independent_ned_pixel_fixture_and_rejects_stale_identity():
    profile, priors, artifact, annotation = _inputs()
    # Independent pinhole fixture: RDF=(East-2, Down-3, North-1), fx=fy=800.
    points = (
        (21.0, -1.0, 2.0),
        (25.0, 5.0, 4.0),
        (31.0, 4.0, 0.0),
        (19.0, 0.0, 5.0),
        (27.0, 6.0, 6.0),
        (23.0, -2.0, -1.0),
        (34.0, 1.0, 7.0),
        (29.0, 3.0, 1.0),
    )
    annotation["ridge_polyline"] = tuple(
        (320.0 + 800.0 * (east - 2.0) / (north - 1.0), 240.0 + 800.0 * (down - 3.0) / (north - 1.0))
        for north, east, down in points
    )
    metadata = TerrainMetadata(
        path="terrain/fixture.tif",
        sha256="b" * 64,
        width=100,
        height=100,
        band_count=1,
        dtype="float32",
        crs_wkt="LOCAL_CS[fixture]",
        affine_gdal=(0.0, 1.0, 0.0, 0.0, 0.0, -1.0),
        bounds=(0.0, 0.0, 100.0, 100.0),
        nodata=None,
        horizontal_datum="WGS84",
        vertical_datum="ellipsoid",
        transformer_definition="offline fixture",
        transformer_accuracy_m=0.1,
    )
    kwargs = dict(
        profile=profile,
        priors=priors,
        annotations={"left": annotation},
        artifacts={"left": artifact},
        correspondence_root={"left": {"object_points_ned_m": points}},
        correspondence_file_sha256="",
        terrain_metadata=metadata,
    )
    result = solve_terrain_candidate(**kwargs)
    assert result.left_pose_ned.center_ned_m == pytest.approx((1.0, 2.0, 3.0), abs=1e-5)
    pose = pose_parameters_from_camera_pose(result.left_pose_ned)
    assert (pose.roll_deg, pose.pitch_deg, pose.yaw_deg) == pytest.approx((0.0, 0.0, 0.0), abs=1e-5)
    assert result.right_pose_ned is priors[1]
    assert result.source_observation_ids == ("terrain:" + "b" * 64, "frozen-left")
    annotation["frame_id"] = "other-burst"
    with pytest.raises(ValueError, match="frame_id does not match"):
        solve_terrain_candidate(**kwargs)


def test_solver_imports_reject_live_ui_dependencies_in_fresh_process():
    apps = Path(__file__).resolve().parents[2]
    script = textwrap.dedent("""
        import importlib, importlib.abc, sys
        from pathlib import Path
        apps = Path(sys.argv[1])
        sys.path[:0] = [str(apps), str(apps.parent / 'common/python')]
        class Guard(importlib.abc.MetaPathFinder):
            def find_spec(self, fullname, path=None, target=None):
                if fullname.startswith(('PyQt5', 'Workstation.panel_', 'Workstation.standalone_gui',
                                        'Workstation.optical_pose_controller',
                                        'Workstation.optical_pose.observation_history',
                                        'Workstation.workstation_core.native_frame_store')):
                    raise AssertionError('forbidden live owner: ' + fullname)
                return None
        sys.meta_path.insert(0, Guard())
        for name in ('horizon', 'terrain_solver', 'mountain', 'drone', 'observations'):
            importlib.import_module('Workstation.optical_pose.' + name)
        for forbidden in ('Workstation.optical_pose_controller',
                          'Workstation.optical_pose.observation_history',
                          'Workstation.workstation_core.native_frame_store'):
            try:
                importlib.import_module(forbidden)
            except AssertionError:
                pass
            else:
                raise AssertionError('negative dependency fixture was accepted: ' + forbidden)
        print('independent optical computation')
    """)
    result = subprocess.run(
        [sys.executable, "-I", "-B", "-c", script, str(apps)], check=False, text=True, capture_output=True, timeout=30
    )
    assert result.returncode == 0, f"isolated optical import failed:\n{result.stdout}\n{result.stderr}"
    assert result.stdout.strip() == "independent optical computation"
