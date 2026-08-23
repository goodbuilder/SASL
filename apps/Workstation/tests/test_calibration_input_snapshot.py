from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from Workstation.calibration_controller import CalibrationWorkflowController
from Workstation.calibration_workers import CheckerboardCalibrationWorker
from Workstation.workstation_core.calibration import (
    CameraIntrinsics,
    CheckerboardConfig,
    StereoExtrinsics,
)
from Workstation.workstation_core.calibration_input import CalibrationInputSnapshot


def _photo_root(base: Path, name: str, count: int = 5) -> Path:
    root = base / name
    for index in range(1, count + 1):
        key = f"{index:03d}"
        for side in ("left", "right"):
            path = root / side / f"{side}_{key}.png"
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(f"{name}-{side}-{key}".encode())
        metadata = root / "capture_metadata" / f"capture_{key}.json"
        metadata.parent.mkdir(parents=True, exist_ok=True)
        metadata.write_text(
            json.dumps(
                {
                    "capture_zoom_level": 8,
                    "capture_left_zoom_level": 8,
                    "capture_right_zoom_level": 8,
                }
            ),
            encoding="utf-8",
        )
    return root


def _controller(tmp_path: Path, hidden_root: Path) -> CalibrationWorkflowController:
    messages: list[str] = []
    panel = SimpleNamespace(
        app_root=tmp_path,
        _checkerboard_validation_root=lambda: hidden_root,
        _checkerboard_validation_mode=lambda: "checkerboard",
        calib_calculation_pairs_spin=SimpleNamespace(value=lambda: 0),
        calibration_status_label=SimpleNamespace(setText=messages.append),
        checkerboard_validation_status_label=object(),
        _set_health_label=lambda _label, text, _level: messages.append(str(text)),
        _log=messages.append,
    )
    return CalibrationWorkflowController(panel)


def test_visible_root_snapshot_overrides_hidden_root_and_freezes_selection_hashes(
    tmp_path: Path,
) -> None:
    visible = _photo_root(tmp_path, "visible-a")
    hidden = _photo_root(tmp_path, "hidden-b")
    for metadata_path in (visible / "capture_metadata").glob("capture_*.json"):
        payload = json.loads(metadata_path.read_text(encoding="utf-8"))
        payload["calibration_timing_evidence"] = {
            "schema": "workstation-calibration-timing-evidence-v1",
            "evidence_class": "HOST_DISPATCH_PROXY",
            "pair_persistence_allowed": True,
            "precision_qualified": False,
            "actual_exposure_verified": False,
            "timestamp_reference": "ftp-cptr-dispatch-monotonic",
            "pair_delta_s": 0.01,
            "maximum_delta_s": 0.1,
            "detail": "host dispatch only",
        }
        metadata_path.write_text(json.dumps(payload), encoding="utf-8")
    sidecar = visible / "calibration_pair_selection.json"
    sidecar.write_text(
        json.dumps(
            {
                "schema": "workstation-calibration-file-selection-v2",
                "root": ".",
                "manual_mode": False,
                "included_keys": [],
                "left_paths": [],
                "right_paths": [],
                "excluded_left_paths": [],
                "excluded_right_paths": [],
                "excluded_pair_keys": ["001"],
            }
        ),
        encoding="utf-8",
    )
    controller = _controller(tmp_path, hidden)

    snapshot, source_pair_count, explicit = controller._calibration_input_snapshot(
        {
            "root": str(visible),
            "mode": "checkerboard",
            "pair_limit": 3,
        }
    )

    assert snapshot.root_path == visible.resolve()
    assert source_pair_count == 5
    assert explicit is False
    assert snapshot.selection_policy == "automatic_even_spread"
    assert snapshot.pair_limit == 3
    assert snapshot.excluded_pair_keys == ("001",)
    assert snapshot.timing_evidence_classes == ("HOST_DISPATCH_PROXY",)
    assert snapshot.precision_timing_qualified is False
    assert snapshot.actual_exposure_verified is False
    assert len(snapshot.pair_paths) == 3
    assert all(key != "001" for key, _left, _right in snapshot.pair_paths)
    assert all(path.is_relative_to(visible) for path in snapshot.left_paths)
    assert all(path.is_relative_to(visible) for path in snapshot.right_paths)
    assert all(not path.is_relative_to(hidden) for path in snapshot.left_paths)
    recovered = CalibrationInputSnapshot.from_mapping(snapshot.to_mapping())
    assert recovered == snapshot
    assert recovered.snapshot_sha256 == snapshot.snapshot_sha256
    worker = CheckerboardCalibrationWorker(
        mode="pixhawk",
        config=CheckerboardConfig(9, 6, 0.03),
        input_snapshot=snapshot,
    )
    assert worker.mode == "checkerboard"
    assert worker.left_intrinsic_paths == snapshot.left_paths
    assert worker.right_intrinsic_paths == snapshot.right_paths
    assert worker.stereo_pairs == snapshot.pair_paths


def test_worker_rechecks_snapshot_bytes_before_solver(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    root = _photo_root(tmp_path, "root", count=3)
    controller = _controller(tmp_path, root)
    snapshot, _count, _explicit = controller._calibration_input_snapshot(
        {"root": str(root), "mode": "checkerboard", "pair_limit": 0}
    )
    snapshot.left_paths[0].write_bytes(b"changed-after-snapshot")
    monkeypatch.setattr(
        "Workstation.calibration_workers.calibrate_camera_with_diagnostics",
        lambda *_args, **_kwargs: pytest.fail("solver must not read changed input"),
    )
    worker = CheckerboardCalibrationWorker(
        mode="checkerboard",
        config=CheckerboardConfig(9, 6, 0.03),
        input_snapshot=snapshot,
    )
    failures: list[str] = []
    worker.failed.connect(failures.append)

    worker.run()

    assert failures and "changed after snapshot" in failures[0]


def test_worker_result_echoes_exact_input_snapshot_identity(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    root = _photo_root(tmp_path, "root", count=3)
    controller = _controller(tmp_path, root)
    snapshot, _count, _explicit = controller._calibration_input_snapshot(
        {"root": str(root), "mode": "checkerboard", "pair_limit": 0}
    )
    intrinsics = CameraIntrinsics(
        width=1280,
        height=720,
        camera_matrix=[
            [900.0, 0.0, 640.0],
            [0.0, 901.0, 360.0],
            [0.0, 0.0, 1.0],
        ],
        distortion_coefficients=[0.0],
        reprojection_error_px=0.2,
        image_count=3,
    )
    quality = SimpleNamespace(
        state="PASS",
        reasons=(),
        effective_pose_count=12,
        image_size=(1280, 720),
        cx_px=640.0,
        cy_px=360.0,
        center_grid_occupied=((0, 0), (1, 1), (2, 2)),
        union_extent_normalized=(0.8, 0.8),
        subset_principal_span_px=(1.0, 1.0),
        subset_focal_relative_span=0.01,
        to_mapping=lambda: {"state": "PASS", "reason_codes": []},
    )

    def solve(paths, _config):
        return SimpleNamespace(
            intrinsics=intrinsics,
            quality_inputs=lambda: {},
            intrinsic_standard_deviations={},
            extrinsic_standard_deviations=(),
            per_view_errors_px=(),
            effective_view_ids=tuple(str(path) for path in paths),
            observations=(),
            subset_estimates=(),
            subset_membership=(),
            subset_failures=(),
            subset_summary={},
            calibration_flags=0,
        )

    monkeypatch.setattr("Workstation.calibration_workers.calibrate_camera_with_diagnostics", solve)
    monkeypatch.setattr(
        "Workstation.calibration_workers.evaluate_camera_calibration_quality",
        lambda **_kwargs: quality,
    )
    monkeypatch.setattr(
        "Workstation.calibration_workers.calibrate_stereo_extrinsics_from_image_pairs",
        lambda *_args, **_kwargs: StereoExtrinsics(
            rotation_left_to_right=[
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0],
            ],
            translation_left_to_right_m=[-0.3, 0.0, 0.0],
            reprojection_error_px=0.1,
            pair_count=3,
        ),
    )
    monkeypatch.setattr(
        "Workstation.calibration_workers.analyze_calibration_images",
        lambda *_args, **_kwargs: (),
    )
    worker = CheckerboardCalibrationWorker(
        mode="checkerboard",
        config=CheckerboardConfig(9, 6, 0.03),
        input_snapshot=snapshot,
    )
    completed: list[object] = []
    worker.completed.connect(completed.append)

    worker.run()

    assert len(completed) == 1
    diagnostics = completed[0].solver_diagnostics
    assert diagnostics["calibration_input_snapshot_sha256"] == snapshot.snapshot_sha256
    assert diagnostics["calibration_input_snapshot"] == snapshot.to_mapping()
