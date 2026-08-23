from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

import pytest


def test_selection_paths_are_relative_and_legacy_absolute_paths_rebase(
    tmp_path: Path,
) -> None:
    from Workstation.workstation_core.calibration_selection import (
        portable_calibration_selection_paths,
        resolve_calibration_selection_paths,
    )

    current_root = tmp_path / "new-checkout" / "calibration" / "session-a"
    current = current_root / "left" / "left_001.png"
    current.parent.mkdir(parents=True)
    current.write_bytes(b"image")
    old_root = Path("/old/dated-checkout/apps/Workstation/calibration/session-a")

    resolved, errors = resolve_calibration_selection_paths(
        [str(old_root / "left" / "left_001.png")],
        current_root,
        stored_root=str(old_root),
    )

    assert errors == ()
    assert resolved == [current.resolve()]
    assert portable_calibration_selection_paths(current_root, resolved) == [
        "left/left_001.png"
    ]


def test_selection_loader_distinguishes_absent_valid_and_invalid_sidecars(
    tmp_path: Path,
) -> None:
    from Workstation.workstation_core.calibration_selection import (
        load_calibration_selection_sidecar,
    )

    sidecar = tmp_path / "calibration_pair_selection.json"
    assert load_calibration_selection_sidecar(sidecar).state == "ABSENT"
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
    assert load_calibration_selection_sidecar(sidecar).state == "VALID"
    sidecar.write_text("{not-json", encoding="utf-8")
    malformed = load_calibration_selection_sidecar(sidecar)
    assert malformed.state == "INVALID"
    assert "cannot parse" in malformed.errors[0]


def test_v2_excluded_pair_remains_absent_from_new_checkout_worker_inputs(
    tmp_path: Path,
) -> None:
    from Workstation.calibration_controller import CalibrationWorkflowController

    root = tmp_path / "new-checkout" / "calibration" / "session-a"
    left_paths = [root / "left" / f"left_{key}.png" for key in ("001", "002")]
    right_paths = [root / "right" / f"right_{key}.png" for key in ("001", "002")]
    for path in (*left_paths, *right_paths):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(b"image")
    sidecar = root / "calibration_pair_selection.json"
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
    panel = SimpleNamespace(
        _calibration_pair_selection_path=lambda: sidecar,
        _calibration_dirs=lambda: (root, root / "left", root / "right"),
    )
    controller = CalibrationWorkflowController(panel)
    panel._load_calibration_pair_selection = controller._load_calibration_pair_selection
    controller._calibration_calculation_pair_limit = lambda: 0
    scanned = [
        (key, left, right)
        for key, left, right in zip(
            ("001", "002"), left_paths, right_paths, strict=True
        )
    ]

    selected_left, selected_right, worker_pairs, explicit = (
        controller._effective_calibration_inputs(
            root,
            left_paths,
            right_paths,
            scanned,
            mode="checkerboard",
        )
    )

    assert explicit is False
    assert selected_left == left_paths
    assert selected_right == right_paths
    assert [(key, left.name, right.name) for key, left, right in worker_pairs] == [
        ("002", "left_002.png", "right_002.png")
    ]


@pytest.mark.parametrize(
    ("sidecar_text", "expected_error"),
    (
        ("{not-json", "cannot parse calibration selection sidecar"),
        (
            json.dumps(
                {
                    "schema": "unknown-selection-v99",
                    "root": ".",
                    "manual_mode": False,
                }
            ),
            "unsupported calibration selection schema",
        ),
        (
            json.dumps(
                {
                    "schema": "workstation-calibration-file-selection-v2",
                    "root": ".",
                    "manual_mode": "false",
                    "included_keys": [],
                    "left_paths": [],
                    "right_paths": [],
                    "excluded_left_paths": [],
                    "excluded_right_paths": [],
                    "excluded_pair_keys": [],
                }
            ),
            "manual_mode must be a JSON boolean",
        ),
    ),
)
def test_unknown_or_semantically_invalid_sidecar_blocks_before_worker_creation(
    tmp_path: Path,
    monkeypatch,
    sidecar_text: str,
    expected_error: str,
) -> None:
    from Workstation import calibration_controller as calibration_module
    from Workstation.calibration_controller import CalibrationWorkflowController

    root = tmp_path / "calibration"
    left_paths = [root / "left" / f"left_{index:03d}.png" for index in range(3)]
    right_paths = [root / "right" / f"right_{index:03d}.png" for index in range(3)]
    for path in (*left_paths, *right_paths):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(b"image")
    sidecar = root / "calibration_pair_selection.json"
    sidecar.write_text(sidecar_text, encoding="utf-8")
    messages: list[str] = []
    label = SimpleNamespace(setText=messages.append)
    panel = SimpleNamespace(
        checkerboard_calibration_thread=None,
        checkerboard_validation_status_label=label,
        calibration_result_text=SimpleNamespace(setPlainText=messages.append),
        calibration_status_label=label,
        calib_min_images_spin=SimpleNamespace(value=lambda: 3),
        _checkerboard_validation_mode=lambda: "checkerboard",
        _checkerboard_validation_scan_counts=lambda: (
            root,
            left_paths,
            right_paths,
            [
                (f"{index:03d}", left_paths[index], right_paths[index])
                for index in range(3)
            ],
        ),
        _calibration_pair_selection_path=lambda: sidecar,
        _set_health_label=lambda _label, text, _level: messages.append(str(text)),
        _log=messages.append,
    )
    controller = CalibrationWorkflowController(panel)
    controller._ensure_calibration_session_root = lambda _purpose: root
    controller._calibration_calculation_pair_limit = lambda: 0
    panel._load_calibration_pair_selection = controller._load_calibration_pair_selection
    monkeypatch.setattr(calibration_module, "opencv_available", lambda: (True, None))
    monkeypatch.setattr(
        calibration_module,
        "CheckerboardCalibrationWorker",
        lambda **_kwargs: pytest.fail("invalid sidecar must block worker creation"),
    )

    controller.run_intrinsic_calibration()

    assert any(expected_error in message for message in messages)
    assert any("all-photo fallback blocked" in message for message in messages)
    assert panel.checkerboard_calibration_thread is None


def test_unrecoverable_exclusion_path_blocks_all_photo_fallback(tmp_path: Path) -> None:
    from Workstation.calibration_controller import CalibrationWorkflowController

    root = tmp_path / "calibration"
    left = root / "left" / "left_001.png"
    right = root / "right" / "right_001.png"
    for path in (left, right):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(b"image")
    selection = {
        "root": "/old/checkout/calibration",
        "manual_mode": False,
        "left_paths": [],
        "right_paths": [],
        "excluded_left_paths": [
            "/old/checkout/calibration/left/left_missing.png"
        ],
        "excluded_right_paths": [],
        "excluded_pair_keys": [],
    }
    messages: list[str] = []
    panel = SimpleNamespace(
        _load_calibration_pair_selection=lambda: selection,
        calibration_status_label=SimpleNamespace(setText=messages.append),
        _log=messages.append,
    )
    controller = CalibrationWorkflowController.__new__(CalibrationWorkflowController)
    controller.panel = panel

    selected_left, selected_right, explicit = controller._selected_calibration_file_lists(
        root,
        [left],
        [right],
    )

    assert selected_left == []
    assert selected_right == []
    assert explicit is True
    assert any("all-photo fallback blocked" in message for message in messages)


def test_workspace_manual_selection_sidecar_contains_no_checkout_absolute_paths(
    tmp_path: Path,
) -> None:
    from Workstation.workspaces import CalibrationWorkspace

    root = tmp_path / "checkout" / "calibration" / "session-a"
    left = root / "left" / "left_001.png"
    right = root / "right" / "right_001.png"
    for path in (left, right):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(b"image")
    workspace = CalibrationWorkspace.__new__(CalibrationWorkspace)
    workspace.calibration_file_selection_summary = SimpleNamespace(
        setText=lambda _text: None
    )
    workspace._load_calibration_pair_selection = lambda: {
        "root": ".",
        "excluded_left_paths": [],
        "excluded_right_paths": [],
        "excluded_pair_keys": [],
    }

    workspace._save_calibration_file_selection(root, [left], [right])

    import json

    payload = json.loads(
        (root / "calibration_pair_selection.json").read_text(encoding="utf-8")
    )
    assert payload["schema"] == "workstation-calibration-file-selection-v2"
    assert payload["root"] == "."
    assert payload["left_paths"] == ["left/left_001.png"]
    assert payload["right_paths"] == ["right/right_001.png"]
    assert str(tmp_path) not in json.dumps(payload)


def test_workspace_surfaces_unrecoverable_selection_without_preview_fallback(
    tmp_path: Path,
) -> None:
    import json

    from Workstation.workspaces import CalibrationWorkspace

    root = tmp_path / "calibration"
    root.mkdir()
    selection = {
        "schema": "workstation-calibration-file-selection-v1",
        "root": "/old/checkout/calibration",
        "manual_mode": True,
        "left_paths": ["/old/checkout/calibration/left/left_missing.png"],
        "right_paths": [],
        "included_keys": [],
        "excluded_left_paths": [],
        "excluded_right_paths": [],
        "excluded_pair_keys": [],
    }
    (root / "calibration_pair_selection.json").write_text(
        json.dumps(selection), encoding="utf-8"
    )
    row_counts: list[int] = []
    messages: list[str] = []
    workspace = CalibrationWorkspace.__new__(CalibrationWorkspace)
    workspace._calibration_session_explicit_root = str(root)
    workspace.calibration_selected_files_table = SimpleNamespace(
        setRowCount=row_counts.append
    )
    workspace.calibration_file_selection_summary = SimpleNamespace(
        setText=messages.append
    )
    workspace._load_calibration_pair_selection = lambda: selection

    workspace._sync_calibration_file_selection()

    assert row_counts == [0]
    assert messages and "fallback 차단" in messages[-1]


def _intrinsics(image_count: int = 3):
    from Workstation.workstation_core.calibration import CameraIntrinsics

    return CameraIntrinsics(
        width=1280,
        height=720,
        camera_matrix=[
            [900.0, 0.0, 640.0],
            [0.0, 901.0, 360.0],
            [0.0, 0.0, 1.0],
        ],
        distortion_coefficients=[0.0],
        reprojection_error_px=0.2,
        image_count=image_count,
    )


def _result():
    from Workstation.workstation_core.calibration import (
        CheckerboardConfig,
        StereoIntrinsicsResult,
    )

    return StereoIntrinsicsResult(
        left=_intrinsics(),
        right=_intrinsics(),
        board=CheckerboardConfig(inner_cols=8, inner_rows=6, square_size_m=0.03),
    )


def test_worker_keeps_intrinsic_inputs_independent_and_stereo_uses_only_pairs(
    tmp_path: Path,
    monkeypatch,
) -> None:
    from Workstation import calibration_workers
    from Workstation.workstation_core.calibration import CheckerboardConfig, StereoExtrinsics

    left_paths = tuple(tmp_path / f"left_{index}.png" for index in (1, 2, 3))
    right_paths = tuple(tmp_path / f"right_{index}.png" for index in (1, 2, 3, 4))
    stereo_pairs = tuple(
        (f"{index:03d}", left_paths[index - 1], right_paths[index - 1])
        for index in (1, 2, 3)
    )
    calls: list[tuple[str, tuple[Path, ...], tuple[Path, ...] | None]] = []
    intrinsics = _intrinsics()
    extrinsics = StereoExtrinsics(
        rotation_left_to_right=[
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ],
        translation_left_to_right_m=[-0.3, 0.0, 0.0],
        reprojection_error_px=0.1,
        pair_count=1,
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

    def calibrate(paths, _config):
        calls.append(("intrinsic", tuple(paths), None))
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

    def stereo(left, right, _config, _left_intrinsics, _right_intrinsics):
        calls.append(("stereo", tuple(left), tuple(right)))
        return extrinsics

    monkeypatch.setattr(calibration_workers, "calibrate_camera_with_diagnostics", calibrate)
    monkeypatch.setattr(
        calibration_workers,
        "evaluate_camera_calibration_quality",
        lambda **_kwargs: quality,
    )
    monkeypatch.setattr(
        calibration_workers,
        "calibrate_stereo_extrinsics_from_image_pairs",
        stereo,
    )
    monkeypatch.setattr(calibration_workers, "analyze_calibration_images", lambda *_args, **_kwargs: ())

    worker = calibration_workers.CheckerboardCalibrationWorker(
        mode="checkerboard",
        config=CheckerboardConfig(inner_cols=8, inner_rows=6, square_size_m=0.03),
        left_intrinsic_paths=left_paths,
        right_intrinsic_paths=right_paths,
        stereo_pairs=stereo_pairs,
    )
    completed: list[object] = []
    worker.completed.connect(completed.append)
    worker.run()

    assert len(completed) == 1
    assert completed[0].calibration_quality["state"] == "PASS"
    assert completed[0].solver_diagnostics["left"]["calibration_flags"] == 0
    assert completed[0].solver_diagnostics["effective_selection"] == {
        "left_intrinsic_paths": [str(path) for path in left_paths],
        "right_intrinsic_paths": [str(path) for path in right_paths],
        "stereo_pairs": [
            [key, str(left), str(right)] for key, left, right in stereo_pairs
        ],
    }
    assert calls == [
        ("intrinsic", left_paths, None),
        ("intrinsic", right_paths, None),
        ("stereo", left_paths, right_paths[:3]),
    ]


def test_effective_selection_excludes_one_side_without_dropping_other_intrinsic(
    tmp_path: Path,
) -> None:
    from Workstation.calibration_controller import CalibrationWorkflowController

    root = tmp_path / "calibration"
    left_paths = [root / "left" / f"left_{key}.png" for key in ("001", "002", "003")]
    right_paths = [root / "right" / f"right_{key}.png" for key in ("001", "002", "003")]
    for path in (*left_paths, *right_paths):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(b"image")
    selection = {
        "manual_mode": False,
        "included_keys": [],
        "left_paths": [],
        "right_paths": [],
        "excluded_left_paths": [str(left_paths[0])],
        "excluded_right_paths": [],
        "excluded_pair_keys": [],
    }
    panel = SimpleNamespace(
        _load_calibration_pair_selection=lambda: selection,
        _calibration_dirs=lambda: (root, root / "left", root / "right"),
    )
    controller = CalibrationWorkflowController.__new__(CalibrationWorkflowController)
    controller.panel = panel
    scanned_pairs = [
        (key, left, right)
        for key, left, right in zip(("001", "002", "003"), left_paths, right_paths, strict=True)
    ]

    selected_left, selected_right, stereo_pairs, explicit = (
        controller._effective_calibration_inputs(
            root,
            left_paths,
            right_paths,
            scanned_pairs,
            mode="checkerboard",
        )
    )

    assert explicit is False
    assert selected_left == left_paths[1:]
    assert selected_right == right_paths
    assert [key for key, _left, _right in stereo_pairs] == ["002", "003"]


def test_pair_limit_is_applied_after_asymmetric_exclusions(tmp_path: Path) -> None:
    from Workstation.calibration_controller import CalibrationWorkflowController

    root = tmp_path / "calibration"
    keys = tuple(f"{index:03d}" for index in range(10))
    left_paths = [root / "left" / f"left_{key}.png" for key in keys]
    right_paths = [root / "right" / f"right_{key}.png" for key in keys]
    for path in (*left_paths, *right_paths):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(b"image")
    selection = {
        "manual_mode": False,
        "included_keys": [],
        "left_paths": [],
        "right_paths": [],
        "excluded_left_paths": [str(left_paths[0])],
        "excluded_right_paths": [],
        "excluded_pair_keys": [],
    }
    panel = SimpleNamespace(
        _load_calibration_pair_selection=lambda: selection,
        _calibration_dirs=lambda: (root, root / "left", root / "right"),
        calib_calculation_pairs_spin=SimpleNamespace(value=lambda: 4),
    )
    controller = CalibrationWorkflowController.__new__(CalibrationWorkflowController)
    controller.panel = panel
    scanned_pairs = [
        (key, left, right)
        for key, left, right in zip(keys, left_paths, right_paths, strict=True)
    ]

    selected_left, selected_right, stereo_pairs, explicit = (
        controller._effective_calibration_inputs(
            root,
            left_paths,
            right_paths,
            scanned_pairs,
            mode="checkerboard",
        )
    )

    assert explicit is False
    assert selected_left == left_paths[1:]
    assert selected_right == right_paths
    assert len(stereo_pairs) == 4
    assert all(key != "000" for key, _left, _right in stereo_pairs)
    assert all(left in selected_left and right in selected_right for _key, left, right in stereo_pairs)


def test_random_validation_reuses_reference_effective_selection_snapshot(tmp_path: Path) -> None:
    from Workstation.calibration_controller import CalibrationWorkflowController
    from Workstation.calibration_workers import CheckerboardRandomValidationWorker
    from Workstation.workstation_core.calibration import CheckerboardConfig

    reference = _result()
    left_paths = (tmp_path / "left_002.png", tmp_path / "left_003.png")
    right_paths = (
        tmp_path / "right_001.png",
        tmp_path / "right_002.png",
        tmp_path / "right_003.png",
    )
    pairs = (
        ("002", left_paths[0], right_paths[1]),
        ("003", left_paths[1], right_paths[2]),
    )

    def unexpected_rescan():
        raise AssertionError("reference validation must not rescan the original unfiltered dataset")

    panel = SimpleNamespace(
        checkerboard_calibration_selection_snapshot={
            "reference_result": reference,
            "root": tmp_path,
            "mode": "checkerboard",
            "left_intrinsic_paths": left_paths,
            "right_intrinsic_paths": right_paths,
            "stereo_pairs": pairs,
        },
        _checkerboard_validation_scan_counts=unexpected_rescan,
    )
    controller = CalibrationWorkflowController.__new__(CalibrationWorkflowController)
    controller.panel = panel

    _root, effective_left, effective_right, effective_pairs = (
        controller._random_validation_inputs(reference, mode="checkerboard")
    )
    worker = CheckerboardRandomValidationWorker(
        mode="checkerboard",
        config=CheckerboardConfig(inner_cols=8, inner_rows=6, square_size_m=0.03),
        runs=1,
        subset_count=2,
        seed=1,
        left_intrinsic_paths=effective_left,
        right_intrinsic_paths=effective_right,
        stereo_pairs=effective_pairs,
    )

    assert worker.left_intrinsic_paths == left_paths
    assert worker.right_intrinsic_paths == right_paths
    assert worker.stereo_pairs == pairs


def test_completed_result_replaces_snapshot_with_post_deduplication_selection(
    tmp_path: Path,
) -> None:
    from dataclasses import replace

    from Workstation.calibration_controller import CalibrationWorkflowController

    input_left = tuple(tmp_path / f"left_{index:03d}.png" for index in range(1, 4))
    input_right = tuple(tmp_path / f"right_{index:03d}.png" for index in range(1, 4))
    input_pairs = tuple(
        (f"{index:03d}", input_left[index - 1], input_right[index - 1])
        for index in range(1, 4)
    )
    effective_left = input_left[1:]
    effective_right = input_right[1:]
    effective_pairs = input_pairs[1:]
    result = replace(
        _result(),
        calibration_quality={"state": "PASS", "reason_codes": [], "reasons": []},
        solver_diagnostics={
            "effective_selection": {
                "left_intrinsic_paths": [str(path) for path in effective_left],
                "right_intrinsic_paths": [str(path) for path in effective_right],
                "stereo_pairs": [
                    [key, str(left), str(right)]
                    for key, left, right in effective_pairs
                ],
            }
        },
    )
    calls: list[dict[str, object]] = []
    panel = SimpleNamespace(
        _format_checkerboard_validation_result_summary=lambda *_args, **_kwargs: "summary",
        _checkerboard_validation_set_result=lambda *_args, **kwargs: calls.append(kwargs),
        _log=lambda _message: None,
        _refresh_operation_header=lambda: None,
    )
    controller = CalibrationWorkflowController.__new__(CalibrationWorkflowController)
    controller.panel = panel

    controller._handle_intrinsic_calibration_completed(
        result,
        mode="checkerboard",
        root=tmp_path,
        pair_count=len(input_pairs),
        min_images=1,
        capture_zoom_level=8,
        left_count=len(input_left),
        right_count=len(input_right),
        selection_snapshot={
            "root": tmp_path,
            "mode": "checkerboard",
            "left_intrinsic_paths": input_left,
            "right_intrinsic_paths": input_right,
            "stereo_pairs": input_pairs,
        },
    )

    snapshot = calls[0]["selection_snapshot"]
    assert snapshot["left_intrinsic_paths"] == effective_left
    assert snapshot["right_intrinsic_paths"] == effective_right
    assert snapshot["stereo_pairs"] == effective_pairs
    assert snapshot["input_stereo_pairs"] == input_pairs
    assert calls[0]["summary"]["stereo_pair_count"] == len(effective_pairs)
    assert calls[0]["summary"]["stereo_input_pair_count"] == len(input_pairs)


def test_missing_quality_snapshot_is_unassessed_and_side_fail_is_factual() -> None:
    from Workstation.calibration_controller import CalibrationWorkflowController

    controller = CalibrationWorkflowController.__new__(CalibrationWorkflowController)
    controller.panel = SimpleNamespace()
    state, reasons = controller._calibration_quality_state(_result())
    assert state == "UNASSESSED"
    assert "quality_snapshot_unavailable" in reasons

    controller.panel.calibration_quality_snapshot = {
        "left_quality": {"state": "PASS", "reason_codes": []},
        "right_quality": {
            "state": "FAIL",
            "reason_codes": ["principal_point_outside_image"],
        },
    }
    state, reasons = controller._calibration_quality_state(_result())
    assert state == "FAIL"
    assert "principal_point_outside_image" in reasons
