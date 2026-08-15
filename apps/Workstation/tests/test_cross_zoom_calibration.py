from pathlib import Path
from types import SimpleNamespace

import pytest

from Workstation.workstation_core.calibration import (
    CameraIntrinsics,
    CheckerboardConfig,
    StereoExtrinsics,
    StereoExtrinsicsProfile,
    StereoIntrinsicsResult,
    compose_stereo_calibration,
    independent_zoom_selection_mode,
    intrinsics_content_sha256,
    load_stereo_extrinsics_json,
    save_intrinsics_json,
    save_stereo_extrinsics_json,
)
from Workstation.workstation_core.measurement_attempt import StereoIntrinsicsSnapshot
from Workstation.workstation_core.wiris_zoom import parse_visible_zoom_response


def _intrinsics_result(focal_px: float, *, extrinsics=None) -> StereoIntrinsicsResult:
    camera = CameraIntrinsics(
        width=1280,
        height=720,
        camera_matrix=[
            [focal_px, 0.0, 640.0],
            [0.0, focal_px, 360.0],
            [0.0, 0.0, 1.0],
        ],
        distortion_coefficients=[0.01, -0.02, 0.0, 0.0, 0.0],
        reprojection_error_px=0.2,
        image_count=24,
    )
    return StereoIntrinsicsResult(
        left=camera,
        right=camera,
        board=CheckerboardConfig(13, 8, 0.04),
        extrinsics=extrinsics,
    )


def _extrinsics() -> StereoExtrinsics:
    return StereoExtrinsics(
        rotation_left_to_right=[
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ],
        translation_left_to_right_m=[-0.3, 0.0, 0.0],
        reprojection_error_px=0.15,
        pair_count=20,
    )


def test_cross_zoom_composition_uses_arbitrary_operating_and_calibration_zooms_without_gate() -> None:
    operating_zoom = _intrinsics_result(7200.0)
    profile = StereoExtrinsicsProfile(
        extrinsics=_extrinsics(),
        source_zoom_level=3,
        source_intrinsics_path="intrinsics_zoom_03.json",
        source_intrinsics_sha256=intrinsics_content_sha256(_intrinsics_result(900.0)),
    )

    composed = compose_stereo_calibration(operating_zoom, profile)

    assert composed.left.camera_matrix[0][0] == 7200.0
    assert composed.extrinsics is profile.extrinsics
    assert profile.source_zoom_level == 3
    assert profile.right_camera_origin_in_left_rdf_m == pytest.approx((0.3, 0.0, 0.0))
    assert profile.baseline_m == pytest.approx(0.3)


@pytest.mark.parametrize(
    ("intrinsics_zoom", "extrinsics_zoom", "expected"),
    (
        (12, 3, "manual_cross_zoom"),
        (7, 7, "manual_same_zoom"),
        (12, None, "manual_independent_zoom"),
        (None, 3, "manual_independent_zoom"),
    ),
)
def test_independent_zoom_selection_mode_describes_actual_source_relationship(
    intrinsics_zoom,
    extrinsics_zoom,
    expected,
) -> None:
    assert independent_zoom_selection_mode(intrinsics_zoom, extrinsics_zoom) == expected


def test_standalone_extrinsics_profile_round_trip_preserves_contract_and_provenance(
    tmp_path: Path,
) -> None:
    profile = StereoExtrinsicsProfile(
        extrinsics=_extrinsics(),
        source_zoom_level=3,
        method="checkerboard_external_only",
        source_intrinsics_path="intrinsics_zoom_03.json",
        source_intrinsics_sha256=intrinsics_content_sha256(_intrinsics_result(900.0)),
        profile_id="large-board-03",
        metadata={"validation_managed_by_application": False},
    )
    path = tmp_path / "extrinsics_zoom_03_large_board.json"

    save_stereo_extrinsics_json(path, profile)
    loaded = load_stereo_extrinsics_json(path)

    assert loaded.source_zoom_level == 3
    assert loaded.method == "checkerboard_external_only"
    assert loaded.profile_id == "large-board-03"
    assert loaded.extrinsics.translation_left_to_right_m == [-0.3, 0.0, 0.0]
    assert loaded.content_sha256 == profile.content_sha256
    assert loaded.metadata["validation_managed_by_application"] is False


def test_extrinsics_loader_accepts_legacy_intrinsics_json_with_embedded_rt(
    tmp_path: Path,
) -> None:
    path = tmp_path / "intrinsics_zoom_01.json"
    save_intrinsics_json(
        path,
        _intrinsics_result(900.0, extrinsics=_extrinsics()),
        metadata={"zoom_level": 1, "calculation_mode": "checkerboard"},
    )

    profile = load_stereo_extrinsics_json(path)

    assert profile.source_zoom_level == 1
    assert profile.method == "checkerboard"
    assert profile.source_intrinsics_path == str(path)


def test_measurement_snapshot_records_independent_cross_zoom_sources() -> None:
    composed = compose_stereo_calibration(_intrinsics_result(7200.0), _extrinsics())

    snapshot = StereoIntrinsicsSnapshot.from_result(
        composed,
        active_zoom_level=12,
        selected_profile_path="intrinsics_zoom_12.json",
        selected_extrinsics_profile_path="extrinsics_zoom_03_large_board.json",
        intrinsics_source_zoom_level=12,
        extrinsics_source_zoom_level=3,
        selection_mode="manual_cross_zoom",
    )

    assert snapshot.active_zoom_level == 12
    assert snapshot.intrinsics_source_zoom_level == 12
    assert snapshot.extrinsics_source_zoom_level == 3
    assert snapshot.selected_extrinsics_profile_path.endswith("zoom_03_large_board.json")
    assert len(snapshot.intrinsics_content_sha256) == 64
    assert len(snapshot.extrinsics_content_sha256) == 64
    assert snapshot.validation_gate == "disabled"


def test_extrinsics_only_worker_holds_selected_intrinsics_fixed(monkeypatch) -> None:
    from Workstation import calibration_workers

    selected = _intrinsics_result(900.0)
    expected = _extrinsics()
    captured = {}

    def fake_calibrate(
        left_paths,
        right_paths,
        config,
        left_intrinsics,
        right_intrinsics,
        *,
        progress_callback=None,
    ):
        captured.update(
            {
                "left_paths": tuple(left_paths),
                "right_paths": tuple(right_paths),
                "config": config,
                "left": left_intrinsics,
                "right": right_intrinsics,
            }
        )
        if progress_callback is not None:
            progress_callback(3, 3, "detect")
        return expected

    monkeypatch.setattr(
        calibration_workers,
        "calibrate_stereo_extrinsics_from_image_pairs",
        fake_calibrate,
    )
    worker = calibration_workers.CheckerboardExtrinsicsWorker(
        left_paths=(Path("left_001.png"), Path("left_002.png"), Path("left_003.png")),
        right_paths=(Path("right_001.png"), Path("right_002.png"), Path("right_003.png")),
        config=selected.board,
        left_intrinsics=selected.left,
        right_intrinsics=selected.right,
    )
    results = []
    worker.completed.connect(results.append)

    worker.run()

    assert results == [expected]
    assert captured["left"] is selected.left
    assert captured["right"] is selected.right


def test_panel_apply_method_composes_cross_zoom_selection_without_validation_state() -> None:
    from Workstation import standalone_gui

    class ValueWidget:
        def __init__(self, value=None):
            self.value_state = value

        def value(self):
            return self.value_state

        def setValue(self, value):
            self.value_state = value

        def setChecked(self, value):
            self.value_state = bool(value)

        def setText(self, value):
            self.value_state = str(value)

        def text(self):
            return str(self.value_state)

    operating_zoom = _intrinsics_result(7200.0)
    profile = StereoExtrinsicsProfile(
        extrinsics=_extrinsics(),
        source_zoom_level=3,
        profile_id="zoom03-large-board",
    )
    panel = SimpleNamespace(
        last_calibration_result=operating_zoom,
        last_calibration_zoom_level=12,
        selected_extrinsics_profile=profile,
        _active_intrinsics_profile_path="intrinsics_zoom_12.json",
        _selected_extrinsics_profile_path="extrinsics_zoom_03_large_board.json",
        _selected_extrinsics_profile_hash=profile.content_sha256,
        distance_mode_combo=SimpleNamespace(current=None),
        focal_spin=ValueWidget(),
        manual_focal_checkbox=ValueWidget(),
        calibration_status_label=ValueWidget(),
        _set_combo_data=lambda combo, value: setattr(combo, "current", value),
        _refresh_camera_parameters_tab=lambda: None,
        _refresh_distance=lambda: None,
        _refresh_optical_alignment_view=lambda: None,
        _refresh_operation_header=lambda: None,
    )

    standalone_gui.WirisStereoDistancePanel.apply_intrinsics_to_distance(panel)

    assert panel.active_intrinsics.left.camera_matrix[0][0] == 7200.0
    assert panel.active_intrinsics.extrinsics is profile.extrinsics
    assert panel._active_intrinsics_source_zoom_level == 12
    assert panel._active_extrinsics_source_zoom_level == 3
    assert panel._active_calibration_selection_mode == "manual_cross_zoom"
    assert panel.distance_mode_combo.current == "checkerboard"
    assert "validation not gated" in panel.calibration_status_label.text()


def test_intrinsics_zoom_mismatch_is_logged_but_never_blocks() -> None:
    from Workstation import standalone_gui

    messages = []
    panel = SimpleNamespace(
        zoom_level_spin=SimpleNamespace(value=lambda: 8),
        calibration_status_label=SimpleNamespace(setText=messages.append),
        _log=messages.append,
    )

    allowed = standalone_gui.WirisStereoDistancePanel._confirm_intrinsics_profile_mismatch(
        panel,
        Path("intrinsics_zoom_01.json"),
        _intrinsics_result(900.0),
        1,
    )

    assert allowed is True
    assert any("non-blocking" in message for message in messages)


@pytest.mark.parametrize(
    ("response", "expected"),
    (("8 8.0", (8, 8.0)), ("OK GZVV 1 1.0x", (1, 1.0)), ("ERR", (None, None))),
)
def test_visible_zoom_readback_parser_is_informational(response, expected) -> None:
    assert parse_visible_zoom_response(response) == expected
