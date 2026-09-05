from __future__ import annotations

from dataclasses import replace
import json
from pathlib import Path
from types import SimpleNamespace

import time
import pytest
from PyQt5 import QtGui, QtWidgets

from Workstation.calibration_controller import CalibrationWorkflowController
from Workstation.workstation_core.calibration import (
    CameraIntrinsics,
    CheckerboardConfig,
    StereoExtrinsics,
    StereoExtrinsicsProfile,
    StereoIntrinsicsResult,
    compose_stereo_calibration,
    load_intrinsics_json,
    save_intrinsics_json,
    save_stereo_extrinsics_json,
)


class _Label:
    def __init__(self) -> None:
        self.text_value = ""

    def setText(self, value: str) -> None:  # noqa: N802 - Qt-compatible test double
        self.text_value = str(value)


def _intrinsics() -> CameraIntrinsics:
    return CameraIntrinsics(
        width=1280,
        height=720,
        camera_matrix=[
            [1200.0, 0.0, 640.0],
            [0.0, 1201.0, 360.0],
            [0.0, 0.0, 1.0],
        ],
        distortion_coefficients=[0.01, -0.02, 0.0, 0.0, 0.0],
        reprojection_error_px=0.2,
        image_count=20,
    )


def _result(state: str) -> StereoIntrinsicsResult:
    quality = {
        "schema": "workstation-camera-calibration-quality-v1",
        "state": state,
        "reason_codes": ([] if state == "PASS" else ["sentinel_quality_reason"]),
        "reasons": ([] if state == "PASS" else [{"code": "sentinel_quality_reason"}]),
    }
    return StereoIntrinsicsResult(
        left=_intrinsics(),
        right=_intrinsics(),
        board=CheckerboardConfig(10, 8, 0.1),
        left_quality={"state": state, "reason_codes": quality["reason_codes"]},
        right_quality={"state": state, "reason_codes": quality["reason_codes"]},
        calibration_quality=quality,
        solver_diagnostics={"left": {"subset_count": 3}, "right": {"subset_count": 3}},
    )


def test_quality_and_solver_diagnostics_survive_intrinsics_json_round_trip(tmp_path: Path) -> None:
    path = tmp_path / "intrinsics.json"
    expected = _result("WARNING")

    save_intrinsics_json(path, expected, metadata={"zoom_level": 8})
    recovered = load_intrinsics_json(path)

    assert recovered.calibration_quality == expected.calibration_quality
    assert recovered.left_quality == expected.left_quality
    assert recovered.right_quality == expected.right_quality
    assert recovered.solver_diagnostics == expected.solver_diagnostics


def test_intrinsics_json_hard_validation_overrides_serialized_pass(tmp_path: Path) -> None:
    path = tmp_path / "tampered-intrinsics.json"
    save_intrinsics_json(path, _result("PASS"), metadata={"zoom_level": 8})
    payload = json.loads(path.read_text(encoding="utf-8"))
    payload["left"]["camera_matrix"][0][2] = 1280.0
    path.write_text(json.dumps(payload), encoding="utf-8")

    with pytest.raises(ValueError, match="cx=.*outside"):
        load_intrinsics_json(path)


def test_backend_quality_gate_blocks_fail_and_requires_warning_confirmation() -> None:
    health: list[tuple[str, str]] = []
    panel = SimpleNamespace(
        checkerboard_validation_result=_result("FAIL"),
        checkerboard_validation_summary={},
        checkerboard_validation_status_label=_Label(),
        _set_health_label=lambda _label, text, level: health.append((str(text), str(level))),
    )
    controller = CalibrationWorkflowController(panel)

    assert controller._calibration_quality_allows_action("적용") is False
    assert health[-1][1] == "bad"

    panel.checkerboard_validation_result = _result("WARNING")
    assert controller._calibration_quality_allows_action("적용") is False
    assert health[-1][1] == "warn"
    assert controller._calibration_quality_allows_action(
        "적용", warning_confirmed=True
    ) is True
    assert panel.checkerboard_validation_summary["quality_override"]["confirmed"] is True


def test_direct_intrinsics_apply_requires_warning_confirmation(tmp_path: Path) -> None:
    health: list[tuple[str, str]] = []
    focal_values: list[float] = []
    panel = SimpleNamespace(
        last_calibration_result=_result("WARNING"),
        last_calibration_zoom_level=8,
        selected_extrinsics_profile=None,
        calibration_status_label=_Label(),
        checkerboard_validation_status_label=_Label(),
        checkerboard_validation_summary={},
        _active_intrinsics_profile_path="memory:warning",
        left_pane=SimpleNamespace(),
        right_pane=SimpleNamespace(),
        focal_spin=SimpleNamespace(setValue=lambda value: focal_values.append(float(value))),
        manual_focal_checkbox=SimpleNamespace(setChecked=lambda _value: None),
        _set_health_label=lambda _label, text, level: health.append((str(text), str(level))),
        _refresh_camera_parameters_tab=lambda: None,
        _refresh_distance=lambda: None,
        _refresh_optical_alignment_view=lambda: None,
        _refresh_operation_header=lambda: None,
        _calibration_dirs=lambda: (tmp_path, tmp_path / "left", tmp_path / "right"),
    )
    controller = CalibrationWorkflowController(panel)

    assert controller.apply_intrinsics_to_distance() is False
    assert not hasattr(panel, "active_intrinsics")
    assert health[-1][1] == "warn"

    assert controller.apply_intrinsics_to_distance(quality_warning_confirmed=True) is True
    assert panel.active_intrinsics is not None
    assert Path(panel._active_intrinsics_profile_path).is_file()
    assert Path(panel._active_intrinsics_profile_path).parent == tmp_path / "profiles" / "intrinsics"
    assert focal_values
    assert panel.checkerboard_validation_summary["quality_override"]["confirmed"] is True


def test_warning_stereo_profile_is_saved_as_attempt_without_promotion(
    tmp_path: Path,
) -> None:
    profile = StereoExtrinsicsProfile(
        extrinsics=StereoExtrinsics(
            rotation_left_to_right=[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            translation_left_to_right_m=[-0.3, 0.0, 0.0],
            reprojection_error_px=0.2,
            pair_count=12,
        ),
        source_zoom_level=8,
    )
    panel = SimpleNamespace(
        selected_extrinsics_profile=profile,
        last_calibration_result=_result("WARNING"),
        calibration_status_label=_Label(),
        _extrinsics_storage_path=lambda _zoom=None: tmp_path / "extrinsics_zoom_08.json",
        _ensure_calibration_session_root=lambda _purpose: tmp_path,
        _log=lambda _message: None,
        _refresh_operation_header=lambda: None,
    )

    saved = CalibrationWorkflowController(panel).save_stereo_extrinsics_profile()

    assert saved is True
    attempt_path = Path(panel._last_calibration_attempt_path)
    assert attempt_path.is_file()
    assert attempt_path.parent == tmp_path / "attempts"
    assert "warning" in attempt_path.name
    assert not hasattr(panel, "_selected_extrinsics_profile_path")
    assert "non-applicable" in panel.calibration_status_label.text_value


@pytest.mark.parametrize("quality_state", ["FAIL", "WARNING"])
def test_nonpass_intrinsics_are_saved_as_attempt_without_active_alias(
    tmp_path: Path,
    quality_state: str,
) -> None:
    health: list[tuple[str, str]] = []
    official = tmp_path / "intrinsics_zoom_08.json"
    base = tmp_path / "intrinsics.json"
    panel = SimpleNamespace(
        checkerboard_validation_result=_result(quality_state),
        checkerboard_validation_summary={"quality_state": quality_state},
        checkerboard_validation_status_label=_Label(),
        checkerboard_validation_result_mode="checkerboard",
        checkerboard_validation_result_saved=False,
        last_calibration_zoom_level=8,
        zoom_target_combo=SimpleNamespace(currentData=lambda: "both"),
        _calibration_dirs=lambda: (tmp_path, tmp_path / "left", tmp_path / "right"),
        _intrinsics_storage_path=lambda _zoom=None: official,
        _intrinsics_storage_base_path=lambda: base,
        _set_health_label=lambda _label, text, level: health.append((str(text), str(level))),
        _refresh_operation_header=lambda: None,
        _refresh_zoom_intrinsics_tab=lambda: None,
        _add_zoom_intrinsics_comparison_path=lambda _path: pytest.fail(
            "attempt must not be promoted into the approved comparison path"
        ),
        _save_current_settings=lambda: pytest.fail(
            "attempt must not replace persisted active settings"
        ),
    )
    controller = CalibrationWorkflowController(panel)
    controller._ensure_calibration_session_root = lambda _purpose: tmp_path

    controller.save_checkerboard_validation_result()

    attempt = Path(panel._last_calibration_attempt_path)
    assert attempt.is_file()
    assert attempt.parent == tmp_path / "attempts"
    assert quality_state.lower() in attempt.name
    assert not official.exists()
    assert not base.exists()
    assert not hasattr(panel, "_active_intrinsics_profile_path")
    assert health[-1][1] == ("bad" if quality_state == "FAIL" else "warn")


def test_current_result_quality_snapshot_wins_over_stale_panel_snapshot() -> None:
    panel = SimpleNamespace(
        checkerboard_validation_result=_result("PASS"),
        last_calibration_result=_result("FAIL"),
        calibration_quality={"state": "PASS", "reason_codes": []},
    )
    controller = CalibrationWorkflowController(panel)

    snapshot = controller.workstation_calibration_quality_snapshot()

    assert snapshot["state"] == "FAIL"
    assert "sentinel_quality_reason" in snapshot["reason_codes"]


def test_current_legacy_result_does_not_inherit_stale_pass_snapshot() -> None:
    legacy = StereoIntrinsicsResult(
        left=_intrinsics(),
        right=_intrinsics(),
        board=CheckerboardConfig(10, 8, 0.1),
    )
    panel = SimpleNamespace(
        last_calibration_result=legacy,
        checkerboard_validation_result=_result("PASS"),
        calibration_quality={"state": "PASS", "reason_codes": []},
    )

    snapshot = CalibrationWorkflowController(panel).workstation_calibration_quality_snapshot()

    assert snapshot["state"] == "UNVERIFIED"
    assert snapshot["reason_codes"] == ["quality_snapshot_unavailable"]


def test_legacy_unassessed_result_remains_compatible_but_is_not_pass() -> None:
    panel = SimpleNamespace(
        checkerboard_validation_result=StereoIntrinsicsResult(
            left=_intrinsics(),
            right=_intrinsics(),
            board=CheckerboardConfig(10, 8, 0.1),
        ),
        checkerboard_validation_summary={},
        checkerboard_validation_status_label=_Label(),
        _set_health_label=lambda *_args: None,
    )
    controller = CalibrationWorkflowController(panel)

    state, reasons = controller._calibration_quality_state(
        panel.checkerboard_validation_result
    )
    assert state == "UNASSESSED"
    assert "quality_snapshot_unavailable" in reasons
    assert controller._calibration_quality_allows_action("적용") is True


def _checkerboard_corners(
    *, columns: int = 10, rows: int = 8, x0: float = 250.0, y0: float = 160.0
) -> list[tuple[float, float]]:
    return [
        (x0 + column * 55.0, y0 + row * 48.0)
        for row in range(rows)
        for column in range(columns)
    ]


def test_auto_capture_metadata_requires_exact_au_and_decoded_pts() -> None:
    from Workstation.calibration_controller import (
        is_nal1_non_idr_auto_capture_metadata,
    )

    metadata = {
        "h264_access_unit_kind": "nal1_non_idr_intra",
        "h264_access_unit_nal_types": [7, 8, 1],
        "h264_access_unit_is_delta": False,
        "h264_access_unit_pts_ns": 123,
        "decoded_buffer_pts_ns": 123,
        "h264_nal1_non_idr_intra": True,
        "h264_random_access_guaranteed": False,
    }
    assert is_nal1_non_idr_auto_capture_metadata(metadata) is True

    metadata["decoded_buffer_pts_ns"] = 124
    assert is_nal1_non_idr_auto_capture_metadata(metadata) is False
    metadata["decoded_buffer_pts_ns"] = 123
    metadata["h264_access_unit_nal_types"] = [1, 5]
    assert is_nal1_non_idr_auto_capture_metadata(metadata) is False


def test_auto_capture_uses_native_raster_and_rejects_repeated_pose() -> None:
    from Workstation.workstation_core.native_frame_ring import FrameKey

    _app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    preview = QtGui.QImage(1280, 720, QtGui.QImage.Format_RGB888)
    native = QtGui.QImage(1920, 1080, QtGui.QImage.Format_RGB888)
    preview.fill(10)
    native.fill(20)
    now_s = time.monotonic()

    class Pane:
        def __init__(self, side: str, rtcp: float) -> None:
            self.side = side
            self.rtcp = rtcp

        def calibration_native_frame_for_key(self, key: FrameKey, **_kwargs):
            assert key.camera_id == self.side
            metadata = {
                **key.to_mapping(),
                "timestamp_s": now_s,
                "rtcp_frame_ntp_unix_s": self.rtcp,
                "native_width": 1920,
                "native_height": 1080,
                "native_lossless_source": True,
                "overlay_applied": False,
                "source_native_frame_id": key.text,
                "h264_access_unit_kind": "nal1_non_idr_intra",
                "h264_access_unit_nal_types": [1],
                "h264_access_unit_is_delta": False,
                "h264_access_unit_pts_ns": key.frame_id * 1_000,
                "decoded_buffer_pts_ns": key.frame_id * 1_000,
                "h264_nal1_non_idr_intra": True,
                "h264_random_access_guaranteed": False,
            }
            return (
                key.text,
                native.copy(),
                metadata,
            )

    status = QtWidgets.QLabel()
    panel = SimpleNamespace(
        checkerboard_calibration_thread=None,
        calibration_status_label=status,
        left_pane=Pane("left", 100.000),
        right_pane=Pane("right", 100.002),
        calibration_preview_detection_results={},
        _checkerboard_config=lambda: CheckerboardConfig(10, 8, 0.1),
        _checkerboard_capture_pair_timing=lambda _metadata: (
            True,
            "rtcp-ntp",
            0.002,
            "ok",
        ),
    )
    corners = _checkerboard_corners()
    for side, request_id, rtcp in (("left", 1, 100.000), ("right", 2, 100.002)):
        key = FrameKey(side, 7, request_id)
        panel.calibration_preview_detection_results[side] = {
            "request_id": request_id,
            "image": preview.copy(),
            "found": True,
            "corners": corners,
            "metadata": {
                **key.to_mapping(),
                "timestamp_s": now_s,
                "rtcp_frame_ntp_unix_s": rtcp,
                "h264_access_unit_kind": "nal1_non_idr_intra",
                "h264_access_unit_nal_types": [1],
                "h264_access_unit_is_delta": False,
                "h264_access_unit_pts_ns": request_id * 1_000,
                "decoded_buffer_pts_ns": request_id * 1_000,
                "h264_nal1_non_idr_intra": True,
                "h264_random_access_guaranteed": False,
            },
        }
    controller = CalibrationWorkflowController(panel)
    controller._auto_capture_active = True
    captures: list[dict[str, object]] = []
    controller.capture_calibration_images = lambda **kwargs: captures.append(kwargs) or True  # type: ignore[method-assign]

    controller._auto_calibration_capture_tick()
    assert len(captures) == 1
    assert captures[0]["frames"]["left"].size() == native.size()
    assert captures[0]["metadata_by_camera"]["left"]["frame_id"] == "left:7:1"

    controller._auto_capture_pose_views = {
        "left": [controller._auto_capture_pending_pose_views["left"]],
        "right": [],
    }
    controller._auto_capture_inflight_pair_key = None
    panel.calibration_preview_detection_results["left"]["request_id"] = 3
    panel.calibration_preview_detection_results["right"]["request_id"] = 4
    controller._auto_calibration_capture_tick()

    assert len(captures) == 1
    assert "거의 같" in status.text()


def test_auto_capture_waits_for_positive_nal1_metadata_before_native_lookup() -> None:
    preview = QtGui.QImage(32, 24, QtGui.QImage.Format_RGB888)
    preview.fill(0)
    native_calls: list[object] = []

    class Pane:
        def calibration_native_frame_for_key(self, key, **_kwargs):
            native_calls.append(key)
            return None

    panel = SimpleNamespace(
        checkerboard_calibration_thread=None,
        calibration_status_label=QtWidgets.QLabel(),
        left_pane=Pane(),
        right_pane=Pane(),
        calibration_preview_detection_results={
            side: {
                "request_id": index,
                "image": preview.copy(),
                "found": True,
                "corners": _checkerboard_corners(columns=2, rows=2),
                "metadata": {"timestamp_s": time.monotonic()},
            }
            for index, side in enumerate(("left", "right"), start=1)
        },
    )
    controller = CalibrationWorkflowController(panel)
    controller._auto_capture_active = True

    controller._auto_calibration_capture_tick()

    assert native_calls == []
    assert "NAL1 non-delta" in panel.calibration_status_label.text()


def test_auto_capture_does_not_fall_back_when_exact_frame_key_is_evicted() -> None:
    from Workstation.workstation_core.native_frame_ring import FrameKey

    preview = QtGui.QImage(640, 480, QtGui.QImage.Format_RGB888)
    preview.fill(0)
    exact_calls: list[FrameKey] = []

    class Pane:
        def calibration_native_frame_for_key(self, key: FrameKey, **_kwargs):
            exact_calls.append(key)
            return None

        def calibration_native_frame_near(self, *_args, **_kwargs):
            raise AssertionError("Auto capture must not use timestamp-near fallback")

    detections: dict[str, dict[str, object]] = {}
    for index, side in enumerate(("left", "right"), start=1):
        key = FrameKey(side, 9, index)
        detections[side] = {
            "request_id": index,
            "image": preview.copy(),
            "found": True,
            "corners": _checkerboard_corners(columns=2, rows=2),
            "metadata": {
                **key.to_mapping(),
                "timestamp_s": time.monotonic(),
                "h264_access_unit_kind": "nal1_non_idr_intra",
                "h264_access_unit_nal_types": [1],
                "h264_access_unit_is_delta": False,
                "h264_access_unit_pts_ns": index * 100,
                "decoded_buffer_pts_ns": index * 100,
                "h264_nal1_non_idr_intra": True,
                "h264_random_access_guaranteed": False,
            },
        }
    panel = SimpleNamespace(
        checkerboard_calibration_thread=None,
        calibration_status_label=QtWidgets.QLabel(),
        left_pane=Pane(),
        right_pane=Pane(),
        calibration_preview_detection_results=detections,
        _checkerboard_config=lambda: CheckerboardConfig(2, 2, 0.1),
    )
    controller = CalibrationWorkflowController(panel)
    controller._auto_capture_active = True
    captures: list[object] = []
    controller.capture_calibration_images = lambda **kwargs: captures.append(kwargs) or True  # type: ignore[method-assign]

    controller._auto_calibration_capture_tick()

    assert exact_calls == [FrameKey("left", 9, 1)]
    assert captures == []
    assert "exact native FrameKey" in panel.calibration_status_label.text()


def test_manual_capture_rejects_implicit_preview_fallback() -> None:
    _app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    preview = QtGui.QImage(1280, 720, QtGui.QImage.Format_RGB888)
    preview.fill(0)
    messages: list[str] = []
    panel = SimpleNamespace(
        checkerboard_calibration_thread=None,
        calibration_status_label=_Label(),
        left_pane=SimpleNamespace(current_image=lambda: preview),
        right_pane=SimpleNamespace(current_image=lambda: preview),
        _log=messages.append,
    )

    captured = CalibrationWorkflowController(panel).capture_calibration_images()

    assert captured is False
    assert "native lossless frame" in panel.calibration_status_label.text_value
    assert messages


def test_stopped_auto_capture_does_not_count_inflight_completion() -> None:
    status = _Label()
    panel = SimpleNamespace(
        calibration_status_label=status,
        _calibration_session_root=None,
        _log=lambda _message: None,
    )
    controller = CalibrationWorkflowController.__new__(CalibrationWorkflowController)
    controller.panel = panel
    controller._capture_origin = "auto"
    controller._auto_capture_active = False
    controller._auto_capture_saved_pairs = 0
    controller._auto_capture_inflight_pair_key = None
    controller._auto_capture_pending_pose_views = None
    controller._auto_capture_pending_pose_novelty = None

    controller._handle_checkerboard_capture_completed(
        {
            "stem": "capture-after-stop",
            "saved_sides": ("left", "right"),
            "status_parts": ("left saved", "right saved"),
            "pair_timing_ok": True,
            "pair_time_basis": "rtcp-ntp",
            "pair_delta_s": 0.001,
        }
    )

    assert controller._auto_capture_saved_pairs == 0
    assert "횟수에는 포함하지 않았습니다" in status.text_value
    assert controller._capture_origin == "manual"


def test_apply_blocks_profile_grid_that_differs_from_live_native_frame() -> None:
    native = QtGui.QImage(1920, 1080, QtGui.QImage.Format_RGB888)
    native.fill(0)
    health: list[tuple[str, str]] = []
    pane = SimpleNamespace(
        latest_native_calibration_frame=lambda **_kwargs: (
            "native-grid",
            native.copy(),
            {
                "timestamp_s": time.monotonic(),
                "native_width": 1920,
                "native_height": 1080,
                "native_lossless_source": True,
                "overlay_applied": False,
                "source_native_frame_id": "native-grid",
            },
        )
    )
    panel = SimpleNamespace(
        last_calibration_result=_result("PASS"),
        calibration_status_label=_Label(),
        checkerboard_validation_status_label=_Label(),
        left_pane=pane,
        right_pane=pane,
        _set_health_label=lambda _label, text, level: health.append((str(text), str(level))),
    )

    CalibrationWorkflowController(panel).apply_intrinsics_to_distance()

    assert "differs from live native grid" in panel.calibration_status_label.text_value
    assert health[-1] == ("적용 차단: left native pixel grid mismatch", "bad")


def test_full_checkerboard_apply_uses_its_own_rt_not_stale_selected_profile(
    tmp_path: Path,
) -> None:
    old_extrinsics = StereoExtrinsics(
        rotation_left_to_right=[
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ],
        translation_left_to_right_m=[-0.3, 0.0, 0.0],
        reprojection_error_px=0.4,
        pair_count=12,
    )
    fresh_extrinsics = replace(
        old_extrinsics,
        translation_left_to_right_m=[-0.7, 0.02, -0.01],
        reprojection_error_px=0.15,
        pair_count=24,
    )
    old_profile = StereoExtrinsicsProfile(
        extrinsics=old_extrinsics,
        source_zoom_level=4,
        method="checkerboard_external_only",
        profile_id="stale-a",
    )
    old_path = tmp_path / "profiles" / "extrinsics" / "extrinsics-old.json"
    old_path.parent.mkdir(parents=True)
    save_stereo_extrinsics_json(old_path, old_profile)
    fresh_result = replace(_result("PASS"), extrinsics=fresh_extrinsics)
    health: list[tuple[str, str]] = []
    distance_mode = SimpleNamespace(current=None)
    panel = SimpleNamespace(
        checkerboard_validation_result=fresh_result,
        checkerboard_validation_result_mode="checkerboard",
        checkerboard_validation_result_saved=False,
        checkerboard_validation_summary={},
        checkerboard_validation_status_label=_Label(),
        last_calibration_result=fresh_result,
        last_calibration_zoom_level=8,
        selected_extrinsics_profile=old_profile,
        active_extrinsics_profile=old_profile,
        checkerboard_extrinsics_profile=old_profile,
        optical_pose_extrinsics_profile=None,
        active_intrinsics=compose_stereo_calibration(_result("PASS"), old_profile),
        _candidate_intrinsics_profile_path="memory:fresh-full-result",
        _active_intrinsics_profile_path="memory:old-intrinsics",
        _active_intrinsics_profile_hash="a" * 64,
        _active_intrinsics_source_zoom_level=4,
        _selected_extrinsics_profile_path=str(old_path),
        _selected_extrinsics_profile_hash=old_profile.content_sha256,
        _active_extrinsics_profile_path=str(old_path),
        _active_extrinsics_profile_hash=old_profile.content_sha256,
        _active_extrinsics_source_zoom_level=4,
        _active_calibration_selection_mode="manual_same_zoom",
        distance_mode_combo=distance_mode,
        left_pane=SimpleNamespace(),
        right_pane=SimpleNamespace(),
        focal_spin=SimpleNamespace(setValue=lambda _value: None),
        manual_focal_checkbox=SimpleNamespace(setChecked=lambda _value: None),
        calibration_status_label=_Label(),
        _calibration_dirs=lambda: (tmp_path, tmp_path / "left", tmp_path / "right"),
        _set_combo_data=lambda combo, value: setattr(combo, "current", value),
        _set_health_label=lambda _label, text, level: health.append((str(text), str(level))),
        _save_current_settings=lambda: None,
        _refresh_camera_parameters_tab=lambda: None,
        _refresh_distance=lambda: None,
        _refresh_optical_alignment_view=lambda: None,
        _refresh_operation_header=lambda: None,
    )
    controller = CalibrationWorkflowController(panel)
    panel._calibration_quality_allows_action = controller._calibration_quality_allows_action
    panel.apply_intrinsics_to_distance = controller.apply_intrinsics_to_distance

    controller.apply_checkerboard_validation_result()

    assert panel.active_intrinsics.extrinsics is fresh_extrinsics
    assert panel.active_intrinsics.extrinsics is not old_extrinsics
    assert panel.selected_extrinsics_profile.extrinsics is fresh_extrinsics
    assert panel.active_extrinsics_profile is panel.selected_extrinsics_profile
    assert panel.checkerboard_extrinsics_profile is panel.selected_extrinsics_profile
    assert panel._active_extrinsics_profile_hash == (
        panel.selected_extrinsics_profile.content_sha256
    )
    assert panel._active_extrinsics_profile_path != str(old_path)
    assert Path(panel._active_extrinsics_profile_path).is_file()
    assert distance_mode.current == "checkerboard"
    assert health[-1][1] == "good"
