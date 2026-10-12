"""Detached domain evidence, producer frames, schema shapes and failure isolation."""

import ast
import json
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from Workstation import evidence as evidence_module
from Workstation.evidence import EvidenceAdapter, _copy_gps_state
from Workstation.evidence_views.geometry import project_baseline
from Workstation.evidence_views.snapshot import snapshot_mapping, thaw_snapshot
from Workstation.workstation_core.gnss_operation import GnssOperationConfig, HeightType
from Workstation.workstation_core.gnss_profile import GnssCoordinateSample
from Workstation.workstation_core.pixhawk_attitude import MAVLINK_MAG_STREAMS
from Workstation.workstation_core.rtk_geometry import GpsPositionState


def test_calibration_capture_owns_nested_arrays_profiles_and_optical_origin():
    matrix = np.array([[801.0, 0.0, 321.0], [0.0, 902.0, 242.0], [0.0, 0.0, 1.0]])
    camera = SimpleNamespace(
        camera_matrix=matrix, width=640, height=480, distortion_coefficients=[0.1, -0.2], image_count=12
    )
    rotation = [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]
    translation = [2.0, 3.0, 5.0]
    extrinsics = SimpleNamespace(
        rotation_left_to_right=rotation,
        translation_left_to_right_m=translation,
        pair_count=9,
        reprojection_error_px=0.3,
    )
    metadata = {"source": {"reasons": ["captured"]}}
    profile = SimpleNamespace(
        profile_id="profile-A", method="fixture", metadata=metadata, source_zoom_level=4, extrinsics=extrinsics
    )
    quality = {"state": "PASS", "reason_codes": ["old"]}
    panel = SimpleNamespace(
        active_intrinsics=SimpleNamespace(left=camera, right=camera, extrinsics=extrinsics),
        active_extrinsics_profile=profile,
        calibration_quality=quality,
        _active_intrinsics_source_zoom_level=2,
        _active_extrinsics_source_zoom_level=4,
    )
    captured = EvidenceAdapter(panel)._capture_calibration()
    matrix[0, 0] = 999.0
    rotation[0][1] = 0.0
    translation[0] = 100.0
    profile.profile_id = "profile-B"
    metadata["source"]["reasons"].append("new")
    quality["reason_codes"].append("new")
    with pytest.raises(TypeError):
        captured.geometry["left"]["fx_px"] = 7.0
    result = captured.project()
    assert result["left"]["fx_px"] == 801.0
    assert result["left"]["fy_px"] == 902.0
    assert result["extrinsics"]["translation_left_to_right_m"] == (2.0, 3.0, 5.0)
    # Independent -R.T @ T for a +90 degree optical rotation, asymmetric T.
    assert result["extrinsics"]["right_camera_origin_in_left_rdf_m"] == (-3.0, 2.0, -5.0)
    assert result["extrinsics_profile_id"] == "profile-A"
    assert result["extrinsics_metadata"] == {"source": {"reasons": ["captured"]}}
    assert result["calibration_quality"]["reason_codes"] == ["old"]
    assert result["cross_zoom"] is True
    json.dumps(result)
    result["extrinsics_metadata"]["source"]["reasons"].append("caller")
    assert captured.project()["extrinsics_metadata"] == {"source": {"reasons": ["captured"]}}


def test_mag_capture_preserves_sensor_axes_units_and_independent_yaw():
    sensor_slot = MAVLINK_MAG_STREAMS[0][0]
    source = SimpleNamespace(
        x=3.0, y=-4.0, z=12.0, magnitude=13.0, device_id=12345, timestamp_s=99.9, topic="mavlink/SCALED_IMU"
    )
    attitude = {"yaw_deg": 75.0, "age_s": 0.1, "source": "fixture", "q_wxyz": [1.0, 0.0, 0.0, 0.0]}
    panel = SimpleNamespace(
        latest_mag_sensors={"left": {sensor_slot: source}}, latest_vehicle_mag_device_ids={"left": 12345}
    )
    adapter = EvidenceAdapter(panel)
    adapter.attitude = lambda side, now_s: attitude
    projection = adapter._magnetometer_projection("left", 100.0)
    source.x = 100.0
    panel.latest_vehicle_mag_device_ids["left"] = 777
    attitude["yaw_deg"] = 180.0
    result = projection.magnetometers("left")[0]
    assert (result["xmag"], result["ymag"], result["zmag"], result["norm"]) == (3.0, -4.0, 12.0, 13.0)
    assert result["unit"] == "mG"
    assert result["raw_yaw_deg"] == pytest.approx(53.13010235415598)
    assert result["active_device_id"] == 12345
    assert result["px4_yaw_deg"] == 75.0
    assert "sensor axes" in result["axis_contract"]


def test_gnss_capture_isolates_optional_manager_failure_and_preserves_datums_ecef():
    def unavailable():
        raise RuntimeError("metadata unavailable")

    readiness = {"ready": False, "reasons": ["old"]}
    coordinate = GnssCoordinateSample(
        35.25, -122.5, 55.0, msl_altitude_m=22.0, height_type=HeightType.ELLIPSOID, timestamp_s=99.2, fixed=True
    )
    stats = {
        "start_s": 90.0,
        "frames": 10,
        "bytes": 400,
        "last_frame_s": 99.5,
        "message_types": {1005: 1, 1077: 9},
        "injection": {"left": {"bytes": 300, "last_delivery_s": 99.0}},
    }
    survey = {"mean_x_m": 101.0, "mean_y_m": -203.0, "mean_z_m": 307.0, "valid": True}
    panel = SimpleNamespace(
        gnss_operation_manager=SimpleNamespace(
            metadata=unavailable,
            config=GnssOperationConfig(),
            state="CONFIGURED",
            readiness=lambda: SimpleNamespace(to_metadata=lambda: readiness),
        ),
        gnss_coordinate_samples=[coordinate],
        gnss_base_rtcm_stats=stats,
        latest_gnss_base_survey_in=survey,
    )
    adapter = EvidenceAdapter(panel)
    captured = adapter._capture_gnss_operation(100.0)
    stats["injection"]["left"]["bytes"] = 999
    survey["mean_y_m"] = 0.0
    readiness["reasons"].append("new")
    panel.gnss_coordinate_samples.clear()
    result = captured.project(now=100.0)
    assert result["survey_in"]["mean_ecef_m"] == (101.0, -203.0, 307.0)
    assert result["base_coordinate"]["ellipsoid_height_m"] == 55.0
    assert result["base_coordinate"]["msl_altitude_m"] == 22.0
    assert result["base_coordinate"]["age_s"] == pytest.approx(0.8)
    assert result["routes"]["left"]["bytes"] == 300
    assert result["readiness"]["reasons"] == ["old"]
    assert result["source_identity"] == {}
    assert result["ntrip"]["rtcm_base_position_present"] is True
    assert result["ntrip"]["rtcm_msm_present"] is True


def test_recording_snapshot_failure_does_not_consume_row_spools_or_late_state():
    class CountOnly:
        def __len__(self):
            return 17

        def __iter__(self):
            raise AssertionError("evidence must not materialize measurement rows")

    def failed():
        raise OSError("storage read failed")

    panel = SimpleNamespace(
        _measurement_session_hierarchy_enabled=False,
        data_storage_snapshot=failed,
        measurement_run_snapshot=lambda: {"state": "MEASURING"},
        distance_recording_rows=CountOnly(),
        distance_recording_enabled=True,
        _recording_elapsed_s=lambda now: now - 3.0,
        video_recording_paths={"left": "left.mp4"},
    )
    captured = EvidenceAdapter(panel)._capture_recording(10.0)
    panel.distance_recording_enabled = False
    panel.video_recording_paths["left"] = "later.mp4"
    result = captured.project()
    assert result["state"] == "ACTIVE"
    assert result["storage"] == {"state": "UNAVAILABLE", "error": "storage read failed", "streams": {}}
    assert result["measurement_state"] == "MEASURING"
    assert result["sample_counts"]["pairs"] == 17
    assert result["video_paths"] == {"left": "left.mp4"}
    assert result["elapsed_s"] == 7.0


def test_measurement_controls_are_captured_before_pure_projection(monkeypatch):
    settings = SimpleNamespace(
        measurement_method_fallback_order=["first", "second"],
        camera_sync_receive_pair_max_delta_s=0.012,
        camera_sync_max_pc_offset_s=0.023,
        camera_sync_max_clock_rate_error=0.005,
        camera_sync_interval_s=1800.0,
    )
    adapter = EvidenceAdapter(SimpleNamespace(settings=settings))
    captures = []
    project = evidence_module.project_measurement_settings

    def capture(controls):
        captures.append(controls)
        settings.measurement_method_fallback_order.append("later")
        settings.camera_sync_receive_pair_max_delta_s = 1.0
        return project(controls)

    monkeypatch.setattr(evidence_module, "project_measurement_settings", capture)
    result = adapter.measurement_settings()
    assert result["measurement_method_fallback_order"] == ["first", "second"]
    assert result["camera_sync_receive_pair_max_delta_ms"] == 12.0
    assert result["camera_sync_max_pc_offset_ms"] == 23.0
    assert result["camera_sync_clock_rate_error_pct"] == 0.5
    assert result["camera_sync_interval_s"] == 1800.0
    with pytest.raises(TypeError):
        captures[0]["focal_px"] = 10.0


def test_baseline_projection_keeps_raw_ned_separate_from_p1_body_frd():
    baseline = {"north_m": 2.0, "east_m": -7.0, "down_m": 11.0, "baseline_m": 13.19, "fix_quality": "GOOD"}
    body = {"forward_m": -7.0, "right_m": -2.0, "down_m": 11.0, "target_frame": "px4_frd"}
    result = project_baseline(
        baseline=snapshot_mapping(baseline),
        projection=snapshot_mapping(body),
        quality={},
        selected_left={"fix_type": 6},
        selected_right={"fix_type": 5},
        left_source="GPS2",
        right_source="GPS1",
    )
    assert (result["north_m"], result["east_m"], result["down_m"]) == (2.0, -7.0, 11.0)
    assert (result["p1_body_frd_forward_m"], result["p1_body_frd_right_m"], result["p1_body_frd_down_m"]) == (
        -7.0,
        -2.0,
        11.0,
    )
    assert result["left_source"] == "GPS2"
    assert result["right_fix_type"] == 5


def test_geometry_gps_capture_excludes_live_raw_transport_payload():
    raw = {"mutable": [1]}
    source = GpsPositionState(
        "left",
        "left",
        9.0,
        "mavlink/GPS_RAW_INT",
        latitude_deg=35.0,
        longitude_deg=129.0,
        altitude_m=25.0,
        altitude_ellipsoid_m=50.0,
        fix_type=6,
        raw=raw,
    )
    captured = _copy_gps_state(source)
    assert captured is not source
    assert captured.raw is None
    assert captured.altitude_m == 25.0
    assert captured.altitude_ellipsoid_m == 50.0


def test_snapshot_roundtrip_retains_nested_public_container_shapes():
    original = {"list": [{"tuple": (1, 2)}], "matrix": np.array([[1.0, 2.0]])}
    captured = snapshot_mapping(original)
    original["list"][0]["tuple"] = (8, 9)
    original["matrix"][0, 0] = 99.0
    output = thaw_snapshot(captured)
    assert output == {"list": [{"tuple": (1, 2)}], "matrix": [[1.0, 2.0]]}
    with pytest.raises(TypeError):
        captured["matrix"][0][0] = 88.0


def _assert_no_live_dependencies(tree):
    for node in ast.walk(tree):
        if isinstance(node, ast.Attribute):
            assert node.attr not in {"panel", "read_text", "read_bytes", "iterdir", "glob", "monotonic", "time_ns"}
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name):
            assert node.func.id not in {"open", "_widget_text", "_widget_value", "_widget_checked", "_combo_data"}
        if isinstance(node, (ast.Import, ast.ImportFrom)):
            modules = [node.module or ""] if isinstance(node, ast.ImportFrom) else [item.name for item in node.names]
            assert all(
                not name.startswith(("PyQt5", "Workstation.evidence", "Workstation.panel_"))
                or name.startswith("Workstation.evidence_views.")
                for name in modules
            )


def test_projection_leaves_reject_panel_clock_widget_and_file_backreferences():
    folder = Path(__file__).resolve().parents[1] / "evidence_views"
    for name in ("calibration", "connections", "geometry", "gnss", "magnetometers", "recording", "settings"):
        _assert_no_live_dependencies(ast.parse((folder / (name + ".py")).read_text()))
    for bad in ("self.panel.state", "time.monotonic()", "root.read_text()", "open('sample')"):
        with pytest.raises(AssertionError):
            _assert_no_live_dependencies(ast.parse(bad))


def test_remaining_projection_modules_import_without_gui_or_facade():
    apps = Path(__file__).resolve().parents[2]
    script = """
import importlib.abc, sys
sys.path[:0] = [sys.argv[1], sys.argv[2]]
class Guard(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.startswith(('PyQt5', 'Workstation.panel_')) or fullname == 'Workstation.evidence':
            raise AssertionError(fullname)
sys.meta_path.insert(0, Guard())
from Workstation.evidence_views import calibration, connections, geometry, gnss, magnetometers, recording, settings
assert calibration.project_camera_intrinsics(None) == {'state': 'NO DATA'}
"""
    result = subprocess.run(
        [sys.executable, "-I", "-B", "-c", script, str(apps), str(apps.parent / "common/python")],
        check=False,
        timeout=30,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr


def test_malformed_intrinsic_matrix_stays_partial_evidence():
    result = EvidenceAdapter._camera_intrinsics(SimpleNamespace(camera_matrix=42, width=640, height=480))
    assert result["state"] == "AVAILABLE"
    assert result["fx_px"] is None
    assert result["cy_px"] is None
    assert result["width_px"] == 640
