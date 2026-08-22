from __future__ import annotations

from dataclasses import replace
import hashlib
import json

from Workstation.workstation_core.geometry_context import (
    GEOMETRY_CONTEXT_SCHEMA_VERSION,
    geometry_context_record_for_attempt,
    materialize_geometry_contexts,
)
from Workstation.workstation_core.measurement_attempt import (
    AttitudeSnapshot,
    BaselineSnapshot,
    CameraIntrinsicsSnapshot,
    CheckerboardExtrinsicsSnapshot,
    DetectionSnapshot,
    GeometryConfigSnapshot,
    GpsSnapshot,
    MeasurementAttempt,
    StereoIntrinsicsSnapshot,
)
from Workstation.workstation_core.recording_session import save_session_files


def _detection(side: str, timestamp_s: float) -> DetectionSnapshot:
    return DetectionSnapshot(
        camera=side,
        app_receive_monotonic_s=timestamp_s,
        width_px=640,
        height_px=480,
        bbox_xyxy_px=(10.0, 20.0, 30.0, 40.0),
        center_px=(20.0, 30.0),
        confidence=0.9,
        class_id=1,
        class_name="target",
        pairing_timestamp_basis="app_receive_monotonic",
        pairing_timestamp_s=timestamp_s,
    )


def _camera(*, fx: float = 900.0, d0: float = 0.01) -> CameraIntrinsicsSnapshot:
    return CameraIntrinsicsSnapshot(
        width_px=640,
        height_px=480,
        camera_matrix=((fx, 0.0, 320.0), (0.0, 910.0, 240.0), (0.0, 0.0, 1.0)),
        distortion_coefficients=(d0, -0.02, 0.001, 0.002, 0.0),
        reprojection_error_px=0.25,
        image_count=24,
    )


def _extrinsics(*, tx: float = 0.4, yaw_term: float = 0.0) -> CheckerboardExtrinsicsSnapshot:
    if yaw_term == 0.0:
        rotation = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    else:
        # Exact 90-degree RDF-frame rotation avoids an oracle derived by code
        # under test while still producing a valid orthonormal matrix.
        rotation = ((0.0, -1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0))
    return CheckerboardExtrinsicsSnapshot(
        rotation_left_to_right=rotation,
        translation_left_to_right_m=(tx, 0.0, 0.0),
        reprojection_error_px=0.3,
        pair_count=18,
        frame_contract=(
            "opencv_optical:RDF(right,down,forward); "
            "X_right = R_left_to_right * X_left + T_left_to_right_m"
        ),
    )


def _intrinsics(
    *,
    left: CameraIntrinsicsSnapshot | None = None,
    extrinsics: CheckerboardExtrinsicsSnapshot | None = None,
    profile_sha: str = "a" * 64,
) -> StereoIntrinsicsSnapshot:
    return StereoIntrinsicsSnapshot(
        left=left or _camera(),
        right=_camera(fx=905.0, d0=0.011),
        checkerboard_extrinsics=extrinsics or _extrinsics(),
        active_zoom_level=3,
        selected_profile_path="calibration/intrinsics/profile.json",
        content_sha256=profile_sha,
        intrinsics_content_sha256="b" * 64,
        selected_extrinsics_profile_path="calibration/extrinsics/profile.json",
        extrinsics_content_sha256="c" * 64,
        intrinsics_source_zoom_level=2,
        extrinsics_source_zoom_level=2,
        selection_mode="independent_profiles",
    )


def _attempt(*, intrinsics: StereoIntrinsicsSnapshot | None = None) -> MeasurementAttempt:
    return MeasurementAttempt(
        attempt_id=1,
        attempt_session_id="attempt-session",
        detection_pair_key=("checkerboard", 1.0, 1.01),
        created_monotonic_s=1.02,
        created_wall_s=2.0,
        detection_midpoint_monotonic_s=1.005,
        timestamp_basis="app_receive_monotonic",
        triangulation_source="checkerboard",
        left_detection=_detection("left", 1.0),
        right_detection=_detection("right", 1.01),
        left_attitude=None,
        right_attitude=None,
        left_gps=None,
        right_gps=None,
        intrinsics=intrinsics or _intrinsics(),
        baseline=None,
        geometry_config=GeometryConfigSnapshot(
            focal_length_px=900.0,
            max_time_delta_s=0.1,
            max_vertical_delta_px=5.0,
            require_same_class=True,
            timestamp_basis="app_receive_monotonic",
            manual_focal_override=False,
        ),
        left_attitude_age_s=None,
        right_attitude_age_s=None,
        attitude_pair_delta_s=None,
        relative_roll_deg=None,
        relative_pitch_deg=None,
        relative_yaw_deg=None,
        source_quality="GOOD",
        warnings=(),
        measurement_method_id="checkerboard_stereo",
        measurement_method_instance_id="checkerboard:profile-c",
        measurement_component_sources=(("rotation", "checkerboard_profile"),),
    )


def _attitude(side: str, yaw: float) -> AttitudeSnapshot:
    return AttitudeSnapshot(
        camera=side,
        app_receive_monotonic_s=5.0,
        quaternion_wxyz=(1.0, 0.0, 0.0, 0.0),
        namespace=f"/{side}",
        topic="vehicle_attitude",
        px4_timestamp_s=4.9,
        roll_deg=0.0,
        pitch_deg=0.0,
        yaw_deg=yaw,
    )


def _gps(side: str, latitude: float) -> GpsSnapshot:
    return GpsSnapshot(
        camera=side,
        app_receive_monotonic_s=5.0,
        latitude_deg=latitude,
        longitude_deg=127.0,
        altitude_m=100.0,
        fix_type=6,
        satellites_used=20,
        eph_m=0.01,
        epv_m=0.02,
        rtk_label="RTK_FIXED",
        namespace=f"/{side}",
        topic="vehicle_gps_position",
        px4_timestamp_s=4.9,
    )


def _context_id(attempt: MeasurementAttempt) -> str:
    return str(geometry_context_record_for_attempt(attempt)["geometry_context_id"])


def test_static_geometry_changes_control_context_identity() -> None:
    base = _attempt()
    same = _attempt()
    k_changed = replace(
        base,
        intrinsics=replace(base.intrinsics, left=_camera(fx=901.0)),
    )
    d_changed = replace(
        base,
        intrinsics=replace(base.intrinsics, left=_camera(d0=0.012)),
    )
    r_changed = replace(
        base,
        intrinsics=replace(
            base.intrinsics,
            checkerboard_extrinsics=_extrinsics(yaw_term=1.0),
        ),
    )
    t_changed = replace(
        base,
        intrinsics=replace(
            base.intrinsics,
            checkerboard_extrinsics=_extrinsics(tx=0.41),
        ),
    )
    profile_changed = replace(
        base,
        intrinsics=replace(base.intrinsics, content_sha256="d" * 64),
    )

    assert _context_id(base) == _context_id(same)
    assert len(
        {
            _context_id(base),
            _context_id(k_changed),
            _context_id(d_changed),
            _context_id(r_changed),
            _context_id(t_changed),
            _context_id(profile_changed),
        }
    ) == 6


def test_live_attitude_and_gps_do_not_explode_static_contexts() -> None:
    base = _attempt()
    live_changed = replace(
        base,
        left_attitude=_attitude("left", 10.0),
        right_attitude=_attitude("right", 20.0),
        left_gps=_gps("left", 37.0),
        right_gps=_gps("right", 37.0001),
        relative_yaw_deg=10.0,
    )

    assert _context_id(base) == _context_id(live_changed)


def test_live_dynamic_baseline_values_do_not_change_static_context_id() -> None:
    left_attitude = _attitude("left", 0.0)
    right_attitude = _attitude("right", 0.0)
    base = replace(
        _attempt(),
        triangulation_source="pixhawk",
        measurement_method_id="pixhawk_rtk_live",
        measurement_method_instance_id="pixhawk_rtk_live",
        measurement_geometry_sha256="e" * 64,
        measurement_component_sources=(
            ("rotation", "live_pixhawk"),
            ("translation", "live_gnss"),
        ),
        left_attitude=left_attitude,
        right_attitude=right_attitude,
        left_gps=_gps("left", 37.0),
        right_gps=_gps("right", 37.0001),
        baseline=BaselineSnapshot(
            source="gps-rtk-live",
            baseline_m=0.4,
            ned_right_from_left_m=(0.0, 0.4, 0.0),
            translation_right_in_left_rdf_m=(0.4, 0.0, 0.0),
            left_basis_quaternion_wxyz=left_attitude.quaternion_wxyz,
            fix_quality="RTK_FIXED",
            warnings=(),
            usable_for_translation=True,
            unusable_reason="",
            gps_pair_delta_s=0.01,
            left_age_at_capture_s=0.02,
            right_age_at_capture_s=0.03,
            left_age_from_detection_midpoint_s=0.01,
            right_age_from_detection_midpoint_s=0.02,
            left_basis_attitude_monotonic_s=left_attitude.timestamp_s,
        ),
    )
    changed = replace(
        base,
        measurement_geometry_sha256="f" * 64,
        left_gps=_gps("left", 36.5),
        right_gps=_gps("right", 36.5002),
        baseline=replace(
            base.baseline,
            baseline_m=0.8,
            ned_right_from_left_m=(0.1, 0.79, 0.0),
            translation_right_in_left_rdf_m=(0.79, 0.0, 0.1),
        ),
    )

    assert _context_id(base) == _context_id(changed)


def test_context_hash_and_frame_contract_are_independently_checkable() -> None:
    record = geometry_context_record_for_attempt(_attempt())
    encoded = json.dumps(
        record["context"],
        ensure_ascii=False,
        allow_nan=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")

    assert record["geometry_context_sha256"] == hashlib.sha256(encoded).hexdigest()
    assert record["geometry_context_id"] == f"gc:{record['geometry_context_sha256']}"
    context = record["context"]
    assert context["schema_version"] == GEOMETRY_CONTEXT_SCHEMA_VERSION
    assert context["calibration"]["left"]["camera_frame"] == "camera_optical_rdf"
    assert context["transform_contract"]["rotation_notation"] == (
        "R_B_A means v_B = R_B_A @ v_A"
    )
    assert context["public_output_contract"]["frame"] == "baseline_center_frd"
    assert context["public_output_contract"]["unit"] == "m"


def test_terminal_rows_resolve_to_one_deduplicated_static_context() -> None:
    first = _attempt()
    second = replace(
        first,
        attempt_id=2,
        created_monotonic_s=3.0,
        created_wall_s=4.0,
        left_attitude=_attitude("left", 12.0),
        right_attitude=_attitude("right", 13.0),
        left_gps=_gps("left", 36.9),
        right_gps=_gps("right", 36.9001),
    )
    rows = [
        {
            "measurement_attempt_sha256": attempt.canonical_sha256(),
            "measurement_attempt_canonical_json": attempt.canonical_json(),
        }
        for attempt in (first, second)
    ]

    contexts = materialize_geometry_contexts(rows)

    assert len(contexts) == 1
    assert {row["geometry_context_id"] for row in rows} == {
        contexts[0]["geometry_context_id"]
    }


def test_finalized_terminal_jsonl_resolves_to_hashed_context_fragment(
    tmp_path,
) -> None:
    attempt = _attempt()
    output = tmp_path / "recordings" / "geometry-context"

    saved = save_session_files(
        output,
        metadata={"session_id": output.name},
        pair_rows=[
            {
                "measurement_attempt_sha256": attempt.canonical_sha256(),
                "measurement_attempt_canonical_json": attempt.canonical_json(),
            }
        ],
    )
    manifest = json.loads(saved.manifest.read_text(encoding="utf-8"))
    terminal = json.loads(
        (output / manifest["resource_files"]["raw_stereo_measurements"]).read_text(
            encoding="utf-8"
        )
    )
    contexts = [
        json.loads(line)
        for line in (
            output / manifest["resource_files"]["geometry_contexts"]
        ).read_text(encoding="utf-8").splitlines()
    ]

    assert len(contexts) == 1
    assert terminal["geometry_context_id"] == contexts[0]["geometry_context_id"]
    descriptor = manifest["artifact_details"]["geometry_contexts"]
    context_path = output / descriptor["relative_path"]
    assert descriptor["authoritative"] is True
    assert descriptor["row_count"] == 1
    assert descriptor["sha256"] == hashlib.sha256(context_path.read_bytes()).hexdigest()
