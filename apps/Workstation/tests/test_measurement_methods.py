from __future__ import annotations

import time

import pytest
from types import SimpleNamespace

from Workstation.measurement_view_controller import MeasurementViewController
from Workstation.recording_controller import RecordingController
from Workstation.protocol import evaluate_recording_protocol
from Workstation.workstation_core.measurement_attempt import (
    AttitudeSnapshot,
    BlockedMeasurementPair,
    DetectionSnapshot,
)
from Workstation.workstation_core.gnss_operation import (
    GnssOperationConfig,
    ReadinessConfig,
)
from Workstation.workstation_core.rtk_geometry import (
    GpsPositionState,
    RtkBaselineEstimate,
    estimate_rtk_baseline,
    validate_gps_pair_for_baseline,
)
from Workstation.workstation_core.stereo import DetectionObservation

from Workstation.workstation_core.measurement_methods import (
    CHECKERBOARD_STEREO,
    DEFAULT_MEASUREMENT_METHOD_FALLBACK_ORDER,
    DRONE_RTK_MULTIVIEW,
    DRONE_RTK_ROTATION_HYBRID,
    HORIZON_HYBRID,
    MEASUREMENT_METHOD_IDS,
    MOUNTAIN_GNSS,
    MeasurementMethodStatus,
    PIXHAWK_RTK_LIVE,
    choose_ready_measurement_method,
    latest_status_by_method,
    measurement_method_id_for_candidate,
    normalize_measurement_method_fallback_order,
    normalize_measurement_method_fill_policy,
    normalize_measurement_method_id,
    MEASUREMENT_FILL_LIVE_GNSS,
    MEASUREMENT_FILL_LIVE_PIXHAWK,
    MEASUREMENT_FILL_NONE,
)


def test_tolerant_unverified_detection_is_admitted_to_measurement_pipeline() -> None:
    submitted: list[tuple[object, ...]] = []
    left_buffer: list[DetectionObservation] = []
    panel = SimpleNamespace(
        latest_left=None,
        latest_right=None,
        detection_buffers={
            "left": SimpleNamespace(add=left_buffer.append),
            "right": SimpleNamespace(add=lambda _value: None),
        },
    )
    controller = MeasurementViewController(panel)
    controller._ensure_detection_pair_coordinator = lambda: SimpleNamespace(
        submit=lambda *args, **kwargs: submitted.append((*args, kwargs))
    )
    controller._pairing_policy_snapshot = lambda: "policy"
    detection = DetectionObservation(
        camera="left",
        timestamp_s=1.0,
        width=640,
        height=480,
        x1=10.0,
        y1=20.0,
        x2=30.0,
        y2=40.0,
        confidence=0.9,
        class_id=0,
        timestamp_metadata={
            "media_integrity_mode": "tolerant_decoded",
            "media_integrity_unverified": True,
            "measurement_input_admitted": True,
        },
    )

    controller._update_detection("left", detection)

    assert panel.latest_left is detection
    assert left_buffer == [detection]
    assert submitted and submitted[0][0:2] == ("left", detection)
from Workstation.workstation_core.settings import load_settings, save_settings


def _gps_state(
    side: str,
    *,
    timestamp_s: float,
    longitude_deg: float = 127.0,
    fix_type: int = 6,
) -> GpsPositionState:
    return GpsPositionState(
        camera=side,
        namespace=f"{side}-pixhawk",
        timestamp_s=timestamp_s,
        topic="mavlink/GPS_RAW_INT",
        latitude_deg=37.0,
        longitude_deg=longitude_deg,
        altitude_m=100.0,
        fix_type=fix_type,
    )


def _full_candidate(*, profile_hash: str | None):
    dofs = (
        "roll_deg",
        "pitch_deg",
        "yaw_deg",
        "north_m",
        "east_m",
        "down_m",
    )
    mask = SimpleNamespace(**{name: True for name in dofs})
    return SimpleNamespace(
        candidate_id="mountain-1",
        method="gnss-constrained-mountain-uv",
        optimized_dofs=mask,
        observability=tuple((name, True) for name in dofs),
        observability_by_side=(),
        input_hashes=(
            ()
            if profile_hash is None
            else (("operating_intrinsics_profile_sha256", profile_hash),)
        ),
        created_utc_ns=10,
        warnings=(),
    )


def _method_readiness_controller(
    *,
    geometry_allowed: bool,
    profile_hash: str,
    applicability_reasons: tuple[str, ...] = (),
    candidate_profile_hash: str | None = None,
):
    checkerboard = SimpleNamespace(
        profile_id="checkerboard-3x",
        content_sha256="c" * 64,
        source_zoom_level=3,
    )
    loaded = SimpleNamespace(
        profile_id="optical-3x",
        content_sha256="d" * 64,
        source_zoom_level=3,
        method="optical_pose/terrain-pnp-ensemble",
        metadata={"operating_intrinsics_profile_sha256": profile_hash},
    )
    candidate = _full_candidate(profile_hash=candidate_profile_hash)
    owner = SimpleNamespace(
        _candidates={candidate.candidate_id: candidate},
        _intrinsics_snapshot=lambda **_kwargs: {
            "applicability": {
                "geometry_allowed": geometry_allowed,
                "profile_sha256": profile_hash,
                "reasons": applicability_reasons,
            }
        },
    )
    panel = SimpleNamespace(
        active_intrinsics=object(),
        checkerboard_extrinsics_profile=checkerboard,
        optical_pose_extrinsics_profile=loaded,
        _optical_pose_controller=lambda: owner,
    )
    controller = MeasurementViewController(panel)
    controller._measurement_live_readiness = lambda: (True, True, (), ())
    return controller


def _status(method_id: str, *, ready: bool = True, created: int = 1, instance: str = ""):
    return MeasurementMethodStatus(
        method_id=method_id,
        instance_id=instance or f"{method_id}-{created}",
        ready=ready,
        reasons=() if ready else ("missing input",),
        created_utc_ns=created,
    )


def test_legacy_method_ids_and_fallback_order_are_normalized() -> None:
    assert normalize_measurement_method_id("pixhawk") == PIXHAWK_RTK_LIVE
    assert normalize_measurement_method_id("calibration") == CHECKERBOARD_STEREO
    order = normalize_measurement_method_fallback_order(
        ["checkerboard", "checkerboard", "unknown", "mountain_gnss"]
    )
    assert order[:2] == (CHECKERBOARD_STEREO, MOUNTAIN_GNSS)
    assert set(order) == set(DEFAULT_MEASUREMENT_METHOD_FALLBACK_ORDER)
    assert len(order) == len(set(order))


def test_dof_fill_policy_defaults_rotation_to_pixhawk_and_translation_to_gnss() -> None:
    policy = normalize_measurement_method_fill_policy(None)

    for method_id, components in policy.items():
        for side in ("left", "right"):
            assert components[f"{side}.roll_deg"] == MEASUREMENT_FILL_LIVE_PIXHAWK
            assert components[f"{side}.pitch_deg"] == MEASUREMENT_FILL_LIVE_PIXHAWK
            assert components[f"{side}.yaw_deg"] == MEASUREMENT_FILL_LIVE_PIXHAWK
            assert components[f"{side}.north_m"] == MEASUREMENT_FILL_LIVE_GNSS
            assert components[f"{side}.east_m"] == MEASUREMENT_FILL_LIVE_GNSS
            assert components[f"{side}.down_m"] == MEASUREMENT_FILL_LIVE_GNSS


def test_dof_fill_policy_is_independent_per_method_and_invalid_sources_fail_closed() -> None:
    policy = normalize_measurement_method_fill_policy(
        {
            HORIZON_HYBRID: {
                "left.yaw_deg": MEASUREMENT_FILL_NONE,
                "right.north_m": "checkerboard",
            },
            DRONE_RTK_ROTATION_HYBRID: {
                "left.roll_deg": MEASUREMENT_FILL_NONE,
            },
        }
    )

    assert policy[HORIZON_HYBRID]["left.yaw_deg"] == MEASUREMENT_FILL_NONE
    assert policy[HORIZON_HYBRID]["right.north_m"] == MEASUREMENT_FILL_NONE
    assert policy[HORIZON_HYBRID]["left.roll_deg"] == MEASUREMENT_FILL_LIVE_PIXHAWK
    assert (
        policy[DRONE_RTK_ROTATION_HYBRID]["left.roll_deg"]
        == MEASUREMENT_FILL_NONE
    )
    assert policy[MOUNTAIN_GNSS]["left.roll_deg"] == MEASUREMENT_FILL_LIVE_PIXHAWK


def test_candidate_methods_map_to_stable_measurement_categories() -> None:
    assert measurement_method_id_for_candidate("flat-horizon-roll-pitch") == HORIZON_HYBRID
    assert measurement_method_id_for_candidate("optical_pose/gnss-constrained-mountain-uv") == MOUNTAIN_GNSS
    assert measurement_method_id_for_candidate("drone-rtk-multiview-r-t") == DRONE_RTK_MULTIVIEW


def test_preferred_ready_method_wins_and_fallback_uses_configured_order() -> None:
    statuses = (
        _status(PIXHAWK_RTK_LIVE, ready=False),
        _status(CHECKERBOARD_STEREO),
        _status(MOUNTAIN_GNSS),
    )
    assert choose_ready_measurement_method(
        statuses,
        preferred_method_id=MOUNTAIN_GNSS,
        fallback_order=[CHECKERBOARD_STEREO, MOUNTAIN_GNSS],
    ).method_id == MOUNTAIN_GNSS
    assert choose_ready_measurement_method(
        statuses,
        preferred_method_id=PIXHAWK_RTK_LIVE,
        fallback_order=[CHECKERBOARD_STEREO, MOUNTAIN_GNSS],
    ).method_id == CHECKERBOARD_STEREO


def test_latest_candidate_instance_is_used_per_method() -> None:
    latest = latest_status_by_method(
        (
            _status(MOUNTAIN_GNSS, created=1, instance="old"),
            _status(MOUNTAIN_GNSS, created=2, instance="new"),
        )
    )
    assert latest[MOUNTAIN_GNSS].instance_id == "new"

    fallback = latest_status_by_method(
        (
            _status(MOUNTAIN_GNSS, ready=True, created=1, instance="older-ready"),
            _status(MOUNTAIN_GNSS, ready=False, created=2, instance="newer-blocked"),
        )
    )
    assert fallback[MOUNTAIN_GNSS].instance_id == "older-ready"


def test_no_ready_method_returns_none_and_ready_rows_cannot_have_reasons() -> None:
    assert choose_ready_measurement_method(
        (_status(PIXHAWK_RTK_LIVE, ready=False),),
        preferred_method_id=PIXHAWK_RTK_LIVE,
        fallback_order=DEFAULT_MEASUREMENT_METHOD_FALLBACK_ORDER,
    ) is None
    with pytest.raises(ValueError, match="blocking reasons"):
        MeasurementMethodStatus(
            method_id=PIXHAWK_RTK_LIVE,
            instance_id="bad",
            ready=True,
            reasons=("contradiction",),
        )


def test_settings_round_trip_and_legacy_preference_migration(tmp_path) -> None:
    path = tmp_path / "settings.json"
    path.write_text(
        '{"triangulation_extrinsics_source":"checkerboard",'
        '"measurement_method_fallback_order":["mountain_gnss","pixhawk"]}'
    )
    settings = load_settings(path)
    assert settings.measurement_preferred_method == CHECKERBOARD_STEREO
    assert settings.measurement_method_fallback_order[:2] == (
        MOUNTAIN_GNSS,
        PIXHAWK_RTK_LIVE,
    )

    save_settings(settings, path)
    reloaded = load_settings(path)
    assert reloaded.measurement_method_fallback_order == settings.measurement_method_fallback_order


def test_runtime_fallback_and_stable_auto_recovery(monkeypatch) -> None:
    clock = {"now": 10.0}
    monkeypatch.setattr(
        "Workstation.measurement_view_controller.time.monotonic",
        lambda: clock["now"],
    )
    panel = SimpleNamespace(
        settings=SimpleNamespace(
            measurement_preferred_method=PIXHAWK_RTK_LIVE,
            measurement_method_fallback_order=(CHECKERBOARD_STEREO, MOUNTAIN_GNSS),
        ),
        _workstation_setting_overrides={},
        _workstation_setting=lambda _name, default=None: default,
        _last_measurement_attempt_key=None,
        _last_measurement_attempt=None,
    )
    controller = MeasurementViewController(panel)
    rows = [
        _status(PIXHAWK_RTK_LIVE, ready=False),
        _status(CHECKERBOARD_STEREO, ready=True),
    ]
    controller._measurement_method_statuses = lambda: tuple(rows)

    selected = controller._select_measurement_method_for_attempt()
    assert selected.method_id == CHECKERBOARD_STEREO
    assert panel._measurement_selection_reason == "auto_fallback"

    rows[0] = _status(PIXHAWK_RTK_LIVE, ready=True)
    assert controller._select_measurement_method_for_attempt().method_id == CHECKERBOARD_STEREO
    clock["now"] += 0.5
    rows[0] = _status(PIXHAWK_RTK_LIVE, ready=False)
    assert controller._select_measurement_method_for_attempt().method_id == CHECKERBOARD_STEREO
    clock["now"] += 1.0
    rows[0] = _status(PIXHAWK_RTK_LIVE, ready=True)
    assert controller._select_measurement_method_for_attempt().method_id == CHECKERBOARD_STEREO
    clock["now"] += 1.1
    assert controller._select_measurement_method_for_attempt().method_id == PIXHAWK_RTK_LIVE
    assert panel._measurement_selection_reason == "auto_recovery"


def test_session_method_catalog_lists_missing_candidate_methods_as_blocked() -> None:
    panel = SimpleNamespace(
        settings=SimpleNamespace(
            measurement_preferred_method=PIXHAWK_RTK_LIVE,
            measurement_method_fallback_order=DEFAULT_MEASUREMENT_METHOD_FALLBACK_ORDER,
        ),
        _workstation_setting_overrides={},
        _workstation_setting=lambda _name, default=None: default,
        _last_measurement_attempt_key=None,
        _last_measurement_attempt=None,
    )
    controller = MeasurementViewController(panel)
    controller._measurement_method_statuses = lambda: (
        _status(PIXHAWK_RTK_LIVE, ready=True, instance="live:pixhawk-rtk"),
    )
    controller._measurement_component_controls = lambda _status: {}
    controller._measurement_method_input_snapshot = lambda _status: {
        "uses_live_pixhawk": True,
        "uses_live_gnss": True,
    }

    snapshot = controller.workstation_measurement_methods_snapshot()

    catalog = snapshot["method_catalog"]
    assert tuple(row["method_id"] for row in catalog) == MEASUREMENT_METHOD_IDS
    assert tuple(row["method_id"] for row in snapshot["method_capabilities"]) == (
        PIXHAWK_RTK_LIVE,
    )
    mountain = next(row for row in catalog if row["method_id"] == MOUNTAIN_GNSS)
    drone = next(
        row
        for row in catalog
        if row["method_id"] == DRONE_RTK_MULTIVIEW
    )
    assert mountain["ready"] is False
    assert mountain["reasons"] == [
        "no Mountain + GNSS candidate or compatible loaded profile"
    ]
    assert mountain["input_values"] == {}
    assert "Mountain + GNSS full 6-DOF candidate" in mountain["required_inputs"]
    assert drone["ready"] is False
    assert "Drone RTK multi-view full 6-DOF candidate" in drone["required_inputs"]


def test_geometry_hash_changes_with_the_frozen_live_attitude() -> None:
    status = MeasurementMethodStatus(
        method_id=HORIZON_HYBRID,
        instance_id="candidate:horizon-1",
        ready=True,
        component_sources=(("left.yaw_deg", "live_pixhawk"),),
    )

    def attitude(yaw: float) -> AttitudeSnapshot:
        return AttitudeSnapshot(
            camera="left",
            app_receive_monotonic_s=10.0,
            quaternion_wxyz=(1.0, 0.0, 0.0, 0.0),
            namespace="left-pixhawk",
            topic="mavlink/ATTITUDE_QUATERNION",
            px4_timestamp_s=None,
            roll_deg=1.0,
            pitch_deg=2.0,
            yaw_deg=yaw,
        )

    first = MeasurementViewController._measurement_geometry_hash(
        status,
        geometry_kind="relative_pose",
        intrinsics=None,
        left_attitude=attitude(3.0),
        right_attitude=None,
        left_gps=None,
        right_gps=None,
    )
    second = MeasurementViewController._measurement_geometry_hash(
        status,
        geometry_kind="relative_pose",
        intrinsics=None,
        left_attitude=attitude(4.0),
        right_attitude=None,
        left_gps=None,
        right_gps=None,
    )
    assert first != second


def test_method_input_snapshot_keeps_live_attitude_and_gnss_baseline_contracts_separate(
    monkeypatch,
) -> None:
    monkeypatch.setattr(
        "Workstation.measurement_view_controller.time.monotonic",
        lambda: 20.0,
    )
    panel = SimpleNamespace(
        latest_extrinsics={
            "left": SimpleNamespace(
                timestamp_s=19.8,
                roll_deg=1.0,
                pitch_deg=2.0,
                yaw_deg=3.0,
                namespace="p1",
                topic="ATTITUDE_QUATERNION",
            ),
            "right": SimpleNamespace(
                timestamp_s=19.7,
                roll_deg=4.0,
                pitch_deg=5.0,
                yaw_deg=6.0,
                namespace="p2",
                topic="ATTITUDE_QUATERNION",
            ),
        },
        latest_rtk_baseline=RtkBaselineEstimate(
            baseline_m=1.25,
            north_m=0.1,
            east_m=1.2,
            down_m=-0.2,
            left_age_s=0.08,
            right_age_s=0.09,
            fix_quality="GOOD",
        ),
    )
    status = MeasurementMethodStatus(
        method_id=PIXHAWK_RTK_LIVE,
        instance_id="live:pixhawk-rtk",
        ready=True,
        component_sources=(
            ("left.roll_deg", MEASUREMENT_FILL_LIVE_PIXHAWK),
            ("right.roll_deg", MEASUREMENT_FILL_LIVE_PIXHAWK),
            ("right.north_m", MEASUREMENT_FILL_LIVE_GNSS),
        ),
    )

    values = MeasurementViewController(panel)._measurement_method_input_snapshot(status)

    assert values["live_attitude_rpy_deg"]["left"] == {
        "roll_deg": 1.0,
        "pitch_deg": 2.0,
        "yaw_deg": 3.0,
        "age_s": pytest.approx(0.2),
        "source": ["p1", "ATTITUDE_QUATERNION"],
    }
    assert values["live_baseline_ned_m"]["north_m"] == pytest.approx(0.1)
    assert values["live_baseline_ned_m"]["east_m"] == pytest.approx(1.2)
    assert values["live_baseline_ned_m"]["down_m"] == pytest.approx(-0.2)
    assert values["candidate_pose_virtual_frd_to_world_ned"] == {}
    assert values["stereo_extrinsics_opencv_rdf"] is None


def test_resolution_mismatch_blocks_every_measurement_geometry_source() -> None:
    profile_hash = "a" * 64
    controller = _method_readiness_controller(
        geometry_allowed=False,
        profile_hash=profile_hash,
        applicability_reasons=("left_frame_resolution_mismatch",),
        candidate_profile_hash=profile_hash,
    )

    statuses = controller._measurement_method_statuses()
    expected_instances = {
        "live:pixhawk-rtk",
        "profile:checkerboard-3x",
        "candidate:mountain-1",
        "profile:optical-3x",
    }
    selected = [row for row in statuses if row.instance_id in expected_instances]

    assert {row.instance_id for row in selected} == expected_instances
    assert all(row.ready is False for row in selected)
    assert all("exact operating K,D not applicable" in row.reasons for row in selected)
    assert all(
        "K,D applicability: left_frame_resolution_mismatch" in row.reasons
        for row in selected
    )


def test_nominal_unverified_profile_can_satisfy_geometry_readiness() -> None:
    profile_hash = "f" * 64
    owner = SimpleNamespace(
        _intrinsics_snapshot=lambda **_kwargs: {
            "profile": {
                "provenance": {
                    "quality": "NOMINAL_UNVERIFIED",
                    "precision_qualified": False,
                }
            },
            "applicability": {
                "geometry_allowed": True,
                "profile_sha256": profile_hash,
                "reasons": (),
            },
        }
    )
    controller = MeasurementViewController(SimpleNamespace(active_intrinsics=None))

    ready, digest, reasons = controller._operating_intrinsics_readiness(owner)

    assert ready is True
    assert digest == profile_hash
    assert reasons == ()


def test_field_live_readiness_accepts_same_measurement_stale_samples(monkeypatch) -> None:
    monkeypatch.setattr("Workstation.measurement_view_controller.time.monotonic", lambda: 100.0)
    left_gps = _gps_state("left", timestamp_s=91.0)
    right_gps = _gps_state("right", timestamp_s=91.1, longitude_deg=127.00001)
    baseline = SimpleNamespace(
        baseline_m=1.0,
        warnings=("left NO_FIX",),
    )
    panel = SimpleNamespace(
        _measurement_started_monotonic_s=90.0,
        latest_extrinsics={
            "left": SimpleNamespace(timestamp_s=91.0, q=(1.0, 0.0, 0.0, 0.0)),
            "right": SimpleNamespace(timestamp_s=91.1, q=(1.0, 0.0, 0.0, 0.0)),
        },
        latest_rtk_baseline=baseline,
        _selected_baseline_gps_states=lambda: (left_gps, right_gps),
        _gps_baseline_usable_for_translation=lambda _baseline: (False, "fix quality low"),
        gnss_operation_gate_active=False,
    )

    pixhawk_ready, gnss_ready, hard, advisories = MeasurementViewController(
        panel
    )._measurement_live_readiness()

    assert pixhawk_ready is True
    assert gnss_ready is True
    assert hard == ()
    assert "left Pixhawk attitude stale (9.000s)" in advisories
    assert "fix quality low" in advisories


def test_field_live_readiness_reuses_valid_inputs_from_before_measurement(monkeypatch) -> None:
    monkeypatch.setattr("Workstation.measurement_view_controller.time.monotonic", lambda: 100.0)
    left_gps = _gps_state("left", timestamp_s=80.0)
    right_gps = _gps_state("right", timestamp_s=80.1, longitude_deg=127.00001)
    panel = SimpleNamespace(
        _measurement_started_monotonic_s=90.0,
        latest_extrinsics={
            "left": SimpleNamespace(timestamp_s=80.0, q=(1.0, 0.0, 0.0, 0.0)),
            "right": SimpleNamespace(timestamp_s=80.1, q=(1.0, 0.0, 0.0, 0.0)),
        },
        latest_rtk_baseline=SimpleNamespace(baseline_m=1.0, warnings=()),
        _selected_baseline_gps_states=lambda: (left_gps, right_gps),
        _gps_baseline_usable_for_translation=lambda _baseline: (True, "usable"),
        gnss_operation_gate_active=False,
    )

    pixhawk_ready, gnss_ready, hard, advisories = MeasurementViewController(
        panel
    )._measurement_live_readiness()

    assert pixhawk_ready is True
    assert gnss_ready is True
    assert hard == ()
    assert any("latest valid 3D fix" in warning for warning in advisories)
    assert any(
        "Pixhawk attitude reused from latest valid sample" in warning
        for warning in advisories
    )


def test_measurement_gps_cache_updates_sides_independently_and_ignores_sub_3d_fix() -> None:
    current = {
        "left": _gps_state("left", timestamp_s=10.0),
        "right": _gps_state(
            "right", timestamp_s=10.0, longitude_deg=127.00001
        ),
    }
    panel = SimpleNamespace(
        _selected_baseline_gps_states=lambda: (current["left"], current["right"]),
    )
    controller = MeasurementViewController(panel)

    first_left, first_right, first_reused = controller._cached_measurement_gps_states()
    current["left"] = _gps_state("left", timestamp_s=11.0, longitude_deg=127.00002)
    current["right"] = _gps_state(
        "right",
        timestamp_s=12.0,
        longitude_deg=127.00003,
        fix_type=2,
    )
    second_left, second_right, second_reused = controller._cached_measurement_gps_states()

    assert first_reused == ()
    assert second_left is current["left"]
    assert second_right is first_right
    assert second_left is not first_left
    assert second_reused == ("right_gps",)


def test_measurement_gnss_contract_uses_selected_gps2_device_identity() -> None:
    from dataclasses import replace

    selected = {
        "left": _gps_state("left", timestamp_s=10.0),
        "right": _gps_state(
            "right", timestamp_s=10.0, longitude_deg=127.00001
        ),
    }
    selected["left"] = replace(selected["left"], topic="mavlink/GPS2_RAW")
    captured = []
    manager = SimpleNamespace(
        config=SimpleNamespace(readiness_gate_enabled=True),
        capture_measurement_contract=lambda readiness, **kwargs: captured.append(kwargs) or kwargs,
    )
    panel = SimpleNamespace(
        gnss_operation_manager=manager,
        gnss_fixed_base_profile=None,
        latest_baseline_vector=None,
        _selected_rover_gps_state=lambda side: selected.get(side),
        _gnss_base_rtcm_target_sides=lambda: ("left", "right"),
        latest_gps={},
    )

    result = MeasurementViewController(panel)._capture_measurement_gnss_contract(
        SimpleNamespace()
    )

    assert result["device_ids"] == {
        "left": "mavlink/GPS2_RAW",
        "right": "mavlink/GPS_RAW_INT",
    }


def test_profile_sha_evidence_is_required_for_candidate_geometry() -> None:
    controller = _method_readiness_controller(
        geometry_allowed=True,
        profile_hash="b" * 64,
        candidate_profile_hash=None,
    )

    status = next(
        row
        for row in controller._measurement_method_statuses()
        if row.instance_id == "candidate:mountain-1"
    )

    assert status.ready is False
    assert "optical pose candidate not applied" in status.reasons
    assert "candidate operating K,D profile SHA-256 missing" in status.reasons


def test_unapplied_candidate_with_stale_kd_cannot_be_measurement_fallback() -> None:
    controller = _method_readiness_controller(
        geometry_allowed=True,
        profile_hash="b" * 64,
        candidate_profile_hash="a" * 64,
    )

    status = next(
        row
        for row in controller._measurement_method_statuses()
        if row.instance_id == "candidate:mountain-1"
    )

    assert status.ready is False
    assert "optical pose candidate not applied" in status.reasons
    assert "candidate K,D differs from active K,D" in status.reasons
    assert "candidate K,D differs from active K,D" not in status.warnings


def test_applied_candidate_becomes_unready_if_active_kd_changes() -> None:
    controller = _method_readiness_controller(
        geometry_allowed=True,
        profile_hash="b" * 64,
        candidate_profile_hash="a" * 64,
    )
    owner = controller.panel._optical_pose_controller()
    owner._active_selection = SimpleNamespace(
        candidate_id="mountain-1",
        selected_dofs=SimpleNamespace(
            selected_names=(
                "roll_deg",
                "pitch_deg",
                "yaw_deg",
                "north_m",
                "east_m",
                "down_m",
            )
        ),
    )

    status = next(
        row
        for row in controller._measurement_method_statuses()
        if row.instance_id == "candidate:mountain-1"
    )

    assert status.ready is False
    assert "optical pose candidate not applied" not in status.reasons
    assert "candidate K,D differs from active K,D" in status.reasons


def test_unapplied_checkerboard_profile_is_not_measurement_ready() -> None:
    controller = _method_readiness_controller(
        geometry_allowed=True,
        profile_hash="b" * 64,
        candidate_profile_hash="b" * 64,
    )
    panel = controller.panel
    panel.checkerboard_extrinsics_candidate_profile = panel.checkerboard_extrinsics_profile
    panel.checkerboard_extrinsics_profile = None

    status = next(
        row
        for row in controller._measurement_method_statuses()
        if row.method_id == CHECKERBOARD_STEREO
    )

    assert status.ready is False
    assert status.reasons == ("checkerboard stereo R,t candidate not applied",)


def test_valid_exact_kd_keeps_only_applied_geometry_profiles_ready() -> None:
    profile_hash = "e" * 64
    controller = _method_readiness_controller(
        geometry_allowed=True,
        profile_hash=profile_hash,
        candidate_profile_hash=profile_hash,
    )

    statuses = controller._measurement_method_statuses()
    expected_instances = {
        "live:pixhawk-rtk",
        "profile:checkerboard-3x",
        "candidate:mountain-1",
        "profile:optical-3x",
    }
    selected = [row for row in statuses if row.instance_id in expected_instances]

    assert {row.instance_id for row in selected} == expected_instances
    readiness = {row.instance_id: row for row in selected}
    assert readiness["live:pixhawk-rtk"].ready is True
    assert readiness["profile:checkerboard-3x"].ready is True
    assert readiness["profile:optical-3x"].ready is True
    assert readiness["candidate:mountain-1"].ready is False
    assert readiness["candidate:mountain-1"].reasons == (
        "optical pose candidate not applied",
    )
    assert next(
        row for row in selected if row.instance_id == "profile:checkerboard-3x"
    ).geometry_sha256 != "c" * 64


def test_stale_gps_pair_is_retained_but_cannot_feed_baseline_geometry() -> None:
    left = _gps_state("left", timestamp_s=100.0)
    right = _gps_state("right", timestamp_s=100.01, longitude_deg=127.00001)

    baseline = estimate_rtk_baseline(left, right, now_s=200.0, require_fixed=False)
    readiness = validate_gps_pair_for_baseline(
        left,
        right,
        now_s=200.0,
        maximum_age_s=1.0,
        maximum_pair_delta_s=0.05,
    )

    assert baseline is not None
    assert baseline.baseline_m > 0.0
    assert baseline.gps_pair_delta_s == pytest.approx(0.01)
    assert readiness.usable is False
    assert readiness.left_age_s == pytest.approx(100.0)
    assert readiness.right_age_s == pytest.approx(99.99)
    assert sum("GPS sample stale" in reason for reason in readiness.reasons) == 2


def test_one_sided_fresh_gps_pair_is_not_usable() -> None:
    readiness = validate_gps_pair_for_baseline(
        _gps_state("left", timestamp_s=99.9),
        None,
        now_s=100.0,
        maximum_age_s=1.0,
        maximum_pair_delta_s=0.05,
    )

    assert readiness.usable is False
    assert readiness.reasons == ("right GPS sample missing",)
    assert readiness.left_age_s == pytest.approx(0.1)
    assert readiness.right_age_s is None


@pytest.mark.parametrize(
    ("right", "expected_reason"),
    (
        (None, "right GPS sample missing"),
        (_gps_state("right", timestamp_s=0.01, longitude_deg=127.00001), "GPS sample stale"),
    ),
)
def test_measurement_controller_rejects_one_sided_or_stale_gps_pair(
    monkeypatch,
    right,
    expected_reason,
) -> None:
    monkeypatch.setattr(
        "Workstation.measurement_view_controller.time.monotonic",
        lambda: 100.0,
    )
    left = _gps_state("left", timestamp_s=99.9)
    baseline = RtkBaselineEstimate(
        baseline_m=1.0,
        north_m=0.0,
        east_m=1.0,
        down_m=0.0,
        left_age_s=0.1,
        right_age_s=0.1,
        fix_quality="GOOD",
    )
    panel = SimpleNamespace(
        _selected_baseline_gps_states=lambda: (left, right),
    )

    usable, reason = MeasurementViewController(panel)._gps_baseline_usable_for_translation(
        baseline
    )

    assert usable is False
    assert expected_reason in reason


def test_skewed_gps_pair_is_not_usable_even_when_both_samples_are_fresh() -> None:
    readiness = validate_gps_pair_for_baseline(
        _gps_state("left", timestamp_s=99.9),
        _gps_state("right", timestamp_s=99.7, longitude_deg=127.00001),
        now_s=100.0,
        maximum_age_s=1.0,
        maximum_pair_delta_s=0.05,
    )

    assert readiness.usable is False
    assert readiness.pair_delta_s == pytest.approx(0.2)
    assert any("GPS pair timestamp skew" in reason for reason in readiness.reasons)


def test_fresh_gps_pair_uses_configured_age_and_skew_limits(monkeypatch) -> None:
    monkeypatch.setattr(
        "Workstation.measurement_view_controller.time.monotonic",
        lambda: 100.0,
    )
    left = _gps_state("left", timestamp_s=99.8)
    right = _gps_state("right", timestamp_s=99.79, longitude_deg=127.00001)
    baseline = estimate_rtk_baseline(left, right, now_s=100.0, require_fixed=False)
    config = GnssOperationConfig(
        readiness=ReadinessConfig(
            maximum_gps_sample_age_s=0.25,
            maximum_left_right_timestamp_delta_ms=20.0,
        )
    )
    panel = SimpleNamespace(
        gnss_operation_manager=SimpleNamespace(config=config),
        _selected_baseline_gps_states=lambda: (left, right),
    )

    usable, reason = MeasurementViewController(panel)._gps_baseline_usable_for_translation(
        baseline
    )

    assert usable is True
    assert reason == "usable"


def test_no_ready_method_pair_is_retained_as_blocked_recording_evidence() -> None:
    def detection(side: str) -> DetectionSnapshot:
        observation = DetectionObservation(
            camera=side,
            timestamp_s=10.0 if side == "left" else 10.01,
            width=640,
            height=480,
            x1=100.0,
            y1=120.0,
            x2=180.0,
            y2=220.0,
            confidence=0.9,
            class_id=1,
        )
        return DetectionSnapshot.from_observation(
            observation,
            pairing_timestamp_basis="detection",
            pairing_timestamp_s=observation.timestamp_s,
        )

    panel = SimpleNamespace(
        distance_recording_enabled=True,
        recording_session_id="session-1",
        _recording_pair_row_index={},
        distance_recording_rows=[],
        distance_recording_keys=[],
        _append_recording_table_row=lambda _row: None,
        _refresh_recording_table=lambda: None,
        _refresh_recording_status_label=lambda: None,
        _refresh_operation_header=lambda: None,
    )
    blocked = BlockedMeasurementPair(
        detection_pair_key=("no-ready", 10.0, 10.01),
        created_monotonic_s=11.0,
        created_wall_s=20.0,
        timestamp_basis="detection",
        left_detection=detection("left"),
        right_detection=detection("right"),
        status="matched_no_ready_measurement_method",
        reasons=("no ready measurement method",),
        readiness_sha256="a" * 64,
    )

    RecordingController(panel)._record_blocked_measurement_pair(blocked)

    assert len(panel.distance_recording_rows) == 1
    row = panel.distance_recording_rows[0]
    assert row["measurement_valid"] == "false"
    assert row["pair_status"] == "matched_no_ready_measurement_method"
    assert row["measurement_geometry_kind"] == "unresolved"
    assert row["pairing_time_basis"] == "app_receive_monotonic"
    assert row["selected_pair_delta_ns"] == 10_000_000
    assert row["sync_delta_ns"] == 10_000_000
    assert row["ntp_evidence_role"] == "record_only"


def _blocked_pair_fixture(pair_key: tuple[str, float, float]) -> BlockedMeasurementPair:
    def detection(side: str) -> DetectionSnapshot:
        observation = DetectionObservation(
            camera=side,
            timestamp_s=10.0 if side == "left" else 10.01,
            width=640,
            height=480,
            x1=100.0,
            y1=120.0,
            x2=180.0,
            y2=220.0,
            confidence=0.9,
            class_id=1,
        )
        return DetectionSnapshot.from_observation(
            observation,
            pairing_timestamp_basis="detection",
            pairing_timestamp_s=observation.timestamp_s,
        )

    return BlockedMeasurementPair(
        detection_pair_key=pair_key,
        created_monotonic_s=11.0,
        created_wall_s=20.0,
        timestamp_basis="detection",
        left_detection=detection("left"),
        right_detection=detection("right"),
        status="matched_no_ready_measurement_method",
        reasons=("no ready measurement method",),
        readiness_sha256="a" * 64,
    )


def _blocked_pair_panel(rows: object, writes: list[dict[str, object]]) -> SimpleNamespace:
    def append_workstation_data(stream: str, payload: dict, *, priority: bool = False) -> bool:
        writes.append({"stream": stream, "payload": dict(payload), "priority": priority})
        return True

    return SimpleNamespace(
        distance_recording_enabled=True,
        recording_session_id="session-1",
        _recording_pair_row_index={},
        distance_recording_rows=rows,
        distance_recording_keys=[],
        _append_recording_table_row=lambda _row: None,
        _refresh_recording_table=lambda: None,
        _refresh_recording_status_label=lambda: None,
        _refresh_operation_header=lambda: None,
        append_workstation_data=append_workstation_data,
    )


def test_blocked_pair_append_on_unavailable_owner_goes_direct_and_stays_untracked() -> None:
    from Workstation.workstation_core.recording_session import SpoolRows

    rows = SpoolRows(supports_updates=True)  # unbound: append is UNAVAILABLE
    writes: list[dict[str, object]] = []
    panel = _blocked_pair_panel(rows, writes)
    controller = RecordingController(panel)

    controller._record_blocked_measurement_pair(_blocked_pair_fixture(("no-owner", 10.0, 10.01)))

    assert len(rows) == 0
    assert panel._recording_pair_row_index == {}
    assert controller._pair_recording_row_leases == {}
    assert controller._native_row_owners == {}
    assert len(writes) == 1
    assert writes[0]["stream"] == "measurements"
    assert writes[0]["priority"] is True
    payload = writes[0]["payload"]
    assert payload["row_persistence_mode"] == "direct_owner_unavailable"
    assert payload["measurement_valid"] == "false"
    assert payload["pair_status"] == "matched_no_ready_measurement_method"
    assert payload["row_owner_unavailable_reason"]


def test_blocked_pair_replace_on_unavailable_owner_goes_direct_and_drops_tracking(tmp_path) -> None:
    from Workstation.workstation_core.recording_session import SpoolRows

    rows = SpoolRows(supports_updates=True)
    rows.bind(tmp_path / "blocked.jsonl")
    writes: list[dict[str, object]] = []
    panel = _blocked_pair_panel(rows, writes)
    controller = RecordingController(panel)
    key = ("replace-later", 10.0, 10.01)

    controller._record_blocked_measurement_pair(_blocked_pair_fixture(key))
    assert len(rows) == 1
    assert panel._recording_pair_row_index == {key: 0}
    assert writes == []

    assert rows.close() is True
    controller._record_blocked_measurement_pair(_blocked_pair_fixture(key))

    assert len(rows) == 1
    assert key not in panel._recording_pair_row_index
    assert key not in controller._pair_recording_row_leases
    assert len(writes) == 1
    assert writes[0]["stream"] == "measurements"
    assert writes[0]["payload"]["row_persistence_mode"] == "direct_owner_unavailable"
    assert writes[0]["payload"]["sample_index"] == 0


def test_preflight_explicitly_blocks_when_method_snapshot_has_no_ready_source() -> None:
    snapshot = {
        "camera.left": SimpleNamespace(state="LIVE"),
        "camera.right": SimpleNamespace(state="LIVE"),
        "synchronization": SimpleNamespace(state="SYNCED"),
        "detection": SimpleNamespace(state="RUNNING"),
    }
    readiness = evaluate_recording_protocol(
        snapshot,
        distance_mode="checkerboard",
        calibration={
            "active": True,
            "left": {"state": "AVAILABLE"},
            "right": {"state": "AVAILABLE"},
            "extrinsics": {"state": "AVAILABLE"},
        },
        baseline={"state": "GOOD", "length_m": 1.0},
        require_rtk_fixed=False,
        measurement_method={
            "method_id": "",
            "instance_id": "",
            "label": "Ready measurement method",
            "ready": False,
            "reasons": ["no ready measurement method"],
            "requires_live_pixhawk": False,
            "requires_live_gnss": False,
        },
    )
    blocking = {check.key for check in readiness.blocking}
    assert "geometry" in blocking
    assert "pixhawk" not in {check.key for check in readiness.checks}
    assert "baseline" not in {check.key for check in readiness.checks}


@pytest.mark.parametrize(
    "missing_key,in_best",
    (("timestamp_s", False), ("width", False), ("x1", True), ("class_id", True)),
)
def test_detection_observation_malformed_payload_returns_none(
    missing_key: str,
    in_best: bool,
) -> None:
    payload = {
        "timestamp_s": 10.0,
        "width": 640,
        "height": 480,
        "best_detection": {
            "x1": 1.0,
            "y1": 2.0,
            "x2": 3.0,
            "y2": 4.0,
            "confidence": 0.9,
            "class_id": 1,
        },
    }
    target = payload["best_detection"] if in_best else payload
    target.pop(missing_key)

    assert DetectionObservation.from_payload("left", payload) is None


def test_zero_quaternion_returns_invalid_triangulation_attempt() -> None:
    from Workstation.workstation_core.calibration import CameraIntrinsics
    from Workstation.workstation_core.triangulation import (
        triangulate_pixels_with_pixhawk_mount_attempt,
    )

    intrinsics = CameraIntrinsics(
        width=640,
        height=480,
        camera_matrix=((900.0, 0.0, 320.0), (0.0, 900.0, 240.0), (0.0, 0.0, 1.0)),
        distortion_coefficients=(0.0, 0.0, 0.0, 0.0, 0.0),
        reprojection_error_px=0.1,
        image_count=10,
    )

    attempt = triangulate_pixels_with_pixhawk_mount_attempt(
        left_pixel=(340.0, 240.0),
        right_pixel=(300.0, 240.0),
        left_intrinsics=intrinsics,
        right_intrinsics=intrinsics,
        left_body_to_world_q=(0.0, 0.0, 0.0, 0.0),
        right_body_to_world_q=(1.0, 0.0, 0.0, 0.0),
        translation_right_in_left_m=(0.4, 0.0, 0.0),
    )

    assert attempt.result is None
    assert attempt.reason == "invalid_quaternion"


def test_missing_translation_with_valid_quaternions_reports_missing_rtk_translation() -> None:
    from Workstation.workstation_core.calibration import CameraIntrinsics
    from Workstation.workstation_core.triangulation import (
        triangulate_pixels_with_pixhawk_mount_attempt,
    )

    intrinsics = CameraIntrinsics(
        width=640,
        height=480,
        camera_matrix=((900.0, 0.0, 320.0), (0.0, 900.0, 240.0), (0.0, 0.0, 1.0)),
        distortion_coefficients=(0.0, 0.0, 0.0, 0.0, 0.0),
        reprojection_error_px=0.1,
        image_count=10,
    )

    attempt = triangulate_pixels_with_pixhawk_mount_attempt(
        left_pixel=(340.0, 240.0),
        right_pixel=(300.0, 240.0),
        left_intrinsics=intrinsics,
        right_intrinsics=intrinsics,
        left_body_to_world_q=(1.0, 0.0, 0.0, 0.0),
        right_body_to_world_q=(1.0, 0.0, 0.0, 0.0),
        translation_right_in_left_m=None,
    )

    assert attempt.result is None
    assert attempt.reason == "missing_rtk_translation"


def test_missing_translation_takes_precedence_over_quaternion_validation() -> None:
    from Workstation.workstation_core.calibration import CameraIntrinsics
    from Workstation.workstation_core.triangulation import (
        triangulate_pixels_with_pixhawk_mount_attempt,
    )

    intrinsics = CameraIntrinsics(
        width=640,
        height=480,
        camera_matrix=((900.0, 0.0, 320.0), (0.0, 900.0, 240.0), (0.0, 0.0, 1.0)),
        distortion_coefficients=(0.0, 0.0, 0.0, 0.0, 0.0),
        reprojection_error_px=0.1,
        image_count=10,
    )

    attempt = triangulate_pixels_with_pixhawk_mount_attempt(
        left_pixel=(340.0, 240.0),
        right_pixel=(300.0, 240.0),
        left_intrinsics=intrinsics,
        right_intrinsics=intrinsics,
        left_body_to_world_q=(0.0, 0.0, 0.0, 0.0),
        right_body_to_world_q=(0.0, 0.0, 0.0, 0.0),
        translation_right_in_left_m=None,
    )

    assert attempt.reason == "missing_rtk_translation"


def test_pixhawk_triangulation_normalizes_each_attitude_once(monkeypatch) -> None:
    from Workstation.workstation_core import triangulation
    from Workstation.workstation_core.calibration import CameraIntrinsics

    intrinsics = CameraIntrinsics(
        width=640,
        height=480,
        camera_matrix=((900.0, 0.0, 320.0), (0.0, 900.0, 240.0), (0.0, 0.0, 1.0)),
        distortion_coefficients=(0.0, 0.0, 0.0, 0.0, 0.0),
        reprojection_error_px=0.1,
        image_count=10,
    )
    original = triangulation.normalize_quaternion
    calls: list[tuple[float, ...]] = []

    def counted(value):
        frozen = tuple(value)
        calls.append(frozen)
        return original(frozen)

    monkeypatch.setattr(triangulation, "normalize_quaternion", counted)

    translation = triangulation.camera_translation_right_in_left_camera(
        baseline_right_in_left_world_m=(0.0, 0.4, 0.0),
        left_body_to_world_q=(2.0, 0.0, 0.0, 0.0),
    )
    assert translation is not None
    assert len(calls) == 1

    calls.clear()
    triangulation.triangulate_pixels_with_pixhawk_mount_attempt(
        left_pixel=(340.0, 240.0),
        right_pixel=(300.0, 240.0),
        left_intrinsics=intrinsics,
        right_intrinsics=intrinsics,
        left_body_to_world_q=(2.0, 0.0, 0.0, 0.0),
        right_body_to_world_q=(3.0, 0.0, 0.0, 0.0),
        translation_right_in_left_m=translation,
    )
    assert len(calls) == 2


def _attitude_state(
    side: str,
    *,
    timestamp_s: float,
    q: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0),
) -> SimpleNamespace:
    return SimpleNamespace(
        camera=side,
        namespace=f"{side}-pixhawk",
        timestamp_s=timestamp_s,
        topic="mavlink/ATTITUDE",
        q=q,
    )


def _retained_attitude_panel(**overrides) -> SimpleNamespace:
    panel = SimpleNamespace(
        _measurement_started_monotonic_s=90.0,
        latest_extrinsics={},
        gnss_operation_gate_active=False,
    )
    for name, value in overrides.items():
        setattr(panel, name, value)
    return panel


def test_measurement_attitude_cache_retains_disconnected_side() -> None:
    panel = _retained_attitude_panel(
        latest_extrinsics={
            "left": _attitude_state("left", timestamp_s=91.0),
            "right": _attitude_state("right", timestamp_s=91.1),
        }
    )
    controller = MeasurementViewController(panel)

    first_left, first_right, first_reused = controller._cached_measurement_attitude_states()
    assert first_left is not None and first_right is not None
    assert first_reused == ()

    # The right side disconnects; its owner stops publishing a live sample.
    panel.latest_extrinsics = {"left": _attitude_state("left", timestamp_s=92.0)}
    second_left, second_right, second_reused = controller._cached_measurement_attitude_states()

    assert second_left.timestamp_s == 92.0
    assert second_right is first_right
    assert second_reused == ("right_attitude",)


def test_measurement_attitude_cache_replaces_retained_with_newer_live_sample() -> None:
    panel = _retained_attitude_panel(
        latest_extrinsics={"left": _attitude_state("left", timestamp_s=91.0)}
    )
    controller = MeasurementViewController(panel)
    controller._cached_measurement_attitude_states()

    panel.latest_extrinsics = {}
    _left, _right, reused = controller._cached_measurement_attitude_states()
    assert reused == ("left_attitude",)

    newer = _attitude_state("left", timestamp_s=95.0)
    panel.latest_extrinsics = {"left": newer}
    left, _right, reused = controller._cached_measurement_attitude_states()

    assert left is newer
    assert reused == ()


def test_measurement_attitude_cache_rejects_never_observed_and_invalid_samples() -> None:
    panel = _retained_attitude_panel(
        latest_extrinsics={
            "left": _attitude_state("left", timestamp_s=float("nan")),
            "right": _attitude_state("right", timestamp_s=91.0, q=(0.0, 0.0, 0.0, 0.0)),
        }
    )
    controller = MeasurementViewController(panel)

    left, right, reused = controller._cached_measurement_attitude_states()

    assert left is None
    assert right is None
    assert reused == ()


def test_measurement_attitude_cache_is_process_local_per_controller() -> None:
    panel = _retained_attitude_panel(
        latest_extrinsics={"left": _attitude_state("left", timestamp_s=91.0)}
    )
    MeasurementViewController(panel)._cached_measurement_attitude_states()

    restarted_panel = _retained_attitude_panel()
    left, right, reused = MeasurementViewController(
        restarted_panel
    )._cached_measurement_attitude_states()

    assert left is None
    assert right is None
    assert reused == ()


def test_field_live_readiness_reuses_attitude_after_disconnect(monkeypatch) -> None:
    monkeypatch.setattr("Workstation.measurement_view_controller.time.monotonic", lambda: 100.0)
    left_gps = _gps_state("left", timestamp_s=99.0)
    right_gps = _gps_state("right", timestamp_s=99.1, longitude_deg=127.00001)
    panel = _retained_attitude_panel(
        latest_extrinsics={
            "left": _attitude_state("left", timestamp_s=91.0),
            "right": _attitude_state("right", timestamp_s=91.1),
        },
        latest_rtk_baseline=SimpleNamespace(baseline_m=1.0, warnings=()),
        _selected_baseline_gps_states=lambda: (left_gps, right_gps),
        _gps_baseline_usable_for_translation=lambda _baseline: (True, "usable"),
    )
    controller = MeasurementViewController(panel)
    controller._measurement_live_readiness()

    panel.latest_extrinsics = {}
    pixhawk_ready, gnss_ready, hard, advisories = controller._measurement_live_readiness()

    assert pixhawk_ready is True
    assert gnss_ready is True
    assert hard == ()
    assert any(
        "Pixhawk attitude reused from latest valid sample" in warning
        for warning in advisories
    )


def test_field_live_readiness_blocks_side_that_never_produced_attitude(monkeypatch) -> None:
    monkeypatch.setattr("Workstation.measurement_view_controller.time.monotonic", lambda: 100.0)
    left_gps = _gps_state("left", timestamp_s=99.0)
    right_gps = _gps_state("right", timestamp_s=99.1, longitude_deg=127.00001)
    panel = _retained_attitude_panel(
        latest_extrinsics={"left": _attitude_state("left", timestamp_s=91.0)},
        latest_rtk_baseline=SimpleNamespace(baseline_m=1.0, warnings=()),
        _selected_baseline_gps_states=lambda: (left_gps, right_gps),
        _gps_baseline_usable_for_translation=lambda _baseline: (True, "usable"),
    )

    pixhawk_ready, _gnss_ready, hard, _advisories = MeasurementViewController(
        panel
    )._measurement_live_readiness()

    assert pixhawk_ready is False
    assert "right Pixhawk attitude missing" in hard


def test_measurement_gps_cache_honours_configured_minimum_fix_type() -> None:
    fixed = _gps_state("left", timestamp_s=91.0, fix_type=6)
    floated = _gps_state("left", timestamp_s=92.0, fix_type=5)
    panel = SimpleNamespace(
        _measurement_started_monotonic_s=90.0,
        gnss_operation_manager=SimpleNamespace(
            config=GnssOperationConfig(readiness=ReadinessConfig(minimum_fix_type=6))
        ),
        _pixhawk_gnss_source=lambda _side: "GPS1",
        _selected_baseline_gps_states=lambda: (fixed, None),
    )
    controller = MeasurementViewController(panel)
    controller._cached_measurement_gps_states()

    # A later RTK-float sample must not displace the retained RTK-fixed value.
    panel._selected_baseline_gps_states = lambda: (floated, None)
    left, _right, reused = controller._cached_measurement_gps_states()

    assert left is fixed
    assert "left_gps" in reused


def _correction_frame_panel(*, identity_id: str, base_hash: str, **overrides):
    panel = SimpleNamespace(
        _measurement_started_monotonic_s=90.0,
        gnss_operation_manager=SimpleNamespace(
            config=GnssOperationConfig(),
            source_identity=SimpleNamespace(stable_id=identity_id),
        ),
        gnss_fixed_base_profile=SimpleNamespace(profile_hash=base_hash),
        _pixhawk_gnss_source=lambda _side: "GPS1",
    )
    for name, value in overrides.items():
        setattr(panel, name, value)
    return panel


def test_measurement_gps_cache_survives_unchanged_correction_frame() -> None:
    left = _gps_state("left", timestamp_s=91.0)
    right = _gps_state("right", timestamp_s=95.0, longitude_deg=127.00001)
    panel = _correction_frame_panel(
        identity_id="source-a",
        base_hash="base-1",
        _selected_baseline_gps_states=lambda: (left, right),
    )
    controller = MeasurementViewController(panel)
    controller._cached_measurement_gps_states()

    # P1 is unplugged so P2 can be connected; the frame is unchanged.
    panel._selected_baseline_gps_states = lambda: (None, right)
    cached_left, cached_right, reused = controller._cached_measurement_gps_states()

    assert cached_left is left
    assert cached_right is right
    assert "left_gps" in reused


def test_measurement_gps_cache_drops_retained_pair_when_base_coordinate_changes() -> None:
    left = _gps_state("left", timestamp_s=91.0)
    right = _gps_state("right", timestamp_s=95.0, longitude_deg=127.00001)
    panel = _correction_frame_panel(
        identity_id="source-a",
        base_hash="base-1",
        _selected_baseline_gps_states=lambda: (left, right),
    )
    controller = MeasurementViewController(panel)
    controller._cached_measurement_gps_states()

    # The Base is re-surveyed while only P2 is connected.  Both retained fixes
    # resolve against a coordinate that no longer applies, so both are dropped.
    panel.gnss_fixed_base_profile = SimpleNamespace(profile_hash="base-2")
    panel._selected_baseline_gps_states = lambda: (None, right)
    cached_left, cached_right, _reused = controller._cached_measurement_gps_states()

    assert cached_left is None
    assert cached_right is None

    # P2 is still connected, so its next sample restores it under the new frame.
    refixed = _gps_state("right", timestamp_s=time.monotonic() + 1.0, longitude_deg=127.00001)
    panel._selected_baseline_gps_states = lambda: (None, refixed)
    cached_left, cached_right, _reused = controller._cached_measurement_gps_states()

    assert cached_left is None
    assert cached_right is refixed


def test_measurement_gps_cache_drops_retained_pair_when_correction_source_changes() -> None:
    left = _gps_state("left", timestamp_s=91.0)
    panel = _correction_frame_panel(
        identity_id="source-a",
        base_hash="base-1",
        _selected_baseline_gps_states=lambda: (left, None),
    )
    controller = MeasurementViewController(panel)
    controller._cached_measurement_gps_states()

    panel.gnss_operation_manager = SimpleNamespace(
        config=GnssOperationConfig(),
        source_identity=SimpleNamespace(stable_id="source-b"),
    )
    cached_left, _cached_right, _reused = controller._cached_measurement_gps_states()

    assert cached_left is None


def test_field_live_readiness_reports_sequential_gps_pair_span(monkeypatch) -> None:
    monkeypatch.setattr("Workstation.measurement_view_controller.time.monotonic", lambda: 400.0)
    left_gps = _gps_state("left", timestamp_s=100.0)
    right_gps = _gps_state("right", timestamp_s=340.0, longitude_deg=127.00001)
    panel = _correction_frame_panel(
        identity_id="source-a",
        base_hash="base-1",
        latest_extrinsics={
            "left": _attitude_state("left", timestamp_s=340.0),
            "right": _attitude_state("right", timestamp_s=340.1),
        },
        latest_rtk_baseline=SimpleNamespace(baseline_m=1.0, warnings=()),
        _selected_baseline_gps_states=lambda: (left_gps, right_gps),
        _gps_baseline_usable_for_translation=lambda _baseline: (True, "usable"),
        gnss_operation_gate_active=False,
    )

    _pixhawk_ready, gnss_ready, hard, advisories = MeasurementViewController(
        panel
    )._measurement_live_readiness()

    assert gnss_ready is True
    assert hard == ()
    assert any("240.000s apart" in warning for warning in advisories)


def test_field_live_readiness_omits_pair_span_advisory_within_limit(monkeypatch) -> None:
    monkeypatch.setattr("Workstation.measurement_view_controller.time.monotonic", lambda: 400.0)
    left_gps = _gps_state("left", timestamp_s=399.98)
    right_gps = _gps_state("right", timestamp_s=400.0, longitude_deg=127.00001)
    panel = _correction_frame_panel(
        identity_id="source-a",
        base_hash="base-1",
        latest_extrinsics={
            "left": _attitude_state("left", timestamp_s=399.9),
            "right": _attitude_state("right", timestamp_s=400.0),
        },
        latest_rtk_baseline=SimpleNamespace(baseline_m=1.0, warnings=()),
        _selected_baseline_gps_states=lambda: (left_gps, right_gps),
        _gps_baseline_usable_for_translation=lambda _baseline: (True, "usable"),
        gnss_operation_gate_active=False,
    )

    _pixhawk_ready, gnss_ready, _hard, advisories = MeasurementViewController(
        panel
    )._measurement_live_readiness()

    assert gnss_ready is True
    assert all("apart" not in warning for warning in advisories)


def _vrs_station(x: int = -31780381194, y: int = 41852590653, z: int = 36025161845) -> dict:
    return {"station_id": 4095, "ecef_01mm": (x, y, z), "observed_monotonic_s": 0.0}


def test_measurement_gps_cache_survives_identical_vrs_reference_station() -> None:
    left = _gps_state("left", timestamp_s=91.0)
    right = _gps_state("right", timestamp_s=95.0, longitude_deg=127.00001)
    panel = _correction_frame_panel(
        identity_id="source-a",
        base_hash="",
        latest_rtcm_reference_station=_vrs_station(),
        _selected_baseline_gps_states=lambda: (left, right),
    )
    controller = MeasurementViewController(panel)
    controller._cached_measurement_gps_states()

    # NTRIP reconnects for the P2 session and the caster reissues the same
    # virtual station, so the retained P1 fix stays comparable.
    panel._selected_baseline_gps_states = lambda: (None, right)
    cached_left, cached_right, reused = controller._cached_measurement_gps_states()

    assert cached_left is left
    assert cached_right is right
    assert "left_gps" in reused


def test_measurement_gps_cache_drops_pair_when_vrs_reference_station_moves() -> None:
    left = _gps_state("left", timestamp_s=91.0)
    right = _gps_state("right", timestamp_s=95.0, longitude_deg=127.00001)
    panel = _correction_frame_panel(
        identity_id="source-a",
        base_hash="",
        latest_rtcm_reference_station=_vrs_station(),
        _selected_baseline_gps_states=lambda: (left, right),
    )
    controller = MeasurementViewController(panel)
    controller._cached_measurement_gps_states()

    # The P2 session reconnects with its own GGA, so the caster places the
    # virtual station elsewhere.  The caster/mountpoint identity is unchanged,
    # so only the observed station can reveal the move.
    panel.latest_rtcm_reference_station = _vrs_station(x=-31780381194 + 20000)
    panel._selected_baseline_gps_states = lambda: (None, right)
    cached_left, cached_right, _reused = controller._cached_measurement_gps_states()

    assert cached_left is None
    assert cached_right is None


def test_measurement_correction_frame_token_tracks_the_observed_station() -> None:
    panel = _correction_frame_panel(
        identity_id="source-a",
        base_hash="",
        latest_rtcm_reference_station=_vrs_station(),
        _selected_baseline_gps_states=lambda: (None, None),
    )
    controller = MeasurementViewController(panel)
    first = controller._measurement_correction_frame_token()

    panel.latest_rtcm_reference_station = _vrs_station(z=36025161845 + 1)
    assert controller._measurement_correction_frame_token() != first

    panel.latest_rtcm_reference_station = _vrs_station()
    assert controller._measurement_correction_frame_token() == first


def test_field_live_readiness_explains_a_correction_frame_drop(monkeypatch) -> None:
    monkeypatch.setattr("Workstation.measurement_view_controller.time.monotonic", lambda: 400.0)
    left = _gps_state("left", timestamp_s=390.0)
    right = _gps_state("right", timestamp_s=395.0, longitude_deg=127.00001)
    panel = _correction_frame_panel(
        identity_id="source-a",
        base_hash="",
        latest_rtcm_reference_station=_vrs_station(),
        latest_extrinsics={
            "left": _attitude_state("left", timestamp_s=399.0),
            "right": _attitude_state("right", timestamp_s=399.1),
        },
        latest_rtk_baseline=SimpleNamespace(baseline_m=1.0, warnings=()),
        _selected_baseline_gps_states=lambda: (left, right),
        _gps_baseline_usable_for_translation=lambda _baseline: (True, "usable"),
        gnss_operation_gate_active=False,
    )
    controller = MeasurementViewController(panel)
    controller._measurement_live_readiness()

    # The P2 session reconnects and the caster issues a different virtual
    # station, so both retained fixes are discarded.
    panel.latest_rtcm_reference_station = _vrs_station(x=-31780381194 + 20000)
    panel._selected_baseline_gps_states = lambda: (None, None)
    _pixhawk_ready, gnss_ready, hard, advisories = controller._measurement_live_readiness()

    assert gnss_ready is False
    assert "live GNSS baseline missing" in hard
    assert any(
        "GNSS correction frame changed" in warning
        and "left,right" in warning
        and "reconnect that board" in warning
        for warning in advisories
    )


def test_field_live_readiness_has_no_frame_drop_advisory_in_normal_operation(monkeypatch) -> None:
    monkeypatch.setattr("Workstation.measurement_view_controller.time.monotonic", lambda: 400.0)
    left = _gps_state("left", timestamp_s=399.0)
    right = _gps_state("right", timestamp_s=399.1, longitude_deg=127.00001)
    panel = _correction_frame_panel(
        identity_id="source-a",
        base_hash="",
        latest_rtcm_reference_station=_vrs_station(),
        latest_extrinsics={
            "left": _attitude_state("left", timestamp_s=399.0),
            "right": _attitude_state("right", timestamp_s=399.1),
        },
        latest_rtk_baseline=SimpleNamespace(baseline_m=1.0, warnings=()),
        _selected_baseline_gps_states=lambda: (left, right),
        _gps_baseline_usable_for_translation=lambda _baseline: (True, "usable"),
        gnss_operation_gate_active=False,
    )

    _pixhawk_ready, gnss_ready, _hard, advisories = MeasurementViewController(
        panel
    )._measurement_live_readiness()

    assert gnss_ready is True
    assert all("correction frame changed" not in warning for warning in advisories)


def test_cloud_gnss_is_the_tenth_catalog_method() -> None:
    from Workstation.workstation_core.measurement_methods import (
        CLOUD_GNSS,
        MEASUREMENT_METHOD_LABELS,
        MEASUREMENT_METHOD_MISSING_INSTANCE_REASONS,
        MEASUREMENT_METHOD_REQUIRED_INPUTS,
        measurement_method_id_for_candidate,
    )
    from Workstation.workstation_core.measurement_record import METHOD_SHEET_NAMES as RECORD_SHEETS
    from Workstation.workstation_core.session_review import METHOD_SHEET_NAMES as REVIEW_SHEETS

    assert len(MEASUREMENT_METHOD_IDS) == 10
    assert MEASUREMENT_METHOD_IDS[-1] == CLOUD_GNSS == "cloud_gnss"
    assert "Cloud" in MEASUREMENT_METHOD_LABELS[CLOUD_GNSS]
    assert "configured GNSS translation fill" in MEASUREMENT_METHOD_REQUIRED_INPUTS[CLOUD_GNSS]
    assert CLOUD_GNSS in MEASUREMENT_METHOD_MISSING_INSTANCE_REASONS
    assert measurement_method_id_for_candidate("cloud-far-field-rotation") == CLOUD_GNSS
    assert measurement_method_id_for_candidate("optical_pose/cloud-far-field-rotation") == CLOUD_GNSS
    assert RECORD_SHEETS[CLOUD_GNSS] == "19_Cloud_GNSS"
    assert REVIEW_SHEETS[CLOUD_GNSS] == "M_Cloud_GNSS"
