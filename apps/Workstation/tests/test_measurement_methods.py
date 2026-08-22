from __future__ import annotations

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
from Workstation.workstation_core.settings import load_settings, save_settings


def _gps_state(
    side: str,
    *,
    timestamp_s: float,
    longitude_deg: float = 127.0,
) -> GpsPositionState:
    return GpsPositionState(
        camera=side,
        namespace=f"{side}-pixhawk",
        timestamp_s=timestamp_s,
        topic="mavlink/GPS_RAW_INT",
        latitude_deg=37.0,
        longitude_deg=longitude_deg,
        altitude_m=100.0,
        fix_type=6,
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


def test_zoom_mismatch_blocks_every_measurement_geometry_source() -> None:
    profile_hash = "a" * 64
    controller = _method_readiness_controller(
        geometry_allowed=False,
        profile_hash=profile_hash,
        applicability_reasons=("left_operating_zoom_mismatch",),
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
        "K,D applicability: left_operating_zoom_mismatch" in row.reasons
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
            "left": SimpleNamespace(timestamp_s=91.0),
            "right": SimpleNamespace(timestamp_s=91.1),
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


def test_field_live_readiness_rejects_samples_from_before_measurement(monkeypatch) -> None:
    monkeypatch.setattr("Workstation.measurement_view_controller.time.monotonic", lambda: 100.0)
    left_gps = _gps_state("left", timestamp_s=80.0)
    right_gps = _gps_state("right", timestamp_s=80.1, longitude_deg=127.00001)
    panel = SimpleNamespace(
        _measurement_started_monotonic_s=90.0,
        latest_extrinsics={
            "left": SimpleNamespace(timestamp_s=80.0),
            "right": SimpleNamespace(timestamp_s=80.1),
        },
        latest_rtk_baseline=SimpleNamespace(baseline_m=1.0, warnings=()),
        _selected_baseline_gps_states=lambda: (left_gps, right_gps),
        _gps_baseline_usable_for_translation=lambda _baseline: (True, "usable"),
        gnss_operation_gate_active=False,
    )

    pixhawk_ready, gnss_ready, hard, _advisories = MeasurementViewController(
        panel
    )._measurement_live_readiness()

    assert pixhawk_ready is False
    assert gnss_ready is False
    assert any("not received in current Measurement" in reason for reason in hard)


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

    assert status.ready is True
    assert "candidate operating K,D profile SHA-256 missing" in status.warnings


def test_valid_exact_kd_keeps_cross_zoom_rt_geometry_ready() -> None:
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
    assert all(row.ready is True for row in selected)
    assert all(row.reasons == () for row in selected)
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
