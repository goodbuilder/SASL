from __future__ import annotations

from dataclasses import replace

import pytest

from wiris_stereo_distance.fixed_base_initializer import FixedBaseInitializer
from wiris_stereo_distance.gnss_operation import (
    CorrectionSourceIdentity,
    FixedBaseConfig,
    GnssReadinessInputs,
    GnssOperationConfig,
    GnssOperationManager,
    GnssOperationMode,
    GnssOperationState,
    HeightType,
    MeasurementGnssContract,
    NetworkConfig,
    ReadinessConfig,
    RoverCorrectionStatus,
    RoverReadinessInput,
    RoutingConfig,
    build_measurement_gnss_contract,
    evaluate_gnss_readiness,
)
from wiris_stereo_distance.gnss_profile import (
    GnssCoordinateSample,
    build_coordinate_profile,
    geodetic_to_ecef_m,
)


def fixed_sample(timestamp_s: float, *, lat: float = 37.0, lon: float = 127.0) -> GnssCoordinateSample:
    return GnssCoordinateSample(
        latitude_deg=lat,
        longitude_deg=lon,
        ellipsoid_height_m=52.0,
        msl_altitude_m=48.0,
        height_type=HeightType.ELLIPSOID,
        timestamp_s=timestamp_s,
        fixed=True,
        device_id="base-f9p",
    )


def test_operation_config_roundtrip_keeps_mode_and_network_identity() -> None:
    config = GnssOperationConfig(
        operation_mode=GnssOperationMode.NETWORK_ASSISTED_FIXED_BASE,
        network=NetworkConfig(caster_host="caster.example", mountpoint="MOUNT"),
    )

    restored = GnssOperationConfig.from_mapping(config.to_mapping())

    assert restored == config
    assert restored.network.mountpoint == "MOUNT"


def test_ecef_conversion_keeps_wgs84_geodetic_contract() -> None:
    x, y, z = geodetic_to_ecef_m(0.0, 0.0, 0.0)

    assert x == pytest.approx(6378137.0, abs=1e-3)
    assert y == pytest.approx(0.0, abs=1e-3)
    assert z == pytest.approx(0.0, abs=1e-3)


def test_fixed_base_initializer_requires_duration_ratio_and_scatter_limits() -> None:
    config = FixedBaseConfig(
        acquisition_duration_s=10.0,
        minimum_fixed_duration_s=8.0,
        minimum_fixed_ratio=0.8,
        maximum_horizontal_scatter_m=0.03,
        maximum_vertical_scatter_m=0.05,
        height_type=HeightType.ELLIPSOID,
    )
    validation = FixedBaseInitializer(config).validate([fixed_sample(t) for t in range(0, 11)])

    assert validation.valid is True
    assert validation.profile is not None
    assert validation.profile.fixed_duration_s == pytest.approx(10.0)
    assert validation.profile.profile_hash


def test_fixed_base_initializer_rejects_mixed_height_contract() -> None:
    rows = [fixed_sample(0.0), replace(fixed_sample(1.0), height_type=HeightType.MSL)]
    validation = FixedBaseInitializer(FixedBaseConfig(height_type=HeightType.ELLIPSOID)).validate(rows)

    assert validation.valid is False
    assert validation.profile is None
    assert "one known height_type" in validation.reasons[0]


def test_operation_manager_records_source_and_rejects_invalid_transition() -> None:
    manager = GnssOperationManager(GnssOperationConfig())
    identity = CorrectionSourceIdentity(source_kind="ntrip-direct", caster_host="caster.example", mountpoint="MOUNT")
    manager.configure(manager.config)
    manager.set_source_identity(identity)
    manager.transition(GnssOperationState.INITIALIZING, "start")
    manager.transition(GnssOperationState.NTRIP_CONNECTING, "connect")

    with pytest.raises(ValueError):
        manager.transition(GnssOperationState.READY, "skip transport")

    metadata = manager.metadata()
    assert metadata["correction_source_id"] == identity.stable_id
    assert metadata["correction_source"]["mountpoint"] == "MOUNT"


def readiness_route(target: str, now_s: float) -> RoverCorrectionStatus:
    return RoverCorrectionStatus(
        target=target,
        enabled=True,
        endpoint_identity=f"{target}-endpoint",
        bytes_sent=128,
        packets_sent=2,
        last_delivery_monotonic_s=now_s - 0.5,
    )


def readiness_rover(target: str, now_s: float) -> RoverReadinessInput:
    return RoverReadinessInput(
        target=target,
        fix_type=6,
        gps_sample_monotonic_s=now_s - 0.5,
        eph_m=0.03,
        epv_m=0.05,
        rtk_fixed_since_monotonic_s=now_s - 30.0,
        device_id=f"{target}-f9p",
    )


def two_rover_readiness_inputs(now_s: float, *, source_id: str = "source") -> GnssReadinessInputs:
    return GnssReadinessInputs(
        state=GnssOperationState.READY,
        now_monotonic_s=now_s,
        correction_received_monotonic_s=now_s - 3.0,
        correction_source_id=source_id,
        routes={
            "left": readiness_route("left", now_s),
            "right": readiness_route("right", now_s),
        },
        rover_samples={
            "left": readiness_rover("left", now_s),
            "right": readiness_rover("right", now_s),
        },
        left_right_timestamp_delta_ms=50.0,
        baseline_std_m=0.02,
    )


def test_readiness_evaluator_accepts_exact_threshold_boundaries() -> None:
    config = GnssOperationConfig(
        operation_mode=GnssOperationMode.NETWORK_RTK_DIRECT,
        readiness=ReadinessConfig(
            minimum_fixed_duration_s=30.0,
            maximum_correction_age_s=3.0,
            maximum_gps_sample_age_s=1.0,
            maximum_horizontal_accuracy_m=0.03,
            maximum_vertical_accuracy_m=0.05,
            maximum_left_right_timestamp_delta_ms=50.0,
            maximum_baseline_std_m=0.02,
        ),
    )

    snapshot = evaluate_gnss_readiness(config, two_rover_readiness_inputs(100.0))

    assert snapshot.ready is True
    assert snapshot.reasons == ()


@pytest.mark.parametrize(
    ("inputs", "reason"),
    [
        (
            replace(two_rover_readiness_inputs(100.0), correction_received_monotonic_s=96.999),
            "correction_stale",
        ),
        (
            replace(
                two_rover_readiness_inputs(100.0),
                routes={**two_rover_readiness_inputs(100.0).routes, "left": readiness_route("left", 95.0)},
            ),
            "route_stale",
        ),
        (
            replace(
                two_rover_readiness_inputs(100.0),
                rover_samples={
                    **two_rover_readiness_inputs(100.0).rover_samples,
                    "left": replace(readiness_rover("left", 100.0), rtk_fixed_since_monotonic_s=70.001),
                },
            ),
            "rover_fixed_duration_short",
        ),
        (
            replace(
                two_rover_readiness_inputs(100.0),
                rover_samples={
                    **two_rover_readiness_inputs(100.0).rover_samples,
                    "left": replace(readiness_rover("left", 100.0), gps_sample_monotonic_s=98.999),
                },
            ),
            "gps_sample_stale",
        ),
        (
            replace(
                two_rover_readiness_inputs(100.0),
                rover_samples={
                    **two_rover_readiness_inputs(100.0).rover_samples,
                    "left": replace(readiness_rover("left", 100.0), eph_m=0.0301),
                },
            ),
            "eph_high",
        ),
        (
            replace(
                two_rover_readiness_inputs(100.0),
                rover_samples={
                    **two_rover_readiness_inputs(100.0).rover_samples,
                    "left": replace(readiness_rover("left", 100.0), epv_m=0.0501),
                },
            ),
            "epv_high",
        ),
        (replace(two_rover_readiness_inputs(100.0), left_right_timestamp_delta_ms=50.1), "left_right_timestamp_delta_high"),
        (replace(two_rover_readiness_inputs(100.0), baseline_std_m=0.0201), "baseline_std_high"),
    ],
)
def test_readiness_evaluator_fails_just_outside_thresholds(inputs: GnssReadinessInputs, reason: str) -> None:
    snapshot = evaluate_gnss_readiness(GnssOperationConfig(operation_mode=GnssOperationMode.NETWORK_RTK_DIRECT), inputs)

    assert snapshot.ready is False
    assert reason in snapshot.reasons
    assert snapshot.reason_messages[reason]


def test_readiness_evaluator_fails_closed_for_missing_required_values() -> None:
    inputs = GnssReadinessInputs(
        state=GnssOperationState.READY,
        now_monotonic_s=100.0,
        correction_received_monotonic_s=None,
        routes={"left": replace(readiness_route("left", 100.0), endpoint_identity="", bytes_sent=0, packets_sent=0)},
        rover_samples={"left": replace(readiness_rover("left", 100.0), eph_m=None, epv_m=None, gps_sample_monotonic_s=None)},
    )
    config = GnssOperationConfig(routing=RoutingConfig(left_enabled=True, right_enabled=False))

    snapshot = evaluate_gnss_readiness(config, inputs)

    assert snapshot.ready is False
    assert "correction_missing" in snapshot.reasons
    assert "route_endpoint_missing" in snapshot.reasons
    assert "route_not_injected" in snapshot.reasons
    assert "gps_sample_missing" in snapshot.reasons
    assert "eph_missing" in snapshot.reasons
    assert "epv_missing" in snapshot.reasons


def test_single_rover_readiness_does_not_require_two_rover_baseline_metrics() -> None:
    now_s = 100.0
    config = GnssOperationConfig(
        operation_mode=GnssOperationMode.NETWORK_RTK_DIRECT,
        routing=RoutingConfig(left_enabled=True, right_enabled=False),
    )
    inputs = GnssReadinessInputs(
        state=GnssOperationState.READY,
        now_monotonic_s=now_s,
        correction_received_monotonic_s=now_s - 0.5,
        routes={"left": readiness_route("left", now_s)},
        rover_samples={"left": readiness_rover("left", now_s)},
    )

    snapshot = evaluate_gnss_readiness(config, inputs)

    assert snapshot.ready is True
    assert "left_right_timestamp_delta_missing" not in snapshot.reasons
    assert "baseline_std_missing" not in snapshot.reasons


def test_required_drone_rtk_fixed_fails_closed_until_fixed() -> None:
    now_s = 100.0
    config = GnssOperationConfig(
        operation_mode=GnssOperationMode.NETWORK_RTK_DIRECT,
        require_drone_rtk_fixed=True,
        routing=RoutingConfig(left_enabled=True, right_enabled=False, drone_enabled=True),
    )
    missing_drone = GnssReadinessInputs(
        state=GnssOperationState.READY,
        now_monotonic_s=now_s,
        correction_received_monotonic_s=now_s - 0.5,
        routes={"left": readiness_route("left", now_s), "drone": readiness_route("drone", now_s)},
        rover_samples={"left": readiness_rover("left", now_s)},
    )
    float_drone = replace(
        missing_drone,
        rover_samples={"left": readiness_rover("left", now_s), "drone": replace(readiness_rover("drone", now_s), fix_type=5)},
    )

    assert "drone_missing" in evaluate_gnss_readiness(config, missing_drone).reasons
    assert "drone_not_fixed" in evaluate_gnss_readiness(config, float_drone).reasons


def test_gate_disabled_preserves_computed_not_ready_with_warning() -> None:
    config = GnssOperationConfig(readiness_gate_enabled=False, routing=RoutingConfig(left_enabled=True, right_enabled=False))
    inputs = GnssReadinessInputs(
        state=GnssOperationState.READY,
        now_monotonic_s=100.0,
        correction_received_monotonic_s=None,
        routes={"left": readiness_route("left", 100.0)},
        rover_samples={"left": readiness_rover("left", 100.0)},
    )

    snapshot = evaluate_gnss_readiness(config, inputs)

    assert snapshot.ready is False
    assert "correction_missing" in snapshot.reasons
    assert snapshot.warnings == ("gate_disabled",)


def test_measurement_contract_stable_id_is_order_independent_and_in_metadata() -> None:
    source = CorrectionSourceIdentity(source_kind="ntrip-direct", caster_host="caster.example", caster_port=2101, mountpoint="MOUNT")
    readiness = evaluate_gnss_readiness(
        GnssOperationConfig(operation_mode=GnssOperationMode.NETWORK_RTK_DIRECT),
        two_rover_readiness_inputs(100.0, source_id=source.stable_id),
    )

    left_first = build_measurement_gnss_contract(
        GnssOperationConfig(operation_mode=GnssOperationMode.NETWORK_RTK_DIRECT),
        source,
        readiness,
        base_coordinate_profile_hash="base-hash",
        baseline_profile_hash="baseline-hash",
        device_ids={"left": "left-f9p", "right": "right-f9p"},
        route_endpoints={"left": "left-endpoint", "right": "right-endpoint"},
    )
    right_first = build_measurement_gnss_contract(
        GnssOperationConfig(operation_mode=GnssOperationMode.NETWORK_RTK_DIRECT),
        source,
        readiness,
        base_coordinate_profile_hash="base-hash",
        baseline_profile_hash="baseline-hash",
        device_ids={"right": "right-f9p", "left": "left-f9p"},
        route_endpoints={"right": "right-endpoint", "left": "left-endpoint"},
    )

    assert left_first.stable_id == right_first.stable_id
    assert left_first.to_metadata()["gnss_contract_id"] == left_first.stable_id


def transition_manager_to_ready(manager: GnssOperationManager) -> None:
    manager.configure(manager.config)
    manager.transition(GnssOperationState.INITIALIZING, "start")
    manager.transition(GnssOperationState.NTRIP_ACTIVE, "stream active")
    manager.transition(GnssOperationState.ROVERS_CONVERGING, "rovers")
    manager.transition(GnssOperationState.ALIGNMENT_COLLECTING, "alignment")
    manager.transition(GnssOperationState.ALIGNMENT_VALID, "alignment valid")
    manager.transition(GnssOperationState.READY, "ready")


def test_manager_captures_validates_and_invalidates_on_contract_change() -> None:
    source = CorrectionSourceIdentity(source_kind="ntrip-direct", caster_host="caster.example", caster_port=2101, mountpoint="MOUNT")
    config = GnssOperationConfig(operation_mode=GnssOperationMode.NETWORK_RTK_DIRECT)
    manager = GnssOperationManager(config)
    manager.set_source_identity(source)
    transition_manager_to_ready(manager)
    readiness = evaluate_gnss_readiness(config, two_rover_readiness_inputs(100.0, source_id=source.stable_id))

    contract = manager.capture_measurement_contract(
        readiness,
        base_coordinate_profile_hash="base",
        baseline_profile_hash="baseline",
        device_ids={"left": "left-f9p", "right": "right-f9p"},
        route_endpoints={"left": "left-endpoint", "right": "right-endpoint"},
    )
    manager.begin_measurement(contract, readiness)

    assert manager.validate_measurement_contract(contract, readiness) is True
    changed = replace(contract, route_endpoints=(("left", "other-endpoint"), ("right", "right-endpoint")))
    assert manager.validate_measurement_contract(changed, readiness) is False
    assert manager.state == GnssOperationState.INVALIDATED
    assert manager.invalidation_reason == "GNSS measurement contract changed"


def test_gate_disabled_measurement_starts_without_asserted_gnss_contract() -> None:
    config = GnssOperationConfig(readiness_gate_enabled=False, routing=RoutingConfig(left_enabled=True, right_enabled=False))
    source = CorrectionSourceIdentity(source_kind="ntrip-direct", caster_host="caster.example", caster_port=2101, mountpoint="MOUNT")
    manager = GnssOperationManager(config)
    manager.set_source_identity(source)
    transition_manager_to_ready(manager)
    readiness = evaluate_gnss_readiness(
        config,
        GnssReadinessInputs(
            state=GnssOperationState.READY,
            now_monotonic_s=100.0,
            correction_received_monotonic_s=None,
            correction_source_id=source.stable_id,
            routes={"left": readiness_route("left", 100.0)},
            rover_samples={"left": readiness_rover("left", 100.0)},
        ),
    )

    contract = manager.capture_measurement_contract(readiness)
    manager.begin_measurement(contract, readiness)

    assert contract is None
    assert manager.state == GnssOperationState.MEASURING
    assert manager.measurement_contract is None
    assert manager.metadata()["readiness"]["warnings"] == ["gate_disabled"]
