import json
from types import SimpleNamespace

from Workstation.workstation_core.gnss_monitor import (
    NtripHealthState,
    evaluate_ntrip_health,
)
from Workstation.workstation_core.gnss_network import (
    build_ntrip_request,
    gga_from_coordinate_sample,
    gga_from_position,
)
from Workstation.workstation_core.gnss_operation import (
    CorrectionSourceIdentity,
    GgaPolicy,
    GnssOperationConfig,
    GnssOperationManager,
    GnssOperationMode,
    GnssOperationState,
    GnssReadinessSnapshot,
    HeightType,
    NetworkConfig,
)


def test_gga_from_position_reports_rtk_fixed_as_nmea_quality_4() -> None:
    # NMEA GGA fix_quality is a different enum than MAVLink fix_type: 4 is
    # RTK Fixed there, not 6 (which means "Estimated/dead reckoning").
    fixed = SimpleNamespace(
        latitude_deg=37.0, longitude_deg=127.0, altitude_m=52.0, fix_type=6, satellites_used=14, eph_m=0.02
    )
    not_fixed = SimpleNamespace(
        latitude_deg=37.0, longitude_deg=127.0, altitude_m=52.0, fix_type=3, satellites_used=14, eph_m=0.02
    )

    assert gga_from_position(fixed).split(",")[6] == "4"
    assert gga_from_position(not_fixed).split(",")[6] == "1"


def test_gga_from_coordinate_sample_reports_rtk_fixed_as_nmea_quality_4() -> None:
    fixed = SimpleNamespace(latitude_deg=37.0, longitude_deg=127.0, msl_altitude_m=52.0, fixed=True)
    not_fixed = SimpleNamespace(latitude_deg=37.0, longitude_deg=127.0, msl_altitude_m=52.0, fixed=False)

    assert gga_from_coordinate_sample(fixed).split(",")[6] == "4"
    assert gga_from_coordinate_sample(not_fixed).split(",")[6] == "1"


def test_ngii_rts2_base_assisted_profile_uses_non_gating_field_defaults() -> None:
    config = GnssOperationConfig.ngii_rts2_base_assisted()

    assert config.operation_mode is GnssOperationMode.NETWORK_ASSISTED_FIXED_BASE
    assert config.network.caster_host == "RTS2.ngii.go.kr"
    assert config.network.caster_port == 2101
    assert config.network.mountpoint == "VRS-RTCM32"
    assert config.network.username_env == "NGII_RTS2_USERNAME"
    assert config.network.password_env == "NGII_RTS2_PASSWORD"
    assert config.network.gga_policy is GgaPolicy.BASE_GNSS
    assert config.fixed_base.height_type is HeightType.ELLIPSOID
    assert config.routing.left_enabled is True
    assert config.routing.right_enabled is True
    assert config.routing.field_base_enabled is True
    assert config.readiness_gate_enabled is False


def test_new_settings_file_selects_ngii_rts2_profile(tmp_path) -> None:
    from Workstation.workstation_core.settings import load_settings

    settings = load_settings(tmp_path / "settings.json")

    assert settings.gnss_operation == GnssOperationConfig.ngii_rts2_base_assisted()


def test_retired_caster_settings_are_migrated_to_ngii_rts2(tmp_path) -> None:
    from Workstation.workstation_core.settings import load_settings

    path = tmp_path / "settings.json"
    path.write_text(
        json.dumps(
            {
                "gnss_operation": {
                    "operation_mode": "NETWORK_ASSISTED_FIXED_BASE",
                    "network": {
                        "caster_host": "rtk2go.com",
                        "mountpoint": "KAIST-RTK-RTCM",
                    },
                }
            }
        )
    )

    settings = load_settings(path)

    assert settings.gnss_operation == GnssOperationConfig.ngii_rts2_base_assisted()


def test_ngii_rts2_readiness_is_observable_without_blocking_measurement() -> None:
    config = GnssOperationConfig.ngii_rts2_base_assisted()
    manager = GnssOperationManager(config)
    manager.configure(config)
    source = CorrectionSourceIdentity(
        source_kind="NTRIP",
        caster_host=config.network.caster_host,
        caster_port=config.network.caster_port,
        mountpoint=config.network.mountpoint,
        gga_policy=config.network.gga_policy,
    )
    manager.set_source_identity(source)
    for state in (
        GnssOperationState.INITIALIZING,
        GnssOperationState.NTRIP_CONNECTING,
        GnssOperationState.NTRIP_ACTIVE,
        GnssOperationState.BASE_AS_ROVER_FLOAT,
        GnssOperationState.BASE_AS_ROVER_FIXED,
        GnssOperationState.FIXED_COORDINATE_COLLECTING,
        GnssOperationState.FIXED_COORDINATE_VALIDATED,
        GnssOperationState.BASE_RECONFIGURING,
        GnssOperationState.LOCAL_RTCM_ACTIVE,
        GnssOperationState.ROVERS_CONVERGING,
        GnssOperationState.ALIGNMENT_COLLECTING,
        GnssOperationState.ALIGNMENT_VALID,
        GnssOperationState.READY,
    ):
        manager.transition(state, "test")

    snapshot = GnssReadinessSnapshot(
        ready=False,
        state=GnssOperationState.READY,
        reasons=("rover_not_fixed",),
        warnings=("gate_disabled",),
        mode=config.operation_mode,
        correction_source_id=source.stable_id,
        timestamp_s=1.0,
    )
    manager.begin_measurement(None, snapshot)

    assert manager.state is GnssOperationState.MEASURING
    assert manager.measurement_contract is None
    assert manager.validate_measurement_contract(None, snapshot) is True


def test_new_ntrip_configuration_is_provider_neutral() -> None:
    config = NetworkConfig()

    assert config.caster_host == ""
    assert config.mountpoint == ""
    assert config.caster_port == 2101


def test_arbitrary_ntrip_caster_round_trips_and_builds_request() -> None:
    original = NetworkConfig(
        caster_host="caster.example.org",
        caster_port=2201,
        mountpoint="/VRS_RTCM32",
        username_env="EXAMPLE_NTRIP_USER",
        password_env="EXAMPLE_NTRIP_PASSWORD",
        tls=True,
    )

    restored = NetworkConfig.from_mapping(original.to_mapping())
    request = build_ntrip_request(restored, username="field-user", password="secret")

    assert restored == original
    assert restored.mountpoint == "VRS_RTCM32"
    assert request.startswith(b"GET /VRS_RTCM32 HTTP/1.0\r\n")
    assert b"Host: caster.example.org\r\n" in request
    assert b"Authorization: Basic " in request
    assert b"secret" not in request


def test_ntrip_health_distinguishes_transport_from_valid_rtcm() -> None:
    active = evaluate_ntrip_health(
        network_mode=True,
        connected=True,
        running=True,
        phase="STREAMING",
        connected_at_s=90.0,
        last_chunk_s=99.5,
        last_rtcm_s=99.0,
        now_s=100.0,
        transport_stale_after_s=10.0,
        correction_stale_after_s=3.0,
        bytes_received=2400,
        chunks_received=12,
    )
    assert active.state == NtripHealthState.RTCM_ACTIVE
    assert active.transport_age_s == 0.5
    assert active.rtcm_age_s == 1.0

    streaming_only = evaluate_ntrip_health(
        network_mode=True,
        connected=True,
        running=True,
        phase="STREAMING",
        connected_at_s=90.0,
        last_chunk_s=99.5,
        last_rtcm_s=None,
        now_s=100.0,
        transport_stale_after_s=10.0,
        correction_stale_after_s=3.0,
    )
    assert streaming_only.state == NtripHealthState.STREAMING

    rtcm_stale = evaluate_ntrip_health(
        network_mode=True,
        connected=True,
        running=True,
        phase="STREAMING",
        connected_at_s=90.0,
        last_chunk_s=99.5,
        last_rtcm_s=96.0,
        now_s=100.0,
        transport_stale_after_s=10.0,
        correction_stale_after_s=3.0,
    )
    assert rtcm_stale.state == NtripHealthState.RTCM_STALE

    reconnecting = evaluate_ntrip_health(
        network_mode=True,
        connected=False,
        running=True,
        phase="RECONNECTING",
        connected_at_s=90.0,
        last_chunk_s=99.5,
        last_rtcm_s=99.0,
        now_s=100.0,
        transport_stale_after_s=10.0,
        correction_stale_after_s=3.0,
    )
    assert reconnecting.state == NtripHealthState.RECONNECTING


def test_ntrip_worker_emits_updated_stream_stats_after_each_chunk(monkeypatch) -> None:
    import Workstation.gnss_runtime_workers as runtime_workers

    class FakeClient:
        def __init__(self, _config) -> None:
            self._stats = None

        @property
        def stats(self):
            return self._stats

        def connect(self, *, gga_sentence=None):
            self._stats = SimpleNamespace(
                connected_at_monotonic_s=10.0,
                bytes_received=0,
                chunks_received=0,
                last_chunk_monotonic_s=None,
                last_gga_monotonic_s=None,
            )
            return "ICY 200 OK", b""

        def iter_chunks(self, stop_event, **_kwargs):
            self._stats = SimpleNamespace(
                connected_at_monotonic_s=10.0,
                bytes_received=128,
                chunks_received=1,
                last_chunk_monotonic_s=11.0,
                last_gga_monotonic_s=None,
            )
            yield b"rtcm-chunk"
            stop_event.set()

        def close(self):
            return None

    monkeypatch.setattr(runtime_workers, "NtripClient", FakeClient)
    worker = runtime_workers.NtripNetworkWorker(config=NetworkConfig())
    connected: list[object] = []
    stats: list[object] = []
    chunks: list[object] = []
    worker.connected.connect(lambda _status, value: connected.append(value))
    worker.stats_changed.connect(stats.append)
    worker.chunk_received.connect(chunks.append)

    worker.run()

    assert len(connected) == 1
    assert len(stats) == 1
    assert stats[0].chunks_received == 1
    assert stats[0].bytes_received == 128
    assert chunks == [b"rtcm-chunk"]
