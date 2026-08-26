import json
import threading
import time
from types import SimpleNamespace

import pytest
from PyQt5 import QtCore, QtWidgets

from Workstation.gnss_runtime_controller import (
    GnssRuntimeController,
    NtripCredentialError,
    NtripCredentialResolution,
    _connect_gui_signal,
)
from Workstation.settings_workspace import SettingsWorkspace
from Workstation.workstation_core import ntrip_credentials
from Workstation.workstation_core.gnss_monitor import (
    NtripHealthState,
    evaluate_ntrip_health,
)
from Workstation.workstation_core.gnss_network import (
    NtripClient,
    NtripProtocolError,
    build_ntrip_request,
    gga_from_coordinate_sample,
    gga_from_position,
)
from Workstation.workstation_core.gnss_operation import (
    CorrectionSourceIdentity,
    DEFAULT_NTRIP_SESSION_PASSWORD,
    DEFAULT_NTRIP_USERNAME,
    GgaPolicy,
    GnssOperationConfig,
    GnssOperationManager,
    GnssOperationMode,
    GnssOperationState,
    GnssReadinessSnapshot,
    HeightType,
    NetworkConfig,
    RoutingConfig,
)
from Workstation.workstation_core.rtcm_pipeline import RtcmRouteConfig
from Workstation.workstation_core.settings import AppSettings, load_settings, save_settings


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


def test_gga_preserves_msl_altitude_and_geoid_separation_contract() -> None:
    coordinate = SimpleNamespace(
        latitude_deg=37.0,
        longitude_deg=127.0,
        msl_altitude_m=48.0,
        ellipsoid_height_m=52.0,
        fixed=True,
        satellites_used=17,
    )
    rover = SimpleNamespace(
        latitude_deg=37.0,
        longitude_deg=127.0,
        altitude_m=48.0,
        altitude_ellipsoid_m=52.0,
        fix_type=6,
        satellites_used=17,
        eph_m=0.02,
    )

    for sentence in (gga_from_coordinate_sample(coordinate), gga_from_position(rover)):
        assert sentence is not None
        fields = sentence.split(",")
        assert fields[9:13] == ["48.000", "M", "4.000", "M"]

    unknown = SimpleNamespace(
        latitude_deg=37.0,
        longitude_deg=127.0,
        altitude_m=48.0,
        fix_type=3,
        satellites_used=8,
        eph_m=1.0,
    )
    assert gga_from_position(unknown).split(",")[11] == ""


def test_ngii_rts2_base_assisted_profile_uses_non_gating_field_defaults() -> None:
    config = GnssOperationConfig.ngii_rts2_base_assisted()

    assert config.operation_mode is GnssOperationMode.NETWORK_ASSISTED_FIXED_BASE
    assert config.network.caster_host == "RTS2.ngii.go.kr"
    assert config.network.caster_port == 2101
    assert config.network.mountpoint == "VRS-RTCM32"
    assert config.network.username == DEFAULT_NTRIP_USERNAME
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


def test_rtcm_receive_throttle_does_not_suppress_injection_success_refresh() -> None:
    calls: list[tuple[object, ...]] = []
    panel = SimpleNamespace(
        gnss_base_status_labels={"rtcm_injection": object()},
        _record_gnss_base_rtcm_injection=lambda *args, **kwargs: calls.append(
            ("record", args, kwargs)
        ),
        _px4_mag_camera_label=lambda side: str(side),
        _set_health_label=lambda _label, text, level: calls.append(
            ("label", text, level)
        ),
        _refresh_gnss_base_rtcm_stats=lambda: calls.append(("stats",)),
        _refresh_gnss_operation_readiness=lambda: calls.append(("readiness",)),
        _log=lambda _message: None,
    )
    controller = GnssRuntimeController(panel)

    assert controller._rtcm_ui_update_due() is True
    controller._apply_pixhawk_rtcm_result(
        {"payload_bytes": 16, "sent": (("left", 1),), "failed": (), "dropped": 0}
    )

    assert ("label", "RTCM 16 bytes -> left", "good") in calls
    assert ("stats",) in calls
    assert ("readiness",) in calls


def test_rtcm_stall_block_is_not_recorded_as_transport_failure() -> None:
    calls: list[tuple[object, ...]] = []
    panel = SimpleNamespace(
        gnss_base_status_labels={"rtcm_injection": object()},
        _record_gnss_base_rtcm_injection=lambda *args, **kwargs: calls.append(
            ("record", args, kwargs)
        ),
        _px4_mag_camera_label=lambda side: str(side),
        _set_health_label=lambda _label, text, level: calls.append(("label", text, level)),
        _refresh_gnss_base_rtcm_stats=lambda: calls.append(("stats",)),
        _refresh_gnss_operation_readiness=lambda: calls.append(("readiness",)),
        _log=lambda message: calls.append(("log", message)),
    )
    controller = GnssRuntimeController(panel)

    controller._apply_pixhawk_rtcm_result(
        {
            "payload_bytes": 16,
            "sent": (("right", 1),),
            "failed": (),
            "dropped": 0,
            "blocked_stall": 1,
            "blocked_targets": (
                {"side": "left", "category": "stall", "reason": "ALL_EXPECTED_RX_SILENT"},
            ),
        }
    )

    assert (
        "record",
        ("left",),
        {"payload_bytes": 16, "blocked_stall": True},
    ) in calls
    assert not any(
        call[0] == "record" and bool(call[2].get("failed"))
        for call in calls
        if len(call) == 3
    )
    assert any(call[0] == "label" and "차단" in str(call[1]) for call in calls)


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


class _FakeConnectSocket:
    """Minimal fake standing in for a raw ``socket.socket`` during connect()."""

    def __init__(self, response: bytes) -> None:
        self._response = bytearray(response)
        self.sent: list[bytes] = []

    def settimeout(self, _value: float) -> None:
        return None

    def sendall(self, data: bytes) -> None:
        self.sent.append(bytes(data))

    def recv(self, size: int) -> bytes:
        chunk = bytes(self._response[:size])
        del self._response[:size]
        return chunk

    def close(self) -> None:
        return None


def test_ntrip_client_connect_writes_initial_gga_in_the_same_transmission_as_the_request() -> None:
    """Regression: most NTRIP 1.0 casters ignore the ``Ntrip-GGA`` request
    header, and a caster that instead waits for an on-the-wire raw GGA to
    confirm a live rover position must never see a window with none. An
    earlier version of this method sent the request first and the raw GGA in
    a *second* ``sendall()`` only after the response arrived -- leaving a
    live, position-evidence-free connection for that round trip. This client
    now folds the raw GGA line into the exact same initial ``sendall()`` as
    the request, directly after its blank-line terminator, matching standard
    NTRIP 1.x client wire behaviour.
    """

    fake_socket = _FakeConnectSocket(b"HTTP/1.1 200 OK\r\n\r\ninitial-rtcm")
    client = NtripClient(
        NetworkConfig(caster_host="caster.example.org", mountpoint="MOUNT"),
        username="user",
        password="pass",
        socket_factory=lambda *args, **kwargs: fake_socket,
    )

    status_line, initial_payload = client.connect(gga_sentence="$GPGGA,fake*00")

    assert status_line == "HTTP/1.1 200 OK"
    assert initial_payload == b"initial-rtcm"
    # Exactly one sendall() call: request header block immediately followed
    # by the raw GGA line, not a separate post-handshake write.
    assert len(fake_socket.sent) == 1
    sent = fake_socket.sent[0]
    assert sent.startswith(b"GET /MOUNT HTTP/1.0\r\n")
    assert b"Ntrip-GGA: $GPGGA,fake*00\r\n" in sent
    assert sent.endswith(b"\r\n\r\n$GPGGA,fake*00\r\n")
    assert client.stats is not None
    assert client.stats.last_gga_monotonic_s is not None
    assert client.stats.gga_sent_count == 1


def test_ntrip_client_connect_without_gga_sends_only_the_request() -> None:
    """No GGA available yet (e.g. no fix) must not write a stray empty line."""

    fake_socket = _FakeConnectSocket(b"HTTP/1.1 200 OK\r\n\r\n")
    client = NtripClient(
        NetworkConfig(caster_host="caster.example.org", mountpoint="MOUNT"),
        username="user",
        password="pass",
        socket_factory=lambda *args, **kwargs: fake_socket,
    )

    client.connect(gga_sentence=None)

    assert len(fake_socket.sent) == 1
    assert fake_socket.sent[0].endswith(b"\r\n\r\n")
    assert client.stats.last_gga_monotonic_s is None
    assert client.stats.gga_sent_count == 0


def test_ntrip_client_reconnect_sends_initial_gga_again_on_the_new_connection() -> None:
    """Each fresh ``connect()`` -- including on reconnect -- must resend the
    initial GGA on its own new socket; it is per-connection wire state, not
    something the caster remembers across a dropped TCP session.
    """

    first_socket = _FakeConnectSocket(b"HTTP/1.1 200 OK\r\n\r\n")
    second_socket = _FakeConnectSocket(b"HTTP/1.1 200 OK\r\n\r\n")
    sockets = iter([first_socket, second_socket])
    config = NetworkConfig(caster_host="caster.example.org", mountpoint="MOUNT")

    first_client = NtripClient(
        config, username="user", password="pass", socket_factory=lambda *a, **k: next(sockets)
    )
    first_client.connect(gga_sentence="$GPGGA,one*00")
    first_client.close()

    second_client = NtripClient(
        config, username="user", password="pass", socket_factory=lambda *a, **k: next(sockets)
    )
    second_client.connect(gga_sentence="$GPGGA,two*00")

    assert first_socket.sent[0].endswith(b"$GPGGA,one*00\r\n")
    assert second_socket.sent[0].endswith(b"$GPGGA,two*00\r\n")
    assert second_client.stats.gga_sent_count == 1


def test_ntrip_client_connect_diagnostics_report_connected_and_initial_gga_events() -> None:
    # A trailing non-printable byte after the single CRLF is what tells
    # _ntrip1_status_line the legacy ICY-style header is actually complete
    # (real casters follow it with binary RTCM, e.g. the 0xD3 sync byte).
    fake_socket = _FakeConnectSocket(b"ICY 200 OK\r\n\xd3")
    events: list[str] = []
    client = NtripClient(
        NetworkConfig(caster_host="caster.example.org", mountpoint="MOUNT"),
        username="user",
        password="pass",
        socket_factory=lambda *args, **kwargs: fake_socket,
        diagnostics=events.append,
    )

    client.connect(gga_sentence="$GPGGA,fake*00")

    assert any(event.startswith("INITIAL GGA TX ok") for event in events)
    assert any(event.startswith("NTRIP CONNECTED status='ICY 200 OK'") for event in events)
    # Chronological order matters: the GGA is on the wire before the
    # handshake response is even parsed.
    assert events.index([e for e in events if e.startswith("INITIAL GGA TX")][0]) < events.index(
        [e for e in events if e.startswith("NTRIP CONNECTED")][0]
    )


def test_ntrip_client_connect_diagnostics_report_skip_reason_without_gga() -> None:
    fake_socket = _FakeConnectSocket(b"HTTP/1.1 200 OK\r\n\r\n")
    events: list[str] = []
    client = NtripClient(
        NetworkConfig(caster_host="caster.example.org", mountpoint="MOUNT"),
        username="user",
        password="pass",
        socket_factory=lambda *args, **kwargs: fake_socket,
        diagnostics=events.append,
    )

    client.connect(gga_sentence=None)

    assert any("INITIAL GGA TX skipped" in event for event in events)


def test_ntrip_client_diagnostics_callback_never_breaks_the_connect_path() -> None:
    """A raising diagnostics hook (e.g. a broken log widget) must not take
    down the network thread -- it is purely an observability side channel.
    """

    fake_socket = _FakeConnectSocket(b"HTTP/1.1 200 OK\r\n\r\n")

    def _broken(_message: str) -> None:
        raise RuntimeError("log widget exploded")

    client = NtripClient(
        NetworkConfig(caster_host="caster.example.org", mountpoint="MOUNT"),
        username="user",
        password="pass",
        socket_factory=lambda *args, **kwargs: fake_socket,
        diagnostics=_broken,
    )

    status_line, _payload = client.connect(gga_sentence="$GPGGA,fake*00")

    assert status_line == "HTTP/1.1 200 OK"


class _FakeStreamSocket:
    """Fake socket for iter_chunks(): scripted recv() results, tracked sends."""

    def __init__(self, recv_results: list[bytes]) -> None:
        self._results = list(recv_results)
        self.sent: list[bytes] = []

    def sendall(self, data: bytes) -> None:
        self.sent.append(bytes(data))

    def recv(self, _size: int) -> bytes:
        if not self._results:
            return b""
        return self._results.pop(0)


def _connected_client(*, gga_provider=None, diagnostics=None) -> NtripClient:
    from Workstation.workstation_core.gnss_network import NtripStreamStats

    client = NtripClient(
        NetworkConfig(caster_host="caster.example.org", mountpoint="MOUNT"),
        username="user",
        password="pass",
        diagnostics=diagnostics,
    )
    client._socket = _FakeStreamSocket([])  # noqa: SLF001 -- test seam, no public setter
    client._stats = NtripStreamStats(connected_at_monotonic_s=time.monotonic())
    return client


def test_iter_chunks_sends_periodic_gga_after_the_configured_interval(monkeypatch) -> None:
    import Workstation.workstation_core.gnss_network as gnss_network_module

    client = _connected_client()
    socket_obj = _FakeStreamSocket([])
    client._socket = socket_obj  # noqa: SLF001

    stop_event = threading.Event()

    def stop_after_one_chunk(_size: int) -> bytes:
        stop_event.set()
        return b"rtcm-chunk-1"

    socket_obj.recv = stop_after_one_chunk

    calls = {"count": 0}

    def provider() -> str:
        calls["count"] += 1
        return "$GPGGA,periodic*00"

    # iter_chunks() reads time.monotonic() exactly once for last_gga_s at
    # entry, then once per loop iteration for now_s. Scripting two calls
    # 2s apart makes the >= on_gga_interval_s check fire deterministically,
    # without a real sleep. (Must be set up after _connected_client(), which
    # also calls time.monotonic() once for its own unrelated setup.)
    times = iter([100.0, 102.0])
    monkeypatch.setattr(gnss_network_module.time, "monotonic", lambda: next(times))

    list(
        client.iter_chunks(
            stop_event,
            gga_provider=provider,
            on_gga_interval_s=1.0,
        )
    )

    assert calls["count"] == 1
    assert socket_obj.sent == [b"$GPGGA,periodic*00\r\n"]
    assert client.stats.gga_sent_count == 1


def test_iter_chunks_does_not_send_blank_line_when_provider_returns_none() -> None:
    stop_event = threading.Event()

    def provider() -> None:
        stop_event.set()
        return None

    client = _connected_client()
    socket_obj = _FakeStreamSocket([b""])
    client._socket = socket_obj  # noqa: SLF001

    with pytest.raises(NtripProtocolError):
        list(client.iter_chunks(stop_event, gga_provider=provider, on_gga_interval_s=0.0))

    assert socket_obj.sent == []


def test_iter_chunks_socket_eof_raises_with_full_disconnect_diagnostics() -> None:
    """The EOF exception message is what actually reaches the visible log
    (via the existing reconnecting/failed handlers), so every field the
    operator needs to diagnose *why* the caster closed the stream must be in
    this one message: connection duration, last GGA age, last RTCM age, GGA
    send count, and RTCM byte/chunk counts.
    """

    stop_event = threading.Event()
    client = _connected_client()
    socket_obj = _FakeStreamSocket([b"one-chunk", b""])
    client._socket = socket_obj  # noqa: SLF001

    with pytest.raises(NtripProtocolError) as excinfo:
        list(client.iter_chunks(stop_event, gga_provider=None))

    message = str(excinfo.value)
    assert message.startswith("NTRIP caster closed the stream")
    assert "connected_duration=" in message
    assert "last_gga_age=never" in message
    assert "last_rtcm_age=" in message
    assert "gga_sent=0" in message
    assert "chunks_received=1" in message
    assert "bytes_received=9" in message


def test_iter_chunks_socket_eof_after_stop_requested_does_not_raise() -> None:
    """An operator-requested Stop closing the socket must not be reported as
    a caster-side failure."""

    stop_event = threading.Event()
    client = _connected_client()
    socket_obj = _FakeStreamSocket([b""])
    client._socket = socket_obj  # noqa: SLF001
    stop_event.set()

    assert list(client.iter_chunks(stop_event, gga_provider=None)) == []


def test_iter_chunks_default_gga_interval_is_five_seconds() -> None:
    import inspect

    signature = inspect.signature(NtripClient.iter_chunks)
    assert signature.parameters["on_gga_interval_s"].default == 5.0


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
        def __init__(self, _config, *, username=None, password=None, diagnostics=None) -> None:
            self._stats = None
            self.username = username
            self.password = password

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
    stats_snapshots: list[object] = []
    chunks: list[object] = []
    envelopes: list[dict[str, object]] = []
    worker.connected.connect(lambda _status, value: connected.append(value))
    worker.stats_changed.connect(stats.append)
    worker.stats_snapshot_changed.connect(stats_snapshots.append)
    worker.chunk_received.connect(chunks.append)
    worker.chunk_envelope_received.connect(envelopes.append)

    worker.run()

    assert len(connected) == 1
    assert len(stats) == 1
    assert stats[0].chunks_received == 1
    assert stats[0].bytes_received == 128
    assert stats_snapshots == stats
    assert chunks == [b"rtcm-chunk"]
    assert len(envelopes) == 1
    assert envelopes[0]["payload"] == b"rtcm-chunk"
    assert int(envelopes[0]["receive_monotonic_ns"]) > 0
    assert int(envelopes[0]["receive_utc_ns"]) > 0


def test_worker_ui_signal_is_queued_to_the_gui_thread_without_text_document_warnings() -> None:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    qt_messages: list[str] = []

    class Emitter(QtCore.QObject):
        diagnostic = QtCore.pyqtSignal(str)

        @QtCore.pyqtSlot()
        def run(self) -> None:
            self.diagnostic.emit("NTRIP diagnostic")

    class Panel(QtWidgets.QWidget):
        def __init__(self) -> None:
            super().__init__()
            self.log = QtWidgets.QPlainTextEdit(self)
            self.callback_threads: list[QtCore.QThread] = []

        def delegated_log_handler(self, message: str) -> None:
            self.callback_threads.append(QtCore.QThread.currentThread())
            self.log.appendPlainText(message)

    previous_handler = QtCore.qInstallMessageHandler(
        lambda _kind, _context, message: qt_messages.append(str(message))
    )
    thread = QtCore.QThread()
    emitter = Emitter()
    panel = Panel()
    emitter.moveToThread(thread)
    _connect_gui_signal(emitter.diagnostic, panel.delegated_log_handler)
    thread.started.connect(emitter.run)
    thread.start()
    try:
        deadline = time.monotonic() + 2.0
        while not panel.callback_threads and time.monotonic() < deadline:
            app.processEvents()
            time.sleep(0.005)

        assert panel.callback_threads == [app.thread()]
        assert panel.log.toPlainText() == "NTRIP diagnostic"
        assert not any("QTextBlock" in message or "QTextCursor" in message for message in qt_messages)
    finally:
        thread.quit()
        assert thread.wait(1000)
        panel.deleteLater()
        app.processEvents()
        QtCore.qInstallMessageHandler(previous_handler)


def test_panel_log_called_from_worker_is_requeued_before_touching_qt_text(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path,
) -> None:
    from Workstation import standalone_gui

    monkeypatch.setattr(
        standalone_gui.WirisStereoDistancePanel,
        "_initial_status",
        lambda _self: None,
    )
    monkeypatch.setattr(
        standalone_gui.WirisStereoDistancePanel,
        "_auto_load_intrinsics",
        lambda _self: None,
    )
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    panel = standalone_gui.WirisStereoDistancePanel(
        app_root=tmp_path,
        settings_path=tmp_path / "settings.json",
    )
    panel.wiris_log.clear()
    callback_threads: list[QtCore.QThread] = []
    qt_messages: list[str] = []

    class Observer(QtCore.QObject):
        @QtCore.pyqtSlot()
        def changed(self) -> None:
            callback_threads.append(QtCore.QThread.currentThread())

    class Emitter(QtCore.QObject):
        finished = QtCore.pyqtSignal()

        @QtCore.pyqtSlot()
        def run(self) -> None:
            panel._log("[GNSS][NTRIP] worker-thread log")
            self.finished.emit()

    observer = Observer()
    emitter = Emitter()
    thread = QtCore.QThread()
    panel.wiris_log.textChanged.connect(observer.changed)
    emitter.moveToThread(thread)
    thread.started.connect(emitter.run)
    emitter.finished.connect(thread.quit)
    previous_handler = QtCore.qInstallMessageHandler(
        lambda _kind, _context, message: qt_messages.append(str(message))
    )
    thread.start()
    try:
        deadline = time.monotonic() + 2.0
        while (
            panel.wiris_log.toPlainText() != "[GNSS][NTRIP] worker-thread log"
            and time.monotonic() < deadline
        ):
            app.processEvents()
            time.sleep(0.005)

        assert panel.wiris_log.toPlainText() == "[GNSS][NTRIP] worker-thread log"
        assert callback_threads == [app.thread()]
        assert not any(
            "QTextBlock" in message or "QTextCursor" in message
            for message in qt_messages
        )
    finally:
        thread.quit()
        assert thread.wait(1000)
        QtCore.qInstallMessageHandler(previous_handler)
        panel._save_current_settings = lambda: None
        panel.close()
        panel.deleteLater()
        app.processEvents()


# --- Direct username/password UI: credential plumbing, storage, and UI ----


def test_ntrip_worker_passes_explicit_credentials_to_client_and_reuses_them_on_reconnect(monkeypatch) -> None:
    """Required test 1 + 8: explicit credentials reach NtripClient, including on retry."""

    import Workstation.gnss_runtime_workers as runtime_workers

    seen_credentials: list[tuple[object, object]] = []

    class FakeClient:
        def __init__(self, _config, *, username=None, password=None, diagnostics=None) -> None:
            self._stats = None
            self._attempt = len(seen_credentials)
            seen_credentials.append((username, password))

        @property
        def stats(self):
            return self._stats

        def connect(self, *, gga_sentence=None):
            if self._attempt == 0:
                raise RuntimeError("simulated transient failure")
            self._stats = SimpleNamespace(
                connected_at_monotonic_s=10.0,
                bytes_received=0,
                chunks_received=0,
                last_chunk_monotonic_s=None,
                last_gga_monotonic_s=None,
            )
            return "ICY 200 OK", b""

        def iter_chunks(self, stop_event, **_kwargs):
            stop_event.set()
            return iter(())

        def close(self):
            return None

    class ImmediateWaiter:
        def __init__(self, _config) -> None:
            pass

        def wait_for_reconnect(self, stop_event, **_kwargs):
            return not stop_event.is_set()

    def fake_client_factory(config, *, username=None, password=None, diagnostics=None):
        if username is None and password is None:
            return ImmediateWaiter(config)
        return FakeClient(config, username=username, password=password)

    monkeypatch.setattr(runtime_workers, "NtripClient", fake_client_factory)
    worker = runtime_workers.NtripNetworkWorker(
        config=NetworkConfig(), username="field-user", password="field-secret"
    )
    reconnecting: list[str] = []
    connected: list[object] = []
    worker.reconnecting.connect(reconnecting.append)
    worker.connected.connect(lambda status, _stats: connected.append(status))

    worker.run()

    assert reconnecting, "the first attempt should have failed and reconnected"
    assert connected == ["ICY 200 OK"]
    credential_attempts = [pair for pair in seen_credentials if pair != (None, None)]
    assert len(credential_attempts) == 2
    assert credential_attempts[0] == ("field-user", "field-secret")
    assert credential_attempts[1] == ("field-user", "field-secret")


def test_network_config_carries_plain_username_and_never_a_password_field() -> None:
    """Required test 2 (structural half): NetworkConfig has nowhere to put a password."""

    config = NetworkConfig(username="field-user")
    mapping = config.to_mapping()

    assert mapping["username"] == "field-user"
    assert "password" not in mapping
    assert NetworkConfig.from_mapping(mapping).username == "field-user"


def test_saved_settings_json_never_contains_a_ntrip_password(tmp_path) -> None:
    """Required test 2: a full settings round trip never writes a password."""

    settings = AppSettings(
        gnss_operation=GnssOperationConfig(
            network=NetworkConfig(
                caster_host="caster.example.org",
                mountpoint="VRS",
                username="field-user",
                remember_password=True,
            )
        )
    )
    path = tmp_path / "settings.json"
    save_settings(settings, path)

    raw_text = path.read_text(encoding="utf-8")
    payload = json.loads(raw_text)

    assert payload["gnss_operation"]["network"]["username"] == "field-user"
    assert "password" not in payload["gnss_operation"]["network"]
    assert "secret" not in raw_text.lower()
    reloaded = load_settings(path)
    assert reloaded.gnss_operation.network.username == "field-user"
    assert reloaded.gnss_operation.network.remember_password is True


class _FakeKeyringBackend:
    """In-memory stand-in for a real OS credential store."""

    def __init__(self) -> None:
        self.store: dict[tuple[str, str], str] = {}

    def get_keyring(self):
        return self

    def set_password(self, service: str, username: str, password: str) -> None:
        self.store[(service, username)] = password

    def get_password(self, service: str, username: str) -> str | None:
        return self.store.get((service, username))

    def delete_password(self, service: str, username: str) -> None:
        key = (service, username)
        if key not in self.store:
            raise ntrip_credentials.PasswordDeleteError("not found")
        del self.store[key]


def test_ntrip_credentials_keyring_save_load_delete_roundtrip(monkeypatch) -> None:
    """Required test 3: save/load/delete against the OS credential store."""

    backend = _FakeKeyringBackend()
    monkeypatch.setattr(ntrip_credentials, "keyring", backend)

    assert ntrip_credentials.secure_storage_available() is True
    assert ntrip_credentials.credential_status("caster", "MOUNT", "field-user") == (
        ntrip_credentials.CredentialStorageStatus.NOT_SAVED
    )

    assert ntrip_credentials.save_password("caster", "MOUNT", "field-user", "field-secret") is True
    assert ntrip_credentials.load_password("caster", "MOUNT", "field-user") == "field-secret"
    assert ntrip_credentials.credential_status("caster", "MOUNT", "field-user") == (
        ntrip_credentials.CredentialStorageStatus.SAVED
    )

    # A different mountpoint/username must not see this caster's credential.
    assert ntrip_credentials.load_password("caster", "OTHER-MOUNT", "field-user") is None
    assert ntrip_credentials.load_password("caster", "MOUNT", "other-user") is None

    assert ntrip_credentials.delete_password("caster", "MOUNT", "field-user") is True
    assert ntrip_credentials.load_password("caster", "MOUNT", "field-user") is None
    # Deleting an already-absent entry is idempotent, not an error.
    assert ntrip_credentials.delete_password("caster", "MOUNT", "field-user") is True


def test_ntrip_credentials_report_unavailable_without_a_usable_backend(monkeypatch) -> None:
    monkeypatch.setattr(ntrip_credentials, "keyring", None)

    assert ntrip_credentials.secure_storage_available() is False
    assert ntrip_credentials.save_password("caster", "MOUNT", "field-user", "secret") is False
    assert ntrip_credentials.load_password("caster", "MOUNT", "field-user") is None
    assert ntrip_credentials.credential_status("caster", "MOUNT", "field-user") == (
        ntrip_credentials.CredentialStorageStatus.UNAVAILABLE
    )


def test_resolve_ntrip_credentials_remember_off_never_persists_session_password(monkeypatch) -> None:
    """Required test 4: Remember=OFF keeps a typed password to this process only."""

    import Workstation.gnss_runtime_controller as gnss_runtime_controller

    saved: list[tuple[str, str, str, str]] = []
    monkeypatch.setattr(
        gnss_runtime_controller.ntrip_credentials,
        "save_password",
        lambda host, mount, user, password: saved.append((host, mount, user, password)),
    )

    controller = GnssRuntimeController(SimpleNamespace())
    session_credential = NtripCredentialResolution(username="field-user", password="field-secret", source="session")

    controller._maybe_persist_ntrip_credential(
        NetworkConfig(caster_host="caster", mountpoint="MOUNT", remember_password=False),
        session_credential,
    )
    assert saved == []

    controller._maybe_persist_ntrip_credential(
        NetworkConfig(caster_host="caster", mountpoint="MOUNT", remember_password=True),
        session_credential,
    )
    assert saved == [("caster", "MOUNT", "field-user", "field-secret")]

    # A credential that was *not* freshly typed this session (already came
    # from the keyring or a legacy env var) is never re-saved.
    saved.clear()
    keyring_credential = NtripCredentialResolution(username="field-user", password="field-secret", source="keyring")
    controller._maybe_persist_ntrip_credential(
        NetworkConfig(caster_host="caster", mountpoint="MOUNT", remember_password=True),
        keyring_credential,
    )
    assert saved == []


def _panel_with_password(text: str) -> SimpleNamespace:
    return SimpleNamespace(gnss_ntrip_password_edit=QtWidgets.QLineEdit(text))


def test_resolve_ntrip_credentials_prefers_session_password(monkeypatch) -> None:
    controller = GnssRuntimeController(_panel_with_password("typed-secret"))
    network = NetworkConfig(caster_host="caster", mountpoint="MOUNT", username="field-user")

    resolved = controller._resolve_ntrip_credentials(network)

    assert resolved == NtripCredentialResolution(username="field-user", password="typed-secret", source="session")


def test_resolve_ntrip_credentials_prefers_keyring_over_unedited_deployment_default(
    monkeypatch,
) -> None:
    """A masked deployment default must not shadow a saved account secret.

    Regression: both password widgets are pre-populated for the field profile,
    so treating every non-empty widget as an explicit session edit made the
    keyring branch unreachable after restart and could produce caster 401s.
    """

    panel = _panel_with_password(DEFAULT_NTRIP_SESSION_PASSWORD)
    panel._gnss_ntrip_session_password_explicit = False
    controller = GnssRuntimeController(panel)
    monkeypatch.setattr(ntrip_credentials, "secure_storage_available", lambda: True)
    monkeypatch.setattr(
        ntrip_credentials,
        "load_password",
        lambda _host, _mountpoint, _username: "stored-account-secret",
    )
    network = NetworkConfig(
        caster_host="RTS2.ngii.go.kr",
        mountpoint="VRS-RTCM32",
        username=DEFAULT_NTRIP_USERNAME,
    )

    resolved = controller._resolve_ntrip_credentials(network)

    assert resolved == NtripCredentialResolution(
        username=DEFAULT_NTRIP_USERNAME,
        password="stored-account-secret",
        source="keyring",
    )


def test_resolve_ntrip_credentials_uses_explicit_session_edit_over_keyring(
    monkeypatch,
) -> None:
    panel = _panel_with_password("rotated-session-secret")
    panel._gnss_ntrip_session_password_explicit = True
    controller = GnssRuntimeController(panel)
    monkeypatch.setattr(ntrip_credentials, "secure_storage_available", lambda: True)
    monkeypatch.setattr(
        ntrip_credentials,
        "load_password",
        lambda _host, _mountpoint, _username: "old-stored-secret",
    )

    resolved = controller._resolve_ntrip_credentials(
        NetworkConfig(caster_host="caster", mountpoint="MOUNT", username="field-user")
    )

    assert resolved == NtripCredentialResolution(
        username="field-user",
        password="rotated-session-secret",
        source="session",
    )


def test_set_gnss_ntrip_session_password_bridges_settings_workspace_field_into_resolve() -> None:
    """Regression: the Settings workspace's password field lives on a
    different object than ``panel`` (SettingsWorkspace.gnss_network_password_edit
    vs panel.gnss_ntrip_password_edit). settings_controller.py used to try
    ``getattr(panel, "gnss_network_password_edit", None)``, which is always
    ``None`` since that attribute was never on ``panel`` -- so typing a
    password in Settings and clicking Apply never reached the field
    ``_resolve_ntrip_credentials`` reads, and Start NTRIP kept reporting "no
    saved password" no matter what was typed. main_window.py now calls this
    method directly with the live text, where both objects are in scope.
    """

    controller = GnssRuntimeController(_panel_with_password(""))

    controller.set_gnss_ntrip_session_password("typed-in-settings-workspace")

    network = NetworkConfig(caster_host="caster", mountpoint="MOUNT", username="field-user")
    resolved = controller._resolve_ntrip_credentials(network)

    assert resolved == NtripCredentialResolution(
        username="field-user", password="typed-in-settings-workspace", source="session"
    )


def test_ntrip_route_preparation_allows_transport_without_pixhawk() -> None:
    health_calls: list[tuple[object, str, str]] = []
    logs: list[str] = []
    panel = SimpleNamespace(
        mavlink_pixhawk=None,
        gnss_base_status_labels={"network": "network-label"},
        _set_health_label=lambda label, text, tone: health_calls.append((label, text, tone)),
        _log=lambda text: logs.append(text),
    )
    controller = GnssRuntimeController(panel)
    controller._ensure_pixhawk_rtcm_worker = lambda: pytest.fail("transport-only start must not require a worker")

    assert controller._prepare_ntrip_rtcm_route(RtcmRouteConfig(ntrip_targets=("left", "right"))) is True
    assert health_calls == [
        ("network-label", "NTRIP transport 시작 | MAVLink RTCM route 대기 중", "warn")
    ]
    assert logs == ["[GNSS] NTRIP transport 시작 | MAVLink RTCM route 대기 중"]


def test_ntrip_route_preparation_keeps_pixhawk_worker_failure_blocking() -> None:
    health_calls: list[tuple[object, str, str]] = []
    logs: list[str] = []
    panel = SimpleNamespace(
        mavlink_pixhawk=object(),
        gnss_base_status_labels={"network": "network-label"},
        _set_health_label=lambda label, text, tone: health_calls.append((label, text, tone)),
        _log=lambda text: logs.append(text),
    )
    controller = GnssRuntimeController(panel)
    controller._ensure_pixhawk_rtcm_worker = lambda: None

    assert controller._prepare_ntrip_rtcm_route(RtcmRouteConfig(ntrip_targets=("left",))) is False
    assert health_calls == [("network-label", "RTCM 주입 worker를 시작할 수 없습니다", "bad")]
    assert logs == ["[GNSS] NTRIP 시작 거부: RTCM 주입 worker를 시작할 수 없습니다"]


def test_resolve_ntrip_credentials_falls_back_to_keyring(monkeypatch) -> None:
    import Workstation.gnss_runtime_controller as gnss_runtime_controller

    backend = _FakeKeyringBackend()
    monkeypatch.setattr(ntrip_credentials, "keyring", backend)
    ntrip_credentials.save_password("caster", "MOUNT", "field-user", "stored-secret")

    controller = GnssRuntimeController(_panel_with_password(""))
    network = NetworkConfig(caster_host="caster", mountpoint="MOUNT", username="field-user")

    resolved = controller._resolve_ntrip_credentials(network)

    assert resolved == NtripCredentialResolution(username="field-user", password="stored-secret", source="keyring")


def test_resolve_ntrip_credentials_falls_back_to_legacy_env_vars(monkeypatch) -> None:
    """Required test 6: the pre-existing username_env/password_env path keeps working."""

    monkeypatch.setattr(ntrip_credentials, "keyring", None)
    monkeypatch.setenv("EXAMPLE_LEGACY_USER", "legacy-user")
    monkeypatch.setenv("EXAMPLE_LEGACY_PASS", "legacy-secret")

    controller = GnssRuntimeController(_panel_with_password(""))
    network = NetworkConfig(
        caster_host="caster",
        mountpoint="MOUNT",
        username="",
        username_env="EXAMPLE_LEGACY_USER",
        password_env="EXAMPLE_LEGACY_PASS",
    )

    resolved = controller._resolve_ntrip_credentials(network)

    assert resolved == NtripCredentialResolution(username="legacy-user", password="legacy-secret", source="legacy_env")


def test_resolve_ntrip_credentials_fails_clearly_with_no_credential_anywhere(monkeypatch) -> None:
    """Required test 5: no session/keyring/legacy credential -> a specific, clear failure."""

    monkeypatch.setattr(ntrip_credentials, "keyring", None)
    monkeypatch.delenv("NTRIP_USERNAME", raising=False)
    monkeypatch.delenv("NTRIP_PASSWORD", raising=False)

    controller = GnssRuntimeController(_panel_with_password(""))
    network = NetworkConfig(caster_host="caster", mountpoint="MOUNT", username="field-user")

    with pytest.raises(NtripCredentialError) as excinfo:
        controller._resolve_ntrip_credentials(network)
    assert "password" in str(excinfo.value).lower()
    # The failure never guesses which credential (if any) is wrong.
    assert "field-user" not in str(excinfo.value)


def test_resolve_ntrip_credentials_requires_a_username() -> None:
    controller = GnssRuntimeController(_panel_with_password(""))
    network = NetworkConfig(caster_host="caster", mountpoint="MOUNT", username="", username_env="UNSET_ENV_USER", password_env="UNSET_ENV_PASS")

    with pytest.raises(NtripCredentialError) as excinfo:
        controller._resolve_ntrip_credentials(network)
    assert "username" in str(excinfo.value).lower()


def test_resolved_credential_password_never_appears_in_ntrip_request_bytes() -> None:
    """Required test 7: the Authorization header is built without leaking the raw password."""

    controller = GnssRuntimeController(_panel_with_password("field-secret"))
    network = NetworkConfig(caster_host="caster.example.org", mountpoint="VRS", username="field-user")

    resolved = controller._resolve_ntrip_credentials(network)
    request = build_ntrip_request(network, username=resolved.username, password=resolved.password)

    assert b"Authorization: Basic " in request
    assert b"field-secret" not in request
    assert resolved.password == "field-secret"  # sanity: the credential itself was resolved correctly


def test_settings_workspace_ntrip_defaults_are_masked_and_not_in_settings_payload() -> None:
    """Untouched deployment defaults are usable without becoming persisted settings."""

    QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    workspace = SettingsWorkspace(backend=object())
    try:
        assert workspace.gnss_network_password_edit.echoMode() == QtWidgets.QLineEdit.Password
        assert workspace.settings_widgets["gnss_network_username"].text() == DEFAULT_NTRIP_USERNAME
        assert workspace.gnss_network_password_edit.text() == DEFAULT_NTRIP_SESSION_PASSWORD
        # The password widget must never be reachable through the generic
        # settings payload path that gets serialized to disk.
        assert "gnss_network_password" not in workspace.settings_widgets
        payload = workspace.settings_payload()
        assert payload["gnss_operation"]["network"]["username"] == DEFAULT_NTRIP_USERNAME
        assert "gnss_network_password" not in payload
        assert "password" not in payload["gnss_operation"]["network"]
    finally:
        workspace.deleteLater()


def test_empty_ntrip_password_requires_operator_input(monkeypatch) -> None:
    monkeypatch.setattr(ntrip_credentials, "secure_storage_available", lambda: False)
    monkeypatch.delenv("NTRIP_USERNAME", raising=False)
    monkeypatch.delenv("NTRIP_PASSWORD", raising=False)
    panel = _panel_with_password(DEFAULT_NTRIP_SESSION_PASSWORD)
    controller = GnssRuntimeController(panel)
    network = NetworkConfig(caster_host="caster", mountpoint="MOUNT")

    with pytest.raises(NtripCredentialError):
        controller._resolve_ntrip_credentials(network)

    panel.gnss_ntrip_password_edit.setText("operator-password")
    overridden = NetworkConfig(
        caster_host="caster",
        mountpoint="MOUNT",
        username="operator-user",
    )
    assert controller._resolve_ntrip_credentials(overridden) == NtripCredentialResolution(
        username="operator-user",
        password="operator-password",
        source="session",
    )


def test_runtime_owner_requires_password_before_settings_apply(
    monkeypatch, tmp_path
) -> None:
    """Direct Start NTRIP never obtains a password from a plaintext default."""

    from Workstation import standalone_gui

    monkeypatch.setattr(
        standalone_gui.WirisStereoDistancePanel,
        "_initial_status",
        lambda _self: None,
    )
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    panel = standalone_gui.WirisStereoDistancePanel(
        app_root=tmp_path,
        settings_path=tmp_path / "settings.json",
    )
    try:
        monkeypatch.delenv("NGII_RTS2_USERNAME", raising=False)
        monkeypatch.delenv("NGII_RTS2_PASSWORD", raising=False)
        monkeypatch.setattr(ntrip_credentials, "secure_storage_available", lambda: False)
        assert panel.gnss_ntrip_username_edit.text() == DEFAULT_NTRIP_USERNAME
        assert panel.gnss_ntrip_password_edit.text() == DEFAULT_NTRIP_SESSION_PASSWORD
        with pytest.raises(NtripCredentialError):
            panel._resolve_ntrip_credentials(
                panel._build_gnss_operation_config().network
            )
        panel.gnss_ntrip_password_edit.setText("operator-password")
        assert panel._resolve_ntrip_credentials(
            panel._build_gnss_operation_config().network
        ) == NtripCredentialResolution(
            username=DEFAULT_NTRIP_USERNAME,
            password="operator-password",
            source="session",
        )
    finally:
        panel._save_current_settings = lambda: None
        panel.close()
        panel.deleteLater()
        app.processEvents()


def test_network_reconnect_after_ready_does_not_attempt_invalid_state_transition() -> None:
    """Regression: NtripNetworkWorker reconnects transparently on any
    transient stream drop (caster closes the connection, network blip) and
    re-emits ``connected`` every time -- including well after the operation
    state machine already progressed past NTRIP_ACTIVE to READY/MEASURING.
    Only INITIALIZING/NTRIP_CONNECTING can legally reach NTRIP_ACTIVE;
    retrying it from READY used to log a spurious "상태 전환 실패: invalid
    GNSS state transition" fault on every ordinary reconnect, even though
    RTCM kept flowing the whole time (chunk routing never depended on this
    state machine).
    """

    transition_calls: list[tuple[object, str]] = []
    health_calls: list[tuple[str, str, str]] = []

    panel = SimpleNamespace(
        gnss_network_connected=False,
        gnss_network_phase="",
        gnss_network_stats=None,
        gnss_network_last_error="",
        gnss_network_status_line="",
        gnss_operation_manager=SimpleNamespace(state=GnssOperationState.READY),
        gnss_base_status_labels={"network": "network-label"},
        _gnss_transition=lambda state, reason: transition_calls.append((state, reason)),
        _set_health_label=lambda label, text, tone: health_calls.append((label, text, tone)),
    )
    controller = GnssRuntimeController(panel)

    controller._handle_gnss_network_connected("ICY 200 OK", stats=object())

    assert transition_calls == []
    assert panel.gnss_network_connected is True
    assert health_calls == [("network-label", "ICY 200 OK | RTCM 대기", "good")]


def test_network_connect_from_ntrip_connecting_still_transitions_to_active() -> None:
    """The legitimate first-connect path must keep transitioning normally."""

    transition_calls: list[tuple[object, str]] = []

    panel = SimpleNamespace(
        gnss_network_connected=False,
        gnss_network_phase="",
        gnss_network_stats=None,
        gnss_network_last_error="",
        gnss_network_status_line="",
        gnss_operation_manager=SimpleNamespace(state=GnssOperationState.NTRIP_CONNECTING),
        gnss_base_status_labels={"network": "network-label"},
        _gnss_transition=lambda state, reason: transition_calls.append((state, reason)),
        _set_health_label=lambda *args: None,
    )
    controller = GnssRuntimeController(panel)

    controller._handle_gnss_network_connected("ICY 200 OK", stats=object())

    assert transition_calls == [(GnssOperationState.NTRIP_ACTIVE, "NTRIP active: ICY 200 OK")]


def test_ntrip_chunk_ui_status_is_coalesced_without_dropping_forwarding() -> None:
    class Worker:
        def __init__(self) -> None:
            self.sent: list[tuple[str, bytes]] = []

        def send_bytes(self, label: str, payload: bytes) -> None:
            self.sent.append((label, payload))

    health_calls: list[tuple[object, str, str]] = []
    worker = Worker()
    config = SimpleNamespace(operation_mode=GnssOperationMode.NETWORK_ASSISTED_FIXED_BASE)
    panel = SimpleNamespace(
        gnss_network_rtcm_parser=SimpleNamespace(feed=lambda _payload: []),
        _gnss_active_operation_config=lambda: config,
        gnss_base_serial_worker=worker,
        gnss_base_serial_connected=True,
        gnss_base_status_labels={"network": object()},
        _set_health_label=lambda label, text, tone: health_calls.append((label, text, tone)),
        drone_telemetry_rtcm_source="base",
        drone_telemetry_rtcm_forward=False,
    )
    controller = GnssRuntimeController(panel)

    controller._handle_gnss_network_chunk(b"first")
    controller._handle_gnss_network_chunk(b"second")

    assert len(worker.sent) == 2
    assert len(health_calls) == 1


def test_handle_gnss_network_diagnostic_forwards_to_the_visible_log() -> None:
    """NtripClient's connect/GGA-TX events have no status of their own and no
    other signal to ride on -- without this handler they never reach the log.
    """

    logged: list[str] = []
    panel = SimpleNamespace(_log=lambda text: logged.append(text))
    controller = GnssRuntimeController(panel)

    controller._handle_gnss_network_diagnostic("NTRIP CONNECTED status='HTTP/1.1 200 OK' at=123.456")

    assert logged == ["[GNSS][NTRIP] NTRIP CONNECTED status='HTTP/1.1 200 OK' at=123.456"]


def _gga_panel(
    *,
    gga_policy: GgaPolicy,
    log_sink: list[str],
    latest_gps: dict | None = None,
    latest_gnss_base_position=None,
) -> SimpleNamespace:
    config = GnssOperationConfig(network=NetworkConfig(caster_host="caster", mountpoint="MOUNT", gga_policy=gga_policy))
    return SimpleNamespace(
        _gnss_active_operation_config=lambda: config,
        latest_gps=latest_gps or {},
        latest_gnss_base_position=latest_gnss_base_position,
        _log=log_sink.append,
    )


def test_gga_provider_logs_specific_reason_for_manual_policy() -> None:
    logged: list[str] = []
    panel = _gga_panel(gga_policy=GgaPolicy.MANUAL, log_sink=logged)
    controller = GnssRuntimeController(panel)

    result = controller._gnss_gga_provider()

    assert result is None
    assert any("MANUAL" in message for message in logged)


def test_gga_provider_logs_specific_reason_for_missing_rover_positions() -> None:
    logged: list[str] = []
    panel = _gga_panel(gga_policy=GgaPolicy.SYSTEM_CENTER, log_sink=logged, latest_gps={})
    controller = GnssRuntimeController(panel)

    result = controller._gnss_gga_provider()

    assert result is None
    assert any(
        "no left/right GNSS position available" in message and "left=no message received" in message
        for message in logged
    )


def test_gga_provider_throttles_repeated_missing_position_logs() -> None:
    logged: list[str] = []
    panel = _gga_panel(gga_policy=GgaPolicy.SYSTEM_CENTER, log_sink=logged, latest_gps={})
    controller = GnssRuntimeController(panel)

    assert controller._gnss_gga_provider() is None
    first_messages = list(logged)
    assert controller._gnss_gga_provider() is None

    assert logged == first_messages


def test_camera_state_log_throttles_identical_liveness_updates() -> None:
    from Workstation import standalone_gui

    logged: list[str] = []
    panel = SimpleNamespace(
        _last_logged_camera_states={},
        _extract_active_decoder_from_state=lambda _state: None,
        _set_active_stream_decoder=lambda _camera, _decoder: None,
        _camera_name_ko=lambda camera: camera,
        _log=logged.append,
    )

    standalone_gui.WirisStereoDistancePanel._log_state(
        panel,
        "left",
        "카메라 스트림 실행 중",
    )
    standalone_gui.WirisStereoDistancePanel._log_state(
        panel,
        "left",
        "카메라 스트림 실행 중",
    )

    assert logged == ["[카메라] left 스트림 상태: 카메라 스트림 실행 중"]


def test_gga_provider_logs_stale_base_sample_age_for_base_gnss_policy() -> None:
    logged: list[str] = []
    stale_sample = SimpleNamespace(timestamp_s=time.monotonic() - 30.0)
    panel = _gga_panel(
        gga_policy=GgaPolicy.BASE_GNSS, log_sink=logged, latest_gnss_base_position=stale_sample
    )
    controller = GnssRuntimeController(panel)

    result = controller._gnss_gga_provider()

    assert result is None
    assert any("stale" in message and "age=" in message for message in logged)


def test_gga_provider_system_center_success_does_not_log_anything() -> None:
    logged: list[str] = []
    now_s = time.monotonic()
    left = SimpleNamespace(
        latitude_deg=37.0,
        longitude_deg=127.0,
        altitude_m=50.0,
        has_position=True,
        timestamp_s=now_s - 0.02,
    )
    right = SimpleNamespace(
        latitude_deg=37.0001,
        longitude_deg=127.0001,
        altitude_m=51.0,
        has_position=True,
        timestamp_s=now_s - 0.01,
    )
    panel = _gga_panel(
        gga_policy=GgaPolicy.SYSTEM_CENTER, log_sink=logged, latest_gps={"left": left, "right": right}
    )
    controller = GnssRuntimeController(panel)

    result = controller._gnss_gga_provider()

    assert result is not None
    assert result.startswith("$GPGGA,")
    assert logged == []


def test_gga_provider_records_skip_reason_and_clears_it_on_success() -> None:
    """A VRS caster withholds RTCM without periodic GGA while the transport
    still reports "connected" -- _gnss_last_gga_skip_reason is the only
    signal the NTRIP status panel has for that failure mode.
    """

    logged: list[str] = []
    now_s = time.monotonic()
    panel = _gga_panel(gga_policy=GgaPolicy.SYSTEM_CENTER, log_sink=logged, latest_gps={})
    controller = GnssRuntimeController(panel)

    assert controller._gnss_gga_provider() is None
    assert "no left/right GNSS position available" in panel._gnss_last_gga_skip_reason
    assert panel._gnss_last_gga_skip_monotonic_s is not None

    left = SimpleNamespace(
        latitude_deg=37.0,
        longitude_deg=127.0,
        altitude_m=50.0,
        has_position=True,
        timestamp_s=now_s - 0.02,
    )
    right = SimpleNamespace(
        latitude_deg=37.0001,
        longitude_deg=127.0001,
        altitude_m=51.0,
        has_position=True,
        timestamp_s=now_s - 0.01,
    )
    panel.latest_gps = {"left": left, "right": right}

    result = controller._gnss_gga_provider()

    assert result is not None
    assert panel._gnss_last_gga_skip_reason == ""
    assert panel._gnss_last_gga_skip_monotonic_s is None


def test_gga_provider_uses_the_operator_selected_gps2_source() -> None:
    logged: list[str] = []
    now_s = time.monotonic()
    gps1 = SimpleNamespace(
        latitude_deg=35.0,
        longitude_deg=125.0,
        altitude_m=40.0,
        has_position=True,
        timestamp_s=now_s,
        fix_type=3,
        satellites_used=8,
        eph_m=1.0,
    )
    gps2 = SimpleNamespace(
        latitude_deg=37.0,
        longitude_deg=127.0,
        altitude_m=50.0,
        has_position=True,
        timestamp_s=now_s,
        fix_type=6,
        satellites_used=18,
        eph_m=0.02,
    )
    panel = _gga_panel(
        gga_policy=GgaPolicy.LEFT_GNSS,
        log_sink=logged,
        latest_gps={"left": gps1, "left_GPS2": gps2},
    )
    panel._selected_baseline_gps_state = lambda side: panel.latest_gps.get(f"{side}_GPS2")

    result = GnssRuntimeController(panel)._gnss_gga_provider()

    assert result == gga_from_position(gps2)
    assert logged == []


def test_gnss_readiness_uses_the_operator_selected_gps2_source(monkeypatch) -> None:
    from Workstation import gnss_runtime_controller

    now_s = time.monotonic()
    gps1 = SimpleNamespace(
        fix_type=3,
        timestamp_s=now_s,
        eph_m=1.0,
        epv_m=2.0,
        topic="mavlink/GPS_RAW_INT",
    )
    gps2 = SimpleNamespace(
        fix_type=6,
        timestamp_s=now_s,
        eph_m=0.02,
        epv_m=0.03,
        topic="mavlink/GPS2_RAW",
    )
    captured = []
    config = GnssOperationConfig(operation_mode=GnssOperationMode.NETWORK_RTK_DIRECT)
    manager = SimpleNamespace(
        config=config,
        state=GnssOperationState.MEASURING,
        source_identity=None,
        update_readiness=lambda snapshot: None,
        invalidate=lambda reason: None,
    )
    panel = SimpleNamespace(
        gnss_operation_gate_active=True,
        gnss_operation_manager=manager,
        gnss_active_operation_config=None,
        _build_gnss_operation_config=lambda: config,
        _gnss_base_rtcm_target_sides=lambda: (),
        gnss_base_rtcm_stats={},
        mavlink_pixhawk=None,
        latest_gps={"left": gps1, "left_GPS2": gps2},
        _selected_baseline_gps_state=lambda side: gps2 if side == "left" else None,
        gnss_rover_fixed_since={"left": None, "right": None, "drone": None},
        latest_baseline_vector=None,
        latest_baseline_quality=None,
    )

    def capture_readiness(_config, inputs):
        captured.append(inputs)
        return SimpleNamespace(reasons=(), warnings=(), reason_messages={})

    monkeypatch.setattr(gnss_runtime_controller, "evaluate_gnss_readiness", capture_readiness)

    GnssRuntimeController(panel)._refresh_gnss_operation_readiness()

    selected = captured[0].rover_samples["left"]
    assert selected.fix_type == 6
    assert selected.device_id == "mavlink/GPS2_RAW"


def test_gnss_readiness_marks_stalled_rtcm_target_endpoint_missing_and_faulted(monkeypatch) -> None:
    from Workstation import gnss_runtime_controller

    now_s = time.monotonic()
    config = GnssOperationConfig(
        operation_mode=GnssOperationMode.NETWORK_RTK_DIRECT,
        routing=RoutingConfig(left_enabled=True, right_enabled=False),
    )
    manager = SimpleNamespace(
        config=config,
        state=GnssOperationState.MEASURING,
        source_identity=None,
        update_readiness=lambda _snapshot: None,
        invalidate=lambda _reason: None,
    )
    stalled_health = SimpleNamespace(
        identity_verified=True,
        rtcm_eligible=False,
        state=SimpleNamespace(value="ETH MAVLINK RX STALLED"),
        stall_reason="ALL_EXPECTED_RX_SILENT",
        observed_system_id=1,
        connection_generation=4,
    )
    captured = []
    panel = SimpleNamespace(
        gnss_operation_gate_active=True,
        gnss_operation_manager=manager,
        gnss_active_operation_config=None,
        _build_gnss_operation_config=lambda: config,
        _gnss_base_rtcm_target_sides=lambda: ("left",),
        gnss_base_rtcm_stats={
            "injection": {
                "left": {
                    "frames": 10,
                    "bytes": 100,
                    "packets": 10,
                    "fail": 0,
                    "blocked_stall": 1,
                    "last_delivery_s": now_s,
                }
            },
            "last_frame_s": now_s,
        },
        mavlink_pixhawk=SimpleNamespace(
            connections={"left": object()},
            mavlink_health_snapshot=lambda _side, **_kwargs: stalled_health,
        ),
        latest_gps={},
        _selected_baseline_gps_state=lambda _side: None,
        gnss_rover_fixed_since={"left": None, "right": None, "drone": None},
        latest_baseline_vector=None,
        latest_baseline_quality=None,
    )

    def capture_readiness(_config, inputs):
        captured.append(inputs)
        return SimpleNamespace(reasons=("route_fault",), warnings=(), reason_messages={})

    monkeypatch.setattr(gnss_runtime_controller, "evaluate_gnss_readiness", capture_readiness)
    GnssRuntimeController(panel)._refresh_gnss_operation_readiness()

    route = captured[0].routes["left"]
    assert route.endpoint_identity == ""
    assert "ETH MAVLINK RX STALLED" in route.fault_reason
    assert "ALL_EXPECTED_RX_SILENT" in route.fault_reason


def test_gga_provider_rejects_stale_or_skewed_rover_positions() -> None:
    now_s = time.monotonic()
    base = {
        "latitude_deg": 37.0,
        "longitude_deg": 127.0,
        "altitude_m": 50.0,
        "has_position": True,
    }
    for left_timestamp, right_timestamp, reason in (
        (now_s - 10.0, now_s - 0.01, "stale"),
        (now_s - 0.20, now_s - 0.01, "timestamp delta"),
    ):
        logged: list[str] = []
        panel = _gga_panel(
            gga_policy=GgaPolicy.SYSTEM_CENTER,
            log_sink=logged,
            latest_gps={
                "left": SimpleNamespace(**base, timestamp_s=left_timestamp),
                "right": SimpleNamespace(**base, timestamp_s=right_timestamp),
            },
        )

        assert GnssRuntimeController(panel)._gnss_gga_provider() is None
        assert any(reason in message.lower() for message in logged)


def test_left_gga_policy_never_substitutes_the_right_rover() -> None:
    logged: list[str] = []
    right = SimpleNamespace(
        latitude_deg=37.0,
        longitude_deg=127.0,
        altitude_m=50.0,
        has_position=True,
        timestamp_s=time.monotonic(),
    )
    panel = _gga_panel(
        gga_policy=GgaPolicy.LEFT_GNSS,
        log_sink=logged,
        latest_gps={"right": right},
    )

    assert GnssRuntimeController(panel)._gnss_gga_provider() is None
    assert any("left" in message.lower() for message in logged)


def test_survey_in_valid_requires_current_command_ack_and_active_observation() -> None:
    manager = SimpleNamespace(state=GnssOperationState.BASE_SURVEYING)
    transitions: list[GnssOperationState] = []
    sent: list[tuple[str, bytes]] = []

    def transition(state: GnssOperationState, _reason: str) -> bool:
        transitions.append(state)
        manager.state = state
        return True

    panel = SimpleNamespace(
        gnss_operation_manager=manager,
        gnss_base_serial_worker=SimpleNamespace(
            send_bytes=lambda label, payload: sent.append((label, bytes(payload)))
        ),
        gnss_base_status_labels={"connection": object(), "survey_in": object()},
        _set_health_label=lambda *_args: None,
        _update_gnss_base_survey_visualization=lambda **_kwargs: None,
        _refresh_gnss_operation_readiness=lambda: None,
        _gnss_active_operation_config=lambda: SimpleNamespace(
            operation_mode=GnssOperationMode.LOCAL_SURVEY_IN
        ),
        _gnss_transition=transition,
        _gnss_survey_generation=3,
        _gnss_survey_command_sent_generation=0,
        _gnss_survey_ack_generation=0,
        _gnss_survey_active_seen_generation=0,
        _gnss_survey_requested_duration_s=60,
        _gnss_survey_requested_accuracy_m=0.1,
    )
    controller = GnssRuntimeController(panel)
    stale_valid = {
        "active": False,
        "valid": True,
        "duration_s": 120,
        "mean_accuracy_m": 0.01,
        "observations": 100,
    }

    controller._handle_gnss_base_survey_in(stale_valid)
    assert transitions == []
    controller._handle_gnss_base_command_sent("CFG-TMODE3 survey-in start", 40)
    controller._handle_gnss_base_ubx_message(0x05, 0x01, bytes((0x06, 0x71)))
    assert panel._gnss_survey_ack_generation == 3
    assert sent and sent[-1][0] == "NAV-SVIN poll after survey-in ACK"
    controller._handle_gnss_base_survey_in(
        {
            "active": True,
            "valid": False,
            "duration_s": 10,
            "mean_accuracy_m": 0.5,
            "observations": 10,
        }
    )
    controller._handle_gnss_base_survey_in(stale_valid)

    assert transitions == [GnssOperationState.LOCAL_RTCM_ACTIVE]
