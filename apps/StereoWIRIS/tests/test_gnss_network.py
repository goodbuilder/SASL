from __future__ import annotations

import base64
from datetime import datetime, timezone
import threading
import time

from wiris_stereo_distance.gnss_network import (
    NtripClient,
    NtripProtocolError,
    build_ntrip_request,
    gga_from_coordinate_sample,
    format_nmea_gga,
    parse_ntrip_response_header,
)
from wiris_stereo_distance.gnss_operation import GgaPolicy, NetworkConfig


def test_ntrip_request_contains_route_and_basic_auth_without_plain_password() -> None:
    config = NetworkConfig(caster_host="caster.example", caster_port=2101, mountpoint="MOUNT")

    request = build_ntrip_request(
        config,
        username="pilot",
        password="secret",
        gga_sentence="$GPGGA,123456.00,3700.0000,N,12700.0000,E,1,12,1.0,50.000,M,0.000,M,,",
    )
    text = request.decode("ascii")

    assert "GET /MOUNT HTTP/1.0" in text
    assert "Host: caster.example" in text
    assert "Authorization: Basic " + base64.b64encode(b"pilot:secret").decode("ascii") in text
    assert "Ntrip-GGA: $GPGGA,123456.00" in text
    assert "secret" not in text


def test_ntrip_response_header_preserves_initial_rtcm_bytes() -> None:
    status, payload = parse_ntrip_response_header(b"ICY 200 OK\r\nServer: test\r\n\r\n\xd3\x00\x00")

    assert status == "ICY 200 OK"
    assert payload == b"\xd3\x00\x00"


def test_ntrip_response_rejection_is_explicit() -> None:
    try:
        parse_ntrip_response_header(b"HTTP/1.1 401 Unauthorized\r\n\r\n")
    except NtripProtocolError as exc:
        assert "401" in str(exc)
    else:
        raise AssertionError("rejected NTRIP response was accepted")


def test_gga_is_well_formed_and_checksummed() -> None:
    sentence = format_nmea_gga(
        37.1234567,
        127.7654321,
        51.25,
        utc_datetime=datetime(2026, 8, 2, 12, 34, 56, 123456, tzinfo=timezone.utc),
        fix_quality=6,
        satellites=18,
    )
    body, checksum = sentence[1:].split("*", 1)
    expected = 0
    for char in body:
        expected ^= ord(char)

    assert sentence.startswith("$GPGGA,123456.12,")
    assert body.endswith(",M,0.000,M,,")
    assert checksum == f"{expected:02X}"


def test_base_coordinate_gga_uses_wgs84_sample_and_msl_altitude_for_ntrip() -> None:
    from wiris_stereo_distance.gnss_profile import GnssCoordinateSample
    from wiris_stereo_distance.gnss_operation import HeightType

    sample = GnssCoordinateSample(
        latitude_deg=37.0,
        longitude_deg=127.0,
        ellipsoid_height_m=52.0,
        msl_altitude_m=48.0,
        height_type=HeightType.ELLIPSOID,
        timestamp_s=10.0,
        fixed=True,
    )

    sentence = gga_from_coordinate_sample(
        sample,
        utc_datetime=datetime(2026, 8, 2, 12, 34, 56, tzinfo=timezone.utc),
    )

    assert sentence is not None
    assert ",6,12,1.0,48.000,M,0.000,M,," in sentence


class _FakeSocket:
    def __init__(self) -> None:
        self.sent: list[bytes] = []
        self.responses = [b"ICY 200 OK\r\n\r\n\xd3\x00\x00", b"\x01\x02"]
        self.close_count = 0
        self.shutdown_count = 0
        self.timeout = None

    def settimeout(self, value: float) -> None:
        self.timeout = value

    def sendall(self, payload: bytes) -> None:
        self.sent.append(payload)

    def recv(self, _size: int) -> bytes:
        if self.responses:
            return self.responses.pop(0)
        raise TimeoutError

    def shutdown(self, _how: int) -> None:
        self.shutdown_count += 1
        return None

    def close(self) -> None:
        self.close_count += 1
        return None


def test_ntrip_client_yields_initial_and_stream_payload() -> None:
    fake_socket = _FakeSocket()
    config = NetworkConfig(
        caster_host="caster.example",
        mountpoint="MOUNT",
        gga_policy=GgaPolicy.MANUAL,
        read_timeout_s=0.1,
    )
    client = NtripClient(config, username="u", password="p", socket_factory=lambda *_args, **_kwargs: fake_socket)
    status, initial = client.connect()
    stop = threading.Event()
    chunks = client.iter_chunks(stop, initial_payload=initial)

    assert status == "ICY 200 OK"
    assert next(chunks) == b"\xd3\x00\x00"
    assert next(chunks) == b"\x01\x02"
    stop.set()
    client.close()
    assert fake_socket.sent[0].startswith(b"GET /MOUNT HTTP/1.0")


def test_ntrip_client_close_is_idempotent() -> None:
    fake_socket = _FakeSocket()
    config = NetworkConfig(caster_host="caster.example", mountpoint="MOUNT")
    client = NtripClient(config, socket_factory=lambda *_args, **_kwargs: fake_socket)

    client.connect()
    client.close()
    client.close()

    assert fake_socket.shutdown_count == 1
    assert fake_socket.close_count == 1


class _BlockingSocket:
    def __init__(self) -> None:
        self.sent: list[bytes] = []
        self.responses = [b"ICY 200 OK\r\n\r\n"]
        self.recv_entered = threading.Event()
        self.closed = threading.Event()
        self.timeout = None

    def settimeout(self, value: float) -> None:
        self.timeout = value

    def sendall(self, payload: bytes) -> None:
        self.sent.append(payload)

    def recv(self, _size: int) -> bytes:
        if self.responses:
            return self.responses.pop(0)
        self.recv_entered.set()
        self.closed.wait(1.0)
        raise OSError("socket closed")

    def shutdown(self, _how: int) -> None:
        self.closed.set()

    def close(self) -> None:
        self.closed.set()


def test_ntrip_client_close_stops_blocked_receive_without_error() -> None:
    fake_socket = _BlockingSocket()
    config = NetworkConfig(
        caster_host="caster.example",
        mountpoint="MOUNT",
        read_timeout_s=30.0,
    )
    client = NtripClient(config, socket_factory=lambda *_args, **_kwargs: fake_socket)
    client.connect()
    stop = threading.Event()
    chunks = client.iter_chunks(stop)
    result: list[str] = []

    def read_next() -> None:
        try:
            next(chunks)
        except StopIteration:
            result.append("stopped")
        except Exception as exc:
            result.append(f"error:{exc}")

    thread = threading.Thread(target=read_next)
    thread.start()
    assert fake_socket.recv_entered.wait(1.0)

    stop.set()
    client.close()
    thread.join(1.0)

    assert not thread.is_alive()
    assert result == ["stopped"]


def test_ntrip_client_reconnect_wait_uses_configured_delay() -> None:
    config = NetworkConfig(
        caster_host="caster.example",
        mountpoint="MOUNT",
        reconnect_interval_s=0.02,
    )
    client = NtripClient(config)
    stop = threading.Event()

    started = time.monotonic()
    should_retry = client.wait_for_reconnect(stop)
    elapsed = time.monotonic() - started

    assert should_retry is True
    assert elapsed >= 0.02


def test_ntrip_client_reconnect_wait_stops_early() -> None:
    config = NetworkConfig(
        caster_host="caster.example",
        mountpoint="MOUNT",
        reconnect_interval_s=5.0,
    )
    client = NtripClient(config)
    stop = threading.Event()
    timer = threading.Timer(0.02, stop.set)

    started = time.monotonic()
    timer.start()
    try:
        should_retry = client.wait_for_reconnect(stop)
    finally:
        timer.cancel()
    elapsed = time.monotonic() - started

    assert should_retry is False
    assert elapsed < 1.0
