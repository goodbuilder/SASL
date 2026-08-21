"""Small, dependency-free NTRIP client primitives for StereoWIRIS.

The client only transports RTCM bytes.  It does not interpret those bytes as a
position or convert them into the stereo baseline.  NTRIP coordinates are
WGS84 geodetic latitude/longitude and metres above the selected height datum;
the existing rover baseline remains NED metres in :mod:`rtk_geometry`.
"""

from __future__ import annotations

import base64
from dataclasses import dataclass
from datetime import datetime, timezone
import math
import os
import socket
import ssl
import time
from typing import Any, Callable, Iterator

from Workstation.workstation_core.gnss_operation import NetworkConfig


class NtripProtocolError(RuntimeError):
    """The caster did not return a usable NTRIP response."""


def _header_value(value: str, name: str) -> str:
    text = str(value).strip()
    if "\r" in text or "\n" in text:
        raise ValueError(f"{name} must not contain CR/LF")
    return text


def build_ntrip_request(
    config: NetworkConfig,
    *,
    username: str = "",
    password: str = "",
    gga_sentence: str | None = None,
    user_agent: str = "StereoWIRIS-NTRIP/1.0",
) -> bytes:
    """Build an NTRIP caster request without embedding credentials in logs."""

    host = _header_value(config.caster_host, "caster_host")
    mountpoint = _header_value(config.mountpoint, "mountpoint").lstrip("/")
    if not host:
        raise ValueError("network.caster_host is required")
    if not mountpoint:
        raise ValueError("network.mountpoint is required")
    lines = [
        f"GET /{mountpoint} HTTP/1.0",
        f"Host: {host}",
        f"User-Agent: {_header_value(user_agent, 'user_agent')}",
        "Accept: */*",
        "Connection: close",
    ]
    if gga_sentence:
        lines.append(f"Ntrip-GGA: {_header_value(gga_sentence, 'gga_sentence')}")
    if username:
        token = base64.b64encode(f"{username}:{password}".encode("utf-8")).decode("ascii")
        lines.append(f"Authorization: Basic {token}")
    return ("\r\n".join(lines) + "\r\n\r\n").encode("ascii")


_NTRIP1_STATUS_LINE_PREFIXES = ("ICY ", "SOURCETABLE ", "ERROR")


def _ntrip1_status_line(data: bytes) -> str | None:
    """Return the decoded first line when a legacy NTRIP 1.0 header is complete.

    NTRIP 1.0 casters using the common ICY-style response terminate their
    status line with a single
    ``\\r\\n`` and start streaming raw RTCM bytes immediately after -- there
    is no second ``\\r\\n`` to close out an HTTP-style header block. This is
    only recognised once we can see that what follows the single ``\\r\\n``
    is not further printable header text, so a caster that *does* send
    additional header lines before the real blank-line terminator is not
    mistaken for the legacy single-line style while more header bytes are
    still in flight.
    """

    index = data.find(b"\r\n")
    if index < 0:
        return None
    first_line = data[:index].decode("latin-1", "replace").strip()
    if not first_line.upper().startswith(_NTRIP1_STATUS_LINE_PREFIXES):
        return None
    remainder = data[index + 2 :]
    if not remainder:
        return None
    next_byte = remainder[0]
    if 0x20 <= next_byte <= 0x7E and next_byte not in (0x0D, 0x0A):
        return None
    return first_line


def ntrip_header_ready(data: bytes) -> bool:
    """Return whether ``data`` contains a complete NTRIP response header."""

    if b"\r\n\r\n" in data:
        return True
    return _ntrip1_status_line(data) is not None


def parse_ntrip_response_header(data: bytes) -> tuple[str, bytes]:
    """Return the status line and bytes received after the HTTP header."""

    separator = data.find(b"\r\n\r\n")
    if separator >= 0:
        header = data[:separator].decode("latin-1", "replace")
        status_line = header.splitlines()[0].strip() if header.splitlines() else ""
        payload_start = separator + 4
    else:
        legacy_line = _ntrip1_status_line(data)
        if legacy_line is None:
            raise NtripProtocolError("NTRIP response header is incomplete")
        status_line = legacy_line
        payload_start = data.find(b"\r\n") + 2
    upper = status_line.upper()
    if not (" 200 " in f" {upper} " or upper.endswith(" 200")):
        raise NtripProtocolError(f"NTRIP caster rejected request: {status_line or 'empty response'}")
    return status_line, data[payload_start:]


def _nmea_checksum(body: str) -> str:
    checksum = 0
    for char in body:
        checksum ^= ord(char)
    return f"{checksum:02X}"


def _nmea_coordinate(value: float, *, latitude: bool) -> tuple[str, str]:
    absolute = abs(float(value))
    degrees = int(absolute)
    minutes = (absolute - degrees) * 60.0
    width = 2 if latitude else 3
    direction = ("N" if value >= 0 else "S") if latitude else ("E" if value >= 0 else "W")
    return f"{degrees:0{width}d}{minutes:07.4f}", direction


def format_nmea_gga(
    latitude_deg: float,
    longitude_deg: float,
    altitude_m: float,
    *,
    utc_datetime: datetime | None = None,
    fix_quality: int = 1,
    satellites: int = 12,
    hdop: float = 1.0,
    geoid_separation_m: float | None = None,
) -> str:
    """Format a checksummed GGA sentence for a caster/VRS position hint."""

    moment = utc_datetime or datetime.now(timezone.utc)
    if moment.tzinfo is None:
        moment = moment.replace(tzinfo=timezone.utc)
    moment = moment.astimezone(timezone.utc)
    lat, lat_dir = _nmea_coordinate(float(latitude_deg), latitude=True)
    lon, lon_dir = _nmea_coordinate(float(longitude_deg), latitude=False)
    time_text = moment.strftime("%H%M%S.%f")[:9]
    if geoid_separation_m is None:
        geoid_text = ""
    else:
        geoid_value = float(geoid_separation_m)
        if not math.isfinite(geoid_value):
            raise ValueError("geoid_separation_m must be finite")
        geoid_text = f"{geoid_value:.3f}"
    body = (
        f"GPGGA,{time_text}"
        f",{lat},{lat_dir},{lon},{lon_dir},{int(fix_quality)},{int(satellites):02d},"
        f"{float(hdop):.1f},{float(altitude_m):.3f},M,{geoid_text},M,,"
    )
    return f"${body}*{_nmea_checksum(body)}"


def gga_from_position(position: Any, *, utc_datetime: datetime | None = None) -> str | None:
    """Format GGA from a ``GpsPositionState``-like object when possible."""

    latitude = getattr(position, "latitude_deg", None)
    longitude = getattr(position, "longitude_deg", None)
    altitude = getattr(position, "altitude_m", None)
    if latitude is None or longitude is None or altitude is None:
        return None
    ellipsoid = getattr(position, "altitude_ellipsoid_m", None)
    geoid_separation = (
        None
        if ellipsoid is None
        else float(ellipsoid) - float(altitude)
    )
    return format_nmea_gga(
        latitude,
        longitude,
        altitude,
        utc_datetime=utc_datetime,
        # NMEA GGA fix_quality is a different enum than MAVLink fix_type: 4 is
        # RTK Fixed there (6 means "Estimated/dead reckoning"), even though
        # MAVLink's RTK_FIXED happens to be numbered 6.
        fix_quality=4 if getattr(position, "fix_type", None) == 6 else 1,
        satellites=getattr(position, "satellites_used", None) or 12,
        hdop=getattr(position, "eph_m", None) or 1.0,
        geoid_separation_m=geoid_separation,
    )


def gga_from_coordinate_sample(sample: Any, *, utc_datetime: datetime | None = None) -> str | None:
    """Format NMEA GGA from a WGS84 geodetic Base coordinate sample.

    ``GnssCoordinateSample`` stores the Base position as latitude/longitude
    degrees and heights in metres.  GGA's altitude field is MSL; when both MSL
    and ellipsoid heights are present their difference is emitted as the geoid
    separation.  Missing MSL height fails closed instead of relabeling an
    ellipsoid height as MSL.  This helper never converts the coordinate into
    the rover NED/ENU baseline frame.
    """

    latitude = getattr(sample, "latitude_deg", None)
    longitude = getattr(sample, "longitude_deg", None)
    if latitude is None or longitude is None:
        return None
    altitude = getattr(sample, "msl_altitude_m", None)
    if altitude is None:
        return None
    ellipsoid = getattr(sample, "ellipsoid_height_m", None)
    geoid_separation = (
        None
        if ellipsoid is None
        else float(ellipsoid) - float(altitude)
    )
    return format_nmea_gga(
        latitude,
        longitude,
        altitude,
        utc_datetime=utc_datetime,
        # See gga_from_position: GGA fix_quality 4 (not 6) is RTK Fixed.
        fix_quality=4 if bool(getattr(sample, "fixed", False)) else 1,
        satellites=getattr(sample, "satellites_used", None) or 12,
        hdop=1.0,
        geoid_separation_m=geoid_separation,
    )


@dataclass(frozen=True)
class NtripStreamStats:
    connected_at_monotonic_s: float
    bytes_received: int = 0
    chunks_received: int = 0
    last_chunk_monotonic_s: float | None = None
    last_gga_monotonic_s: float | None = None
    # Total GGA sentences actually written to the socket (initial + periodic),
    # not merely "handed to gga_provider" -- a send that raises OSError does
    # not count. Distinct diagnostic evidence for "did we ever confirm a live
    # rover position to the caster" versus "did RTCM keep flowing".
    gga_sent_count: int = 0


class NtripClient:
    """Blocking NTRIP stream client intended to run outside the Qt GUI thread.

    ``close()`` is idempotent and can interrupt an active ``recv()`` once a
    socket exists.  It does not promise to cancel DNS lookup or
    ``socket.create_connection()`` before the socket factory returns.
    """

    def __init__(
        self,
        config: NetworkConfig,
        *,
        username: str | None = None,
        password: str | None = None,
        socket_factory: Callable[..., socket.socket] | None = None,
        diagnostics: Callable[[str], None] | None = None,
    ) -> None:
        self.config = config
        self.username = os.getenv(config.username_env, "") if username is None else str(username)
        self.password = os.getenv(config.password_env, "") if password is None else str(password)
        self._socket_factory = socket_factory or socket.create_connection
        self._socket: socket.socket | ssl.SSLSocket | None = None
        self._closed = False
        self._stats: NtripStreamStats | None = None
        # Optional one-line-per-event hook (e.g. wired to a Qt log signal by
        # the caller). Never allowed to break the network loop: a raising
        # callback is swallowed, not propagated.
        self._diagnostics = diagnostics

    @property
    def stats(self) -> NtripStreamStats | None:
        return self._stats

    def _diagnostic(self, message: str) -> None:
        if self._diagnostics is None:
            return
        try:
            self._diagnostics(message)
        except Exception:
            pass

    def _eof_diagnostic_message(self, now_s: float) -> str:
        """Build the disconnect evidence bundle for a caster-closed stream.

        Every field the operator needs to tell "we never confirmed a live
        rover position" (last_gga_age) apart from "the caster just stopped
        sending RTCM" (last_rtcm_age) apart from "the caster capped the
        session duration regardless" (connected_duration) lands in this one
        message, which is what reaches the visible log via the existing
        reconnecting/failed handlers -- no separate plumbing needed.
        """

        stats = self._stats
        if stats is None:
            return "NTRIP caster closed the stream (no stream stats available)"
        connected_duration_s = now_s - stats.connected_at_monotonic_s
        last_gga_age = (
            "never" if stats.last_gga_monotonic_s is None else f"{now_s - stats.last_gga_monotonic_s:.2f}s"
        )
        last_rtcm_age = (
            "never" if stats.last_chunk_monotonic_s is None else f"{now_s - stats.last_chunk_monotonic_s:.2f}s"
        )
        return (
            "NTRIP caster closed the stream "
            f"(connected_duration={connected_duration_s:.2f}s, last_gga_age={last_gga_age}, "
            f"last_rtcm_age={last_rtcm_age}, gga_sent={stats.gga_sent_count}, "
            f"chunks_received={stats.chunks_received}, bytes_received={stats.bytes_received})"
        )

    def connect(self, *, gga_sentence: str | None = None) -> tuple[str, bytes]:
        if self._socket is not None:
            raise RuntimeError("NTRIP client is already connected")
        self._closed = False
        stream_socket: socket.socket | ssl.SSLSocket | None = None
        try:
            if self.config.tls:
                raw_socket = self._socket_factory(
                    (self.config.caster_host, int(self.config.caster_port)),
                    timeout=float(self.config.connect_timeout_s),
                )
                stream_socket = raw_socket
                if self._closed:
                    self._close_socket(raw_socket)
                    raise RuntimeError("NTRIP client was closed before connection completed")
                context = ssl.create_default_context()
                stream_socket: socket.socket | ssl.SSLSocket = context.wrap_socket(
                    raw_socket,
                    server_hostname=self.config.caster_host,
                )
            else:
                stream_socket = self._socket_factory(
                    (self.config.caster_host, int(self.config.caster_port)),
                    timeout=float(self.config.connect_timeout_s),
                )
            if self._closed:
                self._close_socket(stream_socket)
                raise RuntimeError("NTRIP client was closed before connection completed")
            self._socket = stream_socket
            self._socket.settimeout(float(self.config.read_timeout_s))
            request = build_ntrip_request(
                self.config,
                username=self.username,
                password=self.password,
                gga_sentence=gga_sentence,
            )
            # NTRIP 1.x client behaviour: the initial GGA (when available) is
            # part of the same initial wire transmission as the request, sent
            # as a raw NMEA line directly following the request's blank-line
            # terminator -- not merely referenced via the non-standard
            # ``Ntrip-GGA:`` request header above, which most casters ignore.
            # Folding it into one sendall() (rather than a second write after
            # the response arrives, as an earlier version of this method did)
            # means a caster that requires an on-the-wire GGA before it will
            # keep a session open never sees a window with none: previously,
            # a caster that both required this AND rejected the ``Ntrip-GGA:``
            # header saw a live connection with zero position evidence for
            # the round trip between the request and this second write.
            initial_gga_bytes = (
                (gga_sentence.rstrip("\r\n") + "\r\n").encode("ascii") if gga_sentence else b""
            )
            self._socket.sendall(request + initial_gga_bytes)
            if gga_sentence:
                self._diagnostic(
                    f"INITIAL GGA TX ok len={len(gga_sentence)} sent_with_request=True at={time.monotonic():.3f}"
                )
            else:
                self._diagnostic(
                    "INITIAL GGA TX skipped: no gga_sentence available (gga_provider returned None)"
                )
            response = bytearray()
            deadline = time.monotonic() + float(self.config.connect_timeout_s)
            while not ntrip_header_ready(bytes(response)):
                if self._closed:
                    raise RuntimeError("NTRIP client was closed before response header completed")
                if time.monotonic() > deadline:
                    raise NtripProtocolError("NTRIP response header timeout")
                chunk = self._socket.recv(4096)
                if not chunk:
                    break
                response.extend(chunk)
            status_line, initial_payload = parse_ntrip_response_header(bytes(response))
            now_s = time.monotonic()
            self._stats = NtripStreamStats(
                connected_at_monotonic_s=now_s,
                last_gga_monotonic_s=now_s if gga_sentence else None,
                gga_sent_count=1 if gga_sentence else 0,
            )
            self._diagnostic(f"NTRIP CONNECTED status={status_line!r} at={now_s:.3f}")
            return status_line, initial_payload
        except Exception:
            self._socket = None
            self._stats = None
            self._close_socket(stream_socket)
            raise

    def iter_chunks(
        self,
        stop_event: Any,
        *,
        gga_provider: Callable[[], str | None] | None = None,
        initial_payload: bytes = b"",
        on_gga_interval_s: float = 5.0,
    ) -> Iterator[bytes]:
        if self._socket is None:
            raise RuntimeError("NTRIP client is not connected")
        if initial_payload and not self._closed and not stop_event.is_set():
            yield initial_payload
        last_gga_s = time.monotonic()
        while not self._closed and not stop_event.is_set():
            now_s = time.monotonic()
            if gga_provider is not None and now_s - last_gga_s >= max(1.0, float(on_gga_interval_s)):
                # Advance the interval clock unconditionally, even when the
                # provider has nothing to send this cycle. Only bumping it
                # inside the ``if sentence:`` branch below meant a provider
                # stuck returning ``None`` (e.g. GNSS fix lost mid-session)
                # got re-polled on every loop iteration -- as fast as
                # ``recv()`` returns -- instead of once per interval.
                last_gga_s = now_s
                sentence = gga_provider()
                if sentence:
                    try:
                        self._socket.sendall((sentence.rstrip("\r\n") + "\r\n").encode("ascii"))
                    except OSError:
                        if self._closed or stop_event.is_set():
                            return
                        raise
                    if self._stats is not None:
                        self._stats = NtripStreamStats(
                            **{
                                **self._stats.__dict__,
                                "last_gga_monotonic_s": now_s,
                                "gga_sent_count": self._stats.gga_sent_count + 1,
                            }
                        )
                    self._diagnostic(
                        f"PERIODIC GGA TX #{self._stats.gga_sent_count if self._stats else '?'} "
                        f"ok len={len(sentence)} at={now_s:.3f}"
                    )
                else:
                    self._diagnostic(f"PERIODIC GGA TX skipped: gga_provider returned no sentence at={now_s:.3f}")
            try:
                chunk = self._socket.recv(4096)
            except socket.timeout:
                continue
            except OSError:
                if self._closed or stop_event.is_set():
                    return
                raise
            if not chunk:
                if self._closed or stop_event.is_set():
                    return
                raise NtripProtocolError(self._eof_diagnostic_message(now_s))
            if self._stats is not None:
                self._stats = NtripStreamStats(
                    **{
                        **self._stats.__dict__,
                        "bytes_received": self._stats.bytes_received + len(chunk),
                        "chunks_received": self._stats.chunks_received + 1,
                        "last_chunk_monotonic_s": now_s,
                    }
                )
            yield bytes(chunk)

    def wait_for_reconnect(self, stop_event: Any, *, interval_s: float | None = None) -> bool:
        """Wait before reconnecting, returning ``False`` when stopped or closed.

        The default delay is ``NetworkConfig.reconnect_interval_s``.  This is
        intended for worker loops after a stream failure; it is only a retry
        delay helper and does not cancel an already-running ``connect()`` call.
        """

        interval = self.config.reconnect_interval_s if interval_s is None else interval_s
        deadline = time.monotonic() + max(0.0, float(interval))
        while not self._closed and not stop_event.is_set():
            remaining_s = deadline - time.monotonic()
            if remaining_s <= 0.0:
                return not self._closed and not stop_event.is_set()
            wait_s = min(0.1, remaining_s)
            waiter = getattr(stop_event, "wait", None)
            if callable(waiter):
                if waiter(wait_s):
                    return False
            else:
                time.sleep(wait_s)
        return False

    def close(self) -> None:
        self._closed = True
        stream_socket = self._socket
        self._socket = None
        self._close_socket(stream_socket)

    @staticmethod
    def _close_socket(stream_socket: Any) -> None:
        if stream_socket is None:
            return
        try:
            stream_socket.shutdown(socket.SHUT_RDWR)
        except Exception:
            pass
        try:
            stream_socket.close()
        except Exception:
            pass
