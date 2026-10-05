from __future__ import annotations

import errno
import socket

from Workstation.gnss_runtime_workers import (
    NtripNetworkWorker,
    _is_transient_network_error,
)


def test_temporary_name_resolution_failure_is_transient() -> None:
    # The exact error a dropped hotspot uplink raises from getaddrinfo.
    exc = socket.gaierror(socket.EAI_AGAIN, "Temporary failure in name resolution")
    assert exc.errno == -3
    assert _is_transient_network_error(exc)


def test_unknown_host_is_not_transient() -> None:
    # A typo in caster_host repeats forever; it must stay on the short count.
    exc = socket.gaierror(socket.EAI_NONAME, "Name or service not known")
    assert not _is_transient_network_error(exc)


def test_link_outages_are_transient_but_rejections_are_not() -> None:
    assert _is_transient_network_error(TimeoutError("connect timed out"))
    assert _is_transient_network_error(ConnectionRefusedError("refused"))
    assert _is_transient_network_error(OSError(errno.ENETUNREACH, "Network is unreachable"))
    assert _is_transient_network_error(OSError(errno.EHOSTUNREACH, "No route to host"))

    assert not _is_transient_network_error(RuntimeError("HTTP/1.1 401 Unauthorized"))
    assert not _is_transient_network_error(RuntimeError("unknown mountpoint"))
    assert not _is_transient_network_error(OSError(errno.EACCES, "Permission denied"))


def test_transient_failures_get_an_outage_length_budget_not_three_strikes() -> None:
    # A dropped uplink must outlast the three strikes meant for a caster that
    # keeps rejecting us, or a momentary DNS blip ends the session.
    assert NtripNetworkWorker.MAX_CONSECUTIVE_FAILURES == 3
    assert (
        NtripNetworkWorker.MAX_CONSECUTIVE_TRANSIENT_FAILURES
        > NtripNetworkWorker.MAX_CONSECUTIVE_FAILURES
    )
    # At the 5 s default reconnect interval this must cover a multi-minute
    # outage rather than seconds.
    assert NtripNetworkWorker.MAX_CONSECUTIVE_TRANSIENT_FAILURES * 5 >= 300
