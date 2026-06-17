from types import SimpleNamespace

import pytest

import wiris_stereo_distance.network as network


@pytest.fixture(autouse=True)
def no_real_arp_scan(monkeypatch) -> None:
    monkeypatch.setattr(network.shutil, "which", lambda command: None)


def test_local_ipv4_interfaces_skips_virtual_links(monkeypatch) -> None:
    output = "\n".join(
        [
            "2: enx0    inet 192.168.0.254/24 brd 192.168.0.255 scope global enx0",
            "3: docker0 inet 172.17.0.1/16 brd 172.17.255.255 scope global docker0",
        ]
    )

    monkeypatch.setattr(
        network.subprocess,
        "run",
        lambda *_args, **_kwargs: SimpleNamespace(returncode=0, stdout=output, stderr=""),
    )

    interfaces = network.local_ipv4_interfaces()

    assert len(interfaces) == 1
    assert interfaces[0].name == "enx0"
    assert str(interfaces[0].address) == "192.168.0.254/24"


def test_interface_has_address_matches_exact_interface_address(monkeypatch) -> None:
    def fake_run(args, **_kwargs):
        if args[:5] == ["ip", "-o", "-4", "addr", "show"]:
            return SimpleNamespace(
                returncode=0,
                stdout="2: enx0    inet 10.0.0.10/24 brd 10.0.0.255 scope global enx0\n",
                stderr="",
            )
        raise AssertionError(args)

    monkeypatch.setattr(network.subprocess, "run", fake_run)

    assert network.interface_has_address("enx0", "10.0.0.10/24") is True
    assert network.interface_has_address("enx0", "10.0.0.11/24") is False
    assert network.interface_has_address("enx1", "10.0.0.10/24") is False


def test_resolve_nm_interface_address_uses_active_profile_camera_subnet(monkeypatch) -> None:
    def fake_run(args, **_kwargs):
        if args[:6] == ["nmcli", "-g", "GENERAL.CONNECTION", "device", "show", "enx0"]:
            return SimpleNamespace(returncode=0, stdout="pixhawk-dds\n", stderr="")
        if args[:6] == ["nmcli", "-g", "ipv4.addresses", "connection", "show", "pixhawk-dds"]:
            return SimpleNamespace(
                returncode=0,
                stdout="10.41.10.1/24, 192.168.0.254/24, 10.0.0.11/24\n",
                stderr="",
            )
        raise AssertionError(args)

    monkeypatch.setattr(network.shutil, "which", lambda command: "/usr/bin/nmcli" if command == "nmcli" else None)
    monkeypatch.setattr(network.subprocess, "run", fake_run)

    resolved = network.resolve_nm_interface_address("enx0", ("10.0.0.231", "10.0.0.232"))

    assert resolved is not None
    assert str(resolved) == "10.0.0.11/24"


def test_resolve_nm_interface_address_ignores_unrelated_profile_addresses(monkeypatch) -> None:
    def fake_run(args, **_kwargs):
        if args[:6] == ["nmcli", "-g", "GENERAL.CONNECTION", "device", "show", "enx0"]:
            return SimpleNamespace(returncode=0, stdout="pixhawk-dds\n", stderr="")
        if args[:6] == ["nmcli", "-g", "ipv4.addresses", "connection", "show", "pixhawk-dds"]:
            return SimpleNamespace(
                returncode=0,
                stdout="10.41.10.1/24, 192.168.0.254/24\n",
                stderr="",
            )
        raise AssertionError(args)

    monkeypatch.setattr(network.shutil, "which", lambda command: "/usr/bin/nmcli" if command == "nmcli" else None)
    monkeypatch.setattr(network.subprocess, "run", fake_run)

    assert network.resolve_nm_interface_address("enx0", ("10.0.0.231", "10.0.0.232")) is None


def test_discover_rtsp_candidates_uses_neighbors_and_selects_host_interface(monkeypatch) -> None:
    calls = []

    def fake_run(args, **_kwargs):
        if args[:5] == ["ip", "-o", "-4", "addr", "show"]:
            return SimpleNamespace(
                returncode=0,
                stdout="2: enx0    inet 192.168.0.254/24 brd 192.168.0.255 scope global enx0\n",
                stderr="",
            )
        if args[:3] == ["ip", "-4", "neigh"]:
            return SimpleNamespace(
                returncode=0,
                stdout="192.168.0.3 dev enx0 lladdr 00:11:22:33:44:55 REACHABLE\n",
                stderr="",
            )
        raise AssertionError(args)

    def fake_check_tcp_port(host, port, *, timeout):
        calls.append((host, port, timeout))
        return host == "192.168.0.3" and port == 8554

    monkeypatch.setattr(network.subprocess, "run", fake_run)
    monkeypatch.setattr(network, "check_tcp_port", fake_check_tcp_port)

    candidates = network.discover_rtsp_candidates(timeout=0.2, max_hosts_per_network=0, fallback_scan_if_empty=False)

    assert [candidate.url for candidate in candidates] == ["rtsp://192.168.0.3:8554/visible"]
    assert candidates[0].interface is not None
    assert candidates[0].interface.name == "enx0"
    assert calls == [("192.168.0.3", 8554, 0.2)]


def test_discover_rtsp_candidates_falls_back_to_subnet_scan(monkeypatch) -> None:
    calls = []

    def fake_run(args, **_kwargs):
        if args[:5] == ["ip", "-o", "-4", "addr", "show"]:
            return SimpleNamespace(
                returncode=0,
                stdout="2: enx0    inet 192.168.0.254/24 brd 192.168.0.255 scope global enx0\n",
                stderr="",
            )
        if args[:3] == ["ip", "-4", "neigh"]:
            return SimpleNamespace(returncode=0, stdout="", stderr="")
        raise AssertionError(args)

    def fake_check_tcp_port(host, port, *, timeout):
        calls.append((host, port))
        return host == "192.168.0.3" and port == 8554

    monkeypatch.setattr(network.subprocess, "run", fake_run)
    monkeypatch.setattr(network, "check_tcp_port", fake_check_tcp_port)

    candidates = network.discover_rtsp_candidates(timeout=0.2, max_hosts_per_network=0)

    assert [candidate.url for candidate in candidates] == ["rtsp://192.168.0.3:8554/visible"]
    assert ("192.168.0.3", 8554) in calls


def test_discover_rtsp_candidates_uses_arp_scan_hosts(monkeypatch) -> None:
    calls = []

    def fake_run(args, **_kwargs):
        if args[:5] == ["ip", "-o", "-4", "addr", "show"]:
            return SimpleNamespace(
                returncode=0,
                stdout="2: enx0    inet 192.168.0.254/24 brd 192.168.0.255 scope global enx0\n",
                stderr="",
            )
        if args[:3] == ["ip", "-4", "neigh"]:
            return SimpleNamespace(returncode=0, stdout="", stderr="")
        if args[:3] == ["arp-scan", "--interface", "enx0"]:
            return SimpleNamespace(
                returncode=0,
                stdout=(
                    "Interface: enx0, type: EN10MB, MAC: 00:aa:bb:cc:dd:ee, IPv4: 192.168.0.254\n"
                    "192.168.0.42\t00:11:22:33:44:55\tCamera vendor\n"
                    "192.168.0.254\t00:aa:bb:cc:dd:ee\tLocal host\n"
                ),
                stderr="",
            )
        raise AssertionError(args)

    def fake_check_tcp_port(host, port, *, timeout):
        calls.append((host, port, timeout))
        return host == "192.168.0.42" and port == 8554

    monkeypatch.setattr(network.shutil, "which", lambda command: "/usr/bin/arp-scan" if command == "arp-scan" else None)
    monkeypatch.setattr(network.subprocess, "run", fake_run)
    monkeypatch.setattr(network, "check_tcp_port", fake_check_tcp_port)

    candidates = network.discover_rtsp_candidates(timeout=0.2, max_hosts_per_network=0, fallback_scan_if_empty=False)

    assert [candidate.url for candidate in candidates] == ["rtsp://192.168.0.42:8554/visible"]
    assert candidates[0].interface is not None
    assert candidates[0].interface.name == "enx0"
    assert calls == [("192.168.0.42", 8554, 0.2)]


def test_discover_tcp_port_candidates_finds_control_port_from_arp_hosts(monkeypatch) -> None:
    calls = []

    def fake_run(args, **_kwargs):
        if args[:5] == ["ip", "-o", "-4", "addr", "show"]:
            return SimpleNamespace(
                returncode=0,
                stdout="2: enx0    inet 192.168.0.254/24 brd 192.168.0.255 scope global enx0\n",
                stderr="",
            )
        if args[:3] == ["ip", "-4", "neigh"]:
            return SimpleNamespace(returncode=0, stdout="", stderr="")
        if args[:3] == ["arp-scan", "--interface", "enx0"]:
            return SimpleNamespace(
                returncode=0,
                stdout="192.168.0.42\t00:11:22:33:44:55\tCamera vendor\n",
                stderr="",
            )
        raise AssertionError(args)

    def fake_check_tcp_port(host, port, *, timeout):
        calls.append((host, port, timeout))
        return host == "192.168.0.42" and port == 2240

    monkeypatch.setattr(network.shutil, "which", lambda command: "/usr/bin/arp-scan" if command == "arp-scan" else None)
    monkeypatch.setattr(network.subprocess, "run", fake_run)
    monkeypatch.setattr(network, "check_tcp_port", fake_check_tcp_port)

    reports = []
    candidates = network.discover_tcp_port_candidates(
        2240,
        timeout=0.2,
        max_hosts_per_network=0,
        fallback_scan_if_empty=False,
        arp_scan_reports=reports,
    )

    assert [(candidate.host, candidate.port) for candidate in candidates] == [("192.168.0.42", 2240)]
    assert candidates[0].interface is not None
    assert candidates[0].interface.name == "enx0"
    assert reports[0].hosts == ("192.168.0.42",)
    assert calls == [("192.168.0.42", 2240, 0.2)]


def test_tcp_port_scan_sequence_prioritizes_configured_port_without_duplicates() -> None:
    ports = network.tcp_port_scan_sequence(3, 2, start=1, end=5)

    assert ports == (3, 2, 1, 4, 5)


def test_discover_tcp_port_range_candidates_scans_host_hints_across_ports(monkeypatch) -> None:
    calls = []

    def fake_run(args, **_kwargs):
        if args[:5] == ["ip", "-o", "-4", "addr", "show"]:
            return SimpleNamespace(
                returncode=0,
                stdout="2: enx0    inet 10.0.0.10/24 brd 10.0.0.255 scope global enx0\n",
                stderr="",
            )
        if args[:3] == ["ip", "-4", "neigh"]:
            return SimpleNamespace(returncode=0, stdout="", stderr="")
        raise AssertionError(args)

    def fake_check_tcp_port(host, port, *, timeout):
        calls.append((host, port, timeout))
        return host == "10.0.0.230" and port == 4321

    monkeypatch.setattr(network.subprocess, "run", fake_run)
    monkeypatch.setattr(network, "check_tcp_port", fake_check_tcp_port)

    candidates = network.discover_tcp_port_range_candidates(
        (2240, 4321, 8554),
        timeout=0.08,
        max_hosts_per_network=0,
        fallback_scan_if_empty=False,
        host_hints=("10.0.0.230",),
    )

    assert [(candidate.host, candidate.port) for candidate in candidates] == [("10.0.0.230", 4321)]
    assert candidates[0].interface is not None
    assert candidates[0].interface.name == "enx0"
    assert set(calls) == {
        ("10.0.0.230", 2240, 0.08),
        ("10.0.0.230", 4321, 0.08),
        ("10.0.0.230", 8554, 0.08),
    }


def test_apply_interface_address_skips_existing_address(monkeypatch) -> None:
    def fake_run(args, **_kwargs):
        if args[:5] == ["ip", "-o", "-4", "addr", "show"]:
            return SimpleNamespace(
                returncode=0,
                stdout="2: enx0    inet 10.0.0.10/24 brd 10.0.0.255 scope global enx0\n",
                stderr="",
            )
        raise AssertionError(args)

    monkeypatch.setattr(network.subprocess, "run", fake_run)

    ok, message = network.apply_interface_address("enx0", "10.0.0.10/24")

    assert ok is True
    assert "already has" in message


def test_apply_interface_address_reports_sudo_command_when_unprivileged(monkeypatch) -> None:
    calls = []

    def fake_run(args, **_kwargs):
        calls.append(args)
        if args[:5] == ["ip", "-o", "-4", "addr", "show"]:
            return SimpleNamespace(returncode=0, stdout="", stderr="")
        if args[:4] == ["ip", "addr", "add", "10.0.0.10/24"]:
            return SimpleNamespace(returncode=2, stdout="", stderr="RTNETLINK answers: Operation not permitted\n")
        raise AssertionError(args)

    monkeypatch.setattr(network.shutil, "which", lambda command: None)
    monkeypatch.setattr(network.subprocess, "run", fake_run)

    ok, message = network.apply_interface_address("enx0", "10.0.0.10/24")

    assert ok is False
    assert "sudo ip addr add 10.0.0.10/24 dev enx0" in message
    assert calls[-1] == ["ip", "addr", "add", "10.0.0.10/24", "dev", "enx0"]


def test_apply_interface_address_skips_pkexec_when_qt_is_offscreen(monkeypatch) -> None:
    calls = []

    def fake_run(args, **_kwargs):
        calls.append(args)
        if args[:5] == ["ip", "-o", "-4", "addr", "show"]:
            return SimpleNamespace(returncode=0, stdout="", stderr="")
        if args[:4] == ["ip", "addr", "add", "10.0.0.10/24"]:
            return SimpleNamespace(returncode=2, stdout="", stderr="RTNETLINK answers: Operation not permitted\n")
        raise AssertionError(args)

    monkeypatch.setenv("QT_QPA_PLATFORM", "offscreen")
    monkeypatch.setattr(network.shutil, "which", lambda command: "/usr/bin/pkexec" if command == "pkexec" else None)
    monkeypatch.setattr(network.subprocess, "run", fake_run)

    ok, message = network.apply_interface_address("enx0", "10.0.0.10/24")

    assert ok is False
    assert "sudo ip addr add 10.0.0.10/24 dev enx0" in message
    assert all(call[:1] != ["pkexec"] for call in calls)


def test_discover_rtsp_candidates_reports_arp_scan_permission_failure(monkeypatch) -> None:
    def fake_run(args, **_kwargs):
        if args[:5] == ["ip", "-o", "-4", "addr", "show"]:
            return SimpleNamespace(
                returncode=0,
                stdout="2: enx0    inet 192.168.0.254/24 brd 192.168.0.255 scope global enx0\n",
                stderr="",
            )
        if args[:3] == ["ip", "-4", "neigh"]:
            return SimpleNamespace(returncode=0, stdout="", stderr="")
        if args[:3] == ["arp-scan", "--interface", "enx0"]:
            return SimpleNamespace(
                returncode=1,
                stdout="",
                stderr="pcap_activate: enx0: You don't have permission to capture on that device\n",
            )
        raise AssertionError(args)

    monkeypatch.setattr(network.shutil, "which", lambda command: "/usr/bin/arp-scan" if command == "arp-scan" else None)
    monkeypatch.setattr(network.subprocess, "run", fake_run)
    monkeypatch.setattr(network, "check_tcp_port", lambda *_args, **_kwargs: False)

    reports = []
    candidates = network.discover_rtsp_candidates(
        timeout=0.2,
        max_hosts_per_network=0,
        fallback_scan_if_empty=False,
        arp_scan_reports=reports,
    )

    assert candidates == []
    assert len(reports) == 1
    assert reports[0].returncode == 1
    assert "permission" in reports[0].error


def test_discover_rtsp_candidates_accepts_host_hints(monkeypatch) -> None:
    monkeypatch.setattr(
        network,
        "local_ipv4_interfaces",
        lambda: [network.InterfaceCandidate("enx0", network.ip_interface("10.0.0.10/24"))],
    )
    monkeypatch.setattr(network, "_probe_default_rtsp_urls", lambda *_args, **_kwargs: [])
    monkeypatch.setattr(network, "neighbor_ipv4_addresses", lambda: [])
    monkeypatch.setattr(network, "arp_scan_ipv4_addresses", lambda *_args, **_kwargs: [])
    probed_hosts = []

    def fake_check_tcp_port(host, port, **_kwargs):
        probed_hosts.append((host, port))
        return host == "10.0.0.231" and port == 8554

    monkeypatch.setattr(network, "check_tcp_port", fake_check_tcp_port)

    candidates = network.discover_rtsp_candidates(
        timeout=0.2,
        max_hosts_per_network=0,
        fallback_scan_if_empty=False,
        host_hints=("10.0.0.231",),
    )

    assert [candidate.url for candidate in candidates] == ["rtsp://10.0.0.231:8554/visible"]
    assert ("10.0.0.231", 8554) in probed_hosts


def test_discover_host_interface_candidates_prefers_neighbor_networks(monkeypatch) -> None:
    def fake_run(args, **_kwargs):
        if args[:5] == ["ip", "-o", "-4", "addr", "show"]:
            return SimpleNamespace(
                returncode=0,
                stdout=(
                    "2: enx0    inet 10.41.10.1/24 brd 10.41.10.255 scope global enx0\n"
                    "2: enx0    inet 10.0.0.10/24 brd 10.0.0.255 scope global enx0\n"
                    "2: enx0    inet 192.168.0.254/24 brd 192.168.0.255 scope global enx0\n"
                    "3: wlo1    inet 100.100.113.89/24 brd 100.100.113.255 scope global wlo1\n"
                ),
                stderr="",
            )
        if args[:3] == ["ip", "-4", "neigh"]:
            return SimpleNamespace(
                returncode=0,
                stdout=(
                    "192.168.0.3 dev enx0 lladdr 00:11:22:33:44:55 REACHABLE\n"
                    "10.41.10.2 dev enx0 lladdr 00:11:22:33:44:56 REACHABLE\n"
                    "10.0.0.230 dev enx0 lladdr 00:11:22:33:44:57 REACHABLE\n"
                ),
                stderr="",
            )
        raise AssertionError(args)

    monkeypatch.setattr(network.subprocess, "run", fake_run)

    candidates = network.discover_host_interface_candidates()

    assert [str(candidate.interface.address) for candidate in candidates] == [
        "10.0.0.10/24",
        "10.41.10.1/24",
        "192.168.0.254/24",
    ]
    assert candidates[0].neighbor_hosts == ("10.0.0.230",)
    assert candidates[1].neighbor_hosts == ("10.41.10.2",)
    assert candidates[2].neighbor_hosts == ("192.168.0.3",)


def test_discover_rtsp_candidates_ignores_neighbors_outside_local_interfaces(monkeypatch) -> None:
    calls = []

    def fake_run(args, **_kwargs):
        if args[:5] == ["ip", "-o", "-4", "addr", "show"]:
            return SimpleNamespace(
                returncode=0,
                stdout="2: enx0    inet 10.0.0.10/24 brd 10.0.0.255 scope global enx0\n",
                stderr="",
            )
        if args[:3] == ["ip", "-4", "neigh"]:
            return SimpleNamespace(
                returncode=0,
                stdout=(
                    "10.0.0.230 dev enx0 lladdr 00:11:22:33:44:55 REACHABLE\n"
                    "100.100.113.26 dev wlo1 lladdr 00:11:22:33:44:56 REACHABLE\n"
                ),
                stderr="",
            )
        raise AssertionError(args)

    def fake_check_tcp_port(host, port, *, timeout):
        calls.append((host, port))
        return host == "10.0.0.230" and port == 8554

    monkeypatch.setattr(network.subprocess, "run", fake_run)
    monkeypatch.setattr(network, "check_tcp_port", fake_check_tcp_port)

    candidates = network.discover_rtsp_candidates(timeout=0.2, max_hosts_per_network=0, fallback_scan_if_empty=False)

    assert [candidate.url for candidate in candidates] == ["rtsp://10.0.0.230:8554/visible"]
    assert calls == [
        ("10.0.0.231", 8554),
        ("10.0.0.232", 8554),
        ("10.0.0.230", 8554),
        ("10.0.0.230", 8554),
    ]


def test_discover_host_interface_candidates_falls_back_to_local_interfaces(monkeypatch) -> None:
    def fake_run(args, **_kwargs):
        if args[:5] == ["ip", "-o", "-4", "addr", "show"]:
            return SimpleNamespace(
                returncode=0,
                stdout="2: enx0    inet 192.168.0.254/24 brd 192.168.0.255 scope global enx0\n",
                stderr="",
            )
        if args[:3] == ["ip", "-4", "neigh"]:
            return SimpleNamespace(returncode=0, stdout="", stderr="")
        raise AssertionError(args)

    monkeypatch.setattr(network.subprocess, "run", fake_run)

    candidates = network.discover_host_interface_candidates()

    assert len(candidates) == 1
    assert str(candidates[0].interface.address) == "192.168.0.254/24"
    assert candidates[0].neighbor_hosts == ()


def test_discover_host_interface_candidates_can_include_unconfigured_wired_links(monkeypatch) -> None:
    def fake_run(args, **_kwargs):
        if args[:5] == ["ip", "-o", "-4", "addr", "show"]:
            return SimpleNamespace(returncode=0, stdout="", stderr="")
        if args[:3] == ["ip", "-4", "neigh"]:
            return SimpleNamespace(returncode=0, stdout="", stderr="")
        if args[:4] == ["ip", "-br", "link", "show"]:
            return SimpleNamespace(
                returncode=0,
                stdout=(
                    "lo               UNKNOWN        00:00:00:00:00:00\n"
                    "wlp0s20f3        UP             00:11:22:33:44:55\n"
                    "enx1a2b3c4d5e6f  DOWN           58:86:94:fc:e0:7e\n"
                ),
                stderr="",
            )
        raise AssertionError(args)

    monkeypatch.setattr(network.subprocess, "run", fake_run)

    candidates = network.discover_host_interface_candidates(
        host_address="10.0.0.10/24",
        include_unconfigured=True,
    )

    assert len(candidates) == 1
    assert candidates[0].interface.name == "enx1a2b3c4d5e6f"
    assert str(candidates[0].interface.address) == "10.0.0.10/24"


def test_discover_rtsp_candidates_probes_default_urls_before_full_fallback(monkeypatch) -> None:
    calls = []

    def fake_run(args, **_kwargs):
        if args[:5] == ["ip", "-o", "-4", "addr", "show"]:
            return SimpleNamespace(
                returncode=0,
                stdout="2: enx0    inet 10.0.0.10/24 brd 10.0.0.255 scope global enx0\n",
                stderr="",
            )
        if args[:3] == ["ip", "-4", "neigh"]:
            return SimpleNamespace(returncode=0, stdout="", stderr="")
        raise AssertionError(args)

    def fake_check_tcp_port(host, port, *, timeout):
        calls.append((host, port, timeout))
        return host in {"10.0.0.231", "10.0.0.232"} and port == 8554

    monkeypatch.setattr(network.subprocess, "run", fake_run)
    monkeypatch.setattr(network, "check_tcp_port", fake_check_tcp_port)

    candidates = network.discover_rtsp_candidates(timeout=0.2, max_hosts_per_network=0)

    assert [candidate.url for candidate in candidates] == [
        "rtsp://10.0.0.231:8554/visible",
        "rtsp://10.0.0.232:8554/visible",
    ]
    assert calls == [
        ("10.0.0.231", 8554, 0.2),
        ("10.0.0.232", 8554, 0.2),
        ("10.0.0.230", 8554, 0.2),
    ]


def test_discover_rtsp_candidates_continues_fallback_until_two_candidates(monkeypatch) -> None:
    def fake_run(args, **_kwargs):
        if args[:5] == ["ip", "-o", "-4", "addr", "show"]:
            return SimpleNamespace(
                returncode=0,
                stdout="2: enx0    inet 10.0.0.10/24 brd 10.0.0.255 scope global enx0\n",
                stderr="",
            )
        if args[:3] == ["ip", "-4", "neigh"]:
            return SimpleNamespace(returncode=0, stdout="10.0.0.42 dev enx0 lladdr 00:11:22:33:44:55 REACHABLE\n", stderr="")
        raise AssertionError(args)

    def fake_check_tcp_port(host, port, *, timeout):
        return (host, port) in {("10.0.0.231", 8554), ("10.0.0.42", 8554)}

    monkeypatch.setattr(network.subprocess, "run", fake_run)
    monkeypatch.setattr(network, "check_tcp_port", fake_check_tcp_port)

    candidates = network.discover_rtsp_candidates(timeout=0.2, max_hosts_per_network=0)

    assert [candidate.url for candidate in candidates] == [
        "rtsp://10.0.0.231:8554/visible",
        "rtsp://10.0.0.42:8554/visible",
    ]
