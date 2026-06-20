#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path


DEFAULT_SETTINGS = Path("stereo_distance_settings.json")
DEFAULT_ROS_SETUP = Path("/opt/ros/humble/setup.bash")
DEFAULT_PX4_SETUP = Path(__file__).resolve().parents[2] / "workspace" / "main" / "install" / "setup.bash"
DEFAULT_AGENT_LOG = Path("/tmp/stereo_microxrceagent.log")
PX4_TOPIC_SUFFIXES = (
    "vehicle_attitude",
    "sensor_mag",
    "vehicle_magnetometer",
    "sensor_gps",
)


def _load_settings(path: Path) -> dict[str, object]:
    if not path.exists():
        return {}
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception as exc:
        return {"_error": f"{type(exc).__name__}: {exc}"}


def _run(command: list[str], *, timeout_s: float = 5.0, env: dict[str, str] | None = None) -> subprocess.CompletedProcess[str]:
    try:
        return subprocess.run(command, check=False, capture_output=True, text=True, timeout=timeout_s, env=env)
    except subprocess.TimeoutExpired as exc:
        return subprocess.CompletedProcess(command, 124, exc.stdout or "", exc.stderr or "timeout")
    except FileNotFoundError as exc:
        return subprocess.CompletedProcess(command, 127, "", str(exc))


def _bash_ros(command: str, *, domain_id: int, timeout_s: float = 8.0) -> subprocess.CompletedProcess[str]:
    setup_lines = []
    if DEFAULT_ROS_SETUP.exists():
        setup_lines.append(f"source {DEFAULT_ROS_SETUP}")
    if DEFAULT_PX4_SETUP.exists():
        setup_lines.append(f"source {DEFAULT_PX4_SETUP}")
    setup_lines.append(f"export ROS_DOMAIN_ID={int(domain_id)}")
    setup_lines.append(command)
    return _run(["/bin/bash", "-lc", "; ".join(setup_lines)], timeout_s=timeout_s)


def _print_section(title: str) -> None:
    print(f"\n## {title}")


def _print_command_result(result: subprocess.CompletedProcess[str], *, max_lines: int = 80) -> None:
    output = "\n".join(part for part in (result.stdout.strip(), result.stderr.strip()) if part)
    lines = output.splitlines()
    if not lines:
        print("(no output)")
        return
    for line in lines[:max_lines]:
        print(line)
    if len(lines) > max_lines:
        print(f"... {len(lines) - max_lines} more lines")


def _setting(settings: dict[str, object], key: str, default: object) -> object:
    return settings.get(key, default)


def _host_ip(text: str) -> str:
    return text.split("/", 1)[0].strip()


def _mavlink_heartbeat(url: str, baud: int, timeout_s: float) -> tuple[bool, str]:
    if not url.strip():
        return False, "empty URL"
    try:
        from pymavlink import mavutil
    except Exception as exc:
        return False, f"pymavlink unavailable: {exc}"
    connection = None
    try:
        connection = mavutil.mavlink_connection(
            url.strip(),
            baud=int(baud),
            autoreconnect=True,
            robust_parsing=True,
        )
        heartbeat = connection.wait_heartbeat(timeout=timeout_s)
        if heartbeat is None:
            return False, "heartbeat timeout"
        return True, f"heartbeat OK sysid={connection.target_system} compid={connection.target_component}"
    except Exception as exc:
        return False, f"{type(exc).__name__}: {exc}"
    finally:
        if connection is not None:
            try:
                connection.close()
            except Exception:
                pass


def _ros_topic_info(topic: str, *, domain_id: int) -> tuple[int | None, str]:
    result = _bash_ros(f"timeout 6 ros2 topic info -v {topic}", domain_id=domain_id, timeout_s=8.0)
    text = "\n".join(part for part in (result.stdout, result.stderr) if part)
    publisher_count: int | None = None
    for line in text.splitlines():
        if line.strip().startswith("Publisher count:"):
            try:
                publisher_count = int(line.split(":", 1)[1].strip())
            except ValueError:
                publisher_count = None
            break
    return publisher_count, text.strip()


def _ros_topic_hz(topic: str, *, domain_id: int, timeout_s: float) -> tuple[bool, str]:
    result = _bash_ros(
        f"timeout {max(1, int(timeout_s))} ros2 topic hz {topic} --window 3",
        domain_id=domain_id,
        timeout_s=timeout_s + 2.0,
    )
    text = "\n".join(part for part in (result.stdout, result.stderr) if part).strip()
    return "average rate:" in text, text


def _ros_topic_names(*, domain_id: int) -> list[str]:
    result = _bash_ros("timeout 6 ros2 topic list -t", domain_id=domain_id, timeout_s=8.0)
    topics: list[str] = []
    for line in result.stdout.splitlines():
        name = line.split(" ", 1)[0].strip()
        if "/fmu/out/" in name:
            topics.append(name)
    return sorted(dict.fromkeys(topics))


def _network_prefix(host_ip: str) -> str | None:
    parts = host_ip.split(".")
    if len(parts) != 4:
        return None
    return ".".join(parts[:3]) + ".0/24"


def _arp_scan(interface: str, host_ip: str) -> subprocess.CompletedProcess[str] | None:
    if not interface or not host_ip or shutil.which("arp-scan") is None:
        return None
    network = _network_prefix(host_ip)
    if network is None:
        return None
    return _run(
        ["arp-scan", f"--interface={interface}", f"--arpspa={host_ip}", network],
        timeout_s=8.0,
    )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Diagnose Pixhawk MAVLink and ROS2/uXRCE connectivity.")
    parser.add_argument("--settings", type=Path, default=DEFAULT_SETTINGS)
    parser.add_argument("--timeout", type=float, default=4.0)
    parser.add_argument("--no-arp-scan", action="store_true")
    args = parser.parse_args(argv)

    settings = _load_settings(args.settings)
    domain_id = int(_setting(settings, "ros_domain_id", 0))
    interface = str(_setting(settings, "interface_name", ""))
    pixhawk_host = _host_ip(str(_setting(settings, "pixhawk_host_address", "10.41.10.1")))
    left_url = str(_setting(settings, "left_mavlink_url", "udpin:0.0.0.0:14550"))
    right_url = str(_setting(settings, "right_mavlink_url", "udpin:0.0.0.0:14551"))
    left_baud = int(_setting(settings, "left_mavlink_baud", 57600))
    right_baud = int(_setting(settings, "right_mavlink_baud", 57600))
    left_ns = str(_setting(settings, "left_namespace", "uav_0")).strip().strip("/")
    right_ns = str(_setting(settings, "right_namespace", "uav_1")).strip().strip("/")
    uxrce_port = int(_setting(settings, "uxrce_port", 8888))

    _print_section("Settings")
    print(f"settings={args.settings}")
    if "_error" in settings:
        print(f"settings_error={settings['_error']}")
    print(f"interface={interface}")
    print(f"pixhawk_host_ip={pixhawk_host}")
    print(f"ros_domain_id={domain_id}")
    print(f"uxrce_port={uxrce_port}")
    print(f"left_namespace={left_ns} left_mavlink={left_url} baud={left_baud}")
    print(f"right_namespace={right_ns} right_mavlink={right_url} baud={right_baud}")

    _print_section("Host Network")
    _print_command_result(_run(["ip", "-br", "addr"], timeout_s=4.0))
    _print_command_result(_run(["ip", "route"], timeout_s=4.0))

    _print_section("Processes And UDP Ports")
    _print_command_result(_run(["ps", "-ef"], timeout_s=4.0), max_lines=12)
    ss = _run(["ss", "-lunp"], timeout_s=4.0)
    filtered = [
        line
        for line in (ss.stdout + ss.stderr).splitlines()
        if any(token in line for token in (":8888", ":14550", ":14551", "MicroXRCEAgent", "python"))
    ]
    print("\n".join(filtered) if filtered else "(no relevant UDP sockets)")

    _print_section("USB MAVLink Candidates")
    usb_patterns = ("/dev/ttyACM*", "/dev/ttyUSB*", "/dev/serial/by-id/*")
    found_usb = False
    import glob

    for pattern in usb_patterns:
        for path in sorted(glob.glob(pattern)):
            found_usb = True
            print(path)
    if not found_usb:
        print("(none)")

    _print_section("MAVLink Heartbeat")
    mavlink_results = {}
    for side, url, baud in (("left", left_url, left_baud), ("right", right_url, right_baud)):
        ok, message = _mavlink_heartbeat(url, baud, args.timeout)
        mavlink_results[side] = ok
        print(f"{side}: {'OK' if ok else 'FAIL'} - {message}")

    _print_section("ROS2 PX4 Topics")
    old_style_topics = _ros_topic_names(domain_id=domain_id)
    print(f"topic_names_seen={len(old_style_topics)}")
    for topic in old_style_topics[:24]:
        print(f"name: {topic}")
    ros_results: dict[str, bool] = {}
    for namespace in (left_ns, right_ns):
        label = namespace or "<root>"
        namespace_ok = False
        for suffix in PX4_TOPIC_SUFFIXES:
            topic = f"/{namespace}/fmu/out/{suffix}" if namespace else f"/fmu/out/{suffix}"
            publishers, info = _ros_topic_info(topic, domain_id=domain_id)
            print(f"{label} {suffix}: publishers={publishers}")
            if publishers and publishers > 0:
                hz_ok, hz_text = _ros_topic_hz(topic, domain_id=domain_id, timeout_s=args.timeout)
                print(f"{label} {suffix}: hz={'OK' if hz_ok else 'NO_SAMPLE'}")
                if hz_text:
                    print(hz_text.splitlines()[0])
                namespace_ok = namespace_ok or hz_ok
            elif info:
                first = info.splitlines()[0] if info.splitlines() else ""
                if first:
                    print(first)
        ros_results[label] = namespace_ok

    _print_section("MicroXRCEAgent Log")
    if DEFAULT_AGENT_LOG.exists():
        lines = DEFAULT_AGENT_LOG.read_text(encoding="utf-8", errors="replace").splitlines()
        for line in lines[-80:]:
            print(line)
    else:
        print(f"(missing {DEFAULT_AGENT_LOG})")

    if not args.no_arp_scan:
        _print_section("ARP Scan")
        host_ips = [pixhawk_host]
        for extra in ("10.0.0.10", "10.41.10.1", "192.168.0.254"):
            if extra not in host_ips:
                host_ips.append(extra)
        for host_ip in host_ips:
            result = _arp_scan(interface, host_ip)
            print(f"-- {host_ip} --")
            if result is None:
                print("arp-scan unavailable or invalid interface/host")
            else:
                _print_command_result(result, max_lines=80)

    mavlink_ok = any(mavlink_results.values())
    ros_ok = any(ros_results.values())
    _print_section("Summary")
    print(f"mavlink_any_ok={mavlink_ok}")
    print(f"ros2_any_sample_ok={ros_ok}")
    if mavlink_ok and ros_ok:
        print("result=OK")
        return 0
    print("result=NOT_CONNECTED")
    print("note=Topic names alone are not success; require publishers and samples.")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
