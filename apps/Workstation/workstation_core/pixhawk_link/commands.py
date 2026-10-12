"""Command transactions using the transport's existing connection and locks.

This service owns command progress state, never a socket, receive timer or
second send lock. PARAM reserves reception while holding the send lock only
for its request. Shell/FTP keep their existing caller-owned reservation API.
"""

from __future__ import annotations

from contextlib import AbstractContextManager
from dataclasses import dataclass, fields
import time
from typing import Any, Callable

from . import wire_commands
from .endpoints import camera_label_ko
from .policy import MAVLINK_RTCM_STATUS_INTERVAL_S
from .types import MavlinkShellCommandResult


@dataclass(frozen=True)
class CommandIO:
    """Borrowed handles/callbacks for one command on an existing connection."""

    connection: Any
    send_lock: Any
    on_unmatched: Callable[[Any], None]
    on_receive_poll: Callable[[], None]


@dataclass(frozen=True)
class CommandPorts:
    io_for: Callable[[str], CommandIO]
    reserve: Callable[..., AbstractContextManager]
    health: Callable[..., Any]

    def __post_init__(self) -> None:
        for field in fields(self):
            if not callable(getattr(self, field.name)):
                raise TypeError(f"Missing mandatory command operation: {field.name}")


class MavlinkCommands:
    def __init__(self, ports: CommandPorts, *, status: Callable[[str], None]) -> None:
        if not callable(status):
            raise TypeError("Missing mandatory command status callback")
        self.ports = ports
        self.status = status
        self.last_rtcm_status_emit_s: dict[str, float] = {}

    def clear(self, camera: str) -> None:
        self.last_rtcm_status_emit_s.pop(str(camera), None)

    def read_param_int(self, camera: str, name: str, *, timeout_s: float = 2.0) -> int | None:
        with self.ports.reserve(camera, hold_send_lock=False):
            io = self.ports.io_for(camera)
            return wire_commands._mavlink_read_param_int(
                io.connection,
                name,
                timeout_s=timeout_s,
                on_unmatched=io.on_unmatched,
                on_receive_poll=io.on_receive_poll,
                send_lock=io.send_lock,
            )

    def read_param(self, camera: str, name: str, *, timeout_s: float = 2.0) -> int | float | None:
        with self.ports.reserve(camera, hold_send_lock=False):
            io = self.ports.io_for(camera)
            return wire_commands._mavlink_read_param(
                io.connection,
                name,
                timeout_s=timeout_s,
                on_unmatched=io.on_unmatched,
                on_receive_poll=io.on_receive_poll,
                send_lock=io.send_lock,
            )

    def set_param_int(self, camera: str, name: str, value: int, *, timeout_s: float = 2.0) -> int | None:
        with self.ports.reserve(camera, hold_send_lock=False):
            io = self.ports.io_for(camera)
            actual = wire_commands._mavlink_set_param_int(
                io.connection,
                name,
                int(value),
                timeout_s=timeout_s,
                on_unmatched=io.on_unmatched,
                on_receive_poll=io.on_receive_poll,
                send_lock=io.send_lock,
            )
        if actual == int(value):
            self.status(f"{camera_label_ko(camera)} PX4 param 설정 확인: {name}={actual}")
        else:
            self.status(
                f"{camera_label_ko(camera)} PX4 param 설정 불일치: {name} expected={int(value)} actual={actual}"
            )
        return actual

    def send_rtcm_data(self, camera: str, payload: bytes) -> int:
        io = self.ports.io_for(camera)
        with io.send_lock:
            packet_count = wire_commands._send_mavlink_rtcm_data(io.connection, payload)
        self._emit_rtcm_status_if_due(camera, payload, packet_count)
        return packet_count

    def send_rtcm_data_if_eligible(
        self,
        camera: str,
        payload: bytes,
        *,
        expected_generation: int | None = None,
    ) -> dict[str, object]:
        camera = str(camera)
        io = self.ports.io_for(camera)
        with io.send_lock:
            health = self.ports.health(camera, now_s=time.monotonic())
            if expected_generation is not None and health.connection_generation != int(expected_generation):
                return {
                    "sent": False,
                    "packet_count": 0,
                    "health": health,
                    "block_reason": "connection-generation-changed",
                }
            if not health.rtcm_eligible:
                return {
                    "sent": False,
                    "packet_count": 0,
                    "health": health,
                    "block_reason": "target-mavlink-ineligible",
                }
            packet_count = int(wire_commands._send_mavlink_rtcm_data(io.connection, payload) or 0)
        self._emit_rtcm_status_if_due(camera, payload, packet_count)
        return {"sent": True, "packet_count": packet_count, "health": health, "block_reason": ""}

    def _emit_rtcm_status_if_due(self, camera: str, payload: bytes, packet_count: int) -> None:
        camera = str(camera)
        now_s = time.monotonic()
        last_s = self.last_rtcm_status_emit_s.get(camera)
        if last_s is not None and now_s - last_s < MAVLINK_RTCM_STATUS_INTERVAL_S:
            return
        self.last_rtcm_status_emit_s[camera] = now_s
        self.status(f"{camera_label_ko(camera)} RTCM 주입: {len(payload)} bytes / {int(packet_count)} MAVLink packets")

    def exchange_shell(
        self,
        camera: str,
        command: str,
        *,
        timeout_s: float = 2.0,
        lock_sends: bool = False,
    ) -> MavlinkShellCommandResult:
        io = self.ports.io_for(camera)
        return wire_commands._send_mavlink_shell_command(
            io.connection,
            command,
            timeout_s=timeout_s,
            on_unmatched=io.on_unmatched,
            on_receive_poll=io.on_receive_poll,
            send_lock=io.send_lock if lock_sends else None,
        )

    def _report_shell_result(self, camera: str, command: str, result: MavlinkShellCommandResult) -> None:
        if result.flag_label != "respond+exclusive":
            self.status(f"{camera_label_ko(camera)} PX4 shell fallback 사용: {result.flag_label}")
        self.status(f"{camera_label_ko(camera)} PX4 shell 명령 전송: {command}")

    def send_shell_command(self, camera: str, command: str) -> None:
        result = self.exchange_shell(camera, command)
        self._report_shell_result(camera, command, result)

    def send_shell_command_with_result(
        self,
        camera: str,
        command: str,
        *,
        timeout_s: float = 2.0,
    ) -> MavlinkShellCommandResult:
        result = self.exchange_shell(camera, command, timeout_s=timeout_s, lock_sends=True)
        self._report_shell_result(camera, command, result)
        return result

    def write_remote_file_mavftp(self, camera: str, remote_path: str, content: str | bytes) -> None:
        io = self.ports.io_for(camera)
        wire_commands._write_mavlink_ftp_file(io.connection, remote_path, content)
        self.status(f"{camera_label_ko(camera)} MAVLink FTP 파일 저장: {remote_path}")

    def reboot_autopilot(self, camera: str) -> None:
        connection = self.ports.io_for(camera).connection
        mav = getattr(connection, "mav", None)
        if mav is None or not hasattr(mav, "command_long_send"):
            raise RuntimeError("MAVLink COMMAND_LONG을 지원하지 않는 연결입니다.")
        mavlink = getattr(connection, "mavlink", None)
        command_id = wire_commands._mavlink_constant(mavlink, "MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN", 246)
        target_system = int(getattr(connection, "target_system", 0) or 1)
        target_component = int(getattr(connection, "target_component", 0) or 1)
        mav.command_long_send(target_system, target_component, command_id, 0, 1, 0, 0, 0, 0, 0, 0)
        self.status(f"{camera_label_ko(camera)} PX4 MAVLink 재부팅 명령 전송")
