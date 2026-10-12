"""MAVLink command wire operations on an explicitly supplied connection.

No connection, clock epoch, thread or resource lifecycle is owned here.
The runtime supplies send locks and callbacks for unmatched telemetry and
heartbeat polling; request bytes and receive/timeout semantics are preserved.
"""

from __future__ import annotations

import io
import struct
import time
from contextlib import nullcontext
from pathlib import Path
from typing import Any, Callable
from .types import MavlinkShellCommandResult
from .policy import (
    _NSH_PROMPT_TAIL_RE,
    MAVLINK_COMMAND_RECEIVE_POLL_S,
    MAVLINK_SHELL_OUTPUT_IDLE_S,
    _MAVLINK_FLOAT_PARAM_TYPES,
)


def _serial_control_output_from_message(message: Any) -> str:
    try:
        count = int(getattr(message, "count", 0) or 0)
        data = bytes(getattr(message, "data", [])[:count])
    except Exception:
        return ""
    return data.decode("utf-8", errors="replace")


def _mavlink_message_type(message: Any) -> str:
    getter = getattr(message, "get_type", None)
    if callable(getter):
        try:
            return str(getter() or "")
        except Exception:
            return ""
    return str(getattr(message, "type", "") or "")


def _shell_output_ends_with_prompt(chunks: list[str]) -> bool:
    """True when the accumulated NSH output ends with the ``nsh>`` prompt."""

    return _NSH_PROMPT_TAIL_RE.search("".join(chunks[-3:])) is not None


def _read_mavlink_shell_output(
    connection: Any,
    *,
    timeout_s: float,
    on_unmatched: Callable[[Any], None] | None = None,
    on_receive_poll: Callable[[], None] | None = None,
    idle_timeout_s: float | None = MAVLINK_SHELL_OUTPUT_IDLE_S,
    stop_at_prompt: bool = True,
) -> str:
    """Collect SERIAL_CONTROL output until the prompt, an idle gap, or ``timeout_s``.

    ``timeout_s`` stays the hard upper bound. ``idle_timeout_s`` ends the read
    once output has started and then paused, and ``stop_at_prompt`` ends it as
    soon as the NSH prompt is back; pass ``None``/``False`` to wait the full
    timeout as before.
    """

    recv_match = getattr(connection, "recv_match", None)
    if recv_match is None:
        return ""
    deadline = time.monotonic() + max(0.0, float(timeout_s))
    chunks: list[str] = []
    last_output_s: float | None = None
    while time.monotonic() < deadline:
        if (
            idle_timeout_s is not None
            and last_output_s is not None
            and time.monotonic() - last_output_s >= float(idle_timeout_s)
        ):
            break
        if on_receive_poll is not None:
            on_receive_poll()
        message = recv_match(blocking=True, timeout=MAVLINK_COMMAND_RECEIVE_POLL_S)
        if on_receive_poll is not None:
            on_receive_poll()
        if message is None:
            continue
        if _mavlink_message_type(message) != "SERIAL_CONTROL":
            if on_unmatched is not None:
                on_unmatched(message)
            continue
        text = _serial_control_output_from_message(message)
        if text:
            chunks.append(text)
            last_output_s = time.monotonic()
            if stop_at_prompt and _shell_output_ends_with_prompt(chunks):
                break
    return "".join(chunks)


def _send_mavlink_shell_payload(mav: Any, *, device: int, flags: int, payload: bytes) -> None:
    for offset in range(0, len(payload), 70):
        chunk = payload[offset : offset + 70]
        data = list(chunk) + [0] * (70 - len(chunk))
        mav.serial_control_send(device, flags, 0, 0, len(chunk), data)


def _send_mavlink_shell_command(
    connection: Any,
    command: str,
    *,
    timeout_s: float = 0.8,
    on_unmatched: Callable[[Any], None] | None = None,
    on_receive_poll: Callable[[], None] | None = None,
    send_lock: Any = None,
) -> MavlinkShellCommandResult:
    """Run one NSH command over SERIAL_CONTROL.

    ``send_lock`` (the side's re-entrant MAVLink send lock) is held only around
    each write, never across the blocking reads, so RTCM injection and GCS
    heartbeats on the same link keep flowing while the reply is awaited.
    """

    mav = getattr(connection, "mav", None)
    if mav is None or not hasattr(mav, "serial_control_send"):
        raise RuntimeError("MAVLink serial control을 지원하지 않는 연결입니다.")
    write_lock = send_lock if send_lock is not None else nullcontext()
    # A multi-line payload returns one prompt per line; only a single command
    # may treat the first prompt as the end of its reply.
    stop_at_prompt = "\n" not in command.strip()
    mavlink = getattr(connection, "mavlink", None)
    device = _mavlink_constant(mavlink, "SERIAL_CONTROL_DEV_SHELL", 10)
    respond_flag = _mavlink_constant(mavlink, "SERIAL_CONTROL_FLAG_RESPOND", 2)
    exclusive_flag = _mavlink_constant(mavlink, "SERIAL_CONTROL_FLAG_EXCLUSIVE", 4)
    variants = (
        ("respond+exclusive", respond_flag | exclusive_flag),
        ("respond", respond_flag),
    )
    try:
        payload = (command.rstrip() + "\n").encode("utf-8")
        selected_label, selected_flags = variants[-1]
        probe_output = ""
        for label, flags in variants:
            # Opening the shell first matters on PX4 NSH links that only respond after a read request.
            with write_lock:
                mav.serial_control_send(device, flags, 0, 0, 0, [0] * 70)
            output = _read_mavlink_shell_output(
                connection,
                timeout_s=min(timeout_s, 0.25),
                on_unmatched=on_unmatched,
                on_receive_poll=on_receive_poll,
            )
            if output or label == variants[-1][0]:
                selected_label = label
                selected_flags = flags
                probe_output = output
                break
        with write_lock:
            _send_mavlink_shell_payload(mav, device=device, flags=selected_flags, payload=payload)
        command_output = _read_mavlink_shell_output(
            connection,
            timeout_s=timeout_s,
            on_unmatched=on_unmatched,
            on_receive_poll=on_receive_poll,
            stop_at_prompt=stop_at_prompt,
        )
        return MavlinkShellCommandResult(
            command=command,
            output=probe_output + command_output,
            flag_label=selected_label,
        )
    finally:
        # SERIAL_CONTROL_FLAG_EXCLUSIVE must be explicitly released. Closing
        # the host-side serial descriptor alone does not hand the PX4 shell
        # back and can leave later USB diagnostics with a dead-looking link.
        try:
            with write_lock:
                mav.serial_control_send(device, 0, 0, 0, 0, [0] * 70)
        except Exception:
            pass


def _send_mavlink_rtcm_data(connection: Any, payload: bytes) -> int:
    mav = getattr(connection, "mav", None)
    if mav is None or not hasattr(mav, "gps_rtcm_data_send"):
        raise RuntimeError("MAVLink GPS_RTCM_DATA를 지원하지 않는 연결입니다.")
    data = bytes(payload)
    if not data:
        return 0
    max_chunk = 180
    max_fragmented_payload = max_chunk * 4
    if len(data) > max_fragmented_payload:
        raise ValueError(
            f"RTCM frame too large for MAVLink GPS_RTCM_DATA: {len(data)} > {max_fragmented_payload} bytes"
        )
    chunks = [data[offset : offset + max_chunk] for offset in range(0, len(data), max_chunk)]
    sequence_id = int(time.monotonic() * 1000) & 0x1F
    fragmented = len(chunks) > 1
    for fragment_id, chunk in enumerate(chunks):
        flags = 0
        if fragmented:
            flags = 1 | ((fragment_id & 0x03) << 1) | ((sequence_id & 0x1F) << 3)
        mav.gps_rtcm_data_send(flags, len(chunk), list(chunk) + [0] * (max_chunk - len(chunk)))
    return len(chunks)


def _write_mavlink_ftp_file(connection: Any, remote_path: str, content: str | bytes) -> None:
    try:
        from pymavlink import mavftp
        from pymavlink.mavftp import FtpError
    except Exception as exc:
        raise RuntimeError(f"pymavlink MAVFTP를 사용할 수 없습니다. ({exc})") from exc
    if not remote_path.startswith("/"):
        raise ValueError(f"MAVLink FTP remote path must be absolute: {remote_path}")
    payload = content.encode("utf-8") if isinstance(content, str) else bytes(content)
    target_system = int(getattr(connection, "target_system", 0) or 1)
    target_component = int(getattr(connection, "target_component", 0) or 1)
    ftp = mavftp.MAVFTP(connection, target_system, target_component)
    directory = str(Path(remote_path).parent)
    if directory and directory not in {"/", "/fs/microsd"}:
        mkdir_result = ftp.cmd_mkdir([directory])
        if mkdir_result.return_code not in (FtpError.Success, FtpError.FileExists):
            raise RuntimeError(f"MAVLink FTP mkdir 실패: {directory} ({mkdir_result.return_code})")
    rm_result = ftp.cmd_rm([remote_path])
    if rm_result.return_code not in (FtpError.Success, FtpError.FileNotFound):
        raise RuntimeError(f"MAVLink FTP 기존 파일 삭제 실패: {remote_path} ({rm_result.return_code})")
    handle = io.BytesIO(payload)
    create_result = ftp.cmd_put(["extras.txt", remote_path], fh=handle)
    if create_result.return_code != FtpError.Success:
        raise RuntimeError(f"MAVLink FTP 파일 생성 실패: {remote_path} ({create_result.return_code})")
    write_result = ftp.process_ftp_reply("WriteFile", timeout=8)
    if write_result.return_code != FtpError.Success:
        raise RuntimeError(f"MAVLink FTP 파일 쓰기 실패: {remote_path} ({write_result.return_code})")


def _mavlink_param_id(name: str) -> bytes:
    encoded = name.encode("ascii", errors="strict")
    if len(encoded) > 16:
        raise ValueError(f"MAVLink PARAM id is longer than 16 bytes: {name}")
    return encoded


def _mavlink_param_id_text(message: Any) -> str:
    raw = getattr(message, "param_id", "")
    if isinstance(raw, bytes):
        return raw.rstrip(b"\x00").decode("ascii", errors="replace")
    return str(raw).rstrip("\x00")


def _mavlink_int_to_param_float(value: int) -> float:
    return struct.unpack("<f", struct.pack("<I", int(value) & 0xFFFFFFFF))[0]


def _mavlink_param_float_to_int(value: Any) -> int:
    return struct.unpack("<I", struct.pack("<f", float(value)))[0]


def _mavlink_recv_param_value_loop(
    connection: Any,
    name: str,
    *,
    timeout_s: float,
    decode: Callable[[float, int | None], int | float],
    on_unmatched: Callable[[Any], None] | None = None,
    on_receive_poll: Callable[[], None] | None = None,
) -> int | float | None:
    deadline = time.monotonic() + float(timeout_s)
    while time.monotonic() < deadline:
        remaining = max(0.0, deadline - time.monotonic())
        if on_receive_poll is not None:
            on_receive_poll()
        message = connection.recv_match(
            blocking=True,
            timeout=min(MAVLINK_COMMAND_RECEIVE_POLL_S, remaining),
        )
        if on_receive_poll is not None:
            on_receive_poll()
        if message is None:
            continue
        if _mavlink_message_type(message) != "PARAM_VALUE" or _mavlink_param_id_text(message) != name:
            if on_unmatched is not None:
                on_unmatched(message)
            continue
        return decode(getattr(message, "param_value"), getattr(message, "param_type", None))
    return None


def _mavlink_recv_param_value(
    connection: Any,
    name: str,
    *,
    timeout_s: float,
    on_unmatched: Callable[[Any], None] | None = None,
    on_receive_poll: Callable[[], None] | None = None,
) -> int | None:
    return _mavlink_recv_param_value_loop(
        connection,
        name,
        timeout_s=timeout_s,
        decode=lambda raw_param_value, _param_type: _mavlink_param_float_to_int(raw_param_value),
        on_unmatched=on_unmatched,
        on_receive_poll=on_receive_poll,
    )


def _mavlink_decode_param_value(raw_param_value: float, param_type: int | None) -> int | float:
    if param_type in _MAVLINK_FLOAT_PARAM_TYPES:
        return float(raw_param_value)
    return _mavlink_param_float_to_int(raw_param_value)


def _mavlink_recv_param_value_typed(
    connection: Any,
    name: str,
    *,
    timeout_s: float,
    on_unmatched: Callable[[Any], None] | None = None,
    on_receive_poll: Callable[[], None] | None = None,
) -> int | float | None:
    return _mavlink_recv_param_value_loop(
        connection,
        name,
        timeout_s=timeout_s,
        decode=_mavlink_decode_param_value,
        on_unmatched=on_unmatched,
        on_receive_poll=on_receive_poll,
    )


def _mavlink_read_param(
    connection: Any,
    name: str,
    *,
    timeout_s: float = 2.0,
    on_unmatched: Callable[[Any], None] | None = None,
    on_receive_poll: Callable[[], None] | None = None,
    send_lock: Any = None,
) -> int | float | None:
    mav = getattr(connection, "mav", None)
    if mav is None or not hasattr(mav, "param_request_read_send"):
        return None
    target_system = int(getattr(connection, "target_system", 0) or 1)
    target_component = int(getattr(connection, "target_component", 0) or 1)
    with send_lock if send_lock is not None else nullcontext():
        mav.param_request_read_send(target_system, target_component, _mavlink_param_id(name), -1)
    return _mavlink_recv_param_value_typed(
        connection,
        name,
        timeout_s=timeout_s,
        on_unmatched=on_unmatched,
        on_receive_poll=on_receive_poll,
    )


def _mavlink_read_param_int(
    connection: Any,
    name: str,
    *,
    timeout_s: float = 2.0,
    on_unmatched: Callable[[Any], None] | None = None,
    on_receive_poll: Callable[[], None] | None = None,
    send_lock: Any = None,
) -> int | None:
    if on_unmatched is None and hasattr(connection, "param_fetch_one"):
        try:
            value = connection.param_fetch_one(name, timeout=timeout_s)
        except TypeError:
            value = connection.param_fetch_one(name)
        except Exception:
            value = None
        if value is not None:
            try:
                return _mavlink_param_float_to_int(value)
            except Exception:
                try:
                    return int(value)
                except Exception:
                    pass
    mav = getattr(connection, "mav", None)
    if mav is None or not hasattr(mav, "param_request_read_send"):
        return None
    target_system = int(getattr(connection, "target_system", 0) or 1)
    target_component = int(getattr(connection, "target_component", 0) or 1)
    with send_lock if send_lock is not None else nullcontext():
        mav.param_request_read_send(target_system, target_component, _mavlink_param_id(name), -1)
    return _mavlink_recv_param_value(
        connection,
        name,
        timeout_s=timeout_s,
        on_unmatched=on_unmatched,
        on_receive_poll=on_receive_poll,
    )


def _mavlink_set_param_int(
    connection: Any,
    name: str,
    value: int,
    *,
    timeout_s: float = 2.0,
    on_unmatched: Callable[[Any], None] | None = None,
    on_receive_poll: Callable[[], None] | None = None,
    send_lock: Any = None,
) -> int | None:
    mav = getattr(connection, "mav", None)
    if mav is None or not hasattr(mav, "param_set_send"):
        raise RuntimeError("MAVLink PARAM_SET을 지원하지 않는 연결입니다.")
    mavlink = getattr(connection, "mavlink", None)
    target_system = int(getattr(connection, "target_system", 0) or 1)
    target_component = int(getattr(connection, "target_component", 0) or 1)
    param_type = _mavlink_constant(mavlink, "MAV_PARAM_TYPE_INT32", 6)
    with send_lock if send_lock is not None else nullcontext():
        mav.param_set_send(
            target_system,
            target_component,
            _mavlink_param_id(name),
            _mavlink_int_to_param_float(int(value)),
            param_type,
        )
    return _mavlink_recv_param_value(
        connection,
        name,
        timeout_s=timeout_s,
        on_unmatched=on_unmatched,
        on_receive_poll=on_receive_poll,
    )


def _decode_mavlink_serial_control(message: Any) -> str:
    count = int(getattr(message, "count", 0) or 0)
    data = getattr(message, "data", b"")
    if isinstance(data, bytes):
        payload = data[:count]
    else:
        payload = bytes(int(value) & 0xFF for value in list(data)[:count])
    return payload.replace(b"\x00", b"").decode("utf-8", errors="replace")


def _mavlink_constant(mavlink: Any, name: str, fallback: int) -> int:
    if mavlink is not None and hasattr(mavlink, name):
        return int(getattr(mavlink, name))
    return fallback
