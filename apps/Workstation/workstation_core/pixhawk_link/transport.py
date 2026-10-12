"""Single owner of Pixhawk connections, receive reservations and send locks.

The Qt subscriber supplies explicit message/policy/timer callbacks. This owner
contains no Qt object and never reads a panel or a compatibility facade. The
existing receive clock, generation, lock scope and qualification rules remain.
"""

from __future__ import annotations

from collections import deque
from contextlib import contextmanager, nullcontext
from dataclasses import dataclass, fields
import fcntl
import hashlib
import os
from pathlib import Path
import threading
import time
from typing import Any, Callable

from Workstation.workstation_core.error_throttle import should_emit_throttled_error
from Workstation.workstation_core.mavlink_liveness import (
    MavlinkLivenessState,
    MavlinkSideHealthSnapshot,
    MavlinkSideLiveness,
)
from . import endpoints
from .commands import CommandIO
from .decode import mavlink_message_provenance
from .timebase import _MavlinkBootTimeTracker
from .wire_commands import _mavlink_message_type, _mavlink_constant
from .policy import (
    MAVLINK_EXPECTED_SYSTEM_IDS,
    MAVLINK_GCS_SOURCE_SYSTEM,
    MAVLINK_GCS_SOURCE_COMPONENT,
    MAVLINK_DUPLICATE_CACHE_SIZE,
    MAVLINK_DUPLICATE_TTL_S,
    MAVLINK_GCS_HEARTBEAT_INTERVAL_S,
    MAVLINK_MAX_MESSAGES_PER_TICK,
    MAVLINK_MAX_PROCESS_TIME_S,
    MAVLINK_RECEIVE_ERROR_INTERVAL_S,
)


@dataclass(frozen=True)
class TransportCallbacks:
    status: Callable[[str], None]
    message: Callable[..., None]
    reset_sources: Callable[[str], None]
    reset_stream_requests: Callable[[str], None]
    advance_stream_policy: Callable[..., None]
    apply_pending_setup: Callable[[str, Any], None]
    start_timer: Callable[[], None]
    stop_timer: Callable[[], None]

    def __post_init__(self) -> None:
        for field in fields(self):
            if not callable(getattr(self, field.name)):
                raise TypeError(f"Missing mandatory transport callback: {field.name}")


class MavlinkTransport:
    def __init__(
        self,
        *,
        left_url: str,
        right_url: str,
        left_baud: int = 57600,
        right_baud: int = 57600,
        drone_url: str = "",
        drone_baud: int = 57600,
        expected_system_ids: dict[str, int] | None = None,
        peer_hosts: dict[str, str] | None = None,
        transports: dict[str, str] | None = None,
        callbacks: TransportCallbacks,
    ) -> None:
        self.callbacks = callbacks
        self.urls = {"left": left_url.strip(), "right": right_url.strip(), "drone": drone_url.strip()}
        self.baudrates = {"left": int(left_baud), "right": int(right_baud), "drone": int(drone_baud)}
        self.peer_hosts = {side: "" for side in self.urls}
        if peer_hosts is not None:
            self.peer_hosts.update({str(side): str(host).strip() for (side, host) in peer_hosts.items()})
        self.expected_system_ids = dict(MAVLINK_EXPECTED_SYSTEM_IDS)
        if expected_system_ids is not None:
            self.expected_system_ids.update(
                {str(side): int(system_id) for (side, system_id) in expected_system_ids.items()}
            )
        self.transports = {side: "ethernet" for side in self.urls}
        if transports is not None:
            self.transports.update(
                {str(side): endpoints._normalize_transport(value) for (side, value) in transports.items()}
            )
        self._radio_lock_handles: dict[str, Any] = {}
        self.connections: dict[str, Any] = {}
        self.last_connect_errors: dict[str, str] = {}
        self._mavlink_boot_time_trackers = {side: _MavlinkBootTimeTracker() for side in self.urls}
        self._command_state_lock = threading.RLock()
        self._mavlink_send_locks = {side: threading.RLock() for side in self.urls}
        self._active_command_cameras: set[str] = set()
        self._deferred_command_messages: dict[str, deque[Any]] = {side: deque() for side in self.urls}
        self._recent_raw_packets: dict[str, deque[tuple[float, bytes]]] = {
            side: deque(maxlen=MAVLINK_DUPLICATE_CACHE_SIZE) for side in self.urls
        }
        self._last_receive_errors: dict[str, tuple[str, float]] = {}
        self._last_gcs_heartbeat_errors: dict[str, tuple[str, float]] = {}
        self._gcs_heartbeat_last_attempt_s: dict[str, float] = {}
        self._identity_mismatch_reported: set[tuple[str, int]] = set()
        self._liveness_connection_ids: dict[str, int] = {}
        self._seeded_udp_peers: dict[str, tuple[str, int]] = {}
        self._last_health_states: dict[str, MavlinkLivenessState] = {}
        self._mavlink_liveness = {
            side: MavlinkSideLiveness(
                side=side,
                expected_system_id=self.expected_system_ids[side],
                endpoint=self.urls.get(side, ""),
                peer_host=self.peer_hosts.get(side, ""),
                clock=lambda: time.monotonic(),
            )
            for side in self.urls
        }

    @contextmanager
    def exclusive_command(
        self,
        camera: str,
        *,
        hold_send_lock: bool = True,
        command_label: str = "MAVLINK COMMAND",
        preserve_qualification: bool = False,
    ):
        """Reserve one MAVLink receive path for a blocking command worker.

        ``spin_once`` is normally the sole receiver.  A PARAM or SERIAL_CONTROL
        request must temporarily exclude that timer or the timer can consume
        the command response first.  The reservation is intentionally narrow:
        callers still own the worker/thread and this method does not touch Qt
        objects other than the existing subscriber state.
        """

        camera = str(camera)
        send_lock = self._mavlink_send_locks.setdefault(camera, threading.RLock())
        with send_lock if hold_send_lock else nullcontext():
            with self._command_state_lock:
                if camera not in self.connections:
                    raise RuntimeError(f"{endpoints.camera_label_ko(camera)} MAVLink 연결이 없습니다.")
                if camera in self._active_command_cameras:
                    raise RuntimeError(f"{endpoints.camera_label_ko(camera)} MAVLink 명령이 이미 진행 중입니다.")
                model = self._ensure_liveness_listener(camera)
                model.begin_command(
                    str(command_label),
                    now_s=time.monotonic(),
                    preserve_qualification=bool(preserve_qualification),
                )
                self._active_command_cameras.add(camera)
            try:
                # The command worker owns both recv_match and this side's
                # re-entrant send lock until the reservation ends. Keep the
                # PX4 peer current from that owner; the GUI timer deliberately
                # skips an active side and must never wait on this lock.
                self._send_due_gcs_heartbeat(
                    camera,
                    self.connections[camera],
                    now_s=time.monotonic(),
                )
                yield
            finally:
                with self._command_state_lock:
                    self._active_command_cameras.discard(camera)
                    model.end_command(now_s=time.monotonic())

    def start(
        self,
        camera: str | None = None,
        *,
        start_timer: bool = True,
        apply_pending_setup: bool = False,
    ) -> None:
        try:
            from pymavlink import mavutil
        except Exception as exc:
            raise RuntimeError(
                f"pymavlink를 사용할 수 없습니다. MAVLink 연결에는 pymavlink 설치가 필요합니다. ({exc})"
            ) from exc
        from Workstation.workstation_core.mavlink_dialects import ensure_registered as _ensure_sasl_dialect

        sasl_dialect = _ensure_sasl_dialect()
        cameras = (camera,) if camera is not None else tuple(self.urls.keys())
        started_any = False
        for side in cameras:
            url = self.urls.get(side, "").strip()
            if not url:
                self.last_connect_errors[side] = "MAVLink URL이 설정되지 않았습니다."
                continue
            if side in self.connections:
                self._ensure_liveness_listener(side)
                started_any = True
                self.last_connect_errors.pop(side, None)
                continue
            connection = None
            radio_lock = None
            try:
                endpoints._assert_mavlink_udpin_available(url)
                if endpoints._is_serial_transport(self.transports.get(side)):
                    transport_label = str(self.transports.get(side) or "").upper()
                    for other, active_url in self.urls.items():
                        if other != side and other in self.connections and active_url == url:
                            raise RuntimeError(
                                f"{transport_label} endpoint is already owned by {endpoints.camera_label_ko(other)}"
                            )
                    radio_lock = self._acquire_radio_endpoint_lock(side, url)
                connection = mavutil.mavlink_connection(
                    url,
                    baud=self.baudrates.get(side, 57600),
                    autoreconnect=False,
                    robust_parsing=True,
                    dialect=sasl_dialect,
                    source_system=MAVLINK_GCS_SOURCE_SYSTEM,
                    source_component=MAVLINK_GCS_SOURCE_COMPONENT,
                )
                endpoints._disable_mavlink_udpin_reuse(connection, url)
                seeded_peer = endpoints._seed_mavlink_udpin_peer(
                    connection,
                    url,
                    self.peer_hosts.get(side, ""),
                )
            except Exception as exc:
                # A multi-side request (``camera=None``) must not fail the sides
                # that did connect just because one side raised; the caller reads
                # ``last_connect_errors`` afterwards to surface a partial-connect
                # status instead of a misleading full-success label.
                if connection is not None:
                    try:
                        connection.close()
                    except Exception:
                        pass
                if radio_lock is not None:
                    self._release_radio_endpoint_lock(side)
                self.last_connect_errors[side] = str(exc)
                self.callbacks.status(f"{endpoints.camera_label_ko(side)} MAVLink 연결 실패: {exc}")
                continue
            with self._command_state_lock:
                self.connections[side] = connection
                self._open_liveness_generation(side, connection)
                if seeded_peer is None:
                    self._seeded_udp_peers.pop(side, None)
                else:
                    self._seeded_udp_peers[side] = seeded_peer
            started_any = True
            self.last_connect_errors.pop(side, None)
            self.callbacks.status(
                f"{endpoints.camera_label_ko(side)} MAVLink 연결 시작: {url} @ {self.baudrates.get(side, 57600)}"
            )
            self._send_due_gcs_heartbeat(side, connection, now_s=time.monotonic())
            if apply_pending_setup:
                self.callbacks.apply_pending_setup(side, connection)
        if not started_any:
            label = f"{endpoints.camera_label_ko(camera)} " if camera is not None else ""
            raise RuntimeError(f"{label}MAVLink 연결 URL이 없거나 연결 시작에 실패했습니다.")
        if not self.connections:
            raise RuntimeError("MAVLink 연결 URL이 없거나 모든 연결 시작에 실패했습니다.")
        if start_timer:
            self.callbacks.start_timer()

    def _acquire_radio_endpoint_lock(self, side: str, endpoint: str):
        side = str(side)
        if side in self._radio_lock_handles:
            return self._radio_lock_handles[side]
        resolved = os.path.realpath(str(endpoint))
        digest = hashlib.sha256(resolved.encode("utf-8")).hexdigest()[:20]
        runtime_root = Path(f"/run/user/{os.getuid()}")
        if not runtime_root.is_dir():
            runtime_root = Path("/tmp")
        lock_path = runtime_root / f"workstation-pixhawk-radio-{digest}.lock"
        handle = lock_path.open("a+", encoding="utf-8")
        try:
            fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError as exc:
            handle.close()
            raise RuntimeError(f"Radio endpoint is busy: {endpoint}") from exc
        handle.seek(0)
        handle.truncate()
        handle.write(f"pid={os.getpid()} role={side} endpoint={resolved}\n")
        handle.flush()
        self._radio_lock_handles[side] = handle
        return handle

    def _release_radio_endpoint_lock(self, side: str) -> None:
        handle = self._radio_lock_handles.pop(str(side), None)
        if handle is None:
            return
        try:
            fcntl.flock(handle.fileno(), fcntl.LOCK_UN)
        finally:
            handle.close()

    def _open_liveness_generation(self, side: str, connection: Any) -> MavlinkSideLiveness:
        side = str(side)
        model = self._mavlink_liveness[side]
        model.open_listener(
            endpoint=self.urls.get(side, ""),
            peer_host=self.peer_hosts.get(side, ""),
            now_s=time.monotonic(),
        )
        self._liveness_connection_ids[side] = id(connection)
        self._gcs_heartbeat_last_attempt_s.pop(side, None)
        self._mavlink_boot_time_trackers.setdefault(side, _MavlinkBootTimeTracker()).reset()
        return model

    def _ensure_liveness_listener(self, side: str) -> MavlinkSideLiveness:
        side = str(side)
        model = self._mavlink_liveness[side]
        connection = self.connections.get(side)
        if connection is not None and self._liveness_connection_ids.get(side) != id(connection):
            return self._open_liveness_generation(side, connection)
        return model

    def mavlink_health_snapshot(
        self,
        side: str,
        *,
        now_s: float | None = None,
    ) -> MavlinkSideHealthSnapshot:
        model = self._ensure_liveness_listener(str(side))
        return model.snapshot(now_s=now_s)

    def mavlink_health_snapshots(
        self,
        *,
        now_s: float | None = None,
    ) -> dict[str, MavlinkSideHealthSnapshot]:
        observed_s = time.monotonic() if now_s is None else float(now_s)
        return {side: self.mavlink_health_snapshot(side, now_s=observed_s) for side in self.urls}

    def stop(self, camera: str | None = None) -> None:
        if camera is None:
            connections = list(self.connections.items())
        else:
            connection = self.connections.get(camera)
            connections = [] if connection is None else [(camera, connection)]
        for side, connection in connections:
            with self._command_state_lock:
                try:
                    connection.close()
                except Exception:
                    pass
                self.connections.pop(side, None)
                self._mavlink_liveness[side].close_listener(now_s=time.monotonic())
                self._liveness_connection_ids.pop(side, None)
                self._seeded_udp_peers.pop(side, None)
            self.callbacks.reset_sources(side)
            self._mavlink_boot_time_trackers.setdefault(side, _MavlinkBootTimeTracker()).reset()
            self._deferred_command_messages.setdefault(side, deque()).clear()
            self._recent_raw_packets.setdefault(side, deque(maxlen=MAVLINK_DUPLICATE_CACHE_SIZE)).clear()
            self._last_receive_errors.pop(side, None)
            self._last_gcs_heartbeat_errors.pop(side, None)
            self._gcs_heartbeat_last_attempt_s.pop(side, None)
            self._identity_mismatch_reported = {item for item in self._identity_mismatch_reported if item[0] != side}
            self._last_health_states.pop(side, None)
            self._release_radio_endpoint_lock(side)
        if not self.connections:
            self.callbacks.stop_timer()
        if camera is None:
            self.callbacks.status("MAVLink 연결 중지됨")
        else:
            self.callbacks.status(f"{endpoints.camera_label_ko(camera)} MAVLink 연결 중지됨")

    def spin_once(self) -> None:
        for camera, connection in list(self.connections.items()):
            self._ensure_liveness_listener(camera)
            # ``spin_once`` runs from a Qt timer.  A continuously busy serial
            # link must not monopolize the GUI thread while we drain it.
            deadline = time.monotonic() + MAVLINK_MAX_PROCESS_TIME_S
            for _ in range(MAVLINK_MAX_MESSAGES_PER_TICK):
                deferred = False
                received_at_s: float | None = None
                with self._command_state_lock:
                    try:
                        pending = self._deferred_command_messages.setdefault(camera, deque())
                        if pending:
                            received_at_s, message = pending.popleft()
                            deferred = True
                        elif camera in self._active_command_cameras:
                            # A blocking PARAM/SERIAL_CONTROL worker owns
                            # ``recv_match`` so it can receive its response.
                            # It still forwards non-command telemetry into
                            # this queue.  Drain that queue while the worker
                            # is active; otherwise a Radio Shell poll can
                            # make ATTITUDE age for the entire six-command
                            # sequence even though packets are arriving.
                            message = None
                        else:
                            message = connection.recv_match(blocking=False)
                            if message is not None:
                                received_at_s = time.monotonic()
                    except Exception as exc:
                        self._emit_receive_error(camera, exc)
                        # A serial read error means the device node is gone
                        # (radio unpowered, USB cable pulled); the fd cannot
                        # recover, so close and wait for an explicit Connect.
                        if endpoints._is_serial_transport(self.transports.get(camera)):
                            transport_label = str(self.transports.get(camera) or "").upper()
                            self.stop(camera)
                            self.callbacks.status(
                                f"{endpoints.camera_label_ko(camera)} {transport_label} DISCONNECTED · PRESS CONNECT"
                            )
                        break
                if message is None:
                    break
                if not deferred and not self._observe_mavlink_transport_message(
                    camera,
                    message,
                    received_at_s=received_at_s,
                ):
                    continue
                if self._is_duplicate_packet(camera, message):
                    continue
                self.callbacks.message(
                    camera,
                    message,
                    received_at_s=received_at_s,
                    allow_heartbeat_actions=not deferred,
                )
                if time.monotonic() >= deadline:
                    break
            self._send_due_gcs_heartbeat(
                camera,
                connection,
                now_s=time.monotonic(),
                wait_for_send_lock=False,
            )
            self._emit_liveness_transition(camera, now_s=time.monotonic())
            with self._command_state_lock:
                command_active = camera in self._active_command_cameras
            if command_active:
                # A blocking command worker (Shell/PARAM/MAVFTP) owns
                # recv_match while it runs, so spin_once sees no fresh
                # traffic here even though the link is fine. Declaring the
                # link dead and closing the shared connection out from under
                # that in-flight worker would abort its command with a
                # spurious I/O error instead of a real disconnect.
                continue
            health = self.mavlink_health_snapshot(camera, now_s=time.monotonic())
            if health.state in {
                MavlinkLivenessState.INITIAL_RX_TIMEOUT,
                MavlinkLivenessState.RX_STALLED,
            }:
                transport = self.transports.get(camera, "ethernet").upper()
                self.stop(camera)
                self.callbacks.status(f"{endpoints.camera_label_ko(camera)} {transport} DISCONNECTED · PRESS CONNECT")
                continue
            self.callbacks.advance_stream_policy(camera, now_s=time.monotonic())

    def _defer_command_message(self, camera: str, message: Any) -> None:
        """Retain telemetry consumed while a blocking command owns recv_match."""

        if message is None:
            return
        received_at_s = time.monotonic()
        if not self._observe_mavlink_transport_message(
            str(camera),
            message,
            received_at_s=received_at_s,
        ):
            return
        with self._command_state_lock:
            self._deferred_command_messages.setdefault(str(camera), deque()).append((received_at_s, message))

    def _observe_mavlink_transport_message(
        self,
        camera: str,
        message: Any,
        *,
        received_at_s: float | None = None,
    ) -> bool:
        """Record arrival before duplicate filtering and reject known wrong peers."""

        camera = str(camera)
        model = self._ensure_liveness_listener(camera)
        provenance = mavlink_message_provenance(message)
        system_id = provenance["mavlink_source_system"]
        component_id = provenance["mavlink_source_component"]
        message_type = message.get_type() if hasattr(message, "get_type") else ""
        # SiK/RFD local status frames are link diagnostics, not vehicle
        # identity. Accept them without allowing them to establish or refresh
        # expected-system liveness.
        if message_type == "RADIO_STATUS":
            return True
        expected_system_id = self.expected_system_ids[camera]
        if system_id is not None and int(system_id) != expected_system_id:
            mismatch = (camera, int(system_id))
            if mismatch not in self._identity_mismatch_reported:
                self._identity_mismatch_reported.add(mismatch)
                self.callbacks.status(
                    f"{endpoints.camera_label_ko(camera)} MAVLink identity 불일치: "
                    f"expected sysid {expected_system_id}, got {int(system_id)}"
                )
            return False
        liveness_message_type = message_type
        if message_type == "ATTITUDE" and getattr(message, "time_boot_ms", None) is not None:
            # Count parseable transport now, but let the Qt receive owner
            # classify the PX4 boot epoch before this packet refreshes ATTITUDE
            # liveness. Deferred command telemetry follows the same owner path.
            liveness_message_type = "ATTITUDE_PENDING_BOOT_CHECK"
        accepted = model.observe_packet(
            system_id=system_id,
            component_id=component_id,
            message_type=liveness_message_type,
            received_at_s=received_at_s,
            generation=model.connection_generation,
        )
        # Small legacy/test doubles may not expose MAVLink provenance.  They
        # can still exercise downstream decoders, but never establish link
        # identity or qualify the transport.
        return accepted or system_id is None

    def _send_due_gcs_heartbeat(
        self,
        camera: str,
        connection: Any,
        *,
        now_s: float,
        wait_for_send_lock: bool = True,
    ) -> None:
        camera = str(camera)
        model = self._ensure_liveness_listener(camera)
        health = model.snapshot(now_s=now_s)
        if not health.listener_open:
            return
        if (
            not health.identity_verified
            and endpoints._mavlink_udpin_endpoint(self.urls.get(camera, "")) is not None
            and camera not in self._seeded_udp_peers
        ):
            return
        mav = getattr(connection, "mav", None)
        sender = getattr(mav, "heartbeat_send", None)
        if not callable(sender):
            return
        mavlink = getattr(connection, "mavlink", None)
        send_lock = self._mavlink_send_locks.setdefault(camera, threading.RLock())
        acquired = send_lock.acquire(blocking=bool(wait_for_send_lock))
        if not acquired:
            return
        send_error: Exception | None = None
        try:
            # The due check and attempt marker share the same side lock as all
            # MAVLink writers. This prevents a command-owner poll and a GUI
            # timer edge from both emitting the same one-second heartbeat.
            last_attempt_s = self._gcs_heartbeat_last_attempt_s.get(camera)
            if last_attempt_s is not None and now_s - last_attempt_s < MAVLINK_GCS_HEARTBEAT_INTERVAL_S:
                return
            self._gcs_heartbeat_last_attempt_s[camera] = now_s
            try:
                sender(
                    _mavlink_constant(mavlink, "MAV_TYPE_GCS", 6),
                    _mavlink_constant(mavlink, "MAV_AUTOPILOT_INVALID", 8),
                    0,
                    0,
                    _mavlink_constant(mavlink, "MAV_STATE_ACTIVE", 4),
                )
            except Exception as exc:
                send_error = exc
        finally:
            send_lock.release()
        if send_error is not None:
            exc = send_error
            message = f"{type(exc).__name__}: {exc}"
            if should_emit_throttled_error(
                self._last_gcs_heartbeat_errors,
                camera,
                message,
                now_s,
                MAVLINK_GCS_HEARTBEAT_INTERVAL_S,
            ):
                self.callbacks.status(f"{endpoints.camera_label_ko(camera)} GCS heartbeat 전송 실패: {exc}")
            return
        self._last_gcs_heartbeat_errors.pop(camera, None)
        model.note_gcs_heartbeat_sent(
            sent_at_s=now_s,
            generation=health.connection_generation,
        )

    def _emit_liveness_transition(self, camera: str, *, now_s: float) -> None:
        health = self.mavlink_health_snapshot(str(camera), now_s=now_s)
        previous = self._last_health_states.get(str(camera))
        if previous is health.state:
            return
        if (
            previous is MavlinkLivenessState.LIVE
            and health.state is MavlinkLivenessState.QUALIFYING
            and health.stall_reason == "ACTIVE_COMMAND"
            and health.qualification_elapsed_s >= health.qualification_required_s
        ):
            # A qualification-preserving command (sensor_mag shell poll every
            # 2 s) parks the model in QUALIFYING only while it owns the
            # receive path; the side is still qualified, so logging the
            # LIVE -> QUALIFYING -> LIVE blip twice per poll is pure noise.
            return
        self._last_health_states[str(camera)] = health.state
        detail = ""
        if health.state is MavlinkLivenessState.INITIAL_RX_TIMEOUT:
            detail = (
                f"; peer={health.peer_host or '--'}; listener age="
                f"{float(health.listener_age_s or 0.0):.1f}s; "
                "expected-sysid HEARTBEAT not received"
            )
        elif health.state is MavlinkLivenessState.RX_STALLED:
            detail = (
                f"; ANY age={health.any_rx_age_s:.1f}s; RTCM/commands blocked; PX4/NuttX UDP task deadlock suspected"
            )
        elif health.state in {MavlinkLivenessState.QUALIFYING, MavlinkLivenessState.RECOVERING}:
            elapsed_s = min(
                max(0, int(health.qualification_elapsed_s)),
                max(1, int(health.qualification_required_s)),
            )
            required_s = max(1, int(health.qualification_required_s))
            detail = (
                f"; qualification={elapsed_s // 60:02d}:{elapsed_s % 60:02d}/"
                f"{required_s // 60:02d}:{required_s % 60:02d}"
            )
        self.callbacks.status(f"{endpoints.camera_label_ko(camera)} MAVLink liveness: {health.state.value}{detail}")

    def _is_duplicate_packet(self, camera: str, message: Any) -> bool:
        """Drop an exact duplicate transport packet before any downstream signal."""

        reader = getattr(message, "get_msgbuf", None)
        if not callable(reader):
            return False
        try:
            raw = bytes(reader())
        except Exception:
            return False
        if not raw:
            return False
        now_s = time.monotonic()
        recent = self._recent_raw_packets.setdefault(str(camera), deque(maxlen=MAVLINK_DUPLICATE_CACHE_SIZE))
        cutoff_s = now_s - MAVLINK_DUPLICATE_TTL_S
        while recent and recent[0][0] < cutoff_s:
            recent.popleft()
        duplicate = any(previous == raw for _timestamp_s, previous in recent)
        if not duplicate:
            recent.append((now_s, raw))
        return duplicate

    def _emit_receive_error(self, camera: str, exc: BaseException) -> None:
        message = f"{type(exc).__name__}: {exc}"
        now_s = time.monotonic()
        if not should_emit_throttled_error(
            self._last_receive_errors,
            str(camera),
            message,
            now_s,
            MAVLINK_RECEIVE_ERROR_INTERVAL_S,
        ):
            return
        self.callbacks.status(f"{endpoints.camera_label_ko(camera)} MAVLink 수신 실패: {exc}")

    def _observe_remote_px4_reboot(
        self,
        camera: str,
        message: Any,
        *,
        received_at_s: float,
        provenance: dict[str, int | None],
    ) -> bool:
        """Classify ATTITUDE and rearm after authoritative new-boot evidence.

        This method is called only from the subscriber's receive owner.  It
        never emits the persistent ``stream_setup_required`` signal and sends
        no command itself.  Clearing the ephemeral request marker lets the
        next expected-system HEARTBEAT use the existing one-shot transport-profile
        path.
        """

        camera = str(camera)
        source_system = provenance.get("mavlink_source_system")
        if source_system != self.expected_system_ids.get(camera):
            return True
        time_boot_ms = getattr(message, "time_boot_ms", None)
        if time_boot_ms is None:
            return True
        connection = self.connections.get(camera)
        if connection is None:
            return True
        model = self._ensure_liveness_listener(camera)
        observation = self._mavlink_boot_time_trackers.setdefault(camera, _MavlinkBootTimeTracker()).observe(
            time_boot_ms, received_at_s=received_at_s
        )
        if not observation.accept_for_downstream:
            return False
        evidence = observation.reboot
        if evidence is not None:
            self.callbacks.reset_stream_requests(camera)
            model.note_remote_reboot(now_s=received_at_s)
        model.observe_packet(
            system_id=source_system,
            component_id=provenance.get("mavlink_source_component"),
            message_type="ATTITUDE",
            received_at_s=received_at_s,
            generation=model.connection_generation,
        )
        if evidence is None:
            return True
        self._last_health_states.pop(camera, None)
        self.callbacks.status(
            f"{endpoints.camera_label_ko(camera)} PX4 재부팅 감지: ATTITUDE time_boot_ms "
            f"{evidence.previous_time_boot_ms}->{evidence.current_time_boot_ms}; "
            "liveness 재검증/transport stream profile 1회 재요청 대기"
        )
        return True

    def resource_inventory(self) -> dict[str, object]:
        return {
            **{f"pixhawk.{side}.connection": handle for side, handle in self.connections.items()},
            **{f"pixhawk.{side}.endpoint_lease": handle for side, handle in self._radio_lock_handles.items()},
        }

    def send_lock(self, camera: str):
        return self._mavlink_send_locks.setdefault(str(camera), threading.RLock())

    def command_io(self, camera: str) -> CommandIO:
        """Lend the existing connection and callbacks; never open another link."""
        camera = str(camera)
        connection = self.connections.get(camera)
        if connection is None:
            raise RuntimeError(f"{endpoints.camera_label_ko(camera)} MAVLink 연결이 없습니다.")
        return CommandIO(
            connection=connection,
            send_lock=self.send_lock(camera),
            on_unmatched=lambda message: self._defer_command_message(camera, message),
            on_receive_poll=lambda: self._send_due_gcs_heartbeat(
                camera,
                connection,
                now_s=time.monotonic(),
            ),
        )
