"""Bounded cross-process native-frame storage with exact identity and leases.

The producer writes decoded BGR rasters into preallocated shared-memory slots.
Consumers exchange only :class:`FrameKey` descriptors until a measurement pair
is accepted.  Pinning a slot prevents overwrite until evidence persistence has
durably acknowledged the frame and released its lease.
"""

from __future__ import annotations

import ctypes
import hashlib
import json
import math
import multiprocessing
import os
from pathlib import Path
import re
import secrets
import stat
import threading
import time
from collections.abc import Mapping
from dataclasses import dataclass
from multiprocessing import shared_memory
from typing import Any


EMPTY = 0
READY = 1
WRITING = 2
PIXEL_FORMAT_BGR24 = "BGR24"
DEFAULT_NATIVE_RING_CAPACITY = 16
DEFAULT_NATIVE_MAX_WIDTH = 1280
DEFAULT_NATIVE_MAX_HEIGHT = 720
DEFAULT_NATIVE_CHANNELS = 3
NATIVE_FRAME_SHM_PREFIX = "sasl_ws_native_"
_NATIVE_FRAME_SHM_NAME_RE = re.compile(
    rf"^{re.escape(NATIVE_FRAME_SHM_PREFIX)}(?P<pid>\d+)_(?P<token>[0-9a-f]{{16}})$"
)

# All header fields use signed 64-bit storage so one RawArray can be shared
# through Python's spawn context without platform-dependent struct padding.
_HEADER_FIELDS = 12
(
    _STATE,
    _GENERATION,
    _FRAME_ID,
    _TIMESTAMP_NS,
    _SOURCE_TIMESTAMP_NS,
    _WIDTH,
    _HEIGHT,
    _CHANNELS,
    _DATA_LENGTH,
    _LEASE_COUNT,
    _PUBLISHED_MONOTONIC_NS,
    _RESERVED,
) = range(_HEADER_FIELDS)


def _finite_timestamp_ns(value: float | int | None) -> int:
    if value is None:
        return -1
    try:
        numeric = float(value)
    except (TypeError, ValueError):
        return -1
    if not math.isfinite(numeric):
        return -1
    return int(round(numeric * 1_000_000_000.0))


def _timestamp_s(value_ns: int) -> float | None:
    return None if int(value_ns) < 0 else int(value_ns) / 1_000_000_000.0


def _process_is_alive(pid: int) -> bool:
    if int(pid) <= 0:
        return False
    try:
        os.kill(int(pid), 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    except OSError:
        return False
    return True


def reclaim_orphaned_native_frame_segments(
    *,
    current_pid: int | None = None,
    shm_root: str | os.PathLike[str] = "/dev/shm",
) -> tuple[str, ...]:
    """Unlink only dead, same-user Workstation native-frame segments.

    ``SIGKILL`` cannot execute owner cleanup, and a multiprocessing resource
    tracker is not a sufficient oracle once several spawned camera processes
    share registrations.  The Workstation-specific name embeds its owner PID;
    startup may therefore recover a previous dead instance without scanning or
    deleting arbitrary Python ``psm_*`` objects.
    """

    if os.name != "posix" or not hasattr(os, "getuid"):
        return ()
    root = Path(shm_root)
    if not root.is_dir():
        return ()
    owner_pid = os.getpid() if current_pid is None else int(current_pid)
    owner_uid = os.getuid()
    reclaimed: list[str] = []
    try:
        entries = tuple(root.iterdir())
    except OSError:
        return ()
    for entry in entries:
        match = _NATIVE_FRAME_SHM_NAME_RE.fullmatch(entry.name)
        if match is None:
            continue
        stale_pid = int(match.group("pid"))
        if stale_pid == owner_pid or _process_is_alive(stale_pid):
            continue
        try:
            details = entry.stat(follow_symlinks=False)
        except (FileNotFoundError, OSError):
            continue
        if details.st_uid != owner_uid or not stat.S_ISREG(details.st_mode):
            continue
        try:
            segment = shared_memory.SharedMemory(name=entry.name, create=False)
        except (FileNotFoundError, PermissionError, OSError):
            continue
        try:
            segment.unlink()
        except (FileNotFoundError, PermissionError, OSError):
            pass
        else:
            reclaimed.append(entry.name)
        finally:
            segment.close()
    return tuple(sorted(reclaimed))


@dataclass(frozen=True, order=True)
class FrameKey:
    camera_id: str
    stream_generation: int
    frame_id: int

    def to_mapping(self) -> dict[str, str | int]:
        return {
            "camera_id": self.camera_id,
            "stream_generation": self.stream_generation,
            "frame_id": self.frame_id,
            "frame_key": self.text,
        }

    @property
    def text(self) -> str:
        return f"{self.camera_id}:{self.stream_generation}:{self.frame_id}"

    @classmethod
    def from_mapping(cls, payload: Mapping[str, Any]) -> "FrameKey":
        """Parse one frame identity and reject contradictory redundancy.

        Runtime metadata intentionally carries both the three typed components
        and their compact ``frame_key`` projection.  Treating either copy as
        authoritative would let a partially stale or tampered payload select a
        different ring slot than the evidence row names.  When both forms are
        present they therefore have to identify exactly the same frame.
        """

        if not isinstance(payload, Mapping):
            raise TypeError("frame key payload must be a mapping")

        compact_value = payload.get("frame_key")
        compact_text = str(compact_value or "").strip()
        compact_key: FrameKey | None = None
        if compact_text:
            parts = compact_text.rsplit(":", 2)
            if len(parts) != 3 or not parts[0]:
                raise ValueError("frame_key must be camera:generation:frame")
            try:
                compact_key = cls(
                    camera_id=parts[0],
                    stream_generation=int(parts[1]),
                    frame_id=int(parts[2]),
                )
            except (TypeError, ValueError) as exc:
                raise ValueError(
                    "frame_key generation and frame components must be integers"
                ) from exc

        component_names = ("camera_id", "stream_generation", "frame_id")
        component_present = {
            name: name in payload and payload.get(name) not in (None, "")
            for name in component_names
        }
        try:
            component_key = cls(
                camera_id=str(payload.get("camera_id") or ""),
                stream_generation=int(payload.get("stream_generation") or 0),
                frame_id=int(payload.get("frame_id") or 0),
            )
        except (TypeError, ValueError) as exc:
            raise ValueError("frame key components are malformed") from exc

        if compact_key is None:
            return component_key

        comparisons = {
            "camera_id": (component_key.camera_id, compact_key.camera_id),
            "stream_generation": (
                component_key.stream_generation,
                compact_key.stream_generation,
            ),
            "frame_id": (component_key.frame_id, compact_key.frame_id),
        }
        mismatched = [
            name
            for name, (component, compact) in comparisons.items()
            if component_present[name] and component != compact
        ]
        if mismatched:
            raise ValueError(
                "frame_key/component mismatch: " + ",".join(mismatched)
            )
        return compact_key


@dataclass(frozen=True)
class NativeFrameDescriptor:
    key: FrameKey
    available: bool
    slot_index: int
    timestamp_s: float | None
    source_timestamp_s: float | None
    width: int
    height: int
    channels: int
    byte_length: int
    pixel_format: str = PIXEL_FORMAT_BGR24
    unavailable_reason: str = ""
    published_monotonic_ns: int | None = None

    def to_mapping(self) -> dict[str, Any]:
        identity = {
            **self.key.to_mapping(),
            "timestamp_s": self.timestamp_s,
            "source_timestamp_s": self.source_timestamp_s,
            "native_width": self.width,
            "native_height": self.height,
            "native_channels": self.channels,
            "native_byte_length": self.byte_length,
            "native_pixel_format": self.pixel_format,
        }
        canonical = json.dumps(
            identity,
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
        ).encode("utf-8")
        return {
            **identity,
            "native_ring_available": self.available,
            "native_ring_slot": self.slot_index,
            "native_ring_unavailable_reason": self.unavailable_reason,
            "native_published_monotonic_ns": self.published_monotonic_ns,
            "frame_descriptor_sha256": hashlib.sha256(canonical).hexdigest(),
        }


@dataclass(frozen=True)
class NativeFrameLease:
    key: FrameKey
    slot_index: int


@dataclass(frozen=True)
class NativeFrameSnapshot:
    descriptor: NativeFrameDescriptor
    data: bytes


class SharedNativeFrameRing:
    """One camera generation's bounded shared-memory BGR frame ring."""

    def __init__(
        self,
        *,
        camera_id: str,
        stream_generation: int,
        capacity: int,
        max_width: int,
        max_height: int,
        channels: int,
        shm_name: str,
        headers,
        lock,
        next_frame_id,
        write_cursor,
        published_count,
        overwrite_count,
        dropped_count,
        pin_count,
        pin_high_water,
        owner: bool,
        shm: shared_memory.SharedMemory | None = None,
    ) -> None:
        self.camera_id = str(camera_id)
        self.stream_generation = int(stream_generation)
        self.capacity = max(1, int(capacity))
        self.max_width = max(1, int(max_width))
        self.max_height = max(1, int(max_height))
        self.channels = max(1, int(channels))
        self.slot_bytes = self.max_width * self.max_height * self.channels
        self.shm_name = str(shm_name)
        self._headers = headers
        self._lock = lock
        self._next_frame_id = next_frame_id
        self._write_cursor = write_cursor
        self._published_count = published_count
        self._overwrite_count = overwrite_count
        self._dropped_count = dropped_count
        self._pin_count = pin_count
        self._pin_high_water = pin_high_water
        self._owner = bool(owner)
        self._shm = shm
        self._local_lock = threading.Lock()
        self._closed = False
        self._unlinked = False

    @classmethod
    def create(
        cls,
        *,
        camera_id: str,
        stream_generation: int,
        capacity: int = DEFAULT_NATIVE_RING_CAPACITY,
        max_width: int = DEFAULT_NATIVE_MAX_WIDTH,
        max_height: int = DEFAULT_NATIVE_MAX_HEIGHT,
        channels: int = DEFAULT_NATIVE_CHANNELS,
        mp_context: multiprocessing.context.BaseContext | None = None,
    ) -> "SharedNativeFrameRing":
        context = mp_context or multiprocessing.get_context("spawn")
        normalized_capacity = max(1, int(capacity))
        slot_bytes = max(1, int(max_width)) * max(1, int(max_height)) * max(1, int(channels))
        shm = shared_memory.SharedMemory(
            name=(
                f"{NATIVE_FRAME_SHM_PREFIX}{os.getpid()}_"
                f"{secrets.token_hex(8)}"
            ),
            create=True,
            size=normalized_capacity * slot_bytes,
        )
        headers = context.RawArray(
            ctypes.c_longlong,
            normalized_capacity * _HEADER_FIELDS,
        )
        return cls(
            camera_id=camera_id,
            stream_generation=stream_generation,
            capacity=normalized_capacity,
            max_width=max_width,
            max_height=max_height,
            channels=channels,
            shm_name=shm.name,
            headers=headers,
            lock=context.RLock(),
            next_frame_id=context.Value("q", 0),
            write_cursor=context.Value("i", 0),
            published_count=context.Value("q", 0),
            overwrite_count=context.Value("q", 0),
            dropped_count=context.Value("q", 0),
            pin_count=context.Value("q", 0),
            pin_high_water=context.Value("q", 0),
            owner=True,
            shm=shm,
        )

    def __getstate__(self) -> dict[str, Any]:
        state = dict(self.__dict__)
        state["_shm"] = None
        state["_owner"] = False
        state["_local_lock"] = None
        state["_closed"] = False
        state["_unlinked"] = False
        return state

    def __setstate__(self, state: dict[str, Any]) -> None:
        self.__dict__.update(state)
        self._local_lock = threading.Lock()

    def _ensure_shm(self) -> shared_memory.SharedMemory:
        if self._closed:
            raise RuntimeError("native frame ring is closed")
        with self._local_lock:
            if self._shm is None:
                shm = shared_memory.SharedMemory(name=self.shm_name, create=False)
                # Camera workers are created through the parent's ``spawn``
                # context and share its resource-tracker process.  Unregistering
                # this attachment would therefore remove the owning parent's
                # global registration as well, leaking the segment if the
                # parent is later SIGKILLed.  Keep the shared registration;
                # graceful owner cleanup still calls ``unlink()`` explicitly,
                # while tracker EOF reclaims it after an abnormal parent exit.
                self._shm = shm
            return self._shm

    def _header_index(self, slot_index: int, field: int) -> int:
        return int(slot_index) * _HEADER_FIELDS + int(field)

    def _get(self, slot_index: int, field: int) -> int:
        return int(self._headers[self._header_index(slot_index, field)])

    def _set(self, slot_index: int, field: int, value: int) -> None:
        self._headers[self._header_index(slot_index, field)] = int(value)

    def _next_key_locked(self) -> FrameKey:
        self._next_frame_id.value += 1
        return FrameKey(
            camera_id=self.camera_id,
            stream_generation=self.stream_generation,
            frame_id=int(self._next_frame_id.value),
        )

    def publish_frame(
        self,
        frame,
        *,
        timestamp_s: float | int | None,
        source_timestamp_s: float | int | None = None,
    ) -> NativeFrameDescriptor:
        shape = getattr(frame, "shape", ())
        if len(shape) < 2:
            raise ValueError("native frame must expose height and width")
        height = int(shape[0])
        width = int(shape[1])
        channels = int(shape[2]) if len(shape) >= 3 else 1
        try:
            payload = frame.tobytes(order="C")
        except TypeError:
            payload = frame.tobytes()
        return self.publish_bytes(
            payload,
            width=width,
            height=height,
            channels=channels,
            timestamp_s=timestamp_s,
            source_timestamp_s=source_timestamp_s,
        )

    def publish_bytes(
        self,
        payload: bytes | bytearray | memoryview,
        *,
        width: int,
        height: int,
        channels: int,
        timestamp_s: float | int | None,
        source_timestamp_s: float | int | None = None,
    ) -> NativeFrameDescriptor:
        data = memoryview(payload).cast("B")
        width_value = int(width)
        height_value = int(height)
        channels_value = int(channels)
        expected_length = width_value * height_value * channels_value
        with self._lock:
            key = self._next_key_locked()
            if (
                width_value <= 0
                or height_value <= 0
                or channels_value != self.channels
                or expected_length != len(data)
                or len(data) > self.slot_bytes
            ):
                self._dropped_count.value += 1
                return NativeFrameDescriptor(
                    key=key,
                    available=False,
                    slot_index=-1,
                    timestamp_s=_timestamp_s(_finite_timestamp_ns(timestamp_s)),
                    source_timestamp_s=_timestamp_s(_finite_timestamp_ns(source_timestamp_s)),
                    width=width_value,
                    height=height_value,
                    channels=channels_value,
                    byte_length=len(data),
                    unavailable_reason="frame-size-or-format-out-of-contract",
                )

            selected_slot = -1
            cursor = int(self._write_cursor.value) % self.capacity
            for offset in range(self.capacity):
                candidate = (cursor + offset) % self.capacity
                if self._get(candidate, _LEASE_COUNT) == 0:
                    selected_slot = candidate
                    break
            if selected_slot < 0:
                self._dropped_count.value += 1
                return NativeFrameDescriptor(
                    key=key,
                    available=False,
                    slot_index=-1,
                    timestamp_s=_timestamp_s(_finite_timestamp_ns(timestamp_s)),
                    source_timestamp_s=_timestamp_s(_finite_timestamp_ns(source_timestamp_s)),
                    width=width_value,
                    height=height_value,
                    channels=channels_value,
                    byte_length=len(data),
                    unavailable_reason="all-native-ring-slots-pinned",
                )

            if self._get(selected_slot, _STATE) == READY:
                self._overwrite_count.value += 1
            self._set(selected_slot, _STATE, WRITING)
            shm = self._ensure_shm()
            start = selected_slot * self.slot_bytes
            shm.buf[start : start + len(data)] = data
            published_ns = time.monotonic_ns()
            self._set(selected_slot, _GENERATION, key.stream_generation)
            self._set(selected_slot, _FRAME_ID, key.frame_id)
            self._set(selected_slot, _TIMESTAMP_NS, _finite_timestamp_ns(timestamp_s))
            self._set(
                selected_slot,
                _SOURCE_TIMESTAMP_NS,
                _finite_timestamp_ns(source_timestamp_s),
            )
            self._set(selected_slot, _WIDTH, width_value)
            self._set(selected_slot, _HEIGHT, height_value)
            self._set(selected_slot, _CHANNELS, channels_value)
            self._set(selected_slot, _DATA_LENGTH, len(data))
            self._set(selected_slot, _LEASE_COUNT, 0)
            self._set(selected_slot, _PUBLISHED_MONOTONIC_NS, published_ns)
            self._set(selected_slot, _STATE, READY)
            self._write_cursor.value = (selected_slot + 1) % self.capacity
            self._published_count.value += 1
            return self._descriptor_locked(selected_slot)

    def _descriptor_locked(self, slot_index: int) -> NativeFrameDescriptor:
        key = FrameKey(
            camera_id=self.camera_id,
            stream_generation=self._get(slot_index, _GENERATION),
            frame_id=self._get(slot_index, _FRAME_ID),
        )
        return NativeFrameDescriptor(
            key=key,
            available=self._get(slot_index, _STATE) == READY,
            slot_index=int(slot_index),
            timestamp_s=_timestamp_s(self._get(slot_index, _TIMESTAMP_NS)),
            source_timestamp_s=_timestamp_s(
                self._get(slot_index, _SOURCE_TIMESTAMP_NS)
            ),
            width=self._get(slot_index, _WIDTH),
            height=self._get(slot_index, _HEIGHT),
            channels=self._get(slot_index, _CHANNELS),
            byte_length=self._get(slot_index, _DATA_LENGTH),
            published_monotonic_ns=self._get(
                slot_index,
                _PUBLISHED_MONOTONIC_NS,
            ),
        )

    def descriptor(self, key: FrameKey) -> NativeFrameDescriptor | None:
        if key.camera_id != self.camera_id:
            return None
        with self._lock:
            for slot_index in range(self.capacity):
                if (
                    self._get(slot_index, _STATE) == READY
                    and self._get(slot_index, _GENERATION) == key.stream_generation
                    and self._get(slot_index, _FRAME_ID) == key.frame_id
                ):
                    return self._descriptor_locked(slot_index)
        return None

    def latest_descriptor(self) -> NativeFrameDescriptor | None:
        with self._lock:
            candidates = [
                slot_index
                for slot_index in range(self.capacity)
                if self._get(slot_index, _STATE) == READY
                and self._get(slot_index, _GENERATION) == self.stream_generation
            ]
            if not candidates:
                return None
            slot_index = max(candidates, key=lambda item: self._get(item, _FRAME_ID))
            return self._descriptor_locked(slot_index)

    def pin(self, key: FrameKey) -> NativeFrameLease | None:
        if key.camera_id != self.camera_id:
            return None
        with self._lock:
            for slot_index in range(self.capacity):
                if (
                    self._get(slot_index, _STATE) == READY
                    and self._get(slot_index, _GENERATION) == key.stream_generation
                    and self._get(slot_index, _FRAME_ID) == key.frame_id
                ):
                    leases = self._get(slot_index, _LEASE_COUNT) + 1
                    self._set(slot_index, _LEASE_COUNT, leases)
                    self._pin_count.value += 1
                    if self._pin_count.value > self._pin_high_water.value:
                        self._pin_high_water.value = self._pin_count.value
                    return NativeFrameLease(key=key, slot_index=slot_index)
        return None

    def read(self, lease: NativeFrameLease) -> NativeFrameSnapshot:
        with self._lock:
            slot_index = int(lease.slot_index)
            if not 0 <= slot_index < self.capacity:
                raise KeyError("native frame lease slot is out of range")
            if (
                self._get(slot_index, _STATE) != READY
                or self._get(slot_index, _GENERATION)
                != lease.key.stream_generation
                or self._get(slot_index, _FRAME_ID) != lease.key.frame_id
                or self._get(slot_index, _LEASE_COUNT) <= 0
            ):
                raise KeyError("native frame lease no longer identifies a pinned frame")
            descriptor = self._descriptor_locked(slot_index)
            start = slot_index * self.slot_bytes
            length = descriptor.byte_length
            data = bytes(self._ensure_shm().buf[start : start + length])
            return NativeFrameSnapshot(descriptor=descriptor, data=data)

    def release(self, lease: NativeFrameLease) -> bool:
        with self._lock:
            slot_index = int(lease.slot_index)
            if not 0 <= slot_index < self.capacity:
                return False
            if (
                self._get(slot_index, _STATE) != READY
                or self._get(slot_index, _GENERATION)
                != lease.key.stream_generation
                or self._get(slot_index, _FRAME_ID) != lease.key.frame_id
            ):
                return False
            leases = self._get(slot_index, _LEASE_COUNT)
            if leases <= 0:
                return False
            self._set(slot_index, _LEASE_COUNT, leases - 1)
            self._pin_count.value = max(0, int(self._pin_count.value) - 1)
            return True

    def metrics(self) -> dict[str, int | str]:
        with self._lock:
            ready = 0
            pinned_slots = 0
            for slot_index in range(self.capacity):
                if self._get(slot_index, _STATE) == READY:
                    ready += 1
                if self._get(slot_index, _LEASE_COUNT) > 0:
                    pinned_slots += 1
            return {
                "camera_id": self.camera_id,
                "stream_generation": self.stream_generation,
                "capacity": self.capacity,
                "ready_slots": ready,
                "pinned_slots": pinned_slots,
                "active_leases": int(self._pin_count.value),
                "pin_high_water": int(self._pin_high_water.value),
                "published": int(self._published_count.value),
                "overwritten": int(self._overwrite_count.value),
                "dropped": int(self._dropped_count.value),
                "slot_bytes": self.slot_bytes,
            }

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        with self._local_lock:
            shm = self._shm
            self._shm = None
        if shm is not None:
            try:
                shm.close()
            except BufferError:
                pass

    def unlink(self) -> None:
        if not self._owner or self._unlinked:
            return
        self._unlinked = True
        shm = self._shm
        temporary = False
        if shm is None:
            try:
                shm = shared_memory.SharedMemory(name=self.shm_name, create=False)
            except FileNotFoundError:
                return
            temporary = True
        try:
            shm.unlink()
        except FileNotFoundError:
            pass
        finally:
            if temporary:
                shm.close()


__all__ = [
    "DEFAULT_NATIVE_CHANNELS",
    "DEFAULT_NATIVE_MAX_HEIGHT",
    "DEFAULT_NATIVE_MAX_WIDTH",
    "DEFAULT_NATIVE_RING_CAPACITY",
    "FrameKey",
    "NativeFrameDescriptor",
    "NativeFrameLease",
    "NativeFrameSnapshot",
    "PIXEL_FORMAT_BGR24",
    "SharedNativeFrameRing",
    "NATIVE_FRAME_SHM_PREFIX",
    "reclaim_orphaned_native_frame_segments",
]
