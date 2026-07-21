"""Pure simulation-time pairing and Ground Truth alignment primitives."""

from __future__ import annotations

from bisect import bisect_left
from collections import deque
from dataclasses import dataclass, replace
import math
from typing import Any, Iterable


NANOSECONDS_PER_SECOND = 1_000_000_000


class BoundedIdHistory:
    """Bounded membership history for non-monotonic late pair completion."""

    def __init__(self, capacity: int) -> None:
        if int(capacity) < 1:
            raise ValueError("capacity must be positive")
        self.capacity = int(capacity)
        self._order: deque[int] = deque()
        self._ids: set[int] = set()

    def __contains__(self, pair_id: object) -> bool:
        return pair_id in self._ids

    def __len__(self) -> int:
        return len(self._ids)

    def add(self, pair_id: int) -> bool:
        value = int(pair_id)
        if value in self._ids:
            return False
        self._order.append(value)
        self._ids.add(value)
        while len(self._order) > self.capacity:
            self._ids.discard(self._order.popleft())
        return True

    def discard(self, pair_id: int) -> None:
        value = int(pair_id)
        if value not in self._ids:
            return
        self._ids.discard(value)
        self._order = deque(item for item in self._order if item != value)

    def clear(self) -> None:
        self._order.clear()
        self._ids.clear()


def stamp_ns_from_sec_nanosec(sec: Any, nanosec: Any) -> int:
    """Convert a ROS simulation stamp to signed int64-style nanoseconds."""

    try:
        seconds = int(sec)
        nanos = int(nanosec)
    except (TypeError, ValueError):
        raise ValueError("timestamp sec/nanosec must be integers") from None
    if nanos < 0 or nanos >= NANOSECONDS_PER_SECOND:
        raise ValueError("timestamp nanosec is out of range")
    stamp_ns = seconds * NANOSECONDS_PER_SECOND + nanos
    if stamp_ns < -(1 << 63) or stamp_ns > (1 << 63) - 1:
        raise ValueError("timestamp does not fit int64 nanoseconds")
    return stamp_ns


def image_stamp_ns(message: Any) -> int:
    """Read authoritative simulation time from ``Image.header.stamp``."""

    try:
        stamp = message.header.stamp
        return stamp_ns_from_sec_nanosec(stamp.sec, stamp.nanosec)
    except AttributeError:
        raise ValueError("image header.stamp is missing") from None


@dataclass(frozen=True)
class ImageRecord:
    side: str
    stamp_ns: int
    message: Any
    sequence_in_side: int
    epoch: int


@dataclass(frozen=True)
class StereoImagePair:
    pair_id: int
    left_stamp_ns: int
    right_stamp_ns: int
    pair_midpoint_stamp_ns: int
    pair_delta_ns: int
    pairing_mode: str
    left_message: Any
    right_message: Any
    epoch: int


@dataclass(frozen=True)
class SimulationEpochEvent:
    epoch: int
    reset: bool
    stamp_ns: int


class SimulationEpochTracker:
    """Single authoritative simulation epoch derived only from ``/clock``."""

    def __init__(self) -> None:
        self.epoch = 0
        self.last_clock_stamp_ns: int | None = None
        self.reset_count = 0

    def observe_clock(self, stamp_ns: int) -> SimulationEpochEvent:
        stamp = int(stamp_ns)
        reset = self.last_clock_stamp_ns is not None and stamp < self.last_clock_stamp_ns
        if reset:
            self.epoch += 1
            self.reset_count += 1
        self.last_clock_stamp_ns = stamp
        return SimulationEpochEvent(self.epoch, reset, stamp)

    def accepts_stamp(self, stamp_ns: int) -> bool:
        """Reject a pre-reset future stamp from the current clock epoch."""

        return self.last_clock_stamp_ns is None or int(stamp_ns) <= self.last_clock_stamp_ns


class StereoImagePairer:
    """Deterministic, bounded, one-to-one pairing of stamped image records.

    Exact matches are emitted immediately. Nearest matches are emitted only
    after both side watermarks have advanced beyond the candidate records, or
    by explicit ``flush``. This prevents callback receive order from becoming
    the pairing authority.
    """

    def __init__(self, max_buffer_size: int = 64) -> None:
        if int(max_buffer_size) < 1:
            raise ValueError("max_buffer_size must be positive")
        self.max_buffer_size = int(max_buffer_size)
        self.epoch = 0
        self._next_pair_id = 1
        self._sequence = {"left": 0, "right": 0}
        self._last_stamp: dict[str, int | None] = {"left": None, "right": None}
        self._pending: dict[str, list[ImageRecord]] = {"left": [], "right": []}
        self._ready: deque[StereoImagePair] = deque()
        self.left_received_count = 0
        self.right_received_count = 0
        self.paired_count = 0
        self.exact_pair_count = 0
        self.nearest_pair_count = 0
        self.unmatched_left_count = 0
        self.unmatched_right_count = 0
        self.image_reset_count = 0
        self.out_of_order_count = 0
        self.invalid_timestamp_count = 0

    def add_message(
        self,
        side: str,
        message: Any,
        *,
        epoch: int | None = None,
    ) -> list[StereoImagePair]:
        try:
            stamp_ns = image_stamp_ns(message)
        except ValueError:
            self.invalid_timestamp_count += 1
            raise
        return self.add(side, stamp_ns, message, epoch=epoch)

    def add(
        self,
        side: str,
        stamp_ns: int,
        message: Any,
        *,
        epoch: int | None = None,
    ) -> list[StereoImagePair]:
        side = str(side)
        if side not in self._pending:
            raise ValueError("side must be 'left' or 'right'")
        try:
            stamp = int(stamp_ns)
        except (TypeError, ValueError):
            self.invalid_timestamp_count += 1
            raise ValueError("image timestamp must be an integer nanosecond value") from None
        record_epoch = self.epoch if epoch is None else int(epoch)
        if record_epoch != self.epoch:
            raise ValueError(
                f"image epoch {record_epoch} does not match active epoch {self.epoch}"
            )
        previous = self._last_stamp[side]
        if previous is not None and stamp < previous:
            self.out_of_order_count += 1
            return []
        self._last_stamp[side] = stamp
        self._sequence[side] += 1
        record = ImageRecord(side, stamp, message, self._sequence[side], record_epoch)
        self._pending[side].append(record)
        self._pending[side].sort(key=lambda item: (item.stamp_ns, item.sequence_in_side))
        if side == "left":
            self.left_received_count += 1
        else:
            self.right_received_count += 1
        self._pair_exact()
        self._pair_watermarked_nearest()
        self._enforce_bound(side)
        return self.pop_ready()

    def pop_ready(self) -> list[StereoImagePair]:
        provisional = sorted(
            self._ready,
            key=lambda pair: (
                pair.pair_midpoint_stamp_ns,
                pair.left_stamp_ns,
                pair.right_stamp_ns,
                0 if pair.pairing_mode == "exact" else 1,
            ),
        )
        self._ready.clear()
        ready = []
        for pair in provisional:
            ready.append(replace(pair, pair_id=self._next_pair_id))
            self._next_pair_id += 1
        return ready

    def flush(self) -> list[StereoImagePair]:
        self._pair_exact()
        self._pair_nearest(self._pending["left"], self._pending["right"])
        self._count_and_clear_unmatched()
        return self.pop_ready()

    @property
    def pending_left_count(self) -> int:
        return len(self._pending["left"])

    @property
    def pending_right_count(self) -> int:
        return len(self._pending["right"])

    def reset_for_epoch(self, epoch: int) -> int:
        """Clear pending frames and adopt an externally supplied clock epoch."""

        requested = int(epoch)
        if requested < self.epoch:
            raise ValueError("image pairer epoch cannot move backwards")
        if requested == self.epoch:
            return 0
        dropped = len(self._pending["left"]) + len(self._pending["right"])
        self._count_and_clear_unmatched()
        self.epoch = requested
        self.image_reset_count += 1
        self._last_stamp = {"left": None, "right": None}
        self._ready.clear()
        return dropped

    def _count_and_clear_unmatched(self) -> None:
        self.unmatched_left_count += len(self._pending["left"])
        self.unmatched_right_count += len(self._pending["right"])
        self._pending["left"].clear()
        self._pending["right"].clear()

    def _pair_exact(self, max_stamp_exclusive: int | None = None) -> None:
        left_by_stamp: dict[int, deque[ImageRecord]] = {}
        right_by_stamp: dict[int, deque[ImageRecord]] = {}
        for record in self._pending["left"]:
            left_by_stamp.setdefault(record.stamp_ns, deque()).append(record)
        for record in self._pending["right"]:
            right_by_stamp.setdefault(record.stamp_ns, deque()).append(record)
        pairs: list[tuple[ImageRecord, ImageRecord]] = []
        for stamp in sorted(set(left_by_stamp) & set(right_by_stamp)):
            if max_stamp_exclusive is not None and stamp >= max_stamp_exclusive:
                continue
            left_records = left_by_stamp[stamp]
            right_records = right_by_stamp[stamp]
            while left_records and right_records:
                pairs.append((left_records.popleft(), right_records.popleft()))
        self._remove_and_emit(pairs, "exact")

    def _pair_watermarked_nearest(self) -> None:
        if self._last_stamp["left"] is None or self._last_stamp["right"] is None:
            return
        watermark = min(int(self._last_stamp["left"]), int(self._last_stamp["right"]))
        left = [record for record in self._pending["left"] if record.stamp_ns < watermark]
        right = [record for record in self._pending["right"] if record.stamp_ns < watermark]
        self._pair_nearest(left, right)

    def _pair_nearest(self, left: Iterable[ImageRecord], right: Iterable[ImageRecord]) -> None:
        left_records = list(left)
        right_records = list(right)
        candidates = sorted(
            (
                (
                    abs(r.stamp_ns - l.stamp_ns),
                    max(l.stamp_ns, r.stamp_ns),
                    l.stamp_ns,
                    r.stamp_ns,
                    l.sequence_in_side,
                    r.sequence_in_side,
                    l,
                    r,
                )
                for l in left_records
                for r in right_records
                if l.epoch == r.epoch
            ),
            key=lambda item: item[:6],
        )
        used_left: set[int] = set()
        used_right: set[int] = set()
        pairs: list[tuple[ImageRecord, ImageRecord]] = []
        for candidate in candidates:
            left_record, right_record = candidate[-2], candidate[-1]
            left_key = id(left_record)
            right_key = id(right_record)
            if left_key in used_left or right_key in used_right:
                continue
            used_left.add(left_key)
            used_right.add(right_key)
            pairs.append((left_record, right_record))
        self._remove_and_emit(pairs, "nearest")

    def _remove_and_emit(
        self,
        pairs: Iterable[tuple[ImageRecord, ImageRecord]],
        mode: str,
    ) -> None:
        pairs = list(pairs)
        if not pairs:
            return
        left_ids = {id(left) for left, _ in pairs}
        right_ids = {id(right) for _, right in pairs}
        self._pending["left"] = [item for item in self._pending["left"] if id(item) not in left_ids]
        self._pending["right"] = [item for item in self._pending["right"] if id(item) not in right_ids]
        for left, right in sorted(pairs, key=lambda pair: (max(pair[0].stamp_ns, pair[1].stamp_ns), pair[0].stamp_ns, pair[1].stamp_ns)):
            delta = right.stamp_ns - left.stamp_ns
            pair = StereoImagePair(
                pair_id=0,
                left_stamp_ns=left.stamp_ns,
                right_stamp_ns=right.stamp_ns,
                pair_midpoint_stamp_ns=left.stamp_ns + delta // 2,
                pair_delta_ns=delta,
                pairing_mode=mode,
                left_message=left.message,
                right_message=right.message,
                epoch=left.epoch,
            )
            self.paired_count += 1
            if mode == "exact":
                self.exact_pair_count += 1
            else:
                self.nearest_pair_count += 1
            self._ready.append(pair)

    def _enforce_bound(self, side: str) -> None:
        while len(self._pending[side]) > self.max_buffer_size:
            self._pending[side].pop(0)
            if side == "left":
                self.unmatched_left_count += 1
            else:
                self.unmatched_right_count += 1


@dataclass(frozen=True)
class GroundTruthPositionRecord:
    stamp_ns: int
    position_world_enu_m: tuple[float, float, float]
    orientation_world_from_reference_xyzw: tuple[float, float, float, float]
    epoch: int


@dataclass(frozen=True)
class PendingGroundTruthRecord:
    """Immutable GT callback payload tagged with its receive-time epoch."""

    stamp_ns: int
    epoch: int
    position_world_enu_m: tuple[float, float, float]
    orientation_world_from_reference_xyzw: tuple[float, float, float, float]


@dataclass(frozen=True)
class GroundTruthAlignment:
    position_world_enu_m: tuple[float, float, float] | None
    orientation_world_from_reference_xyzw: tuple[float, float, float, float] | None
    gt_alignment_mode: str
    gt_before_stamp_ns: int | None
    gt_after_stamp_ns: int | None
    gt_interpolation_alpha: float | None
    gt_nearest_delta_ns: int | None


class GroundTruthPositionBuffer:
    """Bounded simulation-time GT history with linear position interpolation."""

    def __init__(self, max_buffer_size: int = 512) -> None:
        if int(max_buffer_size) < 1:
            raise ValueError("max_buffer_size must be positive")
        self.max_buffer_size = int(max_buffer_size)
        self.epoch = 0
        self.reset_count = 0
        self.inconsistent_clock_count = 0
        self._last_stamp_ns: int | None = None
        self._records: list[GroundTruthPositionRecord] = []

    @property
    def latest_stamp_ns(self) -> int | None:
        return self._records[-1].stamp_ns if self._records else None

    def add(
        self,
        stamp_ns: int,
        position_world_enu_m: Iterable[float],
        orientation_world_from_reference_xyzw: Iterable[float],
        *,
        epoch: int | None = None,
    ) -> GroundTruthPositionRecord:
        stamp = int(stamp_ns)
        position = tuple(float(value) for value in position_world_enu_m)
        orientation = tuple(float(value) for value in orientation_world_from_reference_xyzw)
        if len(position) != 3 or not all(math.isfinite(value) for value in position):
            raise ValueError("Ground Truth position must contain three finite metres")
        if len(orientation) != 4 or not all(math.isfinite(value) for value in orientation):
            raise ValueError("Ground Truth orientation must contain finite x,y,z,w")
        record_epoch = self.epoch if epoch is None else int(epoch)
        if record_epoch != self.epoch:
            raise ValueError(
                f"Ground Truth epoch {record_epoch} does not match active epoch {self.epoch}"
            )
        if self._last_stamp_ns is not None and stamp < self._last_stamp_ns:
            self.inconsistent_clock_count += 1
            raise ValueError("Ground Truth stamp decreased without a /clock reset")
        self._last_stamp_ns = stamp
        record = GroundTruthPositionRecord(stamp, position, orientation, record_epoch)  # type: ignore[arg-type]
        index = bisect_left([item.stamp_ns for item in self._records], stamp)
        if index < len(self._records) and self._records[index].stamp_ns == stamp:
            self._records[index] = record
        else:
            self._records.insert(index, record)
        if len(self._records) > self.max_buffer_size:
            del self._records[: len(self._records) - self.max_buffer_size]
        return record

    def reset_for_epoch(self, epoch: int) -> int:
        requested = int(epoch)
        if requested < self.epoch:
            raise ValueError("Ground Truth epoch cannot move backwards")
        if requested == self.epoch:
            return 0
        dropped = len(self._records)
        self._records.clear()
        self._last_stamp_ns = None
        self.epoch = requested
        self.reset_count += 1
        return dropped

    def align(self, pair_stamp_ns: int, epoch: int, *, allow_nearest: bool = False) -> GroundTruthAlignment:
        if int(epoch) != self.epoch or not self._records:
            return self._unavailable()
        stamp = int(pair_stamp_ns)
        stamps = [record.stamp_ns for record in self._records]
        index = bisect_left(stamps, stamp)
        if index < len(self._records) and self._records[index].stamp_ns == stamp:
            record = self._records[index]
            return GroundTruthAlignment(
                record.position_world_enu_m,
                record.orientation_world_from_reference_xyzw,
                "exact",
                stamp,
                stamp,
                0.0,
                0,
            )
        before = self._records[index - 1] if index > 0 else None
        after = self._records[index] if index < len(self._records) else None
        if before is not None and after is not None:
            alpha = (stamp - before.stamp_ns) / (after.stamp_ns - before.stamp_ns)
            position = tuple(
                (1.0 - alpha) * before.position_world_enu_m[axis]
                + alpha * after.position_world_enu_m[axis]
                for axis in range(3)
            )
            return GroundTruthAlignment(
                position,  # type: ignore[arg-type]
                None,
                "interpolated",
                before.stamp_ns,
                after.stamp_ns,
                alpha,
                None,
            )
        if allow_nearest and before is not None:
            return GroundTruthAlignment(
                before.position_world_enu_m,
                before.orientation_world_from_reference_xyzw,
                "nearest_before",
                before.stamp_ns,
                None,
                None,
                before.stamp_ns - stamp,
            )
        if allow_nearest and after is not None:
            return GroundTruthAlignment(
                after.position_world_enu_m,
                after.orientation_world_from_reference_xyzw,
                "nearest_after",
                None,
                after.stamp_ns,
                None,
                after.stamp_ns - stamp,
            )
        return GroundTruthAlignment(
            None,
            None,
            "unavailable",
            None if before is None else before.stamp_ns,
            None if after is None else after.stamp_ns,
            None,
            None,
        )

    @staticmethod
    def _unavailable() -> GroundTruthAlignment:
        return GroundTruthAlignment(None, None, "unavailable", None, None, None, None)


class PendingPairGroundTruthAligner:
    """Hold image pairs until the GT simulation-time watermark can align them."""

    def __init__(self, gt_buffer: GroundTruthPositionBuffer, max_pending_pairs: int = 64) -> None:
        if int(max_pending_pairs) < 1:
            raise ValueError("max_pending_pairs must be positive")
        self.gt_buffer = gt_buffer
        self.max_pending_pairs = int(max_pending_pairs)
        self._pending: deque[StereoImagePair] = deque()

    def add_pair(self, pair: StereoImagePair) -> list[tuple[StereoImagePair, GroundTruthAlignment]]:
        self._pending.append(pair)
        ready = self.pop_watermarked()
        while len(self._pending) > self.max_pending_pairs:
            oldest = self._pending.popleft()
            ready.append(
                (
                    oldest,
                    self.gt_buffer.align(oldest.pair_midpoint_stamp_ns, oldest.epoch, allow_nearest=True),
                )
            )
        return ready

    def pop_watermarked(self) -> list[tuple[StereoImagePair, GroundTruthAlignment]]:
        latest = self.gt_buffer.latest_stamp_ns
        ready: list[tuple[StereoImagePair, GroundTruthAlignment]] = []
        retained: deque[StereoImagePair] = deque()
        while self._pending:
            pair = self._pending.popleft()
            if pair.epoch != self.gt_buffer.epoch:
                ready.append((pair, self.gt_buffer._unavailable()))
            elif latest is not None and pair.pair_midpoint_stamp_ns <= latest:
                alignment = self.gt_buffer.align(pair.pair_midpoint_stamp_ns, pair.epoch)
                if alignment.gt_alignment_mode == "unavailable":
                    alignment = self.gt_buffer.align(
                        pair.pair_midpoint_stamp_ns,
                        pair.epoch,
                        allow_nearest=True,
                    )
                ready.append((pair, alignment))
            else:
                retained.append(pair)
        self._pending = retained
        return ready

    def flush(self) -> list[tuple[StereoImagePair, GroundTruthAlignment]]:
        ready = [
            (
                pair,
                self.gt_buffer.align(pair.pair_midpoint_stamp_ns, pair.epoch, allow_nearest=True),
            )
            for pair in self._pending
        ]
        self._pending.clear()
        return ready

    def clear(self) -> int:
        """Drop pending pairs at an image simulation reset boundary."""

        count = len(self._pending)
        self._pending.clear()
        return count

    def remove_pair(self, pair_id: int) -> bool:
        before = len(self._pending)
        self._pending = deque(pair for pair in self._pending if pair.pair_id != int(pair_id))
        return len(self._pending) != before

    def reset_for_epoch(self, epoch: int) -> int:
        """Clear pending pairs; epoch ownership remains with the shared GT buffer."""

        if int(epoch) != self.gt_buffer.epoch:
            raise ValueError("GT aligner epoch must match the Ground Truth buffer")
        return self.clear()


__all__ = [
    "BoundedIdHistory",
    "GroundTruthAlignment",
    "GroundTruthPositionBuffer",
    "GroundTruthPositionRecord",
    "ImageRecord",
    "PendingGroundTruthRecord",
    "PendingPairGroundTruthAligner",
    "SimulationEpochEvent",
    "SimulationEpochTracker",
    "StereoImagePair",
    "StereoImagePairer",
    "image_stamp_ns",
    "stamp_ns_from_sec_nanosec",
]
