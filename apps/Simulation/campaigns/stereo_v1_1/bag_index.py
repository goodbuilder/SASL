#!/usr/bin/env python3
"""Index every original image and form unique nominal ≤1 ms stereo pairs.

The input is lossless rosbag2 SQLite from capture_streams.py. Images remain
untouched in their bags. This stage does not claim YOLO detections or a valid
39-run attempt; it writes an auditable source-frame and pair index.
"""

from __future__ import annotations

import argparse
import bisect
import hashlib
import json
import math
import sqlite3
from collections import defaultdict
from pathlib import Path

import numpy as np
import yaml
import zstandard
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image


PAIR_TOLERANCE_NS = 1_000_000


def _bag_files(path: Path) -> list[Path]:
    def index(member: Path) -> int:
        try:
            return int(member.stem.rsplit("_", 1)[1])
        except (IndexError, ValueError):
            raise ValueError(f"unrecognized rosbag2 split name: {member.name}") from None
    return sorted(path.glob("*.db3"), key=index)


def read_images(attempt_dir: Path, side: str) -> list[dict]:
    """Read raw ROS Image records without altering pixels or time domains."""
    if side not in ("left", "right"):
        raise ValueError("camera side must be left or right")
    bag = attempt_dir / "raw_images_or_lossless_bag" / side
    metadata = yaml.safe_load((bag / "metadata.yaml").read_text(encoding="utf-8"))
    info = metadata["rosbag2_bagfile_information"]
    compression = (info.get("compression_format") or "").lower()
    mode = (info.get("compression_mode") or "").upper()
    if compression and (compression != "zstd" or mode != "MESSAGE"):
        raise ValueError(f"unsupported raw-image compression: {compression}/{mode}")
    decoder = zstandard.ZstdDecompressor() if compression else None
    records: list[dict] = []
    epoch = 0
    last_stamp = None
    for db in _bag_files(bag):
        with sqlite3.connect(f"file:{db}?mode=ro", uri=True) as connection:
            result = connection.execute("PRAGMA integrity_check").fetchone()[0]
            if result != "ok":
                raise ValueError(f"SQLite integrity failed: {db.name}: {result}")
            topic = connection.execute(
                "SELECT id, type FROM topics WHERE name=?", (f"/{side}_camera/image",)
            ).fetchone()
            if topic is None or topic[1] != "sensor_msgs/msg/Image":
                raise ValueError(f"missing raw {side} Image topic in {db.name}")
            for message_id, receive_ns, serialized in connection.execute(
                "SELECT id, timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp,id",
                (topic[0],),
            ):
                raw = decoder.decompress(serialized) if decoder is not None else serialized
                message = deserialize_message(raw, Image)
                source_stamp = int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)
                if last_stamp is not None and source_stamp < last_stamp:
                    epoch += 1
                last_stamp = source_stamp
                width, height = int(message.width), int(message.height)
                encoding = str(message.encoding)
                pixels = bytes(message.data)
                if width <= 0 or height <= 0 or encoding not in ("rgb8", "bgr8"):
                    raise ValueError(f"unexpected raw {side} image shape or encoding")
                if int(message.step) != width * 3 or len(pixels) != width * height * 3:
                    raise ValueError(f"raw {side} image byte count/step mismatch")
                array = np.frombuffer(pixels, dtype=np.uint8)
                sequence = len(records) + 1
                records.append({
                    "camera_id": side,
                    "frame_id": f"{side}:{epoch}:{source_stamp}:{sequence:06d}",
                    "camera_sequence": sequence,
                    "simulation_epoch": epoch,
                    "image_timestamp_ns": source_stamp,
                    "receive_timestamp_ns": int(receive_ns),
                    "receive_time_domain": "rosbag2_record_utc_ns",
                    "source_header_frame_id": str(message.header.frame_id),
                    "image_width_px": width,
                    "image_height_px": height,
                    "encoding": encoding,
                    "image_bytes": len(pixels),
                    "image_pixels_sha256": hashlib.sha256(pixels).hexdigest(),
                    "uniform_image": bool(array.min() == array.max()),
                    "original_image_reference": f"raw_images_or_lossless_bag/{side}/{db.name}#message_id={message_id}",
                    "pair_id": None,
                })
    expected = next(
        (int(item["message_count"]) for item in info["topics_with_message_count"]
         if item["topic_metadata"]["name"] == f"/{side}_camera/image"),
        None,
    )
    if expected is None or expected != len(records):
        raise ValueError(f"{side} source Image count differs from rosbag metadata")
    if not records:
        raise ValueError(f"{side} raw image bag is empty")
    return records


def pair_frames(left: list[dict], right: list[dict], tolerance_ns: int = PAIR_TOLERANCE_NS) -> list[dict]:
    """Pair each source frame at most once within one epoch and ≤tolerance."""
    if tolerance_ns < 0:
        raise ValueError("pair tolerance must be nonnegative")
    candidates = []
    right_by_epoch = defaultdict(list)
    for ri, row in enumerate(right):
        right_by_epoch[row["simulation_epoch"]].append((int(row["image_timestamp_ns"]), ri))
    for records in right_by_epoch.values():
        records.sort()
    for li, lrow in enumerate(left):
        matched_epoch = right_by_epoch.get(lrow["simulation_epoch"], ())
        stamp = int(lrow["image_timestamp_ns"])
        start = bisect.bisect_left(matched_epoch, (stamp - tolerance_ns, -1))
        for index in range(start, len(matched_epoch)):
            right_stamp, ri = matched_epoch[index]
            if right_stamp > stamp + tolerance_ns:
                break
            delta = right_stamp - stamp
            candidates.append((abs(delta), max(stamp, right_stamp), li, ri, delta))
    candidates.sort()
    used_left, used_right = set(), set()
    pairs = []
    for _, _, li, ri, delta in candidates:
        if li in used_left or ri in used_right:
            continue
        used_left.add(li)
        used_right.add(ri)
        lrow, rrow = left[li], right[ri]
        pairs.append({
            "simulation_epoch": lrow["simulation_epoch"],
            "left_frame_id": lrow["frame_id"],
            "right_frame_id": rrow["frame_id"],
            "left_image_timestamp_ns": lrow["image_timestamp_ns"],
            "right_image_timestamp_ns": rrow["image_timestamp_ns"],
            "pair_delta_ns": delta,
            "pair_midpoint_timestamp_ns": lrow["image_timestamp_ns"] + delta // 2,
        })
    pairs.sort(key=lambda row: (row["simulation_epoch"], row["pair_midpoint_timestamp_ns"],
                                row["left_frame_id"], row["right_frame_id"]))
    for index, pair in enumerate(pairs, start=1):
        pair_id = f"pair-{index:06d}"
        pair["pair_id"] = pair_id
    by_frame = {pair["left_frame_id"]: pair["pair_id"] for pair in pairs}
    by_frame.update({pair["right_frame_id"]: pair["pair_id"] for pair in pairs})
    for row in (*left, *right):
        row["pair_id"] = by_frame.get(row["frame_id"])
    return pairs


def _stats(rows: list[dict]) -> dict:
    gaps = [b["image_timestamp_ns"] - a["image_timestamp_ns"]
            for a, b in zip(rows, rows[1:])
            if a["simulation_epoch"] == b["simulation_epoch"]]
    duration = rows[-1]["image_timestamp_ns"] - rows[0]["image_timestamp_ns"]
    return {
        "frame_count": len(rows),
        "unique_frame_id_count": len({row["frame_id"] for row in rows}),
        "duplicate_source_stamp_count": len(rows) - len({(row["simulation_epoch"], row["image_timestamp_ns"]) for row in rows}),
        "paired_frame_count": sum(row["pair_id"] is not None for row in rows),
        "unmatched_frame_count": sum(row["pair_id"] is None for row in rows),
        "uniform_image_count": sum(row["uniform_image"] for row in rows),
        "first_source_timestamp_ns": rows[0]["image_timestamp_ns"],
        "last_source_timestamp_ns": rows[-1]["image_timestamp_ns"],
        "observed_source_hz": (len(rows) - 1) * 1e9 / duration if duration > 0 else None,
        "max_source_gap_ns": max(gaps) if gaps else None,
        "simulation_epoch_count": 1 + max(row["simulation_epoch"] for row in rows),
    }


def index_attempt(attempt_dir: Path, run_id: str, attempt_id: str,
                  pair_tolerance_ns: int = PAIR_TOLERANCE_NS) -> dict:
    capture = json.loads((attempt_dir / "raw_capture_summary.json").read_text(encoding="utf-8"))
    if capture["status"] != "CAPTURE_COMPLETE":
        raise ValueError("raw capture was not finalized successfully")
    left = read_images(attempt_dir, "left")
    right = read_images(attempt_dir, "right")
    pairs = pair_frames(left, right, tolerance_ns=pair_tolerance_ns)
    index = sorted((*left, *right),
                   key=lambda row: (row["simulation_epoch"], row["image_timestamp_ns"], row["camera_id"], row["camera_sequence"]))
    for row in index:
        row["run_id"] = run_id
        row["attempt_id"] = attempt_id
    for pair in pairs:
        pair["run_id"] = run_id
        pair["attempt_id"] = attempt_id
    for filename, rows in (("image_index.jsonl", index), ("pair_index.jsonl", pairs)):
        with (attempt_dir / filename).open("x", encoding="utf-8") as handle:
            for row in rows:
                handle.write(json.dumps(row, ensure_ascii=False, sort_keys=True, allow_nan=False) + "\n")
    summary = {
        "schema_version": "stereo_v1_1_image_index_v1",
        "status": "INDEX_ONLY",
        "run_id": run_id,
        "attempt_id": attempt_id,
        "pair_tolerance_ns": pair_tolerance_ns,
        "pair_count": len(pairs),
        "exact_pair_count": sum(pair["pair_delta_ns"] == 0 for pair in pairs),
        "maximum_abs_pair_delta_ns": max((abs(pair["pair_delta_ns"]) for pair in pairs), default=None),
        "left": _stats(left),
        "right": _stats(right),
        "detection_processing_status": "NOT_RUN",
    }
    (attempt_dir / "image_index_summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8"
    )
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--attempt-dir", type=Path, required=True)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--attempt-id", required=True)
    parser.add_argument("--pair-tolerance-s", type=float, default=PAIR_TOLERANCE_NS / 1_000_000_000)
    args = parser.parse_args()
    if not math.isfinite(args.pair_tolerance_s) or args.pair_tolerance_s < 0:
        parser.error("--pair-tolerance-s must be finite and nonnegative")
    result = index_attempt(args.attempt_dir, args.run_id, args.attempt_id,
                           pair_tolerance_ns=round(args.pair_tolerance_s * 1_000_000_000))
    print(json.dumps({"status": result["status"], "pair_count": result["pair_count"],
                      "exact_pair_count": result["exact_pair_count"],
                      "left": result["left"]["frame_count"],
                      "right": result["right"]["frame_count"]}, ensure_ascii=False))


if __name__ == "__main__":
    main()
