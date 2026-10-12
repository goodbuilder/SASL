#!/usr/bin/env python3
"""Record v1.1 raw left/right camera images in separate rosbag2 files.

This process owns only the bag subprocesses it starts. Short tests may use
--duration-s; campaign execution sends SIGINT after post-science/landing.
SQLite3 rosbag serialization is lossless. Every source image retains its ROS
Image header stamp; bag message timestamps remain a distinct receive domain.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import signal
import subprocess
import time
from pathlib import Path

import yaml


STREAM_TOPICS = {
    "left": ("/left_camera/image", "/left_camera/camera_info"),
    "right": ("/right_camera/image", "/right_camera/camera_info"),
}
REQUIRED_NONEMPTY_TOPICS = {
    "left": "/left_camera/image",
    "right": "/right_camera/image",
}
STOP = False


def _request_stop(_signum, _frame):
    global STOP
    STOP = True


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def metadata_counts(path: Path) -> dict[str, int]:
    """Read recorder-written topic counts, retaining missing topics as absent."""
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    info = data["rosbag2_bagfile_information"]
    return {
        item["topic_metadata"]["name"]: int(item["message_count"])
        for item in info["topics_with_message_count"]
    }


def _start_bag(
    stream: str, topics: tuple[str, ...], raw_root: Path,
    attempt_dir: Path, compression: str,
):
    log_path = attempt_dir / f"capture_{stream}.log"
    log_handle = log_path.open("x", encoding="utf-8")
    command = [
        "ros2", "bag", "record", "-s", "sqlite3",
        "-o", str(raw_root / stream),
        "--max-bag-size", str(1_073_741_824),
        "--max-cache-size", str(268_435_456),
    ]
    if compression == "zstd":
        command.extend((
            "--compression-mode", "message", "--compression-format", "zstd",
            "--compression-threads", "2", "--compression-queue-size", "100",
        ))
    command.extend(topics)
    try:
        process = subprocess.Popen(
            command, stdout=log_handle, stderr=subprocess.STDOUT,
            start_new_session=True, text=True,
        )
    except BaseException:
        log_handle.close()
        raise
    return process, log_handle


def _subscribed(log_path: Path, topics: tuple[str, ...]) -> bool:
    if not log_path.is_file():
        return False
    content = log_path.read_text(encoding="utf-8", errors="replace")
    return all(f"Subscribed to topic '{topic}'" in content for topic in topics)


def _stop_owned(processes: dict[str, subprocess.Popen]) -> dict[str, int]:
    for process in processes.values():
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
    codes = {}
    for name, process in processes.items():
        try:
            codes[name] = process.wait(timeout=20)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                codes[name] = process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                codes[name] = process.wait(timeout=5)
    return codes


def capture(attempt_dir: Path, duration_s: float | None = None, compression: str = "zstd") -> dict:
    if compression not in ("none", "zstd"):
        raise ValueError("compression must be none or zstd")
    attempt_dir = attempt_dir.resolve()
    if not attempt_dir.is_dir():
        raise ValueError("attempt directory must already exist")
    raw_root = attempt_dir / "raw_images_or_lossless_bag"
    raw_root.mkdir(exist_ok=False)
    processes: dict[str, subprocess.Popen] = {}
    handles = []
    started = time.monotonic()
    reason = None
    try:
        for stream, topics in STREAM_TOPICS.items():
            process, handle = _start_bag(stream, topics, raw_root, attempt_dir, compression)
            processes[stream] = process
            handles.append(handle)
        ready_deadline = time.monotonic() + 20.0
        while time.monotonic() < ready_deadline:
            failed = {name: process.returncode for name, process in processes.items()
                      if process.poll() is not None}
            if failed:
                raise RuntimeError(f"bag recorder exited before readiness: {failed}")
            if all(_subscribed(attempt_dir / f"capture_{name}.log", topics)
                   for name, topics in STREAM_TOPICS.items()):
                break
            time.sleep(0.1)
        else:
            raise RuntimeError("bag subscriptions were not ready within 20 seconds")
        ready_at = time.monotonic()
        ready = {
            "status": "CAPTURE_RUNNING", "streams": list(STREAM_TOPICS),
            "topics": {name: list(topics) for name, topics in STREAM_TOPICS.items()},
            "capture_mode": "short_test" if duration_s is not None else "campaign_attempt",
            "compression": compression,
        }
        (attempt_dir / "capture_ready.json").write_text(
            json.dumps(ready, indent=2) + "\n", encoding="utf-8"
        )
        print("CAPTURE_READY", flush=True)
        while not STOP and (duration_s is None or time.monotonic() - ready_at < duration_s):
            failed = {name: process.returncode for name, process in processes.items()
                      if process.poll() is not None}
            if failed:
                raise RuntimeError(f"bag recorder stopped during capture: {failed}")
            time.sleep(0.25)
    except BaseException as exc:
        reason = f"{type(exc).__name__}: {exc}"
    finally:
        exit_codes = _stop_owned(processes)
        for handle in handles:
            handle.close()

    streams = {}
    for stream in STREAM_TOPICS:
        bag_dir = raw_root / stream
        metadata_path = bag_dir / "metadata.yaml"
        counts = metadata_counts(metadata_path) if metadata_path.is_file() else {}
        data_files = sorted(path for path in bag_dir.rglob("*") if path.is_file()) if bag_dir.exists() else []
        streams[stream] = {
            "topic_counts": counts,
            "files": [
                {
                    "path_from_attempt_root": str(path.relative_to(attempt_dir)),
                    "size_bytes": path.stat().st_size,
                    "sha256": _sha256(path),
                }
                for path in data_files
            ],
            "exit_code": exit_codes.get(stream),
        }
        if counts.get(REQUIRED_NONEMPTY_TOPICS[stream], 0) <= 0:
            reason = reason or f"no required {stream} messages recorded"
        if exit_codes.get(stream) not in (0, 130):
            reason = reason or f"{stream} recorder exit code {exit_codes.get(stream)}"
    result = {
        "schema_version": "stereo_v1_1_split_raw_capture_v1",
        "status": "CAPTURE_COMPLETE" if reason is None else "CAPTURE_FAILED",
        "failure_reason": reason,
        "duration_wall_s": time.monotonic() - started,
        "capture_mode": "short_test" if duration_s is not None else "campaign_attempt",
        "compression": compression,
        "streams": streams,
    }
    result_path = attempt_dir / "raw_capture_summary.json"
    with result_path.open("x", encoding="utf-8") as handle:
        json.dump(result, handle, ensure_ascii=False, indent=2, allow_nan=False)
        handle.write("\n")
        handle.flush()
        os.fsync(handle.fileno())
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--attempt-dir", type=Path, required=True)
    parser.add_argument("--duration-s", type=float)
    parser.add_argument("--compression", choices=("none", "zstd"), default="zstd")
    args = parser.parse_args()
    if args.duration_s is not None and args.duration_s < 5:
        parser.error("short capture duration must be at least 5 seconds")
    signal.signal(signal.SIGINT, _request_stop)
    signal.signal(signal.SIGTERM, _request_stop)
    result = capture(args.attempt_dir, args.duration_s, args.compression)
    print(json.dumps({"status": result["status"], "failure_reason": result["failure_reason"],
                      "message_counts": {k: v["topic_counts"].get(REQUIRED_NONEMPTY_TOPICS[k], 0)
                                         for k, v in result["streams"].items()}}, ensure_ascii=False))
    return 0 if result["status"] == "CAPTURE_COMPLETE" else 1


if __name__ == "__main__":
    raise SystemExit(main())
