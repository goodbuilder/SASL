"""Raw recorder schema and immutable attempt-directory boundary."""

from __future__ import annotations

import sys
from pathlib import Path

import pytest


SIM_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SIM_ROOT))

from campaigns.stereo_v1_1.capture_streams import (
    STREAM_TOPICS, _subscribed, capture, metadata_counts,
)


def test_raw_topics_are_separate_cameras():
    assert "/left_camera/image" in STREAM_TOPICS["left"]
    assert "/right_camera/image" in STREAM_TOPICS["right"]
    assert "/right_camera/image" not in STREAM_TOPICS["left"]
    assert "/left_camera/image" not in STREAM_TOPICS["right"]
    assert set(STREAM_TOPICS) == {"left", "right"}


def test_metadata_counts_and_existing_raw_directory_do_not_overwrite(tmp_path: Path):
    metadata = tmp_path / "metadata.yaml"
    metadata.write_text(
        "rosbag2_bagfile_information:\n"
        "  topics_with_message_count:\n"
        "    - topic_metadata: {name: /left_camera/image, type: sensor_msgs/msg/Image}\n"
        "      message_count: 47\n"
        "    - topic_metadata: {name: /clock, type: rosgraph_msgs/msg/Clock}\n"
        "      message_count: 100\n",
        encoding="utf-8",
    )
    assert metadata_counts(metadata) == {"/left_camera/image": 47, "/clock": 100}
    attempt = tmp_path / "attempt"
    raw = attempt / "raw_images_or_lossless_bag"
    raw.mkdir(parents=True)
    sentinel = raw / "previous-data.db3"
    sentinel.write_bytes(b"preserve")
    with pytest.raises(FileExistsError):
        capture(attempt, duration_s=5)
    assert sentinel.read_bytes() == b"preserve"


def test_capture_readiness_requires_actual_image_subscriptions(tmp_path: Path):
    log = tmp_path / "capture_right.log"
    log.write_text("Subscribed to topic '/right_camera/camera_info'\n", encoding="utf-8")
    assert not _subscribed(log, STREAM_TOPICS["right"])
    log.write_text(
        "Subscribed to topic '/right_camera/camera_info'\n"
        "Subscribed to topic '/right_camera/image'\n",
        encoding="utf-8",
    )
    assert _subscribed(log, STREAM_TOPICS["right"])
