#!/usr/bin/env python3
import argparse
from pathlib import Path

import cv2
from cv_bridge import CvBridge

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py


def topic_type_map(bag_dir: Path, storage_id: str = "sqlite3"):
    """
    Return dict: topic_name -> type_string
    (Humble: Info.read_metadata(uri, storage_id) 필요)
    """
    info = rosbag2_py.Info()
    meta = info.read_metadata(str(bag_dir), storage_id)
    m = {}
    for t in meta.topics_with_message_count:
        m[t.topic_metadata.name] = t.topic_metadata.type
    return m


def open_reader(bag_dir: Path, storage_id: str = "sqlite3"):
    storage_options = rosbag2_py.StorageOptions(
        uri=str(bag_dir),
        storage_id=storage_id,
    )
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    return reader


def first_frame_and_size(bag_dir: Path, type_map, topic_name: str, storage_id: str = "sqlite3"):
    """
    Read until first message for topic_name appears and return:
      (w, h, first_img)
    """
    reader = open_reader(bag_dir, storage_id)
    bridge = CvBridge()
    msg_type = get_message(type_map[topic_name])

    while reader.has_next():
        topic, data, _t_ns = reader.read_next()
        if topic != topic_name:
            continue
        msg = deserialize_message(data, msg_type)
        img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        h, w = img.shape[:2]
        return w, h, img

    raise RuntimeError(f"No messages found for topic: {topic_name}")


def main():
    ap = argparse.ArgumentParser(description="Convert ROS2 rosbag2 image topics to MP4 (directly from bag)")
    ap.add_argument("--bag", required=True, help="rosbag2 directory (contains metadata.yaml)")
    ap.add_argument("--out_dir", required=True, help="output directory for mp4")
    ap.add_argument("--left_topic", default="/wiris/left/image_raw")
    ap.add_argument("--right_topic", default="/wiris/right/image_raw")
    ap.add_argument("--left_fps", type=float, default=9.18)
    ap.add_argument("--right_fps", type=float, default=9.10)
    ap.add_argument("--storage_id", default="sqlite3", help="sqlite3 (default)")
    ap.add_argument("--codec", default="mp4v", help="mp4v (default). If not playable, try avc1")
    args = ap.parse_args()

    bag_dir = Path(args.bag).resolve()
    out_dir = Path(args.out_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    if not (bag_dir / "metadata.yaml").exists():
        raise FileNotFoundError(f"metadata.yaml not found in: {bag_dir}")

    # topic -> type
    type_map = topic_type_map(bag_dir, args.storage_id)

    # Validate topics exist in bag
    for t in [args.left_topic, args.right_topic]:
        if t not in type_map:
            available = "\n".join(sorted(type_map.keys()))
            raise RuntimeError(
                f"Topic not found in bag: {t}\n\nAvailable topics:\n{available}"
            )

    # Get size from first frame
    wL, hL, firstL = first_frame_and_size(bag_dir, type_map, args.left_topic, args.storage_id)
    wR, hR, firstR = first_frame_and_size(bag_dir, type_map, args.right_topic, args.storage_id)

    bag_name = bag_dir.name
    left_mp4 = out_dir / f"{bag_name}_left.mp4"
    right_mp4 = out_dir / f"{bag_name}_right.mp4"

    fourcc = cv2.VideoWriter_fourcc(*args.codec)
    left_writer = cv2.VideoWriter(str(left_mp4), fourcc, float(args.left_fps), (wL, hL))
    right_writer = cv2.VideoWriter(str(right_mp4), fourcc, float(args.right_fps), (wR, hR))

    if not left_writer.isOpened():
        raise RuntimeError(f"Failed to open VideoWriter for left mp4: {left_mp4} (try --codec avc1)")
    if not right_writer.isOpened():
        raise RuntimeError(f"Failed to open VideoWriter for right mp4: {right_mp4} (try --codec avc1)")

    # Write the first frames so mp4 has content even if bag is tiny
    left_writer.write(firstL)
    right_writer.write(firstR)

    bridge = CvBridge()
    left_msg_type = get_message(type_map[args.left_topic])
    right_msg_type = get_message(type_map[args.right_topic])

    # Full pass
    reader = open_reader(bag_dir, args.storage_id)

    left_count = 0
    right_count = 0

    print(f"[INFO] Bag: {bag_dir}")
    print(f"[INFO] Left : topic={args.left_topic}, size={wL}x{hL}, fps={args.left_fps}, out={left_mp4}")
    print(f"[INFO] Right: topic={args.right_topic}, size={wR}x{hR}, fps={args.right_fps}, out={right_mp4}")

    while reader.has_next():
        topic, data, _t_ns = reader.read_next()

        if topic == args.left_topic:
            msg = deserialize_message(data, left_msg_type)
            img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            if img.shape[1] != wL or img.shape[0] != hL:
                img = cv2.resize(img, (wL, hL))
            left_writer.write(img)
            left_count += 1

        elif topic == args.right_topic:
            msg = deserialize_message(data, right_msg_type)
            img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            if img.shape[1] != wR or img.shape[0] != hR:
                img = cv2.resize(img, (wR, hR))
            right_writer.write(img)
            right_count += 1

    left_writer.release()
    right_writer.release()

    print(f"[DONE] frames: left={left_count}, right={right_count}")
    print(f"[DONE] saved:\n  {left_mp4}\n  {right_mp4}")


if __name__ == "__main__":
    main()
