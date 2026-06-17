#!/usr/bin/env python3
import argparse
from pathlib import Path

import cv2
from cv_bridge import CvBridge

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

import rosbag2_py


def ensure_outdir(p: Path):
    p.mkdir(parents=True, exist_ok=True)


def topic_type_map(bag_dir: Path):
    """Return dict: topic_name -> type_string"""
    info = rosbag2_py.Info()
    meta = info.read_metadata(str(bag_dir), "sqlite3")
    m = {}
    for t in meta.topics_with_message_count:
        m[t.topic_metadata.name] = t.topic_metadata.type
    return m


def infer_size_from_first_frame(reader, type_map, topic_name):
    """Read until first frame of topic_name appears; return (w,h,stamp_ns, cv_img)."""
    bridge = CvBridge()
    msg_type = get_message(type_map[topic_name])

    while reader.has_next():
        (topic, data, t_ns) = reader.read_next()
        if topic != topic_name:
            continue
        msg = deserialize_message(data, msg_type)
        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        h, w = cv_img.shape[:2]
        return w, h, t_ns, cv_img
    raise RuntimeError(f"No messages found for topic: {topic_name}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True, help="rosbag2 directory (contains metadata.yaml)")
    ap.add_argument("--out_dir", required=True, help="output directory")
    ap.add_argument("--left_topic", default="/wiris/left/image_raw")
    ap.add_argument("--right_topic", default="/wiris/right/image_raw")
    ap.add_argument("--left_fps", type=float, default=0.0, help="0이면 타임스탬프로 추정")
    ap.add_argument("--right_fps", type=float, default=0.0, help="0이면 타임스탬프로 추정")
    ap.add_argument("--codec", default="mp4v", help="mp4v or avc1 등")
    args = ap.parse_args()

    bag_dir = Path(args.bag).resolve()
    out_dir = Path(args.out_dir).resolve()
    ensure_outdir(out_dir)

    if not (bag_dir / "metadata.yaml").exists():
        raise FileNotFoundError(f"metadata.yaml not found in: {bag_dir}")

    type_map = topic_type_map(bag_dir)
    for t in [args.left_topic, args.right_topic]:
        if t not in type_map:
            raise RuntimeError(
                f"Topic not in bag: {t}\n"
                f"Available topics:\n" + "\n".join(sorted(type_map.keys()))
            )

    # --- open reader ---
    storage_options = rosbag2_py.StorageOptions(
        uri=str(bag_dir),
        storage_id="sqlite3",
    )
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    # To infer video size and also grab first frames, we need a fresh reader twice.
    # (SequentialReader can't rewind.)
    # 1) first pass: get first frame for each topic and infer size
    reader1 = rosbag2_py.SequentialReader()
    reader1.open(storage_options, converter_options)

    wL, hL, tL0, firstL = infer_size_from_first_frame(reader1, type_map, args.left_topic)

    reader2 = rosbag2_py.SequentialReader()
    reader2.open(storage_options, converter_options)
    wR, hR, tR0, firstR = infer_size_from_first_frame(reader2, type_map, args.right_topic)

    # FPS estimation helper
    def est_fps_from_ns(delta_ns_list):
        if len(delta_ns_list) < 3:
            return 10.0
        # median dt
        ds = sorted(delta_ns_list)
        med = ds[len(ds)//2] / 1e9
        if med <= 0:
            return 10.0
        return max(1.0, min(120.0, 1.0 / med))

    # output files
    bag_name = bag_dir.name
    left_mp4 = out_dir / f"{bag_name}_left.mp4"
    right_mp4 = out_dir / f"{bag_name}_right.mp4"

    fourcc = cv2.VideoWriter_fourcc(*args.codec)

    # If fps is 0, estimate from timestamps on-the-fly (first N deltas)
    left_deltas = []
    right_deltas = []
    left_prev_t = None
    right_prev_t = None

    # We'll create writers after we know fps (either given or estimated).
    left_writer = None
    right_writer = None
    left_fps = args.left_fps
    right_fps = args.right_fps

    # fresh reader for full pass
    reader3 = rosbag2_py.SequentialReader()
    reader3.open(storage_options, converter_options)

    bridge = CvBridge()
    left_msg_type = get_message(type_map[args.left_topic])
    right_msg_type = get_message(type_map[args.right_topic])

    # write first frames later when writer is ready
    pending_firstL = firstL
    pending_firstR = firstR

    print(f"[INFO] Bag: {bag_dir}")
    print(f"[INFO] Left topic:  {args.left_topic} ({wL}x{hL})")
    print(f"[INFO] Right topic: {args.right_topic} ({wR}x{hR})")
    print(f"[INFO] Output: {left_mp4} / {right_mp4}")

    while reader3.has_next():
        topic, data, t_ns = reader3.read_next()

        if topic == args.left_topic:
            if left_prev_t is not None and len(left_deltas) < 120:
                left_deltas.append(t_ns - left_prev_t)
            left_prev_t = t_ns

            if left_writer is None:
                if left_fps <= 0.0 and len(left_deltas) >= 10:
                    left_fps = est_fps_from_ns(left_deltas)
                if left_fps > 0.0:
                    left_writer = cv2.VideoWriter(str(left_mp4), fourcc, left_fps, (wL, hL))
                    if pending_firstL is not None:
                        left_writer.write(pending_firstL)
                        pending_firstL = None

            msg = deserialize_message(data, left_msg_type)
            img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            if left_writer is not None:
                if img.shape[1] != wL or img.shape[0] != hL:
                    img = cv2.resize(img, (wL, hL))
                left_writer.write(img)

        elif topic == args.right_topic:
            if right_prev_t is not None and len(right_deltas) < 120:
                right_deltas.append(t_ns - right_prev_t)
            right_prev_t = t_ns

            if right_writer is None:
                if right_fps <= 0.0 and len(right_deltas) >= 10:
                    right_fps = est_fps_from_ns(right_deltas)
                if right_fps > 0.0:
                    right_writer = cv2.VideoWriter(str(right_mp4), fourcc, right_fps, (wR, hR))
                    if pending_firstR is not None:
                        right_writer.write(pending_firstR)
                        pending_firstR = None

            msg = deserialize_message(data, right_msg_type)
            img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            if right_writer is not None:
                if img.shape[1] != wR or img.shape[0] != hR:
                    img = cv2.resize(img, (wR, hR))
                right_writer.write(img)

    if left_writer is None:
        # fallback: create writer with default fps and write first frame only
        left_fps = left_fps if left_fps > 0 else est_fps_from_ns(left_deltas)
        left_writer = cv2.VideoWriter(str(left_mp4), fourcc, left_fps, (wL, hL))
        left_writer.write(firstL)

    if right_writer is None:
        right_fps = right_fps if right_fps > 0 else est_fps_from_ns(right_deltas)
        right_writer = cv2.VideoWriter(str(right_mp4), fourcc, right_fps, (wR, hR))
        right_writer.write(firstR)

    left_writer.release()
    right_writer.release()

    print(f"[DONE] left_fps={left_fps:.3f}, right_fps={right_fps:.3f}")
    print(f"[DONE] Saved:\n  {left_mp4}\n  {right_mp4}")


if __name__ == "__main__":
    main()
