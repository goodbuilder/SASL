#!/usr/bin/env python3
from __future__ import annotations

import argparse
import statistics
import threading
import time

import gi

gi.require_version("Gst", "1.0")
from gi.repository import Gst  # noqa: E402


def gst_string_literal(value: str) -> str:
    escaped = str(value).replace("\\", "\\\\").replace('"', '\\"').replace("\n", "\\n")
    return f'"{escaped}"'


def build_pipeline(url: str, latency_ms: int):
    pipeline_text = (
        f"rtspsrc location={gst_string_literal(url)} latency={latency_ms} protocols=tcp drop-on-latency=true "
        "! rtph264depay "
        "! queue leaky=downstream max-size-buffers=1 max-size-bytes=0 max-size-time=0 "
        "! h264parse "
        "! queue leaky=downstream max-size-buffers=1 max-size-bytes=0 max-size-time=0 "
        "! avdec_h264 "
        "! queue leaky=downstream max-size-buffers=1 max-size-bytes=0 max-size-time=0 "
        "! videoconvert "
        "! video/x-raw,format=BGR "
        "! appsink name=sink emit-signals=false sync=false max-buffers=1 drop=true"
    )
    return Gst.parse_launch(pipeline_text)


def collect(name: str, url: str, *, duration_s: float, latency_ms: int, results: dict) -> None:
    rows: list[dict[str, float | int | None]] = []
    errors: list[str] = []
    pipeline = build_pipeline(url, latency_ms)
    sink = pipeline.get_by_name("sink")
    bus = pipeline.get_bus()
    pipeline.set_state(Gst.State.PLAYING)
    start_s = time.monotonic()
    first_pts_s: float | None = None
    try:
        while time.monotonic() - start_s < duration_s:
            message = bus.pop_filtered(Gst.MessageType.ERROR | Gst.MessageType.EOS)
            if message is not None:
                if message.type == Gst.MessageType.ERROR:
                    error, debug = message.parse_error()
                    detail = error.message if debug is None else f"{error.message} ({debug})"
                    errors.append(detail)
                else:
                    errors.append("EOS")
                break

            sample = sink.emit("try-pull-sample", 200_000_000)
            if sample is None:
                continue

            recv_s = time.monotonic()
            buffer = sample.get_buffer()
            pts_s = None if buffer.pts == Gst.CLOCK_TIME_NONE else buffer.pts / 1e9
            dts_s = None if buffer.dts == Gst.CLOCK_TIME_NONE else buffer.dts / 1e9
            if pts_s is not None and first_pts_s is None:
                first_pts_s = pts_s
            caps = sample.get_caps().get_structure(0)
            rows.append(
                {
                    "recv_rel_s": recv_s - start_s,
                    "pts_abs_s": pts_s,
                    "pts_rel_s": None if pts_s is None or first_pts_s is None else pts_s - first_pts_s,
                    "dts_abs_s": dts_s,
                    "width": int(caps.get_value("width")),
                    "height": int(caps.get_value("height")),
                }
            )
    finally:
        pipeline.set_state(Gst.State.NULL)
    results[name] = {"rows": rows, "errors": errors}


def print_stream_summary(name: str, rows: list[dict], errors: list[str]) -> None:
    print(f"[{name}] frames={len(rows)} errors={errors}")
    if not rows:
        return
    recv = [row["recv_rel_s"] for row in rows]
    pts_rel = [row["pts_rel_s"] for row in rows if row["pts_rel_s"] is not None]
    pts_abs = [row["pts_abs_s"] for row in rows if row["pts_abs_s"] is not None]
    recv_intervals = [b - a for a, b in zip(recv, recv[1:])]
    pts_intervals = [b - a for a, b in zip(pts_rel, pts_rel[1:])]
    recv_hz = (len(recv) - 1) / (recv[-1] - recv[0]) if len(recv) > 1 and recv[-1] > recv[0] else 0.0
    print(
        f"[{name}] size={rows[-1]['width']}x{rows[-1]['height']} "
        f"recv_span={recv[-1] - recv[0]:.3f}s recv_hz={recv_hz:.2f}"
    )
    if recv_intervals:
        print(
            f"[{name}] recv_dt avg={statistics.mean(recv_intervals) * 1000:.1f}ms "
            f"min={min(recv_intervals) * 1000:.1f} max={max(recv_intervals) * 1000:.1f}"
        )
    if pts_rel:
        print(
            f"[{name}] pts_abs_first={pts_abs[0]:.6f}s "
            f"pts_abs_last={pts_abs[-1]:.6f}s pts_span={pts_rel[-1] - pts_rel[0]:.3f}s"
        )
    if pts_intervals:
        print(
            f"[{name}] pts_dt avg={statistics.mean(pts_intervals) * 1000:.1f}ms "
            f"min={min(pts_intervals) * 1000:.1f} max={max(pts_intervals) * 1000:.1f}"
        )


def print_pair_summary(left_rows: list[dict], right_rows: list[dict]) -> None:
    left_pts = [row for row in left_rows if row["pts_rel_s"] is not None]
    right_pts = [row for row in right_rows if row["pts_rel_s"] is not None]
    if not left_pts or not right_pts:
        print("[pair] PTS unavailable on one or both streams")
        return

    pairs = []
    right_index = 0
    for left in left_pts:
        while (
            right_index + 1 < len(right_pts)
            and abs(right_pts[right_index + 1]["pts_rel_s"] - left["pts_rel_s"])
            <= abs(right_pts[right_index]["pts_rel_s"] - left["pts_rel_s"])
        ):
            right_index += 1
        right = right_pts[right_index]
        pairs.append(
            (
                left,
                right,
                left["pts_rel_s"] - right["pts_rel_s"],
                left["recv_rel_s"] - right["recv_rel_s"],
            )
        )

    pts_diffs = [pair[2] for pair in pairs]
    recv_diffs = [pair[3] for pair in pairs]
    print(
        f"[pair/relative PTS] pairs={len(pairs)} left-right "
        f"avg={statistics.mean(pts_diffs) * 1000:.2f}ms "
        f"median={statistics.median(pts_diffs) * 1000:.2f}ms "
        f"min={min(pts_diffs) * 1000:.2f}ms max={max(pts_diffs) * 1000:.2f}ms"
    )
    print(
        f"[pair/recv time] left-right "
        f"avg={statistics.mean(recv_diffs) * 1000:.2f}ms "
        f"median={statistics.median(recv_diffs) * 1000:.2f}ms "
        f"min={min(recv_diffs) * 1000:.2f}ms max={max(recv_diffs) * 1000:.2f}ms"
    )
    if left_pts[0]["pts_abs_s"] is not None and right_pts[0]["pts_abs_s"] is not None:
        print(f"[absolute PTS] first left-right={(left_pts[0]['pts_abs_s'] - right_pts[0]['pts_abs_s']) * 1000:.2f}ms")


def main() -> int:
    parser = argparse.ArgumentParser(description="Sample two RTSP streams and compare receive/PTS time axes.")
    parser.add_argument("--left", default="rtsp://10.0.0.231:8554/visible")
    parser.add_argument("--right", default="rtsp://10.0.0.232:8554/visible")
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--latency-ms", type=int, default=50)
    args = parser.parse_args()

    Gst.init(None)
    results: dict[str, dict] = {}
    threads = [
        threading.Thread(
            target=collect,
            args=("left", args.left),
            kwargs={"duration_s": args.duration, "latency_ms": args.latency_ms, "results": results},
            daemon=True,
        ),
        threading.Thread(
            target=collect,
            args=("right", args.right),
            kwargs={"duration_s": args.duration, "latency_ms": args.latency_ms, "results": results},
            daemon=True,
        ),
    ]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()

    left = results.get("left", {"rows": [], "errors": ["missing result"]})
    right = results.get("right", {"rows": [], "errors": ["missing result"]})
    print_stream_summary("left", left["rows"], left["errors"])
    print_stream_summary("right", right["rows"], right["errors"])
    print_pair_summary(left["rows"], right["rows"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
