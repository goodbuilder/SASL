#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
import time
from pathlib import Path
from typing import Sequence


PX4_NED_AXIS_CONTRACT = "px4_local_position_ned:north,east,down"
STEREODISTANCE_TOPIC_CONTRACT = "stereodistance_position_topics:FRD_range(forward,right,down,range)"
CALIBRATION_RDF_AXIS_CONTRACT = "stereo_center_opencv_optical:RDF(right,down,forward)"
NED_TO_RDF_CONTRACT = "rdf_right=east,rdf_down=down,rdf_forward=north"


CSV_FIELDS = [
    "time_sec",
    "ros_time_sec",
    "source_contract",
    "ned_axis_contract",
    "rdf_axis_contract",
    "public_topic_contract",
    "truth_north_m",
    "truth_east_m",
    "truth_down_ned_m",
    "truth_right_m",
    "truth_down_m",
    "truth_forward_m",
    "truth_range_m",
    "raw_forward_m",
    "raw_right_m",
    "raw_down_m",
    "raw_range_m",
    "estimated_forward_m",
    "estimated_right_m",
    "estimated_down_m",
    "estimated_range_m",
    "measured_disparity_px",
    "left_valid",
    "left_x1",
    "left_y1",
    "left_x2",
    "left_y2",
    "left_confidence",
    "left_cx_px",
    "left_cy_px",
    "right_valid",
    "right_x1",
    "right_y1",
    "right_x2",
    "right_y2",
    "right_confidence",
    "right_cx_px",
    "right_cy_px",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Record ROS2 PX4 truth and StereoDistance estimates to a calibration CSV. "
            "The CSV is directly consumable by rtk_stereo_calibration_pipeline.py --mode csv."
        )
    )
    parser.add_argument("--output-csv", type=Path, required=True)
    parser.add_argument("--hz", type=float, default=10.0)
    parser.add_argument("--truth-topic", default="/fmu/out/vehicle_local_position")
    parser.add_argument("--estimated-topic", default="/stereo/estimated_position")
    parser.add_argument("--detections-topic", default="/stereo/detections")
    parser.add_argument("--disparity-topic", default="/stereo/disparity")
    parser.add_argument(
        "--sim-truth-topic",
        default="",
        help=(
            "Optional Float32MultiArray FRD+range truth topic, e.g. /ground_truth_position. "
            "Used only when PX4 VehicleLocalPosition is unavailable."
        ),
    )
    return parser.parse_args()


def finite_or_empty(value: object) -> str:
    if value in (None, ""):
        return ""
    try:
        number = float(value)
    except (TypeError, ValueError):
        return ""
    if not math.isfinite(number):
        return ""
    return f"{number:.9f}"


def vector_range(values: Sequence[float]) -> float:
    return math.sqrt(sum(float(value) * float(value) for value in values))


def ned_to_rdf(north_m: float, east_m: float, down_m: float) -> dict[str, float]:
    return {
        "right_m": float(east_m),
        "down_m": float(down_m),
        "forward_m": float(north_m),
        "range_m": vector_range((east_m, down_m, north_m)),
    }


def frd_to_rdf(forward_m: float, right_m: float, down_m: float, range_m: float | None = None) -> dict[str, float]:
    computed_range = vector_range((right_m, down_m, forward_m))
    return {
        "right_m": float(right_m),
        "down_m": float(down_m),
        "forward_m": float(forward_m),
        "range_m": computed_range if range_m is None or not math.isfinite(float(range_m)) else float(range_m),
    }


def fmu_local_position_to_fields(msg: object) -> dict[str, float]:
    # PX4 VehicleLocalPosition uses x/y/z as local NED position. Keep the alias
    # local to this function so CSV output never exposes ambiguous x/y/z fields.
    north_m = float(getattr(msg, "x"))
    east_m = float(getattr(msg, "y"))
    down_m = float(getattr(msg, "z"))
    rdf = ned_to_rdf(north_m, east_m, down_m)
    return {
        "truth_north_m": north_m,
        "truth_east_m": east_m,
        "truth_down_ned_m": down_m,
        "truth_right_m": rdf["right_m"],
        "truth_down_m": rdf["down_m"],
        "truth_forward_m": rdf["forward_m"],
        "truth_range_m": rdf["range_m"],
    }


def frd_array_to_fields(
    data: Sequence[float],
    *,
    prefix: str,
    include_raw_alias: bool = False,
) -> dict[str, float]:
    values = list(data[:4])
    while len(values) < 4:
        values.append(math.nan)
    forward_m, right_m, down_m, range_m = [float(value) for value in values]
    rdf = frd_to_rdf(forward_m, right_m, down_m, range_m)
    fields = {
        f"{prefix}_forward_m": rdf["forward_m"],
        f"{prefix}_right_m": rdf["right_m"],
        f"{prefix}_down_m": rdf["down_m"],
        f"{prefix}_range_m": rdf["range_m"],
    }
    if include_raw_alias:
        fields.update(
            {
                "raw_forward_m": rdf["forward_m"],
                "raw_right_m": rdf["right_m"],
                "raw_down_m": rdf["down_m"],
                "raw_range_m": rdf["range_m"],
            }
        )
    return fields


def detections_array_to_fields(data: Sequence[float]) -> dict[str, float]:
    values = list(data[:16])
    while len(values) < 16:
        values.append(math.nan)
    fields = {}
    for offset, side in ((0, "left"), (8, "right")):
        valid, x1, y1, x2, y2, confidence, cx, cy = [float(value) for value in values[offset : offset + 8]]
        fields.update(
            {
                f"{side}_valid": valid,
                f"{side}_x1": x1,
                f"{side}_y1": y1,
                f"{side}_x2": x2,
                f"{side}_y2": y2,
                f"{side}_confidence": confidence,
                f"{side}_cx_px": cx,
                f"{side}_cy_px": cy,
            }
        )
    return fields


def write_manifest(path: Path, args: argparse.Namespace) -> None:
    lines = [
        "purpose=PX4/StereoDistance calibration CSV recorder",
        f"csv={path.name}",
        f"truth_topic={args.truth_topic}",
        f"sim_truth_topic={args.sim_truth_topic}",
        f"estimated_topic={args.estimated_topic}",
        f"detections_topic={args.detections_topic}",
        f"disparity_topic={args.disparity_topic}",
        f"ned_axis_contract={PX4_NED_AXIS_CONTRACT}",
        f"rdf_axis_contract={CALIBRATION_RDF_AXIS_CONTRACT}",
        f"public_topic_contract={STEREODISTANCE_TOPIC_CONTRACT}",
        f"ned_to_rdf_contract={NED_TO_RDF_CONTRACT}",
        "runtime_truth_contract=truth is recorded for offline calibration and validation only",
        "pipeline_command=Simulation/.venv/bin/python Simulation/scripts/rtk_stereo_calibration_pipeline.py --mode csv --input-csv <this csv> --output-dir Simulation/results/rtk_stereo_calibration_from_csv",
    ]
    (path.parent / "manifest.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")


class CalibrationCsvRecorder:
    def __init__(self, args: argparse.Namespace) -> None:
        try:
            import rclpy
            from rclpy.node import Node
            from std_msgs.msg import Float32, Float32MultiArray
        except Exception as exc:  # pragma: no cover - exercised only without ROS
            raise SystemExit(f"ROS2 Python packages are required to record live topics: {exc}") from exc

        self.rclpy = rclpy
        self.Float32 = Float32
        self.Float32MultiArray = Float32MultiArray
        self.args = args
        self.output_csv = args.output_csv.expanduser().resolve()
        self.output_csv.parent.mkdir(parents=True, exist_ok=True)
        write_manifest(self.output_csv, args)

        class RecorderNode(Node):
            pass

        self.node = RecorderNode("rtk_stereo_calibration_csv_recorder")
        self.latest: dict[str, object] = {}
        self.last_stamp: dict[str, float] = {}
        self.start_monotonic = time.monotonic()
        self.csv_file = self.output_csv.open("w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(self.csv_file, fieldnames=CSV_FIELDS)
        self.writer.writeheader()
        self._create_subscriptions()
        self.node.create_timer(1.0 / max(0.5, float(args.hz)), self.write_row)

    def _create_subscriptions(self) -> None:
        VehicleLocalPosition = None
        try:
            from px4_msgs.msg import VehicleLocalPosition as Px4VehicleLocalPosition

            VehicleLocalPosition = Px4VehicleLocalPosition
        except Exception:
            self.node.get_logger().warning(
                "px4_msgs VehicleLocalPosition is unavailable; PX4 truth subscription is disabled."
            )

        if VehicleLocalPosition is not None:
            self.node.create_subscription(
                VehicleLocalPosition,
                self.args.truth_topic,
                self._fmu_truth_cb,
                10,
            )
        if self.args.sim_truth_topic:
            self.node.create_subscription(
                self.Float32MultiArray,
                self.args.sim_truth_topic,
                self._sim_truth_cb,
                10,
            )
        self.node.create_subscription(
            self.Float32MultiArray,
            self.args.estimated_topic,
            self._estimated_cb,
            10,
        )
        self.node.create_subscription(
            self.Float32MultiArray,
            self.args.detections_topic,
            self._detections_cb,
            10,
        )
        self.node.create_subscription(
            self.Float32,
            self.args.disparity_topic,
            self._disparity_cb,
            10,
        )

    def _now_sec(self) -> float:
        return self.node.get_clock().now().nanoseconds * 1.0e-9

    def _fmu_truth_cb(self, msg: object) -> None:
        self.latest.update(fmu_local_position_to_fields(msg))
        self.last_stamp[self.args.truth_topic] = self._now_sec()

    def _sim_truth_cb(self, msg: object) -> None:
        values = list(getattr(msg, "data", [])[:4])
        while len(values) < 4:
            values.append(math.nan)
        forward_m, right_m, down_m, _range_m = [float(value) for value in values]
        self.latest.update(
            fmu_local_position_to_fields(
                type("LocalNed", (), {"x": forward_m, "y": right_m, "z": down_m})()
            )
        )
        self.last_stamp[self.args.sim_truth_topic] = self._now_sec()

    def _estimated_cb(self, msg: object) -> None:
        self.latest.update(
            frd_array_to_fields(
                list(getattr(msg, "data", [])),
                prefix="estimated",
                include_raw_alias=True,
            )
        )
        self.last_stamp[self.args.estimated_topic] = self._now_sec()

    def _detections_cb(self, msg: object) -> None:
        self.latest.update(detections_array_to_fields(list(getattr(msg, "data", []))))
        left_cx = self.latest.get("left_cx_px")
        right_cx = self.latest.get("right_cx_px")
        if left_cx is not None and right_cx is not None:
            try:
                self.latest["measured_disparity_px"] = abs(float(left_cx) - float(right_cx))
            except (TypeError, ValueError):
                pass
        self.last_stamp[self.args.detections_topic] = self._now_sec()

    def _disparity_cb(self, msg: object) -> None:
        self.latest["measured_disparity_px"] = float(getattr(msg, "data"))
        self.last_stamp[self.args.disparity_topic] = self._now_sec()

    def write_row(self) -> None:
        row: dict[str, str] = {
            "time_sec": f"{time.monotonic() - self.start_monotonic:.6f}",
            "ros_time_sec": f"{self._now_sec():.6f}",
            "source_contract": "px4_ned_truth+stereodistance_frd_estimate",
            "ned_axis_contract": PX4_NED_AXIS_CONTRACT,
            "rdf_axis_contract": CALIBRATION_RDF_AXIS_CONTRACT,
            "public_topic_contract": STEREODISTANCE_TOPIC_CONTRACT,
        }
        for field in CSV_FIELDS:
            row.setdefault(field, finite_or_empty(self.latest.get(field)))
        self.writer.writerow(row)
        self.csv_file.flush()

    def close(self) -> None:
        self.csv_file.close()

    def spin(self) -> None:
        try:
            self.rclpy.spin(self.node)
        finally:
            self.close()
            self.node.destroy_node()


def main() -> int:
    args = parse_args()
    try:
        import rclpy
    except Exception as exc:
        raise SystemExit(f"ROS2 Python packages are required to record live topics: {exc}") from exc
    rclpy.init()
    recorder = CalibrationCsvRecorder(args)
    try:
        recorder.spin()
    finally:
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
