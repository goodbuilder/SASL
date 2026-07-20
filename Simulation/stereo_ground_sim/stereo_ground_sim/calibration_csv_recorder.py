#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import time
from pathlib import Path
from typing import Sequence

from sasl_common.truth_coordinates import (
    GLOBAL_LLA_AXIS_CONTRACT,
    GLOBAL_LLA_FRAME,
    LOCAL_NED_AXIS_CONTRACT,
    LOCAL_NED_FRAME,
    local_ned_from_lla,
    local_ned_from_xyz,
)
from sasl_common.stereo_geometry import BASELINE_CENTER_FRD_FRAME, STEREO_CENTER_OPTICAL_FRAME


PX4_NED_AXIS_CONTRACT = "px4_local_position_ned:north,east,down"
PX4_GLOBAL_AXIS_CONTRACT = "px4_global_lla:lat,lon,alt"
STEREODISTANCE_TOPIC_CONTRACT = f"{BASELINE_CENTER_FRD_FRAME}:FRD_range(forward,right,down,range)"
CALIBRATION_RDF_AXIS_CONTRACT = f"{STEREO_CENTER_OPTICAL_FRAME}:RDF(right,down,forward)"
TRUTH_RDF_TRANSFORM_CONTRACT = (
    "explicit_transform_required: truth RDF must come from a declared T_source_to_stereo_rdf "
    "or a stereo-frame truth producer; direct source-frame axis aliasing is forbidden"
)
NED_TO_RDF_CONTRACT = TRUTH_RDF_TRANSFORM_CONTRACT


CSV_FIELDS = [
    "time_sec",
    "ros_time_sec",
    "source_contract",
    "ned_axis_contract",
    "rdf_axis_contract",
    "public_topic_contract",
    "truth_source_type",
    "truth_source_frame",
    "truth_source_axis_contract",
    "truth_conversion_status",
    "truth_raw_lat_deg",
    "truth_raw_lon_deg",
    "truth_raw_alt_m",
    "truth_origin_lat_deg",
    "truth_origin_lon_deg",
    "truth_origin_alt_m",
    "truth_origin_kind",
    "truth_north_m",
    "truth_east_m",
    "truth_down_ned_m",
    "truth_right_m",
    "truth_down_m",
    "truth_forward_m",
    "truth_range_m",
    "local_truth_source_type",
    "local_truth_source_frame",
    "local_truth_source_axis_contract",
    "local_truth_conversion_status",
    "local_truth_north_m",
    "local_truth_east_m",
    "local_truth_down_ned_m",
    "global_truth_source_type",
    "global_truth_source_frame",
    "global_truth_source_axis_contract",
    "global_truth_conversion_status",
    "global_truth_raw_lat_deg",
    "global_truth_raw_lon_deg",
    "global_truth_raw_alt_m",
    "global_truth_origin_lat_deg",
    "global_truth_origin_lon_deg",
    "global_truth_origin_alt_m",
    "global_truth_origin_kind",
    "global_truth_north_m",
    "global_truth_east_m",
    "global_truth_down_ned_m",
    "gps_truth_source_type",
    "gps_truth_source_frame",
    "gps_truth_source_axis_contract",
    "gps_truth_conversion_status",
    "gps_truth_raw_lat_deg",
    "gps_truth_raw_lon_deg",
    "gps_truth_raw_alt_m",
    "gps_truth_origin_lat_deg",
    "gps_truth_origin_lon_deg",
    "gps_truth_origin_alt_m",
    "gps_truth_origin_kind",
    "gps_truth_north_m",
    "gps_truth_east_m",
    "gps_truth_down_ned_m",
    "sim_truth_role",
    "sim_truth_source_frame",
    "sim_truth_axis_contract",
    "sim_truth_forward_m",
    "sim_truth_right_m",
    "sim_truth_down_m",
    "sim_truth_range_m",
    "raw_forward_m",
    "raw_right_m",
    "raw_down_m",
    "raw_range_m",
    "triangulation_valid",
    "triangulation_reason",
    "ray_closest_gap_m",
    "ray_left_depth_m",
    "ray_right_depth_m",
    "ray_angle_deg",
    "triangulation_point_frame",
    "triangulation_frame",
    "triangulation_method",
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
            "Record raw ROS2 PX4 truth, StereoDistance ray estimates, detections, "
            "and triangulation quality to CSV without applying a runtime correction."
        )
    )
    parser.add_argument("--output-csv", type=Path, required=True)
    parser.add_argument("--hz", type=float, default=10.0)
    parser.add_argument("--truth-topic", default="/fmu/out/vehicle_local_position")
    parser.add_argument("--local-truth-topic", default="/fmu/out/vehicle_local_position")
    parser.add_argument("--global-truth-topic", default="/fmu/out/vehicle_global_position")
    parser.add_argument("--gps-truth-topic", default="/fmu/out/vehicle_gps_position")
    parser.add_argument(
        "--truth-source",
        choices=("local_ned", "global_position", "gps_position"),
        default="local_ned",
        help="PX4 truth topic contract. GPS/global sources are converted to local NED before CSV output.",
    )
    parser.add_argument("--estimated-topic", default="/stereo/estimated_position")
    parser.add_argument("--detections-topic", default="/stereo/detections")
    parser.add_argument(
        "--triangulation-status-topic",
        default="/stereo/triangulation_status",
    )
    parser.add_argument(
        "--sim-truth-topic",
        default="",
        help=(
            "Optional Float32MultiArray FRD+range truth topic, e.g. /ground_truth_position. "
            "Recorded as oracle by default; use --sim-truth-role training to copy it into truth_* fields."
        ),
    )
    parser.add_argument(
        "--sim-truth-role",
        choices=("oracle", "training"),
        default="oracle",
        help="oracle records Gazebo/Simulation truth in sim_truth_* only; training also fills truth_right/down/forward.",
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


def csv_value(value: object) -> str:
    if isinstance(value, str):
        return value
    return finite_or_empty(value)


def vector_range(values: Sequence[float]) -> float:
    return math.sqrt(sum(float(value) * float(value) for value in values))


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
    # Do not derive stereo-center RDF truth here; that requires an explicit
    # camera-to-NED extrinsic and reference-point contract.
    converted = local_ned_from_xyz(getattr(msg, "x"), getattr(msg, "y"), getattr(msg, "z"), source="ros2:local_ned")
    if converted is None:
        return {}
    return {
        "truth_source_type": "local_ned",
        "truth_source_frame": converted.source_frame,
        "truth_source_axis_contract": converted.source_axis_contract,
        "truth_conversion_status": converted.conversion_status,
        "truth_north_m": converted.north_m,
        "truth_east_m": converted.east_m,
        "truth_down_ned_m": converted.down_m,
    }


def prefixed_fields(fields: dict[str, object], prefix: str) -> dict[str, object]:
    mapping = {
        "truth_source_type": f"{prefix}_truth_source_type",
        "truth_source_frame": f"{prefix}_truth_source_frame",
        "truth_source_axis_contract": f"{prefix}_truth_source_axis_contract",
        "truth_conversion_status": f"{prefix}_truth_conversion_status",
        "truth_raw_lat_deg": f"{prefix}_truth_raw_lat_deg",
        "truth_raw_lon_deg": f"{prefix}_truth_raw_lon_deg",
        "truth_raw_alt_m": f"{prefix}_truth_raw_alt_m",
        "truth_origin_lat_deg": f"{prefix}_truth_origin_lat_deg",
        "truth_origin_lon_deg": f"{prefix}_truth_origin_lon_deg",
        "truth_origin_alt_m": f"{prefix}_truth_origin_alt_m",
        "truth_origin_kind": f"{prefix}_truth_origin_kind",
        "truth_north_m": f"{prefix}_truth_north_m",
        "truth_east_m": f"{prefix}_truth_east_m",
        "truth_down_ned_m": f"{prefix}_truth_down_ned_m",
    }
    return {mapped: fields[key] for key, mapped in mapping.items() if key in fields}


def global_position_to_fields(
    msg: object,
    *,
    reference: tuple[float, float, float] | None,
    origin_kind: str,
    source_type: str,
) -> tuple[dict[str, float | str], tuple[float, float, float] | None]:
    lat = getattr(msg, "lat")
    lon = getattr(msg, "lon")
    alt = getattr(msg, "alt")
    if reference is None:
        reference_sample = local_ned_from_lla(
            lat=lat,
            lon=lon,
            alt=alt,
            reference_lat_deg=float(lat) if abs(float(lat)) <= 1000.0 else float(lat) * 1.0e-7,
            reference_lon_deg=float(lon) if abs(float(lon)) <= 1000.0 else float(lon) * 1.0e-7,
            reference_alt_m=float(alt) if abs(float(alt)) <= 20_000.0 else float(alt) * 1.0e-3,
            source=f"ros2:{source_type}",
            origin_kind=origin_kind,
        )
        if reference_sample is None:
            return {}, None
        reference = (
            float(reference_sample.raw_lat_deg),
            float(reference_sample.raw_lon_deg),
            float(reference_sample.raw_alt_m),
        )
    converted = local_ned_from_lla(
        lat=lat,
        lon=lon,
        alt=alt,
        reference_lat_deg=reference[0],
        reference_lon_deg=reference[1],
        reference_alt_m=reference[2],
        source=f"ros2:{source_type}",
        origin_kind=origin_kind,
    )
    if converted is None:
        return {}, reference
    return {
        "truth_source_type": source_type,
        "truth_source_frame": converted.source_frame,
        "truth_source_axis_contract": converted.source_axis_contract,
        "truth_conversion_status": converted.conversion_status,
        "truth_raw_lat_deg": converted.raw_lat_deg,
        "truth_raw_lon_deg": converted.raw_lon_deg,
        "truth_raw_alt_m": converted.raw_alt_m,
        "truth_origin_lat_deg": converted.origin_lat_deg,
        "truth_origin_lon_deg": converted.origin_lon_deg,
        "truth_origin_alt_m": converted.origin_alt_m,
        "truth_origin_kind": converted.origin_kind,
        "truth_north_m": converted.north_m,
        "truth_east_m": converted.east_m,
        "truth_down_ned_m": converted.down_m,
    }, reference


def frd_array_to_fields(
    data: Sequence[float],
    *,
    prefix: str,
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
    truth_source = str(getattr(args, "truth_source", "local_ned") or "local_ned")
    lines = [
        "purpose=PX4/StereoDistance raw ray-measurement CSV recorder",
        f"csv={path.name}",
        f"truth_topic={args.truth_topic}",
        f"local_truth_topic={getattr(args, 'local_truth_topic', '/fmu/out/vehicle_local_position')}",
        f"global_truth_topic={getattr(args, 'global_truth_topic', '/fmu/out/vehicle_global_position')}",
        f"gps_truth_topic={getattr(args, 'gps_truth_topic', '/fmu/out/vehicle_gps_position')}",
        f"truth_source={truth_source}",
        f"sim_truth_topic={args.sim_truth_topic}",
        f"sim_truth_role={getattr(args, 'sim_truth_role', 'oracle')}",
        f"estimated_topic={args.estimated_topic}",
        f"detections_topic={args.detections_topic}",
        f"triangulation_status_topic={args.triangulation_status_topic}",
        f"ned_axis_contract={PX4_NED_AXIS_CONTRACT}",
        f"global_axis_contract={PX4_GLOBAL_AXIS_CONTRACT}",
        f"rdf_axis_contract={CALIBRATION_RDF_AXIS_CONTRACT}",
        f"public_topic_contract={STEREODISTANCE_TOPIC_CONTRACT}",
        f"truth_rdf_transform_contract={TRUTH_RDF_TRANSFORM_CONTRACT}",
        f"truth_rdf_contract={NED_TO_RDF_CONTRACT}",
        "runtime_truth_contract=truth is recorded for offline calibration and validation only",
        "postprocessing=none; this manifest records raw inputs and outputs only",
    ]
    (path.parent / "manifest.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")


class CalibrationCsvRecorder:
    def __init__(self, args: argparse.Namespace) -> None:
        try:
            import rclpy
            from rclpy.node import Node
            from std_msgs.msg import Float32MultiArray, String
        except Exception as exc:  # pragma: no cover - exercised only without ROS
            raise SystemExit(f"ROS2 Python packages are required to record live topics: {exc}") from exc

        self.rclpy = rclpy
        self.Float32MultiArray = Float32MultiArray
        self.String = String
        self.args = args
        self.output_csv = args.output_csv.expanduser().resolve()
        self.output_csv.parent.mkdir(parents=True, exist_ok=True)
        write_manifest(self.output_csv, args)

        class RecorderNode(Node):
            pass

        self.node = RecorderNode("rtk_stereo_calibration_csv_recorder")
        self.latest: dict[str, object] = {}
        self.last_stamp: dict[str, float] = {}
        self.global_truth_lla_reference: tuple[float, float, float] | None = None
        self.gps_truth_lla_reference: tuple[float, float, float] | None = None
        self.start_monotonic = time.monotonic()
        self.csv_file = self.output_csv.open("w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(self.csv_file, fieldnames=CSV_FIELDS)
        self.writer.writeheader()
        self._create_subscriptions()
        self.node.create_timer(1.0 / max(0.5, float(args.hz)), self.write_row)

    def _create_subscriptions(self) -> None:
        VehicleLocalPosition = None
        VehicleGlobalPosition = None
        VehicleGpsPosition = None
        SensorGps = None
        try:
            from px4_msgs.msg import VehicleLocalPosition as Px4VehicleLocalPosition

            VehicleLocalPosition = Px4VehicleLocalPosition
        except Exception:
            self.node.get_logger().warning(
                "px4_msgs VehicleLocalPosition is unavailable; PX4 truth subscription is disabled."
            )
        try:
            from px4_msgs.msg import VehicleGlobalPosition as Px4VehicleGlobalPosition

            VehicleGlobalPosition = Px4VehicleGlobalPosition
        except Exception:
            VehicleGlobalPosition = None
        try:
            from px4_msgs.msg import VehicleGpsPosition as Px4VehicleGpsPosition

            VehicleGpsPosition = Px4VehicleGpsPosition
        except Exception:
            VehicleGpsPosition = None
        try:
            from px4_msgs.msg import SensorGps as Px4SensorGps

            SensorGps = Px4SensorGps
        except Exception:
            SensorGps = None

        if VehicleLocalPosition is not None:
            self.node.create_subscription(
                VehicleLocalPosition,
                getattr(self.args, "local_truth_topic", self.args.truth_topic),
                self._local_truth_cb,
                10,
            )
        if VehicleGlobalPosition is not None:
            self.node.create_subscription(
                VehicleGlobalPosition,
                getattr(self.args, "global_truth_topic", "/fmu/out/vehicle_global_position"),
                self._global_truth_cb,
                10,
            )
        gps_type = VehicleGpsPosition or SensorGps
        if gps_type is not None:
            self.node.create_subscription(
                gps_type,
                getattr(self.args, "gps_truth_topic", "/fmu/out/vehicle_gps_position"),
                self._gps_truth_cb,
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
            self.String,
            self.args.triangulation_status_topic,
            self._triangulation_status_cb,
            10,
        )

    def _now_sec(self) -> float:
        return self.node.get_clock().now().nanoseconds * 1.0e-9

    def _publish_selected_truth_fields(self, fields: dict[str, object], source_type: str) -> None:
        if self.args.truth_source == source_type:
            self.latest.update(fields)

    def _local_truth_cb(self, msg: object) -> None:
        fields = fmu_local_position_to_fields(msg)
        self.latest.update(prefixed_fields(fields, "local"))
        self._publish_selected_truth_fields(fields, "local_ned")
        self.last_stamp[getattr(self.args, "local_truth_topic", self.args.truth_topic)] = self._now_sec()

    def _global_truth_cb(self, msg: object) -> None:
        fields, reference = global_position_to_fields(
            msg,
            reference=self.global_truth_lla_reference,
            origin_kind="first_valid_global_sample",
            source_type="global_position",
        )
        if reference is not None:
            self.global_truth_lla_reference = reference
        self.latest.update(prefixed_fields(fields, "global"))
        self._publish_selected_truth_fields(fields, "global_position")
        self.last_stamp[getattr(self.args, "global_truth_topic", "/fmu/out/vehicle_global_position")] = self._now_sec()

    def _gps_truth_cb(self, msg: object) -> None:
        fields, reference = global_position_to_fields(
            msg,
            reference=self.gps_truth_lla_reference,
            origin_kind="first_valid_gps_sample",
            source_type="gps_position",
        )
        if reference is not None:
            self.gps_truth_lla_reference = reference
        self.latest.update(prefixed_fields(fields, "gps"))
        self._publish_selected_truth_fields(fields, "gps_position")
        self.last_stamp[getattr(self.args, "gps_truth_topic", "/fmu/out/vehicle_gps_position")] = self._now_sec()

    def _sim_truth_cb(self, msg: object) -> None:
        values = list(getattr(msg, "data", [])[:4])
        while len(values) < 4:
            values.append(math.nan)
        forward_m, right_m, down_m, _range_m = [float(value) for value in values]
        rdf = frd_to_rdf(forward_m, right_m, down_m, _range_m)
        self.latest.update(
            {
                "sim_truth_role": getattr(self.args, "sim_truth_role", "oracle"),
                "sim_truth_source_frame": "gazebo_baseline_center_frd",
                "sim_truth_axis_contract": "baseline_center_frd:forward,right,down,range",
                "sim_truth_forward_m": rdf["forward_m"],
                "sim_truth_right_m": rdf["right_m"],
                "sim_truth_down_m": rdf["down_m"],
                "sim_truth_range_m": rdf["range_m"],
            }
        )
        if getattr(self.args, "sim_truth_role", "oracle") == "training":
            self.latest.update(
                {
                    "truth_right_m": rdf["right_m"],
                    "truth_down_m": rdf["down_m"],
                    "truth_forward_m": rdf["forward_m"],
                    "truth_range_m": rdf["range_m"],
                }
            )
        self.last_stamp[self.args.sim_truth_topic] = self._now_sec()

    def _estimated_cb(self, msg: object) -> None:
        self.latest.update(
            frd_array_to_fields(
                list(getattr(msg, "data", [])),
                prefix="raw",
            )
        )
        self.last_stamp[self.args.estimated_topic] = self._now_sec()

    def _detections_cb(self, msg: object) -> None:
        self.latest.update(detections_array_to_fields(list(getattr(msg, "data", []))))
        self.last_stamp[self.args.detections_topic] = self._now_sec()

    def _triangulation_status_cb(self, msg: object) -> None:
        try:
            status = json.loads(str(getattr(msg, "data")))
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        self.latest.update(
            {
                "triangulation_valid": bool(status.get("valid", False)),
                "triangulation_reason": str(status.get("reason", "")),
                "ray_closest_gap_m": status.get("closest_gap_m"),
                "ray_left_depth_m": status.get("left_depth_m"),
                "ray_right_depth_m": status.get("right_depth_m"),
                "ray_angle_deg": status.get("ray_angle_deg"),
                "triangulation_point_frame": str(status.get("point_frame", "")),
                "triangulation_frame": str(status.get("frame", "")),
                "triangulation_method": str(status.get("method", "")),
            }
        )
        self.last_stamp[self.args.triangulation_status_topic] = self._now_sec()

    def write_row(self) -> None:
        row: dict[str, str] = {
            "time_sec": f"{time.monotonic() - self.start_monotonic:.6f}",
            "ros_time_sec": f"{self._now_sec():.6f}",
            "source_contract": f"px4_{self.args.truth_source}_truth+stereodistance_frd_estimate",
            "ned_axis_contract": PX4_NED_AXIS_CONTRACT,
            "rdf_axis_contract": CALIBRATION_RDF_AXIS_CONTRACT,
            "public_topic_contract": STEREODISTANCE_TOPIC_CONTRACT,
        }
        for field in CSV_FIELDS:
            row.setdefault(field, csv_value(self.latest.get(field)))
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
