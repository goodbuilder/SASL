import argparse
import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from sasl_common.stereo_geometry import (
    IDENTITY_ROTATION,
    PinholeIntrinsics,
    project_center_point_to_pixel,
)


def detection_values_from_rdf(
    *,
    right_m: float,
    down_m: float,
    forward_m: float,
    width_px: int,
    height_px: int,
    horizontal_fov_rad: float,
    baseline_m: float,
    bbox_size_px: float,
) -> list[float]:
    if forward_m <= 0.0:
        raise ValueError("forward_m must be positive")
    fx = (float(width_px) * 0.5) / math.tan(float(horizontal_fov_rad) * 0.5)
    intrinsics = PinholeIntrinsics(
        fx_px=fx,
        fy_px=fx,
        cx_px=float(width_px) * 0.5,
        cy_px=float(height_px) * 0.5,
    )
    point_center_rdf_m = (float(right_m), float(down_m), float(forward_m))
    half_baseline_m = float(baseline_m) * 0.5
    left_pixel = project_center_point_to_pixel(
        point_center_rdf_m,
        camera_origin_center_rdf_m=(-half_baseline_m, 0.0, 0.0),
        rotation_center_from_camera_rdf=IDENTITY_ROTATION,
        intrinsics=intrinsics,
    )
    right_pixel = project_center_point_to_pixel(
        point_center_rdf_m,
        camera_origin_center_rdf_m=(half_baseline_m, 0.0, 0.0),
        rotation_center_from_camera_rdf=IDENTITY_ROTATION,
        intrinsics=intrinsics,
    )
    if left_pixel is None or right_pixel is None:
        raise ValueError("point is outside the positive camera depth half-space")
    half_bbox = float(bbox_size_px) * 0.5
    left_cx, left_cy = left_pixel
    right_cx, right_cy = right_pixel

    def one_detection(cx: float, cy: float) -> list[float]:
        return [
            1.0,
            float(cx - half_bbox),
            float(cy - half_bbox),
            float(cx + half_bbox),
            float(cy + half_bbox),
            1.0,
            float(cx),
            float(cy),
        ]

    return one_detection(left_cx, left_cy) + one_detection(right_cx, right_cy)


class ExternalBBoxPublisher(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("external_bbox_publisher")
        self.args = args
        self.publisher = self.create_publisher(Float32MultiArray, args.topic, 10)
        self.start_time = time.monotonic()
        self.create_timer(1.0 / max(0.1, float(args.hz)), self._publish)
        self.get_logger().info(
            f"Publishing external stereo BBox to {args.topic} at {args.hz:g} Hz "
            f"(contract=left/right valid,x1,y1,x2,y2,confidence,cx,cy; pixel frame)"
        )

    def _current_forward_m(self) -> float:
        elapsed = time.monotonic() - self.start_time
        duration = max(0.0, float(self.args.ramp_duration_s))
        if duration <= 0.0:
            return float(self.args.forward_m)
        ratio = min(max(elapsed / duration, 0.0), 1.0)
        return float(self.args.forward_m) + (float(self.args.end_forward_m) - float(self.args.forward_m)) * ratio

    def _publish(self) -> None:
        values = detection_values_from_rdf(
            right_m=float(self.args.right_m),
            down_m=float(self.args.down_m),
            forward_m=self._current_forward_m(),
            width_px=int(self.args.width_px),
            height_px=int(self.args.height_px),
            horizontal_fov_rad=float(self.args.horizontal_fov_rad),
            baseline_m=float(self.args.baseline_m),
            bbox_size_px=float(self.args.bbox_size_px),
        )
        self.publisher.publish(Float32MultiArray(data=values))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Publish a synthetic external stereo BBox Float32MultiArray for operator-view integration tests. "
            "The generated BBox is an input substitute for a real YOLO/BBox publisher; it does not publish truth."
        )
    )
    parser.add_argument("--topic", default="/stereo/external_detections")
    parser.add_argument("--hz", type=float, default=20.0)
    parser.add_argument("--width-px", type=int, default=1280)
    parser.add_argument("--height-px", type=int, default=720)
    parser.add_argument("--horizontal-fov-rad", type=float, default=0.154997710698)
    parser.add_argument("--baseline-m", type=float, default=25.0)
    parser.add_argument("--bbox-size-px", type=float, default=32.0)
    parser.add_argument("--right-m", type=float, default=0.0)
    parser.add_argument("--down-m", type=float, default=0.0)
    parser.add_argument("--forward-m", type=float, default=1000.0)
    parser.add_argument("--end-forward-m", type=float, default=1000.0)
    parser.add_argument("--ramp-duration-s", type=float, default=0.0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = ExternalBBoxPublisher(args)
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
