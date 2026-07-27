import math
import threading
import time

import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image as ImageMsg
from std_msgs.msg import Float32MultiArray


class DetectionOverlayNode(Node):
    def __init__(self):
        super().__init__("detection_overlay_node")
        self.declare_parameter(
            "overlay_publish_hz",
            10.0,
            ParameterDescriptor(dynamic_typing=True),
        )
        self.overlay_publish_hz = max(0.0, float(self.get_parameter("overlay_publish_hz").value))

        self.lock = threading.Lock()
        self.shutdown_event = threading.Event()
        self.left_msg = None
        self.right_msg = None
        self.left_detection = None
        self.right_detection = None
        self.detection_revision = 0
        self.last_left_publish_key = None
        self.last_right_publish_key = None

        camera_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(ImageMsg, "/left_camera/image", self._left_cb, camera_qos)
        self.create_subscription(ImageMsg, "/right_camera/image", self._right_cb, camera_qos)
        self.create_subscription(Float32MultiArray, "/stereo/live_detections", self._detections_cb, 10)

        self.left_pub = self.create_publisher(
            ImageMsg, "/left_camera/detection_image", qos_profile_sensor_data
        )
        self.right_pub = self.create_publisher(
            ImageMsg, "/right_camera/detection_image", qos_profile_sensor_data
        )

        self.add_on_set_parameters_callback(self._on_param_update)
        self.overlay_thread = threading.Thread(target=self._overlay_loop, daemon=True)
        self.overlay_thread.start()
        self.get_logger().info(
            f"Detection overlay node started (overlay_publish_hz={self.overlay_publish_hz:.1f})."
        )

    def _on_param_update(self, params):
        for param in params:
            if param.name == "overlay_publish_hz":
                self.overlay_publish_hz = max(0.0, float(param.value))
        return SetParametersResult(successful=True)

    def _left_cb(self, msg):
        with self.lock:
            self.left_msg = msg

    def _right_cb(self, msg):
        with self.lock:
            self.right_msg = msg

    def _detections_cb(self, msg):
        data = list(msg.data)
        if len(data) < 16:
            return
        with self.lock:
            self.left_detection = self._values_to_detection(data[:8])
            self.right_detection = self._values_to_detection(data[8:16])
            self.detection_revision += 1

    def _values_to_detection(self, values):
        if values[0] < 0.5:
            return None
        x1, y1, x2, y2 = values[1:5]
        if not all(math.isfinite(value) for value in (x1, y1, x2, y2)):
            return None
        return {
            "bbox": (x1, y1, x2, y2),
            "confidence": values[5],
            "cx": values[6],
            "cy": values[7],
        }

    def _overlay_loop(self):
        next_tick = time.monotonic()
        while not self.shutdown_event.is_set():
            if self.overlay_publish_hz <= 0.0:
                self.shutdown_event.wait(0.5)
                continue
            self._publish_overlays()
            next_tick += 1.0 / max(0.1, float(self.overlay_publish_hz))
            sleep_s = next_tick - time.monotonic()
            if sleep_s > 0.0:
                self.shutdown_event.wait(sleep_s)
            else:
                next_tick = time.monotonic()

    def _publish_overlays(self):
        with self.lock:
            left_msg = self.left_msg
            right_msg = self.right_msg
            left_detection = None if self.left_detection is None else dict(self.left_detection)
            right_detection = None if self.right_detection is None else dict(self.right_detection)
            revision = self.detection_revision

        if left_msg is not None and self._should_publish("left", left_msg, revision):
            self._publish_overlay(self.left_pub, left_msg, left_detection)
        if right_msg is not None and self._should_publish("right", right_msg, revision):
            self._publish_overlay(self.right_pub, right_msg, right_detection)

    def _should_publish(self, side, msg, detection_revision):
        key = (
            int(msg.header.stamp.sec),
            int(msg.header.stamp.nanosec),
            int(detection_revision),
        )
        if side == "left":
            if key == self.last_left_publish_key:
                return False
            self.last_left_publish_key = key
            return True
        if key == self.last_right_publish_key:
            return False
        self.last_right_publish_key = key
        return True

    def _publish_overlay(self, publisher, msg, detection):
        if msg.encoding.lower() == "rgb8":
            out = self._copy_image_msg(msg)
            image = self._msg_to_rgb_view(out)
            if image is None:
                return
            self._draw_detection_in_place(image, detection)
            publisher.publish(out)
            return

        image = self._msg_to_rgb_copy(msg)
        if image is not None:
            self._draw_detection_in_place(image, detection)
            publisher.publish(self._image_to_msg(image, msg))

    def _copy_image_msg(self, source_msg):
        msg = ImageMsg()
        msg.header = source_msg.header
        msg.height = source_msg.height
        msg.width = source_msg.width
        msg.encoding = source_msg.encoding
        msg.is_bigendian = source_msg.is_bigendian
        msg.step = source_msg.step
        msg.data = bytearray(source_msg.data)
        return msg

    def _msg_to_rgb_view(self, msg):
        try:
            raw = np.frombuffer(msg.data, dtype=np.uint8)
            rows = raw.reshape((msg.height, msg.step))
            return rows[:, : msg.width * 3].reshape((msg.height, msg.width, 3))
        except ValueError as exc:
            self.get_logger().warning(f"Could not map image buffer: {exc}")
            return None

    def _msg_to_rgb_copy(self, msg):
        encoding = msg.encoding.lower()
        channels_by_encoding = {
            "rgb8": 3,
            "bgr8": 3,
            "rgba8": 4,
            "bgra8": 4,
            "mono8": 1,
        }
        channels = channels_by_encoding.get(encoding)
        if channels is None:
            self.get_logger().warning(f"Unsupported image encoding: {msg.encoding}")
            return None

        raw = np.frombuffer(msg.data, dtype=np.uint8)
        rows = raw.reshape((msg.height, msg.step))
        pixels = rows[:, : msg.width * channels].reshape((msg.height, msg.width, channels))

        if encoding == "rgb8":
            return pixels.copy()
        if encoding == "bgr8":
            return pixels[:, :, ::-1].copy()
        if encoding == "rgba8":
            return pixels[:, :, :3].copy()
        if encoding == "bgra8":
            return pixels[:, :, 2::-1].copy()
        return np.repeat(pixels, 3, axis=2).copy()

    def _image_to_msg(self, image, source_msg):
        msg = ImageMsg()
        msg.header = source_msg.header
        msg.height = image.shape[0]
        msg.width = image.shape[1]
        msg.encoding = "rgb8"
        msg.is_bigendian = 0
        msg.step = msg.width * 3
        msg.data = image.astype(np.uint8, copy=False).tobytes()
        return msg

    def _draw_detection_in_place(
        self,
        image,
        detection,
        found_color=(0, 255, 0),
        missing_color=(255, 0, 0),
    ):
        height, width = image.shape[:2]
        if detection is None or "bbox" not in detection:
            color = np.array(missing_color, dtype=np.uint8)
            image[0:6, :, :] = color
            image[-6:, :, :] = color
            image[:, 0:6, :] = color
            image[:, -6:, :] = color
            return

        x1, y1, x2, y2 = detection["bbox"]
        x1 = max(0, min(width - 1, int(round(x1))))
        x2 = max(0, min(width - 1, int(round(x2))))
        y1 = max(0, min(height - 1, int(round(y1))))
        y2 = max(0, min(height - 1, int(round(y2))))
        if x2 < x1:
            x1, x2 = x2, x1
        if y2 < y1:
            y1, y2 = y2, y1

        color = np.array(found_color, dtype=np.uint8)
        thickness = 4
        image[y1 : min(y1 + thickness, height), x1 : x2 + 1, :] = color
        image[max(y2 - thickness + 1, 0) : y2 + 1, x1 : x2 + 1, :] = color
        image[y1 : y2 + 1, x1 : min(x1 + thickness, width), :] = color
        image[y1 : y2 + 1, max(x2 - thickness + 1, 0) : x2 + 1, :] = color


def main(args=None):
    rclpy.init(args=args)
    node = DetectionOverlayNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.shutdown_event.set()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
