#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped

from cv_bridge import CvBridge
from ultralytics import YOLO
import torch
import cv2
import csv
from datetime import datetime
import os


class WirisYoloNode(Node):
    def __init__(self):
        super().__init__('wiris_yolo_node')

        # ==== Node start time (for file naming) ====
        self.start_time_str = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')

        # ==== Stereo / YOLO params ====
        self.declare_parameter('use_stereo', True)
        self.declare_parameter('model_path', '/home/saslab/ros2_ws/src/wiris_stereo/wiris_stereo/yolov8n_uav.pt')
        self.declare_parameter('conf_thres', 0.4)
        self.declare_parameter('device', 'auto')
        self.declare_parameter('uav_class_id', 0)

        # ==== RTSP(Ethernet) input params ====
        self.declare_parameter('left_ip', '10.0.0.231')
        self.declare_parameter('right_ip', '10.0.0.232')
        self.declare_parameter('rtsp_port', 8554)
        self.declare_parameter('left_path', '/visible')
        self.declare_parameter('right_path', '/visible')

        # RTSP/GStreamer tuning
        self.declare_parameter('rtsp.transport', 'tcp')     # 'tcp' recommended
        self.declare_parameter('rtsp.latency_ms', 100)
        self.declare_parameter('rtsp.drop_on_latency', False)
        self.declare_parameter('decoder', 'avdec')          # 'avdec' recommended

        # Force FHD output (receiver-side)
        self.declare_parameter('force_width', 1920)
        self.declare_parameter('force_height', 1080)

        # ==== CSV Logging ====
        self.declare_parameter('csv_dir', '/data/rosbags/data/')

        # ==== Debug / Logging ====
        # N 프레임마다 원본/최종 해상도 로그 출력
        self.declare_parameter('log_res_every_n_frames', 60)
        # N 프레임마다 센터/박스 로그 출력
        self.declare_parameter('log_center_every_n_frames', 30)

        # (추가) N 프레임마다 "탐지된 모든 박스"를 출력할지
        self.declare_parameter('log_all_dets_every_n_frames', 30)

        # ==== Read parameters ====
        self.use_stereo = bool(self.get_parameter('use_stereo').value)
        self.model_path = self.get_parameter('model_path').value
        self.conf_thres = float(self.get_parameter('conf_thres').value)
        self.uav_class_id = int(self.get_parameter('uav_class_id').value)

        self.left_ip = self.get_parameter('left_ip').value
        self.right_ip = self.get_parameter('right_ip').value
        self.rtsp_port = int(self.get_parameter('rtsp_port').value)
        self.left_path = self.get_parameter('left_path').value
        self.right_path = self.get_parameter('right_path').value

        self.rtsp_transport = self.get_parameter('rtsp.transport').value
        self.rtsp_latency = int(self.get_parameter('rtsp.latency_ms').value)
        self.rtsp_drop_on_latency = bool(self.get_parameter('rtsp.drop_on_latency').value)
        self.decoder = self.get_parameter('decoder').value

        self.force_w = int(self.get_parameter('force_width').value)
        self.force_h = int(self.get_parameter('force_height').value)

        self.csv_dir = self.get_parameter('csv_dir').value
        os.makedirs(self.csv_dir, exist_ok=True)

        self.log_res_every_n = int(self.get_parameter('log_res_every_n_frames').value)
        if self.log_res_every_n <= 0:
            self.log_res_every_n = 60

        self.log_center_every_n = int(self.get_parameter('log_center_every_n_frames').value)
        if self.log_center_every_n <= 0:
            self.log_center_every_n = 30

        self.log_all_dets_every_n = int(self.get_parameter('log_all_dets_every_n_frames').value)
        if self.log_all_dets_every_n <= 0:
            self.log_all_dets_every_n = self.log_center_every_n

        # frame counters for throttled logs
        self._frame_count_left = 0
        self._frame_count_right = 0
        self._frame_tick = 0

        # ==== Device ====
        device_param = self.get_parameter('device').value
        if device_param == 'auto':
            self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        else:
            self.device = device_param
        self.get_logger().info(f"YOLO device = {self.device}")

        self.bridge = CvBridge()

        # =========================================================
        # Cameras (RTSP via GStreamer)
        # =========================================================
        gst_left = self._build_rtsp_gst(
            ip=self.left_ip,
            port=self.rtsp_port,
            path=self.left_path,
            transport=self.rtsp_transport,
            latency=self.rtsp_latency,
            drop_on_latency=self.rtsp_drop_on_latency,
            decoder=self.decoder,
            width=self.force_w,
            height=self.force_h,
        )
        self.get_logger().info(f"[LEFT] GStreamer: {gst_left}")
        self.cap_left = cv2.VideoCapture(gst_left, cv2.CAP_GSTREAMER)

        if not self.cap_left.isOpened():
            self.get_logger().error(f"Left RTSP open failed: rtsp://{self.left_ip}:{self.rtsp_port}{self.left_path}")
        else:
            w = int(self.cap_left.get(cv2.CAP_PROP_FRAME_WIDTH))
            h = int(self.cap_left.get(cv2.CAP_PROP_FRAME_HEIGHT))
            self.get_logger().info(f"[LEFT] CAP_PROP = {w} x {h}")

            ok, frame = self.cap_left.read()
            if ok and frame is not None:
                hh, ww = frame.shape[:2]
                self.get_logger().info(f"[LEFT] first frame = {ww} x {hh}")
            else:
                self.get_logger().warn("[LEFT] opened but first read failed")

        self.cap_right = None
        if self.use_stereo:
            gst_right = self._build_rtsp_gst(
                ip=self.right_ip,
                port=self.rtsp_port,
                path=self.right_path,
                transport=self.rtsp_transport,
                latency=self.rtsp_latency,
                drop_on_latency=self.rtsp_drop_on_latency,
                decoder=self.decoder,
                width=self.force_w,
                height=self.force_h,
            )
            self.get_logger().info(f"[RIGHT] GStreamer: {gst_right}")
            self.cap_right = cv2.VideoCapture(gst_right, cv2.CAP_GSTREAMER)

            if not self.cap_right.isOpened():
                self.get_logger().error(f"Right RTSP open failed: rtsp://{self.right_ip}:{self.rtsp_port}{self.right_path}")
                self.use_stereo = False
            else:
                w = int(self.cap_right.get(cv2.CAP_PROP_FRAME_WIDTH))
                h = int(self.cap_right.get(cv2.CAP_PROP_FRAME_HEIGHT))
                self.get_logger().info(f"[RIGHT] CAP_PROP = {w} x {h}")

                ok, frame = self.cap_right.read()
                if ok and frame is not None:
                    hh, ww = frame.shape[:2]
                    self.get_logger().info(f"[RIGHT] first frame = {ww} x {hh}")
                else:
                    self.get_logger().warn("[RIGHT] opened but first read failed")

        # ==== YOLO MODEL ====
        self.get_logger().info(f"Loading YOLO model: {self.model_path}")
        self.model = YOLO(self.model_path)

        # ==== Publishers ====
        self.left_raw_pub = self.create_publisher(Image, '/wiris/left/image_raw', 10)
        self.right_raw_pub = self.create_publisher(Image, '/wiris/right/image_raw', 10)

        self.left_img_pub = self.create_publisher(Image, '/wiris/left/image', 10)
        self.right_img_pub = self.create_publisher(Image, '/wiris/right/image', 10)

        self.left_center_pub = self.create_publisher(PointStamped, '/wiris/left/center', 10)
        self.right_center_pub = self.create_publisher(PointStamped, '/wiris/right/center', 10)

        # ==== CSV Logging ====
        csv_path = f'{self.csv_dir}/wiris_centers_{self.start_time_str}.csv'
        self.get_logger().info(f"CSV logging to: {csv_path}")

        self.csv_file = open(csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        # stamp_real 기준으로 LEFT/RIGHT를 각각 한 줄씩 기록
        self.csv_writer.writerow([
            'stamp_real',
            'camera',        # LEFT / RIGHT
            'center_x',
            'center_y',
            'box_w',
            'box_h',
            'conf'
        ])
        self.csv_file.flush()

        # 30Hz timer
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)
        self.get_logger().info("WirisYoloNode started.")

    def _build_rtsp_gst(self, ip, port, path, transport, latency, drop_on_latency, decoder, width, height) -> str:
        base = (
            f"rtspsrc location=rtsp://{ip}:{port}{path} "
            f"protocols={'tcp' if str(transport).lower()=='tcp' else 'udp'} "
            f"latency={int(latency)} "
        )
        if drop_on_latency:
            base += "drop-on-latency=true "

        base += "! rtph264depay ! h264parse "

        if str(decoder).lower() == 'avdec':
            dec = "! avdec_h264 "
        else:
            dec = "! decodebin "

        pipe = (
            base
            + dec
            + "! videoconvert ! videoscale "
            + f"! video/x-raw,format=BGR,width={int(width)},height={int(height)} "
            + f"! appsink caps=video/x-raw,format=BGR,width={int(width)},height={int(height)} "
            + "max-buffers=1 drop=true sync=false"
        )
        return pipe

    def _force_fhd(self, frame):
        if frame is None:
            return None
        h, w = frame.shape[:2]
        if (w != self.force_w) or (h != self.force_h):
            frame = cv2.resize(frame, (self.force_w, self.force_h), interpolation=cv2.INTER_LINEAR)
        return frame

    def _log_resolution_throttled(self, cam_name: str, before_hw, after_hw, counter: int):
        if counter % self.log_res_every_n == 0:
            (hb, wb) = before_hw
            (ha, wa) = after_hw
            self.get_logger().info(
                f"[{cam_name}] RES before={wb}x{hb}  after={wa}x{ha}  (force={self.force_w}x{self.force_h})"
            )

    def _log_center_throttled(self, cam_name: str, cx, cy, bw, bh, conf):
        if self._frame_tick % self.log_center_every_n != 0:
            return
        # "no detection" 출력은 제거 (요청사항)
        if cx is None or cy is None:
            return
        self.get_logger().info(f"[{cam_name}] center=({cx},{cy}) box size=({bw},{bh}) conf={conf:.2f}")

    def _write_csv_row(self, stamp_real: str, cam_name: str, cx, cy, bw, bh, conf):
        if cx is None or cy is None:
            self.csv_writer.writerow([stamp_real, cam_name, '', '', '', '', ''])
        else:
            self.csv_writer.writerow([stamp_real, cam_name, cx, cy, bw, bh, f"{conf:.3f}"])

    def timer_callback(self):
        self._frame_tick += 1

        left_ready = self.cap_left is not None and self.cap_left.isOpened()
        right_ready = self.use_stereo and self.cap_right is not None and self.cap_right.isOpened()

        left_grabbed = left_ready and self.cap_left.grab()
        right_grabbed = right_ready and self.cap_right.grab()

        now = self.get_clock().now().to_msg()
        stamp_real = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')

        # ===== Left =====
        if left_grabbed:
            ret_l, frame_l = self.cap_left.retrieve()
            if ret_l and frame_l is not None:
                self._frame_count_left += 1
                before_h, before_w = frame_l.shape[:2]

                frame_l = self._force_fhd(frame_l)
                after_h, after_w = frame_l.shape[:2]
                self._log_resolution_throttled("LEFT", (before_h, before_w), (after_h, after_w), self._frame_count_left)

                raw_l = frame_l.copy()

                annotated_l, cx_l, cy_l, w_l, h_l, conf_l = self.run_yolo_and_draw(frame_l, cam_name="LEFT")

                # 대표 1개(선택된 UAV)의 center 로그
                self._log_center_throttled("LEFT", cx_l, cy_l, w_l, h_l, conf_l)

                # publish raw
                raw_msg_l = self.bridge.cv2_to_imgmsg(raw_l, encoding='bgr8')
                raw_msg_l.header.stamp = now
                raw_msg_l.header.frame_id = "left_camera"
                self.left_raw_pub.publish(raw_msg_l)

                # publish annotated
                img_msg_l = self.bridge.cv2_to_imgmsg(annotated_l, encoding='bgr8')
                img_msg_l.header.stamp = now
                img_msg_l.header.frame_id = "left_camera"
                self.left_img_pub.publish(img_msg_l)

                # publish center
                if cx_l is not None and cy_l is not None:
                    pt_msg_l = PointStamped()
                    pt_msg_l.header = img_msg_l.header
                    pt_msg_l.point.x = float(cx_l)
                    pt_msg_l.point.y = float(cy_l)
                    pt_msg_l.point.z = 0.0
                    self.left_center_pub.publish(pt_msg_l)

                # write CSV (one row per camera per timestamp)
                self._write_csv_row(stamp_real, "LEFT", cx_l, cy_l, w_l, h_l, conf_l)

        # ===== Right =====
        if right_grabbed:
            ret_r, frame_r = self.cap_right.retrieve()
            if ret_r and frame_r is not None:
                self._frame_count_right += 1
                before_h, before_w = frame_r.shape[:2]

                frame_r = self._force_fhd(frame_r)
                after_h, after_w = frame_r.shape[:2]
                self._log_resolution_throttled("RIGHT", (before_h, before_w), (after_h, after_w), self._frame_count_right)

                raw_r = frame_r.copy()

                annotated_r, cx_r, cy_r, w_r, h_r, conf_r = self.run_yolo_and_draw(frame_r, cam_name="RIGHT")

                # 대표 1개(선택된 UAV)의 center 로그
                self._log_center_throttled("RIGHT", cx_r, cy_r, w_r, h_r, conf_r)

                # publish raw
                raw_msg_r = self.bridge.cv2_to_imgmsg(raw_r, encoding='bgr8')
                raw_msg_r.header.stamp = now
                raw_msg_r.header.frame_id = "right_camera"
                self.right_raw_pub.publish(raw_msg_r)

                # publish annotated
                img_msg_r = self.bridge.cv2_to_imgmsg(annotated_r, encoding='bgr8')
                img_msg_r.header.stamp = now
                img_msg_r.header.frame_id = "right_camera"
                self.right_img_pub.publish(img_msg_r)

                # publish center
                if cx_r is not None and cy_r is not None:
                    pt_msg_r = PointStamped()
                    pt_msg_r.header = img_msg_r.header
                    pt_msg_r.point.x = float(cx_r)
                    pt_msg_r.point.y = float(cy_r)
                    pt_msg_r.point.z = 0.0
                    self.right_center_pub.publish(pt_msg_r)

                # write CSV (one row per camera per timestamp)
                self._write_csv_row(stamp_real, "RIGHT", cx_r, cy_r, w_r, h_r, conf_r)

        # flush once per timer tick
        self.csv_file.flush()

    def run_yolo_and_draw(self, frame, cam_name: str = ""):
        """
        변경점:
        - "최적 1개(best_box)"만 선택/표시하는 건 그대로 유지
        - 대신, 탐지된 모든 박스를 (throttle 적용해서) 전부 프린트
        """
        try:
            results = self.model(frame, device=self.device, imgsz=640, verbose=False)[0]
        except Exception as e:
            self.get_logger().error(f"[{cam_name}] YOLO inference failed: {e}")
            return frame, None, None, None, None, None

        boxes = getattr(results, "boxes", None)
        if boxes is None or len(boxes) == 0:
            return frame, None, None, None, None, None

        # ==========================
        # (A) 탐지된 값 "전부" 프린트 (N프레임마다)
        # ==========================
        if self._frame_tick % self.log_all_dets_every_n == 0:
            self.get_logger().info(f"[{cam_name}] detections={len(boxes)}")
            for i, (xyxy, cls_id, conf) in enumerate(zip(boxes.xyxy, boxes.cls, boxes.conf)):
                x1, y1, x2, y2 = [int(v.item()) for v in xyxy]
                cls_i = int(cls_id.item())
                conf_f = float(conf.item())
                w = max(0, x2 - x1)
                h = max(0, y2 - y1)
                cx = (x1 + x2) // 2
                cy = (y1 + y2) // 2
                self.get_logger().info(
                    f"[{cam_name}] #{i} cls={cls_i} conf={conf_f:.3f} "
                    f"xyxy=({x1},{y1},{x2},{y2}) center=({cx},{cy}) size=({w},{h})"
                )

        # ==========================
        # (B) 기존처럼 UAV 클래스 + conf_thres 통과한 것 중 "대표 1개" 선택
        # ==========================
        best_area = 0.0
        best_box = None
        best_conf = 0.0

        for xyxy, cls_id, conf in zip(boxes.xyxy, boxes.cls, boxes.conf):
            if int(cls_id.item()) != int(self.uav_class_id):
                continue

            conf_val = float(conf.item())
            if conf_val < float(self.conf_thres):
                continue

            x1, y1, x2, y2 = [int(v.item()) for v in xyxy]
            area = max(0, x2 - x1) * max(0, y2 - y1)

            if area > best_area:
                best_area = area
                best_box = (x1, y1, x2, y2)
                best_conf = conf_val

        if best_box is None:
            return frame, None, None, None, None, None

        x1, y1, x2, y2 = best_box
        cx = (x1 + x2) // 2
        cy = (y1 + y2) // 2
        width = x2 - x1
        height = y2 - y1

        label = f"UAV {best_conf:.2f}"
        cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
        cv2.putText(frame, label, (x1, max(y1 - 10, 0)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
        cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

        return frame, cx, cy, width, height, best_conf


def main(args=None):
    rclpy.init(args=args)
    node = WirisYoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.get_logger().info("Shutting down WirisYoloNode...")

    if node.cap_left is not None:
        node.cap_left.release()
    if node.cap_right is not None:
        node.cap_right.release()

    if hasattr(node, 'csv_file') and node.csv_file is not None:
        node.csv_file.close()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
