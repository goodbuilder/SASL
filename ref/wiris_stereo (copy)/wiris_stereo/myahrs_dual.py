#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

import serial
import threading
import math
import time
import csv
from datetime import datetime


# ====== 센서 포트 설정 ======
sensor_ports = {
    "imu1_frame": "/dev/ttyACM0",
    "imu2_frame": "/dev/ttyACM1",
}

# ====== 공유 데이터(스레드에서 갱신, 타이머에서 읽음) ======
sensor_data = {
    "imu1_frame": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
    "imu2_frame": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
}

sensor_lock = threading.Lock()


def euler_to_quaternion(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    return {
        "x": sr * cp * cy - cr * sp * sy,
        "y": cr * sp * cy + sr * cp * sy,
        "z": cr * cp * sy - sr * sp * cy,
        "w": cr * cp * cy + sr * sp * sy,
    }


def read_rpy(port_name: str, frame_id: str, logger=None):
    """
    시리얼로부터 $RPY 라인을 읽어 sensor_data[frame_id]를 갱신.
    logger가 있으면 ROS 로그(/rosout)로 출력 -> rqt_console에 뜸.
    """
    ser = None
    try:
        ser = serial.Serial(port_name, 115200, timeout=1)

        ser.write(b"@mode,AC\r\n")
        ser.write(b"@asc_out,RPY\r\n")
        ser.write(b"@divider,1\r\n")

        if logger:
            logger.info(f"[{frame_id}] Serial opened: {port_name}")

        while True:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if not line:
                continue

            if line.startswith("$RPY"):
                parts = line.split(",")
                if len(parts) >= 5:
                    try:
                        roll = math.radians(float(parts[2]))
                        pitch = math.radians(float(parts[3]))
                        yaw = math.radians(float(parts[4].split("*")[0]))

                        with sensor_lock:
                            sensor_data[frame_id] = {"roll": roll, "pitch": pitch, "yaw": yaw}

                    except ValueError as ve:
                        if logger:
                            logger.warning(f"[{frame_id}] parse error: {ve} line={line}")
                        continue

    except Exception as e:
        if logger:
            logger.error(f"[{frame_id}] Error: {e}")
        else:
            print(f"[{frame_id}] Error: {e}")
    finally:
        try:
            if ser and ser.is_open:
                ser.close()
        except Exception:
            pass


class UnifiedPublisher(Node):
    def __init__(self):
        super().__init__("unified_tf_and_rpy")
        self.br = TransformBroadcaster(self)

        # ==== Node start time (for file naming) ====
        self.start_time_str = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')

        # 20 Hz publish
        self.timer = self.create_timer(0.05, self.publish_all)

        # rqt_console 스팸 방지: 1초에 1번만 info 로그 출력
        self._last_log_time = 0.0
        self._log_period_sec = 1.0

        # ===== CSV Logging =====
        csv_path = (
            f'/data/rosbags/data/'
            f'ahrs_{self.start_time_str}.csv'
        )

        self.get_logger().info(f"CSV logging to: {csv_path}")

        self.csv_file = open(csv_path, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)

        self.csv_writer.writerow(
            [
                "stamp_real",
                "imu1_roll_deg",
                "imu1_pitch_deg",
                "imu1_yaw_deg",
                "imu2_roll_deg",
                "imu2_pitch_deg",
                "imu2_yaw_deg",
            ]
        )
        self.csv_file.flush()

    def publish_all(self):
        stamp_real = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")

        deg_values = {}
        print_parts = []

        with sensor_lock:
            local_items = list(sensor_data.items())

        for i, (frame_id, rpy) in enumerate(local_items, start=1):
            q = euler_to_quaternion(rpy["roll"], rpy["pitch"], rpy["yaw"])

            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = "world"
            t.child_frame_id = frame_id

            t.transform.translation.x = 0.0 if frame_id == "imu1_frame" else 1.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0

            t.transform.rotation.x = q["x"]
            t.transform.rotation.y = q["y"]
            t.transform.rotation.z = q["z"]
            t.transform.rotation.w = q["w"]

            self.br.sendTransform(t)

            roll_deg = math.degrees(rpy["roll"])
            pitch_deg = math.degrees(rpy["pitch"])
            yaw_deg = math.degrees(rpy["yaw"])
            deg_values[frame_id] = (roll_deg, pitch_deg, yaw_deg)

            print_parts.append(f"Cam{i} R:{roll_deg:6.2f} P:{pitch_deg:6.2f} Y:{yaw_deg:6.2f}")

        line = "  |  ".join(print_parts)
        print(line)

        now = time.time()
        if now - self._last_log_time >= self._log_period_sec:
            self.get_logger().info(line)
            self._last_log_time = now

        imu1_roll, imu1_pitch, imu1_yaw = deg_values.get("imu1_frame", (0.0, 0.0, 0.0))
        imu2_roll, imu2_pitch, imu2_yaw = deg_values.get("imu2_frame", (0.0, 0.0, 0.0))

        self.csv_writer.writerow(
            [stamp_real, imu1_roll, imu1_pitch, imu1_yaw, imu2_roll, imu2_pitch, imu2_yaw]
        )
        self.csv_file.flush()

    def destroy_node(self):
        try:
            if hasattr(self, "csv_file") and self.csv_file:
                self.csv_file.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = UnifiedPublisher()

    for frame_id, port in sensor_ports.items():
        threading.Thread(
            target=read_rpy,
            args=(port, frame_id, node.get_logger()),
            daemon=True,
        ).start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
