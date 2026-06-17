#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
import socket

# Workswell EO optical zoom table (index ↔ 배율)
# 0→1x, 8→8x, 11→16x, 14→30x 등
ZOOM_TABLE = [1.0, 1.2, 1.5, 2.0, 3.0, 4.0, 5.0, 6.0,
              8.0, 10.0, 12.0, 16.0, 20.0, 25.0, 30.0]
IDX_MIN, IDX_MAX = 0, len(ZOOM_TABLE) - 1


class WirisDualZoomNode(Node):
    def __init__(self):
        super().__init__('zoom_node')

        # ===== Parameters =====
        # 왼/오 카메라 IP.
        self.declare_parameter('left_ip',  '10.0.0.231')
        self.declare_parameter('right_ip', '10.0.0.232')
        self.declare_parameter('wiris_port', 2240)

        self.left_ip  = self.get_parameter('left_ip').get_parameter_value().string_value
        self.right_ip = self.get_parameter('right_ip').get_parameter_value().string_value
        self.wiris_port = self.get_parameter('wiris_port').get_parameter_value().integer_value

        # ===== Sockets (각 카메라별) =====
        self.sock_left = None
        self.sock_right = None
        self.connect_left()
        self.connect_right()

        # ===== Subscribers =====
        # 왼쪽: /wiris_left/zoom_cmd
        self.sub_left = self.create_subscription(
            String,
            '/wiris_left/zoom_cmd',
            self.left_cmd_cb,
            10
        )
        # 오른쪽: /wiris_right/zoom_cmd
        self.sub_right = self.create_subscription(
            String,
            '/wiris_right/zoom_cmd',
            self.right_cmd_cb,
            10
        )

        # ===== Zoom index Publishers =====
        # -> Stereo depth 노드에서 /szvn_left, /szvn_right 구독해서
        #    fx, fy 업데이트할 수 있도록
        self.pub_szvn_left = self.create_publisher(Int32, '/szvn_left', 10)
        self.pub_szvn_right = self.create_publisher(Int32, '/szvn_right', 10)

        self.get_logger().info(
            f"WirisDualZoomNode started | LEFT={self.left_ip}, RIGHT={self.right_ip}, PORT={self.wiris_port}"
        )

    # -------- Socket helpers --------
    def connect_left(self):
        if self.sock_left is not None:
            try:
                self.sock_left.close()
            except Exception:
                pass
        try:
            self.sock_left = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock_left.settimeout(3.0)
            self.sock_left.connect((self.left_ip, self.wiris_port))
            self.get_logger().info(f"[LEFT] Connected to WIRIS at {self.left_ip}:{self.wiris_port}")
        except Exception as e:
            self.get_logger().error(f"[LEFT] Failed to connect: {e}")
            self.sock_left = None

    def connect_right(self):
        if self.sock_right is not None:
            try:
                self.sock_right.close()
            except Exception:
                pass
        try:
            self.sock_right = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock_right.settimeout(3.0)
            self.sock_right.connect((self.right_ip, self.wiris_port))
            self.get_logger().info(f"[RIGHT] Connected to WIRIS at {self.right_ip}:{self.wiris_port}")
        except Exception as e:
            self.get_logger().error(f"[RIGHT] Failed to connect: {e}")
            self.sock_right = None

    def send_cmd_left(self, cmd: str, expect_reply: bool = False):
        if self.sock_left is None:
            self.get_logger().warn("[LEFT] Socket not connected. Reconnecting...")
            self.connect_left()
            if self.sock_left is None:
                self.get_logger().error("[LEFT] Reconnect failed.")
                return None

        msg = cmd.strip() + '\n'
        try:
            self.sock_left.sendall(msg.encode('ascii'))
            self.get_logger().info(f"[LEFT] Sent: {cmd}")
            if expect_reply:
                data = self.sock_left.recv(128)
                resp = data.decode('ascii', errors='ignore').strip()
                self.get_logger().info(f"[LEFT] Reply: {resp}")
                return resp
        except Exception as e:
            self.get_logger().error(f"[LEFT] Send failed: {e}")
            self.sock_left.close()
            self.sock_left = None
        return None

    def send_cmd_right(self, cmd: str, expect_reply: bool = False):
        if self.sock_right is None:
            self.get_logger().warn("[RIGHT] Socket not connected. Reconnecting...")
            self.connect_right()
            if self.sock_right is None:
                self.get_logger().error("[RIGHT] Reconnect failed.")
                return None

        msg = cmd.strip() + '\n'
        try:
            self.sock_right.sendall(msg.encode('ascii'))
            self.get_logger().info(f"[RIGHT] Sent: {cmd}")
            if expect_reply:
                data = self.sock_right.recv(128)
                resp = data.decode('ascii', errors='ignore').strip()
                self.get_logger().info(f"[RIGHT] Reply: {resp}")
                return resp
        except Exception as e:
            self.get_logger().error(f"[RIGHT] Send failed: {e}")
            self.sock_right.close()
            self.sock_right = None
        return None

    # -------- Zoom index helpers --------
    def publish_szvn_left(self, idx: int):
        msg = Int32()
        msg.data = idx
        self.pub_szvn_left.publish(msg)
        self.get_logger().info(f"[LEFT] Publish /szvn_left = {idx}")

    def publish_szvn_right(self, idx: int):
        msg = Int32()
        msg.data = idx
        self.pub_szvn_right.publish(msg)
        self.get_logger().info(f"[RIGHT] Publish /szvn_right = {idx}")

    def parse_gzvv_index(self, resp: str):
        # GZVV 응답 형식: "<idx> <ratio>" 예: "11 16.000000"
        try:
            parts = resp.split()
            idx = int(parts[0])
            return idx
        except Exception:
            return None

    def query_visible_index_left(self):
        resp = self.send_cmd_left("GZVV", expect_reply=True)
        if not resp:
            return None
        return self.parse_gzvv_index(resp)

    def query_visible_index_right(self):
        resp = self.send_cmd_right("GZVV", expect_reply=True)
        if not resp:
            return None
        return self.parse_gzvv_index(resp)

    # -------- Command callbacks --------
    def left_cmd_cb(self, msg: String):
        self.handle_zoom_cmd(side="LEFT", cmd_str=msg.data, send_fn=self.send_cmd_left,
                             query_fn=self.query_visible_index_left,
                             publish_idx_fn=self.publish_szvn_left)

    def right_cmd_cb(self, msg: String):
        self.handle_zoom_cmd(side="RIGHT", cmd_str=msg.data, send_fn=self.send_cmd_right,
                             query_fn=self.query_visible_index_right,
                             publish_idx_fn=self.publish_szvn_right)

    def handle_zoom_cmd(self, side: str, cmd_str: str,
                        send_fn, query_fn, publish_idx_fn):
        """
        cmd_str 예시:
          - "step_in"
          - "step_out"
          - "index 8"      (→ SZVN 8)
          - "1x" / "8x" / "16x" / "30x"
          - "get"
        """
        c = cmd_str.strip().lower()
        self.get_logger().info(f"[{side}] zoom_cmd received: '{c}'")

        # 1) 스텝 인/아웃
        if c == "step_in":
            send_fn("SZIN")   # EO visible 한 스텝 줌 인 (메인 카메라가 VISIBLE이라고 가정)
            idx = query_fn()
            if idx is not None:
                publish_idx_fn(idx)
            return

        if c == "step_out":
            send_fn("SZOT")   # 한 스텝 줌 아웃
            idx = query_fn()
            if idx is not None:
                publish_idx_fn(idx)
            return

        # 2) index N  (예: "index 8" → SZVN 8)
        if c.startswith("index "):
            try:
                idx = int(c.split()[1])
            except Exception:
                self.get_logger().error(f"[{side}] invalid 'index' command: '{c}'")
                return
            if idx < IDX_MIN or idx > IDX_MAX:
                self.get_logger().warn(f"[{side}] index {idx} out of range [{IDX_MIN},{IDX_MAX}]")
                return
            send_fn(f"SZVN {idx}")
            publish_idx_fn(idx)
            return

        # 3) 배율 단축키 ("1x", "8x", "16x", "30x")
        #    네가 말한대로 1x→0, 8x→8, 16x→11, 30x→14
        if c in ("1x", "8x", "16x", "30x"):
            if c == "1x":
                idx = 0
            elif c == "8x":
                idx = 8
            elif c == "16x":
                idx = 11
            elif c == "30x":
                idx = 14
            else:
                idx = 0

            if idx < IDX_MIN or idx > IDX_MAX:
                self.get_logger().warn(f"[{side}] index {idx} out of range")
                return

            send_fn(f"SZVN {idx}")
            publish_idx_fn(idx)
            return

        # 4) 현재 줌 상태 읽기 (index만 publish)
        if c == "get":
            idx = query_fn()
            if idx is not None:
                publish_idx_fn(idx)
            else:
                self.get_logger().warn(f"[{side}] GZVV query failed")
            return

        self.get_logger().warn(f"[{side}] Unknown zoom command: '{cmd_str}'")


def main(args=None):
    rclpy.init(args=args)
    node = WirisDualZoomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.get_logger().info("Shutting down WirisDualZoomNode...")
    if node.sock_left is not None:
        node.sock_left.close()
    if node.sock_right is not None:
        node.sock_right.close()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
