#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Float32MultiArray, Int32
import numpy as np

# Workswell EO optical zoom table
ZOOM_TABLE = [1.0, 1.2, 1.5, 2.0, 3.0, 4.0, 5.0, 6.0, 8.0, 10.0, 12.0, 16.0, 20.0, 25.0, 30.0]
IDX_MIN, IDX_MAX = 0, len(ZOOM_TABLE) - 1


class StereoDepthNode(Node):
    def __init__(self):
        super().__init__('stereo_depth_node')

        # ---------- Subscribers ----------
        # YOLO 객체 중심: PointStamped.point.x=cx, point.y=cy (pixel)
        self.create_subscription(
            PointStamped,
            '/wiris/left/center',
            self.left_cb,
            10
        )
        self.create_subscription(
            PointStamped,
            '/wiris/right/center',
            self.right_cb,
            10
        )

        # Zoom index topics (/szvn, /szvn_left, /szvn_right)
        self.create_subscription(Int32, '/szvn',       self._szvn_both_cb, 10)
        self.create_subscription(Int32, '/szvn_left',  self._szvn_left_cb, 10)
        self.create_subscription(Int32, '/szvn_right', self._szvn_right_cb, 10)

        # ---------- Publishers ----------
        self.pub_depth_point = self.create_publisher(Point, '/object_depth/point', 10)
        self.pub_depth_info  = self.create_publisher(Float32MultiArray, '/object_depth/info', 10)

        # ---------- Base intrinsics (zoom=1 기준) ----------
        # ※ 해상도에 맞게 너가 값 조정해야 함 (아래는 예: 1280x720)
        self.base_fx = 1484.0
        self.base_fy = 1484.0
        self.cx = 1920.0 / 2.0
        self.cy = 1080.0 / 2.0

        # ---------- Zoom scale ----------
        self.declare_parameter('zoom_scale_left', 1.0)
        self.declare_parameter('zoom_scale_right', 1.0)
        self.zL = float(self.get_parameter('zoom_scale_left').value)
        self.zR = float(self.get_parameter('zoom_scale_right').value)

        # 카메라 행렬 계산
        self.update_intrinsics()

        # 런타임 파라미터 변경 대응
        self.add_on_set_parameters_callback(self._on_param_set)

        # ---------- Baseline / Extrinsics ----------
        self.posA = np.array([0.0, 0.0, 0.0])   # LEFT camera origin
        self.posB = np.array([10, 0.0, 0.0])   # RIGHT camera origin (baseline ≈ 1.1m, 너가 실제값으로 수정)
        self.R_A = np.eye(3)
        self.R_B = np.eye(3)

        # optical -> link (camera_optical_frame -> camera_link)
        self.link_from_opt = np.array([
            [0, 0, 1],
            [-1, 0, 0],
            [0, -1, 0]
        ], float)

        self.pxA = None  # (uA, vA)
        self.pxB = None  # (uB, vB)

        self.get_logger().info(
            f"✅ StereoDepthNode started | zL={self.zL:.2f}, zR={self.zR:.2f}, "
            f"fxL={self.fx_L:.1f}, fxR={self.fx_R:.1f}"
        )

    # ---------- Intrinsics update ----------
    def update_intrinsics(self):
        self.fx_L = self.base_fx * self.zL
        self.fy_L = self.base_fy * self.zL
        self.fx_R = self.base_fx * self.zR
        self.fy_R = self.base_fy * self.zR

        self.K_L = np.array([[self.fx_L, 0, self.cx],
                             [0, self.fy_L, self.cy],
                             [0,       0,      1]], float)
        self.K_R = np.array([[self.fx_R, 0, self.cx],
                             [0, self.fy_R, self.cy],
                             [0,       0,      1]], float)

        self.invK_L = np.linalg.inv(self.K_L)
        self.invK_R = np.linalg.inv(self.K_R)

        self.get_logger().info(
            f"[K update] zL={self.zL:.2f}, zR={self.zR:.2f} | "
            f"fxL={self.fx_L:.1f}, fxR={self.fx_R:.1f}"
        )

    # ---------- Zoom param runtime change ----------
    def _on_param_set(self, params):
        ok = True
        for p in params:
            if p.name in ('zoom_scale_left', 'zoom_scale_right'):
                try:
                    v = float(p.value) if p.type_ != Parameter.Type.DOUBLE else p.value
                    if v <= 0.0:
                        ok = False
                except Exception:
                    ok = False

        if not ok:
            return SetParametersResult(successful=False, reason='zoom scale must be positive float')

        for p in params:
            if p.name == 'zoom_scale_left':
                self.zL = float(p.value)
            elif p.name == 'zoom_scale_right':
                self.zR = float(p.value)

        self.update_intrinsics()
        return SetParametersResult(successful=True)

    # ---------- Zoom topics callbacks ----------
    def _idx_to_ratio(self, idx: int) -> float:
        if idx < IDX_MIN or idx > IDX_MAX:
            self.get_logger().warn(f"zoom idx {idx} out of range [{IDX_MIN},{IDX_MAX}], ignore")
            return None
        return ZOOM_TABLE[idx]

    def _szvn_both_cb(self, msg: Int32):
        r = self._idx_to_ratio(int(msg.data))
        if r is None:
            return
        self.zL = r
        self.zR = r
        self.get_logger().info(f"[ZOOM] /szvn -> idx={msg.data} -> ratio={r:.2f}x (L/R 동일)")
        self.update_intrinsics()

    def _szvn_left_cb(self, msg: Int32):
        r = self._idx_to_ratio(int(msg.data))
        if r is None:
            return
        self.zL = r
        self.get_logger().info(f"[ZOOM] /szvn_left -> idx={msg.data} -> ratio={r:.2f}x (LEFT)")
        self.update_intrinsics()

    def _szvn_right_cb(self, msg: Int32):
        r = self._idx_to_ratio(int(msg.data))
        if r is None:
            return
        self.zR = r
        self.get_logger().info(f"[ZOOM] /szvn_right -> idx={msg.data} -> ratio={r:.2f}x (RIGHT)")
        self.update_intrinsics()

    # ---------- Callbacks ----------
    def left_cb(self, msg: PointStamped):
        self.pxA = (float(msg.point.x), float(msg.point.y))
        self.try_solve()

    def right_cb(self, msg: PointStamped):
        self.pxB = (float(msg.point.x), float(msg.point.y))
        self.try_solve()

    # ---------- Triangulation core ----------
    def triangulate_world(self, uA, vA, uB, vB):
        # pixel → optical
        dA_C = self.invK_L @ np.array([uA, vA, 1.0], float)
        dB_C = self.invK_R @ np.array([uB, vB, 1.0], float)

        # optical → link → world
        dA = self.R_A @ (self.link_from_opt @ dA_C)
        dB = self.R_B @ (self.link_from_opt @ dB_C)

        BA = self.posB - self.posA
        A0 = np.array([
            [-np.dot(dA, dA),  np.dot(dB, dA)],
            [-np.dot(dA, dB),  np.dot(dB, dB)]
        ], float)
        B0 = np.array([
            -np.dot(BA, dA),
            -np.dot(BA, dB)
        ], float)

        s, t = np.linalg.solve(A0, B0)
        P_A = self.posA + s * dA
        P_B = self.posB + t * dB
        P = 0.5 * (P_A + P_B)
        return P

    # ---------- Main solver ----------
    def try_solve(self):
        if self.pxA is None or self.pxB is None:
            return

        uA, vA = self.pxA
        uB, vB = self.pxB

        disp = uA - uB
        if abs(disp) < 1e-6:
            self.get_logger().warn("invalid disparity (uA ≈ uB)")
            return

        baseline = float(np.linalg.norm(self.posB - self.posA))

        if self.fx_L != self.fx_R:
            self.get_logger().warn(
                f"zoom mismatch (fx_L={self.fx_L:.1f}, fx_R={self.fx_R:.1f}). "
                f"Use same zoom for both cameras for best accuracy."
            )
        fx_eff = 0.5 * (self.fx_L + self.fx_R)
        z_rect = fx_eff * baseline / abs(disp)
        self.get_logger().info(
            f"[DISP] uA={uA:.1f}, uB={uB:.1f}, disp={disp:.1f}px -> depth≈{z_rect:.3f}m"
        )

        try:
            P = self.triangulate_world(uA, vA, uB, vB)
        except np.linalg.LinAlgError:
            self.get_logger().warn("Triangulation failed (nearly parallel rays)")
            return

        range_ = float(np.linalg.norm(P))

        # 좌표계 변환 (X:→, Y:↑, Z:depth)
        X_out = -P[1]
        Y_out =  P[2]
        Z_out =  P[0]

        pt = Point()
        pt.x = float(X_out)
        pt.y = float(Y_out)
        pt.z = float(Z_out)
        self.pub_depth_point.publish(pt)

        arr = Float32MultiArray()
        arr.data = [float(X_out), float(Y_out), float(Z_out), range_]
        self.pub_depth_info.publish(arr)

        self.get_logger().info(
            f"[Triangulated OUT] X={X_out:.3f}(→), Y={Y_out:.3f}(↑), Z={Z_out:.3f}(depth), range={range_:.3f}m"
        )


def main():
    rclpy.init()
    node = StereoDepthNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
