import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy)
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import platform

def _qos_reliable():
    qos = QoSProfile(depth=1)
    qos.history = QoSHistoryPolicy.KEEP_LAST
    qos.reliability = QoSReliabilityPolicy.RELIABLE
    qos.durability = QoSDurabilityPolicy.VOLATILE
    return qos

class WirisStreamIrNode(Node):
    def __init__(self):
        super().__init__('wiris_stream_ir_node')

        # making parameters
        self.declare_parameter('camera.ip', '10.0.0.230')
        self.declare_parameter('camera.video_port', 8554)
        self.declare_parameter('rtsp.transport', 'udp')
        self.declare_parameter('rtsp.latency_ms', 50)
        self.declare_parameter('rtsp.drop_on_latency', True)
        self.declare_parameter('target', 'auto')             # 'jetson'|'intel'|'auto'
        self.declare_parameter('decoder', 'auto')            # 'nvv4l2'|'vaapi'|'avdec'|'auto'
        self.declare_parameter('appsink.max_buffers', 1)
        self.declare_parameter('appsink.drop', True)
        self.declare_parameter('frame_id', 'wiris_camera')
        self.declare_parameter('publish_ir_topic', '/wiris/ir/image_raw')
        self.declare_parameter('publish_rate_hz', 30.0)
        self.declare_parameter('stamp_source', 'now')        # 'now'|'system'
        self.bridge = CvBridge()

        # parsing parameters
        ip = self.get_parameter('camera.ip').value
        vport = int(self.get_parameter('camera.video_port').value)
        path = "/thermal" # fixed
        transport = self.get_parameter('rtsp.transport').value
        latency = int(self.get_parameter('rtsp.latency_ms').value)
        drop_on_latency = bool(self.get_parameter('rtsp.drop_on_latency').value)
        target = self.get_parameter('target').value
        decoder = self.get_parameter('decoder').value
        max_buffers = int(self.get_parameter('appsink.max_buffers').value)
        appsink_drop = bool(self.get_parameter('appsink.drop').value)
        topic = self.get_parameter('publish_ir_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.period = 1.0 / max(rate_hz, 1.0)

        # detecting target if auto
        if target == 'auto':
            uname = platform.uname().machine.lower()
            target = 'jetson' if ('tegra' in uname or 'aarch64' in uname) else 'intel'

        gst = self._build_gst_pipeline(ip, vport, path, transport, latency, drop_on_latency,
                                       target, decoder, max_buffers, appsink_drop)
        self.get_logger().info(f'GStreamer pipeline: {gst}')

        # setting opencv capture
        self.cap = cv2.VideoCapture(gst, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            self.get_logger().error('Failed to open GStreamer pipeline')

        # setting publishers: reliable (tools/rqt-friendly)
        rel_qos = _qos_reliable()
        self.pub_ir_rel  = self.create_publisher(Image, f"{topic}_reliable", rel_qos)

        # setting timer
        self.timer = self.create_timer(self.period, self._tick)

    # making pipeline
    def _build_gst_pipeline(self, ip, port, path, transport, latency,
                            drop_on_latency, target, decoder, max_buffers, appsink_drop):
        # rtspsrc tuning
        base = (
            f"rtspsrc location=rtsp://{ip}:{port}{path} "
            f"protocols={'tcp' if transport.lower()=='tcp' else 'udp'} "
            f"latency={latency}"
        )
        if drop_on_latency:
            base += " drop-on-latency=true"
        base += " ! rtpjitterbuffer drop-on-late=true"

        # parsing (assume H264)
        base += " ! rtph264depay ! h264parse"

        # setting decoder presets
        if target == 'jetson' or decoder == 'nvv4l2':
            dec = (" ! nvv4l2decoder enable-max-performance=1 "
                   "! nvvidconv ! video/x-raw,format=BGRx "
                   "! videoconvert ! video/x-raw,format=BGR")
        elif decoder == 'vaapi':
            dec = " ! vaapih264dec ! videoconvert ! video/x-raw,format=BGR"
        elif decoder == 'avdec':
            dec = " ! avdec_h264 ! videoconvert ! video/x-raw,format=BGR"
        else: # decoder == 'auto'
            dec = " ! decodebin ! videoconvert ! video/x-raw,format=BGR"

        # setting sink
        sink = f" ! appsink max-buffers={max_buffers} drop={'true' if appsink_drop else 'false'} sync=false"
        return base + dec + sink

    # setting ticking for publisher
    def _tick(self):
        if not hasattr(self, 'cap') or not self.cap.isOpened():
            return
        ok, frame = self.cap.read()
        if not ok or frame is None:
            return

        # publishing ir image
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self.pub_ir_rel.publish(msg)

# setting main function
def main(args=None):
    rclpy.init(args=args)
    node = WirisStreamIrNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()