
from __future__ import annotations
from time import sleep

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from std_msgs.msg import Bool, Float32
from typing import Optional

from .misc_function import MiscFunction
from wiris_enterprise_cam_driver_ros2_interfaces.srv import ZoomCmd, ZoomStatus, ServerReboot

def _qos_reliable():
    qos = QoSProfile(depth=1)
    qos.history = QoSHistoryPolicy.KEEP_LAST
    qos.reliability = QoSReliabilityPolicy.RELIABLE
    qos.durability = QoSDurabilityPolicy.VOLATILE
    return qos

class WirisServiceNode(Node):
    def __init__(self):
        super().__init__('wiris_service_node')

        # -------- declare parameters  --------
        # Activation / network
        self.declare_parameter('WirisEntpCam.activationInfo.ipAddress', '10.0.0.230')
        self.declare_parameter('WirisEntpCam.activationInfo.ipPort',     '2240')
        self.declare_parameter('WirisEntpCam.activationInfo.licenseKey', '')

        # IO timeout only (delimiter fixed to NUL in misc_function)
        self.declare_parameter('WirisEntpCam.settings.ioTimeoutSec',    1.0)

        # Topics
        self.declare_parameter('WirisEntpCam.topics.zoomService.zoomCtrl',     '/wiris_cam/srv/zoom_ctrl')
        self.declare_parameter('WirisEntpCam.topics.zoomService.zoomStatus',   '/wiris_cam/srv/zoom_status')
        self.declare_parameter('WirisEntpCam.topics.booting.rebootServer',     '/wiris_cam/srv/reboot')
        self.declare_parameter('WirisEntpCam.topics.pinger.pingServer',        '/wiris_cam/pub/ping')
        self.declare_parameter('WirisEntpCam.topics.relativeRangePub.relRangeStatus', '/wiris_cam/pub/relative_range_status')

        # Settings
        self.declare_parameter('WirisEntpCam.settings.capturePeriod', 1)    # Hz for ping
        self.declare_parameter('WirisEntpCam.settings.relRangeHz',    5.0)  # Hz for relative distance

        # Command Abbreviations (main/basic)
        self.declare_parameter('WirisEntpCam.commandAbbreviations.basicInfo.setDateTime', '')
        self.declare_parameter('WirisEntpCam.commandAbbreviations.mainSetting.setDateTime', '')
        self.declare_parameter('WirisEntpCam.commandAbbreviations.mainSetting.setAct', '')
        self.declare_parameter('WirisEntpCam.commandAbbreviations.mainSetting.setReboot', '')
        self.declare_parameter('WirisEntpCam.commandAbbreviations.mainSetting.getLaserDist', '')
        self.declare_parameter('WirisEntpCam.commandAbbreviations.mainSetting.setEthMode', '')
        self.declare_parameter('WirisEntpCam.commandAbbreviations.mainSetting.setNullChar', '')
        self.declare_parameter('WirisEntpCam.commandAbbreviations.mainSetting.pingCmd', '')

        # Zoom (EO / Thermal)
        self.declare_parameter('WirisEntpCam.commandAbbreviations.zoomCtrl.setZoomEoDiscrete', '')
        self.declare_parameter('WirisEntpCam.commandAbbreviations.zoomCtrl.getZoomEoDiscrete', '')
        self.declare_parameter('WirisEntpCam.commandAbbreviations.zoomCtrl.getZoomEoDiscCmd', '')
        self.declare_parameter('WirisEntpCam.commandAbbreviations.zoomCtrl.setZoomThermalDiscrete', '')
        self.declare_parameter('WirisEntpCam.commandAbbreviations.zoomCtrl.getZoomThermalDiscrete', '')
        self.declare_parameter('WirisEntpCam.commandAbbreviations.zoomCtrl.getZoomThermalDiscCmd', '')

        # -------- fetch parameters --------
        host = str(self.get_parameter('WirisEntpCam.activationInfo.ipAddress').value or '10.0.0.230')
        port = int(self.get_parameter('WirisEntpCam.activationInfo.ipPort').value or '2240')
        license_key = str(self.get_parameter('WirisEntpCam.activationInfo.licenseKey').value or '')

        io_timeout = float(self.get_parameter('WirisEntpCam.settings.ioTimeoutSec').value or 1.0)
        self._io_timeout = max(0.1, io_timeout)

        # topics
        zoom_srv_name  = str(self.get_parameter('WirisEntpCam.topics.zoomService.zoomCtrl').value)
        zoom_stat_name = str(self.get_parameter('WirisEntpCam.topics.zoomService.zoomStatus').value)
        reboot_name    = str(self.get_parameter('WirisEntpCam.topics.booting.rebootServer').value)
        ping_pub_name  = str(self.get_parameter('WirisEntpCam.topics.pinger.pingServer').value)
        rel_dist_topic = str(self.get_parameter('WirisEntpCam.topics.relativeRangePub.relRangeStatus').value)

        # settings
        ping_hz  = float(self.get_parameter('WirisEntpCam.settings.capturePeriod').value or 1)
        rel_hz   = float(self.get_parameter('WirisEntpCam.settings.relRangeHz').value or 5.0)

        # commands
        self.cmd_set_dt_basic = str(self.get_parameter('WirisEntpCam.commandAbbreviations.basicInfo.setDateTime').value or '')
        self.cmd_set_dt_main  = str(self.get_parameter('WirisEntpCam.commandAbbreviations.mainSetting.setDateTime').value or '')
        self.cmd_set_act      = str(self.get_parameter('WirisEntpCam.commandAbbreviations.mainSetting.setAct').value or '')
        self.cmd_reboot       = str(self.get_parameter('WirisEntpCam.commandAbbreviations.mainSetting.setReboot').value or '')
        self.cmd_get_laser    = str(self.get_parameter('WirisEntpCam.commandAbbreviations.mainSetting.getLaserDist').value or '')
        self.cmd_set_eth      = str(self.get_parameter('WirisEntpCam.commandAbbreviations.mainSetting.setEthMode').value or '')
        self.cmd_set_null     = str(self.get_parameter('WirisEntpCam.commandAbbreviations.mainSetting.setNullChar').value or '')
        self.cmd_ping         = str(self.get_parameter('WirisEntpCam.commandAbbreviations.mainSetting.pingCmd').value or '')

        self.cmd_set_eo   = str(self.get_parameter('WirisEntpCam.commandAbbreviations.zoomCtrl.setZoomEoDiscrete').value or '')
        self.cmd_get_eo_1 = str(self.get_parameter('WirisEntpCam.commandAbbreviations.zoomCtrl.getZoomEoDiscrete').value or '')
        self.cmd_get_eo_2 = str(self.get_parameter('WirisEntpCam.commandAbbreviations.zoomCtrl.getZoomEoDiscCmd').value or '')
        self.cmd_set_th   = str(self.get_parameter('WirisEntpCam.commandAbbreviations.zoomCtrl.setZoomThermalDiscrete').value or '')
        self.cmd_get_th_1 = str(self.get_parameter('WirisEntpCam.commandAbbreviations.zoomCtrl.getZoomThermalDiscrete').value or '')
        self.cmd_get_th_2 = str(self.get_parameter('WirisEntpCam.commandAbbreviations.zoomCtrl.getZoomThermalDiscCmd').value or '')

        # debugging
        self.get_logger().info(f'[wiris] host={host} port={port} timeout={self._io_timeout:.2f}s')
        self.get_logger().info(f'[wiris] zoom srv={zoom_srv_name}, status={zoom_stat_name}, reboot={reboot_name}')
        self.get_logger().info(f'[wiris] pubs ping={ping_pub_name}, rel_dist={rel_dist_topic}')
        if not self.cmd_get_laser:
            self.get_logger().warning('getLaserDist command not configured — publisher will idle until set.')

        # -------- init client / pubs / srvs --------
        self.misc = MiscFunction(host, str(port))

        self.pub_ping = self.create_publisher(Bool,    ping_pub_name, _qos_reliable())
        self.pub_rel  = self.create_publisher(Float32, rel_dist_topic, _qos_reliable())

        self.srv_zoom        = self.create_service(ZoomCmd,     zoom_srv_name,  self.handle_zoom_cmd)
        self.srv_zoom_status = self.create_service(ZoomStatus,  zoom_stat_name, self.handle_zoom_status)
        self.srv_reboot      = self.create_service(ServerReboot, reboot_name,   self.handle_reboot)

        # -------- startup sequence --------
        self.startup_sequence(license_key)

        # timers
        ping_period = max(0.2, 1.0 / max(1.0, ping_hz))
        self.create_timer(ping_period, self.timer_ping)

        rel_period = max(0.05, 1.0 / max(0.5, rel_hz))
        self.create_timer(rel_period, self.timer_rel_dist)

    # ---------------- lifecycle helpers ----------------
    def startup_sequence(self, license_key: str):
        # set command delimiter like C++: "<setNullChar> NULL\n" (write-only)
        if self.cmd_set_null:
            try:
                line = f"{self.cmd_set_null} NULL\n"
                try:
                    self.misc.net.send_raw(line)  # write only; NetworkClient adds trailing NULL
                    sleep(0.05)
                    ans = self.misc.send_raw("GDLM", timeout_sec=self._io_timeout)
                    print(f"GDLM response: {ans}", flush=True)  # Debugging output
                except Exception:
                    # fallback: short read (won't harm if device responds)
                    _ = self.misc.send_raw(line, timeout_sec=min(0.5, self._io_timeout))
                self.get_logger().info('Command delimiter configured (NULL).')
            except Exception as e:
                self.get_logger().warning(f"setNullChar failed: {e}")

        # activation
        if self.cmd_set_act and license_key:
            try:
                self.misc.send_raw(f"{self.cmd_set_act} {license_key}", timeout_sec=self._io_timeout)
                self.get_logger().info("Activation key sent.")
            except Exception as e:
                self.get_logger().warning(f"Activation failed: {e}")

        # optional ETH enable
        if self.cmd_set_eth:
            try:
                self.misc.send_raw(f"{self.cmd_set_eth} TRUE", timeout_sec=self._io_timeout)
            except Exception as e:
                self.get_logger().warning(f"setEthMode failed: {e}")

        # simple ping retries
        for _ in range(3):
            if self.misc.ping(self.cmd_ping, timeout_sec=self._io_timeout):
                break

        # set time
        set_dt = self.cmd_set_dt_basic or self.cmd_set_dt_main
        if set_dt:
            when = self._cam_time_string()
            try:
                if self.misc.send_raw(f"{set_dt} {when}", timeout_sec=self._io_timeout):
                    self.get_logger().info(f"Camera time set: {when}")
            except Exception as e:
                self.get_logger().warning(f"Failed to set camera time: {e}")

    def _cam_time_string(self) -> str:
        from datetime import datetime
        return datetime.now().strftime("%Y %m %d %H %M %S")

    # ---------------- services ----------------
    def handle_zoom_cmd(self, req: ZoomCmd.Request, res: ZoomCmd.Response):
        cam = (getattr(req, 'cam', '') or "EO").upper()
        idx = int(getattr(req, 'index', 0))
        ok = False
        try:
            if cam in ("EO", "BOTH") and self.cmd_set_eo:
                ok = self.misc.set_zoom_discrete(self.cmd_set_eo, idx, timeout_sec=self._io_timeout) or ok
            if cam in ("THERMAL", "BOTH") and self.cmd_set_th:
                ok = self.misc.set_zoom_discrete(self.cmd_set_th, idx, timeout_sec=self._io_timeout) or ok
            res.response = True
            if hasattr(res, 'message'):
                res.message = "ok" if ok else "no-op"
        except Exception as e:
            res.response = False
            if hasattr(res, 'message'):
                res.message  = f"error: {e}"
        return res

    def handle_zoom_status(self, req: ZoomStatus.Request, res: ZoomStatus.Response):
        cam  = (getattr(req, 'cam', '') or "EO").upper()
        idx: Optional[int] = -1
        zoom: float = 0.0
        try:
            if cam == "EO":
                get_eo = self.cmd_get_eo_1
                if get_eo:
                    v = self.misc.get_zoom_discrete(get_eo, timeout_sec=self._io_timeout)
                    idx = v if v is not None else -1
                    self.get_logger().info(get_eo)
                    self.get_logger().info(f"GDLM response: {v}")
            elif cam == "THERMAL":
                get_th = self.cmd_get_th_1
                if get_th:
                    v = self.misc.get_zoom_discrete(get_th, timeout_sec=self._io_timeout)
                    idx = v if v is not None else -1
                    self.get_logger().info(get_th)
                    self.get_logger().info(f"GDLM response: {v}")
        except Exception as e:
            self.get_logger().warning(f"zoom status error: {e}")

        if hasattr(res, 'index'):
            res.index = int(idx if idx is not None else -1)
        if hasattr(res, 'zoom'):
            res.zoom  = float(zoom)
        if hasattr(res, 'response'):
            res.response = True
        return res

    def handle_reboot(self, req: ServerReboot.Request, res: ServerReboot.Response):
        try:
            do_reboot = getattr(req, 'reboot', False)
            if do_reboot and self.cmd_reboot:
                self.misc.send_raw(self.cmd_reboot, timeout_sec=self._io_timeout)
                if hasattr(res, 'response'):
                    res.response = True
                if hasattr(res, 'message'):
                    res.message = "reboot issued"
                if hasattr(res, 'response'):
                    res.response = True
            else:
                if hasattr(res, 'response'):
                    res.response = False
                if hasattr(res, 'message'):
                    res.message = "reboot flag false or command not configured"
                if hasattr(res, 'response'):
                    res.response = False
        except Exception as e:
            if hasattr(res, 'response'):
                res.response = False
            if hasattr(res, 'message'):
                res.message = f"error: {e}"
            if hasattr(res, 'response'):
                res.response = False
        return res

    # ---------------- timers ----------------
    def timer_ping(self):
        ok = self.misc.ping(self.cmd_ping, timeout_sec=self._io_timeout)
        self.pub_ping.publish(Bool(data=bool(ok)))
        self.get_logger().debug(f'ping={ok}', throttle_duration_sec=self._io_timeout)

    def timer_rel_dist(self):
        try:
            if not self.cmd_get_laser:
                self.get_logger().debug('getLaserDist not set', throttle_duration_sec=self._io_timeout)
                return
            v = self.misc.get_relative_distance(self.cmd_get_laser, timeout_sec=self._io_timeout)
            if v is not None:
                self.pub_rel.publish(Float32(data=float(v)))
                self.get_logger().debug(f'rel_dist={v}', throttle_duration_sec=self._io_timeout)
            else:
                self.get_logger().debug('rel_dist=None (parse mismatch?)', throttle_duration_sec=self._io_timeout)
        except Exception as e:
            self.get_logger().debug(f"rel dist read fail: {e}", throttle_duration_sec=self._io_timeout)

def main(args=None):
    rclpy.init(args=args)
    node = WirisServiceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
