from __future__ import annotations
from dataclasses import dataclass, field
from typing import Any, Dict
from rclpy.node import Node
from datetime import datetime

def _dotget(d: Dict[str, Any], path: str, default=None):
    cur = d
    for p in path.split('.'):
        if not isinstance(cur, dict) or p not in cur:
            return default
        cur = cur[p]
    return cur

# C++ params.yaml의 /WirisEntpCam/** 중 본 노드가 실제로 쓰는 핵심 키를 명시
ROS2_KEYS = [
    # Activation / Network
    "WirisEntpCam.activationInfo.ipAddress",
    "WirisEntpCam.activationInfo.ipPort",
    "WirisEntpCam.activationInfo.licenseKey",

    # Topics (services & pubs)
    "WirisEntpCam.topics.zoomService.zoomCtrl",
    "WirisEntpCam.topics.zoomService.zoomStatus",
    "WirisEntpCam.topics.booting.rebootServer",
    "WirisEntpCam.topics.pinger.pingServer",
    "WirisEntpCam.topics.relativeRangePub.relRangeStatus",

    # Command abbreviations used here
    "WirisEntpCam.commandAbbreviations.basicInfo.setDateTime",
    "WirisEntpCam.commandAbbreviations.mainSetting.setDateTime",  # 일부 yaml에 이 키로 있는 경우 대비
    "WirisEntpCam.commandAbbreviations.mainSetting.setAct",
    "WirisEntpCam.commandAbbreviations.mainSetting.setReboot",
    "WirisEntpCam.commandAbbreviations.mainSetting.getLaserDist",
    "WirisEntpCam.commandAbbreviations.mainSetting.setEthMode",
    "WirisEntpCam.commandAbbreviations.mainSetting.setNullChar",

    # Zoom
    "WirisEntpCam.commandAbbreviations.zoomCtrl.setZoomEoDiscrete",
    "WirisEntpCam.commandAbbreviations.zoomCtrl.getZoomEoDiscrete",
    "WirisEntpCam.commandAbbreviations.zoomCtrl.getZoomEoDiscCmd",
    "WirisEntpCam.commandAbbreviations.zoomCtrl.setZoomThermalDiscrete",
    "WirisEntpCam.commandAbbreviations.zoomCtrl.getZoomThermalDiscrete",
    "WirisEntpCam.commandAbbreviations.zoomCtrl.getZoomThermalDiscCmd",

    # Timing
    "WirisEntpCam.settings.capturePeriod",               # ping 간격 근사치
]

@dataclass
class ConfigParam:
    params: Dict[str, Any] = field(default_factory=dict)

    @classmethod
    def from_node(cls, node: Node) -> "ConfigParam":
        tree: Dict[str, Any] = {}
        for name in ROS2_KEYS:
            val = node.get_parameter(name).value if node.has_parameter(name) else None
            cur = tree
            parts = name.split('.')
            for p in parts[:-1]:
                cur = cur.setdefault(p, {})
            cur[parts[-1]] = val
        return cls(params=tree)

    def get(self, path: str, default=None):
        return _dotget(self.params, path, default)

    def gen_local_time_cam_date(self) -> str:
        # 카메라가 기대하는 "YYYY MM DD HH MM SS"
        return datetime.now().strftime("%Y %m %d %H %M %S")