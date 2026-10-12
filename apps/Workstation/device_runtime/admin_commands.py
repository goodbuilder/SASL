"""Captured admin requests and the existing persistent Ethernet command policy.

No command creates a second MAVLink connection. The captured subscriber is the
existing transport/lock owner; USB workers retain their existing identity policy.
All values are protocol/configuration values, with no frame or unit conversion.
SYSID/MAV_2 parameter lines and shell/FTP interface files are built from one
immutable configuration input, independently of widgets or the transport.
"""

from dataclasses import dataclass
from types import MappingProxyType
from copy import deepcopy
from collections.abc import Mapping


@dataclass(frozen=True)
class PersistentEthernetInputs:
    """One GUI-captured role/configuration snapshot; no live readers.

    Endpoint addresses, UDP port text and the unchanged MAVLink rate are device
    configuration, not position or orientation values in NED/FRD/RDF.
    """

    camera: str
    eth_ip: str
    host_ip: str
    mavlink_url: str
    mavlink_rate: int


@dataclass(frozen=True)
class PersistentEthernetConfiguration:
    """Immutable result for the existing shell, FTP and USB bootstrap paths."""

    eth_ip: str
    host_ip: str
    ifup_cmd: str
    ifconfig_cmd: str
    param_lines: tuple[str, ...]
    extras_lines: tuple[str, ...]
    net_cfg_lines: tuple[str, ...]

    def __post_init__(self):
        for name in ("param_lines", "extras_lines", "net_cfg_lines"):
            object.__setattr__(self, name, tuple(getattr(self, name)))

    @classmethod
    def from_legacy_setup(cls, setup: Mapping[str, object]):
        """Accept the finite old setup schema at the outer compatibility edge."""
        return cls(
            eth_ip=setup.get("eth_ip", ""), host_ip=setup.get("host_ip", ""),
            ifup_cmd=setup.get("ifup_cmd", ""), ifconfig_cmd=setup.get("ifconfig_cmd", ""),
            param_lines=tuple(setup.get("param_lines", ())), extras_lines=tuple(setup.get("extras_lines", ())),
            net_cfg_lines=tuple(setup.get("net_cfg_lines", ())),
        )

    def legacy_setup(self) -> dict[str, object]:
        """Keep supported dict/list output shapes without mutable state aliases."""
        return {
            "eth_ip": self.eth_ip,
            "host_ip": self.host_ip,
            "ifup_cmd": self.ifup_cmd,
            "ifconfig_cmd": self.ifconfig_cmd,
            "param_lines": list(self.param_lines),
            "extras_lines": list(self.extras_lines),
            "net_cfg_lines": list(self.net_cfg_lines),
        }

    def persistent_command(self) -> str:
        extras = "/fs/microsd/etc/extras.txt"
        return "\n".join((
            "mkdir /fs/microsd/etc",
            *self.param_lines,
            "param save",
            *self.extras_lines,
            "mavlink status",
            f"echo '{self.extras_lines[0]}' > {extras}",
            *(f"echo '{line}' >> {extras}" for line in self.extras_lines[1:]),
            f"cat {extras}",
        ))

    def extras_text(self) -> str:
        return "\n".join(self.extras_lines) + "\n"

    def net_cfg_text(self) -> str:
        return "\n".join(self.net_cfg_lines) + "\n"

    def bootstrap_command(self, *, mode: str = "normal") -> str:
        if mode not in {"normal", "onboard"}:
            raise ValueError(f"Unsupported Ethernet MAVLink bootstrap mode: {mode}")
        extras = "/fs/microsd/etc/extras.txt"
        net_cfg = "/fs/microsd/net.cfg"
        # MAV_2_CONFIG is the sole owner of the Ethernet MAVLink instance.
        # A second manual start can bind the same UDP port on NuttX and emit a
        # duplicate unicast stream beside the parameter-owned broadcast stream.
        # Persist the interface files and reboot so rc.serial creates exactly
        # one instance; never stop or manually start one from recovery.
        return "\n".join((
            "mkdir /fs/microsd/etc",
            *self.param_lines,
            "param save",
            f"echo '{self.extras_lines[0]}' > {extras}",
            *(f"echo '{line}' >> {extras}" for line in self.extras_lines[1:]),
            f"echo '{self.net_cfg_lines[0]}' > {net_cfg}",
            *(f"echo '{line}' >> {net_cfg}" for line in self.net_cfg_lines[1:]),
            "reboot",
        ))


def build_persistent_ethernet_configuration(inputs: PersistentEthernetInputs) -> PersistentEthernetConfiguration:
    """Preserve the established SYSID/MAV_2/interface-file command policy."""
    role_defaults = {"left": (1, 14550), "right": (2, 14551), "drone": (3, 14552)}
    if inputs.camera not in role_defaults:
        raise ValueError(f"Unknown Pixhawk side: {inputs.camera}")
    system_id, default_port = role_defaults[inputs.camera]
    for label, value in (("Pixhawk ETH IP", inputs.eth_ip), ("Pixhawk Host IP", inputs.host_ip)):
        if not value or any(ch.isspace() or ch in "'\";`$" for ch in value):
            raise ValueError(f"{label} 값이 비어 있거나 shell 명령에 사용할 수 없는 문자를 포함합니다: {value!r}")
    # Same endpoint parsing/fallback as the existing _mavlink_stream_port edge.
    port_text = inputs.mavlink_url.strip().rsplit(":", 1)[-1]
    if "/" in port_text:
        port_text = port_text.split("/", 1)[0]
    try:
        port = int(port_text)
    except ValueError:
        port = int(default_port)
    ifup_cmd = "ifup eth0"
    ifconfig_cmd = f"ifconfig eth0 {inputs.eth_ip} netmask 255.255.255.0 up"
    return PersistentEthernetConfiguration(
        eth_ip=inputs.eth_ip, host_ip=inputs.host_ip, ifup_cmd=ifup_cmd, ifconfig_cmd=ifconfig_cmd,
        param_lines=(
            f"param set MAV_SYS_ID {system_id}",
            "param set MAV_2_CONFIG 1000",
            "param set MAV_2_MODE 0",
            "param set MAV_2_BROADCAST 1",
            f"param set MAV_2_REMOTE_PRT {port}",
            f"param set MAV_2_UDP_PRT {port}",
            f"param set MAV_2_RATE {inputs.mavlink_rate}",
        ),
        # rc.serial owns the one MAVLink instance through MAV_2_CONFIG.
        # extras.txt only establishes the board-specific Ethernet interface.
        extras_lines=(ifup_cmd, ifconfig_cmd),
        net_cfg_lines=(
            "DEVICE=eth0", "BOOTPROTO=static", "NETMASK=255.255.255.0",
            f"IPADDR={inputs.eth_ip}", f"ROUTER={inputs.host_ip}",
        ),
    )


@dataclass(frozen=True)
class TransferCommand:
    subscriber: object
    camera: str
    local_path: str
    remote_path: str
    resume: bool
    evidence_path: str
    direction: str
    category: str
    total: int
    kind = "mavftp"
    per_camera = True
    result_signal = "completed"
    text_progress = False

    def build(self, factory):
        return factory(self.subscriber, camera=self.camera, local_path=self.local_path,
                       remote_path=self.remote_path, resume=self.resume,
                       evidence_path=self.evidence_path, direction=self.direction)


@dataclass(frozen=True)
class FlashCommand:
    camera: str
    expected_system_id: int
    usb_device: str
    firmware_path: str
    uploader_path: str
    kind = "usb_flash"
    per_camera = False
    result_signal = "completed"
    text_progress = True

    def build(self, factory):
        return factory(camera=self.camera, expected_system_id=self.expected_system_id,
                       usb_device=self.usb_device, firmware_path=self.firmware_path,
                       uploader_path=self.uploader_path)


@dataclass(frozen=True)
class BootstrapCommand:
    camera: str
    usb_url: str
    baud_candidates: tuple
    command: str
    fallback_command: str
    kind = "usb_bootstrap"
    per_camera = False
    result_signal = "completed"
    text_progress = False

    def __post_init__(self):
        object.__setattr__(self, "baud_candidates", tuple(self.baud_candidates))

    def build(self, factory):
        return factory(camera=self.camera, usb_url=self.usb_url, baud_candidates=self.baud_candidates,
                       command=self.command, fallback_command=self.fallback_command)


@dataclass(frozen=True)
class RecoveryCommand:
    camera: str
    evidence: object
    expected_system_id: int
    kind = "usb_recovery"
    per_camera = False
    result_signal = "completed"
    text_progress = False

    def __post_init__(self):
        # USB identity evidence is a flat producer record. Copy its values so
        # discovery replacing the GUI list cannot retarget an admitted reboot.
        object.__setattr__(self, "evidence", MappingProxyType(deepcopy(dict(self.evidence))))

    def build(self, factory):
        return factory(camera=self.camera, evidence=deepcopy(dict(self.evidence)),
                       expected_system_id=self.expected_system_id)


@dataclass(frozen=True)
class PersistentCommand:
    subscriber: object
    camera: str
    command: str
    extras_text: str
    net_cfg_text: str
    kind = "persistent_config"
    per_camera = False
    result_signal = "completed"
    text_progress = False

    def build(self, factory):
        return factory(self.subscriber, camera=self.camera, command=self.command,
                       extras_text=self.extras_text, net_cfg_text=self.net_cfg_text)


@dataclass(frozen=True)
class StreamSetupCommand:
    subscriber: object
    host_ip: str
    ports: object
    sysids: object
    rate: int
    cameras: tuple
    kind = "stream_setup"
    per_camera = False
    result_signal = "completed"
    text_progress = False
    camera = ""

    def __post_init__(self):
        object.__setattr__(self, "ports", MappingProxyType(dict(self.ports)))
        object.__setattr__(self, "sysids", MappingProxyType(dict(self.sysids)))
        object.__setattr__(self, "cameras", tuple(self.cameras))

    def build(self, factory):
        return factory(self.subscriber, host_ip=self.host_ip, ports=dict(self.ports),
                       sysids=dict(self.sysids), rate=self.rate, cameras=self.cameras)


ADMIN_COMMAND_TYPES = (TransferCommand, FlashCommand, BootstrapCommand, RecoveryCommand,
                       PersistentCommand, StreamSetupCommand)
