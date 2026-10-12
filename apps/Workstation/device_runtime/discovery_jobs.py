"""Own captured NIC discovery and the legacy camera-address apply job.

Host discovery remains read-only. The legacy address worker keeps its existing
OS mutation policy; production role assignment still uses NetworkRoleRuntime.
These four slots remain independent, with existing UI discovery exclusions at
admission. No worker opens a second rig command transport.
"""

from dataclasses import dataclass

from .qt_jobs import QtOperationJobs


class _DiscoveryCommand:
    camera = ""
    per_camera = False
    result_signal = "discovered"
    text_progress = False


@dataclass(frozen=True)
class CameraHostDiscovery(_DiscoveryCommand):
    host_address: str
    host_hints: tuple
    kind = "camera_host"

    def __post_init__(self):
        object.__setattr__(self, "host_hints", tuple(self.host_hints))

    def build(self, factory):
        return factory(host_address=self.host_address, host_hints=self.host_hints)


@dataclass(frozen=True)
class PixhawkHostDiscovery(_DiscoveryCommand):
    kind = "pixhawk_host"

    def build(self, factory):
        return factory()


@dataclass(frozen=True)
class PixhawkNetworkDiscovery(_DiscoveryCommand):
    p1_peer: str
    p2_peer: str
    p1_endpoint: str
    p2_endpoint: str
    kind = "pixhawk_network"

    def build(self, factory):
        return factory(p1_peer=self.p1_peer, p2_peer=self.p2_peer,
                       p1_endpoint=self.p1_endpoint, p2_endpoint=self.p2_endpoint)


@dataclass(frozen=True)
class CameraAddressApply(_DiscoveryCommand):
    interface_name: str
    address_text: str
    kind = "camera_address"
    result_signal = "applied"

    def build(self, factory):
        return factory(interface_name=self.interface_name, address_text=self.address_text)


DISCOVERY_PREFIXES = {
    "camera_host": "camera_network_discovery",
    "camera_address": "camera_network_apply",
    "pixhawk_host": "pixhawk_host_ip_discovery",
    "pixhawk_network": "pixhawk_network_discovery",
}
DISCOVERY_FIELDS = tuple(f"{prefix}_{field}" for prefix in DISCOVERY_PREFIXES.values()
                         for field in ("thread", "worker"))


class NetworkDiscoveryJobs(QtOperationJobs):
    command_types = (CameraHostDiscovery, PixhawkHostDiscovery, PixhawkNetworkDiscovery, CameraAddressApply)
    prefixes = DISCOVERY_PREFIXES
    handle_fields = DISCOVERY_FIELDS
