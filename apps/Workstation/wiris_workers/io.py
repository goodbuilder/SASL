"""Stateless WIRIS device/network effect adapter shared by worker owners.

Functions are explicit test/simulation injection points. This module owns no
connection, thread, settings or fallback policy; each worker owns its I/O scope.
"""
from Workstation.workstation_core.network import apply_interface_address, check_tcp_port, discover_host_interface_candidates, interface_has_address, interface_link_exists, resolve_nm_interface_address
from Workstation.workstation_core.wiris_zoom import send_tcp_line
