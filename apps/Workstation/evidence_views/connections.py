"""Pure connection discovery/recovery projections from captured endpoint values.

Addresses/IDs identify transports only; HB/ATT ages are host receive monotonic
seconds. No endpoint discovery, widget read or transport operation occurs here.
"""

from collections.abc import Mapping

from Workstation.workstation_core.mavlink_liveness import MAVLINK_QUALIFICATION_S

from .snapshot import thaw_snapshot
from .values import _integer, _number

HEALTH_FIELDS = (
    "qualification_required_s",
    "attitude_rx_age_s",
    "heartbeat_rx_age_s",
    "listener_open",
    "qualification_elapsed_s",
)


def project_network_roles(roles, *, status, active):
    return thaw_snapshot(
        {
            "network_role_status": dict(status),
            "network_role_active": active,
            "network_roles": {role: dict(value or {}) for role, value in roles.items() if isinstance(value, Mapping)},
        }
    )


def project_camera_candidates(rows, *, discovery_active, status, network):
    candidates: list[dict[str, object]] = []
    for row in rows:
        if not isinstance(row, Mapping):
            continue
        interface = row.get("interface")
        candidates.append(
            {
                "kind": str(row.get("kind", "") or ""),
                "address": str(row.get("address", "") or ""),
                "host": str(row.get("host", "") or ""),
                "port": str(row.get("port", "") or ""),
                "status": str(row.get("status", "") or ""),
                "interface": str((interface or {}).get("name", "") or ""),
                "mac": str((interface or {}).get("mac", "") or ""),
                "host_ip": str(row.get("host_ip", "") or ""),
            }
        )
    return {
        "candidates": tuple(candidates),
        "discovery_active": discovery_active,
        "status": status,
        **network,
        "contract": "Network/RTSP/control endpoint metadata; no image coordinates or camera-frame values",
    }


def project_pixhawk_candidates(
    grouped, *, discovery_active, status, revision, pixhawk_assignments, assignment_status, network
):
    candidates: list[dict[str, object]] = []
    for source in ("network", "pc", "mavlink"):
        rows = grouped.get(source, ()) or ()
        for source_row, row in enumerate(rows):
            if not isinstance(row, Mapping):
                continue
            candidates.append(
                {
                    "source": source,
                    "source_row": source_row,
                    "kind": str(row.get("kind", "") or ""),
                    "value": str(row.get("value", "") or ""),
                    "resolved_device": str(row.get("resolved_device", "") or ""),
                    "baud": _integer(row.get("baud")),
                    "system_id": _integer(row.get("system_id")),
                    "component_id": _integer(row.get("component_id")),
                    "observed_monotonic_s": _number(row.get("observed_monotonic_s")),
                    "status": str(row.get("status", "") or ""),
                    "interface": str(row.get("interface", "") or ""),
                    "candidate_id": str(row.get("candidate_id", "") or ""),
                    "carrier": str(row.get("carrier", "") or ""),
                    "local_cidr": str(row.get("local_cidr", "") or ""),
                    "local_ip": str(row.get("local_ip", "") or ""),
                    "p1_route": str(row.get("p1_route", "") or ""),
                    "p2_route": str(row.get("p2_route", "") or ""),
                    "p1_route_state": str(row.get("p1_route_state", "") or ""),
                    "p2_route_state": str(row.get("p2_route_state", "") or ""),
                    "p1_endpoint": str(row.get("p1_endpoint", "") or ""),
                    "p2_endpoint": str(row.get("p2_endpoint", "") or ""),
                    "mac": str(row.get("mac", "") or ""),
                    "driver": str(row.get("driver", "") or ""),
                    "device_path": str(row.get("device_path", "") or ""),
                }
            )
    return {
        "candidates": tuple(candidates),
        "discovery_active": discovery_active,
        "status": status,
        "revision": revision,
        "assignments": {
            side: {
                **dict(pixhawk_assignments.get(side) or {}),
                **dict(assignment_status.get(side) or {}),
            }
            for side in ("left", "right")
        },
        **network,
        "contract": "Host/serial/MAVLink endpoint metadata; no attitude, position, or baseline values",
    }


def project_pixhawk_recovery(side, *, record, active, health, provenance, attitude, identities, usb_defaults, now):
    state = str(record.get("state", "IDLE") or "IDLE").upper()
    if active:
        state = "RUNNING"
    detail = str(record.get("detail", "") or "")
    updated_s = _number(record.get("updated_s"))
    health_state = str((health or {}).get("state", ""))
    qualification_required_s = float((health or {}).get("qualification_required_s", None) or MAVLINK_QUALIFICATION_S)
    qualification_text = f"{qualification_required_s:.0f}초"
    if not active and state == "COMMAND_SENT" and (health_state == "RECOVERING"):
        state = "QUALIFYING"
    elif not active and state in {"COMMAND_SENT", "QUALIFYING"} and (health_state == "LIVE"):
        state = "RECOVERED"
    if health is not None:
        attitude_age = _number((health or {}).get("attitude_rx_age_s", None))
        heartbeat_age = _number((health or {}).get("heartbeat_rx_age_s", None))
        attitude_ok = attitude_age is not None and attitude_age <= 2.0
        heartbeat_ok = heartbeat_age is not None and heartbeat_age <= 3.0
    else:
        attitude_age = _number(attitude.get("age_s"))
        heartbeat_age = None
        attitude_ok = (
            str(attitude.get("state", "WAITING")).upper() == "LIVE"
            and attitude_age is not None
            and (attitude_age < 2.0)
        )
        heartbeat_ok = bool(provenance.get("mavlink_heartbeat_seen", False))
    link_open = (
        bool((health or {}).get("listener_open", False))
        if health is not None
        else bool(provenance.get("mavlink_link_open", False))
    )
    heartbeat_state = "OK" if heartbeat_ok else "WAITING" if link_open else "NO LINK"
    attitude_state = "OK" if attitude_ok else "WAITING"
    expected_system_id = 1 if side == "left" else 2
    matching = [
        item
        for item in identities
        if isinstance(item, Mapping) and _integer(item.get("system_id")) == expected_system_id
    ]
    identity = matching[0] if len(matching) == 1 else {}
    usb_url = str(identity.get("device") or usb_defaults["url"] or "--")
    usb_baud = _integer(identity.get("baud"))
    if usb_baud is None:
        usb_baud = _integer(usb_defaults["baud"])
    if not detail:
        detail = {
            "IDLE": "대기",
            "RUNNING": "USB identity 재검증 및 one-shot reboot 진행 중",
            "COMMAND_SENT": "USB reboot command 전송; Ethernet packet 재개 대기",
            "QUALIFYING": f"Ethernet packet 재개; {qualification_text} qualification 진행 중",
            "RECOVERED": f"{qualification_text} qualification 통과",
            "CANCELLED": "operator 취소",
            "ERROR": "결과 없음",
        }.get(state, "결과 없음")
    return {
        "side": side,
        "usb_url": usb_url,
        "usb_baud": usb_baud,
        "usb_system_id": _integer(identity.get("system_id")),
        "usb_component_id": _integer(identity.get("component_id")),
        "usb_identity_count": len(matching),
        "state": state,
        "detail": detail,
        "updated_age_s": None if updated_s is None else max(0.0, now - updated_s),
        "active": active,
        "mavlink_link_open": link_open,
        "heartbeat_state": heartbeat_state,
        "attitude_state": attitude_state,
        "telemetry": f"Heartbeat {heartbeat_state} · ATTITUDE {attitude_state}",
        "mavlink_endpoint": str(provenance.get("mavlink_endpoint", "") or ""),
        "liveness_state": health_state,
        "qualification_elapsed_s": _number((health or {}).get("qualification_elapsed_s", None)),
        "qualification_required_s": qualification_required_s,
        "contract": f"unique current USB heartbeat sysid + operator confirmation + one-shot reboot; success requires Ethernet HB/ATT {qualification_required_s:.0f}-second qualification",
    }
