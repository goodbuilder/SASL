"""Network role status presentation shared by Camera and Pixhawk."""

from __future__ import annotations

from collections.abc import Mapping
from typing import Sequence


def _network_role_column_text(labels: Sequence[str]) -> str:
    """Render the saved roles one NIC row satisfies for a candidate table."""

    if len(labels) > 1:
        return f"{', '.join(labels)} (shared NIC)"
    return labels[0] if labels else "--"


def _apply_network_role_discovery_status(
    status_widget: object,
    connection: Mapping[str, object],
    *,
    default_status: str = "NOT RUN",
) -> tuple[str, str]:
    """Show discovery status, overridden while a network-role operation runs.

    Returns the role state and its detail so callers can add their own feedback.
    """

    role_status = connection.get("network_role_status")
    role_status = role_status if isinstance(role_status, Mapping) else {}
    role_state = str(role_status.get("state") or "IDLE")
    detail = str(role_status.get("detail") or "")
    discovery_status = str(connection.get("status", default_status) or default_status)
    if role_state != "IDLE":
        discovery_status = f"NETWORK ROLE {role_state}"
    status_widget.set_value(
        discovery_status,
        source=detail if role_state != "IDLE" else "",
        valid=False if discovery_status.startswith("ERROR") else None,
    )
    return role_state, detail
