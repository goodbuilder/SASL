"""Pure measurement-method readiness and fallback contracts.

The UI, optical-pose solvers, and runtime measurement owner all use these
stable method IDs.  Candidate/profile construction remains with its owning
controller; this module only normalizes configuration and makes deterministic
selection decisions from immutable readiness rows.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Mapping, Sequence


PIXHAWK_RTK_LIVE = "pixhawk_rtk_live"
PIXHAWK_RP_GPS_BASELINE_YAW = "pixhawk_rp_gps_baseline_yaw"
CHECKERBOARD_STEREO = "checkerboard_stereo"
HORIZON_HYBRID = "horizon_hybrid"
MOUNTAIN_GNSS = "mountain_gnss"
DRONE_RTK_ROTATION_HYBRID = "drone_rtk_rotation_hybrid"
DRONE_RTK_MULTIVIEW = "drone_rtk_multiview"
TERRAIN_PNP = "terrain_pnp"
COMBINED_HYBRID = "combined_hybrid"
CLOUD_GNSS = "cloud_gnss"

MEASUREMENT_METHOD_IDS: tuple[str, ...] = (
    PIXHAWK_RTK_LIVE,
    PIXHAWK_RP_GPS_BASELINE_YAW,
    CHECKERBOARD_STEREO,
    HORIZON_HYBRID,
    MOUNTAIN_GNSS,
    DRONE_RTK_ROTATION_HYBRID,
    DRONE_RTK_MULTIVIEW,
    TERRAIN_PNP,
    COMBINED_HYBRID,
    CLOUD_GNSS,
)

DEFAULT_MEASUREMENT_METHOD_FALLBACK_ORDER = MEASUREMENT_METHOD_IDS

MEASUREMENT_POSE_SIDES: tuple[str, ...] = ("left", "right")
MEASUREMENT_ROTATION_DOFS: tuple[str, ...] = ("roll_deg", "pitch_deg", "yaw_deg")
MEASUREMENT_TRANSLATION_DOFS: tuple[str, ...] = ("north_m", "east_m", "down_m")
MEASUREMENT_POSE_DOFS: tuple[str, ...] = (
    *MEASUREMENT_ROTATION_DOFS,
    *MEASUREMENT_TRANSLATION_DOFS,
)
MEASUREMENT_FILL_NONE = "none"
MEASUREMENT_FILL_LIVE_PIXHAWK = "live_pixhawk"
MEASUREMENT_FILL_LIVE_GNSS = "live_gnss"

MEASUREMENT_METHOD_LABELS: Mapping[str, str] = {
    PIXHAWK_RTK_LIVE: "Pixhawk attitude + GPS/RTK",
    PIXHAWK_RP_GPS_BASELINE_YAW: "Pixhawk R/P + GPS baseline yaw (90°)",
    CHECKERBOARD_STEREO: "Checkerboard stereo R,t",
    HORIZON_HYBRID: "Horizon R/P + live Pixhawk/GNSS",
    MOUNTAIN_GNSS: "Mountain + GNSS",
    DRONE_RTK_ROTATION_HYBRID: "Drone RTK Rotation + live GNSS",
    DRONE_RTK_MULTIVIEW: "Drone RTK Multi-view R,t",
    TERRAIN_PNP: "Terrain PnP",
    COMBINED_HYBRID: "Combined candidate",
    CLOUD_GNSS: "Cloud far-field rotation + live GNSS",
}

MEASUREMENT_METHOD_REQUIRED_INPUTS: Mapping[str, tuple[str, ...]] = {
    PIXHAWK_RTK_LIVE: (
        "exact operating K,D",
        "fresh left/right Pixhawk attitude",
        "usable P1→P2 GNSS NED baseline",
    ),
    PIXHAWK_RP_GPS_BASELINE_YAW: (
        "exact operating K,D",
        "left/right Pixhawk Roll/Pitch",
        "usable P1→P2 GNSS NED baseline",
        "P1/P2 Pixhawk yaw reference or persistent Reverse branch",
    ),
    CHECKERBOARD_STEREO: (
        "exact operating K,D",
        "checkerboard stereo left→right R,t profile",
    ),
    HORIZON_HYBRID: (
        "Horizon Roll/Pitch candidate",
        "configured Pixhawk rotation fill",
        "configured GNSS translation fill",
    ),
    MOUNTAIN_GNSS: (
        "Mountain + GNSS full 6-DOF candidate",
        "exact operating K,D",
    ),
    DRONE_RTK_ROTATION_HYBRID: (
        "Drone RTK rotation candidate",
        "configured Pixhawk/GNSS fill",
    ),
    DRONE_RTK_MULTIVIEW: (
        "Drone RTK multi-view full 6-DOF candidate",
        "exact operating K,D",
    ),
    TERRAIN_PNP: (
        "Terrain PnP full 6-DOF candidate",
        "exact operating K,D",
    ),
    COMBINED_HYBRID: (
        "compatible non-overlapping candidate set",
        "combined full 6-DOF candidate",
    ),
    CLOUD_GNSS: (
        "exact operating K,D",
        "Cloud far-field rotation candidate",
        "configured GNSS translation fill",
    ),
}

MEASUREMENT_METHOD_MISSING_INSTANCE_REASONS: Mapping[str, str] = {
    PIXHAWK_RTK_LIVE: "live Pixhawk/RTK method status unavailable",
    PIXHAWK_RP_GPS_BASELINE_YAW: "live Pixhawk R/P or GPS baseline yaw branch unavailable",
    CHECKERBOARD_STEREO: "checkerboard method status unavailable",
    HORIZON_HYBRID: "no Horizon candidate or compatible loaded profile",
    MOUNTAIN_GNSS: "no Mountain + GNSS candidate or compatible loaded profile",
    DRONE_RTK_ROTATION_HYBRID: "no Drone RTK Rotation candidate or compatible loaded profile",
    DRONE_RTK_MULTIVIEW: "no Drone RTK Multi-view candidate or compatible loaded profile",
    TERRAIN_PNP: "no Terrain PnP candidate or compatible loaded profile",
    COMBINED_HYBRID: "no Combined candidate or compatible loaded profile",
    CLOUD_GNSS: "no Cloud far-field rotation candidate or compatible loaded profile",
}

_LEGACY_METHOD_IDS: Mapping[str, str] = {
    "pixhawk": PIXHAWK_RTK_LIVE,
    "checkerboard": CHECKERBOARD_STEREO,
    "calibration": CHECKERBOARD_STEREO,
}

_CANDIDATE_METHOD_CATEGORIES: tuple[tuple[str, str], ...] = (
    ("flat-horizon-roll-pitch", HORIZON_HYBRID),
    ("spherical-horizon-roll-pitch", HORIZON_HYBRID),
    ("gnss-constrained-mountain-uv", MOUNTAIN_GNSS),
    ("drone-rtk-target-yaw", DRONE_RTK_ROTATION_HYBRID),
    ("drone-rtk-multiview-r-t", DRONE_RTK_MULTIVIEW),
    ("terrain-pnp-ensemble", TERRAIN_PNP),
    ("combined-independent-axis-evidence", COMBINED_HYBRID),
    ("cloud-far-field-rotation", CLOUD_GNSS),
)


def normalize_measurement_method_id(value: object) -> str:
    """Return one canonical method ID, preserving the legacy defaults."""

    text = str(value or "").strip().lower()
    text = _LEGACY_METHOD_IDS.get(text, text)
    return text if text in MEASUREMENT_METHOD_IDS else PIXHAWK_RTK_LIVE


def measurement_method_id_for_candidate(method: object) -> str | None:
    """Map an optical-pose solver/profile method to a Measurement category."""

    text = str(method or "").strip().lower()
    if text.startswith("optical_pose/"):
        text = text.removeprefix("optical_pose/")
    for prefix, method_id in _CANDIDATE_METHOD_CATEGORIES:
        if text == prefix or text.startswith(prefix + "/"):
            return method_id
    return None


def normalize_measurement_method_fallback_order(values: object) -> tuple[str, ...]:
    """Sanitize an ordered settings value and append newly-added methods.

    Unknown and duplicate values are ignored.  Missing known values are
    appended in the product default order, making settings migration forward
    compatible without losing the operator's existing relative order.
    """

    if isinstance(values, str):
        raw: Iterable[object] = (values,)
    elif isinstance(values, Iterable):
        raw = values
    else:
        raw = ()
    ordered: list[str] = []
    for value in raw:
        text = str(value or "").strip().lower()
        text = _LEGACY_METHOD_IDS.get(text, text)
        if text in MEASUREMENT_METHOD_IDS and text not in ordered:
            ordered.append(text)
    ordered.extend(
        method_id
        for method_id in DEFAULT_MEASUREMENT_METHOD_FALLBACK_ORDER
        if method_id not in ordered
    )
    return tuple(ordered)


def measurement_fill_component_key(side: object, dof: object) -> str:
    side_text = str(side).strip().lower()
    dof_text = str(dof).strip().lower()
    if side_text not in MEASUREMENT_POSE_SIDES:
        raise ValueError(f"invalid measurement pose side: {side_text}")
    if dof_text not in MEASUREMENT_POSE_DOFS:
        raise ValueError(f"invalid measurement pose DOF: {dof_text}")
    return f"{side_text}.{dof_text}"


def compatible_measurement_fill_source(dof: object) -> str:
    dof_text = str(dof).strip().lower()
    if dof_text in MEASUREMENT_ROTATION_DOFS:
        return MEASUREMENT_FILL_LIVE_PIXHAWK
    if dof_text in MEASUREMENT_TRANSLATION_DOFS:
        return MEASUREMENT_FILL_LIVE_GNSS
    raise ValueError(f"invalid measurement pose DOF: {dof_text}")


def default_measurement_method_fill_policy() -> dict[str, dict[str, str]]:
    """Return safe per-method defaults for every potentially missing DOF."""

    return {
        method_id: {
            measurement_fill_component_key(side, dof): compatible_measurement_fill_source(dof)
            for side in MEASUREMENT_POSE_SIDES
            for dof in MEASUREMENT_POSE_DOFS
        }
        for method_id in MEASUREMENT_METHOD_IDS
    }


def normalize_measurement_method_fill_policy(
    values: object,
) -> dict[str, dict[str, str]]:
    """Normalize one Measurement-owned DOF fill policy.

    Missing entries receive the safe defaults. Explicit incompatible/unknown
    values fail closed to ``none`` instead of silently selecting a sensor.
    """

    defaults = default_measurement_method_fill_policy()
    raw_methods = values if isinstance(values, Mapping) else {}
    result: dict[str, dict[str, str]] = {}
    for method_id in MEASUREMENT_METHOD_IDS:
        raw_components = raw_methods.get(method_id)
        components = raw_components if isinstance(raw_components, Mapping) else {}
        normalized: dict[str, str] = {}
        for side in MEASUREMENT_POSE_SIDES:
            for dof in MEASUREMENT_POSE_DOFS:
                key = measurement_fill_component_key(side, dof)
                if key not in components:
                    normalized[key] = defaults[method_id][key]
                    continue
                value = str(components.get(key) or MEASUREMENT_FILL_NONE).strip().lower()
                compatible = compatible_measurement_fill_source(dof)
                normalized[key] = value if value in {MEASUREMENT_FILL_NONE, compatible} else MEASUREMENT_FILL_NONE
        result[method_id] = normalized
    return result


@dataclass(frozen=True)
class MeasurementMethodStatus:
    """One immutable method instance projected into readiness UI/runtime."""

    method_id: str
    instance_id: str
    ready: bool
    reasons: tuple[str, ...] = ()
    candidate_id: str = ""
    profile_id: str = ""
    geometry_sha256: str = ""
    created_utc_ns: int = 0
    observed_dofs: tuple[str, ...] = ()
    component_sources: tuple[tuple[str, str], ...] = ()
    warnings: tuple[str, ...] = ()
    requires_live_pixhawk: bool = False
    requires_live_gnss: bool = False

    def __post_init__(self) -> None:
        method_id = normalize_measurement_method_id(self.method_id)
        object.__setattr__(self, "method_id", method_id)
        object.__setattr__(self, "instance_id", str(self.instance_id or method_id))
        object.__setattr__(self, "ready", bool(self.ready))
        object.__setattr__(self, "reasons", tuple(str(value) for value in self.reasons))
        object.__setattr__(self, "candidate_id", str(self.candidate_id))
        object.__setattr__(self, "profile_id", str(self.profile_id))
        object.__setattr__(self, "geometry_sha256", str(self.geometry_sha256))
        object.__setattr__(self, "created_utc_ns", max(0, int(self.created_utc_ns)))
        object.__setattr__(self, "observed_dofs", tuple(str(value) for value in self.observed_dofs))
        object.__setattr__(
            self,
            "component_sources",
            tuple((str(name), str(source)) for name, source in self.component_sources),
        )
        object.__setattr__(self, "warnings", tuple(str(value) for value in self.warnings))
        object.__setattr__(self, "requires_live_pixhawk", bool(self.requires_live_pixhawk))
        object.__setattr__(self, "requires_live_gnss", bool(self.requires_live_gnss))
        if self.ready and self.reasons:
            raise ValueError("ready measurement methods must not contain blocking reasons")

    @property
    def label(self) -> str:
        return str(MEASUREMENT_METHOD_LABELS[self.method_id])

    def to_mapping(self) -> dict[str, object]:
        return {
            "method_id": self.method_id,
            "label": self.label,
            "instance_id": self.instance_id,
            "ready": self.ready,
            "reasons": list(self.reasons),
            "candidate_id": self.candidate_id,
            "profile_id": self.profile_id,
            "geometry_sha256": self.geometry_sha256,
            "source_identity_sha256": self.geometry_sha256,
            "created_utc_ns": self.created_utc_ns,
            "observed_dofs": list(self.observed_dofs),
            "component_sources": dict(self.component_sources),
            "warnings": list(self.warnings),
            "requires_live_pixhawk": self.requires_live_pixhawk,
            "requires_live_gnss": self.requires_live_gnss,
        }


def latest_status_by_method(
    statuses: Sequence[MeasurementMethodStatus],
) -> dict[str, MeasurementMethodStatus]:
    """Keep the newest deterministic instance for every method category."""

    latest: dict[str, MeasurementMethodStatus] = {}
    for status in statuses:
        current = latest.get(status.method_id)
        if current is None or (status.ready, status.created_utc_ns, status.instance_id) > (
            current.ready,
            current.created_utc_ns,
            current.instance_id,
        ):
            latest[status.method_id] = status
    return latest


def choose_ready_measurement_method(
    statuses: Sequence[MeasurementMethodStatus],
    *,
    preferred_method_id: object,
    fallback_order: object,
) -> MeasurementMethodStatus | None:
    """Select preferred when ready, otherwise the highest-ranked ready row."""

    latest = latest_status_by_method(statuses)
    ready = {method_id: row for method_id, row in latest.items() if row.ready}
    preferred = normalize_measurement_method_id(preferred_method_id)
    if preferred in ready:
        return ready[preferred]
    for method_id in normalize_measurement_method_fallback_order(fallback_order):
        if method_id in ready:
            return ready[method_id]
    return None


__all__ = [
    "CHECKERBOARD_STEREO",
    "CLOUD_GNSS",
    "COMBINED_HYBRID",
    "DEFAULT_MEASUREMENT_METHOD_FALLBACK_ORDER",
    "DRONE_RTK_MULTIVIEW",
    "DRONE_RTK_ROTATION_HYBRID",
    "HORIZON_HYBRID",
    "MEASUREMENT_METHOD_IDS",
    "MEASUREMENT_METHOD_LABELS",
    "MEASUREMENT_METHOD_MISSING_INSTANCE_REASONS",
    "MEASUREMENT_METHOD_REQUIRED_INPUTS",
    "MEASUREMENT_FILL_LIVE_GNSS",
    "MEASUREMENT_FILL_LIVE_PIXHAWK",
    "MEASUREMENT_FILL_NONE",
    "MEASUREMENT_POSE_DOFS",
    "MEASUREMENT_POSE_SIDES",
    "MEASUREMENT_ROTATION_DOFS",
    "MEASUREMENT_TRANSLATION_DOFS",
    "MOUNTAIN_GNSS",
    "MeasurementMethodStatus",
    "PIXHAWK_RTK_LIVE",
    "PIXHAWK_RP_GPS_BASELINE_YAW",
    "TERRAIN_PNP",
    "choose_ready_measurement_method",
    "compatible_measurement_fill_source",
    "default_measurement_method_fill_policy",
    "latest_status_by_method",
    "measurement_method_id_for_candidate",
    "normalize_measurement_method_fallback_order",
    "normalize_measurement_method_fill_policy",
    "normalize_measurement_method_id",
    "measurement_fill_component_key",
]
