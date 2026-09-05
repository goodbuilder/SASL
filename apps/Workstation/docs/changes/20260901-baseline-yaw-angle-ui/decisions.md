# Decisions

Append accepted decisions chronologically. Supersede entries rather than deleting history.

## 2026-09-01 — Signed horizontal angles for both Pixhawks

Show `wrap180(P1→P2 horizontal azimuth - yaw)` independently for P1 and P2, plus P2-minus-P1
yaw. This answers left/right alignment directly and does not substitute the older unsigned 3D aid.

## 2026-09-01 — Observation only

Do not add a +90-degree target, tolerances, or GOOD/WARN/BAD physical-alignment verdict. Source
freshness/availability states remain required evidence, not an alignment quality judgment.

## 2026-09-01 — Pixhawk location with numeric plus top view

Place the new group immediately below raw `P1→P2 baseline`; show compact numbers and a north-based
top view. Do not add a new navigation context or mix it into Alignment R,t candidates.

## 2026-09-01 — Live snapshot only and no derived persistence

No angle history/statistics and no new saved field. Existing raw PX4 attitude and GNSS records are
sufficient for offline reconstruction; preserve their schemas.

## 2026-09-01 — README target and implementation approval

Update `apps/Workstation/README.md`. The user approved the complete proposed plan and explicitly
requested implementation.

## 2026-09-01 — Separate Baseline−P1 and Baseline−P2 fields

The operator reconfirmed that the two relative degrees must be separately visible. Each row keeps
its own ATTITUDE age/state so loss of one Pixhawk does not hide the other comparison.

## 2026-09-03 — Supersede Drone ATTITUDE amendment with GNSS installation angle

The operator clarified that Drone Pixhawk ATTITUDE/yaw is not used. The required observation is the
acute horizontal angle between the P1→P2 GNSS baseline and the vector from the P1/P2 GNSS midpoint
to the Drone GNSS position. Use the existing GNSS `Forward/Right` components:
`angle=atan2(|Forward|,|Right|)`, with 90° as the camera-installation target and
`|90°−angle|` as the displayed error. Move the feature to `Devices > Telemetry`; remove the
Drone-yaw fields/arrow/read-model dependency. Preserve F/R/D/Rg, freshness, raw data, commands,
settings, persistence, and measurement semantics.
