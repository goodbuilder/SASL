# Current Behavior

## Version and worktree context

- Baseline commit: `c2daf81`.
- The worktree contains extensive pre-existing user changes, including `README.md`, `evidence.py`,
  `workspaces.py`, product UI documents, and Pixhawk tests. This packet must preserve them and use
  narrow patches only.

## Reproduction

1. Launch the Workstation and open `Devices > Pixhawk`.
2. Read P1/P2 yaw in `Attitude (R/P/Y)`.
3. Scroll to `P1→P2 baseline` and read NED plus P1-body FRD values.
4. Observe that no visible field or plot combines baseline bearing with P1/P2 yaw.

## Observed frontend behavior

The Pixhawk workspace has a two-row attitude table and an eight-value baseline group. The current
Alignment workspace owns optical-pose R,t candidates, not physical baseline/yaw presentation. The
standalone owner constructs an unexposed `Baseline / 자세 정렬` plot, but it computes an unsigned
3D Forward-to-baseline angle and is not part of the visible Workstation workspace.

## Observed backend and persistence behavior

`EvidenceAdapter.attitude()` exposes P1/P2 yaw, quaternion, source, clock, and age.
`EvidenceAdapter.baseline()` exposes the selected P1→P2 NED vector, P1 body-FRD projection,
quality, fix types, sources, and ages. Measurement recording already writes raw `pixhawk_attitude`
rows with yaw/quaternion/timestamps and `gps_rtk` rows with receiver role, WGS84 position, fix, and
timestamps, so the approved derived angle can be reconstructed offline.

## Documentation, code, test, and runtime conflicts

Product documents require value/unit/source/age/frame clarity and preserve raw NED separately from
derived FRD. No product document currently specifies a visible signed horizontal baseline/yaw
read model. Hardware visual behavior remains unverified until a real P1/P2/GNSS run.
