# Impact Analysis

## Preserved invariants

- Raw P1→P2 NED remains authoritative and separate from P1 body FRD and camera RDF.
- PX4 ATTITUDE retains body-FRD-to-world-NED semantics and original raw display.
- P1/P2 sources, links, workers, sockets, commands, measurement, and recording retain one owner.
- Missing data is never rendered as zero; stale data never appears current.

## Affected layers and components

- Pure geometry: horizontal azimuth and wrapped signed angle calculation.
- Evidence: one read-only composite snapshot using existing attitude/baseline/GPS owners.
- Presentation: Pixhawk workspace summary and fixed-height top view.
- Documentation/tests: Workstation README, UI product contracts, focused and regression tests.

## Feature dependency, ownership, and coupling changes

The Pixhawk presentation gains a dependency on a physical-alignment pure value type and its own
EvidenceAdapter public read model. No widget reads another feature's widget/private state and no
ownership moves.

## Worker CPU/GPU resource impact

No worker, queue, thread, I/O, GPU, or accelerator change. Per visible 4 Hz poll, work is bounded to
a few finite checks, `atan2`, angle wrapping, label replacement, and one small widget repaint.

## Compatibility and migration

No schema, setting, file, command, protocol, or migration. Existing recordings reconstruct the
derived angles from raw yaw/quaternion and GNSS position/timestamp rows.

## Rollback and recovery

Rollback removes the new read model/group/plot and documentation without data conversion. Device
disconnect/reconnect recovery stays owned by existing Pixhawk/GNSS status logic.

## Risks and assumptions

The live display combines latest-state samples rather than asserting simultaneous exposure. It is
for stationary/slow physical alignment and must show source ages/pair delta. Hardware plot behavior
and field readability remain manual evidence until tested on the rig.
