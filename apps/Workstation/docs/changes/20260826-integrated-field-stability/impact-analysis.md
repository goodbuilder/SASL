# Impact Analysis

## Preserved invariants

- Camera encoder/transport and PX4 firmware/parameters are unchanged without separate approval.
- One resource owner, bounded work, exact identities, append-only evidence and truthful status.
- Spatial coordinate contracts and calibration/triangulation math are unchanged.
- Existing data and manual recovery paths remain available.

## Affected layers and components

Potentially affected: camera/media/time workers, MAVLink/NTRIP/RTCM workers and reducers,
Measurement/recording/storage coordinators, existing status projections, launch/runtime checks,
tests and product documentation. Actual source edits are selected only after reproduction.

## Feature dependency, ownership, and coupling changes

No new ownership transfer is approved. The goal verifies existing explicit coordinators and removes
only observed hidden coupling: GUI image delivery to control/evidence, stale side to RTCM routing,
old persistence state to new Measurement admission, and requested decoder to assumed actual state.

## Worker CPU/GPU resource impact

Baseline and candidate use the same named workload. Track process/thread topology, hidden decoder/
inference threads, CPU/GPU/memory, queue depth/age/drop, throughput, UI latency and thermal state.
No affinity, priority, quality/rate or worker-count change is preapproved.

## Compatibility and migration

No schema/data migration. Current settings are read in place. A canonical venv rebuild may be an
operator deployment step but may not silently consume an app/System/Simulation fallback.

## Rollback and recovery

Each defect fix has a narrow regression and reversible patch. Device experiments require pre/post
readback and restoration. If rollback restores false success or unbounded behavior, the feature is
left visibly unavailable rather than silently degraded.

## Risks and assumptions

The camera sender may not supply missing reference payload/real IDR, and PX4/NuttX may deadlock its
Ethernet MAVLink task. Real field evidence can therefore prove an external blocker rather than yield
an app-only solution. Simultaneous high-bitrate recording may expose thermal/storage/resource faults
not seen by software tests.
