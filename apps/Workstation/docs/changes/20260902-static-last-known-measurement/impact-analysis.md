# Impact Analysis

## Preserved invariants

- OpenCV optical RDF camera rays, PX4 body FRD attitude, WGS84 positions, P1-to-P2
  world-NED baseline, and public baseline-center FRD output remain distinct.
- Existing live behavior remains the default until the retained-static workflow is armed.
- Never manufacture a value for a side that has no valid observation.

## Affected layers and components

- Measurement readiness/input freezing: `measurement_view_controller.py`.
- Pure method/readiness contracts and evidence schemas as required.
- Ready-method/status presentation in `workspaces.py`.
- Focused measurement-method, attempt, evidence, and UI tests.
- `docs/ui/01_USER_WORKFLOW.md`, `06_STATUS_MODEL.md`, and `07_COMMAND_MODEL.md` after approval.

## Feature dependency, ownership, and coupling changes

The Measurement owner may retain immutable copies of samples already owned by the
Pixhawk/GNSS receivers. It must not take ownership of device connections or mutate
telemetry caches.

## Worker CPU/GPU resource impact

None expected; this is bounded state selection and evidence, with no new worker or queue.

## Compatibility and migration

To be decided. Safe default is opt-in with no automatic migration of prior settings.

## Rollback and recovery

Disable retained-static mode and return to the existing live-only readiness path.

## Risks and assumptions

The primary risk is silently combining old sensor geometry with a moved rig or current
images. The operator has rejected visible mode/age and added retained provenance. Exact existing
source timestamps must therefore remain the minimum non-falsified evidence boundary, while
reset-on-restart is the only explicit clearing rule.
