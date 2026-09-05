# Impact Analysis

## Preserved invariants

- Canonical measurement `baseline_center_frd` remains optical-center/camera-axis data.
- Raw P1/P2/Drone WGS84/MSL source values and selected P1/P2 receiver ownership remain unchanged.
- No GNSS value becomes Measurement readiness, RTK readiness, alignment acceptance, or persisted
  evidence merely because it is displayed.
- Existing scalar target distance, ideal disparity, connection, RTCM, settings, recording, and
  shutdown ownership remain in their current features.

## Affected layers and components

| Layer | Current owner | Proposed impact |
|---|---|---|
| Pure geometry | `workstation_core/ideal_disparity.py` | add/reuse explicit GNSS midpoint FRD domain contract |
| Read model | `DroneTelemetryController` | publish one immutable mapping from existing live states |
| UI | `TelemetryWorkspace` | fixed read-only group/table and status projection |
| Tests | `tests/test_workstation_telemetry.py` | independent sentinel, timing, snapshot, UI and regression cases |
| Docs | README + selected numbered UI docs + this packet | current operator/coordinate truth |

## Feature dependency, ownership, and coupling changes

Dependency remains one-way: Telemetry controller -> pure GNSS geometry -> Telemetry read model ->
Telemetry workspace. The widget does not read `latest_gps` or another feature's private state.
Ideal disparity may consume the same pure `Rg` so distance has one owner. Measurement, Alignment,
Recording, GNSS operation, and Pixhawk device owners gain no dependency on the new UI.

## Worker CPU/GPU resource impact

No worker, thread, queue, process, timer, I/O, GPU/CUDA operation, persistence, or allocation is
added. Each requested snapshot performs bounded constant-time arithmetic on three in-memory
positions. Existing visible-workspace refresh cadence owns projection; overload/fallback policy is
unchanged.

## Compatibility and migration

Additive runtime mapping and widget only. No setting/schema/file migration. Existing callers of
`target_distance_from_gps()` remain compatible. New mapping keys are ignored by older UI code.

## Rollback and recovery

Code rollback removes the pure read model and one UI group. No stored state or hardware action
requires reversal. Runtime recovery is automatic when three valid/fresh positions return.

## Risks and assumptions

- GNSS antenna midpoint is not optical-center midpoint; antenna/camera lever arms are unmodelled.
- Right uses only the horizontal P1->P2 direction; a near-vertical/zero horizontal baseline cannot
  define the frame and must remain WAITING.
- Down is local NED gravity-down, not a tilted camera/body Down axis.
- Forward is determined by `Right x Down`; it is not inferred from Drone direction, yaw, or camera
  attitude. P1/P2 assignment determines signs.
- Local tangent approximation and MSL altitude semantics remain the existing Telemetry limitation.
- The operator-selected 1000 ms three-source spread favors field availability over the stricter
  existing 200 ms P1/P2 baseline-quality threshold. Motion during that interval can bias F/R/D;
  actual skew remains visible, and this is not a configurable Measurement gate.
