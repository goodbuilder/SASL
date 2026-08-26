# Impact Analysis

## Preserved invariants

- Camera, measurement, calibration, PX4, GNSS/RTK, RTCM delivery, and coordinate math are unchanged.
- One live owner remains mandatory for camera/video, device sockets, and logger commands.
- Existing evidence is never deleted, migrated, relabeled, or rewritten.
- Stored coordinate/frame/unit fields retain their producer contracts.

## Ownership change

Before, `MeasurementSession -> MeasurementPackage` owns paths, mutable state, required artifacts,
finalization, recovery, review, and the next-start gate. After, one application data writer owns
fixed type streams only; Measurement owns transient camera/detection state and no persisted object
owns admission. Calibration, Alignment, capture, ULog, Target, and RTCM use explicit data-root
paths/contracts rather than a Session root.

## Affected boundaries

- Navigation, Measurement/Data/Overview/Inspector/status presentation and commands.
- Backend facade, controller delegates, startup/shutdown, recording and late-result routing.
- Settings, path portability, structured streams, binary filenames, RTCM evidence, and legacy read
  isolation.
- Tests and product/operator documentation.

## Worker and overload impact

Current runtime uses synchronous spool writes plus SessionSaveWorker and video verification.
Proposed runtime removes those finalization workers and adds one bounded CPU append writer. Native
frame capacity stays 16; GPU/model workers and library thread caps remain unchanged. Priority
results/events use 256 slots; bulk telemetry uses 4096. Bulk drops first, every drop is visible,
and no persistence path applies backpressure to measurement computation.

## Compatibility and rollback

Existing direct and legacy Session directories stay where they are and are not scanned at startup.
Legacy parser modules may remain for offline use, but GUI/file creation no longer calls them. New
data lives under `measurement_data_root`; rolling back code leaves those files intact and the prior
app simply ignores them. No schema or evidence migration is performed.

## Risks

- Cross-type completeness is intentionally unavailable; manual time selection can omit or mix data.
- Fixed files grow without rotation; operators move/copy only after closing Workstation.
- Best-effort persistence can lose rows during disk failure or overload, reported by type/count.
- Immediate successive runs may briefly overlap retiring method/native work; generation isolation
  prevents old callbacks from changing current UI state.
- Real camera teardown remains the only start delay that can outlive a command click.
