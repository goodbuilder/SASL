# Implementation Plan

## Approval status

Approved at `2026-08-26T12:36:43+09:00` for the sessionless availability-first vertical slice.

## Slice 1 — Data contract and writer

- Add one data-root-relative `measurement_data_root` setting, fixed stream paths, collision-free
  creation-time binary allocation, and one app-owned background writer.
- Reserve a 256-record priority queue for results/events and a 4096-record bulk queue for camera,
  PX4, GNSS, RTCM, and logs. Never block producers; expose per-stream failure/drop evidence.
- Append independently timestamped JSONL without scanning historical contents, preserve malformed
  historical bytes, and retry failed streams on command or the next capture start.
- Verify append/no-overwrite, restart, malformed-tail, individual failure, overload priority,
  bounded shutdown, path containment, timestamps, and coordinate fields.

## Slice 2 — Transient capture lifecycle

- Replace Session/package states with `IDLE/QUEUED/STARTING/RUNNING/STOPPING/ERROR` runtime state.
- Remove startup Session discovery/adoption and every persisted state/save/completeness start gate.
- Queue start behind actual camera work. Manual record is stopped before a queued Measurement and
  is not resumed. Storage failure does not affect command acceptance.
- Stop waits only for camera/detection/video ownership release; other producer results may finish
  late and cannot mutate the current generation.

## Slice 3 — Producer boundaries

- Route measurement, method bundle, camera, PX4, GNSS, capture, Alignment, application log, and
  artifact records to the typed writer.
- Make the native-frame store app-owned and non-blocking across runs.
- Decouple RTCM delivery/injection from best-effort evidence append and remove package drain gates.
- Remove Session prerequisites from HR/OZ, ULog import, Target Drone, Calibration aggregation, and
  Alignment persistence while preserving their real device ownership rules.

## Slice 4 — Facade and GUI

- Remove live Session/package/save/revision facade methods; add measurement-run and data-storage
  snapshots plus retry storage.
- Replace Sessions with Data; preserve existing dense components and relocate HR/OZ, ULog, Target,
  storage health, and file-manager actions.
- Remove Create/Close Session, Retry Save, completeness, revision, verification, and saved review.
- Keep Measurement and Manual record commands source-backed; never disable Measurement for storage
  or legacy evidence state. Keep dynamic text bounded and focus/selection/scroll stable.

## Slice 5 — Compatibility, docs, and verification

- Leave existing recordings untouched; keep legacy format parsers offline but detach them from GUI
  startup/runtime discovery and all new writes.
- Update the numbered UI documents, workflow, recording schema, and approved Workstation README.
- Run focused tests after every slice, then the full Workstation suite, offscreen UI suite,
  portability checker, runtime check, and documented hardware/manual scenarios.

## Stopping conditions

- Stop for renewed approval if implementation needs to delete/migrate existing evidence, change
  measurement algorithms or coordinate frames, alter device protocols, or make storage a start gate.
- Storage/media/telemetry failures are not stopping conditions; isolate, expose, and test them.
