# Implementation Plan

## Approval status

Approved by the operator on 2026-09-02. Implementation is authorized only for the documented
derived JSONL/XLSX persistence slice.

## Slices and dependencies

1. Schema/stream: fixed key/path, row builder contract and independent sentinel/WAITING fixtures.
2. Trigger: RecordingController public method and primary-Drone-only Sensor handoff; recording gate,
   duplicate GPS2, sequence and writer-admission failure tests.
3. Storage: bulk append/restart/failure/Data Status/checkpoint tests.
4. Latest XLSX: sheet order, latest-generation filtering, zero/old rows, XML/identity validation.
5. Docs/operations: README, recording schema, UI status/workflow/implementation/audit docs.
6. Verification: focused after every slice, affected storage/recording/Telemetry/XLSX/data UI,
   portability/runtime, full suite and known environment residual reporting.

## File and component map

- `workstation_core/measurement_data_store.py`: stream key/path.
- `recording_controller.py`: record builder/admission and recording sequence.
- `sensor_presentation_controller.py`: explicit primary Drone event handoff.
- `standalone_gui.py`: initial counter/delegate generation if required.
- `workstation_core/latest_measurement_xlsx.py`: sheet/discovery spec.
- Tests: measurement data store, recording controller/raw artifacts, Telemetry, latest XLSX, Data UI.
- Docs: `README.md`, `docs/RECORDING_SCHEMA.md`, relevant numbered UI docs and this packet.

## Migration and rollback steps

No backfill or destructive migration. Rollback stops new rows/sheet; existing additive JSONL remains
untouched. Older app versions ignore it.

## Stopping conditions

- Requirement to place values in canonical Measurement fields or legacy Parquet without approval.
- Need for new worker/queue, priority/retry semantics, recording admission gate or UI command.
- Cannot keep one-row-per-primary-Drone trigger or <=4 KiB row budget.
- Existing writer/XLSX failure isolation would be weakened.
