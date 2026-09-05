# Implementation Plan

## Approval status

Approved by the user's explicit CSV-storage request on 2026-09-04. This implementation covers the
stream-wide sessionless raw mirrors and directly coupled Data Status projection; curated quick-view
tables and historical backfill remain deferred unless separately requested.

## Slices and dependencies

Completed slice status: contract, persistence, controller/read-model, and Data Status projection
are implemented and focused-verified. Documentation is updated except for the workspace README,
whose target path still requires confirmation under the README skill procedure.

1. Contract: add the stream-to-CSV mapping, fixed envelope, safe header validation, canonical row
   encoder, and the quick-find catalog/view contracts; focused pure tests must pass first.
2. Persistence: add owner-local CSV handles, independent state/retry/flush/close behavior, and
   restart append; fresh/failure/recovery tests must pass before UI changes.
3. Controller/read model: expose paired CSV paths, mirror evidence, and quick-view/catalog paths
   through the existing storage snapshot without adding a second owner.
4. UI: update the existing Data Status table in place; verify paths, states, long errors, geometry,
   focus, selection, and Measurement enablement.
5. Documentation and integration: update the confirmed README/schema/UI docs, run portability and
   relevant launcher checks, and record all evidence.

## File and component map

- `apps/Workstation/workstation_core/measurement_data_store.py`
- `apps/Workstation/workstation_core/measurement_csv_views.py` (if the quick-view projection stays
  separate from the live storage owner)
- `apps/Workstation/data_storage_controller.py`
- `apps/Workstation/workspaces.py`
- `apps/Workstation/standalone_gui.py` only if its public delegate typing/contract requires it
- `apps/Workstation/tests/test_measurement_data_store.py`
- `apps/Workstation/tests/test_workstation_ui.py` and/or the narrow existing Data workspace test
- `apps/Workstation/docs/RECORDING_SCHEMA.md`
- `apps/Workstation/docs/ui/01_USER_WORKFLOW.md`, `03_WORKSPACE_LAYOUT.md`, `06_STATUS_MODEL.md`,
  `07_COMMAND_MODEL.md`, `ui/README.md`
- README target is pending explicit confirmation under the `update-readme` procedure.

## Migration and rollback steps

No destructive migration. New writes create sidecars; old JSONL remains valid. On rollback, leave
CSV files in place as optional artifacts and continue reading JSONL/XLSX. A conflicting CSV is
reported rather than overwritten.

## Stopping conditions

Stop and return for approval if implementation requires changing Measurement admission, adding a
second storage owner, rewriting historical JSONL, adding an unbounded queue, changing coordinate or
clock fields, or expanding to aggregate/legacy Session exports. Stop verification if a required
JSONL path or next-run availability regresses.
