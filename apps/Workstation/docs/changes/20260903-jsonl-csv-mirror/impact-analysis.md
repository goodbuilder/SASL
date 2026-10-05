# Impact Analysis

## Preserved invariants

- JSONL remains the sole authoritative structured-record source and is never rewritten.
- One storage owner remains responsible for the fixed stream files; producers stay non-blocking.
- Existing priority/bulk queue and essential retry semantics remain unchanged for JSONL.
- Measurement admission, device/camera/GNSS ownership, coordinate frames, clocks, and latest XLSX
  generation remain unchanged.
- CSV is a same-stream convenience mirror, not a merged time series and not a verdict.

## Affected layers and components

| Layer | Owner/component | Change | Oracle/rollback |
| --- | --- | --- | --- |
| Contract | `measurement_data_store.py` | Add same-stem CSV path/header/envelope contract | independent `csv.DictReader` plus JSON parse |
| Persistence | `MeasurementDataStore` | Append CSV after JSONL, track mirror health/retry | row/`record_json` comparison; disable mirror code |
| Controller | `DataStorageController` | Preserve public storage/retry facade while exposing CSV snapshot fields | controller snapshot tests |
| UI | `RecordingWorkspace` | Show paired CSV path/state in existing Data table | Qt model/header/state tests |
| Docs | Workstation README/schema/UI docs | Document files, authority, failure, and usage | path/contract review |

## Feature dependency, ownership, and coupling changes

The Data Storage feature remains the only owner of both JSONL and CSV stream files. No new access to
camera, Pixhawk, GNSS, Measurement, or XLSX internals is introduced. The UI reads the public storage
snapshot only. CSV failure is isolated to storage presentation and never propagates into source
availability or Measurement start.

## Worker CPU/GPU resource impact

No GPU, camera, device, or new high-rate worker is added. The existing single storage writer performs
one additional bounded sequential CSV append per JSONL row and retains one bounded CSV retry reserve.
JSONL/priority work remains first. CSV retry is best effort, capped, and may drop with explicit
evidence under sustained write failure. Acceptance measures producer latency, queue/drop counts, and
writer shutdown; no fixed utilization percentage is introduced.

## Compatibility and migration

Existing JSONL files remain readable and untouched. New stores create CSV headers on first use.
Pre-existing same-stem CSV files with the expected Workstation header may be appended; a symlink,
directory, malformed header, or foreign file is never overwritten and reports a CSV failure. This
slice does not backfill historical JSONL into CSV and does not alter old Session/Legacy artifacts.

## Rollback and recovery

If CSV persistence proves harmful, remove/disable the mirror path while retaining all JSONL files;
existing JSONL and XLSX workflows continue. During runtime, `Retry Storage` reopens failed CSV
handles after the operator fixes the destination. No destructive cleanup is performed automatically.

## Risks and assumptions

- CSV doubles write work for each mirrored row and may expose slower disks sooner.
- A process crash between JSONL and CSV appends can leave a CSV row behind; JSONL remains the
  recovery oracle and the mirror is explicitly non-atomic across files.
- `record_json` is the lossless field; envelope columns are convenience projections only.
- The 16 fixed stream mapping is the current source contract and must be tested against future
  additions rather than silently omitted.
