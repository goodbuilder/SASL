# Verification

## Requirement-to-evidence matrix

| Requirement | Evidence | Status |
|---|---|---|
| chronological All and truthful metadata | core/store tests | NOT_RUN |
| source/device/level/search filters | Qt model/UI tests | NOT_RUN |
| pause/follow/overflow without loss concealment | UI lifecycle/burst tests | NOT_RUN |
| thread-safe exactly-once handoff | panel thread-boundary test | NOT_RUN |
| Workspace/Inspector/FTP/persistence preserved | integration regressions | NOT_RUN |
| geometry/focus/selection/scroll stable | Qt assertions + screenshots | NOT_RUN |

## Focused test evidence

Baseline 2026-08-26: four existing Diagnostics/NTRIP/FTP tests passed with canonical root venv.
Initial local-venv/auto-plugin attempts failed before collection due environment, not test behavior;
details are in `current-behavior.md`.

## Regression and application evidence

Planned: `test_diagnostic_log`, full relevant `test_workstation_ui`, `test_workstation_lifecycle`,
`test_workstation_camera`, `test_workstation_ntrip`, `test_workstation_pixhawk_gnss`,
`test_workstation_contracts`, then the full Workstation suite and `./run.sh --check`/`--no-start`.

## Performance, load, and target-hardware evidence

Planned: 2,000+ event burst, hidden/visible tab comparison, UI event processing latency and bounded
row/pending counts. No GPU or device-worker allocation changes.

## Feature isolation and contract evidence

Planned: unavailable structured backend fallback; Diagnostics model failure cannot stop device,
Measurement, recording or shutdown; existing raw widgets and audit writes remain intact.

## Visual authenticity and dynamic-layout stability evidence

Planned: 900 minimum/1280x720/1600x980, 100/125/150% scale, long Korean error, no data/no match/
warning/error/overflow, rapid append with focus/selection/scroll/column/dock assertions.

## Visual and hardware/manual evidence

Not run. Hardware protocol correctness is out of scope; actual Workstation display with representative
device logs remains required visual/manual evidence.

## Residual conditions

None accepted. Legacy messages without a unique device will intentionally remain
`Shared/Unassigned`; this is truthful behavior, not a verification gap.
