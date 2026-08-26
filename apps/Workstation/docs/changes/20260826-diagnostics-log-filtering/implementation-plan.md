# Implementation Plan

## Approval status

Awaiting explicit approval. `implementation_allowed: false`.

## Slices and dependencies

### S1 — Event contract and classifier (`NOT_RUN`)

- Source of truth: approved frontend/backend contracts and existing raw message wording.
- Input → output: raw message + optional metadata → immutable canonical event.
- Invariants: raw message unchanged; explicit metadata wins; ambiguous device is unassigned;
  severity does not become operational readiness.
- Oracle/tests: hand-built sentinel messages for WIRIS L/R, P1/P2, GNSS Base/NTRIP, FTP,
  unknown/ambiguous/error/warning and bilingual text.

### S2 — Bounded store and thread boundary (`NOT_RUN`, depends on S1)

- sequence/time ordering, incremental cursor batches, 2,000 limit, overflow evidence.
- worker-thread log request reaches store, existing widget, and persistence path exactly once.
- negative tests: duplicate cursor, stale cursor, burst overflow, malformed metadata, shutdown.

### S3 — Backend facade and compatibility (`NOT_RUN`, depends on S2)

- add typed incremental event read facade while retaining `diagnostic_text()`.
- fallback backend without structured facade remains usable.
- no direct UI access to panel widgets/store.

### S4 — Log table and filters (`NOT_RUN`, depends on S1/S3)

- compact table model/proxy and Source/Device/Level/search controls.
- fixed column geometry, non-color level cues, tooltip/copy, empty/error states.
- normal tests plus combined filters, unicode/case search, long messages, no matches.

### S5 — Live viewport lifecycle (`NOT_RUN`, depends on S4)

- following, scrolled-up, paused, resumed, hidden-tab catch-up and overflow behaviors.
- preserve focus, row selection, manual scroll/column widths and dock geometry during bursts.

### S6 — Cross-feature integration (`NOT_RUN`, depends on S2–S5)

- mixed Camera/Pixhawk/GNSS/Measurement/FTP sequence remains chronological.
- NTRIP is GNSS/NTRIP, not WIRIS fallback; ambiguous events remain Shared/Unassigned.
- Workspace/Inspector selection, FTP tab, recording audit, device commands and shutdown remain
  unchanged.

### S7 — Documentation, regression, runtime evidence (`NOT_RUN`, depends on S6)

- update affected numbered UI docs and README after approval.
- run focused suites, then Workstation full regression suite and launcher check.
- capture baseline/after screenshots and rapid-update state sequence.

## File and component map

- add `workstation_core/diagnostic_log.py`: event/classifier/store/batch pure logic.
- change `standalone_gui.py`: store ownership, thread-safe payload, central append; preserve legacy.
- change `standalone_backend.py`, `protocol.py`: read facade/contract.
- change `components.py`: table model/proxy or reusable diagnostic table component.
- change `workspaces.py`: Log toolbar/table/pause/follow/fallback integration.
- change focused tests in `tests/test_diagnostic_log.py`, `tests/test_workstation_ui.py`,
  `tests/test_workstation_ntrip.py`, `tests/workstation_test_support.py`; add boundary regression
  where the existing owner tests live.
- update packet docs, affected `docs/ui/*`, and `README.md`.

## Migration and rollback steps

No persisted migration. Seed the table from the store when opened; compatibility backends use text
fallback. Rollback removes the structured UI/facade and restores the old text projection without
touching device state or saved data.

## Stopping conditions

- source work begins only after explicit packet-scoped approval.
- stop and amend approval if implementation requires disk schema changes, settings persistence,
  device protocol/worker changes, unbounded buffering, or sibling application refactoring.
- do not complete with any required slice `FAIL`, `BLOCKED`, or `NOT_RUN`.
