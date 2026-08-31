# Verification

## Requirement-to-evidence matrix

| Slice | Independent evidence | Status |
| --- | --- | --- |
| generation/filter/XLSX contract | hand-built interleaved JSONL and ZIP/XML inspection | PASS |
| checkpoint/concurrency | queue sequence fixtures and raw-file offsets | PASS |
| async owner/failure/shutdown | controller lifecycle tests and temp-file inspection | PASS |
| stop/UI workflow | fake backend + offscreen Qt end-to-end | PASS |
| docs/launcher/regression | documented commands | PASS |

## Focused test evidence

- `QT_QPA_PLATFORM=offscreen ../../.venv/bin/python -m pytest -q
  tests/test_latest_measurement_xlsx.py tests/test_measurement_data_store.py
  tests/test_measurement_run_controller.py` passed as part of the 42-test slice.
- Independent fixtures prove exact generation exclusion, append/replace materialization,
  nested identity protection, method bundle/outcome/input expansion, `#N/A`, KST/exact ns,
  formula-text safety, malformed/torn issues, Excel shards, foreign-file preservation,
  cancellation, checkpoint partial/recovery, coalesced controller state, and real-panel delegate
  publication.

## Regression and application evidence

- Focused adjacent command covered latest XLSX, store, Measurement run, observation-first,
  recording save, runtime closure, Workstation lifecycle/contracts/UI and returned
  **155 passed in 46.96s**.
- Full Workstation suite from the repository root:
  `QT_QPA_PLATFORM=offscreen QT_OPENGL=software LIBGL_ALWAYS_SOFTWARE=1
  ./.venv/bin/python -m pytest -q apps/Workstation/tests` returned
  **1762 passed, 1 skipped in 137.24s**.
- `python3 tools/gen_panel_delegate_stubs.py --check` passed.
- `./run.sh --check` passed the canonical interpreter, distributions, GStreamer/FFmpeg,
  NVIDIA decoder and Workstation dependency checks.
- Ruff was not installed in the supported venv or on PATH; lint was therefore NOT_RUN.
- The user-confirmed `README.md` and `docs/ui/README.md` targets now cover Purpose, Current Run
  Flow, Inputs/Outputs and Interfaces, Operational Rules, Known Limitations/Workarounds, and the
  dated Change Log. `git diff --check` and delegate-stub `--check` pass after those edits.

## Performance, load, and target-hardware evidence

Excel partition boundaries are covered through injected small row/column maxima. A copy of the
current 43,132-line data root was converted in a temporary directory without touching originals:
42,425 workbook rows, 23,387,002-byte XLSX, 62.268 s elapsed, zero parse issues, and 28,608 KiB
peak RSS. Cooperative cancellation against the same data joined immediately (`0.000 s` measured),
published nothing, and left zero temp files. No GPU/device behavior changes or hardware evidence
applies.

## Feature isolation and contract evidence

PASS in automated-fake/real-filesystem tests: XLSX failure does not gate start/stop/next-run;
JSONL offsets/bytes remain independently readable; app-owned output and temp cleanup are proven.

## Visual authenticity and dynamic-layout stability evidence

PASS automated-offscreen: Data action object names, enabled states, READY/FAILED text, long-error
tooltip, fixed status height, sessionless catalog isolation, and 1366x768 shell regression.
Visual-manual inspection of `/tmp/workstation-latest-xlsx-data-bottom-fixed.png` confirmed the
existing three actions remain on row one, both XLSX buttons remain fully visible on row two, and
the fixed status line, stream table, scroll area and Inspector geometry do not clip or shift.

## Visual and hardware/manual evidence

Hardware is not applicable. Offscreen visual evidence passed; no real display/hardware behavior
is claimed.

## Residual conditions

None. The implementation remains an uncommitted working-tree change based on
`7242452fe50006a61c9bfa2e012120c9c677571c`; unrelated dirty work was preserved.
