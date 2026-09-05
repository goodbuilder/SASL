# Frontend Contract

## Controls and inputs

No new control/input. Existing Measurement/Manual Record gates production. Data Status and Latest
XLSX controls remain unchanged.

## Payloads and commands

No new user command. Existing `Update Latest XLSX` includes the new stream automatically.

## Visible states and transitions

- Data Status stream `gnss.baseline.center`: IDLE/PENDING/OK/DEGRADED/FAILED using existing store
  projection, written/dropped/pending counts and last error/time.
- XLSX sheet `GnssBaselineCenter`: zero rows/`#N/A` when absent, exact latest-generation rows when
  present. Summary reports its saved row count.

## Errors and recovery actions

Use existing `Retry Storage` and `Update Latest XLSX`. No modal or Telemetry error. JSONL success is
not inferred from the live F/R/D/Rg display.

## Keyboard, focus, accessibility, and layout

No control/layout change. Stream and sheet names are textual and non-color-dependent.

## Cross-feature visible behavior and isolation

Telemetry display continues even if the stream fails. Measurement/recording state is not derived
from this stream. XLSX failure does not rewrite/suppress JSONL.
