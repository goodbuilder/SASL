# Frontend Contract

## Controls and inputs

No new control or operator input. Add one read-only GroupBox after `Live position`, before `RTCM
delivery`. The existing position table remains the visible source evidence.

## Payloads and commands

No command/payload. `TelemetryWorkspace.refresh()` consumes
`snapshot["gnss_baseline_center_frd"]` only; it must not read device/controller internals.

## Visible states and transitions

| State | Values | Meaning |
|---|---|---|
| WAITING | `--` | missing/invalid source or undefined horizontal P1->P2 axis |
| CURRENT | finite F/R/D/Rg | three positions are finite, age <=2.0 s, spread <=1000 ms |
| STALE | last finite F/R/D/Rg retained | age/future/spread gate failed; reason and actual timing shown |

Columns are exactly `State`, `Forward m`, `Right m`, `Down m`, `Range Rg m`, `Max age s`,
`Max Δt ms`, and `Fix P1/P2/Drone`. Signed axes use three decimals; range uses three decimals.

## Errors and recovery actions

Missing data and degeneracy are normal WAITING evidence, not dialogs. STALE names age, future, or
time-spread reason. Recovery is passive on the next qualifying snapshot. Unexpected estimator
failure maps to factual WAITING/ERROR detail without breaking the rest of Telemetry.

## Keyboard, focus, accessibility, and layout

The table is read-only and non-selectable. State is textual, never color-only. Full coordinate
contract and long reason are available in tooltip/static hint. Header/row/group extents are fixed
at construction; updates do not resize, focus, select, scroll, or show/hide controls.

## Cross-feature visible behavior and isolation

The group title and hint use `GNSS BASELINE_CENTER`, never the canonical frame identifier alone.
They state `GNSS antenna midpoint`, `Right=horizontal P1->P2`, `Down=NED`, and
`Forward=Right x Down`; the hint explicitly says `not camera baseline_center_frd`.
Ideal disparity below uses the same `Rg`/freshness; RTCM and source connection remain independently
visible and usable when this row is WAITING or STALE.
