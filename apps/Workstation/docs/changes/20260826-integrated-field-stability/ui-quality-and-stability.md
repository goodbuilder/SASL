# UI Quality and Stability

## Existing product-specific visual language and component tokens

Retain the current dense operator workspace, status strip, existing health tones, tables, docks and
detail/tooltip patterns. This packet does not authorize a visual redesign.

## Affected screens, components, text, tables, plots, and conditional controls

Potentially Camera status/CAM tooltip, Time/Sync, Pixhawk/GNSS state, Data writer state and existing
Diagnostics Performance. Only a reproduced truth/interaction defect may change them.

## AI/template-like pattern rejection audit

No new dashboard shell, card grid, pills, gradient, glow, decorative metric or invented health score.
Status remains factual, side-specific and action-oriented.

## Dynamic value formats, precision, units, longest extents, and update rates

Reserve stable formats for `mm:ss / qualification`, HB/ATT/ANY age seconds, RTCP pair/SR-PC ms,
frame/drop counts, MiB/GiB, decoder name and long boundary-specific errors. UI refresh remains
throttled independently from data collection.

## Reserved geometry, overflow, wrapping, elision, and detail access

Use existing reserved regions and elision/tooltip/detail paths. Error strings may not resize Camera,
GNSS or status rows on every update.

## Focus, selection, scroll, table-column, dock/splitter, and window stability

Rapid state transitions and 60-minute updates must preserve focus, row selection, manual scroll,
column widths, dock/splitter sizes and window geometry.

## Accessibility, font scaling, DPI, locale, theme, and target-size checks

Retain text plus color semantics and existing target sizes. Verify Korean/English long states at the
current target display and offscreen regression resolutions if UI changes.

## Before/after screenshots and dynamic-state sequence evidence

No screenshot is required before source changes that do not affect UI. Any visible change requires
before/after plus CONNECTING/QUALIFYING/LIVE/STALE/STALLED/ERROR/RECOVERING sequences.

## Proposed timing-fallback presentation — not yet approved

Reuse the existing Measurement pair/status and Time / Sync surfaces. Add no card, modal, animation
or new workspace. The stable one-line pair status should reserve space for these fixed fields:

`basis=NTP VERIFIED|APP RECEIVE DEGRADED`, selected delta in ms, NTP delta in ms or `--`, and the
fallback/recovery reason in the existing detail/tooltip. Distance text retains its current geometry
and numeric precision but shows `validity=ESTIMATED` and `quality=DEGRADED_TIMING` during fallback.
NTP is labelled `record-only/untrusted`, never hidden.

Updates are coalesced with the existing status cadence. Basis changes flush pair buffers but must
not resize controls, steal focus, change selection, jump scroll or repolish live widgets. Verify
normal NTP, drift fallback, missing evidence, epoch transition, recovery, long Korean reason and
rapid 99.9→100.1 ms boundary states.

## Superseding fixed-basis presentation — awaiting approval

Replace `APP RECEIVE DEGRADED` with fixed `APP RECEIVE` and normal VALID semantics. Reserve one
stable line for authoritative app delta plus record-only NTP delta. The existing live basis control
becomes read-only rather than disappearing, avoiding geometry/focus reflow and making the policy
visible. Verify 99.9/100.0/100.1 ms app boundaries, NTP missing/large/regressed values, Clock failure,
active STUS and long record-only NTP text without layout, focus, selection or scroll changes.

## Proposed indoor experiment presentation — awaiting approval

Reuse the existing Camera state/detail line. Reserve and test `EXPERIMENTAL NON-IDR INTRA`,
`RECOVERING_INTEGRITY` and long side-specific failure details at the current 2880×1800 target plus
offscreen regression sizes. Updates remain coalesced; no synchronous repolish, modal, animation or
new layout is introduced. Rapid recovery/re-latch must preserve focused control, workspace,
splitter/dock geometry, table selection/columns and manual scroll. Text and existing severity color
both convey the state.

## Manual-only PX4 calibration PARAM presentation

No visual component changes. The existing `PARAM readback 대기 중`, `PARAM SYNC`, completed tables,
and failure detail retain their current size, cadence, colors, and action locations. Removing the
implicit transition means fewer dynamic state changes; explicit Refresh must preserve the same
focus, selection, scroll, table widths, docks, and splitters already covered by Pixhawk UI tests.

## 60-second MAVLink qualification presentation

The existing qualification text remains fixed-format `mm:ss / mm:ss`, now ending at `01:00` from
the health snapshot rather than a hard-coded literal. Verify 00:59→01:00, QUALIFYING→LIVE,
LIVE→ATTITUDE STALE/RX STALLED and RECOVERING→LIVE without resize, focus/selection/scroll change,
synchronous style repolish or new decoration.

## Mountain GNSS 3D-fix presentation

This amendment changes only existing dynamic candidate/method text. It adds no widget, geometry,
modal, style, refresh loop or focus path. Verify the current fixed surfaces can display the
accepted-non-fixed `ESTIMATED` warning and the side-specific sub-3D blocker without changing
selection, scroll, dock/splitter state or control availability.

## Mountain suggestion copy presentation — verified 2026-08-29

The reconstructed before state uses the existing fixed group and shows
`LOW_CONFIDENCE · AUTO_PROPOSED` with disabled `Use suggestion`. The after state keeps the same
group dimensions and control positions, enables the button, preserves LOW_CONFIDENCE, and adds
`COPY READY`; invalid exclusions replace it with a bounded `COPY BLOCKED` reason.

No widget, color, style, animation, card, modal or resize-to-content behavior was added. Offscreen
tests cover usable LOW_CONFIDENCE, fewer-than-four after exclusions and accepted quality retention.
Reconstructed before/actual after captures are
`/tmp/workstation_mountain_use_suggestion_before_20260829.png` and
`/tmp/workstation_mountain_use_suggestion_after_20260829.png`; both are 770×164 and preserve
control geometry.

Horizon reuses the same group/status wording. Reconstructed before and actual after captures are
`/tmp/workstation_horizon_use_suggestion_before_20260829.png` and
`/tmp/workstation_horizon_use_suggestion_after_20260829.png`; both are 771×143. LOW_CONFIDENCE,
confidence/residual/reason text and control geometry remain, while the existing button becomes
enabled and `COPY READY` appears. Short endpoints produce `COPY BLOCKED`.

## Approved Settings-preferred timing UI stability — 2026-08-31

Reuse the existing basis control and pair/status line. Enabling the combo must not add/remove rows
or resize the Measurement workspace. Reserve the requested/effective/fallback format for App,
healthy preferred, missing, stale, transition, over-limit and recovered states. Apply/Save and live
status updates must preserve focus, combo selection, scroll, splitter/dock geometry and table
columns. No modal, animation, style repolish or color-only fallback cue is introduced.

The selected RTCP option is operational when healthy. Fallback shows APP RECEIVE plus a bounded
reason in existing status/detail surfaces without changing measurement validity solely for timing
fallback. During an active Measurement, Settings may show the newly saved choice while the runtime
status continues to show the frozen request until the next Measurement.
