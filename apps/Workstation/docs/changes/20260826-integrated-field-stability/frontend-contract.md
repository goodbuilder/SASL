# Frontend Contract

## Controls and inputs

Use existing Preview, MAVLink Connect, NTRIP Start, time-sync/STUS, Measurement and recording controls.
Current camera/Pixhawk endpoints and SASL/NGII profile remain operator-visible. Secrets remain masked.

## Payloads and commands

Existing typed settings/controller payloads remain authoritative. UI intent is not success; every
control waits for the backend/external postcondition defined in `backend-contract.md`.

## Visible states and transitions

- Camera shows requested/effective decoder and per-side current frame or exact error/recovery state.
- Pixhawk separates CONNECTING, QUALIFYING, LIVE, ATTITUDE STALE and ETH MAVLINK RX STALLED.
  Qualification uses the snapshot-owned 60-second requirement and displays `mm:ss / 01:00`; it
  does not imply ten-minute field certification.
- NTRIP separates DNS/TCP, HTTP_AUTH_FAILED, CONNECTED_NO_DATA, STREAMING and route/injection state.
- STUS shows attempt 1/2 or 2/2, both-side HIWS preflight, Left-ACK-then-Right dispatch, the signed
  per-side SR-PC median as diagnostic evidence, relative Left/Right delta and target lead,
  reconnect/generation/SR warmup, 2-round/90-second/±2-second target limits and the terminal reason.
  Per-camera absolute SR-PC never appears as a pass/fail gate. ACK remains pending until three stable
  pad-probe SRs establish a Left/Right relative delta at or below 100 ms and every non-absolute live
  limit passes. A previously VERIFIED state visibly changes to FAILED/revoked on sustained relative
  failure or stream-generation reset; frozen last metadata cannot remain green.
- Measurement/recording shows IDLE/QUEUED/STARTING/RUNNING/STOPPING/ERROR with factual storage detail.
- Camera and Measurement remain STARTING after worker launch until current-generation decoded frame
  ACKs arrive. Video separately shows `video=STARTING/RUNNING/PARTIAL/FAILED`; thread launch or an
  allocated filename alone is never presented as active recording.
- `Decoder LEFT/RIGHT` remains `--` after `PLAYING`/candidate status and changes only with a
  current-generation decoded frame. Auto fallback reason/override is side-specific; one CPU
  fallback cannot relabel or restart the healthy peer. RTSP/source and mux/filesink errors retain
  their own text and are never shown as a decoder fallback.

The Measurement Start control remains available when MAVLink, NTRIP/GNSS, STUS or storage is
unready. The selected method/result surface shows the exact missing input and never labels the
result valid. Only an active camera/resource transition may queue Start.

## Errors and recovery actions

Messages name the failed side, boundary, last valid age/result and next safe operator action. They do
not imply device reboot or success from a host reconnect. Physical power or USB action is requested
from the user only when evidence requires it.

## Keyboard, focus, accessibility, and layout

Rapid numeric/status updates use stable formats and reserved geometry. They must not move adjacent
controls, steal keyboard focus, reset selection or jump Diagnostics/Data scroll. Text conveys state
without color-only meaning; long detail remains available through existing tooltip/detail surfaces.
The Set camera clock control remains available during Measurement/YOLO/recording but is disabled
for duplicate STUS or competing WIRIS control work. It remains unavailable across its own command,
observer restart and SR warmup, not merely while a QThread exists. Raw SDK STUS directs the
operator to this interlocked control instead of sending a mutation. Competing zoom/SDK/STIP/
image-format/RTSP control work is mutually excluded for the full action.

Dynamic Connect/Disconnect/Start/Stop text is authoritative. A role-property change must not force
synchronous QStyle unpolish/polish from a live device-status callback; queued/coalesced repaint is
allowed only when it cannot delay command completion or the receive timer. Desktop accessibility
stays enabled.

## Cross-feature visible behavior and isolation

One side or one writer failing changes only its dependents. Camera remains usable during NTRIP
failure; healthy P1/P2 and unrelated storage streams continue where safe. Measurement admission is
not blocked by old Session/spool or best-effort storage state.

## Workstation STUS scheduling and layered truth

Settings exposes `Manual`/`Auto` plus an interval in minutes. Defaults are Manual and 30 minutes;
the accepted range is 5–1440 minutes. Auto visibly waits for one successful manual seed after every
application start/new camera connection. Time / Sync shows the last trigger/result, clock state,
Frame timing state, Media state and next due time without moving controls or resetting focus/scroll.

Manual drift revokes the clock state and records the reason but performs no command or reconnect.
Auto executes only at the configured deadline. Manual and automatic STUS remain available during
Measurement, YOLO and recording; status names the clock-epoch transition while existing imagery and
recording controls remain active. Clock VERIFIED may coexist with Frame timing NOT_VERIFIED and
Media UNAVAILABLE, and this never promotes a timing-dependent Measurement result.

Time / Sync labels the SR-PC threshold/value as diagnostic-only, not a Gate. Frame timing reports
Left/Right matched-frame delta and dropped/unmatched candidates separately. A single missing or
outlier frame does not revoke Clock VERIFIED; the 100 ms relative quality objective is evaluated
over sustained observations.

## Proposed app-receive fallback UI — not yet approved

Reuse the existing Measurement time-basis combo. Rename/explain `RTCP/NTP preferred` as the explicit
health-aware policy: before its manual seed it waits, while VERIFIED it uses NTP, and after
revocation/evidence loss it uses app receive. `RTCP/NTP required` remains fail-closed and `App
receive` remains explicitly always-app. No silent migration changes an existing saved selection;
the current field profile is changed only through Apply/Save after approval.

The existing runtime pair row displays a stable format such as:

`basis=APP RECEIVE (DEGRADED) | selected Δ=24.3 ms | NTP Δ=146.2 ms (record-only)`.

Distance remains numerical but the recommended suffix is
`validity=ESTIMATED quality=DEGRADED_TIMING`. Tooltip/detail names the Clock state, revocation
reason, basis generation and recovery action. Time / Sync remains FAILED/REVOKED; fallback never
turns it green. During STUS transition the pair row shows waiting and drops candidates. On recovery,
future rows show NTP VERIFIED after a buffer flush. No modal, focus change, scroll jump, dynamic
repolish or layout resize is introduced.

## Superseding fixed app-receive UI contract — awaiting approval

The health-aware fallback UI above is superseded. Live distance timing is not selectable: the
existing live Measurement basis field becomes a read-only `App receive (authoritative)` value.
Legacy/offline compatibility settings remain readable but cannot silently change live pairing.

The stable runtime pair text is:

`basis=APP RECEIVE | app Δ=24.3 ms | NTP Δ=146.2 ms (record-only)`.

An accepted app pair and otherwise valid geometry displays normal `validity=VALID`; Clock/STUS state
does not add degraded timing. Time / Sync continues showing NTP/STUS truth independently. Missing or
over-100-ms app pairs show a waiting/drop reason rather than a distance. NTP missing displays `--`
but never blocks. STUS epoch transitions update record-only Clock detail without moving, clearing or
reclassifying the current app-timed distance controls.

## Proposed indoor experiment status contract — awaiting approval

No new control, card, modal or dashboard is added. The environment-launched mode is visible through
the existing per-side Camera/status/detail surfaces using a stable label such as
`EXPERIMENTAL NON-IDR INTRA`. Recovery detail includes side, exact PTS, blocked duration and
suppressed-frame count. A later gap returns the same surface to `RECOVERING_INTEGRITY` without
resizing controls or changing focus/selection/scroll.

Measurement continues displaying fixed `APP RECEIVE` and record-only NTP. A decoded experimental
frame is not described as camera IDR or exposure truth. Recording status distinguishes bytes/EOS
from `portable integrity FAIL: usable IDR missing`; the experiment flag never turns the recording
green. The 60-minute monitor writes evidence files and periodic commentary but does not add a live
GUI widget.

## Manual-only PX4 calibration PARAM contract

P1/P2 reaching `LIVE` does not start PARAM work or change the visible state. The Pixhawk Parameters
group remains `PARAM readback 대기 중` until the operator invokes the existing side-specific
Refresh command. An explicit request uses the existing disabled/busy behavior, `PARAM SYNC`, result
tables, error text, and retry path without moving controls or changing focus, selection, or scroll.
Reset/apply/save continues to request its required readback through the same owner.

## 60-second MAVLink qualification presentation

No new control or layout is added. Initial/recovery states retain the existing fixed status region,
HB/ATT/ANY ages and side-specific reason. At 59.999 seconds the side is still QUALIFYING or
RECOVERING; at 60.000 seconds with fresh HB+ATT it becomes LIVE. A stale or stalled state replaces
LIVE immediately and existing recovery actions remain available.

## Mountain GNSS 3D-fix presentation

No control, setting or layout changes. Existing candidate observability, warning, method catalog
and Measurement quality surfaces show the new truth: both capture sides at `fix_type >= 3` enable
Mountain translation; a usable non-fixed baseline remains `ESTIMATED`; a sub-3D side shows the
side-specific numeric blocker. Rotation failures remain separately visible and can still block the
candidate even when GNSS translation is ready.

## Mountain suggestion copy presentation — approved 2026-08-29

Reuse the existing `Automatic point suggestion · review required` group and controls. For Mountain,
`Use suggestion` remains visible and optional; it is enabled whenever the current proposal stays
structurally usable after paired exclusions, regardless of detector quality state. The status strip
keeps `GOOD`/`LOW_CONFIDENCE` and separately shows `COPY READY` or `COPY BLOCKED` with the
structural reason. An accepted low-confidence proposal remains visibly `LOW_CONFIDENCE ·
AUTO_ACCEPTED`.

For Horizon, quality state/confidence also remains display-only. `Use suggestion` is enabled when
both current frozen sides contain exactly two finite in-frame endpoints with at least 25%-width
separation. Short, missing, stale or malformed endpoints show `COPY BLOCKED`; accepted
LOW_CONFIDENCE remains visibly `LOW_CONFIDENCE · AUTO_ACCEPTED`.

Busy, stale, malformed, fewer-than-four, out-of-range, duplicate and degenerate Mountain states
remain disabled/rejected. No control, tab, modal, styling, shortcut or layout is added; focus,
scroll, frozen-view zoom and candidate selection remain unchanged. Both solvers receive only the
resulting manual/accepted annotations and have no direct proposal fallback.

## Approved Settings-preferred timing presentation — 2026-08-31

Re-enable the existing Measurement time-basis control without adding a card, dialog or navigation
step. Its default is `App receive`. The recommended second visible choice is `RTCP/NTP preferred`;
when it cannot be used, the existing status line shows a stable form such as
`requested=RTCP/NTP | effective=APP RECEIVE | fallback=<reason>` while distance continues under the
app contract. Time / Sync remains truthful and is not turned green by fallback.

Healthy selected RTCP/NTP actually owns pair selection. Any RTCP timing problem changes the
effective label to APP RECEIVE with a concise reason while keeping an otherwise-valid distance
`VALID`; Time / Sync remains independently truthful. Apply/Save during a running Measurement marks
the value for the next Measurement. When no Measurement is active, Apply advances the policy and
flushes old candidates immediately. The control and dynamic status retain geometry, focus,
selection and scroll.
