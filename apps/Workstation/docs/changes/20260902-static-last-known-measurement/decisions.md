# Decisions

Append accepted decisions chronologically. Supersede entries rather than deleting history.

## 2026-09-02 — Separate change packet

- Status: accepted.
- Decision: handle retained static measurement as an independent
  `static-last-known-measurement` change rather than merging it into another active packet.
- Consequence: no source changes are authorized until this packet reaches explicit approval.

## 2026-09-02 — Automatic retained-static fallback

- Status: accepted.
- Decision: when a previously valid live attitude or GPS source disconnects, Measurement
  automatically uses that source's last valid sample instead of requiring a Freeze action.
- Consequence: the UI and persisted evidence must distinguish `LIVE` from
  `RETAINED STATIC`, expose the retained sample time/age, and return to live input when a
  newer valid sample arrives.
- Safety boundary: automatic fallback cannot create a value for a side that has never
  produced a valid sample.

## 2026-09-02 — Reset retained values on application restart

- Status: accepted (recommended default).
- Decision: retained attitude/GPS values are process-local and are discarded on shutdown or
  restart. Every new application run must receive at least one new valid sample for each required
  side before automatic retained-static fallback is eligible.
- Consequence: saved historical evidence remains reviewable but is never an active input source
  after restart.

## 2026-09-02 — Treat stale reception like disconnect

- Status: accepted (recommended behavior).
- Decision: automatic fallback activates both when the transport explicitly disconnects and when
  required packets stop becoming fresh while the transport still appears connected.
- Consequence: a stalled receiver does not block a static measurement after valid process-local
  observations exist. The visible state remains `RETAINED STATIC`, never `LIVE`.

## 2026-09-02 — No manual clear command

- Status: accepted.
- Decision: do not add a `Clear retained values` action.
- Consequence: retained values are superseded only by newer valid live samples or cleared by
  application shutdown/restart. No new command, confirmation dialog, or keyboard action is added.
- Known tradeoff: if the rig moves while telemetry remains unavailable, the application cannot
  detect that motion and will continue to expose the retained-static method as measurable.

## 2026-09-02 — No persistent retained-static warning or age display

- Status: accepted.
- Decision: do not add a persistent `RETAINED STATIC` warning or continuously displayed
  attitude/GPS sample ages to the Measurement screen.
- Consequence: no warning banner, confirmation, modal, or expanding live-age surface is added.
- Open boundary: whether the ordinary method/status text remains visually identical and whether
  retained provenance is still mandatory in saved evidence must be decided separately.

## 2026-09-02 — No added retained-static evidence marker or log event

- Status: accepted, subject to the existing evidence-schema boundary.
- Decision: do not add a retained-mode boolean/status, dedicated fallback event, retained age,
  or retained-source label to saved Measurement evidence or application logs.
- Preserved boundary: existing exact attitude/GPS payloads and their real source/receive
  timestamps cannot be removed, rewritten, or stamped as current; doing so would falsify the
  measurement record and violate the established evidence contract.
- Open question: whether preserving those already-required unmodified fields is acceptable.

## 2026-09-02 — Preserve existing exact inputs and timestamps only

- Status: accepted.
- Decision: retain the existing attitude/GPS values and true source/receive timestamps exactly as
  today, without adding a retained/fallback/stale marker or event.
- Consequence: evidence remains truthful and schema-compatible while the UI and logs receive no
  new retained-state presentation.

## 2026-09-03 — Symmetric attitude/GPS retention

- Status: accepted.
- Decision: retain the last valid process-local sample for P1/P2 attitude with the same
  mechanism already used for P1/P2 GPS, rather than latching attitude once while GPS keeps
  being re-read live. `_cached_measurement_attitude_states()` mirrors
  `_cached_measurement_gps_states()`, and both feed readiness and pair freezing.
- Consequence: the operator-visible rule is one rule for both sensor halves. The
  `attitude not received in current Measurement` hard block is removed from
  `_measurement_live_readiness()` and from `_freeze_measurement_pair_inputs()`.

## 2026-09-03 — Attitude reuse reported through existing `reused_inputs`

- Status: accepted.
- Decision: emit `left_attitude`/`right_attitude` in the existing `reused_inputs` evidence
  field, alongside the existing `left_gps`/`right_gps` values, and add one method advisory
  string. No new evidence field, marker, age display, UI control, or log event is created.
- Consequence: symmetric reuse provenance without widening the evidence schema.

## 2026-09-03 — Retention cache follows configured `minimum_fix_type`

- Status: accepted; extends the approved scope and was confirmed with the user.
- Decision: `_gps_state_is_valid_for_measurement_cache()` admits samples by the configured
  `ReadinessConfig.minimum_fix_type` instead of a hardcoded 3D fix.
- Motivation: with the literal `3`, a later RTK_FLOAT or plain 3D sample silently displaced a
  retained RTK_FIXED sample, so "reach RTK Fixed, then keep using that position" did not hold.
- Consequence: the default stays `3` and existing behavior is unchanged. Setting the existing
  `Minimum numeric fix_type` to `6` now makes the retained position an RTK_FIXED sample that a
  degraded later sample cannot replace. No new setting is introduced.

## 2026-09-03 — Retained GPS pair bound to its correction frame

- Status: accepted; extends the approved scope. Raised by the operator constraint that P1 and
  P2 cannot be connected over USB at the same time.
- Context: a rover fix is `C_base + V`, where `C_base` is the coordinate the Base asserts. The
  P1->P2 baseline is a difference, so `C_base` cancels and the baseline stays accurate even when
  the Base coordinate is absolutely wrong - but only while both fixes resolve against the same
  `C_base`. Sequential connection puts minutes between the two fixes, so a re-survey between them
  leaves `C_base2 - C_base1` in the baseline as silent error.
- Decision: derive a correction-frame token from `gnss_operation_manager.source_identity.stable_id`
  and `gnss_fixed_base_profile.profile_hash`, and invalidate the retained GPS per side when it
  changes. A sample observed before the change is not re-admitted, so the drop cannot be undone by
  the stale last-received sample; a connected side recovers on its next sample.
- Consequence: unplugging one board and connecting the other never invalidates anything, since it
  does not change the frame. Attitude retention is unaffected - attitude does not resolve against
  the Base.
- Known limit: a Base that physically moves while asserting the same coordinate produces the same
  class of error and is not detectable in software. It stays a field procedure rule.

## 2026-09-03 — Pair capture span reported, not blocked

- Status: accepted; extends the approved scope.
- Decision: when the retained P1/P2 GPS pair spans more than the configured
  `maximum_left_right_timestamp_delta_ms`, emit an advisory carrying the exact span. No hard block.
- Motivation: sequential connection always exceeds the live 50 ms pair limit, so blocking would
  make measurement impossible on a rig that cannot connect both boards at once. The operator still
  needs the span visible, and the measurement path previously recorded `gps_pair_delta_s` without
  surfacing it.
- Consequence: no new setting or evidence field; an existing threshold and the existing advisory
  channel are reused.

## 2026-09-03 — PER_ROVER_SESSION GGA follows the connected rover

- Status: accepted; chosen by the operator over `BASE_GNSS` and `LEFT_GNSS`.
- Context: the field settings are `NETWORK_RTK_DIRECT` against `RTS2.ngii.go.kr/VRS-RTCM32` with
  `gga_policy=SYSTEM_CENTER`. A VRS caster withholds RTCM until it receives a periodic GGA, and
  `SYSTEM_CENTER` builds that GGA from P1 and P2 together. With only one board connectable at a
  time the GGA was never produced, so neither board could reach RTK Fixed at all.
  `PER_ROVER_SESSION` was a duplicate of `LEFT_GNSS` and read only the left side, so it did not
  solve the P2-only session either.
- Decision: `PER_ROVER_SESSION` now sends the GGA from whichever rover has a fresh valid position,
  trying left first. Both-connected behaviour is unchanged because left still wins. When no rover
  delivers, one combined skip reason naming both sides is logged instead of per-side reasons.
- Rationale: a VRS only needs the rover placed inside its service area, and P1/P2 sit metres
  apart, so a single-rover GGA is sufficient. The existing per-policy freshness limit is unchanged.
- Consequence: the operator selects `세션별 rover` as the GGA policy. No new setting is added.

## 2026-09-03 — Correction-frame token includes the observed RTCM reference station

- Status: accepted; follows the operator choosing `PER_ROVER_SESSION`.
- Context: with `PER_ROVER_SESSION` each board's session sends its own GGA, and a VRS caster
  places the virtual station from that GGA. The caster/mountpoint identity and the Base coordinate
  profile stay unchanged across those sessions, so the config-derived token could not see the
  frame move at all.
- Measured against `RTS2.ngii.go.kr/VRS-RTCM32` on 2026-09-03:
  - The advertised RTCM 1006 ARP equals the GGA position sent at session start, to GGA precision.
  - Two consecutive sessions sending the same GGA reported a byte-identical ARP (0.0000 m apart).
  - Changing the GGA by 2 m mid-session did not move the ARP; the station is held for the session.
  - 1006 arrives roughly every 7.5 s, so a new session's station is known within about 8 s.
- Decision: `parse_rtcm3_reference_station_arp()` decodes 1005/1006 into exact 0.1 mm integers,
  `_record_gnss_base_rtcm_frame()` publishes the latest one as
  `panel.latest_rtcm_reference_station`, and `_measurement_correction_frame_token()` includes it.
  Exact comparison is used because the measurement shows the station does not drift within a
  session, so no invented movement threshold and no new setting are needed.
- Consequence: a P2 session whose caster places a different virtual station now drops the retained
  P1 fix instead of forming a baseline across two references. The observed station is replaced only
  by a newly observed one and never cleared on a stream gap, since the retained fixes still belong
  to the station they were measured under.
- Known gap: for about 8 s after a reconnect the previous station still stands. A board cannot
  reach the configured fix threshold that quickly on a fresh correction stream, so this is accepted.

## 2026-09-03 — Correction-frame drop is explained in the existing advisories

- Status: accepted; requested so the field procedure needs no terminal.
- Problem: when a frame change discarded a retained fix, readiness reported only the existing
  `live GNSS baseline missing`. A side emptied by a frame change was indistinguishable from one
  that never delivered, and the observed reference station is deliberately not shown in the UI, so
  the operator had no way to tell why a board that had reached RTK Fixed stopped being usable.
  Under `PER_ROVER_SESSION` against a VRS mountpoint this is expected every session, not an edge
  case.
- Decision: `_cached_measurement_gps_states()` records which sides are currently empty following a
  correction-frame change, and `_measurement_live_readiness()` names them in one advisory that
  states the required action. No new UI element, status, setting, or evidence field is added; this
  reuses the advisory channel already carrying the reuse and pair-span messages.
- Consequence: the whole sequential P1/P2 field procedure is observable inside the app.
