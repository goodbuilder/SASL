# Decisions

Append accepted decisions chronologically. Supersede entries rather than deleting history.

## 2026-08-29 — Proposed for packet approval

- Create a separate `20260829-calibration-app-receive-timing` packet rather than amending another active packet.
- Use `./README.md` as the workspace README target.
- Make decoded-frame app-receive monotonic the ordinary Live/Auto Calibration gate; retain NTP as record-only.
- Keep verified exposure and FTP CPTR-dispatch contracts unchanged.
- Add no timing-basis selector and perform no automatic RTCP/NTP fallback.
- Preserve historical Photo sets and their persisted decisions without migration.
- Record host receipt as a proxy, never as verified exposure or precision timing.

The first two decisions were explicitly confirmed by the user. The remaining scope is awaiting packet approval.

## 2026-08-29 — Approved and implemented

- The user explicitly approved the exact `approval-summary.md` scope.
- Exact integer app-receive metadata is authoritative. The owned native-frame `timestamp_s`
  compatibility field is used only when the exact field is absent; a present but malformed exact
  field fails closed and cannot be hidden by another clock.
- `HOST_RECEIVE_PROXY` may permit pair persistence but cannot claim precision timing or exposure.
- During full regression an independently running Workstation PID 92393 continuously owned native
  shared-memory segments. The unrelated reclaim test treats every PID except pytest itself as dead,
  so it fails and removes those live segments. Do not rerun it until that app is closed or the user
  accepts this named environmental residual.

## 2026-08-29 — Verification completed

- The user closed Workstation PID 92393.
- The shared-memory boundary test passed independently and the clean full suite passed with
  `1630 passed, 1 skipped`.
- Software acceptance is complete. Real dual-WIRIS exposure simultaneity remains a separately
  labelled field limitation rather than an unverified software claim.
