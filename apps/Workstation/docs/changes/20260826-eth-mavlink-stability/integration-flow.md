# Integration Flow

## Action-to-postcondition traces

`Connect P1` → bind UDP 14550 with GCS identity → receive sysid 1 → begin 1 Hz GCS heartbeat →
request Stock stream profile once → begin 10-minute qualification → publish READY only after
continuous heartbeat+ATTITUDE evidence.

`RTCM frame parsed` → resolve currently eligible targets → enqueue complete frame → recheck target
liveness at send boundary → send only eligible side → record sent or `blocked_stall` evidence.

`All RX silent 10 s` → transition side to STALLED → remove from RTCM/command eligibility → update
operator state and evidence once → await recovery decision/action.

## Transition tables

| From | Evidence | To | Side effect |
|---|---|---|---|
| DISCONNECTED | valid expected sysid packet | QUALIFYING | heartbeat/profile start |
| QUALIFYING | continuous HB+ATT for 10 min | LIVE | readiness may pass |
| QUALIFYING/LIVE | ATT old, other RX fresh | ATTITUDE STALE | bounded ATT request |
| QUALIFYING/LIVE/ATTITUDE STALE | all RX silent 10 s | ETH RX STALLED | RTCM/commands blocked |
| ETH RX STALLED | valid RX resumes or reboot succeeds | RECOVERING | new generation/soak |
| RECOVERING | continuous HB+ATT for 10 min | LIVE | readiness may pass |
| any connected | NIC/bind/link loss | DISCONNECTED/ERROR | normal transport cleanup |

## Failure and recovery flows

- P1 stalls, P2 live: P1 RTCM blocked; P2 continues; aggregate UI cannot claim both healthy.
- RTCM queued before stall: send-boundary eligibility check drops P1 copy and preserves P2 copy.
- Host reconnect requested while stalled: explain that Pixhawk-side restart is required; do not
  falsely cycle the listener as recovery.
- USB missing/mapping ambiguous: do not reboot; show side-specific manual power-cycle instruction.
- Reboot during measurement/finalization: reject command with the active lifecycle reason.

## Cross-feature contracts and dependency direction

Transport → immutable liveness snapshot → status/readiness/RTCM routing. Measurement consumes
readiness only. GNSS does not own MAVLink liveness; UI does not own recovery policy; RTK injection
does not inspect private connection collections.
