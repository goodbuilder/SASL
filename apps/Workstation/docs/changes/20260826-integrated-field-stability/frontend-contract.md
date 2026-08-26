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
- NTRIP separates DNS/TCP, HTTP_AUTH_FAILED, CONNECTED_NO_DATA, STREAMING and route/injection state.
- STUS shows attempt 1/2 or 2/2, both-side HIWS preflight, Left-ACK-then-Right dispatch, the signed
  per-side SR-PC median and target lead, reconnect/generation/SR warmup, 2-round/90-second/±2-second
  limits and the terminal reason. ACK remains pending until three stable pad-probe SRs and every
  live limit pass. A previously VERIFIED state visibly changes to FAILED/revoked when current
  evidence exceeds a limit or the stream generation resets; frozen last metadata cannot remain green.
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
The Set camera clock control is disabled during recording, and the backend guard rejects a direct
request even if a stale UI action reaches it. It remains disabled across command, reconnect and SR
warmup, not merely while a QThread exists. Raw SDK STUS directs the operator to this interlocked
control instead of sending a mutation. Competing zoom/SDK/STIP/image-format/RTSP control work is
mutually excluded for the full action.

Dynamic Connect/Disconnect/Start/Stop text is authoritative. A role-property change must not force
synchronous QStyle unpolish/polish from a live device-status callback; queued/coalesced repaint is
allowed only when it cannot delay command completion or the receive timer. Desktop accessibility
stays enabled.

## Cross-feature visible behavior and isolation

One side or one writer failing changes only its dependents. Camera remains usable during NTRIP
failure; healthy P1/P2 and unrelated storage streams continue where safe. Measurement admission is
not blocked by old Session/spool or best-effort storage state.
