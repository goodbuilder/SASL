# Frontend Contract

## Controls and inputs

- Existing P1/P2 connection controls remain.
- Affected side exposes one recovery action only after confirmed stall. It opens a side/sysid/USB
  identity confirmation and sends a one-shot reboot only through a currently verified USB link.
- No automatic parameter/config-file write is added.

## Payloads and commands

The view receives an immutable per-side health projection containing connection state, last
heartbeat age, last ATTITUDE age, last-any-message age, qualification elapsed/required time,
RTCM route state, suspected-cause text, and allowed recovery command.

## Visible states and transitions

- `DISCONNECTED` → listener absent.
- `CONNECTING` → listener open, no valid remote identity yet.
- `QUALIFYING n:nn / 10:00` → heartbeat and ATTITUDE continuously fresh after connection/reboot.
- `LIVE` → qualification complete and telemetry fresh.
- `ATTITUDE STALE` → heartbeat/other packets fresh but ATTITUDE stale.
- `ETH MAVLINK RX STALLED` → valid remote was live, then no inbound MAVLink for 10 seconds while
  the local listener remains open; detail says `PX4 UDP task stall suspected`, not proven.
- `RECOVERING` → approved reboot/reconnect completed and qualification restarted.
- NIC/bind failures keep distinct existing `DISCONNECTED`/`ERROR` semantics.

## Errors and recovery actions

Stall detail includes P1/P2, endpoint, silence age, `RTCM 차단됨`, and the selected action. A host
socket reopen must not be described as Pixhawk recovery. If verified USB mapping is unavailable,
the UI instructs the operator to power-cycle only the named Pixhawk and then reconnect.

## Keyboard, focus, accessibility, and layout

Reuse existing status/action components and keyboard navigation. Dynamic age/qualification text
uses fixed-width or reserved space so 1 Hz updates do not resize rows. Color is supplementary to
state text. Recovery confirmation names the side and explains that measurement readiness resets.

## Cross-feature visible behavior and isolation

A P1 failure changes only P1 and any measurement method that requires P1. P2 and unrelated camera,
calibration, recording, and NTRIP source status remain stable. GNSS status explicitly identifies a
disabled correction target instead of presenting aggregate success.
