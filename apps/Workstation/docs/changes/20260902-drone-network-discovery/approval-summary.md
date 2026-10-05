# Approval Summary

This document explains the proposed modification in plain language. Technical detail remains in the other packet documents.

## What changes and what stays the same

Telemetry gains `Find Network`, a compact factual candidate list and `Use selected`. It identifies
Drone candidates only when MAVLink itself reports system ID 3. Existing USB discovery, manual URL,
RTCM settings and explicit Apply/Save/Connect/Disconnect stay available and unchanged.
Apply and Save are corrected so the accepted MAVLink URL remains visible and is not replaced by an
older runtime/settings snapshot during refresh.

## Operator workflow before and after

Before: discover IP/port outside Workstation and type it manually. After: press Find Network,
select the verified Drone row, use it to fill the URL, then explicitly Apply, Save and Connect.
Discovery never changes networking or opens/closes a device by itself.

## Success, delay, rejection, failure, overload, and recovery

The scan is bounded and runs off the UI thread. It reports searching, found, empty, ambiguous or a
factual error. sysid 1/2 cannot be chosen as Drone. Duplicate scans, stale rows, malformed packets,
no carrier/traffic, timeout, bind conflict and shutdown are fail-closed. Manual URL and Find USB
remain recovery paths. Connection success requires actual sysid-3 heartbeat/attitude/GNSS evidence,
not a button click or open socket.

## Data and compatibility effects

No new persistent schema and no migration. Candidate evidence is transient; only the existing Save
command persists the selected URL. Old settings continue to load.

## Performance priorities and allowed degradation

One bounded low-rate CPU/network worker, no GPU/media work, no continuous background scan and no
unbounded queue. Camera preview and all existing device links must remain responsive; if discovery
cannot run safely, it fails locally rather than interrupting them.

## Visual and interaction changes, including dynamic-layout stability

Reuse native compact Qt controls. The action row adds one text button; a bounded read-only table
shows only State/NIC/IP/port/sysid and does not resize lower groups during updates. Completion does
not steal focus, selection or scroll. Long interface/error text is elided or scrollable with full
accessible detail.

## How acceptance will be demonstrated

Focused pure/worker/controller/UI tests cover valid sysid3, sysid1/2 rejection, empty, malformed,
ambiguous, duplicate, timeout, stale generation, persistence/restart and shutdown. Relevant existing
Telemetry/Pixhawk network regressions and the full pytest suite run afterward. The real launcher and
screen states are checked. Finally the attached `10.0.0.3:14551` sysid-3 Drone is selected,
connected, and verified through live heartbeat/attitude/GNSS evidence.

## Cost, risks, assumptions, rollback, and field limits

The primary risk is confusing source IP/port discovery with connection readiness, especially
because the current Drone uses the traditional P2 address/port. Identity and Connect evidence keep
those boundaries separate; simultaneous conflicting Ethernet ownership is rejected/warned. No PX4
or host-network mutation is authorized. Removing the feature leaves all existing paths intact.

## Exact scope awaiting approval

Approved: only the Telemetry read-only Drone network discovery, explicit candidate-to-URL selection,
existing connection integration, focused/full verification, current hardware connection and the
listed Workstation documentation updates, plus preservation of the accepted URL across Apply/Save
and refresh. Exclude PX4/NetworkManager mutation, automatic
connect/disconnect, P1/P2/Camera/Measurement changes, dependency upgrades and unrelated cleanup.
