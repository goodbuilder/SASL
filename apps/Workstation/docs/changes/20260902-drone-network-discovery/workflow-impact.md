# Workflow Impact

## Affected operators and workflow IDs

Field operator; `WF-DRONE-TELEMETRY-NETWORK-DISCOVER-CONNECT`.

## Current workflow

The operator learns the Drone IP/port outside Workstation, manually replaces a stale URL, applies,
saves, disconnects any old owner, and connects. `Find USB` is the only in-screen discovery path.

## Proposed workflow

`Find Network -> wait for bounded read-only scan -> select verified sysid=3 candidate -> URL filled
-> Apply -> Save -> Connect -> HEARTBEAT/ATTITUDE/GNSS evidence`.

## Preserved, added, removed, and reordered steps

- Added: local `Find Network`, bounded candidate list, explicit `Use selected` action.
- Preserved: `Find USB`, manual URL editing, Apply/Save/Connect/Disconnect and RTCM selection.
- Removed: nothing. Discovery never performs Apply, Save, Disconnect or Connect.

## Decisions, confirmations, handoffs, waits, and artifacts

The operator requested this workflow after the real Drone was proven at `10.0.0.3:14551` with
`sysid=3`. Candidate evidence is transient; only Save persists the selected URL.

## Interruption, cancellation, retry, restart, and resume

One discovery runs at a time. Duplicate clicks are rejected without replacing the active result.
Close waits for bounded worker cleanup. Restart restores only an explicitly saved URL and never
replays discovery or auto-connects.

## Degraded, error, and recovery behavior

No wired carrier, no UDP traffic, timeout, malformed MAVLink, only sysid 1/2, multiple sysid-3
senders, or port conflict produces a factual visible result. Existing manual URL and Find USB stay
usable. Discovery cannot interrupt P1/P2, cameras, GNSS, Measurement or an existing Drone link.

## Completion and next actions

Discovery completes with a candidate or bounded reason. Connection completes operationally only
after expected `sysid=3` live evidence; socket-open/command completion is not success.

## Training, operating-procedure, and compatibility impact

Operators gain one optional read-only step. Existing saved settings and manual/USB procedures stay
compatible; no migration is needed.

## Workflow-to-screen, command, state, and regression traceability

Telemetry button/list -> CommandRegistry -> backend facade -> DroneTelemetryController -> shared
network discovery worker/adapter -> transient snapshot -> selected URL -> existing connection owner
-> StatusModel/Telemetry live evidence.
