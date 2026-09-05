# Impact Analysis

## Preserved invariants

- One shared MAVLink subscriber owns actual Drone/P1/P2 sockets.
- Discovery is read-only and never changes PX4, host networking, settings or active connections.
- Only explicit Save persists; only explicit Connect opens a new link.
- Expected role identity remains Drone=`sysid 3`; P1/P2 and partial-connect behavior are unchanged.

## Affected layers and components

- UI: `workspaces.py` local control/candidate projection.
- Command/facade: `main_window.py`, `standalone_backend.py`.
- Controller/state: `drone_telemetry_controller.py`, initialization/shutdown surface in
  `standalone_gui.py` only as required.
- Boundary: reuse/extract bounded read-only network/MAVLink candidate discovery from existing
  runtime workers; no blocking UI I/O.
- Tests/docs: focused Telemetry/controller/worker tests, README and UI product documents.

## Feature dependency, ownership, and coupling changes

Telemetry consumes a narrow immutable discovery result from the existing device-runtime boundary.
It does not access Pixhawk workspace widgets or another feature's private table/state. No new
reverse dependency or resource owner is introduced.

## Worker CPU/GPU resource impact

One bounded low-rate CPU/network discovery worker; no GPU/media work and no unbounded queue. It is
mutually exclusive with its own duplicate request and must not affect preview throughput.

## Compatibility and migration

Existing settings schema remains valid; candidate state is transient. The saved URL remains the
same string field, so restart and older configurations require no migration.

## Rollback and recovery

Code rollback removes the optional discovery surface without changing saved connection semantics.
At runtime the operator can ignore candidates, type a URL manually or use Find USB.

## Risks and assumptions

- Broadcast UDP may reveal a source IP/port while no unicast route is configured; Connect remains
  the independent oracle.
- Multiple Drone transmitters must be shown as ambiguous rather than guessed.
- The currently observed Drone reuses the traditional P2 IP/port. This is accepted only because
  identity is `sysid=3`; simultaneous P2 Ethernet on the same endpoint must be rejected/warned.
