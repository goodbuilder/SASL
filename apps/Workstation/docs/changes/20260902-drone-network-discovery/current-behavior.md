# Current Behavior

## Version and worktree context

- Repository baseline: `3288aee`; worktree already contains extensive user-owned changes.
- Relevant files are already modified, so implementation must patch narrowly and preserve those diffs.
- Runtime: Workstation PID 159720, Measurement/recording idle during baseline observation.

## Reproduction

1. Physically attach the Drone Pixhawk Ethernet link to `enx0023a40a2e39`.
2. Open `Devices > Telemetry`.
3. Observe that only `Find USB` exists and the saved endpoint is the former DU radio path.
4. Passively capture the wired NIC and independently decode MAVLink on UDP 14551.

## Observed frontend behavior

- Controls: `Find USB`, `Apply`, `Connect`, `Disconnect`, `Save`.
- Status showed `CONNECTED · heartbeat=WAITING` on the stale DU serial endpoint.
- No local network candidate/result surface exists.

## Observed backend and persistence behavior

- `drone_pixhawk_transport` is already `ethernet`.
- Before this change request, `drone_telemetry_url` pointed at the DU serial device and
  `drone_pixhawk_eth_ip`/interface fields were empty.
- Passive capture observed sustained UDP from `10.0.0.3:14551` to broadcast `10.0.0.255:14551`.
- Independent pymavlink decoding over five seconds observed `sysid=3/component=1`, five
  HEARTBEATs plus current ATTITUDE/GPS traffic.
- During diagnosis the operator-approved URL was saved as `udpin:0.0.0.0:14551`; the running
  stale serial connection remained active until explicit Disconnect.

## Documentation, code, test, and runtime conflicts

- Product docs describe read-only `Find network` for P1/P2 but not the local Telemetry surface.
- `TelemetryWorkspace` exposes USB discovery only.
- `DroneTelemetryController` owns USB discovery and connection state but no network discovery.
- `DeviceRuntimeController` already owns shared Pixhawk NIC/route discovery; new work must reuse
  a narrow public result rather than create a second socket or network owner.
