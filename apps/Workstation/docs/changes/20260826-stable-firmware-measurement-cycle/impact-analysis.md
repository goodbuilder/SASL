# Impact Analysis

## Preserved invariants

- Exactly one owner for every camera/MAVLink/NTRIP/writer resource.
- Standard Stable-PX4 telemetry is the normal measurement boundary.
- Calibration remains immutable input until explicitly replaced.
- At most one live Measurement writer per app and per physical rig.
- No coordinate/frame/unit reinterpretation.

## Affected layers and components

- Both launchers, settings, MAVLink subscribers, camera workers, GNSS/RTCM paths, measurement
  commands/status, recording lifecycle, shutdown, docs, and tests.
- The first approval candidate is intentionally narrower: repository `tools`, shared
  `sasl_common` runtime identity/control, both application entry points/top-level close hooks, owned
  child registration, launcher help/aliases if needed, focused lifecycle tests, and operator docs.

## Feature dependency, ownership, and coupling changes

The main cross-app risk is duplicate direct ownership of identical network/device endpoints. The
accepted single-owner policy uses one feature-neutral cross-process ownership guard; no broker,
fanout, shared writer, or cross-app private-state access is introduced.

## Worker CPU/GPU resource impact

Two full camera decode/YOLO stacks are forbidden for one rig under the accepted policy. CPU/GPU,
memory-bandwidth, and RTSP load therefore remain the selected app's current behavior. The guard
must prevent a second stack before workers start.

## Compatibility and migration

Existing settings use the same endpoint defaults. Stable-firmware mode must retain readability of
old custom-firmware evidence while marking private stream fields unavailable rather than false.

## Rollback and recovery

Pending design. Existing app-specific recording formats are preserved initially.

## Risks and assumptions

- Stable PX4 version/parameters and camera calibration are assumed correct but still require
  readback evidence.
- “Repeat all except calibration” does not yet define whether ROS2/uXRCE or physical alignment is
  mandatory each cycle. App restart and forced transport-mode changes are
  explicitly not mandatory.
- The remaining process-safety decision is whether verified orphan children are terminated only
  after operator confirmation, require manual terminal cleanup, or are auto-terminated.
- An explicit shutdown script reduces startup surprise but remains destructive when an app is
  actively writing. The accepted policy therefore displays writer/finalization risk, requires
  confirmation by default, and permits escalation only with explicit `--force`.
- Startup cleanup cannot repair a Pixhawk firmware-side MAVLink task or camera-server state; it only
  releases verified local processes/resources. Device recovery remains separately evidenced.
- The shutdown script likewise repairs only local process/resource ownership; it cannot reboot or
  repair Stable PX4, camera firmware, NTRIP, or the network.
- A dead parent may leave incomplete runtime metadata. Such metadata is diagnostic input only; a
  target must still pass live `/proc` identity and ownership verification before any signal.
- A process can refuse or outlive graceful shutdown. Default mode reports that condition and exits
  nonzero; only the explicit force mode may continue.
