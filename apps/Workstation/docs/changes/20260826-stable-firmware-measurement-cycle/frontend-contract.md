# Frontend Contract

## Controls and inputs

Every operational screen reflects one whole-rig owner identity. Device connect, NTRIP, Detection,
and recording controls cannot independently bypass ownership.

## Payloads and commands

Operational intent first acquires/validates the rig-owner lease, then calls the app's existing
device/measurement command. A rejected lease request sends no device command.

## Visible states and transitions

- `RIG AVAILABLE`
- `RIG OWNED BY WORKSTATION`
- `RIG OWNED BY STEREOWIRIS`
- `RIG OWNER STALE/UNKNOWN` only with factual owner-process evidence and a manual recovery path

Measurement completion retains the existing connected/disconnected states and returns to the
app's current post-save UI.

## Errors and recovery actions

When another owner exists, show its app name, process identity/start time when available, and the
safe action to close/release it, then exit the second app. Do not report camera/Pixhawk connection
failure for an ownership rejection. An orphan case must be labeled separately from a live owner.

## Keyboard, focus, accessibility, and layout

Reuse existing inline status/command feedback. A whole-rig ownership conflict may use one blocking
decision surface, but live status changes must not steal focus or resize the workspace.

## Cross-feature visible behavior and isolation

An ownership rejection occurs before the second operational GUI starts. No second-app review,
settings, waiting, or takeover mode is introduced.

The complete-shutdown path is a terminal command, not a new GUI dashboard. Its output names live
parents, verified children, writer/finalization risk, signals sent, wait results, survivors, and
remaining owned ports/resources.

For the first shutdown-only implementation slice there is no new in-app control, layout, status
label, modal, focus transition, or navigation path. Workstation and StereoWIRIS receive the same
out-of-process close request and continue to present their existing shutdown/finalization UI.
