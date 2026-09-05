# Integration Flow

## Action-to-postcondition traces

`Find Network -> command accepted -> worker inventory/passive receive -> decoded sysid rows ->
generation-checked snapshot -> table`. `Use selected -> validate row -> fill URL only`. `Apply ->
runtime config`; `Save -> settings`; `Connect -> socket owner -> sysid=3 HB/ATT/GNSS -> LIVE`.

## Transition tables

Discovery and connection are independent state machines. SEARCHING disables only discovery/use;
an existing Drone connection stays untouched. Connect remains disabled while connection is active.

## Failure and recovery flows

Empty/unexpected/ambiguous result leaves current URL unchanged. Bind conflict reports the owner and
preserves manual recovery. A disconnect during later Connect uses the existing connection failure
path and does not rewrite the candidate or saved URL.

## Cross-feature contracts and dependency direction

Telemetry UI -> command registry/backend facade -> DroneTelemetryController -> feature-neutral
network discovery adapter -> immutable result. Connection continues through the existing shared
Pixhawk subscriber and status/evidence path.
