# Frontend Contract

## Controls and inputs

`Find Network` starts a scan. A bounded read-only table shows State, NIC, Source IP, UDP port and
sysid. `Use selected` is enabled for exactly one selected verified sysid-3 row.

## Payloads and commands

`telemetry.discover_network` carries no guessed endpoint. `telemetry.use_network_candidate`
projects a candidate generation/identity into `udpin:0.0.0.0:<observed-port>`; controller validation
must recheck generation and sysid.

## Visible states and transitions

`IDLE -> SEARCHING -> FOUND | EMPTY | AMBIGUOUS | FAILED`. Existing connection status remains a
separate line and does not become FOUND merely because discovery succeeded.

Apply/Save response refresh uses the accepted form value as the authoritative endpoint. A stale
connection snapshot may describe the old connection separately but must not overwrite the editable
URL. Periodic refresh also preserves a dirty/focused edit until it is explicitly applied or reset.

## Errors and recovery actions

State text names missing carrier/traffic, unexpected identity, ambiguity, timeout or stale
selection. Recovery is Retry, select another verified row, manual URL, or Find USB.

## Keyboard, focus, accessibility, and layout

Native Tab order: Find USB, Find Network, Apply, Connect, Disconnect, Save, candidate table, Use
selected. No focus steal or scroll reset on refresh; candidate details have accessible text.

## Cross-feature visible behavior and isolation

P1/P2/Cameras/Base/Measurement controls and status remain unchanged during and after discovery.
