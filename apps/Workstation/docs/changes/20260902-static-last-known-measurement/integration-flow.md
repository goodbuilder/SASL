# Integration Flow

## Action-to-postcondition traces

`receiver valid sample -> authoritative live cache -> Measurement last-valid cache -> readiness ->
existing method selection -> existing pair/attempt payload -> existing evidence fields -> existing
UI ready-method projection`.

## Transition tables

| Prior | Observation | Next | Measurement effect |
|---|---|---|---|
| UNAVAILABLE | first valid sample | LIVE | component becomes eligible |
| LIVE | explicit disconnect or freshness timeout | RETAINED STATIC | last valid sample remains eligible |
| RETAINED STATIC | newer valid sample | LIVE | newer sample replaces retained sample |
| any | application restart | UNAVAILABLE | requires new valid sample |

## Failure and recovery flows

- Never valid/invalid/non-finite: remain unavailable and block the affected live method.
- Disconnect/stale after valid: last-valid input remains eligible without a command.
- Newer valid sample: replaces the affected retained value for subsequent attempts.
- GPS source-role change: clear the affected old-source cache, then require a valid new-source
  sample.
- Restart: process-local caches start empty.

## Cross-feature contracts and dependency direction

Receiver owners publish read models; Measurement consumes them. Measurement does not command or
mutate receivers. Recording persists the unchanged attempt payload. UI consumes readiness only.
