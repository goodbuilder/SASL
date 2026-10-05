# Integration Flow

## Action-to-postcondition traces

```text
Start -> freeze run id/time -> RUNNING -> request sources independently
source current evidence -> JOINED -> collect -> method-local gates -> result or BLOCKED
source failure -> UNAVAILABLE -> run continues -> manual recovery -> newer evidence -> JOINED
Stop -> freeze end -> retire generation -> request teardown -> next Start remains admissible
late finalization -> old generation artifact evidence only
```

## Transition tables

| Condition | Run | Source/result |
|---|---|---|
| zero sources | RUNNING | all PENDING/UNAVAILABLE |
| one source fails | RUNNING | that source UNAVAILABLE |
| recovery | RUNNING | source JOINED at evidence time |
| storage fails | RUNNING | bounded core retry; drops visible |
| Stop | IDLE boundary | teardown/finalization may continue |

## Failure and recovery flows

Busy resources auto-join; actual camera failure needs explicit reactivation. A recovered recording
side writes a new segment. No source failure rolls back the observation interval.

## Cross-feature contracts and dependency direction

Device owners publish transitions to Run; Recording publishes segments; DataStorage publishes
health. Run never reads widget or storage/media integrity state for admission.
