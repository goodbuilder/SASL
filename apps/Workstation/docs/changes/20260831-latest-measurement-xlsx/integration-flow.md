# Integration Flow

## Action-to-postcondition traces

Automatic: Measurement Stop -> state becomes `IDLE` -> stop evidence/flush request -> immutable
generation request -> checkpoint -> generation-filtered workbook -> validation/atomic replace ->
READY snapshot -> Data status/open action.

Manual: Update click -> request accepted/rejected -> same single owner -> READY/FAILED status.

## Transition tables

| Event | Prior | Next | Postcondition |
| --- | --- | --- | --- |
| stop/latest request | any non-generating | WAITING/STALE | newest target retained |
| idle + quiet | WAITING/STALE | GENERATING | one worker active |
| verified replace | GENERATING | READY | published=target |
| matching late row | READY/GENERATING | STALE | one regeneration pending |
| error | WAITING/GENERATING | FAILED | previous output preserved |
| new recording/shutdown | WAITING/GENERATING | CANCELLED/WAITING | temp removed, no gate |

## Failure and recovery flows

Missing source -> empty source sheet + Summary `#N/A`; malformed row -> Issues; checkpoint pending ->
partial flag/retry; foreign/locked output -> FAILED/manual recovery; close -> cancel/join/cleanup.

## Cross-feature contracts and dependency direction

MeasurementRunController -> public DataStorageController request. Data workspace -> public request
and snapshot. Export core -> checkpoint/read-only JSONL. No reverse dependency into Measurement UI
or device controllers.
