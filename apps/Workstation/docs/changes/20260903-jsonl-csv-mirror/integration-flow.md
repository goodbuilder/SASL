# Integration Flow

## Action-to-postcondition traces

```text
Measurement/record producer
  -> DataStorageController.append_workstation_data(stream, payload)
  -> MeasurementDataStore canonical JSON validation/enqueue
  -> existing writer selects priority/bulk item
  -> JSONL append succeeds (authoritative postcondition)
  -> same canonical record CSV envelope append/retry
  -> per-stream snapshot records JSONL + CSV state/count/error
  -> Data > Status renders paths and factual degradation
```

The UI never treats enqueue acceptance as file success; tests inspect bytes with independent JSON and
CSV readers.

## Transition tables

| JSONL result | CSV result | Stored outcome | UI/recovery |
| --- | --- | --- | --- |
| accepted/written | written | complete mirror row | normal/ACTIVE |
| accepted/written | retryable failure | JSONL authoritative, CSV pending | DEGRADED/Retry Storage |
| accepted/written | terminal drop | JSONL authoritative, CSV missing row | DEGRADED/drop count |
| rejected/dropped | not attempted | no persisted record | existing JSONL error |
| restart | existing valid header | append continues | ACTIVE after write |
| foreign/invalid CSV | JSONL unaffected | mirror failed | fix destination + Retry Storage |

## Failure and recovery flows

- Permission/full-storage: JSONL state and CSV state are tracked separately; Measurement remains
  usable; retry after the path is repaired.
- Malformed producer payload: existing canonical validation rejects the record before either file
  receives it.
- Partial/torn historical JSONL: existing JSONL behavior remains; no CSV row is invented from a bad
  line.
- Duplicate/retry: the JSONL retry path does not duplicate CSV rows; a CSV retry reuses the already
  canonical record without re-appending JSONL.
- Close: active queues flush; bounded CSV retry leftovers become explicit drops rather than holding
  application shutdown indefinitely.

## Cross-feature contracts and dependency direction

Data Storage owns the mirror. RecordingWorkspace reads its public snapshot. Measurement, Camera,
Pixhawk, GNSS, WIRIS, and XLSX code do not import or write CSV handles. Coordinate fields are copied
as producer-authored values; NED, ENU, body-FRD, optical-RDF, stereo-center, and GNSS baseline
contracts remain distinct inside `record_json`.
