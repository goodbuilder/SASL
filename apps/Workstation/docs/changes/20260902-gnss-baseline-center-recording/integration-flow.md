# Integration Flow

## Action-to-postcondition traces

```text
primary Drone GPS callback
  -> latest_gps["drone"] updated + existing px4.drone raw row attempted
  -> RecordingController snapshots selected P1/P2 + exact Drone
  -> pure 2.0 s / 1000 ms estimator
  -> self-contained row
  -> non-blocking append("gnss.baseline.center")
  -> writer JSONL bytes / Data Status evidence
  -> stop checkpoint
  -> optional Latest XLSX GnssBaselineCenter sheet
```

## Transition tables

| Recording | Source geometry | Outcome |
|---|---|---|
| inactive | any | no row |
| active | CURRENT | finite row |
| active | STALE | finite row + reason/timing |
| active | WAITING | null geometry row + partial source evidence |
| active | writer rejects | no byte claim; stream drop/error evidence |

## Failure and recovery flows

Queue full/write error -> existing DEGRADED/FAILED stream -> raw/Telemetry/Measurement continue ->
operator Retry Storage -> future rows resume. XLSX failure -> JSONL unchanged -> Update Latest XLSX.

## Cross-feature contracts and dependency direction

Sensor -> Recording -> pure geometry -> Data store -> derived XLSX. No reverse dependency; no UI
widget, Measurement result, or raw PX4 stream owns derived persistence.
