# Integration Flow

## Action-to-postcondition traces

`MAVLink/Shell MAG sample` -> existing sensor state -> `EvidenceAdapter` raw vector row -> pure
finite `atan2`/wrap -> `RAW yaw` cell and Inspector.

`SERIAL_CONTROL -> listener sensor_mag instances` -> RM3100 device-ID filter ->
`px4-shell/sensor_mag` state -> existing dedicated CAN store -> five-row IMU projection.

`SERIAL_CONTROL -> listener vehicle_magnetometer` -> active device ID -> existing yaw gate.

`PX4 active vehicle_magnetometer device_id + ATTITUDE` -> existing evidence read model -> exact
row identity match -> `PX4 yaw (selected)` and wrapped `Δyaw`; no exact match -> explicit unavailable
state. A fresh displayed cell is not evidence that priority changed; active-device readback remains
the postcondition.

## Transition tables

| Input condition | RAW yaw | PX4 yaw reference | UI state |
|---|---:|---:|---|
| Complete vector, no active identity | value | identity 확인 대기 | existing LIVE/PARTIAL |
| Complete vector, exact active identity, fresh ATTITUDE | value | value | selected |
| Complete vector, non-selected identity | value | 미선택 | non-selected |
| Missing/near-zero horizontal vector | -- | unchanged/unavailable | existing partial/waiting |
| Stale ATTITUDE or active identity | value if vector fresh | --/STALE | existing stale state |

## Failure and recovery flows

Disconnect, stale sample, missing PARAM/device ID, or fallback sensor leaves the raw telemetry
visible but prevents false PX4 correlation. Reconnect and a fresh exact active-device sample
restore the reference automatically.

Shell timeout, malformed output, or a concurrent manual command leaves the current sample aging;
the bounded poll retries on the next cadence, while manual Shell/PARAM ownership remains intact.

## Cross-feature contracts and dependency direction

Dependency remains `Pixhawk telemetry -> EvidenceAdapter -> PixhawkWorkspace/Inspector`. No new
dependency into Alignment, Measurement, Recording, or shared infrastructure is introduced.
