# Backend Contract

## Commands and validation

For a two-sided non-FTP Live/Auto capture without a verified exposure timestamp:

1. Read Left/Right `app_receive_monotonic_ns`; normalize the owned native-frame compatibility `timestamp_s`
   only when the exact field is absent.
2. Reject missing, non-numeric, non-finite, non-positive, or incomparable values.
3. Compute signed `pair_delta_s = (right_ns - left_ns) / 1e9`.
4. Allow pair persistence iff `abs(pair_delta_s) <= maximum_delta_s`.
5. Do not substitute RTCP/NTP when app receive is unavailable.

Verified exposure timestamps retain highest priority. FTP capture IDs retain their synchronized CPTR-dispatch
contract. Single-sided capture retains the existing UNKNOWN/K,D-only path.

## State, ownership, and concurrency

Camera stream owns receive timestamp creation. Calibration controller owns pair eligibility. Checkerboard worker
owns encoding/detection/persistence. Existing single calibration-operation slot, duplicate rejection, Auto timer,
completion/failure cleanup, and shutdown behavior are unchanged.

## Worker resource budgets, scheduling, and backpressure

No resource-policy change. Timing is frozen synchronously before the existing background worker starts; no new
queue or unbounded state is introduced.

## External boundaries and workers

The host timestamp is sampled after GStreamer appsink yields a decoded frame, before image copying. It is not raw
RTP packet arrival, WIRIS exposure, UTC, RTCP/NTP, or GStreamer PTS. Both camera processes use the same host
monotonic clock domain.

## Results, errors, timeout, retry, and cancellation

- Evidence class: `HOST_RECEIVE_PROXY`
- Timestamp reference: `app-receive-monotonic`
- `precision_qualified=false`, `actual_exposure_verified=false`
- Missing/invalid: `UNKNOWN`, pair persistence false, explicit detail
- Threshold accepted at equality; gate compares absolute value and preserves signed delta
- Existing capture worker timeout/cancel/error behavior remains unchanged

## Persistence, restart, and shutdown

`capture_metadata/capture_<id>.json` records the existing fields plus per-side app-receive monotonic ns and
the receive-proxy evidence. Existing files are never rewritten. Input snapshots hash the resulting metadata and
carry the evidence class; restart reads the persisted allow/reject decision without recomputing it.
