# Backend Contract

## Commands and validation

Discovery enumerates current physical wired interfaces/carrier/address/route evidence and passively
observes bounded UDP/MAVLink candidates. A selectable candidate requires valid source IPv4, UDP
port 1..65535 and independently decoded HEARTBEAT `sysid=3`.

## State, ownership, and concurrency

DroneTelemetryController owns one discovery lifecycle/generation and transient result. Existing
subscriber remains sole connection owner. Duplicate discovery is rejected; stale results cannot
overwrite a newer generation or edited URL.

## Worker resource budgets, scheduling, and backpressure

One off-GUI-thread worker, bounded interface list, candidate count and timeout. No GPU, no media
decoder, no periodic background scan and no queue growth.

## External boundaries and workers

Use existing read-only OS network inventory and passive MAVLink parsing. Do not invoke tcpdump,
sudo, NetworkManager mutation, PX4 commands, socket connection ownership or serial probing.

## Results, errors, timeout, retry, and cancellation

Return immutable factual rows and reason. Malformed datagrams are ignored/counted. Timeout/empty,
unexpected sysids, ambiguity, bind conflict and shutdown cancellation are distinct. Close cancels
or joins bounded work and late callbacks are generation-rejected.

## Persistence, restart, and shutdown

Discovery state is not persisted. Existing Save persists only the chosen `drone_telemetry_url` and
other existing Telemetry inputs atomically. Restart is disconnected and requires explicit Connect.
Apply returns the newly accepted configuration endpoint, and Save persists that exact accepted
value before returning. An old connection object's endpoint cannot replace either result.
