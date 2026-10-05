# Backend Contract

## Commands and validation

새 command는 없다. read facade는 `after_sequence` 이후 event batch만 반환하며 음수/잘못된
cursor는 안전하게 정규화하거나 명시 오류로 처리한다.

## State, ownership, and concurrency

- immutable event fields: sequence, utc_ns, level, source, device, code, message.
- sequence는 panel process 안에서 단조 증가하며 All ordering의 authoritative key다.
- panel은 maxlen 2,000 bounded store의 유일 writer/owner다.
- worker-thread request는 GUI-thread queue를 한 번 거쳐 event store와 기존 widgets에 각각
  정확히 한 번 반영된다.
- explicit source/device/level이 strict legacy classification보다 우선한다.

## Worker resource budgets, scheduling, and backpressure

새 worker는 없다. UI는 incremental batch를 소비하고 paused pending도 max 2,000으로 제한한다.
overflow/drop count는 반환 batch와 UI에 노출한다.

## External boundaries and workers

카메라, MAVLink, GNSS/NTRIP, FTP protocol/worker는 바꾸지 않는다. 핵심 producer가 이미
알고 있는 side/device만 optional structured metadata로 전달하고, 값이 없으면 추측하지 않는다.

## Results, errors, timeout, retry, and cancellation

batch에는 events, oldest/latest sequence, dropped count를 포함한다. 조회는 blocking I/O를
하지 않는다. malformed event는 전체 stream을 중단하지 않고 명시적으로 제외/보고한다.

## Persistence, restart, and shutdown

structured event store는 in-memory이며 restart 시 초기화된다. 기존 recording
`application.log`/`application_log.jsonl` raw message와 FTP log persistence/lifecycle은
변경하지 않는다. shutdown에서 flush나 새 파일 작업을 추가하지 않는다.
