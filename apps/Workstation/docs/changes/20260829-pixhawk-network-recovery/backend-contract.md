# Backend Contract

## Commands and validation

Discovery는 physical wired links, IPv4/cidr, carrier와 side peer route를 읽는다. Assign은 candidate를
settings에 저장한다. Connect는 interface 존재, address ownership, endpoint와 kernel `dev/src` 일치를
side별 검증한다.

## State, ownership, and concurrency

DeviceRuntimeController가 discovery/assignment/connect orchestration을 소유하고 기존 subscriber가
유일한 socket owner다. discovery/connect/disconnect duplicate와 recording lock을 보존한다.

## Worker resource budgets, scheduling, and backpressure

한 번에 하나의 bounded network discovery worker; subprocess별 timeout과 finite candidate list.

## External boundaries and workers

`ip -j`, sysfs, optional read-only `nmcli`가 oracle이다. 어떤 host network mutation도 실행하지 않는다.

## Results, errors, timeout, retry, and cancellation

candidate tuple 또는 factual error를 반환한다. Connect는 valid disconnected sides만 worker에 넘기고
invalid sides를 UNAVAILABLE로 projection한다. shutdown은 worker completion/cleanup을 기다린다.

## Persistence, restart, and shutdown

side별 interface/local address를 기존 atomic AppSettings JSON에 저장한다. missing fields는 legacy
compatible이고 missing saved interface는 자동 remap하지 않는다.
