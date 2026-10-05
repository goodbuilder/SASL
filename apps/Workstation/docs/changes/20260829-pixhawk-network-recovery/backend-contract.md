# Backend Contract

## Commands and validation

Discovery는 physical wired links, IPv4/cidr, carrier와 side peer route를 읽는다. Assign은 candidate를
settings에 저장한다. Connect는 interface 존재, address ownership, endpoint와 kernel `dev/src` 일치를
side별 검증한다.

Amendment: a separate operator-confirmed role-application command may mutate only the selected
physical NIC's app-owned IPv4 addresses/routes through NetworkManager. It must capture current
state, validate the role template and selected identity, request bounded privilege, apply, then
independently read back both NetworkManager and kernel state before persisting `APPLIED`.

Roles compose by union. The current fixed role entries are Camera local `10.0.0.11/32` with
`.231/.232` preferred-source host routes, P1/P2 local `10.0.0.10/24` with side `.2/.3` routes, and no
default route. A NIC may hold all of them. Shared entries such as the Pixhawk local address are
reference-owned by remaining P1/P2 roles and are removed only when no role still requires them.
Physical NIC inventory is finite but runtime-variable. The role-state model maps stable discovered
identities to role sets and derives the desired per-NIC entry union without a configured NIC count
or interface-name role convention.

## State, ownership, and concurrency

DeviceRuntimeController가 discovery/assignment/connect orchestration을 소유하고 기존 subscriber가
유일한 socket owner다. discovery/connect/disconnect duplicate와 recording lock을 보존한다.

## Worker resource budgets, scheduling, and backpressure

한 번에 하나의 bounded network discovery worker; subprocess별 timeout과 finite candidate list.

## External boundaries and workers

`ip -j`, sysfs and read-only `nmcli` remain discovery/readback oracles. Mutation uses a narrowly
scoped NetworkManager operation only after operator confirmation; it must not invoke PX4 mutation,
replace unrelated NIC configuration, or create a default route.
The mutation owner calculates a delta between the union of desired app roles and current app-owned
entries; it never treats the whole NetworkManager profile as Workstation-owned.

## Results, errors, timeout, retry, and cancellation

candidate tuple 또는 factual error를 반환한다. Connect는 valid disconnected sides만 worker에 넘기고
invalid sides를 UNAVAILABLE로 projection한다. shutdown은 worker completion/cleanup을 기다린다.

Role application must fail closed on stale candidate identity, active conflicting operation,
cancelled/denied authentication, NetworkManager error, timeout or readback mismatch.
Adding a role is atomic with respect to existing roles: any failure restores the pre-apply
app-owned entry set and does not intentionally stop the already-present role.

Cross-NIC movement snapshots both NICs, applies and verifies the target first, then removes only the
moved role's source entries and verifies both the target and every retained source role. Assignment
persistence commits last. Target failure leaves the source untouched. Source-cleanup failure rolls
back the target role and readbacks the original arrangement; rollback failure persists no false
assignment and reports exact partial host state for explicit recovery.

Before mutation, the coordinator snapshots active Camera/P1/P2 owners on source and target NICs. It
uses their existing public asynchronous stop/disconnect contracts, waits for bounded owner release,
runs the host transaction, then restarts exactly that prior active set through existing public
start/connect contracts. Unaffected-NIC owners are never stopped. One host-role owner rejects or
coalesces duplicate operations and stale generations.

Camera restart success is a decoded frame from the new stream generation. Pixhawk restart success
is fresh expected-system HEARTBEAT and ATTITUDE; its established 60-second qualification remains
the readiness transition. Failure before these postconditions rolls back roles and attempts to
restore the original active-owner set. Measurement/recording/save/finalization rejects before any
owner stop or host mutation.

The user retained this automatic device-evidence rollback policy on 2026-08-30. A missing required
device on the target must restore the previous host role and leave assignment persistence unchanged;
the alternative of committing `ASSIGNED · DEVICE NOT SEEN` is excluded. The implementation must be
revisited to derive the required-evidence set from the pre-operation health snapshot rather than the
whole affected-interface open set. The selected role and other affected owners that were healthy
immediately before mutation are required; an already-offline unrelated owner is excluded, remains
offline and is not auto-started. Camera health is a current-generation frame; Pixhawk health is the
expected system ID with fresh HEARTBEAT and ATTITUDE. How a selected role with no pre-operation live
session behaves in the normal Assign-before-Connect workflow remains open.

## Persistence, restart, and shutdown

side별 interface/local address를 기존 atomic AppSettings JSON에 저장한다. missing fields는 legacy
compatible이고 missing saved interface는 자동 remap하지 않는다.
