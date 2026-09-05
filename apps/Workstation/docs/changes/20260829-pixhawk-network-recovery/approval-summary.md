# Approval Summary

This is the revised approval request for explicit host-network role application. The completed
read-only Find/Assign/Connect implementation remains the baseline until this amendment is approved.

## What changes

The operator can select any discovered physical wired NIC and explicitly assign Camera, P1 or P2
to it. After a native before/after confirmation and OS authentication, Workstation applies only the
address/routes required by that role and proves the resulting NetworkManager and kernel state.
Find, row selection and startup never change networking by themselves.

NIC names, models and count are not roles. One NIC may carry Camera+P1+P2 together, or any number of
NICs may split those roles. Applying a role composes with roles already on that NIC and preserves
unrelated operator/system configuration.

## Operator workflow

`Find network -> select NIC -> Assign Camera/P1/P2 -> review exact changes and affected active
devices -> confirm/authenticate -> apply and verify -> reconnect -> device evidence`.

If the role is active, a healthy connection does not block the requested move. Workstation stops
only Camera/Pixhawk owners on source/target NICs, moves the role, and restarts exactly those that
were active. Devices on unrelated NICs continue. A previously stopped Camera or disconnected
Pixhawk is not started automatically.

## Success, failure and rollback

Target NIC settings are applied and verified before the moved role is removed from the source NIC.
Only that role's Workstation-owned entries are removed; other roles and unrelated settings remain.
Assignment persistence commits last.

When reconnecting an active role, Camera success requires a current-generation decoded frame and
Pixhawk success requires its expected system ID with fresh HEARTBEAT and ATTITUDE. The existing
60-second Pixhawk qualification remains visible and is not bypassed. Failure rolls back the host
role and attempts to restore the original active devices. An incomplete rollback shows exact
partial state and recovery guidance; it is never reported as success.

Measurement, recording, save and finalization retain their existing mapping lock and reject a role
change before any device is stopped or host setting is changed.

## Preserved behavior and excluded scope

Existing expected-sysid/liveness/RTCM/partial-connect, Camera frame truth, USB recovery, settings
compatibility, Measurement rules and unrelated NIC operation remain. This amendment does not alter
PX4 firmware, parameters or SD files; Camera decoding; MAVLink parsing; GNSS/coordinates;
Measurement algorithms; media/CPU/GPU policy; or unrelated host settings. Same-NIC topology remains
allowed without a warning/readiness gate, while historical long-load evidence remains unqualified.

## UI, performance and operations

The current compact tables/cards and native Qt style remain. One confirmation names current and
proposed roles, exact address/routes and active devices that will restart. Bounded status text shows
STOPPING/APPLYING/VERIFYING/RECONNECTING/ROLLING BACK without moving selection, focus outside the
confirmation, scroll, dock or controls. Network work stays off the GUI thread and introduces no
GPU/media load change.

The privileged operation is narrowly scoped to Workstation-owned deltas on source/target NICs. It
never creates a default route, silently applies at startup, or treats an entire existing profile as
application-owned.

## Verification and residual field conditions

Approval authorizes pure transaction/ownership tests, worker/controller/UI/integration failure
matrices, the existing full regression and launcher/visual checks, and real NetworkManager/device
verification for one combined NIC and representative multi-NIC moves. Hardware evidence must show
actual routes plus Camera frames and expected-system Pixhawk traffic; fakes cannot close that field
boundary. Shared-NIC long-duration stability remains a separate field qualification.

## Exact approval scope

Approve the NetworkManager role-application amendment described here and in this packet. The prior
read-only source and unrelated dirty-worktree changes are preserved. No source implementation begins
until the user explicitly approves this revised summary.
