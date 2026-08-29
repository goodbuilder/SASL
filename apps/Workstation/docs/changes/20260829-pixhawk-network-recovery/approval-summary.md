# Approval Summary

This document explains the proposed modification in plain language. Technical detail remains in the other packet documents.

## What changes and what stays the same

`Find network` becomes real NIC/address/route discovery. Assign saves P1/P2 choices; Connect uses them.
Existing device truth, liveness, USB recovery, Camera ownership and Measurement rules stay the same.

## Operator workflow before and after

Before: generate generic URLs and diagnose NICs externally. After: Find -> select -> Assign P1/P2 -> Connect.

## Success, delay, rejection, failure, overload, and recovery

One failed side does not close the other. Missing/wrong host routes fail before socket creation. Socket open
is not success; existing expected heartbeat/ATTITUDE and 60-second LIVE remain authoritative.

## Data and compatibility effects

Four additive settings fields remember side interface/local address. Old settings continue to work.

## Performance priorities and allowed degradation

Discovery is bounded and off the GUI thread. No GPU/media resource change.

## Visual and interaction changes, including dynamic-layout stability

The existing compact candidate table moves near Connection and gains fixed network evidence columns.
Selection/focus/scroll remain stable under status refresh.

User clarification supersedes the paired route/endpoint columns: discovery rows show NIC candidates
only, while independent P1/P2 assignment summaries show each side's State, NIC, Local IPv4,
Endpoint and Route.

## How acceptance will be demonstrated

Pure/backend/UI/integration tests, launcher and portability checks, target-size screenshots, and real
P1/P2 route/heartbeat checks where hardware is available.

## Cost, risks, assumptions, rollback, and field limits

Workstation will not fix OS networking. Same Camera/Pixhawk NIC is silently permitted by explicit user
decision even though historical long-load instability remains unqualified.

## Exact scope awaiting approval

Approved scope: the `20260829-pixhawk-network-recovery` packet described above. No host or PX4 network
mutation, firmware work, coordinate/measurement change, or unrelated cleanup is authorized.
