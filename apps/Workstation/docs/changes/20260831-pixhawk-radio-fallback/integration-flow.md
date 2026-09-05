# Integration Flow

`Connect side` -> read selected transport -> transport-specific preflight -> open one endpoint ->
verify expected sysid -> request the selected stream profile -> enable side RTCM only after current
liveness evidence -> publish transport-labelled status.

`RTCM frame` -> resolve side -> verify current connection generation and eligibility -> send through
the selected MAVLink connection -> record intent and actual rover correction/fix evidence.

`Disconnect` -> stop RTCM eligibility -> close the selected owner -> clear process-local health ->
leave the alternate transport configured but inactive.

`Change transport while connected` -> side-only orderly disconnect -> retire old generation and
pending RTCM -> show selected endpoint as disconnected -> wait for explicit operator `Connect`.

`Application startup` -> load Radio-default settings -> inspect endpoint availability without
opening serial devices -> render both sides disconnected -> wait for independent `Connect` actions.

`Connect with missing saved Radio` -> discover candidates -> operator selects one -> bounded
heartbeat identity probe -> reject mismatch/busy/duplicate or persist verified endpoint -> continue
the same explicit side Connect flow.

`Radio overload` -> keep critical telemetry/RTCM -> suppress optional demand -> show degraded state
-> observe stable recovery interval -> restore one optional tier -> re-evaluate before next tier.

`Verified Radio identity` -> start critical profile and RTCM -> qualify for 10 continuous seconds ->
publish transport LIVE -> after 30 healthy seconds restore one optional tier -> observe 15 seconds
between later tiers; any regression returns to RTCM priority.

`Active Radio removed` -> revoke side RTCM -> retire generation and pending work -> show disconnected
-> device may reappear as available -> wait for explicit Connect -> reverify identity and requalify.

`Stage firmware over Radio` -> explicit file selection -> transfer with progress -> remote size/hash
readback -> mark STAGED -> wait for verified USB -> explicit USB flash -> erase/program/verify/reboot
evidence -> read running firmware identity.

`Bulk transfer interrupted` -> preserve local/remote provenance -> verify partial offset/integrity ->
show Resume and Restart or disable Resume with reason -> wait for operator action -> complete full
size/hash verification before success.

`Find USB` -> reject if the same discovery is already active, otherwise clear transient candidates,
selection, feedback and old status -> probe deduplicated physical serial endpoints -> publish one
row per endpoint with heartbeat identity -> show only the new USB snapshot -> wait for explicit
assignment/connect.

`Find network` -> reject if network discovery is already active, otherwise clear transient
candidates, selection, feedback and old status -> read NIC/IPv4/route evidence -> show only the new
network snapshot -> preserve saved assignments -> wait for explicit assignment/connect.

## Guided P1 Radio · P2 Ethernet · Drone Radio amendment

`Check` -> reject active Measurement/recording, armed/owned/ambiguous endpoints, or another setup
generation -> read settings/journal -> passive USB/radio/network inventory -> publish five-row
evidence -> perform no writes.

`Start guided setup` -> freeze requested preset and pre-operation settings -> require current Check
evidence -> persist operation generation -> enable only the first unproven safe action.

`Drone Radio step` -> require only Drone sysid3 on D30 and no P2 sysid2 on DU -> show exact
NETID25→26 intent -> operator confirmation -> remote AT set/query/save -> persist readback -> wait
for full power reapply -> verify sysid3 only on DU -> advance.

`P2 Ethernet step` -> require P2 air radio absent and unique disarmed USB sysid2 when provisioning
is needed -> validate/provision `10.0.0.3:14551` -> save/readback/reboot as required -> require host
NIC/carrier/route plus expected sysid2 UDP evidence -> advance.

`Save topology` -> revalidate both postconditions and prior settings generation -> atomically persist
P1 Radio/D30, P2 Ethernet/network assignment, Drone Radio/DU -> publish READY -> wait for existing
explicit per-board Connect actions.

`Restart with active record` -> load intent/evidence only -> show PARTIAL -> run a new read-only
Check -> offer Resume from first unproven postcondition or Guided rollback -> never repeat a write
or assume a power cycle.

`Guided rollback` -> revalidate identity/disarm/isolation -> apply only independently justified
inverse steps -> restore prior Workstation settings after device postconditions -> mark COMPLETE
only after readback and cross-endpoint verification; otherwise remain PARTIAL.

`P2 Ethernet failure after completion` -> retire only P2 Ethernet generation and RTCM eligibility ->
show P2 DISCONNECTED -> leave P1 Radio and Drone Radio untouched -> wait for explicit P2 recovery;
no automatic Radio takeover.
