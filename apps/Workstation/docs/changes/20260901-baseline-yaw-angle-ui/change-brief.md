# Change Brief

## Operator outcome

On `Devices > Telemetry`, directly below the existing `Live position` group, the operator can
read the horizontal angle formed by the selected P1→P2 GNSS baseline and the Drone GNSS vector
from the P1/P2 antenna midpoint, then adjust the camera installation until the angle reaches 90°.

## Motivation

The current Telemetry workspace shows only the GNSS-derived midpoint-to-Drone `F/R/D/Rg` values.
The operator must calculate the baseline-to-Drone horizontal angle mentally. The Drone Pixhawk
ATTITUDE/yaw is not part of this installation-alignment contract.

## Included scope

- GNSS-only horizontal baseline-to-Drone angle and absolute error from the 90° target.
- Source-backed CURRENT/STALE/WAITING presentation with the existing three-GNSS age/spread evidence.
- Two additional read-only columns in the existing fixed-height `GNSS BASELINE_CENTER` table.
- Pure geometry tests, controller/read-model tests, Qt integration tests, and operator documentation.

## Excluded scope and non-goals

- No Drone ATTITUDE/yaw input, target acceptance verdict, automatic camera movement, history, or stability statistic.
- No command, worker, device/protocol, measurement algorithm, setting, or navigation change.
- No new persisted derived-angle field or recording schema change; existing F/R/D/Rg records remain unchanged.
- No replacement of raw NED, P1 body FRD, camera RDF, or public measurement coordinates.

## Acceptance summary

Fresh finite P1/P2/Drone GNSS inputs produce the horizontal angle and 90° error at the next normal
UI poll. Missing or stale sources remain explicit, rapid updates do not move controls or scroll,
and existing raw telemetry/recording behavior is unchanged.
