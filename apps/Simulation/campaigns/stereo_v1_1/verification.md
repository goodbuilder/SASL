# Stereo campaign v1.1 verification

The detailed requirement-to-evidence ledger is maintained in the campaign
change packet: [20261008-stereo-campaign-v1-1](../../../../docs/changes/20261008-stereo-campaign-v1-1/verification.md).

Current evidence includes 81 passing targeted tests, a 20-case legacy sweep
plan, a fresh 39/39 read-only source-QC audit, and an actual-data 100-trial
analysis with 6,822 calibration results, 6,732 post-freeze V0 rows, 50
one-factor sensitivity cases, and 36,071 verified checksums. The selected G0
path is P1. QtTest clicked the CampaignPanel's Run configured action button
and the child controller returned `ANALYSIS_COMPLETE`. A separate visible RQT
launch showed the direct input root, repo output root, MC=100, and 39 valid
source slots. Fresh G0/G1 no-flight probes passed and the disk estimate fits
the selected output filesystem. The direct separate-input-root route also
completed a one-trial RQT reprocess with 3,258 calibration results, 2,376 V0
rows, and 12,905 verified checksums. The 100-trial button run used a staging
root with a read-only source link; the direct-root 100-trial form was not
repeated. No new flight was added because the existing 39-run source is reused.

Coordinate contract checked: Gazebo world ENU `[east,north,up]`, PX4 local NED
`[north,east,down]`, camera-link FLU `[forward,left,up]`, and optical RDF
`[right,down,forward]`, with explicit link-to-optical conversion.
