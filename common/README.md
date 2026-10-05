# SASL Common

This directory contains assets and contracts intentionally shared across the
four independently launched SASL applications.

- `python/sasl_common/`: UI-neutral Python lifecycle and coordinate helpers.
- `ros/simulation_interfaces/`: Simulation-owned facts consumed across the ROS boundary.
- `ros/pose_simulation_interfaces/`: typed PoseSimulation result messages.
- `firmware/`: reproducible Pixhawk patches, parameters, manifests, and selected flash artifacts.

Application UI, controller, runtime state, recordings, build products, and
downloaded dependencies do not belong here. Python launchers add
`common/python` explicitly; ROS builds must include `common/ros` as a colcon
base path.

Coordinate contract checked: moving these sources does not change NED, ENU,
body-FRD, optical RDF, baseline-center FRD, GPS, or message-field semantics.
