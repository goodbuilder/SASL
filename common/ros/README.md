# Common ROS interfaces

The interface packages in this directory are shared across application
boundaries. Include this directory in colcon discovery, for example:

```bash
colcon build \
  --base-paths common/ros PoseSimulation Simulation \
  --packages-select simulation_interfaces pose_simulation_interfaces pose_simulation
```

`apps/Simulation/run_operator_view.sh` includes `common/ros/simulation_interfaces`
automatically when it builds the Simulation runtime.

These message definitions preserve their existing field names, units, frame
labels, and orientation conventions; only their repository location changed.
