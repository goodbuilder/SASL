# Simulation experiment context

The Simulation launch publishes one immutable
`simulation_interfaces/msg/SimulationExperimentContext` message on
`/simulation/experiment_context` with reliable, transient-local, keep-last-1
QoS. It is a source contract for independent consumers; it does not start or
control PoseSimulation.

The message is built from the final launch-resolved values used to construct
the generated world. Nominal optical poses come from the same
`nominal_camera_poses` helper used by world generation, before optical error
injection. Direct, common, and relative optical RPY values are published in
degrees and the publisher recomputes the effective values using:

```text
left  = left_direct  + common - relative / 2
right = right_direct + common + relative / 2
```

The context hash is a canonical SHA-256 over the immutable source contract,
including nominal poses, error inputs, camera contract, frame/rotation
conventions, generated-SDF identity, and provenance. PoseSimulation
recomputes this hash and fails closed on mismatch or same-run mutation.

`simulation_epoch` is currently `0` with `simulation_epoch_valid=false` because
the launch has no public epoch interface. An unresolved default `zoom_index`
is represented as `0` with `zoom_index_valid=false`, never as NaN.
