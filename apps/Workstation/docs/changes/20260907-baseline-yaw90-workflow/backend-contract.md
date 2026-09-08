# Backend Contract

baseline_yaw90_snapshot returns read-only installation evidence. Azimuth derives from selected P1/P2 NED now; Drone and attitude are optional for it. Vertex angles remain unsigned [0,180] in horizontal N/E. R/P is body FRD in NED. apply_baseline_yaw90_settings accepts only the original boolean key, checks runtime idle before writing, then optionally invokes existing persistence. No DOF-fill policy change, method selection or recording schema change.
