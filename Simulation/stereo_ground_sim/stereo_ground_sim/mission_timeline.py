"""Pure mission timeline calculation for the simulation target."""

from dataclasses import dataclass


@dataclass(frozen=True)
class MissionTimelineConfig:
    profile: str
    start_distance_m: float
    end_distance_m: float
    initial_world_enu_z_m: float
    hover_world_enu_z_m: float
    takeoff_duration_s: float
    first_hover_duration_s: float
    approach_duration_s: float
    final_hover_duration_s: float
    land_duration_s: float


def mission_state(config, elapsed_s):
    """Return mission scalars; altitude/vertical velocity use world ENU (+up).

    ``distance_m`` and ``velocity_x_mps`` are the existing scenario approach
    scalar and its derivative.  They are not PX4 NED or camera-frame vectors;
    conversion to a commanded world pose remains the node's responsibility.
    """

    elapsed = float(elapsed_s)
    start = config.start_distance_m
    end = config.end_distance_m
    initial_z = config.initial_world_enu_z_m
    hover_z = config.hover_world_enu_z_m

    def state(phase, distance, altitude, velocity_x=0.0, velocity_z=0.0):
        return {
            "phase": phase,
            "distance_m": distance,
            "altitude_m": altitude,
            "velocity_x_mps": velocity_x,
            "velocity_z_mps": velocity_z,
        }

    if config.profile == "manual":
        return state("manual_qgc", start, initial_z)

    if config.profile != "takeoff_forward_land":
        if config.approach_duration_s <= 0.0:
            return state("complete", end, initial_z)
        ratio = min(max(elapsed / config.approach_duration_s, 0.0), 1.0)
        distance = start + (end - start) * ratio
        velocity_x = 0.0 if ratio >= 1.0 else (end - start) / config.approach_duration_s
        return state("approach" if ratio < 1.0 else "complete", distance, initial_z, velocity_x)

    takeoff = max(config.takeoff_duration_s, 0.0)
    hover_before = max(config.first_hover_duration_s, 0.0)
    approach = max(config.approach_duration_s, 0.0)
    hover_after = max(config.final_hover_duration_s, 0.0)
    landing = max(config.land_duration_s, 0.0)

    if elapsed < takeoff and takeoff > 0.0:
        ratio = elapsed / takeoff
        return state(
            "takeoff", start, initial_z + (hover_z - initial_z) * ratio,
            velocity_z=(hover_z - initial_z) / takeoff,
        )
    elapsed -= takeoff
    if elapsed < hover_before:
        return state("hover_before_approach", start, hover_z)
    elapsed -= hover_before

    if config.approach_duration_s <= 0.0:
        distance, velocity_x = end, 0.0
    else:
        ratio = min(max(elapsed / approach, 0.0), 1.0)
        distance = start + (end - start) * ratio
        velocity_x = 0.0 if ratio >= 1.0 else (end - start) / approach
    if elapsed < approach:
        return state("approach", distance, hover_z, velocity_x)
    elapsed -= approach
    if elapsed < hover_after:
        return state("hover_after_approach", end, hover_z)
    elapsed -= hover_after
    if elapsed < landing and landing > 0.0:
        ratio = elapsed / landing
        return state(
            "landing", end, hover_z + (initial_z - hover_z) * ratio,
            velocity_z=(initial_z - hover_z) / landing,
        )
    return state("complete", end, initial_z)
