"""Gazebo world-ENU target pose command ownership."""

import math
import subprocess
import time


def pose_request(model_name, x_m, y_m, z_m, yaw_deg):
    """Encode world ENU position metres and yaw-about-+Z degrees."""

    half_yaw = math.radians(float(yaw_deg)) * 0.5
    return (
        f'name: "{model_name}", '
        f"position: {{x: {float(x_m):.6f}, y: {float(y_m):.6f}, z: {float(z_m):.6f}}}, "
        f"orientation: {{x: 0.0, y: 0.0, z: {math.sin(half_yaw):.9f}, "
        f"w: {math.cos(half_yaw):.9f}}}"
    )


class GazeboTargetController:
    def __init__(self, model_name, *, warning, monotonic=time.monotonic, runner=subprocess.run):
        self.model_name = str(model_name)
        self.warning = warning
        self.monotonic = monotonic
        self.runner = runner
        self.last_command_monotonic_s = 0.0

    def command_mission_state(self, *, spawn_world_enu_x_m, state, yaw_world_enu_deg):
        now = self.monotonic()
        if now - self.last_command_monotonic_s < 0.5:
            return None
        self.last_command_monotonic_s = now
        return self.set_pose(
            spawn_world_enu_x_m,
            state["distance_m"],
            state["altitude_m"],
            yaw_world_enu_deg,
        )

    def set_pose(self, x_m, y_m, z_m, yaw_deg):
        request = pose_request(self.model_name, x_m, y_m, z_m, yaw_deg)
        try:
            completed = self.runner(
                [
                    "gz", "service", "-s", "/world/stereo_ground_drone/set_pose",
                    "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
                    "--timeout", "1000", "--req", request,
                ],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=5.0,
                check=False,
            )
            if completed.returncode != 0:
                self.warning(f"Gazebo set_pose returned exit code {completed.returncode}")
                return False
            return True
        except (OSError, subprocess.SubprocessError):
            self.warning("Failed to command Gazebo target pose; will retry.")
            return False
