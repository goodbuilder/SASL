import os
from glob import glob

from setuptools import find_packages, setup


package_name = "pose_simulation"

model_files_by_dir = {}
for path in glob("models/**/*", recursive=True):
    if os.path.isdir(path):
        continue
    target_dir = os.path.join("share", package_name, os.path.dirname(path))
    model_files_by_dir.setdefault(target_dir, []).append(path)
model_files = list(model_files_by_dir.items())

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [os.path.join("resource", package_name)],
        ),
        (os.path.join("share", package_name), ["package.xml", "README.md", "plugin.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            [
                "launch/pose_simulation.launch.py",
                "launch/pose_simulation_ui.launch.py",
                "launch/pose_simulation_rviz.launch.py",
            ],
        ),
        (
            os.path.join("share", package_name, "config"),
            [
                "config/pose_simulation.yaml",
                "config/pose_simulation_ui.yaml",
                "config/pose_simulation_visualization.yaml",
                "config/pose_simulation.rviz",
                "config/pose_contract.yaml",
                "config/posesimulation.env",
            ],
        ),
        (os.path.join("share", package_name, "worlds"), ["worlds/stereo_ground_drone.sdf"]),
    ] + model_files,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="moontaehyung",
    maintainer_email="user@example.com",
    description="Independent read-only ROS 2 consumer for external pose diagnostics.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "pose_simulation = pose_simulation.gt_attitude_measurement_node:main",
            "pose_visualization = pose_simulation.pose_visualization_node:main",
            "pose_input_probe = pose_simulation.input_probe_node:main",
        ],
    },
)
