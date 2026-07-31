import os
from glob import glob

from setuptools import setup

package_name = "stereo_ground_sim"

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
    packages=[package_name, "sasl_common"],
    package_dir={"sasl_common": "../../sasl_common"},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name, ["plugin.xml"]),
        ("share/" + package_name + "/launch", ["launch/stereo_ground_sim.launch.py"]),
        ("share/" + package_name + "/rviz", ["rviz/stereo_ground_sim.rviz"]),
        ("share/" + package_name + "/worlds", ["worlds/stereo_ground_drone.sdf"]),
    ]
    + model_files,
    install_requires=["setuptools", "XlsxWriter>=3.0.2,<4"],
    zip_safe=True,
    maintainer="moontaehyung",
    maintainer_email="user@example.com",
    description="Ground-fixed stereo camera simulation for drone distance measurement.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "stereo_distance_node = stereo_ground_sim.stereo_distance_node:main",
            "gazebo_ground_truth_adapter = stereo_ground_sim.gazebo_ground_truth_adapter:main",
            "gazebo_camera_pose_diagnostics = stereo_ground_sim.camera_pose_diagnostics:main",
            "camera_px4_pose_diagnostics = stereo_ground_sim.camera_px4_pose_diagnostics:main",
            "detection_overlay_node = stereo_ground_sim.detection_overlay_node:main",
            "external_bbox_publisher = stereo_ground_sim.external_bbox_publisher:main",
            "operator_recorder = stereo_ground_sim.operator_recorder:main",
            "calibration_csv_recorder = stereo_ground_sim.calibration_csv_recorder:main",
            "simulation_experiment_context = stereo_ground_sim.experiment_context_publisher:main",
        ],
    },
)
