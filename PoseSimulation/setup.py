import os

from setuptools import find_packages, setup


package_name = "pose_simulation"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [os.path.join("resource", package_name)],
        ),
        (os.path.join("share", package_name), ["package.xml", "README.md"]),
        (os.path.join("share", package_name, "launch"), ["launch/pose_simulation.launch.py"]),
        (os.path.join("share", package_name, "config"), ["config/pose_simulation.yaml"]),
        (os.path.join("share", package_name, "rviz"), ["config/pose_simulation.rviz"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="moontaehyung",
    maintainer_email="user@example.com",
    description="Diagnostic-only Gazebo and RTK stereo pose evaluation.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "pose_simulation = pose_simulation.pose_simulation_node:main",
        ],
    },
)
