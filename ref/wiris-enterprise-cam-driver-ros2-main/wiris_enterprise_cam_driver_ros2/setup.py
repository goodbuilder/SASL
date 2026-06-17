from setuptools import setup, find_packages

package_name = 'wiris_enterprise_cam_driver_ros2'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(include=[package_name]),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/wiris_service.launch.py',
            'launch/wiris_stream.launch.py',
        ]),
        ('share/' + package_name + '/config', ['config/params.yaml', 'config/full_params.yaml']),
        ('lib/' + package_name, [
            'scripts/wiris_service_node',
            'scripts/wiris_stream_eo_node',
            'scripts/wiris_stream_ir_node',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'wiris_service_node = wiris_enterprise_cam_driver_ros2.wiris_service_node:main',
            'wiris_stream_eo_node  = wiris_enterprise_cam_driver_ros2.wiris_stream_eo_node:main',
            'wiris_stream_ir_node  = wiris_enterprise_cam_driver_ros2.wiris_stream_ir_node:main',
        ],
    },
)