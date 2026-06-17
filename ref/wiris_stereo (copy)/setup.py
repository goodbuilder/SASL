from setuptools import setup
from glob import glob

package_name = 'wiris_stereo'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        # 패키지 인덱스 등록
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # package.xml 설치
        ('share/' + package_name, ['package.xml']),
        # launch/ 폴더의 모든 launch.py 설치 (실제 파일이 launch/ 에 있을 때!)
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='saslab',
    maintainer_email='saslab@example.com',
    description='WIRIS stereo RTSP + zoom filter launch and nodes',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'zoom_node = wiris_stereo.wiris_zoom_node:main',
            'yolo_node = wiris_stereo.wiris_yolo_node:main',
            'depth_node= wiris_stereo.depth_node:main',
            'ahrs_node = wiris_stereo.myahrs_dual:main',
            
        ],
    },
)

