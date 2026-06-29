from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'radar_lidar_fusion'

setup(
    name=package_name,
    version='0.0.0',

    packages=find_packages(exclude=['test']),

    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),

        (
            'share/' + package_name,
            ['package.xml']
        ),

        (
            os.path.join(
                'share',
                package_name,
                'launch'
            ),
            glob('launch/*.launch.py')
        ),
    ],

    install_requires=[
        'setuptools',
    ],

    zip_safe=True,

    maintainer='rika',
    maintainer_email='rika@todo.todo',

    description='Radar and LiDAR sensor fusion package for ROS 2 Jazzy',

    license='Apache-2.0',

    extras_require={
        'test': [
            'pytest',
        ],
    },

    entry_points={
        'console_scripts': [
            'radar_node = radar_lidar_fusion.radar_node:main',
            'lidar_node = radar_lidar_fusion.lidar_node:main',
            'fusion_node = radar_lidar_fusion.fusion_node:main',
            'camera_node = radar_lidar_fusion.camera_node:main',
            'yolo_node = radar_lidar_fusion.yolo_node:main',
        ],
    },
)