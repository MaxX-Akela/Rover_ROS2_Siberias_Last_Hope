from glob import glob

from setuptools import find_packages, setup

package_name = 'rover_slam'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='MaxX-Akela',
    maintainer_email='mstepanov2610@gmail.com',
    description='2D SLAM (RPLIDAR A1 + slam_toolbox) and vision_to_px4 pose feed for the rover',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'vision_to_px4 = rover_slam.vision_to_px4:main',
        ],
    },
)
