from setuptools import find_packages, setup

package_name = 'rover_vision'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='MaxX-Akela',
    maintainer_email='mstepanov2610@gmail.com',
    description='Camera and image processing for the rover',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'video_streamer = rover_vision.video_streamer:main',
        ],
    },
)
