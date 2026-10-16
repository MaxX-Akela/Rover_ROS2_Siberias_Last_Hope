from setuptools import find_packages, setup

package_name = 'rover_teleop'

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
    description='QGroundControl joystick (MAVLink MANUAL_CONTROL) to /cmd_vel',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'mavlink_teleop = rover_teleop.mavlink_teleop:main',
        ],
    },
)
