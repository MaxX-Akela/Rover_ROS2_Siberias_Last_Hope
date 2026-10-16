from setuptools import find_packages, setup

package_name = 'rover_fcu_bridge'

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
    description='Bridge between ROS 2 and the rover flight controller (FCU)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'rover_fcu_bridge_node = rover_fcu_bridge.rover_fcu_bridge_node:main',
        ],
    },
)
