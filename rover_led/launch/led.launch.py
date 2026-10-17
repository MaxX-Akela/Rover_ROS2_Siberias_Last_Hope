import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('rover_led'), 'config')
    params = [LaunchConfiguration('led_file'), LaunchConfiguration('notify_file')]
    return LaunchDescription([
        DeclareLaunchArgument('led_file', default_value=os.path.join(config, 'led.yaml')),
        DeclareLaunchArgument('notify_file', default_value=os.path.join(config, 'led_notify.yaml')),
        Node(package='rover_led', executable='led_driver', name='led_driver',
             output='screen', parameters=params),
        Node(package='rover_led', executable='led_effect', name='led_effect',
             output='screen', parameters=params),
    ])
