import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    share = get_package_share_directory('rover_description')
    doc = xacro.process_file(os.path.join(share, 'urdf', 'rover.urdf.xacro'))
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': doc.toxml()}],
        ),
    ])
