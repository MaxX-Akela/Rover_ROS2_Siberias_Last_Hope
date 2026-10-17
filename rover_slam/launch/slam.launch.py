import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('rover_slam')
    lidar_frame = LaunchConfiguration('lidar_frame')

    return LaunchDescription([
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(share, 'config', 'slam_toolbox.yaml')),
        DeclareLaunchArgument('lidar', default_value='true',
                              description='Start the RPLIDAR A1 driver (sllidar_ros2)'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/rover_lidar'),
        DeclareLaunchArgument('lidar_baud', default_value='115200',
                              description='115200 for A1 (256000 for A2M12/A3)'),
        DeclareLaunchArgument('lidar_frame', default_value='laser'),
        DeclareLaunchArgument('lidar_inverted', default_value='false'),
        DeclareLaunchArgument('lidar_tf', default_value='true',
                              description='Publish static TF base_link -> lidar_frame'),
        # lidar pose in base_link: x y z [m], yaw [rad]
        DeclareLaunchArgument('lidar_x', default_value='0.0'),
        DeclareLaunchArgument('lidar_y', default_value='0.0'),
        DeclareLaunchArgument('lidar_z', default_value='0.15'),
        DeclareLaunchArgument('lidar_yaw', default_value='0.0'),
        DeclareLaunchArgument('vision_to_px4', default_value='true',
                              description='Send the SLAM pose to PX4 (vehicle_visual_odometry)'),

        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            output='screen',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': LaunchConfiguration('lidar_port'),
                'serial_baudrate': ParameterValue(LaunchConfiguration('lidar_baud'), value_type=int),
                'frame_id': lidar_frame,
                'inverted': ParameterValue(LaunchConfiguration('lidar_inverted'), value_type=bool),
                'angle_compensate': True,
            }],
            condition=IfCondition(LaunchConfiguration('lidar')),
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_tf',
            arguments=[
                '--x', LaunchConfiguration('lidar_x'), '--y', LaunchConfiguration('lidar_y'),
                '--z', LaunchConfiguration('lidar_z'), '--yaw', LaunchConfiguration('lidar_yaw'),
                '--frame-id', 'base_link', '--child-frame-id', lidar_frame],
            condition=IfCondition(LaunchConfiguration('lidar_tf')),
        ),
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
        Node(
            package='rover_slam',
            executable='vision_to_px4',
            name='vision_to_px4',
            output='screen',
            condition=IfCondition(LaunchConfiguration('vision_to_px4')),
        ),
    ])
