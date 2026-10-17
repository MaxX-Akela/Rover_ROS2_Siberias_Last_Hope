import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def load_network(path):
    with open(path) as f:
        return yaml.safe_load(f) or {}


def web_video_and_stream(context):
    network = load_network(LaunchConfiguration('network_file').perform(context))
    cameras_file = LaunchConfiguration('cameras_file').perform(context)
    with open(cameras_file) as f:
        cameras = (yaml.safe_load(f) or {}).get('cameras') or []
    topics = [f"/{cam.get('namespace', cam['name'])}/camera/image_raw" for cam in cameras]

    return [
        Node(
            package='rover_vision',
            executable='video_streamer',
            name='video_streamer',
            output='screen',
            parameters=[{
                'port': int(network.get('rtsp_port', 8554)),
                'host': str(network.get('rover_ip', '')),
                'topics': topics,
            }],
            condition=IfCondition(LaunchConfiguration('stream')),
        ),
        Node(
            package='web_video_server',
            executable='web_video_server',
            name='web_video_server',
            output='screen',
            parameters=[{'port': int(network.get('web_video_port', 8081))}],
            condition=IfCondition(LaunchConfiguration('web_video')),
        ),
    ]


def control_nodes(context):
    control_file = LaunchConfiguration('control_file')
    mode = LaunchConfiguration('control_mode').perform(context)
    bridge_params = [control_file] + ([{'control_mode': mode}] if mode else [])
    return [
        ExecuteProcess(
            cmd=['MicroXRCEAgent', 'serial', '--dev', LaunchConfiguration('agent_dev'),
                 '-b', LaunchConfiguration('agent_baud')],
            name='micro_xrce_agent',
            output='screen',
            condition=IfCondition(LaunchConfiguration('agent')),
        ),
        Node(
            package='rover_fcu_bridge',
            executable='rover_fcu_bridge_node',
            name='rover_fcu_bridge',
            output='screen',
            parameters=bridge_params,
            condition=IfCondition(LaunchConfiguration('fcu_bridge')),
        ),
        Node(
            package='rover_teleop',
            executable='mavlink_teleop',
            name='rover_teleop',
            output='screen',
            parameters=[control_file],
            condition=IfCondition(LaunchConfiguration('teleop')),
        ),
    ]


def optional_includes(context):
    slam = get_package_share_directory('rover_slam')
    led = get_package_share_directory('rover_led')
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(slam, 'launch', 'slam.launch.py')),
            condition=IfCondition(LaunchConfiguration('slam'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(led, 'launch', 'led.launch.py')),
            condition=IfCondition(LaunchConfiguration('led'))),
    ]


def generate_launch_description():
    description = get_package_share_directory('rover_description')
    bringup = get_package_share_directory('rover_bringup')
    config = os.path.join(bringup, 'config')

    return LaunchDescription([
        DeclareLaunchArgument('cameras', default_value='true',
                              description='Start the cameras from cameras_file'),
        DeclareLaunchArgument('stream', default_value='true',
                              description='Start the RTSP video_streamer (video for QGC)'),
        DeclareLaunchArgument('web_video', default_value='true',
                              description='Start web_video_server (MJPEG over HTTP)'),
        DeclareLaunchArgument('agent', default_value='true',
                              description='Start Micro XRCE-DDS Agent (Pixracer TELEM2 <-> ROS 2)'),
        DeclareLaunchArgument('agent_dev', default_value='/dev/rover_fcu_telem'),
        DeclareLaunchArgument('agent_baud', default_value='921600'),
        DeclareLaunchArgument('fcu_bridge', default_value='true',
                              description='Start rover_fcu_bridge (/cmd_vel -> PX4, /odom)'),
        DeclareLaunchArgument('teleop', default_value='true',
                              description='Start rover_teleop (QGC joystick -> /cmd_vel)'),
        DeclareLaunchArgument('slam', default_value='false',
                              description='Start RPLIDAR A1 + slam_toolbox and send the pose to PX4'),
        DeclareLaunchArgument('led', default_value='false',
                              description='Start the WS2812 strip (SPI) with PX4 status effects'),
        DeclareLaunchArgument('control_mode', default_value='',
                              description='direct | velocity; empty = value from control_file'),
        DeclareLaunchArgument('control_file', default_value=os.path.join(config, 'control.yaml')),
        DeclareLaunchArgument('cameras_file', default_value=os.path.join(config, 'cameras.yaml')),
        DeclareLaunchArgument('network_file', default_value=os.path.join(config, 'network.yaml')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(description, 'launch', 'description.launch.py'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup, 'launch', 'cameras.launch.py')),
            launch_arguments={'cameras_file': LaunchConfiguration('cameras_file')}.items(),
            condition=IfCondition(LaunchConfiguration('cameras'))),
        OpaqueFunction(function=web_video_and_stream),
        OpaqueFunction(function=control_nodes),
        OpaqueFunction(function=optional_includes),
    ])
