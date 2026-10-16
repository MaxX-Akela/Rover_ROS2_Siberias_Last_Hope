import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
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
    ])
