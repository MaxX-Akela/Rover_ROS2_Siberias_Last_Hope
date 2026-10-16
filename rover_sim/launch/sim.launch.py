from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    px4_dir = LaunchConfiguration('px4_dir')
    port = LaunchConfiguration('agent_port')

    # MicroXRCEAgent udp4 -p 8888
    agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', port],
        name='micro_xrce_agent',
        output='screen',
    )

    # make gz_rover_differential (PX4 SITL + Gazebo), run from the PX4-Autopilot checkout
    px4_sitl = ExecuteProcess(
        cmd=['make', 'px4_sitl', LaunchConfiguration('px4_target')],
        cwd=px4_dir,
        name='px4_sitl',
        output='screen',
        additional_env={'HEADLESS': LaunchConfiguration('headless')},
        condition=IfCondition(LaunchConfiguration('px4')),
    )

    bridge = Node(
        package='rover_fcu_bridge',
        executable='rover_fcu_bridge_node',
        name='rover_fcu_bridge',
        output='screen',
        condition=IfCondition(LaunchConfiguration('bridge')),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'px4_dir',
            default_value=PathJoinSubstitution([EnvironmentVariable('HOME'), 'PX4-Autopilot']),
            description='Path to the PX4-Autopilot checkout'),
        DeclareLaunchArgument('px4_target', default_value='gz_rover_differential'),
        DeclareLaunchArgument('agent_port', default_value='8888'),
        DeclareLaunchArgument('px4', default_value='true',
                              description='Start PX4 SITL (false when it runs elsewhere, e.g. in Docker)'),
        DeclareLaunchArgument('headless', default_value='0', description='1 = no Gazebo GUI'),
        DeclareLaunchArgument('bridge', default_value='true',
                              description='Also start rover_fcu_bridge'),
        agent,
        # give the agent a moment to open the UDP port before PX4 connects
        TimerAction(period=2.0, actions=[px4_sitl]),
        TimerAction(period=5.0, actions=[bridge]),
    ])
