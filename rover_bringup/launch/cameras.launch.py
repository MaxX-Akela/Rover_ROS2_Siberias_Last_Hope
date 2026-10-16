import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DRIVERS = ('camera_ros', 'usb_cam')


def camera_node(cam):
    name = cam['name']
    driver = cam['driver']
    width, height, fps = int(cam['width']), int(cam['height']), int(cam['fps'])
    device = str(cam.get('device', ''))
    namespace = cam.get('namespace', name)
    frame_id = cam.get('frame_id', f'{name}_camera_optical_frame')

    if driver == 'camera_ros':
        # libcamera has no framerate parameter, it is set through the frame duration (us)
        duration = int(1e6 / fps)
        params = {
            'camera': int(device) if device.isdigit() else device,
            'width': width,
            'height': height,
            'frame_id': frame_id,
            'FrameDurationLimits': [duration, duration],
        }
        if 'format' in cam:
            params['format'] = str(cam['format'])
        package, executable = 'camera_ros', 'camera_node'
    elif driver == 'usb_cam':
        params = {
            'video_device': device,
            'image_width': width,
            'image_height': height,
            'framerate': float(fps),
            'pixel_format': str(cam.get('pixel_format', 'yuyv2rgb')),
            'camera_name': name,
            'frame_id': frame_id,
        }
        package, executable = 'usb_cam', 'usb_cam_node_exe'
    else:
        raise RuntimeError(f"camera '{name}': unknown driver '{driver}', expected one of {DRIVERS}")

    return Node(
        package=package,
        executable=executable,
        name='camera',
        namespace=namespace,
        parameters=[params],
        output='screen',
    )


def launch_cameras(context):
    path = LaunchConfiguration('cameras_file').perform(context)
    with open(path) as f:
        cameras = (yaml.safe_load(f) or {}).get('cameras') or []
    return [camera_node(cam) for cam in cameras]


def generate_launch_description():
    default = os.path.join(get_package_share_directory('rover_bringup'), 'config', 'cameras.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('cameras_file', default_value=default,
                              description='YAML with the list of cameras'),
        OpaqueFunction(function=launch_cameras),
    ])
