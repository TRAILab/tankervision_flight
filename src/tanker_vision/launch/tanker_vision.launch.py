#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


# PAL:  720x576 @ 25 fps,      used in Europe / Australia
# NTSC: 720x480 @ 29.97 fps,   used in North America / Japan
_VIDEO_STANDARDS = {
    'PAL':  {'image_size': [720, 576], 'time_per_frame': [1,    25]},
    'NTSC': {'image_size': [720, 480], 'time_per_frame': [1001, 30000]},
}


def _launch_lucid_camera(_):
    node = Node(
        package='arena_camera_node',
        executable='start',
        name='arena_camera_node',
        output='screen',
        parameters=[
            {'qos_reliability': 'reliable'},
            {'width': 2880},
            {'height': 1860},
            {'exposure_time': 2000.0},
            {'pixelformat': 'rgb8'},
            {'topic': '/cam0/image_raw'},
            {'trigger_mode': True},
        ],
    )

    return [node]


def _launch_analog_camera(context):
    standard = LaunchConfiguration('video_standard').perform(context).upper()
    device   = LaunchConfiguration('video_device').perform(context)

    if standard not in _VIDEO_STANDARDS:
        raise ValueError(
            f"video_standard must be 'PAL' or 'NTSC', got '{standard}'"
        )

    params = _VIDEO_STANDARDS[standard]

    set_standard = ExecuteProcess(
        cmd=['v4l2-ctl', '--device', device, '--set-standard', standard],
        output='screen',
    )

    node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='analog_camera',
        output='screen',
        parameters=[
            {'video_device':    device},
            {'image_size':      params['image_size']},
            {'time_per_frame':  params['time_per_frame']},
            {'camera_frame_id': 'analog_camera'},
            {'topic':           '/cam1/image_raw'},
        ],
    )

    return [set_standard, node]


def generate_launch_description():
    xsens_share_dir = get_package_share_directory('xsens_mti_ros2_driver')
    xsens_launch_file = os.path.join(xsens_share_dir, 'launch', 'xsens_mti_node.launch.py')

    arg_video_standard = DeclareLaunchArgument(
        'video_standard',
        default_value='PAL',
        description="Analog video standard for the capture card: 'PAL' or 'NTSC'",
    )
    arg_video_device = DeclareLaunchArgument(
        'video_device',
        default_value='/dev/video0',
        description='V4L2 device path for the analog capture card',
    )

    xsens_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(xsens_launch_file)
    )

    nmea_to_serial_node = Node(
        package='tanker_vision',
        executable='nmea_to_serial_node',
        name='nmea_to_serial_node',
        output='screen',
    )

    record_data_node = Node(
        package='tanker_vision',
        executable='record_data_node',
        name='record_data_node',
        output='screen',
    )

    lucid_camera  = OpaqueFunction(function=_launch_lucid_camera)
    analog_camera = OpaqueFunction(function=_launch_analog_camera)

    return LaunchDescription([
        arg_video_standard,
        arg_video_device,
        xsens_launch,
        nmea_to_serial_node,
        record_data_node,
        lucid_camera,
        analog_camera,
    ])

if __name__ == '__main__':
    generate_launch_description()