#!/usr/bin/env python3
"""
Top-level launch file for TankerVision flight computer.
Launches: Lucid Triton2 (arena_camera_node), IM19 IMU, analog camera (v4l2_camera),
          status_node
"""
import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# PAL:  720x576 @ 25 fps,    Europe / Australia
# NTSC: 720x480 @ 29.97 fps, North America / Japan
_VIDEO_STANDARDS = {
    'PAL':  {'image_size': [720, 576], 'time_per_frame': [1,    25]},
    'NTSC': {'image_size': [720, 480], 'time_per_frame': [1001, 30000]},
}

# Path to global config — read by status_node at runtime
_CONFIG_YAML = os.path.expanduser(
    '~/tankervision_flight/config/tankervision.yaml'
)


def _launch_lucid_camera(_):
    node = Node(
        package='arena_camera_node',
        executable='start',
        name='arena_camera_node',
        output='screen',
        parameters=[
            {'qos_reliability': 'reliable'},
            {'width': 5320},
            {'height': 4600},
            {'exposure_time': 3000.0},
            {'pixelformat': 'bayer_rggb16'},
        ],
        remappings=[
            ('/arena_camera_node/images', '/cam0/image_raw'),
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
        ],
        remappings=[
            ('/image_raw',    '/cam1/image_raw'),
            ('/camera_info',  '/cam1/camera_info'),
        ],
    )
    return [node]


def _launch_record_node(context):
    import yaml, os
    cfg_path = os.path.join(os.path.expanduser('~'),
                            'tankervision_flight', 'config', 'tankervision.yaml')
    try:
        with open(cfg_path) as f:
            cfg = yaml.safe_load(f)
        mode = cfg.get('mode', 'testing')
    except Exception:
        mode = 'testing'

    if mode != 'flight':
        return []

    node = Node(
        package='tanker_vision',
        executable='record_data_node',
        name='record_data_node',
        output='screen',
        parameters=[{'config': cfg_path}],
    )
    return [node]


def _launch_status_node(_):
    node = Node(
        package='tanker_vision',
        executable='status_node',
        name='status_node',
        output='screen',
        parameters=[
            {'config': _CONFIG_YAML},
        ],
    )
    return [node]


def generate_launch_description():
    # IM19 IMU
    my_bringup_dir = get_package_share_directory('my_bringup')
    im19_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(my_bringup_dir, 'launch', 'sensors.launch.py')
        )
    )

    arg_video_standard = DeclareLaunchArgument(
        'video_standard',
        default_value='NTSC',
        description="Analog video standard: 'PAL' or 'NTSC'",
    )
    arg_video_device = DeclareLaunchArgument(
        'video_device',
        default_value='/dev/video0',
        description='V4L2 device path for analog capture card',
    )

    lucid_camera  = OpaqueFunction(function=_launch_lucid_camera)
    analog_camera = OpaqueFunction(function=_launch_analog_camera)
    status_node   = OpaqueFunction(function=_launch_status_node)
    record_node   = OpaqueFunction(function=_launch_record_node)

    return LaunchDescription([
        arg_video_standard,
        arg_video_device,
        im19_launch,
        lucid_camera,
        analog_camera,
        status_node,
        record_node,
    ])


if __name__ == '__main__':
    generate_launch_description()
    