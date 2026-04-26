#!/usr/bin/env python3
"""
Top-level launch file for TankerVision flight computer.
Reads all parameters from config/tankervision.yaml.
Launches: Lucid Triton2 (arena_camera_node), IM19 IMU, analog camera (v4l2_camera),
          status_node, record_data_node (flight mode only)
"""
import os
import yaml
from launch import LaunchDescription
from launch.actions import OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import DurabilityPolicy

_CFG_PATH = os.path.join(
    os.path.expanduser('~'),
    'tankervision_flight', 'config', 'tankervision.yaml'
)

_VIDEO_STANDARDS = {
    'PAL':  {'image_size': [720, 576], 'time_per_frame': [1,    25]},
    'NTSC': {'image_size': [720, 480], 'time_per_frame': [1001, 30000]},
}


def _load_cfg() -> dict:
    try:
        with open(_CFG_PATH) as f:
            return yaml.safe_load(f) or {}
    except Exception as e:
        print(f'[flight.launch] WARNING: Could not load {_CFG_PATH}: {e}')
        return {}


def _launch_lucid_camera(_):
    cfg    = _load_cfg()
    lucid  = cfg.get('lucid_camera', {})
    node = Node(
        package='arena_camera_node',
        executable='start',
        name='arena_camera_node',
        output='screen',
        parameters=[
            {'qos_reliability': lucid.get('qos_reliability', 'reliable')},
            {'width':           lucid.get('width',           5320)},
            {'height':          lucid.get('height',          4600)},
            {'exposure_time':   float(lucid.get('exposure_time', 3000.0))},
            {'pixelformat':     lucid.get('pixelformat',     'bayer_rggb16')},
            {'raw_save_root':   cfg.get('storage', {}).get('root', '/mnt/storage')},
        ],
        remappings=[
            ('/arena_camera_node/images', '/cam0/image_raw'),
        ],
    )
    return [node]


def _launch_analog_camera(_):
    cfg     = _load_cfg()
    analog  = cfg.get('analog_camera', {})
    standard = analog.get('standard', 'NTSC').upper()
    device   = analog.get('device', '/dev/video0')

    if standard not in _VIDEO_STANDARDS:
        print(f'[flight.launch] WARNING: unknown standard {standard}, defaulting to NTSC')
        standard = 'NTSC'

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
            ('/image_raw',   '/cam1/image_raw'),
            ('/camera_info', '/cam1/camera_info'),
        ],
    )
    return [node]


def _launch_status_node(_):
    node = Node(
        package='tanker_vision',
        executable='status_node',
        name='status_node',
        output='screen',
        parameters=[{'config': _CFG_PATH}],
    )
    return [node]


def _launch_record_node(_):
    cfg  = _load_cfg()
    mode = cfg.get('mode', 'testing')
    if mode != 'flight':
        return []
    node = Node(
        package='tanker_vision',
        executable='record_data_node',
        name='record_data_node',
        output='screen',
        parameters=[{'config': _CFG_PATH}],
    )
    return [node]


def generate_launch_description():
    my_bringup_dir = get_package_share_directory('my_bringup')
    im19_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(my_bringup_dir, 'launch', 'sensors.launch.py')
        )
    )

    return LaunchDescription([
        im19_launch,
        OpaqueFunction(function=_launch_lucid_camera),
        OpaqueFunction(function=_launch_analog_camera),
        OpaqueFunction(function=_launch_status_node),
        OpaqueFunction(function=_launch_record_node),
    ])


if __name__ == '__main__':
    generate_launch_description()