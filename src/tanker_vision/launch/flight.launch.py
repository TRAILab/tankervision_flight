#!/usr/bin/env python3
"""
Top-level launch file for TankerVision.
Loads config/tankervision.yaml (global) + config/<unit>.yaml (unit-specific).
Unit config is found by matching username then hostname against config/<name>.yaml.
Unit config overrides global for any overlapping keys.
"""
import os
import socket
import yaml

from launch import LaunchDescription
from launch.actions import OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

_CFG_DIR  = os.path.join(os.path.expanduser('~'), 'tankervision_flight', 'config')
_CFG_PATH = os.path.join(_CFG_DIR, 'tankervision.yaml')

_VIDEO_STANDARDS = {
    'PAL':  {'image_size': [720, 576],  'time_per_frame': [1,    25]},
    'NTSC': {'image_size': [720, 480],  'time_per_frame': [1001, 30000]},
}


def _get_unit_cfg_path() -> str:
    for candidate in (os.environ.get('USER', '').lower(), socket.gethostname().lower()):
        if not candidate:
            continue
        path = os.path.join(_CFG_DIR, f'{candidate}.yaml')
        if os.path.exists(path):
            print(f'[flight.launch] Unit config: {path} (matched "{candidate}")')
            return path
    print(f'[flight.launch] WARNING: No unit config found for hostname "{socket.gethostname()}" or user "{os.environ.get("USER", "")}"')
    return ''


_UNIT_CFG_PATH = _get_unit_cfg_path()


def _deep_merge(base: dict, override: dict) -> dict:
    result = dict(base)
    for k, v in override.items():
        if k in result and isinstance(result[k], dict) and isinstance(v, dict):
            result[k] = _deep_merge(result[k], v)
        else:
            result[k] = v
    return result


def _load_cfg() -> dict:
    cfg = {}
    for path in [_CFG_PATH, _UNIT_CFG_PATH]:
        if path and os.path.exists(path):
            try:
                with open(path) as f:
                    data = yaml.safe_load(f) or {}
                cfg = _deep_merge(cfg, data)
            except Exception as e:
                print(f'[flight.launch] WARNING: Could not load {path}: {e}')
    return cfg


def _node_params() -> list:
    """Both config paths passed to every node so they can load and merge config themselves."""
    return [{'config': _CFG_PATH}, {'unit_config': _UNIT_CFG_PATH}]


def _launch_lucid_camera(_):
    cfg   = _load_cfg()
    lucid = cfg.get('lucid_camera', {})
    ht_cfg = lucid.get('hardware_trigger', False)
    ht_bool = (ht_cfg is True) or (str(ht_cfg).lower() == 'true')
    node = Node(
        package='arena_camera_node',
        executable='start',
        name='arena_camera_node',
        output='screen',
        parameters=_node_params() + [
            {'qos_reliability':           lucid.get('qos_reliability',           'reliable')},
            {'width':                     lucid.get('width',                      5320)},
            {'height':                    lucid.get('height',                     4600)},
            {'pixelformat':               lucid.get('pixelformat',               'bayer_rggb16')},
            {'bayer_raw_shift':           int(lucid.get('bayer_raw_shift',        8))},
            {'hardware_trigger':          ht_bool},
            {'exposure_auto':             lucid.get('exposure_auto',             'Continuous')},
            {'gain_auto':                 lucid.get('gain_auto',                 'Continuous')},
            {'target_brightness':         int(lucid.get('target_brightness',      70))},
            {'gamma':                     float(lucid.get('gamma',                0.5))},
            {'exposure_auto_lower_limit': float(lucid.get('exposure_auto_lower_limit', 100.0))},
            {'exposure_auto_upper_limit': float(lucid.get('exposure_auto_upper_limit', 30000.0))},
            {'exposure_auto_algorithm':   lucid.get('exposure_auto_algorithm',   'Mean')},
            {'exposure_auto_damping':     float(lucid.get('exposure_auto_damping', 89.8))},
            {'raw_save_root':             cfg.get('storage', {}).get('root', '/mnt/storage')},
        ],
        remappings=[('/arena_camera_node/images', '/cam0/image_raw')],
    )
    return [node]


def _launch_analog_camera(_):
    cfg      = _load_cfg()
    analog   = cfg.get('analog_camera', {})
    standard = analog.get('standard', 'NTSC').upper()
    device   = analog.get('device', '/dev/video0')
    if standard not in _VIDEO_STANDARDS:
        print(f'[flight.launch] WARNING: unknown video standard "{standard}", defaulting to NTSC')
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
        respawn=True,
        respawn_delay=2.0,
    )
    return [node]


def _launch_status_node(_):
    node = Node(
        package='tanker_vision',
        executable='status_node',
        name='status_node',
        output='screen',
        parameters=_node_params(),
    )
    return [node]


def _launch_record_node(_):
    cfg  = _load_cfg()
    mode = cfg.get('mode', 'testing')
    if mode != 'flight':
        print('[flight.launch] mode=testing — record_data_node will NOT be launched')
        return []
    node = Node(
        package='tanker_vision',
        executable='record_data_node',
        name='record_data_node',
        output='screen',
        parameters=_node_params(),
    )
    return [node]

def _launch_maxvis_record_node(_):
    cfg  = _load_cfg()
    mode = cfg.get('mode', 'testing')
    if mode != 'flight':
        return []
    record_fps = float(cfg.get('maxvis_recording', {}).get('record_fps', 1.0))
    throttle_node = Node(
        package='topic_tools',
        executable='throttle',
        name='cam1_throttle',
        arguments=['messages', '/cam1/image_raw', str(record_fps), '/cam1/image_throttled'],
    )
    maxvis_node = Node(
        package='tanker_vision',
        executable='maxvis_record_node',
        name='maxvis_record_node',
        output='screen',
        parameters=_node_params(),
    )
    return [throttle_node, maxvis_node]

def generate_launch_description():
    im19_dir = get_package_share_directory('im19_ros2')
    im19_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(im19_dir, 'launch', 'sensors.launch.py')
        )
    )

    gpsd_velocity_node = Node(
        package='tanker_vision',
        executable='gpsd_velocity_node',
        name='gpsd_velocity_node',
        output='screen',
        parameters=_node_params(),
    )

    return LaunchDescription([
        im19_launch,
        gpsd_velocity_node,
        OpaqueFunction(function=_launch_lucid_camera),
        OpaqueFunction(function=_launch_analog_camera),
        OpaqueFunction(function=_launch_status_node),
        OpaqueFunction(function=_launch_record_node),
        OpaqueFunction(function=_launch_maxvis_record_node),
    ])


if __name__ == '__main__':
    generate_launch_description()
