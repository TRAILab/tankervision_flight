#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Include the xsens MTi driver's launch file.
    xsens_share_dir = get_package_share_directory('xsens_mti_ros2_driver')
    xsens_launch_file = os.path.join(xsens_share_dir, 'launch', 'xsens_mti_node.launch.py')
    xsens_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(xsens_launch_file)
    )

    arena_camera_node = Node(
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
            {'trigger_mode': True}
        ]
    )

    # image_publisher = Node(
    #     package='tanker_vision',
    #     executable='hardware_trigger_image_publisher',
    #     name='hardware_trigger_image_publisher',
    #     output='screen'
    # )

    # Launch the nmea_to_serial_node
    nmea_to_serial_node = Node(
        package='tanker_vision',
        executable='nmea_to_serial_node',
        name='nmea_to_serial_node',
        output='screen'
    )
    record_data_node = Node(
        package='tanker_vision',
        executable='record_data_node',
        name='record_data_node',
        output='screen'
    )

    return LaunchDescription([
        xsens_launch,
        arena_camera_node,
        nmea_to_serial_node,
        record_data_node
    ])

if __name__ == '__main__':
    generate_launch_description()
