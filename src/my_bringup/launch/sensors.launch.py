from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    bringup_dir = get_package_share_directory('my_bringup')
    im19_yaml = os.path.join(bringup_dir, 'config', 'im19.yaml')

    im19_node = Node(
        package='im19_ros2',
        executable='im19_mems_node',
        name='im19_mems_node',
        output='screen',
        parameters=[im19_yaml],
    )

    return LaunchDescription([
        im19_node,
    ])
