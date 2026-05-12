from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    im19_dir = get_package_share_directory('im19_ros2')
    im19_yaml = os.path.join(im19_dir, 'config', 'im19.yaml')

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
