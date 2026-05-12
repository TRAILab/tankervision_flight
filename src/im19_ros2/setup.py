from setuptools import setup
from glob import glob
import os

package_name = 'im19_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='atlas',
    maintainer_email='atlas@example.com',
    description='IM19 MEMS raw logger and ROS2 publisher',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'im19_mems_node = im19_ros2.im19_mems_node:main',
        ],
    },
)
