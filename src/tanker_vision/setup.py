from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'tanker_vision'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install all launch files in the launch/ directory.
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Publishes images from Lucid camera at 1Hz triggered mode',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nmea_to_serial_node = tanker_vision.nmea_to_serial_node:main',
            'hardware_trigger_image_publisher = tanker_vision.hardware_trigger_image_publisher:main',
            'status_node = tanker_vision.status_node:main',
            'lucid_stream_node = tanker_vision.lucid_stream_node:main',
            'sync_node = tanker_vision.sync_node:main',
            'image_saver = tanker_vision.image_saver:main',
            'record_data_node = tanker_vision.record_data_node:main',
        ],
    },
)
