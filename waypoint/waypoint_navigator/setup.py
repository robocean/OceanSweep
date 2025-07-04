from setuptools import setup
import os
from glob import glob

package_name = 'arduino_interface'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', 
         glob(os.path.join(package_name, 'launch', '*.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@example.com',
    description='ROS 2 interface for GPS/IMU data collection via Arduino, with DGPS correction, TM conversion, Kalman filtering, and waypoint control',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'broker_node = arduino_interface.broker_node:main',
            'tm_converter_node = arduino_interface.tm_converter_node:main',
            'bridge_server_node = arduino_interface.bridge_server_node:main',
            'dgps_node = arduino_interface.dgps_node:main',
            'kalman_node = arduino_interface.kalman_node:main',
            'waypoint_node = arduino_interface.waypoint_node:main',
            'vwr_test_pattern = arduino_interface.vwr_test_pattern:main',     
            'diff_drive_node = arduino_interface.diff_drive_node:main',       
        ],
    },
)

