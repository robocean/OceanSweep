from setuptools import setup

package_name = 'gps_publisher'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='user@example.com',
    description='Package for managing GPS and yaw data.',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'broker_node = gps_publisher.broker_node:main',
            #'gps_node = gps_publisher.gps_node:main',
            #'yaw_node = gps_publisher.yaw_node:main',
            #'test_sock_node = gps_publisher.test_ros_sock:main',
            'gps_to_utm = gps_publisher.gps_to_utm:main',
            'kalman_filter_node = gps_publisher.kalman_filter_node:main',
            'waypoint_controller_node = gps_publisher.waypoint_controller_node:main',
        ],
    },
)
