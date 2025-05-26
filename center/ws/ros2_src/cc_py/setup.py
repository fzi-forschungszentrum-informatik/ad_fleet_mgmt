from setuptools import find_packages, setup
from glob import glob

package_name = 'cc_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='A ROS2 package for managing assisted vehicle operations, including visualization, marker management, and map processing functionalities.',
    maintainer='Martin Gontscharow',
    maintainer_email='gontscharow@fzi.de',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'assisted_vehicle_relay_manager = cc_py.assisted_vehicle_relay_manager:main',
            'clear_a_star_visualization = cc_py.clear_a_star_visualization:main',
            'clear_all_markers = cc_py.clear_all_markers:main',
            'process_map = cc_py.process_map:main',
            'publish_map = cc_py.publish_map:main',
            'reduce_map = cc_py.reduce_map:main',
            'lanelet_saver = cc_py.lanelet_saver:main',
            'lanelet_publisher = cc_py.lanelet_publisher:main',
            'tf_map_to_cart_saver = cc_py.tf_map_to_cart_saver:main',
            'tf_map_to_cart_publisher = cc_py.tf_map_to_cart_publisher:main',
            'switchbox_state_remapper = cc_py.switchbox_state_remapper:main',
        ],
    },
)
