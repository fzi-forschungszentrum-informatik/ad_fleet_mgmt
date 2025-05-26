from setuptools import find_packages, setup

package_name = 'sim_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='The sim_py package is a component of the fleet management system designed for autonomous mobility, specifically serving as a middleware layer that facilitates communication and data handling between various components, particularly for simulated vehicles.',
    maintainer='Martin Gontscharow',
    maintainer_email='gontscharow@fzi.de',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'costmap_remapper = sim_py.costmap_remapper:main',
            'switchbox_state_publisher = sim_py.switchbox_state_publisher:main',
            'tf_remapper = sim_py.tf_remapper:main',
            'marker_remapper = sim_py.marker_remapper:main',
            'waypoint_ack_node = sim_py.waypoint_ack_node:main',
        ],
    },
)
