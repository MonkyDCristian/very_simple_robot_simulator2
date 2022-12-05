from setuptools import setup

package_name = 'very_simple_robot_simulator2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Cristian Nova Santoya',
    maintainer_email='cristian.nova@uc.cl',
    description='A very simple robot simulator using rclpy',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'world_simulator = very_simple_robot_simulator2.pg_world_simulator:main',
		'kobuki_simulator = very_simple_robot_simulator2.kobuki_simulator:main',
		'kinect_simulator = very_simple_robot_simulator2.kinect_simulator:main',
		'lidar_simulator = very_simple_robot_simulator2.lidar_simulator:main'
                           ],
                 },
)
