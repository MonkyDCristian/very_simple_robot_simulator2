from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    node1 = Node(name='kinect_simulator', package='very_simple_robot_simulator2', executable='kinect_simulator')
    launch_description = LaunchDescription([node1])
       
    return launch_description
