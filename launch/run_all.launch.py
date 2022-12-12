from os import path

from ament_index_python import get_package_share_directory as get_pkg_share_dir

from launch import LaunchDescription as LaunchDesc
from launch.actions import IncludeLaunchDescription as IncLaunchDesc
from launch.launch_description_sources import PythonLaunchDescriptionSource as PyLaunchDescSrc

def generate_launch_description():
    launch_minimal = IncLaunchDesc(PyLaunchDescSrc(path.join(get_pkg_share_dir('very_simple_robot_simulator2'), 'launch/minimal_simulator.launch.py')))
    launch_kinect = IncLaunchDesc(PyLaunchDescSrc(path.join(get_pkg_share_dir('very_simple_robot_simulator2'), 'launch/kinect_simulator.launch.py')))
    launch_lidar = IncLaunchDesc(PyLaunchDescSrc(path.join(get_pkg_share_dir('very_simple_robot_simulator2'), 'launch/lidar_simulator.launch.py')))
    
    launch_description = LaunchDesc([launch_minimal, launch_kinect, launch_lidar])
    return launch_description
