from os import path

from ament_index_python import get_package_share_directory as get_pkg_share_dir

from launch import LaunchDescription as LaunchDesc
from launch.actions import IncludeLaunchDescription as IncLaunchDesc
from launch.launch_description_sources import PythonLaunchDescriptionSource as PyLaunchDescSrc

def generate_launch_description():
    launch_world = IncLaunchDesc(PyLaunchDescSrc(path.join(get_pkg_share_dir('very_simple_robot_simulator2'), 'launch/world_simulator.launch.py')))
    launch_kobuki = IncLaunchDesc(PyLaunchDescSrc(path.join(get_pkg_share_dir('very_simple_robot_simulator2'), 'launch/kobuki_simulator.launch.py')))
    launch_description = LaunchDesc([launch_world, launch_kobuki])
    return launch_description
