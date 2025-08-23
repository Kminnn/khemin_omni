import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from pathlib import Path

PACKAGE_NAME = "khemin_omni"

ARGUMENTS = [
    DeclareLaunchArgument('world', 
                          default_value="maze2",
                          description='Gazebo World'),
]

def generate_launch_description():
    nav2_launch_path = PathJoinSubstitution([
                get_package_share_directory("nav2_bringup"), 'launch', 'bringup_launch.py'
            ])
    world = LaunchConfiguration("world")
    map_path = [PathJoinSubstitution([get_package_share_directory(PACKAGE_NAME), 'map', world]), '.yaml']
    param_file_path =  os.path.join(get_package_share_directory(PACKAGE_NAME), 'config', 'nav2_params.yaml')

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_launch_path]),
        launch_arguments={
            "map": map_path,
            "params_file": param_file_path,
            "use_sim_time": 'true'}.items()
    )
    
    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(nav2_launch)
    # ld.add_action(LogInfo(condition=None, msg=map_path))
    return ld