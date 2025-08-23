import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from pathlib import Path

PACKAGE_NAME = "khemin_omni"

ARGUMENTS = [
    DeclareLaunchArgument('world', 
                          default_value="maze2",
                          description='Gazebo World'),
]

def generate_launch_description():
    # use_sim_time = LaunchConfiguration('use_sim_time')
    # launch gazebo with spawned robot
    gazebo_sim_path = PathJoinSubstitution([
                get_package_share_directory(PACKAGE_NAME), 'launch', 'gazebo.launch.py'
            ])
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_sim_path]),
        launch_arguments={'world': LaunchConfiguration('world')}.items()
    )

    slam_toolbox_launch_path = PathJoinSubstitution([
                get_package_share_directory("slam_toolbox"), 'launch', 'online_async_launch.py'
            ])
    
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([slam_toolbox_launch_path]),
        launch_arguments={"use_sim_time": "true"}.items()
    )

    rviz_config_path = PathJoinSubstitution([
                get_package_share_directory(PACKAGE_NAME), 'rviz', 'slam.rviz'
            ])
    rviz2 = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            parameters=[{'use_sim_time': True}]
        )
    
    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(gazebo_sim)
    ld.add_action(slam_toolbox_launch)
    ld.add_action(rviz2)
    return ld