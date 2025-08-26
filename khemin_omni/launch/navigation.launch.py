import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node

PACKAGE_NAME = "khemin_omni"

ARGUMENTS = [
    DeclareLaunchArgument('world', 
                          default_value="maze2",
                          description='Gazebo World'),
]

def generate_launch_description():
    # launch gazebo with spawned robot
    gazebo_sim_path = PathJoinSubstitution([
                get_package_share_directory(PACKAGE_NAME), 'launch', 'gazebo.launch.py'
            ])
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_sim_path]),
        launch_arguments={'world': LaunchConfiguration('world')}.items()
    )

    nav2_launch_path = PathJoinSubstitution([
                get_package_share_directory(PACKAGE_NAME), 'launch', 'nav2.launch.py'
            ])

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_launch_path]),
        launch_arguments={'world': LaunchConfiguration('world')}.items()
    )

    delayed_nav2 = TimerAction(
        period=5.0,
        actions=[nav2_launch]
    )

    

    rviz_config_path = PathJoinSubstitution([
                get_package_share_directory(PACKAGE_NAME), 'rviz', 'navigation.rviz'
            ])
    rviz2 = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            parameters=[{'use_sim_time': True}]
        )

    delayed_rviz = TimerAction(
        period=6.0,   
        actions=[rviz2]
    )
    
    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(gazebo_sim)
    ld.add_action(delayed_nav2)
    # ld.add_action(delayed_rviz)

    ld.add_action(rviz2)
    return ld