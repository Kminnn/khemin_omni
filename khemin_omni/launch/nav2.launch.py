import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from pathlib import Path
from nav2_common.launch import RewrittenYaml

PACKAGE_NAME = "khemin_omni"

ARGUMENTS = [
    DeclareLaunchArgument('world', 
                          default_value="maze2",
                          description='Gazebo World'),

    DeclareLaunchArgument('use_sim_time',
                          default_value='True',
                          description='Use simulation (Gazebo) clock if true')
]

def generate_launch_description():
    nav2_launch_path = PathJoinSubstitution([
                get_package_share_directory("nav2_bringup"), 'launch', 'bringup_launch.py'
            ])
    world = LaunchConfiguration("world")
    map_path = [PathJoinSubstitution([get_package_share_directory(PACKAGE_NAME), 'map', world]), '.yaml']
    param_file_path = os.path.join(get_package_share_directory(PACKAGE_NAME), 'config', 'nav2_params.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')

    configured_params = RewrittenYaml(
        source_file=param_file_path,
        param_rewrites={'use_sim_time': use_sim_time},
        convert_types=True
    )

    
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_launch_path]),
        launch_arguments={
            "map": map_path,
            "params_file": configured_params,
            "use_sim_time": use_sim_time
        }.items()
    )

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[os.path.join(get_package_share_directory(PACKAGE_NAME), 'config', 'ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(nav2_launch)
    ld.add_action(robot_localization_node)
    # ld.add_action(LogInfo(condition=None, msg=map_path))
    return ld