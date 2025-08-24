import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, EmitEvent
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.events import Shutdown
from pathlib import Path

PACKAGE_NAME = "khemin_omni"

ARGUMENTS = [
    DeclareLaunchArgument('world', 
                          default_value="maze2",
                          description='Gazebo World'),
    DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use sim time if true'),
]

def generate_launch_description():
    #robot_model = os.environ.get("OMNI_ROBOT_MODEL", "4w")
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Source Environment (Need it to be able find mesh files)
    pkg_path = get_package_share_directory(PACKAGE_NAME)

    ign_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            str(Path(pkg_path).parent.resolve()), ":",
             os.path.join(pkg_path, 'worlds'),
            ]
    )

    #xacro_file = os.path.join(pkg_path,'urdf', robot_model, 'main.urdf.xacro')
    xacro_file = os.path.join(pkg_path, 'urdf', 'main.urdf.xacro')
    rviz_config_file = os.path.join(pkg_path, 'rviz', 'rviz_config.rviz')
    ekf_config_path = os.path.join(pkg_path, 'config', 'ekf.yaml')



    robot_description_config = Command(['xacro ', xacro_file])
    
    params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # launch gazebo
    gazebo_launch_path = PathJoinSubstitution([
                get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
            ])
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_launch_path]),
        launch_arguments=[
            ('gz_args', [LaunchConfiguration('world'),
                         '.sdf',
                          ' -r',
                          ' -v 4'])
        ]
    )

    robot_name = "my_robot"

    # Spawn the robot in Gazebo
    spawn_robot = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-name', robot_name,
                                   '-z', '0.1'],
                        output='screen')
    
    # gz bridge 
    bridge_params = os.path.join(get_package_share_directory(PACKAGE_NAME),'config', 'gz_bridge', f'gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{'config_file': bridge_params}],
        # arguments=[
        #     '--ros-args',
        #     '-p',
        #     f'config_file:={bridge_params}',
        # ]
    )

    twist_mux_params = os.path.join(get_package_share_directory(PACKAGE_NAME),'config','twist_mux','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': True}],
            remappings=[('/cmd_vel_out','/cmd_vel')]
    )
    
    twist_stamped_frame_id = 'base_footprint'
    twist_stamper_node = Node(
            package='twist_stamper',
            executable='twist_stamper',
            name='twist_stamper',
            output='screen',
            remappings=[
                ('/cmd_vel_in', '/cmd_vel'),
                ('/cmd_vel_out', '/omni_wheel_drive_controller/cmd_vel'),
            ],
            parameters=[
                {'frame_id': twist_stamped_frame_id},
            ]
        )



    # spawn controller     
    # N = int(4)
    # arg = ['joint_state_broadcaster']
    # for i in range(N):
    #     arg.append(f"wheel{i+1}_controller")
    # spawn_wheel_controller = Node(package='controller_manager', executable='spawner',
    #                     arguments=arg,
    #                     output='screen')

    spawn_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['omni_wheel_drive_controller'],
        # remappings=[('/omni_wheel_drive_controller/odom', '/odom')],
        )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_broad'],
        )

    # kinematics = Node(
    #     package=PACKAGE_NAME,
    #     executable="kinematics",
    #     parameters=[{"use_sim_time": use_sim_time}]
    # )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
    )

    
    
    # Relay /omni_wheel_drive_controller/odom to /odom
    relay_odom = Node(
        package='topic_tools',
        executable='relay',
        name='odom_relay',
        arguments=['/omni_wheel_drive_controller/odom', '/odom'],
        output='screen'
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path]
    )

    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(ign_resource_path)
    ld.add_action(node_robot_state_publisher)
    ld.add_action(gazebo)
    ld.add_action(spawn_robot)
    ld.add_action(ros_gz_bridge)
    # ld.add_action(spawn_wheel_controller)
    ld.add_action(spawn_controller)
    ld.add_action(twist_mux)
    ld.add_action(twist_stamper_node)
    ld.add_action(relay_odom)

    # ld.add_action(kinematics)
    # ld.add_action(rviz_node)
    ld.add_action(joint_state_broadcaster)
    return ld
