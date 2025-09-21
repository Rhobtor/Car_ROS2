#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os
import xacro


def generate_launch_description():
    # Get the share directory of your package
    share_dir = get_package_share_directory('car')

    # Instead of using os.path.join to build a string, use PathJoinSubstitution.
    # This creates a substitution that is compatible with the launch system.
    world_arg = PathJoinSubstitution([
        FindPackageShare('car'),
        'worlds',
        'map1_test.world'
    ])
    octomap_arg = PathJoinSubstitution([
        FindPackageShare('car'),
        'config',
        'octomap.yaml'
    ])
    # Get the car URDF by processing the xacro file
    xacro_file = os.path.join(share_dir, 'urdf', 'car.xacro')
    config_file = os.path.join(share_dir, 'config', 'ekf_params.yaml')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    # # Nodo para publicar la transformación estática entre 'odom' y 'map'
    # # En este ejemplo, se asume que 'map' coincide con 'odom'
    # static_tf = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_transform_publisher_map',
    #     arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    # )

    # static_tf2 = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_transform_publisher_map',
    #     arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
    # )

    # Robot State Publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_urdf}]
    )

    # Joint State Publisher node
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    # Gazebo Server with the world argument
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gzserver.launch.py'])
        ]),
        launch_arguments={
            'pause': 'false',
            'verbose': 'true'
        }.items()
    )

    # Gazebo Client
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gzclient.launch.py'])
        ])
    )

    # Spawn the robot into Gazebo
    urdf_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'car',
            '-topic', 'robot_description',
            '-x', '0.0',   # Initial X position
            '-y', '0.0',   # Initial Y position
            '-z', '0.820271'  # Initial Z position (adjust if needed)
        ],
        output='screen'
    )



    map_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )



    return LaunchDescription([
        # static_tf,  # Publica la transformación de 'odom' a 'map'
        # static_tf2,
        robot_state_publisher_node,
        # joint_state_publisher_node,
        gazebo_server,
        gazebo_client,
        urdf_spawn_node,

        map_odom_tf,



    ])
