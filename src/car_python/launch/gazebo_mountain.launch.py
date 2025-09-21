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
        'obstacles_and_traps.world'
    ])
    octomap_arg = PathJoinSubstitution([
        FindPackageShare('car'),
        'config',
        'octomap.yaml'
    ])
    # Get the car URDF by processing the xacro file
    xacro_file = os.path.join(share_dir, 'urdf', 'car.xacro')
    move_object_file = os.path.join(share_dir, 'urdf', 'object_movement.sdf')
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
            'verbose': 'true',
            'world': world_arg
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
            '-z', '0.369183'  # Initial Z position (adjust if needed)
        ],
        output='screen'
    )

        # Agregamos el nodo de odometría
    filter_points_cloud = Node(
        package='car',  # Asegúrate de que el paquete se llame 'car' o el que corresponda
        executable='filter_points_cloud',  # Nombre del ejecutable (por ejemplo, si instalaste el script con entry_point)
        name='filter_points_cloud',  # Nombre del nodo
    )

    frontier_values = Node(
        package='car_cpp',
        executable='frontier_reduction',
        name='frontier_reduction'
    )

    octomap = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        parameters=[{
            'resolution': 0.2,
            'frame_id': 'map',
            'sensor_model.max_range': 40.0
        }],
        remappings=[('cloud_in', 'scan_cloud_filtered')]
    )

    map_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    navigation_nodes_ground = Node(
        package='car_cpp',
        executable='navigation_nodes_ground',
        name='navigation_nodes_ground'
    )

    filtered_navigation_nodes = Node(
        package='car_cpp',
        executable='filtered_navigation_nodes',
        name='filtered_navigation_nodes'
    )

    navigation_nodes = Node(
        package='car_cpp',
        executable='navigation_nodes',
        name='navigation_nodes'
    )

    move_navigation_nodes = Node(
        package='car_cpp',
        executable='move_navigation_nodes_frontier',
        name='move_navigation_nodes_frontier'
    )

    obstacles_in_2d = Node(
        package='car_cpp',
        executable='obstacles_in_2d',
        name='obstacles_in_2d'
    )

    occupied_nodes_near_obstacles = Node(
        package='car_cpp',
        executable='occupied_nodes_near_obstacles',
        name='occupied_nodes_near_obstacles'
    )

    check_goal = Node(
        package='car_cpp',  # Asegúrate de que el paquete se llame 'car' o el que corresponda
        executable='check_goal',  # Nombre del ejecutable (por ejemplo, si instalaste el script con entry_point)
        name='check_goal',  # Nombre del nodo
    )

    poinst_goal = Node(
        package='car',  # Asegúrate de que el paquete se llame 'car' o el que corresponda
        executable='points_goal',  # Nombre del ejecutable (por ejemplo, si instalaste el script con entry_point)
        name='points_goal',  # Nombre del nodo
    )

    colision_zone = Node(
        package='car',  # Asegúrate de que el paquete se llame 'car' o el que corresponda
        executable='colision_zone',  # Nombre del ejecutable (por ejemplo, si instalaste el script con entry_point)
        name='colision_zone',  # Nombre del nodo
    )

    memory_map = Node(
        package='car',  # Asegúrate de que el paquete se llame 'car' o el que corresponda
        executable='memory_map',  # Nombre del ejecutable (por ejemplo, si instalaste el script con entry_point)
        name='memory_map',  # Nombre del nodo
    )

    navegation_map = Node(
        package='car',  # Asegúrate de que el paquete se llame 'car' o el que corresponda
        executable='map_navegation',  # Nombre del ejecutable (por ejemplo, si instalaste el script con entry_point)
        name='map_navegation',  # Nombre del nodo
    )

    frontier_centroid = Node(
        package='car',  # Asegúrate de que el paquete se llame 'car' o el que corresponda
        executable='frontier_centroid',  # Nombre del ejecutable (por ejemplo, si instalaste el script con entry_point)
        name='frontier_centroid',  # Nombre del nodo
    )



    spawn_moving_obstacle = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'moving_obstacle',
            '-file', move_object_file,
            '-x', '14.0',   # Initial X position
            '-y', '6.0',   # Initial Y position
            '-z', '0.0'  # Initial Z position (adjust if needed)

        ],
        output='screen'
    )

    return LaunchDescription([
        # static_tf,  # Publica la transformación de 'odom' a 'map'
        # static_tf2,
        robot_state_publisher_node,
        # joint_state_publisher_node,
        gazebo_server,
        gazebo_client,
        urdf_spawn_node,
        filter_points_cloud,
        frontier_values,
        octomap,
        map_odom_tf,
        navigation_nodes_ground,
        filtered_navigation_nodes,
        navigation_nodes,
        obstacles_in_2d,
        occupied_nodes_near_obstacles,
        #move_navigation_nodes,
        check_goal,
        poinst_goal,
        colision_zone,
        navegation_map,
        memory_map,
        frontier_centroid,
        spawn_moving_obstacle

    ])
