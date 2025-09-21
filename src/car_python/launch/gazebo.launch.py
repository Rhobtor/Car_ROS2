from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
import os
import xacro
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    share_dir = get_package_share_directory('car')

    xacro_file = os.path.join(share_dir, 'urdf', 'car.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_urdf,
                    'use_sim_time': True}]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzserver.launch.py'
            ])
        ]),
        launch_arguments={'verbose': 'true', 'pause': 'false'}.items()
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzclient.launch.py'
            ])
        ]),
        launch_arguments={'verbose': 'true'}.items()
    )

    urdf_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'car',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.369183'
        ],
        output='screen'
    )

    # Agregamos el nodo de odometría
    odometry_node = Node(
        package='car',  # Asegúrate de que el paquete se llame 'car' o el que corresponda
        executable='odometry_node',  # Nombre del ejecutable (por ejemplo, si instalaste el script con entry_point)
        name='odometry_node'
    )




    # diff_drive_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=['diff_cont', '--controller-manager', '/controller_manager'],
    #     output='screen'
    # )

    # joint_broad_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["joint_brand"],
    #     output='screen'
    # )

    return LaunchDescription([
        robot_state_publisher_node,
        # joint_state_publisher_node,
        gazebo_server,
        gazebo_client,
        urdf_spawn_node,
        #odometry_node
    ])
