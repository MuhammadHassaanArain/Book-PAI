#!/usr/bin/env python3
"""
Launch file for physics demo in Gazebo simulation

This launch file starts:
1. Gazebo simulation with physics_demo.world
2. Robot state publisher for the physics humanoid
3. Joint state publisher
4. Physics manager node
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the package share directory
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # World file path
    world_file = PathJoinSubstitution([
        get_package_share_directory('book_pai'),
        'simulation-assets',
        'gazebo',
        'worlds',
        'physics_demo.world'
    ])

    # Robot description file path
    robot_description_file = PathJoinSubstitution([
        get_package_share_directory('book_pai'),
        'src',
        'urdf',
        'humanoid_models',
        'physics_humanoid.urdf'
    ])

    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_file,
            'verbose': 'true'
        }.items()
    )

    # Read robot description from URDF file
    with open(robot_description_file, 'r') as infp:
        robot_description = infp.read()

    # Robot State Publisher Node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'publish_frequency': 50.0
        }]
    )

    # Joint State Publisher Node (for non-fixed joints)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'rate': 50,
            'source_list': ['joint_states']
        }]
    )

    # Physics Manager Node
    physics_manager = Node(
        package='book_pai',
        executable='physics_manager',
        name='physics_manager',
        output='screen',
        parameters=[]
    )

    # Spawn the robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'physics_humanoid',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '1.0'  # Start above ground to test gravity
        ],
        output='screen'
    )

    # Create the launch description
    ld = LaunchDescription()

    # Add actions
    ld.add_action(gazebo)
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)
    ld.add_action(physics_manager)

    # Spawn entity after Gazebo is ready
    ld.add_action(RegisterEventHandler(
        OnProcessExit(
            target_action=gazebo,
            on_exit=[spawn_entity],
        )
    ))

    return ld