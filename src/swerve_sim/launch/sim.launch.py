"""Bring up Gazebo, spawn the rover, start the controllers, run the solver.

    ros2 launch swerve_sim sim.launch.py
    ros2 launch swerve_sim sim.launch.py gui:=false        # headless
    ros2 launch swerve_sim sim.launch.py solver:=false     # drive by hand

Controllers are chained off the spawn event rather than started on a timer --
spawning is slow the first time Gazebo loads its model database, and a fixed
sleep is the usual cause of "controller_manager services not available".
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('swerve_sim')
    xacro_file = os.path.join(pkg, 'urdf', 'rover.urdf.xacro')
    world_file = os.path.join(pkg, 'worlds', 'flat.world')

    gui = LaunchConfiguration('gui')
    solver = LaunchConfiguration('solver')

    robot_description = {
        'robot_description': Command(['xacro ', xacro_file])
    }

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': world_file, 'verbose': 'true'}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}],
    )

    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'swerve_rover',
                   '-z', '0.12'],
        output='screen',
    )

    def spawner(name):
        return Node(package='controller_manager', executable='spawner',
                    arguments=[name, '-c', '/controller_manager'],
                    output='screen')

    joint_state_broadcaster = spawner('joint_state_broadcaster')
    steer_controller = spawner('swerve_steer_controller')
    drive_controller = spawner('swerve_drive_controller')

    swerve_node = Node(
        package='my_swerve_control',
        executable='swerve_optimizer',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(solver),
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('solver', default_value='true'),

        gazebo,
        robot_state_publisher,
        spawn,

        # spawn -> broadcaster -> steer -> drive -> solver
        RegisterEventHandler(OnProcessExit(
            target_action=spawn, on_exit=[joint_state_broadcaster])),
        RegisterEventHandler(OnProcessExit(
            target_action=joint_state_broadcaster, on_exit=[steer_controller])),
        RegisterEventHandler(OnProcessExit(
            target_action=steer_controller, on_exit=[drive_controller])),
        RegisterEventHandler(OnProcessExit(
            target_action=drive_controller, on_exit=[swerve_node])),
    ])
