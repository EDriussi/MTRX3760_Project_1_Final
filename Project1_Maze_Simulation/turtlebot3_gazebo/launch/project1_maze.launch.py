#!/usr/bin/env python3
#
# MTRX3760 Project 1 - TurtleBot3 maze launch file
# SID: 510516950
#
# Purpose:
#   Starts the Project 1 SDF world in Gazebo Harmonic, spawns the selected
#   TurtleBot3 at the maze entrance, publishes the robot state, creates the
#   ROS-Gazebo bridges used by the official Jazzy simulation package, and
#   starts the team's LidarNode + Navigator + WheelController + CameraNode and
#   RViz, so the whole stack comes up with a single launch command.
#
# This file is based on the ROBOTIS Jazzy empty_world.launch.py structure.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """Create the complete Gazebo and TurtleBot3 launch description."""
    turtlebot_launch_dir = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'launch'
    )
    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')

    use_sim_time = LaunchConfiguration('use_sim_time')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')

    world_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds',
        'project1_maze.world'
    )

    rviz_config_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'rviz',
        'tb3_gazebo.rviz'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use the Gazebo simulation clock'
    )
    declare_x_pose = DeclareLaunchArgument(
        'x_pose',
        default_value='-0.75',
        description='Initial TurtleBot3 x coordinate in metres'
    )
    declare_y_pose = DeclareLaunchArgument(
        'y_pose',
        default_value='-0.80',
        description='Initial TurtleBot3 y coordinate in metres'
    )

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r -s -v2 ', world_path],
            'on_exit_shutdown': 'true'
        }.items()
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v2 '}.items()
    )

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                turtlebot_launch_dir,
                'robot_state_publisher.launch.py'
            )
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    spawn_turtlebot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot_launch_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )

    append_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(
            get_package_share_directory('turtlebot3_gazebo'),
            'models'
        )
    )

    lidar_node = Node(
        package='turtlebot3_lidar_processing',
        executable='lidar',
        name='lidar_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    navigator_node = Node(
        package='turtlebot3_wall_follower',
        executable='turtlebot3_navigator',
        name='turtlebot3_navigator',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Single owner of /cmd_vel: forwards the navigator's /nav_cmd_vel to the
    # wheels with speed limits and a stop-on-timeout watchdog
    wheel_controller_node = Node(
        package='wheel_controller',
        executable='wheel_controller_node',
        name='wheel_controller',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Camera processing node (turtlebot3_camera_processing): subscribes to
    # /camera/image_raw and republishes it on /image
    camera_node = Node(
        package='turtlebot3_camera_processing',
        executable='camera_node',
        name='camera_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    launch_description = LaunchDescription()
    launch_description.add_action(declare_use_sim_time)
    launch_description.add_action(declare_x_pose)
    launch_description.add_action(declare_y_pose)
    launch_description.add_action(append_resource_path)
    launch_description.add_action(gazebo_server)
    launch_description.add_action(gazebo_client)
    launch_description.add_action(robot_state_publisher)
    launch_description.add_action(spawn_turtlebot)
    launch_description.add_action(lidar_node)
    launch_description.add_action(navigator_node)
    launch_description.add_action(wheel_controller_node)
    launch_description.add_action(camera_node)
    launch_description.add_action(rviz_node)

    return launch_description
