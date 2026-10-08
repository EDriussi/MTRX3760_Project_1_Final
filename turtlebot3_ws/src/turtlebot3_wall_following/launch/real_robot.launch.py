#!/usr/bin/env python3
#
# MTRX3760 Project 1 - physical TurtleBot3 launch file (no Gazebo)
# SID: 510516950
#
# Purpose:
#   Starts the team's LidarNode + Navigator + WheelController against a REAL
#   TurtleBot3 Burger. Gazebo, the ROBOTIS spawn/bridge launch files and the
#   simulated clock are NOT used - everything runs on wall time.
#
# How it's used (Option A: our nodes run on the laptop, robot runs bringup):
#
#   On the robot (ssh ubuntu@10.70.139.9):
#     export TURTLEBOT3_MODEL=burger
#     export LDS_MODEL=LDS-01            # or LDS-02
#     ros2 launch turtlebot3_bringup robot.launch.py
#
#   On the laptop (same Wi-Fi, same ROS_DOMAIN_ID as the robot):
#     ros2 launch turtlebot3_wall_follower real_robot.launch.py
#
# Arguments:
#   stamped_cmd_vel:=true|false  Message type the robot expects on /cmd_vel.
#                                Check on the robot with
#                                  ros2 topic info /cmd_vel -v
#                                TwistStamped -> true (default), Twist -> false
#   use_rviz:=true|false         Open RViz on the laptop (default true)
#   use_camera:=true|false       Start the camera processing node (default
#                                false - needs a camera driver on the robot
#                                publishing /camera/image_raw)

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Create the launch description for the physical TurtleBot3."""
    stamped_cmd_vel = LaunchConfiguration('stamped_cmd_vel')
    use_rviz = LaunchConfiguration('use_rviz')
    use_camera = LaunchConfiguration('use_camera')

    # Real robot: nodes run on the computer's clock, not Gazebo's /clock.
    # With use_sim_time true the navigator's timer would wait forever for a
    # /clock message that never arrives and the robot would never move.
    use_sim_time = False

    rviz_config_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'rviz',
        'tb3_gazebo.rviz'
    )

    declare_stamped_cmd_vel = DeclareLaunchArgument(
        'stamped_cmd_vel',
        default_value='true',
        description='Publish TwistStamped (true) or Twist (false) on /cmd_vel'
    )
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Open RViz'
    )
    declare_use_camera = DeclareLaunchArgument(
        'use_camera',
        default_value='false',
        description='Start the camera processing node'
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
        parameters=[{
            'use_sim_time': use_sim_time,
            'use_stamped_cmd_vel': ParameterValue(stamped_cmd_vel, value_type=bool),
        }]
    )

    # Camera processing node: republishes /camera/image_raw on /image
    camera_node = Node(
        package='turtlebot3_camera_processing',
        executable='camera_node',
        name='camera_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_camera)
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(use_rviz)
    )

    launch_description = LaunchDescription()
    launch_description.add_action(declare_stamped_cmd_vel)
    launch_description.add_action(declare_use_rviz)
    launch_description.add_action(declare_use_camera)
    launch_description.add_action(lidar_node)
    launch_description.add_action(navigator_node)
    launch_description.add_action(wheel_controller_node)
    launch_description.add_action(camera_node)
    launch_description.add_action(rviz_node)

    return launch_description
