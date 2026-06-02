#!/usr/bin/env python3
"""
Custom World + Nav2 Launch
===========================
Launches your custom world with Nav2 and AMCL using your saved map.

Usage:
  ros2 launch hybrid_localization_pkg custom_world_nav2.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable,
                             IncludeLaunchDescription,
                             DeclareLaunchArgument)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_tb3    = get_package_share_directory('turtlebot3_gazebo')
    pkg_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_nav2   = get_package_share_directory('nav2_bringup')
    pkg_hybrid = get_package_share_directory('hybrid_localization_pkg')

    # ── Paths ─────────────────────────────────────────────────────────────
    world_file  = '/ros_ws/my_worlds/worlds/turtlebot3_world.world'
    map_file    = '/ros_ws/ros_ws/src/hybrid_localization_pkg/maps/my_map.yaml'
    nav2_params = os.path.join(pkg_hybrid, 'config', 'nav2_params.yaml')

    # ── Launch configurations ─────────────────────────────────────────────
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose       = LaunchConfiguration('x_pose',       default='-2.0')
    y_pose       = LaunchConfiguration('y_pose',       default='-0.5')

    return LaunchDescription([

        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('x_pose',       default_value='-2.0'),
        DeclareLaunchArgument('y_pose',       default_value='-0.5'),

        # ── Resource paths ─────────────────────────────────────────────────
        AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(pkg_tb3, 'models')),
        AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            '/ros_ws/my_worlds/worlds'),
        AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            '/ros_ws/models/gazebo_models'),

        # ── Gazebo server ──────────────────────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gz_sim, 'launch', 'gz_sim.launch.py')),
            launch_arguments={
                'gz_args':          ['-r -s -v2 ', world_file],
                'on_exit_shutdown': 'true'
            }.items()),

        # ── Gazebo client GUI ──────────────────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gz_sim, 'launch', 'gz_sim.launch.py')),
            launch_arguments={
                'gz_args':          '-g -v2 ',
                'on_exit_shutdown': 'true'
            }.items()),

        # ── Robot state publisher ──────────────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_tb3, 'launch',
                             'robot_state_publisher.launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time}.items()),

        # ── Spawn TurtleBot3 at map origin ─────────────────────────────────
        # Spawn position matches your map origin offset so the robot
        # starts at a known position relative to the saved map.
        # Your map origin is [-0.935, -2.071] so spawning at (-2.0, -0.5)
        # places the robot inside the mapped area.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_tb3, 'launch',
                             'spawn_turtlebot3.launch.py')),
            launch_arguments={
                'x_pose': x_pose,
                'y_pose': y_pose
            }.items()),

        # ── Nav2 with your custom map ──────────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav2, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'map':          map_file,
                'params_file':  nav2_params,
                'use_sim_time': use_sim_time,
                'autostart':    'true'
            }.items()),

        # ── Ground truth bridge ────────────────────────────────────────────
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/world/default/dynamic_pose/info'
                '@gz.msgs.Pose_V@tf2_msgs/msg/TFMessage'
            ],
            output='screen'),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0','0','0','0','0','0','world','base_link']),
    ])
