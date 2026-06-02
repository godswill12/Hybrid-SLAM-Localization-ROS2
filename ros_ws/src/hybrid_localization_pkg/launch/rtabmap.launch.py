#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'use_sim_time':                  use_sim_time,
            'frame_id':                      'base_footprint',
            'odom_frame_id':                 'odom',
            'map_frame_id':                  'map',

            # ── LiDAR only ────────────────────────────────────────
            'subscribe_depth':               False,
            'subscribe_rgb':                 False,
            'subscribe_scan':                True,
            'subscribe_scan_cloud':          False,
            'subscribe_odom_info':           False,

            # ── Get odometry from TF tree (not topic) ─────────────
            # Your EKF publishes map->odom TF, RTAB-Map reads it
            'odom_frame_id':                 'odom',
            'wait_for_transform':            0.5,

            'approx_sync':                   True,
            'queue_size':                    10,

            # ── SLAM mode ─────────────────────────────────────────
            'Mem/IncrementalMemory':         'true',
            'Mem/InitWMWithAllNodes':        'false',

            # ── ICP scan matching ─────────────────────────────────
            'Reg/Strategy':                  '1',
            'Icp/VoxelSize':                 '0.05',
            'Icp/MaxCorrespondenceDistance': '0.1',
            'Icp/MaxIterations':             '30',

            # ── Update frequently ─────────────────────────────────
            'RGBD/NeighborLinkRefining':     'true',
            'RGBD/ProximityBySpace':         'true',
            'RGBD/LinearUpdate':             '0.01',
            'RGBD/AngularUpdate':            '0.01',
            'RGBD/OptimizeFromGraphEnd':     'false',

            # ── Graph optimizer ───────────────────────────────────
            'Optimizer/Strategy':            '1',
            'Optimizer/Iterations':          '20',

            # ── Let RTAB-Map publish its own TF for odom ──────────
            # We set this true so RTAB-Map publishes map->odom
            # from its own graph — this is separate from EKF TF
            'publish_tf':                    False,

            # ── Topic namespace ───────────────────────────────────
            # Force all outputs under /rtabmap/ namespace
            'topic_queue_size':              10,
        }],
        remappings=[
            ('scan',           '/scan'),
            ('odom',           '/odom'),
            # Remap outputs to /rtabmap/ namespace
            ('info',           '/rtabmap/info'),
            ('localization_pose', '/rtabmap/localization_pose'),
            ('map',            '/rtabmap/map'),
            ('cloud_map',      '/rtabmap/cloud_map'),
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        rtabmap_node,
    ])