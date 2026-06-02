from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hybrid_localization_pkg',
            executable='ekf_node',
            name='ekf_node',
            output='screen'),
        Node(
            package='hybrid_localization_pkg',
            executable='eif_node',
            name='eif_node',
            output='screen'),
        Node(
            package='hybrid_localization_pkg',
            executable='amcl_interface_node',
            name='amcl_interface',
            output='screen'),
        Node(
            package='hybrid_localization_pkg',
            executable='hybrid_fusion_node',
            name='hybrid_fusion',
            output='screen'),
        Node(
            package='hybrid_localization_pkg',
            executable='evaluation_node',
            name='evaluation_node',
            output='screen'),
        Node(
            package='hybrid_localization_pkg',
            executable='ground_truth_publisher',
            name='ground_truth_publisher',
            output='screen'),
    ])
