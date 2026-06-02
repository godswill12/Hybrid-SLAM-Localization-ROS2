from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('hybrid_localization_pkg'),  # ✅ fixed
        'config',
        'ekf_params.yaml'
    )

    ekf_node = Node(
        package='hybrid_localization_pkg',   # ✅ fixed
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[
        config,
        {'use_sim_time': True}   # ← add this
    ]
    )

    return LaunchDescription([
        ekf_node,
    ])