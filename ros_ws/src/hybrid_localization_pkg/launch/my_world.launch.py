import os
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_tb3    = get_package_share_directory('turtlebot3_gazebo')
    pkg_gz_sim = get_package_share_directory('ros_gz_sim')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose       = LaunchConfiguration('x_pose', default='-2.0')
    y_pose       = LaunchConfiguration('y_pose', default='-0.5')
    world = '/ros_ws/my_worlds/worlds/turtlebot3_world.world'

    return LaunchDescription([
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH',
            os.path.join(pkg_tb3, 'models')),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH',
            '/ros_ws/my_worlds/worlds'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gz_sim, 'launch', 'gz_sim.launch.py')),
            launch_arguments={
                'gz_args': ['-r -s -v2 ', world],
                'on_exit_shutdown': 'true'
            }.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gz_sim, 'launch', 'gz_sim.launch.py')),
            launch_arguments={
                'gz_args': '-g -v2 ',
                'on_exit_shutdown': 'true'
            }.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_tb3, 'launch', 'robot_state_publisher.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_tb3, 'launch', 'spawn_turtlebot3.launch.py')),
            launch_arguments={
                'x_pose': x_pose,
                'y_pose': y_pose
            }.items()),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/world/default/dynamic_pose/info@gz.msgs.Pose_V@tf2_msgs/msg/TFMessage'
            ],
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link'],
        ),
    ])
