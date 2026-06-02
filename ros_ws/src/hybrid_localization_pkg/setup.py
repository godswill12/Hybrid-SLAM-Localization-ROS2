import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'hybrid_localization_pkg'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'params'),
            glob('params/*.yaml')),
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*')),
        (os.path.join('share', package_name, 'maps'),
            glob('maps/*')),
        (os.path.join('share', package_name, 'models', 'turtlebot3_waffle_pi'),
            glob('models/turtlebot3_waffle_pi/*')),
        ('share/' + package_name + '/launch', ['launch/ekf_launch.py']),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Hybrid EKF + EIF + AMCL localization for TurtleBot3 thesis',
    license='MIT',
    entry_points={
        'console_scripts': [
        'ekf_node               = hybrid_localization_pkg.nodes.ekf_node:main',
        'eif_node               = hybrid_localization_pkg.nodes.eif_node:main',
        'ekf_node_no_amcl       = hybrid_localization_pkg.nodes.ekf_node_no_amcl:main',
        'eif_node_no_amcl       = hybrid_localization_pkg.nodes.eif_node_no_amcl:main',
        'plotter                = hybrid_localization_pkg.nodes.plotter:main',
        'amcl_interface_node    = hybrid_localization_pkg.nodes.amcl_interface_node:main',
        'hybrid_fusion_node     = hybrid_localization_pkg.nodes.hybrid_fusion_node:main',
        'evaluation_node        = hybrid_localization_pkg.nodes.evaluation_node:main',
        'ground_truth_publisher = hybrid_localization_pkg.nodes.ground_truth_publisher:main',
        'amcl_ekf_logger        = hybrid_localization_pkg.nodes.amcl_ekf_logger:main',
        'fusion_node            = hybrid_localization_pkg.nodes.fusion_node:main',
        'rectangular_loop       = hybrid_localization_pkg.nodes.rectangular_loop:main',
        'rtabmap_pose_bridge    = hybrid_localization_pkg.nodes.rtabmap_pose_bridge:main',
        'ekf_rtab_node          = hybrid_localization_pkg.nodes.ekf_rtab_node:main',
        'compare_analysis       = hybrid_localization_pkg.nodes.compare_analysis:main',
        'experiment_runner      = hybrid_localization_pkg.nodes.experiment_runner:main',
            'navigator              = hybrid_localization_pkg.nodes.navigator:main',
        'resource_monitor       = hybrid_localization_pkg.nodes.resource_monitor:main',
        'smoothness_monitor = hybrid_localization_pkg.nodes.smoothness_monitor:main',
        'dynamic_obstacle_controller = hybrid_localization_pkg.nodes.dynamic_obstacle_controller:main',
        ],
    },
)
