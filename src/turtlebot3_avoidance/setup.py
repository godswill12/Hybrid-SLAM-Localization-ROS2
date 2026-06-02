from setuptools import find_packages, setup

package_name = 'turtlebot3_avoidance'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'avoidance = turtlebot3_avoidance.obstacle_avoidance:main',
            'cmd_vel_gz_publisher = turtlebot3_avoidance.cmd_vel_gz_publisher:main',
            'ekf_node = turtlebot3_avoidance.ekf_node:main', 
            'turtlebot3_navigation = turtlebot3_avoidance.turtlebot3_navigation:main',
            'goal_point_visual_node = turtlebot3_avoidance.goal_point_visual_node:main',
            'twist_stamped_to_twist = turtlebot3_avoidance.twist_stamped_to_twist:main',  
        ],
    },
)
