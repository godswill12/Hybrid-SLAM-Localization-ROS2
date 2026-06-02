#!/usr/bin/env python3
"""
Dynamic Obstacle Controller
============================
Moves a cylinder obstacle back and forth across the robot's path.
Compatible with Gazebo Harmonic (gz-sim) using VelocityControl plugin.

The obstacle moves in the Y direction (perpendicular to robot's
forward X direction), crossing the robot's scan field repeatedly.

Spawn the obstacle first:
  ros2 run ros_gz_sim create \
    -name moving_obstacle \
    -file /ros_ws/models/gazebo_models/moving_obstacle/model.sdf \
    -x 1.2 -y -1.5 -z 0.5
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import subprocess


class DynamicObstacleController(Node):

    def __init__(self):
        super().__init__('dynamic_obstacle_controller')

        self.set_parameters([
            rclpy.parameter.Parameter(
                'use_sim_time', rclpy.Parameter.Type.BOOL, True)
        ])

        self.declare_parameter('speed',           0.3)
        self.declare_parameter('travel_distance', 2.0)
        self.declare_parameter('active',          True)
        self.declare_parameter('model_name', 'moving_obstacle')

        self.speed      = self.get_parameter('speed').value
        self.distance   = self.get_parameter('travel_distance').value
        self.active     = self.get_parameter('active').value
        self.model_name = self.get_parameter('model_name').value

        # gz-sim VelocityControl plugin topic format:
        # /model/<model_name>/cmd_vel
        cmd_topic = f'/model/{self.model_name}/cmd_vel'

        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)

        # Notify plotter on each direction reversal
        self.cross_pub = self.create_publisher(
            Float64, '/obstacle_spawned_second', 10)

        # State
        self.direction     =  1.0
        self.distance_done =  0.0
        self.last_time     = None
        self.cross_count   =  0

        self.create_timer(0.1, self.update)

        self.get_logger().info(
            f'Dynamic obstacle controller started.\n'
            f'Publishing to: {cmd_topic}\n'
            f'Speed: {self.speed}m/s  '
            f'Travel distance: {self.distance}m'
        )

    def update(self):
        if not self.active:
            return

        now = self.get_clock().now().nanoseconds * 1e-9

        if self.last_time is None:
            self.last_time = now
            return

        dt = now - self.last_time
        self.last_time = now

        if dt <= 0.0 or dt > 1.0:
            return

        # Send velocity command — move in Y direction
        cmd = Twist()
        cmd.linear.x  = 0.0
        cmd.linear.y  = self.direction * self.speed
        cmd.linear.z  = 0.0
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)

        self.distance_done += self.speed * dt

        # Reverse when travel distance reached
        if self.distance_done >= self.distance:
            self.direction    *= -1.0
            self.distance_done = 0.0
            self.cross_count  += 1

            # Notify plotter
            msg = Float64()
            msg.data = float(now)
            self.cross_pub.publish(msg)

            self.get_logger().info(
                f'Crossing #{self.cross_count} — reversing direction')


def main(args=None):
    rclpy.init(args=args)
    node = DynamicObstacleController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Send stop command before exiting
        stop = Twist()
        node.cmd_pub.publish(stop)
        node.get_logger().info('Obstacle stopped.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
