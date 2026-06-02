#!/usr/bin/env python3
"""
Rectangular Loop Navigation Script
====================================
Sends the robot around a rectangle so RTAB-Map gets a loop closure.
Also spawns an obstacle mid-way and notifies the plotter.

Waypoints:
  Start  → Goal 1 → Goal 2 → Goal 3 → Goal 4 (return to start)
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
import subprocess
import time
import math


class RectangularLoop(Node):

    def __init__(self):
        super().__init__('rectangular_loop')

        self.set_parameters([
            rclpy.parameter.Parameter(
                'use_sim_time', rclpy.Parameter.Type.BOOL, True)
        ])

        self._action_client = ActionClient(
            self, NavigateToPose, '/navigate_to_pose')

        # Publisher to notify plotter when obstacle is spawned
        self.obs1_pub = self.create_publisher(
            Float64, '/obstacle_spawned_first', 10)
        self.obs2_pub = self.create_publisher(
            Float64, '/obstacle_spawned_second', 10)

        # Rectangular waypoints — adjust to fit your map
        self.waypoints = [
            ( 1.5, -0.5, 'Goal 1 — right'),
            ( 1.5,  1.0, 'Goal 2 — up'),
            (-2.0,  1.0, 'Goal 3 — left'),
            (-2.0, -0.5, 'Goal 4 — return to start'),
        ]

        self.current_waypoint = 0
        self.obstacle_spawned = False

        self.get_logger().info('Rectangular loop node started.')
        self.get_logger().info('Waiting for Nav2 action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('Nav2 ready. Starting loop.')

        # Start the loop
        self.send_next_goal()

    def send_next_goal(self):
        if self.current_waypoint >= len(self.waypoints):
            self.get_logger().info('=== RECTANGULAR LOOP COMPLETE ===')
            self.get_logger().info(
                'RTAB-Map should have detected a loop closure.')
            self.get_logger().info(
                'Press Ctrl+C in the plotter terminal to save results.')
            return

        x, y, label = self.waypoints[self.current_waypoint]
        self.get_logger().info(
            f'Sending waypoint {self.current_waypoint + 1}/4: {label} '
            f'({x}, {y})')

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp    = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return
        self.get_logger().info('Goal accepted.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def feedback_callback(self, feedback_msg):
        # Spawn obstacle after waypoint 1 is halfway done
        # (when robot is moving from Goal 1 to Goal 2)
        if self.current_waypoint == 1 and not self.obstacle_spawned:
            self.spawn_obstacle()

    def goal_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(
            f'Waypoint {self.current_waypoint + 1} reached.')

        self.current_waypoint += 1

        # Small pause between waypoints
        time.sleep(1.0)

        self.send_next_goal()

    def spawn_obstacle(self):
        """Spawn a moving obstacle and notify the plotter."""
        self.obstacle_spawned = True
        self.get_logger().info('Spawning obstacle...')

        try:
            subprocess.Popen([
                'ros2', 'run', 'ros_gz_sim', 'create',
                '-name', 'moving_box',
                '-file', '/ros_ws/models/gazebo_models/box_target_red/model.sdf',
                '-x', '0.0', '-y', '0.2', '-z', '0.1'
            ])
            self.get_logger().info('Obstacle spawned at (0.0, 0.2)')
        except Exception as e:
            self.get_logger().warn(f'Obstacle spawn failed: {e}')

        # Notify plotter
        msg = Float64()
        msg.data = 0.0
        self.obs1_pub.publish(msg)
        self.get_logger().info('Plotter notified of obstacle 1.')


def main(args=None):
    rclpy.init(args=args)
    node = RectangularLoop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
