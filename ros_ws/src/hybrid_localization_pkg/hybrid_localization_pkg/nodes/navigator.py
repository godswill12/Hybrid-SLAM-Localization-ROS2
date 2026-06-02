#!/usr/bin/env python3
"""
Navigator Node
==============
Navigates the robot through 4 waypoints in a rectangular loop.
Waits for the robot to fully reach each waypoint before moving
to the next one. Repeats for a specified number of iterations.

Waypoints:
  1. (2.5,  0.0)
  2. (2.5,  1.0)
  3. (0.0,  1.0)
  4. (0.0,  0.0)  ← back to start = one iteration complete

Usage:
  ros2 run hybrid_localization_pkg navigator
  ros2 run hybrid_localization_pkg navigator --ros-args -p num_iterations:=4
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import time
import threading

from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus


# ── Waypoints ─────────────────────────────────────────────────────────────────

WAYPOINTS = [
    (2.5,  0.0,  'WP1 — forward right'),
    (2.5,  1.0,  'WP2 — turn up'),
    (0.0,  1.0,  'WP3 — back left'),
    (0.0,  0.0,  'WP4 — return to start'),
]


class Navigator(Node):

    def __init__(self):
        super().__init__('navigator')

        self.set_parameters([
            rclpy.parameter.Parameter(
                'use_sim_time', rclpy.Parameter.Type.BOOL, True)
        ])

        self.declare_parameter('num_iterations', 4)
        self.declare_parameter('nav_timeout',  120.0)
        self.declare_parameter('pause_between_waypoints', 1.0)
        self.declare_parameter('pause_between_iterations', 3.0)

        self.num_iterations  = self.get_parameter('num_iterations').value
        self.nav_timeout     = self.get_parameter('nav_timeout').value
        self.pause_wp        = self.get_parameter('pause_between_waypoints').value
        self.pause_iter      = self.get_parameter('pause_between_iterations').value

        self.nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose')

        self.get_logger().info(
            f'\n{"="*50}\n'
            f'  NAVIGATOR READY\n'
            f'  Iterations: {self.num_iterations}\n'
            f'  Waypoints:\n' +
            '\n'.join(f'    {i+1}. ({x}, {y})  {label}'
                      for i, (x, y, label) in enumerate(WAYPOINTS)) +
            f'\n  Starting in 3 seconds...\n'
            f'{"="*50}'
        )

        # Start navigation in a background thread
        self._started = False
        self.create_timer(3.0, self._start_once)

    def _start_once(self):
        if self._started:
            return
        self._started = True
        threading.Thread(target=self.run, daemon=True).start()

    # ── Main loop ─────────────────────────────────────────────────────────────

    def run(self):
        # Wait for Nav2 action server
        self.get_logger().info('Waiting for Nav2 action server...')
        if not self.nav_client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error(
                'Nav2 action server not available after 30s. '
                'Is Nav2 running?')
            return
        self.get_logger().info('Nav2 ready.')

        for iteration in range(1, self.num_iterations + 1):
            self.get_logger().info(
                f'\n{"─"*50}\n'
                f'  ITERATION {iteration} / {self.num_iterations}\n'
                f'{"─"*50}')

            iteration_success = True

            for wp_idx, (x, y, label) in enumerate(WAYPOINTS):
                self.get_logger().info(
                    f'[Iter {iteration}] Navigating to {label} '
                    f'({x}, {y})')

                success = self.navigate_to(x, y, label, iteration)

                if success:
                    self.get_logger().info(
                        f'[Iter {iteration}] ✓ {label} reached')
                else:
                    self.get_logger().warn(
                        f'[Iter {iteration}] ✗ {label} failed — '
                        f'continuing to next waypoint')
                    iteration_success = False

                # Pause between waypoints
                if wp_idx < len(WAYPOINTS) - 1:
                    time.sleep(self.pause_wp)

            # Iteration complete
            status = '✓ COMPLETE' if iteration_success else '⚠ PARTIAL'
            self.get_logger().info(
                f'\n[Iter {iteration}] Loop {status}')

            # Pause between iterations (except after last one)
            if iteration < self.num_iterations:
                self.get_logger().info(
                    f'Pausing {self.pause_iter}s before next iteration...')
                time.sleep(self.pause_iter)

        # All done
        self.get_logger().info(
            f'\n{"="*50}\n'
            f'  ALL {self.num_iterations} ITERATIONS COMPLETE\n'
            f'{"="*50}')

    # ── Navigate to single waypoint ───────────────────────────────────────────

    def navigate_to(self, x, y, label, iteration):
        """
        Send goal and WAIT until robot reaches it or times out.
        Returns True if goal succeeded, False otherwise.
        """

        # Build goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id    = 'map'
        goal_msg.pose.header.stamp       = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x    = float(x)
        goal_msg.pose.pose.position.y    = float(y)
        goal_msg.pose.pose.position.z    = 0.0
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        # Send goal
        send_future = self.nav_client.send_goal_async(goal_msg)

        # Wait for goal to be accepted
        deadline = time.time() + 10.0
        while not send_future.done():
            time.sleep(0.05)
            if time.time() > deadline:
                self.get_logger().error(
                    f'[Iter {iteration}] {label} — '
                    f'goal acceptance timed out')
                return False

        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error(
                f'[Iter {iteration}] {label} — goal rejected by Nav2')
            return False

        self.get_logger().info(
            f'[Iter {iteration}] {label} — goal accepted, navigating...')

        # Get result future — this is what we wait on
        result_future = goal_handle.get_result_async()

        # Wait for navigation to complete or timeout
        deadline = time.time() + self.nav_timeout
        while not result_future.done():
            time.sleep(0.3)
            elapsed = self.nav_timeout - (deadline - time.time())
            if time.time() > deadline:
                self.get_logger().warn(
                    f'[Iter {iteration}] {label} — '
                    f'navigation timed out after {self.nav_timeout}s')
                # Cancel the goal
                try:
                    goal_handle.cancel_goal_async()
                    time.sleep(1.0)
                except Exception:
                    pass
                return False

        # Check result
        result = result_future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            return True
        else:
            self.get_logger().warn(
                f'[Iter {iteration}] {label} — '
                f'navigation ended with status {result.status}')
            return False


def main(args=None):
    rclpy.init(args=args)
    node = Navigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Navigator stopped.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
