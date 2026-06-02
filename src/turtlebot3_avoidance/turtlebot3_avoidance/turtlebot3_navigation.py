#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np

from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from tf_transformations import euler_from_quaternion


class EKFNavigator(Node):
    def __init__(self):
        super().__init__('ekf_navigator')

        # goal point B
        self.gx = 2.0
        self.gy = 2.0

        # current estimated pose from EKF
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.pose_ready = False

        # latest lidar scan
        self.scan = None

        # parameters
        self.obs_threshold = 0.5   # obstacle distance [m]
        self.max_lin = 0.2
        self.max_ang = 0.8

        self.create_subscription(PoseStamped, '/ekf_pose',
                                 self.pose_callback, 10)
        self.create_subscription(LaserScan, '/scan',
                                 self.scan_callback, 10)

        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        self.timer = self.create_timer(0.1, self.control_loop)  # 10 Hz

    def pose_callback(self, msg):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y

        q = msg.pose.orientation
        _, _, self.theta = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.pose_ready = True

    def scan_callback(self, msg):
        self.scan = msg

    def control_loop(self):
        if not self.pose_ready or self.scan is None:
            return

        # -------- direction to goal --------
        dx = self.gx - self.x
        dy = self.gy - self.y
        dist_to_goal = np.hypot(dx, dy)

        angle_to_goal = np.arctan2(dy, dx)
        heading_error = angle_to_goal - self.theta
        heading_error = np.arctan2(np.sin(heading_error),
                                   np.cos(heading_error))  # normalize

        # -------- check front lidar sector --------
        ranges = np.array(self.scan.ranges)
        ranges[np.isinf(ranges)] = 10.0

        n = len(ranges)
        front_width = int(0.1 * n)  # ~front sector (10% of scan)

        front = np.concatenate((ranges[:front_width],
                                ranges[-front_width:]))

        left = ranges[int(0.25*n):int(0.5*n)]
        right = ranges[int(0.5*n):int(0.75*n)]

        min_front = np.min(front)
        min_left = np.min(left)
        min_right = np.min(right)

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "base_link"

        # -------- goal reached --------
        if dist_to_goal < 0.1:
            cmd.twist.linear.x = 0.0
            cmd.twist.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            self.get_logger().info("Goal reached")  
            return

        # -------- obstacle avoidance --------
        if min_front < self.obs_threshold:
            # turn away from closer side
            if min_left < min_right:
                cmd.twist.angular.z = -self.max_ang  # turn right
            else:
                cmd.twist.angular.z = self.max_ang   # turn left
            cmd.linear.x = 0.0
        else:
            # drive toward goal
            cmd.twist.linear.x = min(self.max_lin, dist_to_goal)
            cmd.twist.angular.z = np.clip(2.0 * heading_error,
                                    -self.max_ang, self.max_ang)

        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = EKFNavigator()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
