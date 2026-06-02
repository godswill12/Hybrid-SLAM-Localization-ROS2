#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from math import cos, sin
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from tf_transformations import euler_from_quaternion, quaternion_from_euler

class EKFNode(Node):
    def __init__(self):
        super().__init__('ekf_node')
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.imu_sub  = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/pose_ekf', 10)
        self.state = np.zeros(3)
        self.P     = np.eye(3) * 0.1
        self.Q     = np.diag([0.05, 0.05, 0.02])
        self.R_imu = np.array([[0.01]])
        self.last_time    = None
        self.imu_yaw_rate = 0.0
        self.get_logger().info('EKF node started')

    def imu_callback(self, msg):
        self.imu_yaw_rate = msg.angular_velocity.z

    def odom_callback(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last_time is None:
            self.last_time = now
            return
        dt = now - self.last_time
        self.last_time = now
        if dt <= 0.0 or dt > 1.0:
            return
        v = msg.twist.twist.linear.x
        w = msg.twist.twist.angular.z
        self.predict(v, w, dt)
        self.update_imu(self.imu_yaw_rate, dt)
        self.publish_pose()

    def predict(self, v, w, dt):
        x, y, theta = self.state
        F = np.array([
            [1.0, 0.0, -v * sin(theta) * dt],
            [0.0, 1.0,  v * cos(theta) * dt],
            [0.0, 0.0,  1.0              ]
        ])
        self.state = np.array([
            x + v * cos(theta) * dt,
            y + v * sin(theta) * dt,
            theta + w * dt
        ])
        self.P = F @ self.P @ F.T + self.Q

    def update_imu(self, imu_w, dt):
        H   = np.array([[0.0, 0.0, 1.0 / dt]])
        z   = np.array([imu_w])
        S   = H @ self.P @ H.T + self.R_imu
        K   = self.P @ H.T @ np.linalg.inv(S)
        self.state = self.state + K.flatten() * (z - H @ self.state)[0]
        self.P     = (np.eye(3) - K @ H) @ self.P

    def publish_pose(self):
        x, y, theta = self.state
        q   = quaternion_from_euler(0.0, 0.0, float(theta))
        msg = PoseStamped()
        msg.header.stamp     = self.get_clock().now().to_msg()
        msg.header.frame_id  = 'map'
        msg.pose.position.x  = float(x)
        msg.pose.position.y  = float(y)
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]
        self.pose_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = EKFNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
