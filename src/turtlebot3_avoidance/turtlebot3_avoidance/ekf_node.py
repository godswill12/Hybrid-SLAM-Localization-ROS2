#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from tf_transformations import euler_from_quaternion, quaternion_from_euler

class EKFLocalization(Node):
    def __init__(self):
        super().__init__('ekf_localization')

        # State [x, y, theta]
        self.x = np.zeros((3, 1))
        self.P = np.eye(3) * 0.1  # covariance

        # Noise matrices
        self.Q = np.diag([0.01, 0.01, 0.01])  # process noise
        self.R = np.array([[0.05]])          # measurement noise (imu yaw)

        self.last_time = None
        self.v = 0.0
        self.w = 0.0
        self.imu_yaw = 0.0
        self.imu_ready = False

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)

        self.pub = self.create_publisher(PoseStamped, '/ekf_pose', 10)

        self.timer = self.create_timer(0.02, self.step)  # 50 Hz

    def imu_callback(self, msg):
        q = msg.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.imu_yaw = yaw
        self.imu_ready = True

    def odom_callback(self, msg):
        self.v = msg.twist.twist.linear.x
        self.w = msg.twist.twist.angular.z

        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_time is None:
            self.last_time = t
        self.dt = t - self.last_time
        self.last_time = t

    def step(self):
        if self.last_time is None or not self.imu_ready:
            return

        dt = self.dt if self.dt > 0 else 0.02
        theta = self.x[2, 0]

        # ---------- Prediction ----------
        F = np.array([
            [1, 0, -self.v*np.sin(theta)*dt],
            [0, 1,  self.v*np.cos(theta)*dt],
            [0, 0, 1]
        ])

        B = np.array([
            [np.cos(theta)*dt, 0],
            [np.sin(theta)*dt, 0],
            [0, dt]
        ])

        u = np.array([[self.v], [self.w]])

        self.x = self.x + B @ u
        self.P = F @ self.P @ F.T + self.Q

        # ---------- Update (IMU yaw) ----------
        z = np.array([[self.imu_yaw]])

        H = np.array([[0, 0, 1]])
        y = z - H @ self.x

        # normalize angle residual
        y[0,0] = np.arctan2(np.sin(y[0,0]), np.cos(y[0,0]))

        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        I = np.eye(3)
        self.P = (I - K @ H) @ self.P

        self.publish_pose()

    def publish_pose(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"

        msg.pose.position.x = float(self.x[0])
        msg.pose.position.y = float(self.x[1])
        msg.pose.position.z = 0.0

        q = quaternion_from_euler(0, 0, float(self.x[2]))
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]

        self.pub.publish(msg)

def main():
    rclpy.init()
    node = EKFLocalization()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
