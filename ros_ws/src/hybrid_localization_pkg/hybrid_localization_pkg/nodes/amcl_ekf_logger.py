#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import csv
import os
from datetime import datetime
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
from tf_transformations import euler_from_quaternion


class AMCLEKFLogger(Node):
    def __init__(self):
        super().__init__('amcl_ekf_logger')

        # ── Subscriptions ─────────────────────────────────────────────
        # Your custom EKF output
        self.create_subscription(
            Odometry, '/odometry/filtered', self.ekf_callback, 10)

        # AMCL particle filter
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.amcl_callback, 10)

        # Ground truth from Gazebo
        self.create_subscription(
            Pose, '/ground_truth', self.gt_callback, 10)

        # ── CSV setup ─────────────────────────────────────────────────
        output_dir = os.path.expanduser('~/amcl_ekf_results')
        os.makedirs(output_dir, exist_ok=True)

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filepath  = os.path.join(output_dir, f'amcl_ekf_{timestamp}.csv')

        self.csv_file = open(filepath, 'w', newline='')
        self.writer   = csv.writer(self.csv_file)

        self.writer.writerow([
            'ros_timestamp', 'source',
            'x', 'y', 'theta',
            'cov_x', 'cov_y', 'cov_theta'
        ])
        self.csv_file.flush()

        # ── Counters ──────────────────────────────────────────────────
        self.ekf_count  = 0
        self.amcl_count = 0
        self.gt_count   = 0

        self.create_timer(5.0, self.print_summary)
        self.get_logger().info(f'Logger started — saving to: {filepath}')
        self.get_logger().info(
            'Logging: /odometry/filtered  /amcl_pose  /ground_truth')

    # ── Custom EKF (/odometry/filtered → Odometry) ────────────────────
    def ekf_callback(self, msg):
        ts        = self.get_clock().now().nanoseconds * 1e-9
        x         = msg.pose.pose.position.x
        y         = msg.pose.pose.position.y
        q         = msg.pose.pose.orientation
        _, _, theta = euler_from_quaternion([q.x, q.y, q.z, q.w])
        cov_x     = msg.pose.covariance[0]
        cov_y     = msg.pose.covariance[7]
        cov_theta = msg.pose.covariance[35]
        self.writer.writerow([
            f'{ts:.6f}', 'ekf',
            f'{x:.6f}', f'{y:.6f}', f'{theta:.6f}',
            f'{cov_x:.6f}', f'{cov_y:.6f}', f'{cov_theta:.6f}'
        ])
        self.csv_file.flush()
        self.ekf_count += 1

    # ── AMCL (/amcl_pose → PoseWithCovarianceStamped) ─────────────────
    def amcl_callback(self, msg):
        ts        = self.get_clock().now().nanoseconds * 1e-9
        x         = msg.pose.pose.position.x
        y         = msg.pose.pose.position.y
        q         = msg.pose.pose.orientation
        _, _, theta = euler_from_quaternion([q.x, q.y, q.z, q.w])
        cov_x     = msg.pose.covariance[0]
        cov_y     = msg.pose.covariance[7]
        cov_theta = msg.pose.covariance[35]
        self.writer.writerow([
            f'{ts:.6f}', 'amcl',
            f'{x:.6f}', f'{y:.6f}', f'{theta:.6f}',
            f'{cov_x:.6f}', f'{cov_y:.6f}', f'{cov_theta:.6f}'
        ])
        self.csv_file.flush()
        self.amcl_count += 1

    # ── Ground truth (/ground_truth → Pose) ───────────────────────────
    def gt_callback(self, msg):
        ts        = self.get_clock().now().nanoseconds * 1e-9
        x         = msg.position.x
        y         = msg.position.y
        q         = msg.orientation
        _, _, theta = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.writer.writerow([
            f'{ts:.6f}', 'gt',
            f'{x:.6f}', f'{y:.6f}', f'{theta:.6f}',
            '0.0', '0.0', '0.0'
        ])
        self.csv_file.flush()
        self.gt_count += 1

    # ── Summary ───────────────────────────────────────────────────────
    def print_summary(self):
        self.get_logger().info(
            f'Logged — '
            f'EKF: {self.ekf_count}  '
            f'AMCL: {self.amcl_count}  '
            f'GT: {self.gt_count}'
        )

    def destroy_node(self):
        self.csv_file.close()
        self.get_logger().info(
            f'CSV closed — '
            f'EKF: {self.ekf_count}  '
            f'AMCL: {self.amcl_count}  '
            f'GT: {self.gt_count}'
        )
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AMCLEKFLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()