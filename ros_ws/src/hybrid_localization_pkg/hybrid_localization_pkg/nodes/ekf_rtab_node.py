#!/usr/bin/env python3
"""
EKF-RTAB Localization Node
==========================
Identical EKF architecture to ekf_node.py but uses
RTAB-Map pose as the measurement update instead of AMCL.

State:    [x, y, theta]  — MAP frame
Predict:  /odom velocity
Update:   /rtabmap_pose  (from rtabmap_pose_bridge)

Publishes:
  /ekf_rtab_pose   (nav_msgs/Odometry, frame_id = map)

Does NOT publish TF — ekf_node handles map->odom TF.
This node is purely for comparison/evaluation.
"""

import rclpy
from rclpy.node import Node
import numpy as np
import math

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion


def quaternion_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.w = math.cos(yaw / 2.0)
    q.z = math.sin(yaw / 2.0)
    q.x = 0.0
    q.y = 0.0
    return q

def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))

def joseph_update(P, K, H, R):
    IKH = np.eye(P.shape[0]) - K @ H
    return IKH @ P @ IKH.T + K @ R @ K.T

def make_pd(M, min_eig=1e-6):
    M = (M + M.T) / 2.0
    eigvals, eigvecs = np.linalg.eigh(M)
    eigvals = np.maximum(eigvals, min_eig)
    return eigvecs @ np.diag(eigvals) @ eigvecs.T


class EKFRtabNode(Node):

    def __init__(self):
        super().__init__('ekf_rtab_node')

        self.declare_parameter('process_noise_x',     0.1)
        self.declare_parameter('process_noise_y',     0.1)
        self.declare_parameter('process_noise_theta', 0.05)
        self.declare_parameter('ekf_rate_hz',         50.0)
        self.declare_parameter('rtab_noise_x',        0.05)
        self.declare_parameter('rtab_noise_y',        0.05)
        self.declare_parameter('rtab_noise_theta',    0.02)

        self.set_parameters([
            rclpy.parameter.Parameter(
                'use_sim_time', rclpy.Parameter.Type.BOOL, True)
        ])

        # ── State ─────────────────────────────────────────────────────────
        self.x = np.zeros((3, 1))
        self.P = np.eye(3) * 0.5

        self.Q = np.diag([
            self.get_parameter('process_noise_x').value,
            self.get_parameter('process_noise_y').value,
            self.get_parameter('process_noise_theta').value
        ])

        # Fixed measurement noise for RTAB-Map
        # (RTAB-Map odometry does not publish covariance reliably)
        self.R_rtab = np.diag([
            self.get_parameter('rtab_noise_x').value,
            self.get_parameter('rtab_noise_y').value,
            self.get_parameter('rtab_noise_theta').value
        ])

        # ── Cache ─────────────────────────────────────────────────────────
        self.latest_odom       = None
        self.latest_odom_time  = None
        self.latest_rtab       = None
        self.last_odom_time    = None
        self.initialized       = False
        self.node_start_time   = None

        # ── Subscriptions ─────────────────────────────────────────────────
        self.create_subscription(Odometry, '/odom',
                                 self.odom_callback, 10)
        self.create_subscription(Odometry, '/rtabmap_pose',
                                 self.rtab_callback, 10)

        # ── Publisher ─────────────────────────────────────────────────────
        self.pub = self.create_publisher(Odometry, '/ekf_rtab_pose', 10)

        # ── Timer ─────────────────────────────────────────────────────────
        rate = self.get_parameter('ekf_rate_hz').value
        self.create_timer(1.0 / rate, self.run_ekf)

        self.get_logger().info(
            'EKF-RTAB node started.\n'
            'Waiting for RTAB-Map pose on /rtabmap_pose...'
        )

    # ── Callbacks ─────────────────────────────────────────────────────────

    def odom_callback(self, msg: Odometry):
        self.latest_odom = msg
        self.latest_odom_time = (
            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        )
        if self.node_start_time is None:
            self.node_start_time = self.get_clock().now().nanoseconds * 1e-9

    def rtab_callback(self, msg: Odometry):
        pose = msg.pose.pose

        # Initialize EKF state from first RTAB-Map pose
        if not self.initialized:
            self.x[0, 0] = pose.position.x
            self.x[1, 0] = pose.position.y
            self.x[2, 0] = quaternion_to_yaw(pose.orientation)
            self.initialized = True
            self.get_logger().info(
                f'EKF-RTAB initialized from RTAB-Map: '
                f'x={self.x[0,0]:.3f} y={self.x[1,0]:.3f} '
                f'theta={math.degrees(self.x[2,0]):.1f} deg'
            )
            return

        # Use covariance from RTAB-Map if available and valid
        cov6 = np.array(msg.pose.covariance).reshape(6, 6)
        cov_sum = abs(cov6[0,0]) + abs(cov6[1,1]) + abs(cov6[5,5])

        if cov_sum > 1e-9:
            # RTAB-Map published a real covariance — use it
            R = make_pd(np.array([
                [cov6[0,0], cov6[0,1], cov6[0,5]],
                [cov6[1,0], cov6[1,1], cov6[1,5]],
                [cov6[5,0], cov6[5,1], cov6[5,5]]
            ]))
        else:
            # No covariance available — use fixed noise
            R = self.R_rtab

        self.latest_rtab = {
            'x':   pose.position.x,
            'y':   pose.position.y,
            'yaw': quaternion_to_yaw(pose.orientation),
            'R':   R
        }

    # ── EKF loop ───────────────────────────────────────────────────────────

    def run_ekf(self):

        if not self.initialized:
            if self.node_start_time is not None:
                elapsed = (self.get_clock().now().nanoseconds * 1e-9
                           - self.node_start_time)
                if elapsed > 30.0:
                    self.get_logger().warn(
                        'EKF-RTAB not initialized after 30s. '
                        'Is rtabmap_pose_bridge running?'
                    )
                    self.node_start_time = self.get_clock().now().nanoseconds * 1e-9
            return

        if self.latest_odom is None or self.latest_odom_time is None:
            return

        now = self.latest_odom_time
        if self.last_odom_time is None:
            self.last_odom_time = now
            return

        dt = now - self.last_odom_time
        self.last_odom_time = now

        if dt <= 0.0 or dt > 1.0:
            return

        # ── Velocities ────────────────────────────────────────────────────
        v = self.latest_odom.twist.twist.linear.x
        w = self.latest_odom.twist.twist.angular.z
        if abs(v) < 0.01: v = 0.0
        if abs(w) < 0.01: w = 0.0

        # ── Predict ───────────────────────────────────────────────────────
        theta = self.x[2, 0]
        self.x = np.array([
            [self.x[0,0] + v * math.cos(theta) * dt],
            [self.x[1,0] + v * math.sin(theta) * dt],
            [wrap_angle(self.x[2,0] + w * dt)       ]
        ])
        F = np.array([
            [1.0, 0.0, -v * math.sin(theta) * dt],
            [0.0, 1.0,  v * math.cos(theta) * dt],
            [0.0, 0.0,  1.0                      ]
        ])
        self.P = F @ self.P @ F.T + self.Q

        # ── Update: RTAB-Map ──────────────────────────────────────────────
        if self.latest_rtab is not None:
            z = np.array([
                [self.latest_rtab['x']  ],
                [self.latest_rtab['y']  ],
                [self.latest_rtab['yaw']]
            ])
            H           = np.eye(3)
            R           = self.latest_rtab['R']
            innov       = z - self.x
            innov[2,0]  = wrap_angle(innov[2,0])
            K           = self.P @ H.T @ np.linalg.inv(H @ self.P @ H.T + R)
            self.x      = self.x + K @ innov
            self.x[2,0] = wrap_angle(self.x[2,0])
            self.P      = joseph_update(self.P, K, H, R)
            self.latest_rtab = None

        # ── Publish /ekf_rtab_pose ────────────────────────────────────────
        out = Odometry()
        out.header.stamp    = self.get_clock().now().to_msg()
        out.header.frame_id = 'map'
        out.child_frame_id  = 'base_footprint'

        out.pose.pose.position.x  = float(self.x[0,0])
        out.pose.pose.position.y  = float(self.x[1,0])
        out.pose.pose.position.z  = 0.0
        out.pose.pose.orientation = yaw_to_quaternion(float(self.x[2,0]))

        p   = self.P
        cov = [0.0] * 36
        cov[0]  = p[0,0]; cov[1]  = p[0,1]; cov[5]  = p[0,2]
        cov[6]  = p[1,0]; cov[7]  = p[1,1]; cov[11] = p[1,2]
        cov[30] = p[2,0]; cov[31] = p[2,1]; cov[35] = p[2,2]
        out.pose.covariance = cov

        out.twist.twist.linear.x  = v
        out.twist.twist.angular.z = w
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = EKFRtabNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()