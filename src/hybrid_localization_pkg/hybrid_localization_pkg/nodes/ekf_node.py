#!/usr/bin/env python3
"""
EKF Localization Node
=====================
State:    [x, y, theta]  — expressed in MAP frame
Predict:  /odom velocity  (twist only, never odom position)
Update:   /amcl_pose      (map frame directly, no TF needed)

Publishes:
  /ekf_pose          (nav_msgs/Odometry, frame_id = 'map')
  TF: map → odom     (so Nav2 uses EKF pose, not raw AMCL)

AMCL must have tf_broadcast: false in its params so it does
not conflict with this node's TF publishing.
"""

import rclpy
from rclpy.node import Node
import numpy as np
import math

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, PoseWithCovarianceStamped, TransformStamped
from tf2_ros import Buffer, TransformListener, TransformBroadcaster


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


class EKFNode(Node):

    def __init__(self):
        super().__init__('ekf_node')

        # ── Parameters ───────────────────────────────────────────────────
        self.declare_parameter('process_noise_x',     0.1)
        self.declare_parameter('process_noise_y',     0.1)
        self.declare_parameter('process_noise_theta', 0.05)
        self.declare_parameter('amcl_enabled',        True)
        self.declare_parameter('ekf_rate_hz',         50.0)
        self.declare_parameter('amcl_max_age_sec',    99999.0)
        self.set_parameters([
                                rclpy.parameter.Parameter(
                                    'use_sim_time',
                                    rclpy.Parameter.Type.BOOL,
                                    True
                                )
                            ])

        # ── State ─────────────────────────────────────────────────────────
        self.x = np.zeros((3, 1))
        self.P = np.eye(3) * 0.5

        self.Q = np.diag([
            self.get_parameter('process_noise_x').value,
            self.get_parameter('process_noise_y').value,
            self.get_parameter('process_noise_theta').value
        ])

        # ── Cache ─────────────────────────────────────────────────────────
        self.latest_odom      = None
        self.latest_odom_time = None
        self.latest_amcl      = None
        self.last_odom_time   = None
        self.ekf_initialized  = False
        self.node_start_time  = None

        # ── tf2 ───────────────────────────────────────────────────────────
        self.tf_buffer      = Buffer()
        self.tf_listener    = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)   # ← NEW

        # ── Subscriptions ─────────────────────────────────────────────────
        self.create_subscription(Odometry, '/odom',
                                 self.odom_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                 self.amcl_callback, 10)

        # ── Publisher ─────────────────────────────────────────────────────
        self.pub = self.create_publisher(Odometry, '/ekf_pose', 10)

        # ── Timer ─────────────────────────────────────────────────────────
        rate = self.get_parameter('ekf_rate_hz').value
        self.create_timer(1.0 / rate, self.run_ekf)

        self.get_logger().info(
            'EKF node started — waiting for AMCL initialisation...\n'
            'Reminder: set amcl tf_broadcast: false in Nav2 params\n'
            'so EKF TF does not conflict with AMCL TF.'
        )

    # ── Callbacks ─────────────────────────────────────────────────────────

    def odom_callback(self, msg: Odometry):
        self.latest_odom = msg
        self.latest_odom_time = (
            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        )
        if self.node_start_time is None:
            self.node_start_time = self.get_clock().now().nanoseconds * 1e-9

    def amcl_callback(self, msg: PoseWithCovarianceStamped):
        if not self.get_parameter('amcl_enabled').value:
            return

        # ── Reject stale AMCL messages ────────────────────────────────────
        msg_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        now = (self.latest_odom_time
               if self.latest_odom_time is not None
               else self.get_clock().now().nanoseconds * 1e-9)
        max_age = self.get_parameter('amcl_max_age_sec').value
        if msg_time > 0.0 and (now - msg_time) > max_age:
            self.get_logger().warn(
                f'Stale AMCL message rejected (age={(now - msg_time):.2f}s)'
            )
            return

        pose = msg.pose.pose

        if not self.ekf_initialized:
            self.x[0, 0] = pose.position.x
            self.x[1, 0] = pose.position.y
            self.x[2, 0] = quaternion_to_yaw(pose.orientation)
            self.ekf_initialized = True
            self.get_logger().info(
                f'EKF initialised from AMCL (map frame): '
                f'x={self.x[0,0]:.3f}  y={self.x[1,0]:.3f}  '
                f'theta={math.degrees(self.x[2,0]):.1f} deg'
            )
            return

        cov6 = np.array(msg.pose.covariance).reshape(6, 6)
        R_amcl = make_pd(np.array([
            [cov6[0, 0], cov6[0, 1], cov6[0, 5]],
            [cov6[1, 0], cov6[1, 1], cov6[1, 5]],
            [cov6[5, 0], cov6[5, 1], cov6[5, 5]]
        ]))
        self.latest_amcl = {
            'x':   pose.position.x,
            'y':   pose.position.y,
            'yaw': quaternion_to_yaw(pose.orientation),
            'R':   R_amcl
        }

    # ── Wait for AMCL ─────────────────────────────────────────────────────

    def _wait_for_amcl(self):
        if self.node_start_time is None:
            return
        elapsed = (self.get_clock().now().nanoseconds * 1e-9
                   - self.node_start_time)
        if elapsed > 30.0:
            self.get_logger().warn(
                'EKF not initialized after 30s — is AMCL running?\n'
                'Fix: publish /initialpose, e.g.:\n'
                '  ros2 topic pub --once /initialpose '
                'geometry_msgs/PoseWithCovarianceStamped '
                '"{header: {frame_id: map}, pose: {pose: {position: '
                '{x: -2.0, y: -0.5, z: 0.0}, orientation: {w: 1.0}}, '
                'covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, '
                '0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, '
                '0,0,0,0,0,0.07]}}"'
            )
            self.node_start_time = self.get_clock().now().nanoseconds * 1e-9

    # ── Compute and publish map→odom TF from EKF state ────────────────────

    def _publish_map_odom_tf(self):
        """
        Nav2 looks up TF map→base_footprint for all pose queries.
        This traverses: map → odom → base_footprint.
        We publish the map→odom piece derived from EKF state.

        map→odom answers: where is the odom origin in map frame?
        = EKF map pose minus the rotated odom pose offset.

        Since map/odom yaw confirmed = 0 in this setup,
        the rotation simplifies but the full formula is kept
        for correctness in case that ever changes.
        """
        if self.latest_odom is None:
            return

        # Robot pose in odom frame (raw odometry)
        odom_x   = self.latest_odom.pose.pose.position.x
        odom_y   = self.latest_odom.pose.pose.position.y
        odom_yaw = quaternion_to_yaw(self.latest_odom.pose.pose.orientation)

        # Robot pose in map frame (EKF estimate)
        map_x   = float(self.x[0, 0])
        map_y   = float(self.x[1, 0])
        map_yaw = float(self.x[2, 0])

        # map→odom transform
        # Derived by: T_map_odom = T_map_robot * inverse(T_odom_robot)
        delta_yaw = map_yaw - odom_yaw
        cos_d = math.cos(delta_yaw)
        sin_d = math.sin(delta_yaw)

        tf_x   = map_x - (cos_d * odom_x - sin_d * odom_y)
        tf_y   = map_y - (sin_d * odom_x + cos_d * odom_y)
        tf_yaw = delta_yaw

        t = TransformStamped()
        t.header.stamp    = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id  = 'odom'
        t.transform.translation.x = tf_x
        t.transform.translation.y = tf_y
        t.transform.translation.z = 0.0
        t.transform.rotation      = yaw_to_quaternion(tf_yaw)

        self.tf_broadcaster.sendTransform(t)

    # ── EKF loop ───────────────────────────────────────────────────────────

    def run_ekf(self):

        if not self.ekf_initialized:
            self._wait_for_amcl()
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
            [self.x[0, 0] + v * math.cos(theta) * dt],
            [self.x[1, 0] + v * math.sin(theta) * dt],
            [wrap_angle(self.x[2, 0] + w * dt)       ]
        ])
        F = np.array([
            [1.0, 0.0, -v * math.sin(theta) * dt],
            [0.0, 1.0,  v * math.cos(theta) * dt],
            [0.0, 0.0,  1.0                      ]
        ])
        self.P = F @ self.P @ F.T + self.Q

        # ── Update: AMCL ──────────────────────────────────────────────────
        if self.latest_amcl is not None:
            z = np.array([[self.latest_amcl['x']],
                          [self.latest_amcl['y']],
                          [self.latest_amcl['yaw']]])
            H = np.eye(3)
            R = self.latest_amcl['R']
            innov       = z - self.x
            innov[2, 0] = wrap_angle(innov[2, 0])
            K           = self.P @ H.T @ np.linalg.inv(H @ self.P @ H.T + R)
            self.x      = self.x + K @ innov
            self.x[2, 0] = wrap_angle(self.x[2, 0])
            self.P      = joseph_update(self.P, K, H, R)
            self.latest_amcl = None

        # ── Publish /ekf_pose ─────────────────────────────────────────────
        out = Odometry()
        out.header.stamp    = self.get_clock().now().to_msg()
        out.header.frame_id = 'map'
        out.child_frame_id  = 'base_footprint'

        out.pose.pose.position.x  = float(self.x[0, 0])
        out.pose.pose.position.y  = float(self.x[1, 0])
        out.pose.pose.position.z  = 0.0
        out.pose.pose.orientation = yaw_to_quaternion(float(self.x[2, 0]))

        p   = self.P
        cov = [0.0] * 36
        cov[0]  = p[0, 0]; cov[1]  = p[0, 1]; cov[5]  = p[0, 2]
        cov[6]  = p[1, 0]; cov[7]  = p[1, 1]; cov[11] = p[1, 2]
        cov[30] = p[2, 0]; cov[31] = p[2, 1]; cov[35] = p[2, 2]
        out.pose.covariance = cov

        out.twist.twist.linear.x  = v
        out.twist.twist.angular.z = w
        self.pub.publish(out)

        # ── Publish map→odom TF so Nav2 uses EKF pose ────────────────────
        self._publish_map_odom_tf()


def main(args=None):
    rclpy.init(args=args)
    node = EKFNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()