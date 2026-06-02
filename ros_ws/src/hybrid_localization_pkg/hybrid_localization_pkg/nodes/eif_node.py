#!/usr/bin/env python3
"""
EIF Localization Node
=====================
State:    [x, y, theta]  — expressed in MAP frame
Predict:  /odom velocity  (twist only, never odom position)
Update:   /amcl_pose      (map frame directly, no TF needed)

Information filter dual of EKF:
  Tracks information vector xi = Omega @ x
  and information matrix Omega = P_inv
  instead of mean x and covariance P.

Publishes:
  /eif_pose          (nav_msgs/Odometry, frame_id = 'map')
  TF: map → odom     (so Nav2 uses EIF pose, not raw AMCL)

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


# ─────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────

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

def make_pd(M, min_eig=1e-6):
    """Symmetrise and floor eigenvalues to guarantee positive-definiteness."""
    M = (M + M.T) / 2.0
    eigvals, eigvecs = np.linalg.eigh(M)
    eigvals = np.maximum(eigvals, min_eig)
    return eigvecs @ np.diag(eigvals) @ eigvecs.T


# ─────────────────────────────────────────────
# EIF Node
# ─────────────────────────────────────────────

class EIFNode(Node):

    def __init__(self):
        super().__init__('eif_node')

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

        # ── Process noise ─────────────────────────────────────────────────
        self.Q = np.diag([
            self.get_parameter('process_noise_x').value,
            self.get_parameter('process_noise_y').value,
            self.get_parameter('process_noise_theta').value
        ])

        # ── EIF state: information matrix Omega and information vector xi ──
        # Omega = P^{-1},  xi = P^{-1} @ x
        # Start with low information (high uncertainty) = small Omega
        self.Omega = np.eye(3) * 2.0      # = P^{-1} with P = 0.5 * I
        self.xi    = np.zeros((3, 1))     # = Omega @ x, x starts at 0

        # ── Cache ─────────────────────────────────────────────────────────
        self.latest_odom      = None
        self.latest_odom_time = None
        self.latest_amcl      = None
        self.last_odom_time   = None
        self.eif_initialized  = False
        self.node_start_time  = None

        # ── tf2 ───────────────────────────────────────────────────────────
        self.tf_buffer      = Buffer()
        self.tf_listener    = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # ── Subscriptions ─────────────────────────────────────────────────
        self.create_subscription(Odometry, '/odom',
                                 self.odom_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                 self.amcl_callback, 10)

        # ── Publisher ─────────────────────────────────────────────────────
        self.pub = self.create_publisher(Odometry, '/eif_pose', 10)

        # ── Timer ─────────────────────────────────────────────────────────
        rate = self.get_parameter('ekf_rate_hz').value
        self.create_timer(1.0 / rate, self.run_eif)

        self.get_logger().info(
            'EIF node started — waiting for AMCL initialisation...\n'
            'Reminder: set amcl tf_broadcast: false in Nav2 params.'
        )

    # ── Helpers to recover mean and covariance from information form ───────

    def _mean(self) -> np.ndarray:
        """Recover mean x = Omega^{-1} @ xi."""
        return np.linalg.solve(self.Omega, self.xi)

    def _covariance(self) -> np.ndarray:
        """Recover covariance P = Omega^{-1}."""
        return np.linalg.inv(self.Omega)

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

        # Reject stale AMCL messages
        msg_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        now = (self.latest_odom_time
               if self.latest_odom_time is not None
               else self.get_clock().now().nanoseconds * 1e-9)
        max_age = self.get_parameter('amcl_max_age_sec').value
        if msg_time > 0.0 and (now - msg_time) > max_age:
            self.get_logger().warn(
                f'Stale AMCL rejected (age={(now - msg_time):.2f}s)'
            )
            return

        pose = msg.pose.pose

        # First AMCL message seeds the EIF state
        if not self.eif_initialized:
            x_init = np.array([
                [pose.position.x],
                [pose.position.y],
                [quaternion_to_yaw(pose.orientation)]
            ])
            # Start with moderate information
            P_init     = np.eye(3) * 0.5
            self.Omega = np.linalg.inv(P_init)
            self.xi    = self.Omega @ x_init
            self.eif_initialized = True
            self.get_logger().info(
                f'EIF initialised from AMCL (map frame): '
                f'x={float(x_init[0]):.3f}  y={float(x_init[1]):.3f}  '
                f'theta={math.degrees(float(x_init[2])):.1f} deg'
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
                'EIF not initialized after 30s — is AMCL running?\n'
                'Publish /initialpose to trigger AMCL.'
            )
            self.node_start_time = self.get_clock().now().nanoseconds * 1e-9

    # ── Publish map→odom TF ───────────────────────────────────────────────

    def _publish_map_odom_tf(self, x_mean: np.ndarray):
        if self.latest_odom is None:
            return

        odom_x   = self.latest_odom.pose.pose.position.x
        odom_y   = self.latest_odom.pose.pose.position.y
        odom_yaw = quaternion_to_yaw(self.latest_odom.pose.pose.orientation)

        map_x   = float(x_mean[0, 0])
        map_y   = float(x_mean[1, 0])
        map_yaw = float(x_mean[2, 0])

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

    # ── EIF main loop ─────────────────────────────────────────────────────

    def run_eif(self):

        if not self.eif_initialized:
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

        # ═════════════════════════════════════════════════════════════════
        #  STEP 1 — PREDICT (information form)
        #
        #  EIF predict requires converting back to moment form temporarily
        #  because the nonlinear motion model cannot be applied directly
        #  to the information vector. This is the key tradeoff of the EIF:
        #  prediction is more expensive, but update is cheaper.
        #
        #  Algorithm:
        #    1. Recover mean:       x     = Omega^{-1} @ xi
        #    2. Recover covariance: P     = Omega^{-1}
        #    3. Apply motion model: x_pred = f(x, u)
        #    4. Propagate Jacobian: P_pred = F @ P @ F.T + Q
        #    5. Convert back:       Omega  = P_pred^{-1}
        #                           xi     = Omega @ x_pred
        # ═════════════════════════════════════════════════════════════════

        # Step 1-2: recover moment form
        P     = self._covariance()
        x_vec = self._mean()

        theta = float(x_vec[2, 0])

        # Step 3: nonlinear motion model
        x_pred = np.array([
            [float(x_vec[0, 0]) + v * math.cos(theta) * dt],
            [float(x_vec[1, 0]) + v * math.sin(theta) * dt],
            [wrap_angle(float(x_vec[2, 0]) + w * dt)       ]
        ])

        # Step 4: Jacobian and covariance prediction
        F = np.array([
            [1.0, 0.0, -v * math.sin(theta) * dt],
            [0.0, 1.0,  v * math.cos(theta) * dt],
            [0.0, 0.0,  1.0                      ]
        ])
        P_pred = F @ P @ F.T + self.Q

        # Step 5: convert predicted moment form back to information form
        P_pred     = make_pd(P_pred)          # ensure invertibility
        self.Omega = np.linalg.inv(P_pred)
        self.xi    = self.Omega @ x_pred

        # ═════════════════════════════════════════════════════════════════
        #  STEP 2 — UPDATE with AMCL (information form)
        #
        #  This is where EIF shines — update is purely additive:
        #
        #    Omega_new = Omega_pred + H.T @ R^{-1} @ H
        #    xi_new    = xi_pred   + H.T @ R^{-1} @ z
        #
        #  No matrix inversion of a large matrix needed.
        #  H = I (AMCL observes all 3 states directly), so:
        #
        #    Omega_new = Omega_pred + R^{-1}
        #    xi_new    = xi_pred   + R^{-1} @ z
        # ═════════════════════════════════════════════════════════════════

        if self.latest_amcl is not None:

            z = np.array([
                [self.latest_amcl['x']  ],
                [self.latest_amcl['y']  ],
                [self.latest_amcl['yaw']]
            ])

            R     = self.latest_amcl['R']
            R_inv = np.linalg.inv(R)      # measurement information matrix
            H     = np.eye(3)             # AMCL observes x, y, yaw directly

            # ✅ EIF update — purely additive, no Kalman gain needed
            self.Omega = self.Omega + H.T @ R_inv @ H
            self.xi    = self.xi    + H.T @ R_inv @ z

            # Wrap yaw in information vector by recovering mean,
            # wrapping theta, and re-encoding
            x_upd        = self._mean()
            x_upd[2, 0]  = wrap_angle(float(x_upd[2, 0]))
            self.xi      = self.Omega @ x_upd

            self.latest_amcl = None

        # ═════════════════════════════════════════════════════════════════
        #  STEP 3 — RECOVER MEAN FOR PUBLISHING
        #
        #  Convert back to moment form just for output.
        #  x = Omega^{-1} @ xi
        #  P = Omega^{-1}
        # ═════════════════════════════════════════════════════════════════

        x_out = self._mean()
        P_out = self._covariance()

        # ── Publish /eif_pose ─────────────────────────────────────────────
        out = Odometry()
        out.header.stamp    = self.get_clock().now().to_msg()
        out.header.frame_id = 'map'
        out.child_frame_id  = 'base_footprint'

        out.pose.pose.position.x  = float(x_out[0, 0])
        out.pose.pose.position.y  = float(x_out[1, 0])
        out.pose.pose.position.z  = 0.0
        out.pose.pose.orientation = yaw_to_quaternion(float(x_out[2, 0]))

        p   = P_out
        cov = [0.0] * 36
        cov[0]  = p[0, 0]; cov[1]  = p[0, 1]; cov[5]  = p[0, 2]
        cov[6]  = p[1, 0]; cov[7]  = p[1, 1]; cov[11] = p[1, 2]
        cov[30] = p[2, 0]; cov[31] = p[2, 1]; cov[35] = p[2, 2]
        out.pose.covariance = cov

        out.twist.twist.linear.x  = v
        out.twist.twist.angular.z = w
        self.pub.publish(out)

        # ── Publish map→odom TF ───────────────────────────────────────────
        self._publish_map_odom_tf(x_out)


def main(args=None):
    rclpy.init(args=args)
    node = EIFNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()