# #!/usr/bin/env python3
# """
# Fusion Node
# ===========
# Combines EKF (short-term smoother) with RTAB-Map (global corrector).

# Logic:
#   - Normally: publish EKF pose as /fused_pose  (fast, smooth)
#   - On RTAB-Map loop closure: correct EKF state from RTAB-Map global pose,
#     then resume EKF smoothing from the corrected anchor.

# This gives us:
#   - EKF smoothness between loop closures
#   - RTAB-Map global consistency at loop closures

# Subscribes:
#   /ekf_pose            (nav_msgs/Odometry)       — from ekf_node
#   /rtabmap/odom        (nav_msgs/Odometry)       — RTAB-Map continuous estimate
#   /rtabmap/info        (rtabmap_msgs/Info)        — loop closure detection flag

# Publishes:
#   /fused_pose          (nav_msgs/Odometry)        — final fused estimate
# """

# import rclpy
# from rclpy.node import Node
# import numpy as np
# import math

# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import Quaternion
# from geometry_msgs.msg import PoseWithCovarianceStamped

# # RTAB-Map info message for loop closure detection
# try:
#     from rtabmap_msgs.msg import Info as RtabmapInfo
#     RTABMAP_MSGS_AVAILABLE = True
# except ImportError:
#     RTABMAP_MSGS_AVAILABLE = False


# def quaternion_to_yaw(q) -> float:
#     siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
#     cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
#     return math.atan2(siny_cosp, cosy_cosp)


# def yaw_to_quaternion(yaw: float) -> Quaternion:
#     q = Quaternion()
#     q.w = math.cos(yaw / 2.0)
#     q.z = math.sin(yaw / 2.0)
#     q.x = 0.0
#     q.y = 0.0
#     return q


# def wrap_angle(angle: float) -> float:
#     return math.atan2(math.sin(angle), math.cos(angle))


# class FusionNode(Node):

#     def __init__(self):
#         super().__init__('fusion_node')

#         self.set_parameters([
#             rclpy.parameter.Parameter(
#                 'use_sim_time',
#                 rclpy.Parameter.Type.BOOL,
#                 True
#             )
#         ])

#         # ── Parameters ────────────────────────────────────────────────────
#         self.declare_parameter('loop_closure_correction_weight', 0.7)
#         self.declare_parameter('max_correction_distance',        1.0)
#         self.declare_parameter('publish_rate_hz',               50.0)

#         self.correction_weight  = self.get_parameter(
#             'loop_closure_correction_weight').value
#         self.max_correction_dist = self.get_parameter(
#             'max_correction_distance').value

#         # ── State ─────────────────────────────────────────────────────────
#         self.ekf_pose   = None      # latest EKF pose (Odometry msg)
#         self.rtab_pose  = None      # latest RTAB-Map pose (Odometry msg)
#         self.loop_closure_detected = False
#         self.correction_pending    = False

#         # Fused state: x, y, yaw
#         self.fused_x   = 0.0
#         self.fused_y   = 0.0
#         self.fused_yaw = 0.0
#         self.initialized = False

#         # Statistics for thesis reporting
#         self.total_corrections   = 0
#         self.correction_magnitudes = []

#         # ── Subscriptions ─────────────────────────────────────────────────
#         self.create_subscription(
#             Odometry, '/ekf_pose',
#             self.ekf_callback, 10)

#         self.create_subscription(
#             Odometry, '/rtabmap_pose',
#             self.rtab_callback, 10)

#         # Loop closure detection from RTAB-Map info topic
#         if RTABMAP_MSGS_AVAILABLE:
#             self.create_subscription(
#                 RtabmapInfo, '/info',
#                 self.rtabmap_info_callback, 10)
#             self.get_logger().info('RTAB-Map Info subscription active.')
#         else:
#             # Fallback: detect loop closure by watching for sudden
#             # large jumps in RTAB-Map pose relative to EKF pose
#             self.get_logger().warn(
#                 'rtabmap_msgs not found. Using jump-detection fallback '
#                 'for loop closure. Install rtabmap_msgs for better results.'
#             )

#         # ── Publisher ─────────────────────────────────────────────────────
#         self.pub = self.create_publisher(Odometry, '/fused_pose', 10)

#         # ── Timer ─────────────────────────────────────────────────────────
#         rate = self.get_parameter('publish_rate_hz').value
#         self.create_timer(1.0 / rate, self.run_fusion)

#         self.get_logger().info('Fusion node started.')

#     # ── Callbacks ─────────────────────────────────────────────────────────

#     def ekf_callback(self, msg: Odometry):
#         self.ekf_pose = msg
#         if not self.initialized:
#             # Seed fused state from first EKF message
#             p = msg.pose.pose
#             self.fused_x   = p.position.x
#             self.fused_y   = p.position.y
#             self.fused_yaw = quaternion_to_yaw(p.orientation)
#             self.initialized = True
#             self.get_logger().info(
#                 f'Fusion initialized from EKF: '
#                 f'x={self.fused_x:.3f} y={self.fused_y:.3f}'
#             )

#     def rtab_callback(self, msg):
#         # Works for both PoseWithCovarianceStamped and Odometry
#         self.rtab_pose = msg.pose.pose

#         # Fallback loop closure detection via jump detection
#         if not RTABMAP_MSGS_AVAILABLE and self.ekf_pose is not None:
#             ekf_p  = self.ekf_pose.pose.pose
#             rtab_p = msg.pose.pose
#             dist = math.sqrt(
#                 (ekf_p.position.x - rtab_p.position.x)**2 +
#                 (ekf_p.position.y - rtab_p.position.y)**2
#             )
#             if dist > 0.3:
#                 self.loop_closure_detected = True
#                 self.correction_pending    = True

#     def rtabmap_info_callback(self, msg):
#         """
#         RtabmapInfo.loop_closure_id > 0 means a loop closure was detected
#         and the graph was re-optimized this cycle.
#         """
#         if msg.loop_closure_id > 0:
#             self.loop_closure_detected = True
#             self.correction_pending    = True
#             self.get_logger().info(
#                 f'Loop closure detected! ID={msg.loop_closure_id}. '
#                 f'Applying RTAB-Map correction to fused pose.'
#             )

#     # ── Main fusion loop ───────────────────────────────────────────────────

#     def run_fusion(self):

#         if not self.initialized or self.ekf_pose is None:
#             return

#         ekf_p   = self.ekf_pose.pose.pose
#         ekf_x   = ekf_p.position.x
#         ekf_y   = ekf_p.position.y
#         ekf_yaw = quaternion_to_yaw(ekf_p.orientation)

#         if self.correction_pending and self.rtab_pose is not None:
#             # ── Apply RTAB-Map global correction ──────────────────────────
#             rtab_p   = self.rtab_pose.pose.pose
#             rtab_x   = rtab_p.position.x
#             rtab_y   = rtab_p.position.y
#             rtab_yaw = quaternion_to_yaw(rtab_p.orientation)

#             # Sanity check: reject corrections that are unrealistically large
#             correction_dist = math.sqrt(
#                 (rtab_x - ekf_x)**2 + (rtab_y - ekf_y)**2
#             )

#             if correction_dist < self.max_correction_dist:
#                 # Weighted blend: EKF provides short-term accuracy,
#                 # RTAB-Map provides global consistency after loop closure
#                 w = self.correction_weight   # weight toward RTAB-Map
#                 self.fused_x   = (1 - w) * ekf_x   + w * rtab_x
#                 self.fused_y   = (1 - w) * ekf_y   + w * rtab_y

#                 # Angle blending requires special care (circular mean)
#                 delta_yaw = wrap_angle(rtab_yaw - ekf_yaw)
#                 self.fused_yaw = wrap_angle(ekf_yaw + w * delta_yaw)

#                 self.total_corrections += 1
#                 self.correction_magnitudes.append(correction_dist)

#                 self.get_logger().info(
#                     f'Correction #{self.total_corrections} applied: '
#                     f'magnitude={correction_dist:.4f}m  '
#                     f'fused=({self.fused_x:.3f}, {self.fused_y:.3f})'
#                 )
#             else:
#                 self.get_logger().warn(
#                     f'Correction rejected: magnitude={correction_dist:.4f}m '
#                     f'exceeds max={self.max_correction_dist}m'
#                 )

#             self.correction_pending    = False
#             self.loop_closure_detected = False

#         else:
#             # ── Normal operation: trust EKF ────────────────────────────────
#             self.fused_x   = ekf_x
#             self.fused_y   = ekf_y
#             self.fused_yaw = ekf_yaw

#         # ── Publish fused pose ─────────────────────────────────────────────
#         out = Odometry()
#         out.header.stamp    = self.get_clock().now().to_msg()
#         out.header.frame_id = 'map'
#         out.child_frame_id  = 'base_footprint'

#         out.pose.pose.position.x  = self.fused_x
#         out.pose.pose.position.y  = self.fused_y
#         out.pose.pose.position.z  = 0.0
#         out.pose.pose.orientation = yaw_to_quaternion(self.fused_yaw)

#         # Pass through EKF covariance as approximation
#         out.pose.covariance = self.ekf_pose.pose.covariance

#         self.pub.publish(out)

#     def print_summary(self):
#         self.get_logger().info(
#             f'\n=== Fusion Node Summary ===\n'
#             f'Total loop closure corrections: {self.total_corrections}\n'
#             f'Avg correction magnitude: '
#             f'{np.mean(self.correction_magnitudes):.4f}m '
#             f'if self.correction_magnitudes else "N/A"'
#         )


# def main(args=None):
#     rclpy.init(args=args)
#     node = FusionNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.print_summary()
#     finally:
#         node.destroy_node()
#         if rclpy.ok():
#             rclpy.shutdown()


# if __name__ == '__main__':
#     main()





#!/usr/bin/env python3
"""
Fusion Node
===========
Combines EKF (short-term smoother) with RTAB-Map (global corrector).

Logic:
  - Normally: publish EKF pose as /fused_pose  (fast, smooth)
  - On RTAB-Map loop closure: correct EKF state from RTAB-Map global pose,
    then resume EKF smoothing from the corrected anchor.

This gives us:
  - EKF smoothness between loop closures
  - RTAB-Map global consistency at loop closures

Fixes applied vs original:
  [1] self.rtab_pose now stores the full Odometry msg (not msg.pose.pose)
      → prevents AttributeError in run_fusion when accessing .pose.pose
  [2] /rtabmap/info topic name corrected (was '/info')
  [3] Staleness guard: corrections only fire if RTAB-Map data is < 2s old
      → prevents stale frozen poses from corrupting the fused trajectory
  [4] rtab_received_time tracked to support the staleness guard
  [5] print_summary f-string conditional fixed (was never evaluated)

Subscribes:
  /ekf_pose            (nav_msgs/Odometry)       — from ekf_node
  /rtabmap_pose        (nav_msgs/Odometry)        — from rtabmap_pose_bridge
  /rtabmap/info        (rtabmap_msgs/Info)        — loop closure detection flag

Publishes:
  /fused_pose          (nav_msgs/Odometry)        — final fused estimate
"""

import rclpy
from rclpy.node import Node
import numpy as np
import math

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

# RTAB-Map info message for loop closure detection
try:
    from rtabmap_msgs.msg import Info as RtabmapInfo
    RTABMAP_MSGS_AVAILABLE = True
except ImportError:
    RTABMAP_MSGS_AVAILABLE = False


# ── Helpers ────────────────────────────────────────────────────────────────────

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


# ── Node ───────────────────────────────────────────────────────────────────────

class FusionNode(Node):

    def __init__(self):
        super().__init__('fusion_node')

        self.set_parameters([
            rclpy.parameter.Parameter(
                'use_sim_time',
                rclpy.Parameter.Type.BOOL,
                True
            )
        ])

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter('loop_closure_correction_weight', 0.7)
        self.declare_parameter('max_correction_distance',        1.0)
        self.declare_parameter('publish_rate_hz',               50.0)
        self.declare_parameter('rtab_staleness_threshold_sec',   2.0)

        self.correction_weight    = self.get_parameter(
            'loop_closure_correction_weight').value
        self.max_correction_dist  = self.get_parameter(
            'max_correction_distance').value
        self.staleness_threshold  = self.get_parameter(
            'rtab_staleness_threshold_sec').value

        # ── State ─────────────────────────────────────────────────────────
        self.ekf_pose  = None       # full Odometry msg from EKF
        self.rtab_pose = None       # FIX 1: full Odometry msg from RTAB-Map
                                    #        (was stored as msg.pose.pose before)
        self.rtab_received_time    = None   # FIX 3: wall-clock time of last RTAB-Map msg
        self.loop_closure_detected = False
        self.correction_pending    = False

        # Fused state: x, y, yaw
        self.fused_x   = 0.0
        self.fused_y   = 0.0
        self.fused_yaw = 0.0
        self.initialized = False

        # Statistics for thesis reporting
        self.total_corrections     = 0
        self.correction_magnitudes = []

        # ── Subscriptions ─────────────────────────────────────────────────
        self.create_subscription(
            Odometry, '/ekf_pose',
            self.ekf_callback, 10)

        self.create_subscription(
            Odometry, '/rtabmap_pose',
            self.rtab_callback, 10)

        if RTABMAP_MSGS_AVAILABLE:
            # FIX 2: was '/info' — corrected to '/rtabmap/info'
            self.create_subscription(
                RtabmapInfo, '/rtabmap/info',
                self.rtabmap_info_callback, 10)
            self.get_logger().info(
                'RTAB-Map Info subscription active on /rtabmap/info')
        else:
            self.get_logger().warn(
                'rtabmap_msgs not found. Using jump-detection fallback '
                'for loop closure. Install rtabmap_msgs for better results.'
            )

        # ── Publisher ─────────────────────────────────────────────────────
        self.pub = self.create_publisher(Odometry, '/fused_pose', 10)

        # ── Timer ─────────────────────────────────────────────────────────
        rate = self.get_parameter('publish_rate_hz').value
        self.create_timer(1.0 / rate, self.run_fusion)

        self.get_logger().info('Fusion node started.')

    # ── Callbacks ─────────────────────────────────────────────────────────

    def ekf_callback(self, msg: Odometry):
        self.ekf_pose = msg
        if not self.initialized:
            p = msg.pose.pose
            self.fused_x   = p.position.x
            self.fused_y   = p.position.y
            self.fused_yaw = quaternion_to_yaw(p.orientation)
            self.initialized = True
            self.get_logger().info(
                f'Fusion initialized from EKF: '
                f'x={self.fused_x:.3f} y={self.fused_y:.3f}'
            )

    def rtab_callback(self, msg: Odometry):
        # FIX 1: store the full Odometry message, not msg.pose.pose
        # Previously: self.rtab_pose = msg.pose.pose
        # That caused AttributeError in run_fusion when doing self.rtab_pose.pose.pose
        self.rtab_pose = msg

        # FIX 3: record the time we received this message
        self.rtab_received_time = self.get_clock().now().nanoseconds * 1e-9

        # Fallback loop closure detection via jump detection
        # (only used when rtabmap_msgs is not installed)
        if not RTABMAP_MSGS_AVAILABLE and self.ekf_pose is not None:
            ekf_p  = self.ekf_pose.pose.pose
            rtab_p = msg.pose.pose
            dist = math.sqrt(
                (ekf_p.position.x - rtab_p.position.x)**2 +
                (ekf_p.position.y - rtab_p.position.y)**2
            )
            if dist > 0.3:
                self.loop_closure_detected = True
                self.correction_pending    = True
                self.get_logger().info(
                    f'Jump-detection loop closure: dist={dist:.3f}m')

    def rtabmap_info_callback(self, msg):
        """
        RtabmapInfo.loop_closure_id > 0 means a loop closure was detected
        and the graph was re-optimised this cycle.
        """
        if msg.loop_closure_id > 0:
            self.loop_closure_detected = True
            self.correction_pending    = True
            self.get_logger().info(
                f'Loop closure detected! ID={msg.loop_closure_id}. '
                f'Correction will be applied on next fusion tick.'
            )

    # ── Main fusion loop ───────────────────────────────────────────────────

    def run_fusion(self):

        if not self.initialized or self.ekf_pose is None:
            return

        ekf_p   = self.ekf_pose.pose.pose
        ekf_x   = ekf_p.position.x
        ekf_y   = ekf_p.position.y
        ekf_yaw = quaternion_to_yaw(ekf_p.orientation)

        # FIX 3: staleness guard — only apply correction if RTAB-Map data
        # arrived recently. RTAB-Map publishes /localization_pose only on
        # place recognition (~0.04 Hz in poor conditions). Without this guard,
        # a 25-second-old frozen pose would corrupt the fused trajectory.
        now = self.get_clock().now().nanoseconds * 1e-9
        rtab_is_fresh = (
            self.rtab_pose is not None and
            self.rtab_received_time is not None and
            (now - self.rtab_received_time) < self.staleness_threshold
        )

        if self.correction_pending and rtab_is_fresh:
            # ── Apply RTAB-Map global correction ──────────────────────────
            # FIX 1: self.rtab_pose is now a full Odometry — access correctly
            rtab_p   = self.rtab_pose.pose.pose
            rtab_x   = rtab_p.position.x
            rtab_y   = rtab_p.position.y
            rtab_yaw = quaternion_to_yaw(rtab_p.orientation)

            # Sanity check: reject corrections that are unrealistically large
            correction_dist = math.sqrt(
                (rtab_x - ekf_x)**2 + (rtab_y - ekf_y)**2
            )

            if correction_dist < self.max_correction_dist:
                # Weighted blend: EKF for short-term accuracy,
                # RTAB-Map for global consistency after loop closure
                w = self.correction_weight
                self.fused_x   = (1 - w) * ekf_x + w * rtab_x
                self.fused_y   = (1 - w) * ekf_y + w * rtab_y

                # Angle blending requires circular mean (not linear interp)
                delta_yaw      = wrap_angle(rtab_yaw - ekf_yaw)
                self.fused_yaw = wrap_angle(ekf_yaw + w * delta_yaw)

                self.total_corrections += 1
                self.correction_magnitudes.append(correction_dist)

                self.get_logger().info(
                    f'Correction #{self.total_corrections} applied: '
                    f'magnitude={correction_dist:.4f}m  '
                    f'fused=({self.fused_x:.3f}, {self.fused_y:.3f})'
                )
            else:
                self.get_logger().warn(
                    f'Correction rejected: magnitude={correction_dist:.4f}m '
                    f'exceeds max={self.max_correction_dist}m'
                )

            self.correction_pending    = False
            self.loop_closure_detected = False

        elif self.correction_pending and not rtab_is_fresh:
            # Correction was requested but RTAB-Map data is stale — skip it
            self.get_logger().warn(
                f'Correction skipped: RTAB-Map data is stale '
                f'(last received {now - (self.rtab_received_time or 0):.1f}s ago, '
                f'threshold={self.staleness_threshold}s)'
            )
            self.correction_pending    = False
            self.loop_closure_detected = False

        else:
            # ── Normal operation: follow EKF ──────────────────────────────
            self.fused_x   = ekf_x
            self.fused_y   = ekf_y
            self.fused_yaw = ekf_yaw

        # ── Publish fused pose ─────────────────────────────────────────────
        out = Odometry()
        out.header.stamp    = self.get_clock().now().to_msg()
        out.header.frame_id = 'map'
        out.child_frame_id  = 'base_footprint'

        out.pose.pose.position.x  = self.fused_x
        out.pose.pose.position.y  = self.fused_y
        out.pose.pose.position.z  = 0.0
        out.pose.pose.orientation = yaw_to_quaternion(self.fused_yaw)

        # Pass through EKF covariance as approximation
        out.pose.covariance = self.ekf_pose.pose.covariance

        self.pub.publish(out)

    # ── Summary ───────────────────────────────────────────────────────────

    def print_summary(self):
        # FIX 5: f-string conditional was broken before (never evaluated)
        if self.correction_magnitudes:
            avg_str = f'{np.mean(self.correction_magnitudes):.4f}m'
        else:
            avg_str = 'N/A (no corrections applied)'

        self.get_logger().info(
            f'\n=== Fusion Node Summary ===\n'
            f'Total loop closure corrections : {self.total_corrections}\n'
            f'Avg correction magnitude       : {avg_str}\n'
            f'RTAB-Map msgs available        : {RTABMAP_MSGS_AVAILABLE}\n'
            f'Staleness threshold            : {self.staleness_threshold}s'
        )


# ── Entry point ────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.print_summary()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()