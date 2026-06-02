#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from math import atan2, sin, cos
from geometry_msgs.msg import PoseStamped
from tf_transformations import euler_from_quaternion, quaternion_from_euler

W_EKF  = 0.4
W_EIF  = 0.2
W_AMCL = 0.4

class HybridFusionNode(Node):
    def __init__(self):
        super().__init__('hybrid_fusion')
        self.create_subscription(PoseStamped, '/pose_ekf',  self.ekf_cb,  10)
        self.create_subscription(PoseStamped, '/pose_eif',  self.eif_cb,  10)
        self.create_subscription(PoseStamped, '/pose_amcl', self.amcl_cb, 10)
        self.pub = self.create_publisher(PoseStamped, '/pose_hybrid', 10)
        self.ekf_pose  = None
        self.eif_pose  = None
        self.amcl_pose = None
        self.get_logger().info('Hybrid fusion node started')

    def ekf_cb(self, msg):
        self.ekf_pose = msg
        self.fuse()

    def eif_cb(self, msg):
        self.eif_pose = msg

    def amcl_cb(self, msg):
        self.amcl_pose = msg

    def fuse(self):
        if self.ekf_pose is None or self.eif_pose is None or self.amcl_pose is None:
            return
        x = W_EKF*self.ekf_pose.pose.position.x + W_EIF*self.eif_pose.pose.position.x + W_AMCL*self.amcl_pose.pose.position.x
        y = W_EKF*self.ekf_pose.pose.position.y + W_EIF*self.eif_pose.pose.position.y + W_AMCL*self.amcl_pose.pose.position.y
        t_ekf  = self._yaw(self.ekf_pose)
        t_eif  = self._yaw(self.eif_pose)
        t_amcl = self._yaw(self.amcl_pose)
        theta  = atan2(W_EKF*sin(t_ekf)+W_EIF*sin(t_eif)+W_AMCL*sin(t_amcl),
                       W_EKF*cos(t_ekf)+W_EIF*cos(t_eif)+W_AMCL*cos(t_amcl))
        q   = quaternion_from_euler(0.0, 0.0, theta)
        msg = PoseStamped()
        msg.header.stamp     = self.get_clock().now().to_msg()
        msg.header.frame_id  = 'map'
        msg.pose.position.x  = x
        msg.pose.position.y  = y
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]
        self.pub.publish(msg)

    @staticmethod
    def _yaw(pose_msg):
        q = pose_msg.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        return yaw

def main(args=None):
    rclpy.init(args=args)
    node = HybridFusionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
