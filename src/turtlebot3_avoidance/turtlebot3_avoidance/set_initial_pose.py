#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from builtin_interfaces.msg import Time

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.timer = self.create_timer(1.0, self.publish_pose)
        self.counter = 0

    def publish_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = 1.0
        msg.pose.pose.position.y = 0.5
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = 0.707
        msg.pose.pose.orientation.w = 0.707
        
        # Set covariance (6x6 matrix)
        msg.pose.covariance = [0.0] * 36
        msg.pose.covariance[0] = 0.1   # x variance
        msg.pose.covariance[7] = 0.1   # y variance
        msg.pose.covariance[14] = 0.1  # z variance
        msg.pose.covariance[21] = 0.1  # roll variance
        msg.pose.covariance[28] = 0.1  # pitch variance
        msg.pose.covariance[35] = 0.1  # yaw variance
        
        self.publisher.publish(msg)
        self.get_logger().info('Published initial pose')
        self.counter += 1
        if self.counter >= 1:
            self.timer.cancel()
            self.destroy_node()
            rclpy.shutdown()

def main():
    rclpy.init()
    node = InitialPosePublisher()
    rclpy.spin(node)

if __name__ == '__main__':
    main()