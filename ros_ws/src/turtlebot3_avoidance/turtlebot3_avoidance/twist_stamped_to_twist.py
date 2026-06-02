#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class TwistStampedToTwist(Node):
    def __init__(self):
        super().__init__('twist_stamped_to_twist')
        
        # Subscribe to TwistStamped
        self.subscription = self.create_subscription(
            TwistStamped,
            '/cmd_vel_stamped',  # New topic for your navigator
            self.twist_stamped_callback,
            10)
        
        # Publish as Twist
        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel',  # Original topic for the bridge
            10)
        
        self.get_logger().info('TwistStamped to Twist converter started')

    def twist_stamped_callback(self, msg):
        # Extract Twist from TwistStamped and republish
        self.publisher.publish(msg.twist)

def main():
    rclpy.init()
    node = TwistStampedToTwist()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()