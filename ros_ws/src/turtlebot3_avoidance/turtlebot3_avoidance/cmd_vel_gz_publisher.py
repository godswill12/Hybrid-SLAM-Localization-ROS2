#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class ContinuousMove(Node):
    def __init__(self):
        super().__init__('continuous_move')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
    def move_forward(self, duration=3.0):
        # Send continuous forward commands for specified duration
        msg = Twist()
        msg.linear.x = 0.2
        msg.angular.z = 0.0
        
        start_time = time.time()
        while time.time() - start_time < duration:
            self.publisher.publish(msg)
            self.get_logger().info('Moving forward...', throttle_duration_sec=1)
            time.sleep(0.1)  # Publish every 0.1 seconds
        
        # Stop
        msg.linear.x = 0.0
        self.publisher.publish(msg)
        self.get_logger().info('Stopped')
        
        # Exit
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = ContinuousMove()
    
    # Wait for connection
    time.sleep(1)
    
    # Move forward for 3 seconds then stop
    node.move_forward(3.0)

if __name__ == '__main__':
    main()