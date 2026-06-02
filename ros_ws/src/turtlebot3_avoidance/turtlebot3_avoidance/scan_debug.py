#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class ScanDebug(Node):
    def __init__(self):
        super().__init__('scan_debug')
        self.sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.get_logger().info('Scan debug node started')
        
    def scan_callback(self, msg):
        self.get_logger().info(f'=== Scan Data Received ===')
        self.get_logger().info(f'Number of ranges: {len(msg.ranges)}')
        self.get_logger().info(f'Angle min: {msg.angle_min:.2f} rad')
        self.get_logger().info(f'Angle max: {msg.angle_max:.2f} rad')
        self.get_logger().info(f'Angle increment: {msg.angle_increment:.4f} rad')
        self.get_logger().info(f'Range min: {msg.range_min:.2f} m')
        self.get_logger().info(f'Range max: {msg.range_max:.2f} m')
        
        # Check first few values
        if len(msg.ranges) > 0:
            self.get_logger().info(f'First 5 ranges: {msg.ranges[:5]}')
            
            # Count valid vs invalid readings
            valid = sum(1 for r in msg.ranges[:20] if 0.1 < r < float('inf'))
            self.get_logger().info(f'Valid readings in first 20: {valid}')

def main(args=None):
    rclpy.init(args=args)
    node = ScanDebug()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()