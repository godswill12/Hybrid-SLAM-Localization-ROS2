#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math

class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')
        
        # Subscribe to LiDAR
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        # Publisher for velocity commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Parameters
        self.safe_distance = 0.5  # meters
        self.max_speed = 0.2      # m/s
        self.max_turn = 0.5       # rad/s
        
        # State
        self.obstacle_detected = False
        
        self.get_logger().info('Obstacle avoidance node started')
        
    def scan_callback(self, msg):
        if not msg.ranges:
            return
            
        # Analyze front sector (from -30 to +30 degrees)
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment
        
        # Convert -30 to +30 degrees to indices
        front_start_angle = -30.0 * math.pi / 180.0  # -30° in radians
        front_end_angle = 30.0 * math.pi / 180.0     # +30° in radians
        
        start_idx = max(0, int((front_start_angle - angle_min) / angle_inc))
        end_idx = min(len(msg.ranges) - 1, int((front_end_angle - angle_min) / angle_inc))
        
        # Get ranges in front sector
        front_ranges = msg.ranges[start_idx:end_idx + 1]
        
        # Filter out invalid values (0.0, inf, nan)
        valid_ranges = []
        for r in front_ranges:
            if r > 0.1 and r < float('inf') and not math.isnan(r):
                valid_ranges.append(r)
        
        if not valid_ranges:
            # No valid data, proceed with caution
            self.obstacle_detected = False
            self.move_forward()
            return
            
        # Find minimum distance in front
        min_distance = min(valid_ranges)
        
        # Create twist message
        twist = Twist()
        
        if min_distance < self.safe_distance:
            # Obstacle detected - need to avoid
            self.obstacle_detected = True
            
            # Analyze left and right sectors to decide which way to turn
            left_start_angle = -90.0 * math.pi / 180.0  # -90° to -30°
            left_end_angle = -30.0 * math.pi / 180.0
            right_start_angle = 30.0 * math.pi / 180.0   # 30° to 90°
            right_end_angle = 90.0 * math.pi / 180.0
            
            # Get left sector distances
            left_start_idx = max(0, int((left_start_angle - angle_min) / angle_inc))
            left_end_idx = min(len(msg.ranges) - 1, int((left_end_angle - angle_min) / angle_inc))
            left_ranges = msg.ranges[left_start_idx:left_end_idx + 1]
            left_valid = [r for r in left_ranges if r > 0.1 and r < float('inf') and not math.isnan(r)]
            
            # Get right sector distances
            right_start_idx = max(0, int((right_start_angle - angle_min) / angle_inc))
            right_end_idx = min(len(msg.ranges) - 1, int((right_end_angle - angle_min) / angle_inc))
            right_ranges = msg.ranges[right_start_idx:right_end_idx + 1]
            right_valid = [r for r in right_ranges if r > 0.1 and r < float('inf') and not math.isnan(r)]
            
            # Decide which way to turn based on which side has more space
            if left_valid and right_valid:
                avg_left = sum(left_valid) / len(left_valid)
                avg_right = sum(right_valid) / len(right_valid)
                
                if avg_left > avg_right:
                    # More space on left, turn left
                    twist.linear.x = 0.0
                    twist.angular.z = self.max_turn
                    self.get_logger().info(f'Obstacle at {min_distance:.2f}m - Turning LEFT (left: {avg_left:.2f}m, right: {avg_right:.2f}m)')
                else:
                    # More space on right, turn right
                    twist.linear.x = 0.0
                    twist.angular.z = -self.max_turn
                    self.get_logger().info(f'Obstacle at {min_distance:.2f}m - Turning RIGHT (left: {avg_left:.2f}m, right: {avg_right:.2f}m)')
            elif left_valid:
                # Only left data available, turn left
                twist.linear.x = 0.0
                twist.angular.z = self.max_turn
                self.get_logger().info(f'Obstacle at {min_distance:.2f}m - Turning LEFT (no right data)')
            elif right_valid:
                # Only right data available, turn right
                twist.linear.x = 0.0
                twist.angular.z = -self.max_turn
                self.get_logger().info(f'Obstacle at {min_distance:.2f}m - Turning RIGHT (no left data)')
            else:
                # No side data, just turn in place
                twist.linear.x = 0.0
                twist.angular.z = self.max_turn
                self.get_logger().info(f'Obstacle at {min_distance:.2f}m - Turning in place')
        else:
            # No obstacle in safe distance, move forward
            self.obstacle_detected = False
            
            # Adjust speed based on distance (slow down as we get closer)
            speed_factor = min(1.0, (min_distance - self.safe_distance) / self.safe_distance)
            twist.linear.x = self.max_speed * speed_factor
            twist.angular.z = 0.0
            
            if speed_factor < 0.5:
                self.get_logger().info(f'Approaching obstacle at {min_distance:.2f}m - Slowing down')
            else:
                self.get_logger().info(f'Clear path - Moving forward. Closest: {min_distance:.2f}m', throttle_duration_sec=2)
        
        # Publish the command
        self.cmd_pub.publish(twist)
    
    def move_forward(self):
        """Simple forward movement when no scan data"""
        twist = Twist()
        twist.linear.x = self.max_speed * 0.5  # Half speed for caution
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the robot before exiting
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        node.cmd_pub.publish(twist)
        node.get_logger().info('Stopping robot')
        
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()