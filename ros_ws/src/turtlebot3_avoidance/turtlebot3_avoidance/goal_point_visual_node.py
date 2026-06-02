#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker

class GoalMarker(Node):
    def __init__(self):
        super().__init__('goal_marker')

        self.gx = 2.0
        self.gy = 2.0

        self.pub = self.create_publisher(Marker, '/goal_marker', 1)
        self.timer = self.create_timer(1.0, self.publish_marker)

    def publish_marker(self):
        m = Marker()
        m.header.frame_id = "odom"
        m.header.stamp = self.get_clock().now().to_msg()

        m.ns = "goal"
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD

        m.pose.position.x = self.gx
        m.pose.position.y = self.gy
        m.pose.position.z = 0.1

        m.scale.x = 0.3
        m.scale.y = 0.3
        m.scale.z = 0.3

        m.color.r = 1.0
        m.color.g = 0.0
        m.color.b = 0.0
        m.color.a = 1.0

        self.pub.publish(m)

def main():
    rclpy.init()
    node = GoalMarker()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
