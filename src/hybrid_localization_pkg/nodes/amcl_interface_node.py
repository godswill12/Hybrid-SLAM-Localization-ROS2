#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped

class AMCLInterfaceNode(Node):
    def __init__(self):
        super().__init__('amcl_interface')
        self.sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.callback, 10)
        self.pub = self.create_publisher(PoseStamped, '/pose_amcl', 10)
        self.get_logger().info('AMCL interface node started')

    def callback(self, msg):
        pose = PoseStamped()
        pose.header          = msg.header
        pose.header.frame_id = 'map'
        pose.pose            = msg.pose.pose
        self.pub.publish(pose)

def main(args=None):
    rclpy.init(args=args)
    node = AMCLInterfaceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
