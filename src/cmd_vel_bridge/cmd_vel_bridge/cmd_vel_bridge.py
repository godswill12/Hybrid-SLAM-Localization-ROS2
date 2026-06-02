# import rclpy
# from rclpy.node import Node

# from geometry_msgs.msg import Twist
# from geometry_msgs.msg import TwistStamped


# class CmdVelBridge(Node):

#     def __init__(self):
#         super().__init__('cmd_vel_bridge')

#         # Subscribe to Nav2 velocity commands
#         self.subscription = self.create_subscription(
#             Twist,
#             '/cmd_vel',
#             self.cmd_callback,
#             10)

#         # Publish stamped velocity
#         self.publisher = self.create_publisher(
#             TwistStamped,
#             '/base_velocity',
#             10)

#         self.get_logger().info("CmdVel bridge started")

#     def cmd_callback(self, msg):

#         twist_stamped = TwistStamped()

#         # timestamp
#         twist_stamped.header.stamp = self.get_clock().now().to_msg()

#         # frame
#         twist_stamped.header.frame_id = "base_link"

#         # copy velocity
#         twist_stamped.twist = msg

#         self.publisher.publish(twist_stamped)


# def main(args=None):

#     rclpy.init(args=args)

#     node = CmdVelBridge()

#     rclpy.spin(node)

#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()



#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class TwistToStampedBridge(Node):
    def __init__(self):
        super().__init__('twist_to_stamped_bridge')
        # Subscribe to Nav2's /cmd_vel (Twist)
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.callback, 10)
        # Publish TwistStamped to /cmd_vel (robot driver listens here)
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.get_logger().info("CmdVel bridge converter started")

    def callback(self, msg: Twist):
        ts = TwistStamped()
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.header.frame_id = 'base_link'  # or whatever your robot uses
        ts.twist = msg
        self.pub.publish(ts)

def main(args=None):
    rclpy.init(args=args)
    node = TwistToStampedBridge()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()