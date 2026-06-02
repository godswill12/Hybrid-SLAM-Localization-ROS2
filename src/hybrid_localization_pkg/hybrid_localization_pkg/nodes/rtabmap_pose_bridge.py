#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_ros import Buffer, TransformListener
import tf2_ros


class RtabmapPoseBridge(Node):

    def __init__(self):
        super().__init__('rtabmap_pose_bridge')
        self.set_parameters([
            rclpy.parameter.Parameter('use_sim_time',
                                      rclpy.Parameter.Type.BOOL, True)
        ])

        self.pub = self.create_publisher(Odometry, '/rtabmap_pose', 10)

        self.create_subscription(
            PoseWithCovarianceStamped,
            '/localization_pose',
            self.localization_pose_callback, 10)

        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.last_published = None

        self.create_timer(0.1, self.tf_fallback)
        self.get_logger().info(
            'RTAB-Map pose bridge started.\n'
            'Primary:  /localization_pose\n'
            'Fallback: TF map->base_footprint\n'
            'Output:   /rtabmap_pose'
        )

    def localization_pose_callback(self, msg):
        out = Odometry()
        out.header          = msg.header
        out.header.frame_id = 'map'
        out.child_frame_id  = 'base_footprint'
        out.pose            = msg.pose
        self.pub.publish(out)
        self.last_published = self.get_clock().now().nanoseconds * 1e-9
        self.last_pose      = out   # cache it
        self.get_logger().info('Publishing from /localization_pose', once=True)

    def tf_fallback(self):
        # now = self.get_clock().now().nanoseconds * 1e-9
        # if self.last_published is not None and (now - self.last_published) < 1.0:
        #     return
        # try:
        #     t = self.tf_buffer.lookup_transform(
        #         'map', 'base_footprint',
        #         rclpy.time.Time(),
        #         timeout=rclpy.duration.Duration(seconds=0.05)
        #     )
        #     out = Odometry()
        #     out.header.stamp    = self.get_clock().now().to_msg()
        #     out.header.frame_id = 'map'
        #     out.child_frame_id  = 'base_footprint'
        #     out.pose.pose.position.x  = t.transform.translation.x
        #     out.pose.pose.position.y  = t.transform.translation.y
        #     out.pose.pose.position.z  = 0.0
        #     out.pose.pose.orientation = t.transform.rotation
        #     self.pub.publish(out)
        # except Exception:
        #     pass
        return


def main(args=None):
    rclpy.init(args=args)
    node = RtabmapPoseBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
