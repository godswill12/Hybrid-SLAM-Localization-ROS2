#!/usr/bin/env python3
"""
Ground Truth Publisher
======================
Reads exact robot pose from Gazebo using gz model command
and publishes it as geometry_msgs/Pose on /ground_truth.
Runs at 10 Hz.
"""
import subprocess
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from math import cos, sin


class GroundTruthPublisher(Node):

    def __init__(self):
        super().__init__('ground_truth_publisher')
        self.pub = self.create_publisher(Pose, '/ground_truth', 10)
        self.create_timer(0.1, self.publish_pose)  # 10 Hz
        self.get_logger().info('Ground truth publisher started')

    def publish_pose(self):
        try:
            result = subprocess.run(
                ['gz', 'model', '-m', 'waffle_pi', '-p'],
                capture_output=True,
                text=True,
                timeout=2
            )

            lines = result.stdout.strip().split('\n')
            xyz_line = None
            rpy_line = None

            for i, line in enumerate(lines):
                if 'XYZ' in line:
                    # Next line has the values
                    xyz_line = lines[i + 1].strip()
                    rpy_line = lines[i + 2].strip()
                    break

            if xyz_line is None:
                return

            # Parse [x y z] format
            xyz = xyz_line.strip('[]').split()
            rpy = rpy_line.strip('[]').split()

            x     = float(xyz[0])
            y     = float(xyz[1])
            z     = float(xyz[2])
            roll  = float(rpy[0])
            pitch = float(rpy[1])
            yaw   = float(rpy[2])

            # Convert yaw to quaternion
            cy = cos(yaw * 0.5)
            sy = sin(yaw * 0.5)
            cp = cos(pitch * 0.5)
            sp = sin(pitch * 0.5)
            cr = cos(roll * 0.5)
            sr = sin(roll * 0.5)

            qw = cr * cp * cy + sr * sp * sy
            qx = sr * cp * cy - cr * sp * sy
            qy = cr * sp * cy + sr * cp * sy
            qz = cr * cp * sy - sr * sp * cy

            msg = Pose()
            msg.position.x    = x
            msg.position.y    = y
            msg.position.z    = z
            msg.orientation.x = qx
            msg.orientation.y = qy
            msg.orientation.z = qz
            msg.orientation.w = qw

            self.pub.publish(msg)

        except Exception as e:
            self.get_logger().warn(f'Failed to get pose: {str(e)}')


def main():
    rclpy.init()
    node = GroundTruthPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()