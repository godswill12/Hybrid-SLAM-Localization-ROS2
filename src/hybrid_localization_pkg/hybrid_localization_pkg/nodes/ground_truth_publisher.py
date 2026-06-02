# #!/usr/bin/env python3
# """
# Ground Truth Publisher
# ======================
# Reads exact robot pose from Gazebo using gz model command
# and publishes it as geometry_msgs/Pose on /ground_truth.

# Coordinate frame conversion:
#   gz model returns WORLD frame coordinates
#   Robot spawns at world [-2.0, -0.5] = map frame [0.0, 0.0]
#   Therefore: map_x = world_x - SPAWN_WORLD_X
#              map_y = world_y - SPAWN_WORLD_Y

# Runs at 10 Hz.
# """
# import subprocess
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Pose
# from math import cos, sin


# # ── Spawn position in Gazebo world frame ──────────────────────────────
# # These match x_pose and y_pose in my_world.launch.py
# # Converting: map_frame = world_frame - spawn_offset
# SPAWN_WORLD_X = -2.0
# SPAWN_WORLD_Y = -0.5


# class GroundTruthPublisher(Node):

#     def __init__(self):
#         super().__init__('ground_truth_publisher')
#         self.pub = self.create_publisher(Pose, '/ground_truth', 10)
#         self.create_timer(0.2, self.publish_pose)  # 10 Hz
#         self.get_logger().info(
#             f'Ground truth publisher started — '
#             f'spawn offset: x={SPAWN_WORLD_X}, y={SPAWN_WORLD_Y}'
#         )

#     def publish_pose(self):
#         try:
#             result = subprocess.run(
#                 ['gz', 'model', '-m', 'waffle_pi', '-p'],
#                 capture_output=True,
#                 text=True,
#                 timeout=2
#             )

#             lines = result.stdout.strip().split('\n')
#             xyz_line = None
#             rpy_line = None

#             for i, line in enumerate(lines):
#                 if 'XYZ' in line:
#                     xyz_line = lines[i + 1].strip()
#                     rpy_line = lines[i + 2].strip()
#                     break

#             if xyz_line is None:
#                 return

#             # Parse [x y z] format
#             xyz = xyz_line.strip('[]').split()
#             rpy = rpy_line.strip('[]').split()

#             world_x = float(xyz[0])
#             world_y = float(xyz[1])
#             world_z = float(xyz[2])
#             roll    = float(rpy[0])
#             pitch   = float(rpy[1])
#             yaw     = float(rpy[2])

#             # ── Convert world frame → map frame ───────────────────────
#             # Robot at world [-2.0, -0.5] = map [0.0, 0.0]
#             # map_x = world_x - SPAWN_WORLD_X
#             # map_y = world_y - SPAWN_WORLD_Y
#             map_x = world_x - SPAWN_WORLD_X
#             map_y = world_y - SPAWN_WORLD_Y

#             # Convert yaw to quaternion
#             cy = cos(yaw * 0.5)
#             sy = sin(yaw * 0.5)
#             cp = cos(pitch * 0.5)
#             sp = sin(pitch * 0.5)
#             cr = cos(roll * 0.5)
#             sr = sin(roll * 0.5)

#             qw = cr * cp * cy + sr * sp * sy
#             qx = sr * cp * cy - cr * sp * sy
#             qy = cr * sp * cy + sr * cp * sy
#             qz = cr * cp * sy - sr * sp * cy

#             msg = Pose()
#             msg.position.x    = map_x    # map frame x
#             msg.position.y    = map_y    # map frame y
#             msg.position.z    = world_z  # z unchanged
#             msg.orientation.x = qx
#             msg.orientation.y = qy
#             msg.orientation.z = qz
#             msg.orientation.w = qw

#             self.pub.publish(msg)

#         except Exception as e:
#             self.get_logger().warn(f'Failed to get pose: {str(e)}')


# def main():
#     rclpy.init()
#     node = GroundTruthPublisher()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()






#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import Odometry
import subprocess
import threading
import re

class GroundTruthPublisher(Node):

    def __init__(self):
        super().__init__('ground_truth_publisher')

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.pub = self.create_publisher(Odometry, '/odometry/true_pose', qos)
        self.thread = threading.Thread(target=self.read_gz_topic, daemon=True)
        self.thread.start()
        self.get_logger().info('Ground truth publisher started.')

    def read_gz_topic(self):
        process = subprocess.Popen(
            ['gz', 'topic', '-e', '-t', '/world/default/dynamic_pose/info'],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True
        )
        buffer = []
        capture = False
        for line in process.stdout:
            line = line.rstrip()
            if 'name: "waffle_pi"' in line:
                buffer = []
                capture = True
            if capture:
                buffer.append(line)
            if capture and len(buffer) >= 12:
                self.parse_and_publish(buffer)
                buffer = []
                capture = False

    def parse_and_publish(self, lines):
        text = '\n'.join(lines)
        try:
            # ✅ Match numbers with OR without decimal point e.g. w: 1
            vals = re.findall(r'[xyzw]:\s+(-?[\d.eE+\-]+)', text)

            if len(vals) < 7:
                self.get_logger().warn(f'Not enough values parsed: {vals}')
                return

            x  = float(vals[0])
            y  = float(vals[1])
            z  = float(vals[2])
            ox = float(vals[3])
            oy = float(vals[4])
            oz = float(vals[5])
            ow = float(vals[6])

            # Safety check — waffle_pi body z is always ~0.01
            if not (0.005 < z < 0.02):
                return

            out = Odometry()
            out.header.stamp             = self.get_clock().now().to_msg()
            out.header.frame_id          = 'world'
            out.child_frame_id           = 'base_footprint'
            out.pose.pose.position.x     = x
            out.pose.pose.position.y     = y
            out.pose.pose.position.z     = z
            out.pose.pose.orientation.x  = ox
            out.pose.pose.orientation.y  = oy
            out.pose.pose.orientation.z  = oz
            out.pose.pose.orientation.w  = ow
            self.pub.publish(out)

        except Exception as e:
            self.get_logger().warn(f'Parse error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
