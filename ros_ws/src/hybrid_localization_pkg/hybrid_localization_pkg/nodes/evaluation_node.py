#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import csv
import os
from datetime import datetime
from math import sqrt
from geometry_msgs.msg import PoseStamped, Pose
from tf_transformations import euler_from_quaternion

class EvaluationNode(Node):
    def __init__(self):
        super().__init__('evaluation_node')
        self.gt_sub = self.create_subscription(Pose, '/ground_truth', self.gt_callback, 10)
        self.create_subscription(PoseStamped, '/pose_ekf',    self.ekf_cb,    10)
        self.create_subscription(PoseStamped, '/pose_eif',    self.eif_cb,    10)
        self.create_subscription(PoseStamped, '/pose_amcl',   self.amcl_cb,   10)
        self.create_subscription(PoseStamped, '/pose_hybrid', self.hybrid_cb, 10)
        self.gt_pose     = None
        self.prev_gt     = None
        self.prev_poses  = {k: None for k in ['ekf','eif','amcl','hybrid']}
        self.ate_sq      = {k: [] for k in ['ekf','eif','amcl','hybrid']}
        self.rpe_sq      = {k: [] for k in ['ekf','eif','amcl','hybrid']}
        self.trajectories = {k: [] for k in ['ekf','eif','amcl','hybrid','gt']}
        results_dir = os.path.expanduser('~/thesis_results')
        os.makedirs(results_dir, exist_ok=True)
        ts  = datetime.now().strftime('%Y%m%d_%H%M%S')
        csv_path = os.path.join(results_dir, f'metrics_{ts}.csv')
        self.csv_file = open(csv_path, 'w', newline='')
        self.writer   = csv.writer(self.csv_file)
        self.writer.writerow(['timestamp','filter','gt_x','gt_y','est_x','est_y','pos_error','ate_rmse','rpe'])
        self.create_timer(5.0, self.print_summary)
        self.get_logger().info(f'Evaluation node started — saving to {csv_path}')

    def gt_callback(self, msg: Pose):
        x = msg.position.x
        y = msg.position.y
        q = msg.orientation
        _, _, theta = euler_from_quaternion([q.x, q.y, q.z, q.w])
        t = self.get_clock().now().nanoseconds * 1e-9
        self.gt_pose = (x, y, theta, t)
        self.trajectories['gt'].append(self.gt_pose)

    def ekf_cb(self, msg):    self._process('ekf',    msg)
    def eif_cb(self, msg):    self._process('eif',    msg)
    def amcl_cb(self, msg):   self._process('amcl',   msg)
    def hybrid_cb(self, msg): self._process('hybrid', msg)

    def _process(self, name, msg):
        if self.gt_pose is None:
            return
        x_est = msg.pose.position.x
        y_est = msg.pose.position.y
        q     = msg.pose.orientation
        _, _, theta_est = euler_from_quaternion([q.x, q.y, q.z, q.w])
        t     = self.get_clock().now().nanoseconds * 1e-9
        gt_x, gt_y, _, _ = self.gt_pose
        ate   = sqrt((x_est-gt_x)**2 + (y_est-gt_y)**2)
        self.ate_sq[name].append(ate**2)
        ate_rmse = sqrt(np.mean(self.ate_sq[name]))
        rpe = 0.0
        if self.prev_poses[name] is not None and self.prev_gt is not None:
            dx_est = x_est - self.prev_poses[name][0]
            dy_est = y_est - self.prev_poses[name][1]
            dx_gt  = gt_x  - self.prev_gt[0]
            dy_gt  = gt_y  - self.prev_gt[1]
            rpe    = sqrt((dx_est-dx_gt)**2 + (dy_est-dy_gt)**2)
            self.rpe_sq[name].append(rpe**2)
        self.prev_poses[name] = (x_est, y_est, theta_est, t)
        self.prev_gt           = self.gt_pose
        self.trajectories[name].append((x_est, y_est, theta_est, t))
        self.writer.writerow([f'{t:.3f}', name, f'{gt_x:.4f}', f'{gt_y:.4f}',
                              f'{x_est:.4f}', f'{y_est:.4f}', f'{ate:.4f}',
                              f'{ate_rmse:.4f}', f'{rpe:.4f}'])

    def print_summary(self):
        self.get_logger().info('── Localization Summary ────────────────')
        for name in ['ekf','eif','amcl','hybrid']:
            if self.ate_sq[name]:
                ate_rmse = sqrt(np.mean(self.ate_sq[name]))
                rpe_rmse = sqrt(np.mean(self.rpe_sq[name])) if self.rpe_sq[name] else 0.0
                self.get_logger().info(f'  {name.upper():8s}  ATE={ate_rmse:.4f}m   RPE={rpe_rmse:.4f}m')

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = EvaluationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
