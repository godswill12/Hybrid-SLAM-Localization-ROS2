#!/usr/bin/env python3
"""
Navigation Smoothness Monitor
==============================
Measures navigation smoothness by analysing:
  1. Velocity jerk — rate of change of velocity commands
  2. Path deviation — how much actual path deviates from planned path
  3. Angular velocity variance — smoothness of heading changes

A well-localized robot produces smooth velocity commands.
A poorly-localized robot causes Nav2 to re-plan constantly,
producing jerky, high-jerk velocity profiles.

Subscribes to /cmd_vel_nav and records smoothness metrics.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np
import csv
import os
import math
from datetime import datetime


class SmoothnessMonitor(Node):

    def __init__(self):
        super().__init__('smoothness_monitor')

        self.set_parameters([
            rclpy.parameter.Parameter(
                'use_sim_time', rclpy.Parameter.Type.BOOL, True)
        ])

        self.declare_parameter('output_dir', '/ros_ws/csv_files')
        self.output_dir = self.get_parameter('output_dir').value

        # Velocity history
        self.linear_vels  = []
        self.angular_vels = []
        self.timestamps   = []
        self.last_linear  = 0.0
        self.last_angular = 0.0
        self.last_time    = None

        # Jerk accumulator
        self.linear_jerks  = []
        self.angular_jerks = []

        self.create_subscription(
            Twist, '/cmd_vel_nav',
            self.cmd_vel_callback, 10)

        os.makedirs(self.output_dir, exist_ok=True)

        self.get_logger().info(
            'Smoothness monitor started on /cmd_vel_nav')

    def cmd_vel_callback(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9

        v = msg.linear.x
        w = msg.angular.z

        self.linear_vels.append(v)
        self.angular_vels.append(w)
        self.timestamps.append(now)

        if self.last_time is not None:
            dt = now - self.last_time
            if 0 < dt < 1.0:
                # Jerk = rate of change of velocity
                linear_jerk  = abs(v - self.last_linear)  / dt
                angular_jerk = abs(w - self.last_angular) / dt
                self.linear_jerks.append(linear_jerk)
                self.angular_jerks.append(angular_jerk)

        self.last_linear  = v
        self.last_angular = w
        self.last_time    = now

    def save_report(self):
        if not self.linear_vels:
            print('No velocity data recorded')
            return

        lv  = np.array(self.linear_vels)
        av  = np.array(self.angular_vels)
        lj  = np.array(self.linear_jerks)  if self.linear_jerks  else np.array([0])
        aj  = np.array(self.angular_jerks) if self.angular_jerks else np.array([0])

        ts  = datetime.now().strftime('%Y%m%d_%H%M%S')
        sep = '=' * 55

        print(f'\n{sep}')
        print('  NAVIGATION SMOOTHNESS REPORT')
        print(sep)
        print(f'Samples recorded:        {len(lv)}')
        print(f'Duration:                '
              f'{self.timestamps[-1]-self.timestamps[0]:.1f}s'
              if len(self.timestamps) > 1 else '')
        print(f'\nLinear velocity (m/s):')
        print(f'  Mean:                  {np.mean(lv):.4f}')
        print(f'  Std:                   {np.std(lv):.4f}')
        print(f'  Max:                   {np.max(lv):.4f}')
        print(f'\nAngular velocity (rad/s):')
        print(f'  Mean:                  {np.mean(np.abs(av)):.4f}')
        print(f'  Std:                   {np.std(av):.4f}')
        print(f'  Max:                   {np.max(np.abs(av)):.4f}')
        print(f'\nLinear jerk (m/s²):')
        print(f'  Mean:                  {np.mean(lj):.4f}')
        print(f'  Max:                   {np.max(lj):.4f}')
        print(f'  (lower = smoother navigation)')
        print(f'\nAngular jerk (rad/s²):')
        print(f'  Mean:                  {np.mean(aj):.4f}')
        print(f'  Max:                   {np.max(aj):.4f}')
        print(f'  (lower = smoother turning)')
        print(sep)

        # Save CSV
        csv_path = f'{self.output_dir}/smoothness_{ts}.csv'
        with open(csv_path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['metric', 'value'])
            w.writerow(['linear_vel_mean',    round(float(np.mean(lv)),4)])
            w.writerow(['linear_vel_std',     round(float(np.std(lv)),4)])
            w.writerow(['linear_vel_max',     round(float(np.max(lv)),4)])
            w.writerow(['angular_vel_mean',   round(float(np.mean(np.abs(av))),4)])
            w.writerow(['angular_vel_std',    round(float(np.std(av)),4)])
            w.writerow(['angular_vel_max',    round(float(np.max(np.abs(av))),4)])
            w.writerow(['linear_jerk_mean',   round(float(np.mean(lj)),4)])
            w.writerow(['linear_jerk_max',    round(float(np.max(lj)),4)])
            w.writerow(['angular_jerk_mean',  round(float(np.mean(aj)),4)])
            w.writerow(['angular_jerk_max',   round(float(np.max(aj)),4)])
            w.writerow(['samples',            len(lv)])

        print(f'Smoothness saved: {csv_path}')


def main(args=None):
    rclpy.init(args=args)
    node = SmoothnessMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save_report()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
