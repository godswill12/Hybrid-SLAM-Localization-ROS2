#!/usr/bin/env python3
"""
Resource Monitor
================
Monitors CPU and memory usage of all localization nodes.
Records per-node resource consumption during experiments.

Run alongside experiment to get computational efficiency metrics.

Publishes summary to /resource_stats topic and saves to CSV.
"""

import rclpy
from rclpy.node import Node
import psutil
import os
import csv
import time
from datetime import datetime


# Node names to monitor
MONITOR_NODES = [
    'ekf_node',
    'eif_node',
    'rtabmap',
    'fusion_node',
    'ekf_rtab_node',
    'amcl',
]


class ResourceMonitor(Node):

    def __init__(self):
        super().__init__('resource_monitor')

        self.set_parameters([
            rclpy.parameter.Parameter(
                'use_sim_time', rclpy.Parameter.Type.BOOL, True)
        ])

        self.declare_parameter('output_dir', '/ros_ws/csv_files')
        self.declare_parameter('sample_rate_hz', 1.0)

        self.output_dir  = self.get_parameter('output_dir').value
        self.sample_rate = self.get_parameter('sample_rate_hz').value

        # Storage: {node_name: {cpu: [], mem: [], timestamps: []}}
        self.data = {n: {'cpu': [], 'mem': [], 'timestamps': []}
                     for n in MONITOR_NODES}

        self.start_time = time.time()

        os.makedirs(self.output_dir, exist_ok=True)

        rate = 1.0 / self.sample_rate
        self.create_timer(rate, self.sample)

        self.get_logger().info(
            f'Resource monitor started.\n'
            f'Monitoring: {", ".join(MONITOR_NODES)}'
        )

    def find_pid(self, node_name):
        """Find PID of a process containing node_name."""
        for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
            try:
                cmdline = ' '.join(proc.info['cmdline'] or [])
                if node_name in cmdline:
                    return proc.info['pid']
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                pass
        return None

    def sample(self):
        t = time.time() - self.start_time

        for node_name in MONITOR_NODES:
            pid = self.find_pid(node_name)
            if pid is None:
                continue
            try:
                proc = psutil.Process(pid)
                cpu  = proc.cpu_percent(interval=None)
                mem  = proc.memory_info().rss / 1024 / 1024  # MB
                self.data[node_name]['cpu'].append(cpu)
                self.data[node_name]['mem'].append(mem)
                self.data[node_name]['timestamps'].append(t)
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                pass

    def save_report(self):
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')

        # Print summary table
        sep  = '=' * 65
        dash = '-' * 65

        print(f'\n{sep}')
        print('  COMPUTATIONAL EFFICIENCY REPORT')
        print(sep)
        print(f'{"Node":<22} {"Avg CPU%":>10} {"Max CPU%":>10} '
              f'{"Avg Mem(MB)":>12} {"Max Mem(MB)":>12}')
        print(dash)

        summary = {}
        for node_name in MONITOR_NODES:
            d = self.data[node_name]
            if not d['cpu']:
                continue

            import numpy as np
            avg_cpu = float(np.mean(d['cpu']))
            max_cpu = float(np.max(d['cpu']))
            avg_mem = float(np.mean(d['mem']))
            max_mem = float(np.max(d['mem']))

            summary[node_name] = {
                'avg_cpu': avg_cpu, 'max_cpu': max_cpu,
                'avg_mem': avg_mem, 'max_mem': max_mem,
            }

            print(f'{node_name:<22} {avg_cpu:>10.2f} {max_cpu:>10.2f} '
                  f'{avg_mem:>12.1f} {max_mem:>12.1f}')

        print(sep)

        # Save CSV
        csv_path = f'{self.output_dir}/resources_{ts}.csv'
        with open(csv_path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['node', 'avg_cpu_%', 'max_cpu_%',
                        'avg_mem_mb', 'max_mem_mb'])
            for node_name, s in summary.items():
                w.writerow([
                    node_name,
                    round(s['avg_cpu'], 3),
                    round(s['max_cpu'], 3),
                    round(s['avg_mem'], 2),
                    round(s['max_mem'], 2),
                ])

        print(f'Resources saved: {csv_path}')


def main(args=None):
    rclpy.init(args=args)
    node = ResourceMonitor()
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
