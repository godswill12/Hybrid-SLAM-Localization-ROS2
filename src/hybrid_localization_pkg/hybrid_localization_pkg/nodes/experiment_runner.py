#!/usr/bin/env python3
"""
Experiment Runner
=================
Each iteration runs a full rectangular loop:
  Waypoint 1: (2.5,  0.0)  ← obstacle spawned mid-way here
  Waypoint 2: (2.5, -1.0)
  Waypoint 3: (0.0, -1.0)
  Waypoint 4: (0.0,  0.0)  ← back to start

Obstacle is spawned after obstacle_spawn_delay seconds into
waypoint 1 navigation, then removed after obstacle_duration seconds.

Runs num_iterations times and produces mean ± std report.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import numpy as np
import subprocess
import threading
import time
import csv
import os
import math
from datetime import datetime

from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus


# ── Helpers ───────────────────────────────────────────────────────────────────

def pose_error(x1, y1, x2, y2):
    return math.sqrt((x1-x2)**2 + (y1-y2)**2)

def avg(lst):
    v = [x for x in lst if x is not None]
    return float(np.mean(v)) if v else 0.0

def std_dev(lst):
    v = [x for x in lst if x is not None]
    return float(np.std(v)) if v else 0.0

def rmse(lst):
    v = [x for x in lst if x is not None]
    return float(np.sqrt(np.mean(np.array(v)**2))) if v else 0.0

def max_val(lst):
    v = [x for x in lst if x is not None]
    return float(max(v)) if v else 0.0

def pct_below(lst, thr):
    v = [x for x in lst if x is not None]
    return 100.0 * sum(1 for x in v if x < thr) / len(v) if v else 0.0


# ── Obstacle control ──────────────────────────────────────────────────────────

MODEL_PATH = '/ros_ws/models/gazebo_models/box_target_red/model.sdf'


def spawn_obstacle(name, x, y, z=0.5, model_path=MODEL_PATH):

    full_cmd = f"""
    source /opt/ros/jazzy/setup.bash &&
    source /ros_ws/ros_ws/install/setup.bash &&
    ros2 run ros_gz_sim create \
    -name {name} \
    -file {model_path} \
    -x {x} \
    -y {y} \
    -z {z}
    """

    try:

        print("\n========== SPAWNING OBSTACLE ==========")
        print(full_cmd)
        print("=======================================\n")

        process = subprocess.Popen(
            ['bash', '-c', full_cmd],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )

        # Give Gazebo time to spawn
        time.sleep(3.0)

        # Check if process already exited
        if process.poll() is not None:

            stdout, stderr = process.communicate()

            print("RETURN CODE:", process.returncode)
            print("STDOUT:")
            print(stdout)
            print("STDERR:")
            print(stderr)

            if process.returncode != 0:
                print(f'[spawn] Failed to spawn obstacle: {name}')
                return False

        print(f'[spawn] Obstacle "{name}" spawned successfully')
        return True

    except Exception as e:
        print(f'[spawn] Error: {e}')
        return False


def delete_obstacle(name):

    cmd = f'''
    gz service \
    -s /world/default/remove \
    --reqtype gz.msgs.Entity \
    --reptype gz.msgs.Boolean \
    --timeout 5000 \
    --req 'name: "{name}" type: MODEL'
    '''

    try:

        result = subprocess.run(
            ['bash', '-c', cmd],
            capture_output=True,
            text=True,
            timeout=10
        )

        print("\n========== DELETE OBSTACLE ==========")
        print("RETURN CODE:", result.returncode)
        print("STDOUT:")
        print(result.stdout)
        print("STDERR:")
        print(result.stderr)
        print("=====================================\n")

        return result.returncode == 0

    except Exception as e:
        print(f'[delete] Error: {e}')
        return False


# ── Waypoints ─────────────────────────────────────────────────────────────────

# Each waypoint: (x, y, label)
WAYPOINTS = [
    (2.5,  0.0,  'WP1 — forward'),
    (2.5, 1.0,  'WP2 — left'),
    (0.0, 1.0,  'WP3 — back'),
    (0.0,  0.0,  'WP4 — return to start'),
]

# Obstacle spawns during WP1 after this many seconds
OBSTACLE_WAYPOINT_INDEX = 0   # 0 = first waypoint leg


# ── Node ─────────────────────────────────────────────────────────────────────

class ExperimentRunner(Node):

    OBSTACLE_NAME = 'experiment_obstacle'

    def __init__(self):
        super().__init__('experiment_runner')

        self.set_parameters([
            rclpy.parameter.Parameter(
                'use_sim_time', rclpy.Parameter.Type.BOOL, True)
        ])

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter('num_iterations',       10)
        self.declare_parameter('obstacle_x',            0.0)
        self.declare_parameter('obstacle_y',            -0.5)
        self.declare_parameter('obstacle_z',            0.2)
        self.declare_parameter('obstacle_spawn_delay',  10.0)
        self.declare_parameter('obstacle_duration',    10.0)
        self.declare_parameter('nav_timeout',         120.0)
        self.declare_parameter('return_timeout',       90.0)
        self.declare_parameter('output_dir', '/ros_ws/csv_files')

        self.num_iterations       = self.get_parameter('num_iterations').value
        self.obstacle_x           = self.get_parameter('obstacle_x').value
        self.obstacle_y           = self.get_parameter('obstacle_y').value
        self.obstacle_z           = self.get_parameter('obstacle_z').value
        self.obstacle_spawn_delay = self.get_parameter('obstacle_spawn_delay').value
        self.obstacle_duration    = self.get_parameter('obstacle_duration').value
        self.nav_timeout          = self.get_parameter('nav_timeout').value
        self.return_timeout       = self.get_parameter('return_timeout').value
        self.output_dir           = self.get_parameter('output_dir').value

        # ── State ─────────────────────────────────────────────────────────
        self.iter_results   = []
        self.current_errors = {k: [] for k in
                               ['amcl','ekf','eif','rtab','fused','ekfr']}
        self.recording      = False
        self.iteration      = 0
        self.gt_offset_x    = None
        self.gt_offset_y    = None

        # Pose cache
        self.ekf_pose = self.amcl_pose = self.eif_pose  = None
        self.rtab_pose= self.fused_pose= self.ekfr_pose = None
        self.true_pose= None

        # ── Publishers ────────────────────────────────────────────────────
        self.obs_pub1 = self.create_publisher(
            Float64, '/obstacle_spawned_first',  10)
        self.obs_pub2 = self.create_publisher(
            Float64, '/obstacle_spawned_second', 10)

        # ── Nav2 action client ────────────────────────────────────────────
        self.nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose')

        # ── Subscriptions ─────────────────────────────────────────────────
        tqos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST, depth=10)

        self.create_subscription(
            Odometry, '/ekf_pose',      self.ekf_cb,   10)
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',               self.amcl_cb,  10)
        self.create_subscription(
            Odometry, '/eif_pose',      self.eif_cb,   10)
        self.create_subscription(
            Odometry, '/rtabmap_pose',  self.rtab_cb,  10)
        self.create_subscription(
            Odometry, '/fused_pose',    self.fused_cb, 10)
        self.create_subscription(
            Odometry, '/ekf_rtab_pose', self.ekfr_cb,  10)
        self.create_subscription(
            Odometry, '/odometry/true_pose',
            self.true_cb, tqos)

        # ── Timers ────────────────────────────────────────────────────────
        self.create_timer(0.2, self.record_errors)

        self._started = False
        self.create_timer(5.0, self._start_once)

        os.makedirs(self.output_dir, exist_ok=True)

        self.get_logger().info(
            f'\n{"="*60}\n'
            f'  EXPERIMENT RUNNER READY\n'
            f'  Iterations:       {self.num_iterations}\n'
            f'  Loop waypoints:\n'
            + '\n'.join(f'    {i+1}. ({x}, {y})  {lbl}'
                        for i,(x,y,lbl) in enumerate(WAYPOINTS)) +
            f'\n  Obstacle pos:     ({self.obstacle_x}, {self.obstacle_y})\n'
            f'  Obstacle spawns:  {self.obstacle_spawn_delay}s into WP1\n'
            f'  Obstacle removed: after {self.obstacle_duration}s\n'
            f'  Starting in 5 seconds...\n'
            f'{"="*60}'
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def ekf_cb(self, msg):
        self.ekf_pose = msg.pose.pose
        if self.gt_offset_x is None and self.true_pose is not None:
            tx = self.true_pose.position.x
            ty = self.true_pose.position.y
            self.gt_offset_x = tx - msg.pose.pose.position.x
            self.gt_offset_y = ty - msg.pose.pose.position.y
            self.get_logger().info(
                f'GT offset: dx={self.gt_offset_x:.4f} '
                f'dy={self.gt_offset_y:.4f}')

    def amcl_cb(self,msg):  self.amcl_pose  = msg.pose.pose
    def eif_cb(self,msg):   self.eif_pose   = msg.pose.pose
    def rtab_cb(self,msg):  self.rtab_pose  = msg.pose.pose
    def fused_cb(self,msg): self.fused_pose = msg.pose.pose
    def ekfr_cb(self,msg):  self.ekfr_pose  = msg.pose.pose
    def true_cb(self,msg):  self.true_pose  = msg.pose.pose

    # ── Error recording ───────────────────────────────────────────────────────

    def record_errors(self):
        if not self.recording:
            return
        if self.true_pose is None or self.gt_offset_x is None:
            return

        tx = self.true_pose.position.x - self.gt_offset_x
        ty = self.true_pose.position.y - self.gt_offset_y

        for key, pose in [
            ('ekf',   self.ekf_pose),
            ('amcl',  self.amcl_pose),
            ('eif',   self.eif_pose),
            ('rtab',  self.rtab_pose),
            ('fused', self.fused_pose),
            ('ekfr',  self.ekfr_pose),
        ]:
            if pose is not None:
                self.current_errors[key].append(
                    pose_error(pose.position.x, pose.position.y, tx, ty))

    # ── Experiment control ────────────────────────────────────────────────────

    def _start_once(self):
        if self._started:
            return
        self._started = True
        threading.Thread(
            target=self.run_all_iterations, daemon=True).start()

    def reset_errors(self):
        for k in self.current_errors:
            self.current_errors[k] = []

    def run_all_iterations(self):
        self.get_logger().info(
            f'=== STARTING {self.num_iterations} ITERATIONS ===')

        for i in range(1, self.num_iterations + 1):
            self.iteration = i
            self.get_logger().info(
                f'\n{"─"*55}\n'
                f'  ITERATION {i} / {self.num_iterations}\n'
                f'{"─"*55}')

            # Clean up leftover obstacle
            delete_obstacle(self.OBSTACLE_NAME)
            time.sleep(1.0)

            # Reset error accumulators
            self.reset_errors()
            self.recording = True

            # Run the full loop
            self.run_single_iteration(i)

            self.recording = False

            # Store results
            iter_stat = {}
            for k, errs in self.current_errors.items():
                iter_stat[k] = {
                    'avg':  avg(errs),
                    'std':  std_dev(errs),
                    'max':  max_val(errs),
                    'rmse': rmse(errs),
                    'p5cm': pct_below(errs, 0.05),
                    'n':    len(errs),
                }
            self.iter_results.append(iter_stat)

            self.get_logger().info(
                f'Iter {i} complete:\n'
                f'  AMCL          avg={iter_stat["amcl"]["avg"]:.4f}m\n'
                f'  EKF(AMCL)     avg={iter_stat["ekf"]["avg"]:.4f}m\n'
                f'  EIF(AMCL)     avg={iter_stat["eif"]["avg"]:.4f}m\n'
                f'  RTAB-Map      avg={iter_stat["rtab"]["avg"]:.4f}m\n'
                f'  Fused         avg={iter_stat["fused"]["avg"]:.4f}m\n'
                f'  EKF(RTAB)     avg={iter_stat["ekfr"]["avg"]:.4f}m'
            )

            # Pause before next iteration
            time.sleep(3.0)

        self.get_logger().info(
            f'=== ALL {self.num_iterations} ITERATIONS COMPLETE ===')
        self.generate_report()

    def run_single_iteration(self, n):
        """
        Navigate through all 4 waypoints.
        Spawn obstacle during waypoint 1 leg.
        Remove obstacle before waypoint 2.
        """
        obstacle_spawned = False

        for wp_idx, (x, y, label) in enumerate(WAYPOINTS):
            self.get_logger().info(
                f'[Iter {n}] Navigating to {label} ({x}, {y})')

            # Send this waypoint goal
            success, handle, result_future = self._send_goal(x, y)

            if not success:
                self.get_logger().warn(
                    f'[Iter {n}] {label} goal not accepted — skipping')
                continue

            # ── Obstacle logic for WP1 ────────────────────────────────────
            if wp_idx == OBSTACLE_WAYPOINT_INDEX and not obstacle_spawned:

                # Wait spawn delay then spawn
                self.get_logger().info(
                    f'[Iter {n}] Waiting {self.obstacle_spawn_delay}s '
                    f'then spawning obstacle...')
                time.sleep(self.obstacle_spawn_delay)

                spawned = spawn_obstacle(
                    self.OBSTACLE_NAME,
                    self.obstacle_x,
                    self.obstacle_y,
                    self.obstacle_z
                )

                if spawned:
                    obstacle_spawned = True
                    self.get_logger().info(
                        f'[Iter {n}] Obstacle spawned at '
                        f'({self.obstacle_x}, {self.obstacle_y})')

                    # Notify plotter
                    msg = Float64()
                    msg.data = float(
                        self.get_clock().now().nanoseconds * 1e-9)
                    self.obs_pub1.publish(msg)

                    # Wait obstacle duration
                    self.get_logger().info(
                        f'[Iter {n}] Obstacle active for '
                        f'{self.obstacle_duration}s...')
                    time.sleep(self.obstacle_duration)

                    # Remove obstacle
                    removed = delete_obstacle(self.OBSTACLE_NAME)
                    self.get_logger().info(
                        f'[Iter {n}] Obstacle removed: {removed}')
                else:
                    self.get_logger().warn(
                        f'[Iter {n}] Obstacle spawn failed')

            # ── Wait for this waypoint to complete ────────────────────────
            deadline = time.time() + self.nav_timeout
            while not result_future.done():
                time.sleep(0.3)
                if time.time() > deadline:
                    self.get_logger().warn(
                        f'[Iter {n}] {label} timed out — '
                        f'continuing to next waypoint')
                    try:
                        handle.cancel_goal_async()
                    except Exception:
                        pass
                    time.sleep(1.0)
                    break

            if result_future.done():
                status = result_future.result().status
                if status == GoalStatus.STATUS_SUCCEEDED:
                    self.get_logger().info(
                        f'[Iter {n}] {label} REACHED')
                else:
                    self.get_logger().warn(
                        f'[Iter {n}] {label} status={status}')

        # Make sure obstacle is cleaned up at end of iteration
        if obstacle_spawned:
            delete_obstacle(self.OBSTACLE_NAME)

        self.get_logger().info(
            f'[Iter {n}] Full loop complete.')

    def _send_goal(self, x, y):
        """Send a single nav goal. Returns (accepted, handle, result_future)."""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id    = 'map'
        goal_msg.pose.header.stamp       = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x    = float(x)
        goal_msg.pose.pose.position.y    = float(y)
        goal_msg.pose.pose.position.z    = 0.0
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Nav2 action server not available')
            return False, None, None

        future = self.nav_client.send_goal_async(goal_msg)

        deadline = time.time() + 10.0
        while not future.done():
            time.sleep(0.05)
            if time.time() > deadline:
                self.get_logger().error('Goal acceptance timed out')
                return False, None, None

        handle = future.result()
        if not handle.accepted:
            self.get_logger().error(f'Goal ({x},{y}) rejected')
            return False, None, None

        result_future = handle.get_result_async()
        return True, handle, result_future

    # ── Report ────────────────────────────────────────────────────────────────

    def generate_report(self):
        if not self.iter_results:
            self.get_logger().error('No results to report')
            return

        methods = ['amcl','ekf','eif','rtab','fused','ekfr']
        mnames  = {
            'amcl':  'AMCL',
            'ekf':   'EKF (AMCL corr)',
            'eif':   'EIF (AMCL corr)',
            'rtab':  'RTAB-Map',
            'fused': 'Fused (EKF+RTAB)',
            'ekfr':  'EKF (RTAB corr)',
        }

        # Collect per-iteration values
        data = {m: {'avg':[],'max':[],'rmse':[],'p5cm':[]}
                for m in methods}
        for r in self.iter_results:
            for m in methods:
                if m in r and r[m]['n'] > 0:
                    data[m]['avg'].append(r[m]['avg'])
                    data[m]['max'].append(r[m]['max'])
                    data[m]['rmse'].append(r[m]['rmse'])
                    data[m]['p5cm'].append(r[m]['p5cm'])

        n   = len(self.iter_results)
        sep = '=' * 78
        dsh = '-' * 78
        ts  = datetime.now().strftime('%Y%m%d_%H%M%S')

        lines = []
        lines.append(sep)
        lines.append(f'  EXPERIMENT REPORT — {n} ITERATIONS')
        lines.append(
            f'  Date:      {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}')
        lines.append(f'  Loop:      ' + ' → '.join(
            f'({x},{y})' for x,y,_ in WAYPOINTS))
        lines.append(
            f'  Obstacle:  ({self.obstacle_x}, {self.obstacle_y})  '
            f'spawned {self.obstacle_spawn_delay}s into WP1  '
            f'for {self.obstacle_duration}s')
        lines.append(sep)
        lines.append('')

        # ── Per-iteration table ───────────────────────────────────────────
        lines.append('PER-ITERATION AVERAGE LOCALIZATION ERROR (m):')
        lines.append('')

        hdr = f'{"Iter":<6}'
        for m in methods:
            hdr += f'  {mnames[m]:>16}'
        lines.append(hdr)
        lines.append(dsh)

        for i, r in enumerate(self.iter_results):
            row = f'{i+1:<6}'
            for m in methods:
                val = r.get(m, {}).get('avg', 0.0)
                row += f'  {val:>16.4f}'
            lines.append(row)

        lines.append(dsh)

        # Column means
        mean_row = f'{"Mean":<6}'
        for m in methods:
            v = data[m]['avg']
            mean_row += f'  {np.mean(v) if v else 0:>16.4f}'
        lines.append(mean_row)

        std_row = f'{"Std":<6}'
        for m in methods:
            v = data[m]['avg']
            std_row += f'  {np.std(v) if v else 0:>16.4f}'
        lines.append(std_row)
        lines.append('')

        # ── Summary table ─────────────────────────────────────────────────
        lines.append('SUMMARY TABLE — MEAN ± STD ACROSS ALL ITERATIONS:')
        lines.append('')
        lines.append(
            f'{"Method":<22}  {"Avg err (m)":^20}  '
            f'{"Max err (m)":^20}  {"RMSE (m)":^18}  {"<5cm (%)":^16}')
        lines.append(dsh)

        for m in methods:
            d = data[m]
            if not d['avg']:
                continue
            lines.append(
                f'{mnames[m]:<22}  '
                f'{np.mean(d["avg"]):>8.4f} ± {np.std(d["avg"]):<8.4f}  '
                f'{np.mean(d["max"]):>8.4f} ± {np.std(d["max"]):<8.4f}  '
                f'{np.mean(d["rmse"]):>6.4f} ± {np.std(d["rmse"]):<6.4f}  '
                f'{np.mean(d["p5cm"]):>5.1f} ± {np.std(d["p5cm"]):.1f}%'
            )

        lines.append(sep)

        report_str = '\n'.join(lines)
        print(f'\n{report_str}\n')

        # Save report
        report_path = f'{self.output_dir}/report_{ts}.txt'
        with open(report_path, 'w') as f:
            f.write(report_str + '\n')
        print(f'Report saved: {report_path}')

        # Save CSV
        csv_path = f'{self.output_dir}/iterations_{ts}.csv'
        with open(csv_path, 'w', newline='') as f:
            w = csv.writer(f)

            # Header
            header = ['iteration']
            for m in methods:
                nm = mnames[m].replace(' ','_').replace('(','').replace(')','')
                header += [f'{nm}_avg', f'{nm}_max',
                           f'{nm}_rmse', f'{nm}_p5cm']
            w.writerow(header)

            # Per-iteration rows
            for i, r in enumerate(self.iter_results):
                row = [i+1]
                for m in methods:
                    s = r.get(m, {})
                    row += [
                        round(s.get('avg',  0), 6),
                        round(s.get('max',  0), 6),
                        round(s.get('rmse', 0), 6),
                        round(s.get('p5cm', 0), 2),
                    ]
                w.writerow(row)

            # Summary rows
            w.writerow([])
            for stat, fn in [('mean', np.mean), ('std', np.std)]:
                row = [stat]
                for m in methods:
                    for k in ['avg','max','rmse','p5cm']:
                        v = data[m][k]
                        row.append(round(float(fn(v)), 6) if v else '')
                w.writerow(row)

        print(f'CSV saved:    {csv_path}')
        self.get_logger().info('Report generation complete.')


def main(args=None):
    rclpy.init(args=args)
    node = ExperimentRunner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node.iter_results:
            node.get_logger().info(
                'Interrupted — saving partial report...')
            node.generate_report()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()