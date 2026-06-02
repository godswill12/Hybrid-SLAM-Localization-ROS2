# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
# import math
# import csv
# import os
# import matplotlib
# matplotlib.use('Agg')
# import matplotlib.pyplot as plt
# from datetime import datetime

# from std_msgs.msg import Float64

# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import PoseWithCovarianceStamped


# def pose_error(x1, y1, x2, y2):
#     return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)


# def quaternion_to_yaw(q):
#     siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
#     cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
#     return math.atan2(siny_cosp, cosy_cosp)


# class TrajPlotter(Node):

#     def __init__(self):
#         super().__init__('traj_plotter')

#         # ── Storage ─────────────────────────────────────
#         self.true_xs, self.true_ys, self.true_yaws = [], [], []

#         self.ekf_xs, self.ekf_ys, self.ekf_yaws = [], [], []
#         self.amcl_xs, self.amcl_ys, self.amcl_yaws = [], [], []
#         self.eif_xs, self.eif_ys, self.eif_yaws = [], [], []

#         self.ekf_errors, self.amcl_errors, self.eif_errors = [], [], []
#         self.timestamps = []

#         self.ekf_pose = None
#         self.amcl_pose = None
#         self.eif_pose = None
#         self.true_pose = None

#         # QoS
#         true_qos = QoSProfile(
#             reliability=QoSReliabilityPolicy.BEST_EFFORT,
#             history=QoSHistoryPolicy.KEEP_LAST,
#             depth=10
#         )

#         # ── Obstacle spawn time tracking ──────────────────────────────────────
#         self.obstacle_spawn_time = None   # will be set when obstacle appears

#         # Subscribe to a trigger topic — we publish to this from Terminal 10

#         # Subscriptions
#         self.create_subscription(
#             Float64,
#             '/obstacle_spawned',
#             self.obstacle_callback,
#             10
#         )        
#         self.create_subscription(Odometry, '/ekf_pose', self.ekf_callback, 10)
#         self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_callback, 10)
#         self.create_subscription(Odometry, '/eif_pose', self.eif_callback, 10)
#         self.create_subscription(Odometry, '/odometry/true_pose', self.true_callback, true_qos)

#         self.create_timer(0.2, self.record_data)

#         print("Trajectory plotter started. Press Ctrl+C to save.")

#     def obstacle_callback(self, msg):
#         """Records the sim time when the obstacle was spawned."""
#         self.obstacle_spawn_time = self.get_clock().now().nanoseconds * 1e-9
#         self.get_logger().info(
#             f'Obstacle spawn recorded at t={self.obstacle_spawn_time:.2f}s'
#         )

#     # ── Callbacks ─────────────────────────────────────

#     def ekf_callback(self, msg):
#         self.ekf_pose = msg.pose.pose

#     def amcl_callback(self, msg):
#         self.amcl_pose = msg.pose.pose

#     def eif_callback(self, msg):
#         self.eif_pose = msg.pose.pose

#     def true_callback(self, msg):
#         self.true_pose = msg.pose.pose

#     # ── Recording ─────────────────────────────────────

#     def record_data(self):
#         if self.true_pose is None:
#             return

#         tx = self.true_pose.position.x
#         ty = self.true_pose.position.y
#         tyaw = quaternion_to_yaw(self.true_pose.orientation)

#         self.true_xs.append(tx)
#         self.true_ys.append(ty)
#         self.true_yaws.append(tyaw)

#         now = self.get_clock().now().nanoseconds * 1e-9
#         self.timestamps.append(now)

#         # EKF
#         if self.ekf_pose:
#             ex = self.ekf_pose.position.x
#             ey = self.ekf_pose.position.y
#             eyaw = quaternion_to_yaw(self.ekf_pose.orientation)

#             self.ekf_xs.append(ex)
#             self.ekf_ys.append(ey)
#             self.ekf_yaws.append(eyaw)
#             self.ekf_errors.append(pose_error(ex, ey, tx, ty))
#         else:
#             self.ekf_xs.append(None)
#             self.ekf_ys.append(None)
#             self.ekf_yaws.append(None)
#             self.ekf_errors.append(None)

#         # AMCL
#         if self.amcl_pose:
#             ax = self.amcl_pose.position.x
#             ay = self.amcl_pose.position.y
#             ayaw = quaternion_to_yaw(self.amcl_pose.orientation)

#             self.amcl_xs.append(ax)
#             self.amcl_ys.append(ay)
#             self.amcl_yaws.append(ayaw)
#             self.amcl_errors.append(pose_error(ax, ay, tx, ty))
#         else:
#             self.amcl_xs.append(None)
#             self.amcl_ys.append(None)
#             self.amcl_yaws.append(None)
#             self.amcl_errors.append(None)

#         # EIF
#         if self.eif_pose:
#             ix = self.eif_pose.position.x
#             iy = self.eif_pose.position.y
#             iyaw = quaternion_to_yaw(self.eif_pose.orientation)

#             self.eif_xs.append(ix)
#             self.eif_ys.append(iy)
#             self.eif_yaws.append(iyaw)
#             self.eif_errors.append(pose_error(ix, iy, tx, ty))
#         else:
#             self.eif_xs.append(None)
#             self.eif_ys.append(None)
#             self.eif_yaws.append(None)
#             self.eif_errors.append(None)

#     # ── CSV ───────────────────────────────────────────

#     def save_csv(self, path, t0):
#         def fmt(v): return '' if v is None else round(v, 6)
#         def deg(v): return '' if v is None else round(math.degrees(v), 4)

#         with open(path, 'w', newline='') as f:
#             writer = csv.writer(f)

#             writer.writerow([
#                 'time_s',
#                 'true_x','true_y','true_yaw',
#                 'ekf_x','ekf_y','ekf_yaw','ekf_err',
#                 'amcl_x','amcl_y','amcl_yaw','amcl_err',
#                 'eif_x','eif_y','eif_yaw','eif_err'
#             ])

#             for i in range(len(self.timestamps)):
#                 t = self.timestamps[i] - t0

#                 writer.writerow([
#                     round(t,4),
#                     fmt(self.true_xs[i]), fmt(self.true_ys[i]), deg(self.true_yaws[i]),
#                     fmt(self.ekf_xs[i]), fmt(self.ekf_ys[i]), deg(self.ekf_yaws[i]), fmt(self.ekf_errors[i]),
#                     fmt(self.amcl_xs[i]), fmt(self.amcl_ys[i]), deg(self.amcl_yaws[i]), fmt(self.amcl_errors[i]),
#                     fmt(self.eif_xs[i]), fmt(self.eif_ys[i]), deg(self.eif_yaws[i]), fmt(self.eif_errors[i])
#                 ])
        
#         # ── Append summary stats to the same CSV ──────────────────────────
#         writer.writerow([])
#         writer.writerow(['=== SUMMARY STATS ==='])
#         writer.writerow(['metric', 'AMCL', 'EKF', 'EIF'])

#         def stats_csv(lst):
#             vals = [v for v in lst if v is not None]
#             if not vals:
#                 return 0, 0, 0, 0
#             n   = len(vals)
#             avg = sum(vals) / n
#             mx  = max(vals)
#             mn  = min(vals)
#             std = math.sqrt(sum((v - avg)**2 for v in vals) / n)
#             return round(avg,6), round(mx,6), round(mn,6), round(std,6)

#         def path_length_csv(xs, ys):
#             pts = [(x,y) for x,y in zip(xs,ys) if x is not None and y is not None]
#             if len(pts) < 2:
#                 return 0.0
#             return round(sum(
#                 math.sqrt((pts[i][0]-pts[i-1][0])**2 + (pts[i][1]-pts[i-1][1])**2)
#                 for i in range(1, len(pts))
#             ), 6)

#         a_avg,a_max,a_min,a_std = stats_csv(self.amcl_errors)
#         e_avg,e_max,e_min,e_std = stats_csv(self.ekf_errors)
#         i_avg,i_max,i_min,i_std = stats_csv(self.eif_errors)

#         a_len = path_length_csv(self.amcl_xs, self.amcl_ys)
#         e_len = path_length_csv(self.ekf_xs,  self.ekf_ys)
#         i_len = path_length_csv(self.eif_xs,  self.eif_ys)

#         straight = 0.0
#         if len(self.true_xs) >= 2:
#             straight = round(math.sqrt(
#                 (self.true_xs[-1]-self.true_xs[0])**2 +
#                 (self.true_ys[-1]-self.true_ys[0])**2
#             ), 6)

#         writer.writerow(['avg_error_m',      a_avg, e_avg, i_avg])
#         writer.writerow(['max_error_m',      a_max, e_max, i_max])
#         writer.writerow(['min_error_m',      a_min, e_min, i_min])
#         writer.writerow(['std_deviation_m',  a_std, e_std, i_std])
#         writer.writerow(['path_length_m',    a_len, e_len, i_len])
#         writer.writerow(['straight_line_m',  straight, straight, straight])
#         writer.writerow(['path_deviation_m',
#                         round(a_len-straight,6),
#                         round(e_len-straight,6),
#                         round(i_len-straight,6)])

#         print(f"CSV saved: {path}")

#     # ── Plot ──────────────────────────────────────────

#     def save_plot(self):
#         if len(self.true_xs) < 2:
#             print("Not enough data")
#             return

#         timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
#         os.makedirs('/ros_ws/csv_files', exist_ok=True)

#         plot_path = f'/ros_ws/csv_files/trajectory_{timestamp}.png'
#         err_path  = f'/ros_ws/csv_files/error_{timestamp}.png'
#         csv_path  = f'/ros_ws/csv_files/data_{timestamp}.csv'

#         t0 = self.timestamps[0]
#         times = [t - t0 for t in self.timestamps]

#         def clean(xs, ys):
#             return zip(*[(x,y) for x,y in zip(xs,ys) if x is not None])

#         # ── Trajectory ──
#         fig, ax = plt.subplots()

#         ax.plot(self.true_xs, self.true_ys, label='GT', linewidth=3)

#         if any(self.ekf_xs):
#             x,y = clean(self.ekf_xs,self.ekf_ys)
#             ax.plot(list(x),list(y),'--',label='EKF')

#         if any(self.amcl_xs):
#             x,y = clean(self.amcl_xs,self.amcl_ys)
#             ax.plot(list(x),list(y),':',label='AMCL')

#         if any(self.eif_xs):
#             x,y = clean(self.eif_xs,self.eif_ys)
#             ax.plot(list(x),list(y),'-.',label='EIF')

#         ax.legend()
#         ax.set_aspect('equal')
#         ax.grid()

#         fig.savefig(plot_path)
#         plt.close()

#         # ── Error plot ──────────────────────────────────────────────────────
#         fig2, ax2 = plt.subplots(figsize=(12, 5))

#         def valid(times, errs):
#             pairs = [(t, e) for t, e in zip(times, errs) if e is not None]
#             if not pairs:
#                 return [], []
#             t_out, e_out = zip(*pairs)
#             return list(t_out), list(e_out)

#         t_ekf,  e_ekf  = valid(times, self.ekf_errors)
#         t_amcl, e_amcl = valid(times, self.amcl_errors)
#         t_eif,  e_eif  = valid(times, self.eif_errors)

#         if e_ekf:
#             ax2.plot(t_ekf,  e_ekf,  label='EKF',  color='blue',   linewidth=1.5)
#         if e_amcl:
#             ax2.plot(t_amcl, e_amcl, label='AMCL', color='red',    linewidth=1.5, linestyle='--')
#         if e_eif:
#             ax2.plot(t_eif,  e_eif,  label='EIF',  color='orange', linewidth=1.5, linestyle='-.')

#         # ── Mark obstacle spawn time ────────────────────────────────────────
#         if self.obstacle_spawn_time is not None:
#             obs_t = self.obstacle_spawn_time - t0
#             ax2.axvline(
#                 x=obs_t,
#                 color='green',
#                 linewidth=2,
#                 linestyle=':',
#                 label=f'Obstacle spawned (t={obs_t:.1f}s)'
#             )
#             # Shade the region after obstacle appears
#             ax2.axvspan(
#                 obs_t,
#                 max(times),
#                 alpha=0.06,
#                 color='green',
#                 label='With obstacle'
#             )

#         ax2.axhline(y=0.05, color='gray', linestyle=':', linewidth=1, label='5 cm threshold')
#         ax2.set_xlabel('Time (s)', fontsize=12)
#         ax2.set_ylabel('Position error (m)', fontsize=12)
#         ax2.set_title('Localization Error Over Time', fontsize=14, fontweight='bold')
#         ax2.legend(fontsize=10)
#         ax2.grid(True, alpha=0.3)
#         ax2.set_ylim(bottom=0)

#         fig2.tight_layout()
#         fig2.savefig(err_path, dpi=150)
#         plt.close(fig2)
#         print(f'Error plot saved: {err_path}')


# def main():
#     rclpy.init()
#     node = TrajPlotter()

#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         print("Saving results...")
#         node.save_plot()

#     node.destroy_node()
#     try:
#         rclpy.shutdown()
#     except:
#         pass


# if __name__ == '__main__':
#     main()








# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
# import math
# import csv
# import os
# import matplotlib
# matplotlib.use('Agg')
# import matplotlib.pyplot as plt
# from tf2_ros import Buffer, TransformListener
# from datetime import datetime

# from std_msgs.msg import Float64
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import PoseWithCovarianceStamped


# def pose_error(x1, y1, x2, y2):
#     return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)


# def quaternion_to_yaw(q):
#     siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
#     cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
#     return math.atan2(siny_cosp, cosy_cosp)


# class TrajPlotter(Node):

#     def __init__(self):
#         super().__init__('traj_plotter')

#         # ── Storage ─────────────────────────────────────
#         self.true_xs, self.true_ys, self.true_yaws = [], [], []

#         self.ekf_xs, self.ekf_ys, self.ekf_yaws = [], [], []
#         self.eif_xs, self.eif_ys, self.eif_yaws = [], [], []

#         self.ekf_errors, self.eif_errors = [], []
#         self.timestamps = []

#         self.carto_xs, self.carto_ys = [], []
#         self.carto_errors = []

#         self.tf_buffer = Buffer()
#         self.tf_listener = TransformListener(self.tf_buffer, self)

#         self.ekf_pose = None
#         self.eif_pose = None
#         self.true_pose = None

#         # ✅ MULTIPLE obstacle times
#         self.obstacle_spawn_times = []

#         # QoS
#         true_qos = QoSProfile(
#             reliability=QoSReliabilityPolicy.BEST_EFFORT,
#             history=QoSHistoryPolicy.KEEP_LAST,
#             depth=10
#         )

#         # ✅ Subscribe to BOTH topics you publish
#         self.create_subscription(Float64, '/obstacle_spawned_first',  self.obstacle_callback, 10)
#         self.create_subscription(Float64, '/obstacle_spawned_second', self.obstacle_callback, 10)

#         # Other subscriptions
#         self.create_subscription(Odometry, '/ekf_pose', self.ekf_callback, 10)
#         self.create_subscription(Odometry, '/eif_pose', self.eif_callback, 10)
#         self.create_subscription(Odometry, '/odometry/true_pose', self.true_callback, true_qos)

#         self.create_timer(0.2, self.record_data)

#         print("Trajectory plotter started. Press Ctrl+C to save.")

#     # ── Obstacle Callback ─────────────────────────────────────

#     def obstacle_callback(self, msg):
#         t = self.get_clock().now().nanoseconds * 1e-9
#         self.obstacle_spawn_times.append(t)

#         self.get_logger().info(
#             f'Obstacle #{len(self.obstacle_spawn_times)} at t={t:.2f}s'
#         )

#     # ── Callbacks ─────────────────────────────────────

#     def ekf_callback(self, msg):
#         self.ekf_pose = msg.pose.pose

#     def eif_callback(self, msg):
#         self.eif_pose = msg.pose.pose

#     def true_callback(self, msg):
#         self.true_pose = msg.pose.pose

#     # ── Recording ─────────────────────────────────────

#     def record_data(self):
#         if self.true_pose is None:
#             return

#         # ── Ground Truth ─────────────────────────────
#         tx = self.true_pose.position.x
#         ty = self.true_pose.position.y
#         tyaw = quaternion_to_yaw(self.true_pose.orientation)

#         self.true_xs.append(tx)
#         self.true_ys.append(ty)
#         self.true_yaws.append(tyaw)

#         now = self.get_clock().now().nanoseconds * 1e-9
#         self.timestamps.append(now)

#         # ── EKF + EIF (no AMCL anymore) ──────────────
#         for pose, xs, ys, yaws, errs in [
#             (self.ekf_pose, self.ekf_xs, self.ekf_ys, self.ekf_yaws, self.ekf_errors),
#             (self.eif_pose, self.eif_xs, self.eif_ys, self.eif_yaws, self.eif_errors),
#         ]:
#             if pose:
#                 px = pose.position.x
#                 py = pose.position.y
#                 pyaw = quaternion_to_yaw(pose.orientation)

#                 xs.append(px)
#                 ys.append(py)
#                 yaws.append(pyaw)
#                 errs.append(pose_error(px, py, tx, ty))
#             else:
#                 xs.append(None)
#                 ys.append(None)
#                 yaws.append(None)
#                 errs.append(None)

#         # ── Cartographer (Graph SLAM via TF) ─────────
#         try:
#             transform = self.tf_buffer.lookup_transform(
#                 'map',
#                 'base_footprint',
#                 rclpy.time.Time()
#             )

#             cx = transform.transform.translation.x
#             cy = transform.transform.translation.y

#             self.carto_xs.append(cx)
#             self.carto_ys.append(cy)

#             # Optional: yaw (if you want later)
#             # cyaw = quaternion_to_yaw(transform.transform.rotation)
#             # self.carto_yaws.append(cyaw)

#             self.carto_errors.append(pose_error(cx, cy, tx, ty))

#         except:
#             self.carto_xs.append(None)
#             self.carto_ys.append(None)
#             self.carto_errors.append(None)

#     # ── CSV (FIXED indentation + obstacle logging) ─────────────────────────

#     def save_csv(self, path, t0):
#         def fmt(v): return '' if v is None else round(v, 6)
#         def deg(v): return '' if v is None else round(math.degrees(v), 4)

#         with open(path, 'w', newline='') as f:
#             writer = csv.writer(f)

#             writer.writerow([
#                 'time_s',
#                 'true_x','true_y','true_yaw',
#                 'ekf_x','ekf_y','ekf_yaw','ekf_err',
#                 'eif_x','eif_y','eif_yaw','eif_err',
#                 'carto_x','carto_y','carto_err'
#             ])

#             for i in range(len(self.timestamps)):
#                 t = self.timestamps[i] - t0

#                 writer.writerow([
#                     round(t,4),
#                     fmt(self.true_xs[i]), fmt(self.true_ys[i]), deg(self.true_yaws[i]),

#                     fmt(self.ekf_xs[i]), fmt(self.ekf_ys[i]), deg(self.ekf_yaws[i]), fmt(self.ekf_errors[i]),
#                     fmt(self.eif_xs[i]), fmt(self.eif_ys[i]), deg(self.eif_yaws[i]), fmt(self.eif_errors[i]),

#                     fmt(self.carto_xs[i]), fmt(self.carto_ys[i]), fmt(self.carto_errors[i])
#                 ])

#         print(f"CSV saved: {path}")

#     def rmse(self, errors):
#         vals = [e for e in errors if e is not None]
#         if not vals:
#             return 0.0
#         return math.sqrt(sum(e**2 for e in vals) / len(vals))

#     # ── Plot ──────────────────────────────────────────

#     def save_plot(self):
#         if len(self.true_xs) < 2:
#             print("Not enough data")
#             return

#         timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
#         os.makedirs('/ros_ws/csv_files', exist_ok=True)

#         traj_path = f'/ros_ws/csv_files/trajectory_{timestamp}.png'
#         err_path  = f'/ros_ws/csv_files/error_{timestamp}.png'
#         csv_path  = f'/ros_ws/csv_files/data_{timestamp}.csv'

#         t0 = self.timestamps[0]
#         times = [t - t0 for t in self.timestamps]

#         # ───────────────────────────────────────────────
#         # ✅ TRAJECTORY PLOT (NEW)
#         # ───────────────────────────────────────────────
#         fig1, ax1 = plt.subplots(figsize=(8, 8))

#         def clean(xs, ys):
#             pts = [(x, y) for x, y in zip(xs, ys) if x is not None]
#             if not pts:
#                 return [], []
#             cx, cy = zip(*pts)
#             return list(cx), list(cy)

#         # Ground truth
#         ax1.plot(self.true_xs, self.true_ys,
#                 linewidth=3, label='Ground Truth')

#         # EKF
#         x, y = clean(self.ekf_xs, self.ekf_ys)
#         if x:
#             ax1.plot(x, y, '--', label='EKF')

    

#         # EIF
#         x, y = clean(self.eif_xs, self.eif_ys)
#         if x:
#             ax1.plot(x, y, '-.', label='EIF')

#         # ✅ Start and End markers
#         ax1.scatter(self.true_xs[0], self.true_ys[0],
#                     s=120, marker='o', label='Start')

#         ax1.scatter(self.true_xs[-1], self.true_ys[-1],
#                     s=150, marker='*', label='End')

#         ax1.set_xlabel('X (m)')
#         ax1.set_ylabel('Y (m)')
#         # Cartographer trajectory
#         x, y = clean(self.carto_xs, self.carto_ys)
#         if x:
#             ax1.plot(x, y, label='Graph SLAM (Cartographer)', linestyle=':')

#         ax1.set_title('Trajectory Comparison: Ground Truth vs EKF vs EIF vs Graph SLAM')
#         ax1.legend()
#         ax1.grid()
#         ax1.set_aspect('equal')

#         fig1.tight_layout()
#         fig1.savefig(traj_path, dpi=150)
#         plt.close(fig1)

#         print(f'Trajectory plot saved: {traj_path}')

#         # ───────────────────────────────────────────────
#         # ERROR PLOT (your existing one)
#         # ───────────────────────────────────────────────
#         fig2, ax2 = plt.subplots(figsize=(12, 5))

#         def valid(times, errs):
#             pairs = [(t, e) for t, e in zip(times, errs) if e is not None]
#             if not pairs:
#                 return [], []
#             t_out, e_out = zip(*pairs)
#             return list(t_out), list(e_out)

#         t_ekf, e_ekf = valid(times, self.ekf_errors)
#         t_eif, e_eif = valid(times, self.eif_errors)
#         t_carto, e_carto = valid(times, self.carto_errors)

#         if e_ekf:
#             ax2.plot(t_ekf, e_ekf, label='EKF')

#         if e_eif:
#             ax2.plot(t_eif, e_eif, linestyle='-.', label='EIF')

#         if e_carto:
#             ax2.plot(t_carto, e_carto, linestyle=':', label='Graph SLAM (Cartographer)')

#         # Obstacles
#         for i, obs_time in enumerate(self.obstacle_spawn_times):
#             obs_t = obs_time - t0
#             ax2.axvline(
#                 x=obs_t,
#                 linestyle=':',
#                 linewidth=2,
#                 label=f'Obstacle {i+1}' if i == 0 else None
#             )

#         ax2.axhline(y=0.05, linestyle='--', label='5 cm threshold')

#         ax2.set_xlabel('Time (s)')
#         ax2.set_ylabel('Position Error (m)')
#         ax2.set_title('Localization Error Comparison (EKF vs EIF vs Graph SLAM)')
#         ax2.legend()
#         ax2.grid()

#         fig2.savefig(err_path)
#         plt.close(fig2)

#         print(f'Error plot saved: {err_path}')

#         # Save CSV
#         self.save_csv(csv_path, t0)

#         # ───────────────────────────────────────────────
#         # FINAL RESULTS
#         # ───────────────────────────────────────────────
#         ekf_rmse   = self.rmse(self.ekf_errors)
#         eif_rmse   = self.rmse(self.eif_errors)
#         carto_rmse = self.rmse(self.carto_errors)

#         print("\n" + "=" * 45)
#         print("        FINAL LOCALIZATION RESULTS")
#         print("=" * 45)

#         print(f"EKF RMSE          : {ekf_rmse:.4f} m")
#         print(f"EIF RMSE          : {eif_rmse:.4f} m")
#         print(f"Graph SLAM RMSE   : {carto_rmse:.4f} m")

#         print("=" * 45 + "\n")


# def main():
#     rclpy.init()
#     node = TrajPlotter()

#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         print("Saving results...")
#         node.save_plot()
#     finally:
#         node.destroy_node()
#         if rclpy.ok():
#             rclpy.shutdown()    


# if __name__ == '__main__':
#     main()













#!/usr/bin/env python3
"""
Thesis Plotter — Figures 4.1 to 4.7
=====================================
Records all method trajectories and generates publication-ready figures.

Figures produced on Ctrl+C:
  4.1 — Ground truth vs all trajectories with obstacle markers
  4.2 — Localization error over time
  4.3 — Obstacle-event zoomed error (before/during/after)
  4.4 — Bar chart: mean localization error
  4.5 — Bar chart: RMSE comparison
  4.6 — Computational efficiency (memory usage)
  4.7 — Navigation smoothness comparison
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import math, csv, os, time
import numpy as np
import psutil
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.gridspec import GridSpec
from datetime import datetime

from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist


# ── Helpers ───────────────────────────────────────────────────────────────────

def pose_error(x1, y1, x2, y2):
    return math.sqrt((x1-x2)**2 + (y1-y2)**2)

def quaternion_to_yaw(q):
    return math.atan2(2.0*(q.w*q.z+q.x*q.y), 1.0-2.0*(q.y*q.y+q.z*q.z))

def safe_avg(lst):
    v = [x for x in lst if x is not None]
    return float(np.mean(v)) if v else 0.0

def safe_max(lst):
    v = [x for x in lst if x is not None]
    return float(max(v)) if v else 0.0

def safe_rmse(lst):
    v = [x for x in lst if x is not None]
    return float(np.sqrt(np.mean(np.array(v)**2))) if v else 0.0

def pct_below(lst, thr):
    v = [x for x in lst if x is not None]
    return 100.0 * sum(1 for x in v if x < thr) / len(v) if v else 0.0

# ── Style constants ───────────────────────────────────────────────────────────

STYLES = {
    'ground_truth': dict(color='#000000', lw=3,   ls='solid',          label='Ground Truth',      zorder=10),
    'amcl':         dict(color='#E24B4A', lw=1.8, ls='solid',          label='AMCL',              zorder=5),
    'ekf':          dict(color='#2171B5', lw=1.8, ls='dashed',         label='EKF (AMCL)',        zorder=6),
    'eif':          dict(color='#238B45', lw=1.8, ls='dotted',         label='EIF (AMCL)',        zorder=6),
    'rtab':         dict(color='#D94801', lw=1.8, ls=(0,(8,3,2,3)),    label='RTAB-Map',          zorder=4),
    'fused':        dict(color='#6A0DAD', lw=2.2, ls=(0,(5,2)),        label='Fused (EKF+RTAB)',  zorder=7),
    'ekfr':         dict(color='#08519C', lw=1.5, ls=(0,(3,1,1,1)),    label='EKF (RTAB)',        zorder=5),
}

COLORS = {k: v['color'] for k, v in STYLES.items()}

MONITOR_NODES = [
    'ekf_node', 'eif_node', 'rtabmap', 'fusion_node', 'ekf_rtab_node', 'amcl'
]
NODE_LABELS = {
    'ekf_node':      'EKF',
    'eif_node':      'EIF',
    'rtabmap':       'RTAB-Map',
    'fusion_node':   'Fused',
    'ekf_rtab_node': 'EKF-RTAB',
    'amcl':          'AMCL',
}


# ── Node ─────────────────────────────────────────────────────────────────────

class ThesisPlotter(Node):

    def __init__(self):
        super().__init__('thesis_plotter')

        # Ground truth
        self.true_xs, self.true_ys, self.true_yaws = [], [], []

        # Per-method storage
        self.data = {
            m: {'xs': [], 'ys': [], 'errs': []}
            for m in ['amcl', 'ekf', 'eif', 'rtab', 'fused', 'ekfr']
        }

        self.timestamps = []

        # Pose cache
        self.poses = {m: None for m in
                      ['amcl', 'ekf', 'eif', 'rtab', 'fused', 'ekfr', 'true']}

        # GT offset
        self.gt_offset_x = None
        self.gt_offset_y = None

        # Obstacle events
        self.obstacle_times = []
        self.obstacle_xy    = []

        # Smoothness
        self.lin_jerks  = []
        self.ang_jerks  = []
        self.last_v     = 0.0
        self.last_w     = 0.0
        self.last_vt    = None

        # Resources
        self.res = {n: {'cpu': [], 'mem': []} for n in MONITOR_NODES}

        # QoS
        tqos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST, depth=10)

        # Subscriptions
        self.create_subscription(Float64, '/obstacle_spawned_first',
                                 self.obs_cb, 10)
        self.create_subscription(Float64, '/obstacle_spawned_second',
                                 self.obs_cb, 10)
        self.create_subscription(Odometry, '/ekf_pose',
                                 lambda m: self._pose_cb('ekf', m.pose.pose), 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                                 lambda m: self._pose_cb('amcl', m.pose.pose), 10)
        self.create_subscription(Odometry, '/eif_pose',
                                 lambda m: self._pose_cb('eif', m.pose.pose), 10)
        self.create_subscription(Odometry, '/rtabmap_pose',
                                 lambda m: self._pose_cb('rtab', m.pose.pose), 10)
        self.create_subscription(Odometry, '/fused_pose',
                                 lambda m: self._pose_cb('fused', m.pose.pose), 10)
        self.create_subscription(Odometry, '/ekf_rtab_pose',
                                 lambda m: self._pose_cb('ekfr', m.pose.pose), 10)
        self.create_subscription(Odometry, '/odometry/true_pose',
                                 lambda m: self._pose_cb('true', m.pose.pose),
                                 tqos)
        self.create_subscription(Twist, '/cmd_vel_nav', self.vel_cb, 10)

        self.create_timer(0.2, self.record)
        self.create_timer(1.0, self.record_res)

        self.get_logger().info(
            'Thesis plotter started. Press Ctrl+C to save all figures.')

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def obs_cb(self, msg):
        t = self.get_clock().now().nanoseconds * 1e-9
        self.obstacle_times.append(t)
        p = self.poses.get('ekf')
        self.obstacle_xy.append((p.position.x, p.position.y) if p else (None, None))
        self.get_logger().info(
            f'Obstacle #{len(self.obstacle_times)} at t={t:.1f}s')

    def _pose_cb(self, key, pose):
        self.poses[key] = pose
        if key == 'ekf' and self.gt_offset_x is None and self.poses['true']:
            tp = self.poses['true']
            self.gt_offset_x = tp.position.x - pose.position.x
            self.gt_offset_y = tp.position.y - pose.position.y
            self.get_logger().info(
                f'GT offset: dx={self.gt_offset_x:.4f} dy={self.gt_offset_y:.4f}')

    def vel_cb(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        v, w = msg.linear.x, msg.angular.z
        if self.last_vt is not None:
            dt = now - self.last_vt
            if 0 < dt < 1.0:
                self.lin_jerks.append(abs(v - self.last_v) / dt)
                self.ang_jerks.append(abs(w - self.last_w) / dt)
        self.last_v, self.last_w, self.last_vt = v, w, now

    # ── Recording ─────────────────────────────────────────────────────────────

    def record(self):
        if self.poses['true'] is None or self.gt_offset_x is None:
            return

        tp = self.poses['true']
        tx = tp.position.x - self.gt_offset_x
        ty = tp.position.y - self.gt_offset_y

        self.true_xs.append(tx)
        self.true_ys.append(ty)
        self.true_yaws.append(quaternion_to_yaw(tp.orientation))
        self.timestamps.append(self.get_clock().now().nanoseconds * 1e-9)

        for key in ['amcl', 'ekf', 'eif', 'rtab', 'fused', 'ekfr']:
            p = self.poses[key]
            if p is not None:
                self.data[key]['xs'].append(p.position.x)
                self.data[key]['ys'].append(p.position.y)
                self.data[key]['errs'].append(
                    pose_error(p.position.x, p.position.y, tx, ty))
            else:
                self.data[key]['xs'].append(None)
                self.data[key]['ys'].append(None)
                self.data[key]['errs'].append(None)

    def record_res(self):
        for node_name in MONITOR_NODES:
            for proc in psutil.process_iter(['pid', 'cmdline']):
                try:
                    cmd = ' '.join(proc.info['cmdline'] or [])
                    if node_name in cmd:
                        p = psutil.Process(proc.info['pid'])
                        self.res[node_name]['cpu'].append(
                            p.cpu_percent(interval=None))
                        self.res[node_name]['mem'].append(
                            p.memory_info().rss / 1024 / 1024)
                        break
                except Exception:
                    pass

    # ── Generate all figures ──────────────────────────────────────────────────

    def save_all(self):
        if len(self.true_xs) < 5:
            print('Not enough data.')
            return

        ts  = datetime.now().strftime('%Y%m%d_%H%M%S')
        od  = '/ros_ws/csv_files'
        os.makedirs(od, exist_ok=True)
        t0  = self.timestamps[0]
        tr  = [t - t0 for t in self.timestamps]

        print('\nGenerating thesis figures...')

        self._fig41_trajectory(f'{od}/fig41_trajectory_{ts}.png')
        self._fig42_error_time(f'{od}/fig42_error_time_{ts}.png', tr, t0)
        self._fig43_obstacle_zoom(f'{od}/fig43_obstacle_zoom_{ts}.png', tr, t0)
        self._fig44_mean_error(f'{od}/fig44_mean_error_{ts}.png')
        self._fig45_rmse(f'{od}/fig45_rmse_{ts}.png')
        self._fig46_computation(f'{od}/fig46_computation_{ts}.png')
        self._fig47_smoothness(f'{od}/fig47_smoothness_{ts}.png')
        self._save_csv(f'{od}/data_{ts}.csv', tr)
        self._print_summary()

        print(f'\nAll figures saved to {od}/')

    # ── Figure 4.1 — Trajectories ─────────────────────────────────────────────

    def _fig41_trajectory(self, path):
        fig, ax = plt.subplots(figsize=(12, 8))

        # Ground truth
        s = STYLES['ground_truth']
        ax.plot(self.true_xs, self.true_ys,
                color=s['color'], lw=s['lw'], ls=s['ls'],
                label=s['label'], zorder=s['zorder'])

        if self.true_xs:
            ax.scatter(self.true_xs[0], self.true_ys[0],
                       s=150, color='green', marker='o', zorder=15,
                       label='Start')
            ax.scatter(self.true_xs[-1], self.true_ys[-1],
                       s=150, color='blue', marker='*', zorder=15,
                       label='End')

        # Each method
        for key in ['amcl', 'ekf', 'eif', 'rtab', 'fused', 'ekfr']:
            pts = [(x, y) for x, y in
                   zip(self.data[key]['xs'], self.data[key]['ys'])
                   if x is not None]
            if pts:
                xs, ys = zip(*pts)
                s = STYLES[key]
                ax.plot(xs, ys, color=s['color'], lw=s['lw'],
                        ls=s['ls'], label=s['label'], zorder=s['zorder'],
                        alpha=0.85)

        # Obstacle markers
        for i, (ot, (ox, oy)) in enumerate(
                zip(self.obstacle_times, self.obstacle_xy)):
            if ox is not None:
                ax.scatter(ox, oy, s=300, marker='X', color='red',
                           zorder=20, label='Obstacle' if i == 0 else None)
                ax.annotate(f'Obs {i+1}', (ox, oy),
                            textcoords='offset points', xytext=(8, 8),
                            fontsize=9, color='red', fontweight='bold')

        ax.set_xlabel('X (m)', fontsize=13)
        ax.set_ylabel('Y (m)', fontsize=13)
        ax.set_title('Figure 4.1 — Ground Truth vs All Method Trajectories',
                     fontsize=14, fontweight='bold')
        ax.legend(loc='best', fontsize=9, framealpha=0.9)
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal')
        fig.tight_layout()
        fig.savefig(path, dpi=200, bbox_inches='tight')
        plt.close(fig)
        print(f'Fig 4.1 saved: {path}')

    # ── Figure 4.2 — Error over time ──────────────────────────────────────────

    def _fig42_error_time(self, path, tr, t0):
        fig, ax = plt.subplots(figsize=(14, 5))

        for key in ['amcl', 'ekf', 'eif', 'rtab', 'fused', 'ekfr']:
            pairs = [(t, e) for t, e in
                     zip(tr, self.data[key]['errs']) if e is not None]
            if pairs:
                ts_, es_ = zip(*pairs)
                s = STYLES[key]
                ax.plot(ts_, es_, color=s['color'], lw=s['lw'],
                        ls=s['ls'], label=s['label'], alpha=0.9)

        # Obstacle lines
        for i, ot in enumerate(self.obstacle_times):
            otr = ot - t0
            ax.axvline(x=otr, color='red', ls='--', lw=1.5, alpha=0.7)
            ax.text(otr + 1, ax.get_ylim()[1] * 0.92 if ax.get_ylim()[1] > 0
                    else 0.3, f'Obs {i+1}', fontsize=9, color='red')

        ax.axhline(y=0.05, color='green', ls=':', lw=1.2,
                   alpha=0.7, label='5 cm threshold')

        ax.set_xlabel('Time (s)', fontsize=13)
        ax.set_ylabel('Position Error (m)', fontsize=13)
        ax.set_title('Figure 4.2 — Localization Error Over Time',
                     fontsize=14, fontweight='bold')
        ax.legend(loc='upper right', fontsize=9, framealpha=0.9)
        ax.grid(True, alpha=0.3)
        ax.set_ylim(bottom=0)
        fig.tight_layout()
        fig.savefig(path, dpi=200, bbox_inches='tight')
        plt.close(fig)
        print(f'Fig 4.2 saved: {path}')

    # ── Figure 4.3 — Obstacle zoom ────────────────────────────────────────────

    def _fig43_obstacle_zoom(self, path, tr, t0):
        if not self.obstacle_times:
            print('Fig 4.3 skipped — no obstacle events')
            return

        # Use first obstacle event
        obs_t = self.obstacle_times[0] - t0
        win_before = 15.0   # seconds before obstacle
        win_during = 20.0   # seconds after obstacle (during)
        win_after  = 15.0   # seconds for recovery

        t_start = obs_t - win_before
        t_end   = obs_t + win_during + win_after

        fig, axes = plt.subplots(1, 3, figsize=(15, 5),
                                 sharey=True,
                                 gridspec_kw={'width_ratios': [1, 1, 1]})

        phases = [
            ('Before Obstacle', t_start,      obs_t,               axes[0]),
            ('During Obstacle', obs_t,         obs_t + win_during,  axes[1]),
            ('After Obstacle',  obs_t + win_during,  t_end,         axes[2]),
        ]

        for phase_name, t_lo, t_hi, ax in phases:
            for key in ['amcl', 'ekf', 'eif', 'rtab', 'fused', 'ekfr']:
                pairs = [(t, e) for t, e in
                         zip(tr, self.data[key]['errs'])
                         if e is not None and t_lo <= t <= t_hi]
                if pairs:
                    ts_, es_ = zip(*pairs)
                    s = STYLES[key]
                    ax.plot(ts_, es_, color=s['color'], lw=s['lw'],
                            ls=s['ls'], label=s['label'], alpha=0.9)

            ax.axhline(y=0.05, color='green', ls=':', lw=1, alpha=0.7)
            if phase_name == 'During Obstacle':
                ax.axvspan(obs_t, obs_t + win_during,
                           alpha=0.08, color='red', label='Obstacle active')
            ax.set_title(phase_name, fontsize=12, fontweight='bold')
            ax.set_xlabel('Time (s)', fontsize=11)
            ax.grid(True, alpha=0.3)
            ax.set_ylim(bottom=0)

        axes[0].set_ylabel('Position Error (m)', fontsize=11)
        axes[1].legend(loc='upper right', fontsize=8, framealpha=0.9)

        fig.suptitle(
            'Figure 4.3 — Localization Error: Before / During / After Obstacle',
            fontsize=14, fontweight='bold')
        fig.tight_layout()
        fig.savefig(path, dpi=200, bbox_inches='tight')
        plt.close(fig)
        print(f'Fig 4.3 saved: {path}')

    # ── Figure 4.4 — Mean error bar chart ─────────────────────────────────────

    def _fig44_mean_error(self, path):
        methods = ['amcl', 'ekf', 'eif', 'rtab', 'fused', 'ekfr']
        labels  = [STYLES[m]['label'] for m in methods]
        colors  = [STYLES[m]['color'] for m in methods]
        means   = [safe_avg(self.data[m]['errs']) for m in methods]
        stds    = [float(np.std([e for e in self.data[m]['errs']
                                 if e is not None])) for m in methods]

        fig, ax = plt.subplots(figsize=(10, 6))
        x = np.arange(len(methods))
        bars = ax.bar(x, means, color=colors, alpha=0.85,
                      edgecolor='white', linewidth=0.8, width=0.6,
                      yerr=stds, capsize=5, error_kw=dict(lw=1.5))

        for bar, val, std in zip(bars, means, stds):
            ax.text(bar.get_x() + bar.get_width()/2,
                    bar.get_height() + std + 0.003,
                    f'{val:.4f}m', ha='center', va='bottom',
                    fontsize=9, fontweight='bold')

        ax.axhline(y=0.05, color='green', ls=':', lw=1.5,
                   alpha=0.8, label='5 cm threshold')
        ax.set_xticks(x)
        ax.set_xticklabels(labels, fontsize=10, rotation=15, ha='right')
        ax.set_ylabel('Mean Localization Error (m)', fontsize=12)
        ax.set_title('Figure 4.4 — Mean Localization Error Comparison',
                     fontsize=14, fontweight='bold')
        ax.legend(fontsize=10)
        ax.grid(True, axis='y', alpha=0.3)
        ax.set_ylim(bottom=0)
        fig.tight_layout()
        fig.savefig(path, dpi=200, bbox_inches='tight')
        plt.close(fig)
        print(f'Fig 4.4 saved: {path}')

    # ── Figure 4.5 — RMSE bar chart ───────────────────────────────────────────

    def _fig45_rmse(self, path):
        methods = ['amcl', 'ekf', 'eif', 'rtab', 'fused', 'ekfr']
        labels  = [STYLES[m]['label'] for m in methods]
        colors  = [STYLES[m]['color'] for m in methods]
        rmses   = [safe_rmse(self.data[m]['errs']) for m in methods]

        fig, ax = plt.subplots(figsize=(10, 6))
        x = np.arange(len(methods))
        bars = ax.bar(x, rmses, color=colors, alpha=0.85,
                      edgecolor='white', linewidth=0.8, width=0.6)

        for bar, val in zip(bars, rmses):
            ax.text(bar.get_x() + bar.get_width()/2,
                    bar.get_height() + 0.003,
                    f'{val:.4f}m', ha='center', va='bottom',
                    fontsize=9, fontweight='bold')

        ax.set_xticks(x)
        ax.set_xticklabels(labels, fontsize=10, rotation=15, ha='right')
        ax.set_ylabel('RMSE (m)', fontsize=12)
        ax.set_title('Figure 4.5 — RMSE Comparison',
                     fontsize=14, fontweight='bold')
        ax.grid(True, axis='y', alpha=0.3)
        ax.set_ylim(bottom=0)
        fig.tight_layout()
        fig.savefig(path, dpi=200, bbox_inches='tight')
        plt.close(fig)
        print(f'Fig 4.5 saved: {path}')

    # ── Figure 4.6 — Computational efficiency ────────────────────────────────

    def _fig46_computation(self, path):
        nodes  = [n for n in MONITOR_NODES
                  if self.res[n]['mem']]
        labels = [NODE_LABELS.get(n, n) for n in nodes]
        mems   = [safe_avg(self.res[n]['mem']) for n in nodes]
        colors = ['#2171B5','#238B45','#D94801',
                  '#6A0DAD','#08519C','#E24B4A'][:len(nodes)]

        fig, ax = plt.subplots(figsize=(10, 6))
        x = np.arange(len(nodes))
        bars = ax.bar(x, mems, color=colors, alpha=0.85,
                      edgecolor='white', linewidth=0.8, width=0.6)

        for bar, val in zip(bars, mems):
            ax.text(bar.get_x() + bar.get_width()/2,
                    bar.get_height() + 0.3,
                    f'{val:.1f} MB', ha='center', va='bottom',
                    fontsize=9, fontweight='bold')

        ax.set_xticks(x)
        ax.set_xticklabels(labels, fontsize=11)
        ax.set_ylabel('Average Memory Usage (MB)', fontsize=12)
        ax.set_title('Figure 4.6 — Computational Efficiency: Memory Usage per Node',
                     fontsize=14, fontweight='bold')
        ax.grid(True, axis='y', alpha=0.3)
        ax.set_ylim(bottom=0)
        fig.tight_layout()
        fig.savefig(path, dpi=200, bbox_inches='tight')
        plt.close(fig)
        print(f'Fig 4.6 saved: {path}')

    # ── Figure 4.7 — Navigation smoothness ───────────────────────────────────

    def _fig47_smoothness(self, path):
        if not self.lin_jerks:
            print('Fig 4.7 skipped — no velocity data')
            return

        lj = np.array(self.lin_jerks)
        aj = np.array(self.ang_jerks) if self.ang_jerks else np.array([0])

        fig, axes = plt.subplots(1, 2, figsize=(14, 5))

        # Left — jerk over time
        ax = axes[0]
        t_jerk = np.linspace(0, len(lj) * 0.05, len(lj))
        ax.plot(t_jerk, lj, color='#2171B5', lw=1.2,
                label='Linear jerk', alpha=0.8)
        ax.plot(np.linspace(0, len(aj)*0.05, len(aj)), aj,
                color='#E24B4A', lw=1.2, label='Angular jerk', alpha=0.8)

        for ot in self.obstacle_times:
            ax.axvline(x=ot - self.timestamps[0] if self.timestamps else ot,
                       color='red', ls='--', lw=1.2, alpha=0.6)

        ax.set_xlabel('Time (s)', fontsize=12)
        ax.set_ylabel('Jerk (m/s² or rad/s²)', fontsize=12)
        ax.set_title('Velocity Jerk Over Time', fontsize=12, fontweight='bold')
        ax.legend(fontsize=10)
        ax.grid(True, alpha=0.3)
        ax.set_ylim(bottom=0)

        # Right — summary bar chart
        ax2 = axes[1]
        metrics = ['Lin jerk\nmean', 'Lin jerk\nmax',
                   'Ang jerk\nmean', 'Ang jerk\nmax']
        values  = [float(np.mean(lj)), float(np.max(lj)),
                   float(np.mean(aj)), float(np.max(aj))]
        bar_colors = ['#2171B5','#6BAED6','#E24B4A','#FC8D59']
        bars = ax2.bar(range(4), values, color=bar_colors,
                       alpha=0.85, edgecolor='white', width=0.6)

        for bar, val in zip(bars, values):
            ax2.text(bar.get_x() + bar.get_width()/2,
                     bar.get_height() + 0.3,
                     f'{val:.2f}', ha='center', va='bottom',
                     fontsize=9, fontweight='bold')

        ax2.set_xticks(range(4))
        ax2.set_xticklabels(metrics, fontsize=10)
        ax2.set_ylabel('Value (m/s² or rad/s²)', fontsize=12)
        ax2.set_title('Smoothness Summary\n(lower = smoother)',
                      fontsize=12, fontweight='bold')
        ax2.grid(True, axis='y', alpha=0.3)
        ax2.set_ylim(bottom=0)

        fig.suptitle('Figure 4.7 — Navigation Smoothness Analysis',
                     fontsize=14, fontweight='bold')
        fig.tight_layout()
        fig.savefig(path, dpi=200, bbox_inches='tight')
        plt.close(fig)
        print(f'Fig 4.7 saved: {path}')

    # ── Save CSV ──────────────────────────────────────────────────────────────

    def _save_csv(self, path, tr):
        fmt = lambda v: '' if v is None else round(v, 6)
        with open(path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['time_s', 'true_x', 'true_y',
                        'amcl_x', 'amcl_y', 'amcl_err',
                        'ekf_x',  'ekf_y',  'ekf_err',
                        'eif_x',  'eif_y',  'eif_err',
                        'rtab_x', 'rtab_y', 'rtab_err',
                        'fused_x','fused_y','fused_err',
                        'ekfr_x', 'ekfr_y', 'ekfr_err'])
            for i in range(len(self.timestamps)):
                row = [round(tr[i], 4),
                       fmt(self.true_xs[i]), fmt(self.true_ys[i])]
                for m in ['amcl','ekf','eif','rtab','fused','ekfr']:
                    row += [fmt(self.data[m]['xs'][i]),
                            fmt(self.data[m]['ys'][i]),
                            fmt(self.data[m]['errs'][i])]
                w.writerow(row)
            if self.obstacle_times:
                t0 = self.timestamps[0]
                w.writerow([])
                w.writerow(['=== OBSTACLE EVENTS ==='])
                for i, ot in enumerate(self.obstacle_times):
                    w.writerow([f'obstacle_{i+1}', round(ot-t0, 3)])
        print(f'CSV saved: {path}')

    # ── Print summary ─────────────────────────────────────────────────────────

    def _print_summary(self):
        sep = '=' * 65
        print(f'\n{sep}')
        print('  FINAL RESULTS SUMMARY')
        print(sep)
        print(f'{"Method":<22} {"Avg(m)":>8} {"Max(m)":>8} '
              f'{"RMSE(m)":>8} {"<5cm%":>7}')
        print('-' * 65)
        for key in ['amcl', 'ekf', 'eif', 'rtab', 'fused', 'ekfr']:
            errs = self.data[key]['errs']
            print(f'{STYLES[key]["label"]:<22} '
                  f'{safe_avg(errs):>8.4f} '
                  f'{safe_max(errs):>8.4f} '
                  f'{safe_rmse(errs):>8.4f} '
                  f'{pct_below(errs, 0.05):>6.1f}%')
        print(sep)
        if self.lin_jerks:
            lj = np.array(self.lin_jerks)
            aj = np.array(self.ang_jerks) if self.ang_jerks else np.array([0])
            print(f'\nSmoothhness:')
            print(f'  Linear jerk mean:  {np.mean(lj):.4f} m/s²')
            print(f'  Angular jerk mean: {np.mean(aj):.4f} rad/s²')
        print(sep + '\n')


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    node = ThesisPlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save_all()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()