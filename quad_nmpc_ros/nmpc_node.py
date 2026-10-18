import csv
import os
import signal

import mujoco
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

from actuator_msgs.msg import Actuators
from nav_msgs.msg import Odometry

from .nmpc_controller import NMPC
from .paths import default_log_path

R        = 5.0     # m     circle radius
ALTITUDE = 1.5     # m     flight height
OMEGA    = 0.5     # rad/s about 1 m/s along the circle


def circle(t):
    """Where the drone should be at time t, and how fast it should be moving.

    The velocity is the exact derivative of the position. Feeding zeros makes
    the drone lag the circle badly — the controller uses it to know the
    reference is moving rather than teleporting. Yaw here is heading tangent to the circle
    """
    pos = np.array([R * np.cos(OMEGA * t),
                    R * np.sin(OMEGA * t),
                    ALTITUDE])
    vel = np.array([-R * OMEGA * np.sin(OMEGA * t),
                     R * OMEGA * np.cos(OMEGA * t),
                     0.0])
    yaw = OMEGA*t+np.pi/2
    return pos, vel , yaw

class NmpcNode(Node):
    """Subscribes to the drone's state, publishes motor commands.
    All the maths lives in NMPC and circle(); this node only plumbs."""

    def __init__(self):
        super().__init__('nmpc_node')

        # the controller reads mass, inertia, arm length and k_tau off the XML,
        # so this node loads the model too. It never steps it.
        share_dir  = get_package_share_directory('quad_nmpc_ros')
        world_path = os.path.join(share_dir, 'worlds', 'bare_world.xml')
        model = mujoco.MjModel.from_xml_path(world_path)

        self.get_logger().info('building the NMPC (acados compiles on first run)...')
        self.ctrl = NMPC.from_mujoco_model(model)
        self.get_logger().info('NMPC ready')

        # horizon reference buffers, filled fresh on every odom message
        N = self.ctrl.N
        self.positions  = np.zeros((N + 1, 3))
        self.velocities = np.zeros((N + 1, 3))
        self.yaws       = np.zeros(N + 1)          # heading, tangent to the path

        self.t0 = None          # sim time of the first odom message
        self._turns = None  # double-cover offset, locked on the first odom
        self._next_report = 0.0

        # The flight log is written to disk AS WE FLY, one row per odometry
        # message. Nothing about it depends on a clean shutdown: whatever is on
        # disk when the node dies is valid data. Render it afterwards with
        #     ros2 run quad_nmpc_ros plot_log
        self.declare_parameter('log_file', '')
        self.log_path = (self.get_parameter('log_file').value
                         or default_log_path())
        os.makedirs(os.path.dirname(self.log_path), exist_ok=True)
        self._log_fh  = open(self.log_path, 'w', newline='')
        self._log_csv = csv.writer(self._log_fh)
        self._log_csv.writerow(
            ['t', 'x', 'y', 'z', 'ref_x', 'ref_y', 'ref_z', 'err_cm'])
        self._rows = 0
        self.get_logger().info(f'logging to {self.log_path}')

        # stop after this many SIM seconds. 0 (the default) runs until Ctrl-C.
        # If you set it, set sim_node's to match so both end together.
        self.declare_parameter('duration', 0.0)
        self.duration = float(self.get_parameter('duration').value)

        self.cmd_pub = self.create_publisher(Actuators, '/motor_cmd', 10)

        # no timer: odom arriving IS the clock
        self._odom_sub = self.create_subscription(
            Odometry, '/odom', self.on_odom, 10)

    def on_odom(self, msg):
        # --- sim time, phase-zeroed on the first message we ever see ---
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.t0 is None:
            self.t0 = t
        t -= self.t0

        # --- 13-vector: [pos(3), vel(3), quat(4), omega(3)] ---
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        v = msg.twist.twist.linear
        w = msg.twist.twist.angular
        x_meas = np.array([p.x, p.y, p.z,
                           v.x, v.y, v.z,
                           q.w, q.x, q.y, q.z,     # ROS (x,y,z,w) -> (w,x,y,z)
                           w.x, w.y, w.z])

        # --- where the circle will be across the whole horizon ---
        if self._turns is None:
            psi_meas = np.arctan2(2.0 * (q.w * q.z + q.x * q.y),
                                  1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            _, _, psi0 = circle(t)
            self._turns = 2.0 * np.pi * round((psi_meas - psi0) / (2.0 * np.pi))

        N, dt = self.ctrl.N, self.ctrl.dt

        for k in range(N + 1):
            self.positions[k], self.velocities[k], psi = circle(t + k * dt)
            self.yaws[k] = psi + self._turns

        self.ctrl.set_target_trajectory(self.positions, self.velocities, self.yaws)
        u = self.ctrl.compute_action(x_meas, None)

        cmd = Actuators()
        cmd.header.stamp = msg.header.stamp
        cmd.normalized = u.tolist()
        self.cmd_pub.publish(cmd)

        ref = self.positions[0]
        err_cm = float(np.linalg.norm(ref - np.array([p.x, p.y, p.z]))) * 100.0
        self._log_csv.writerow([f'{t:.4f}',
                                f'{p.x:.6f}', f'{p.y:.6f}', f'{p.z:.6f}',
                                f'{ref[0]:.6f}', f'{ref[1]:.6f}', f'{ref[2]:.6f}',
                                f'{err_cm:.4f}'])
        self._rows += 1
        if self._rows % 50 == 0:            # ~twice a second, costs nothing
            self._log_fh.flush()

        if t >= self._next_report:
            want = self.positions[0]
            err  = float(np.linalg.norm(want - np.array([p.x, p.y, p.z])))
            self.get_logger().info(
                f't={t:5.1f}s  err={err * 100:6.1f} cm  '
                f'pos=({p.x:+.2f},{p.y:+.2f},{p.z:+.2f})')
            self._next_report += 1.0

        if self.duration > 0.0 and t >= self.duration:
            self.get_logger().info(
                f'reached duration {self.duration:.1f} s, stopping')
            raise SystemExit

    def close_log(self):
        """Flush and close the CSV. Best effort: the file is already usable."""
        try:
            self._log_fh.flush()
            self._log_fh.close()
        except Exception:
            pass
        print(f'[nmpc_node] flight log: {self.log_path}', flush=True)
        print('[nmpc_node] plot it with:  ros2 run quad_nmpc_ros plot_log',
              flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = NmpcNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # a second Ctrl-C while we are writing the plot must not kill us
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.close_log()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
