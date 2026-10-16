import os

import mujoco
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

from actuator_msgs.msg import Actuators
from nav_msgs.msg import Odometry

from .nmpc_controller import NMPC

R        = 2.0     # m     circle radius
ALTITUDE = 1.5     # m     flight height
OMEGA    = 0.5     # rad/s about 1 m/s along the circle


def circle(t):
    """Where the drone should be at time t, and how fast it should be moving.

    The velocity is the exact derivative of the position. Feeding zeros makes
    the drone lag the circle badly — the controller uses it to know the
    reference is moving rather than teleporting.
    """
    pos = np.array([R * np.cos(OMEGA * t),
                    R * np.sin(OMEGA * t),
                    ALTITUDE])
    vel = np.array([-R * OMEGA * np.sin(OMEGA * t),
                     R * OMEGA * np.cos(OMEGA * t),
                     0.0])
    return pos, vel

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
        self.yaws       = np.zeros(N + 1)        # hold heading

        self.t0 = None          # sim time of the first odom message
        self._next_report = 0.0

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
        N, dt = self.ctrl.N, self.ctrl.dt
        for k in range(N + 1):
            self.positions[k], self.velocities[k] = circle(t + k * dt)

        self.ctrl.set_target_trajectory(self.positions, self.velocities, self.yaws)
        u = self.ctrl.compute_action(x_meas, None)

        cmd = Actuators()
        cmd.header.stamp = msg.header.stamp
        cmd.normalized = u.tolist()
        self.cmd_pub.publish(cmd)

        if t >= self._next_report:
            want = self.positions[0]
            err  = float(np.linalg.norm(want - np.array([p.x, p.y, p.z])))
            self.get_logger().info(
                f't={t:5.1f}s  err={err * 100:6.1f} cm  '
                f'pos=({p.x:+.2f},{p.y:+.2f},{p.z:+.2f})')
            self._next_report += 1.0
    

def main(args=None):
    rclpy.init(args=args)
    node = NmpcNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
