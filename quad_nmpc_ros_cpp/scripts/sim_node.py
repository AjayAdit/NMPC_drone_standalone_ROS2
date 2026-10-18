#!/usr/bin/env python3
# Installed as a plain program, not a setuptools console_script, so it
# needs its own shebang. env python3 resolves to whichever interpreter is
# first on PATH -- activate the conda env that owns rclpy and mujoco before
# running, exactly as for quad_nmpc_ros.
#
# A COPY of quad_nmpc_ros/sim_node.py, so this package stands alone. The
# simulator is a test fixture, not flight software, which is why it stays
# in Python while the controller does not.
import rclpy
from rclpy.node import Node
import mujoco.viewer


import os

import mujoco
import numpy as np
from ament_index_python.packages import get_package_share_directory

from nav_msgs.msg import Odometry
from actuator_msgs.msg import Actuators

PHYSICS_PER_TICK = 5
TRAIL_EVERY = 10      # timer ticks between breadcrumbs -> 10 Hz
TRAIL_RGBA  = (0.1, 0.9, 0.2, 1.0)
TRAIL_SIZE  = 0.02    # m, sphere radius
OMEGA    = 0.0     # hover test — reference is a fixed point

class SimNode(Node):

    """One MuJoCo world, stepped by a ROS timer.Publishes the drone's true state, subscribes to motor commands."""
    def __init__(self):
        super().__init__('sim_node')
        share_dir  = get_package_share_directory('quad_nmpc_ros_cpp')
        world_path = os.path.join(share_dir, 'worlds', 'bare_world.xml')

        self.model = mujoco.MjModel.from_xml_path(world_path)
        self.data  = mujoco.MjData(self.model)

        # 4  full-throttle thrust per motor, in Newtons.
        #    normalized u in [0,1]  ->  data.ctrl = k_tau * u**2
        self.k_tau = float(self.model.actuator_ctrlrange[0, 1])


        # 5  put the drone where the circle starts, before the first step
        self.joint = self.data.joint('drone_root')
        self.joint.qpos[0:3] = [5.0, 0.0, 1.5]
        # facing +y: at t=0 the circle's tangent is +90 deg, so spawning at
        # identity would hand the solver a 90 deg step on the very first tick.
        self.joint.qpos[3:7] = [0.70710678, 0.0, 0.0, 0.70710678]   # MuJoCo order: w x y z
        mujoco.mj_forward(self.model, self.data)


        # 6  last motor command received; step() runs before nmpc_node speaks
        self.u = np.zeros(4)

        # 7  the drone's true state, out
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # 8  motor commands, in.  Keep the handle or it gets collected.
        self._cmd_sub = self.create_subscription(
            Actuators, '/motor_cmd', self.on_motor_cmd, 10)

        # 9  the viewer. No `with` — we spin, we don't loop.
        self.viewer = mujoco.viewer.launch_passive(self.model, self.data)

        self._tick = 0
        self.viewer.user_scn.ngeom = 0

        # how long to fly, in SIM seconds. 0 (the default) runs until Ctrl-C.
        #   ros2 run quad_nmpc_ros sim_node --ros-args -p duration:=60.0
        self.declare_parameter('duration', 0.0)
        self.duration = float(self.get_parameter('duration').value)

        # 10 LAST: 5 physics steps per tick -> 500 Hz physics, 100 Hz odom
        self.create_timer(self.model.opt.timestep * PHYSICS_PER_TICK, self.step)


    def on_motor_cmd(self,msg):
        # 11 hold the command; the timer applies it
        self.u = np.asarray(msg.normalized, dtype=float)

    def step(self):
        # 12a normalized [0,1] -> Newtons
        self.data.ctrl[:] = self.k_tau * self.u ** 2

        # 12b advance physics
        for _ in range(PHYSICS_PER_TICK):
            mujoco.mj_step(self.model, self.data)

        self._tick += 1
        if self._tick % TRAIL_EVERY == 0:
            self.add_breadcrumb()

        # 12c + 12d
        self.publish_odom() # publish odom 
        self.viewer.sync()

        # stop by ourselves so the drone never drifts off after the demo
        if self.duration > 0.0 and self.data.time >= self.duration:
            self.get_logger().info(
                f'reached duration {self.duration:.1f} s, stopping')
            raise SystemExit

    def publish_odom(self):
        qpos = self.joint.qpos
        qvel = self.joint.qvel

        msg = Odometry()
        # sim time, not wall time: the reference must track the physics clock
        sim_t = self.data.time
        msg.header.stamp.sec     = int(sim_t)
        msg.header.stamp.nanosec = int((sim_t - int(sim_t)) * 1e9)
        msg.header.frame_id = 'world'
        msg.child_frame_id = 'base_link'

        msg.pose.pose.position.x = qpos[0]
        msg.pose.pose.position.y = qpos[1]
        msg.pose.pose.position.z = qpos[2]

        # MuJoCo stores (w,x,y,z); ROS wants (x,y,z,w)
        msg.pose.pose.orientation.w = qpos[3]
        msg.pose.pose.orientation.x = qpos[4]
        msg.pose.pose.orientation.y = qpos[5]
        msg.pose.pose.orientation.z = qpos[6]

        msg.twist.twist.linear.x = qvel[0]
        msg.twist.twist.linear.y = qvel[1]
        msg.twist.twist.linear.z = qvel[2]

        msg.twist.twist.angular.x = qvel[3]
        msg.twist.twist.angular.y = qvel[4]
        msg.twist.twist.angular.z = qvel[5]

        self.odom_pub.publish(msg)

    def add_breadcrumb(self):
        """Drop one sphere at the drone's current position, into the viewer's
        user scene. O(1): geoms accumulate, we never redraw the whole trail."""
        scn = self.viewer.user_scn
        if scn.ngeom >= scn.maxgeom:
            return                              # trail buffer full, stop growing
        mujoco.mjv_initGeom(
            scn.geoms[scn.ngeom],
            type=mujoco.mjtGeom.mjGEOM_SPHERE,
            size=np.array([TRAIL_SIZE, 0.0, 0.0]),
            pos=np.asarray(self.joint.qpos[0:3], dtype=float),
            mat=np.eye(3).flatten(),
            rgba=np.array(TRAIL_RGBA),
        )
        scn.ngeom += 1


def main(args=None):
    rclpy.init(args=args)
    node = SimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.viewer.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__=='__main__': main()




