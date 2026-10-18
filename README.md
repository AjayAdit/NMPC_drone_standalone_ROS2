# quad_nmpc_ros

Trajectory-tracking NMPC for a quadrotor, as a ROS 2 package.

Two nodes: a MuJoCo simulator and an NMPC controller, talking over standard
ROS topics. The controller knows nothing about MuJoCo, and the simulator knows
nothing about the trajectory — swap either one out and the other still works.

The demo flies a 5 m circle at 2.5 m/s with the nose held tangent to the path,
and holds **2 mm** of mean tracking error over 32 laps.

## Architecture

```
         /odom  (nav_msgs/Odometry)
sim_node ──────────────────────────> nmpc_node
    ^                                    │
    └────────────────────────────────────┘
         /motor_cmd  (actuator_msgs/Actuators)
```

| node | owns | publishes | subscribes |
|---|---|---|---|
| `sim_node` | the MuJoCo world and the viewer | `/odom` at 100 Hz | `/motor_cmd` |
| `nmpc_node` | the acados OCP and the reference | `/motor_cmd` | `/odom` |

Physics runs at 500 Hz (5 `mj_step` calls per 100 Hz timer tick). The NMPC
solves once per odometry message, so 100 Hz. The drone is any quadrotor
described by the world XML — mass, inertia, arm length and thrust constant are
read off the model at startup, nothing is hard-coded.

## Requirements

- ROS 2 Humble, with `nav_msgs` and `actuator_msgs`
- Python 3.10 with `mujoco`, `casadi`, `numpy`, `acados_template`
- [acados](https://docs.acados.org) built, with `ACADOS_SOURCE_DIR` set and
  `$ACADOS_SOURCE_DIR/lib` on `LD_LIBRARY_PATH`
- `matplotlib` (only needed by `plot_log`, which renders the flight log afterwards)

All of the above must be importable by **one** interpreter. If you use conda,
install `colcon-common-extensions` into that same env:

```bash
pip install -U colcon-common-extensions
```

## Build

```bash
mkdir -p ~/drone_ws/src && cd ~/drone_ws/src
git clone <this repo>
cd ~/drone_ws
python -m colcon build --symlink-install
source install/setup.bash
```

`python -m colcon`, not bare `colcon`. The generated node launchers inherit the
shebang of whichever interpreter runs colcon — run the system one and the nodes
start under a Python that cannot import mujoco.

## Run

Two terminals, both with the workspace sourced.

```bash
# terminal 1 — wait for "NMPC ready" before starting the sim
ros2 run quad_nmpc_ros nmpc_node

# terminal 2
ros2 run quad_nmpc_ros sim_node
```

A MuJoCo window opens and the drone traces a circle, leaving a green trail, with
its nose pointing along the direction of travel. Terminal 1 prints the tracking
error once a second. The flight is logged to `~/.quad_nmpc_ros/tracking_log.csv`
as it flies; render it any time with `ros2 run quad_nmpc_ros plot_log`.

> The first run of `nmpc_node` spends ~30 s compiling the acados solver to C.
> It is cached in `~/.cache/quad_nmpc` and every later run starts instantly.

Inspect the graph while it flies:

```bash
ros2 topic list
ros2 topic hz /odom      # ~100
rqt_graph
```

## Conventions

Two places where ROS and MuJoCo disagree, both handled at the message boundary:

- **Quaternions.** ROS is scalar-last `(x,y,z,w)`; MuJoCo and the controller are
  scalar-first `(w,x,y,z)`. Reordered in `sim_node.publish_odom` and
  `nmpc_node.on_odom`.
- **Twist frames.** `Odometry` says everything in `twist` is in `child_frame_id`.
  MuJoCo's free joint gives linear velocity in the **world** frame and angular
  velocity in the **body** frame, which is what the controller wants, so both are
  passed through unchanged. Linear velocity therefore deviates from the ROS
  convention.

- **Quaternion double cover.** The yaw reference is turned into a quaternion as
  `(cos(psi/2), sin(psi/2))`, so `psi` and `psi + 2*pi` are the same heading but
  *opposite* quaternions, and the solver's quadratic cost reads that sign flip as
  a 180 degree error. Because the reference heading climbs without bound and the
  measured yaw comes back wrapped, the two need a constant `2*pi` offset to agree.
  `nmpc_node` locks that offset on the first odometry message and never
  recomputes it. Recomputing it per tick puts a `round()` in the loop, and
  `round()` steps once per lap -- which negates the reference quaternion, once
  every revolution, forever. That failure is silent for about fourteen laps and
  then drops the drone out of the sky.

Odometry is stamped with MuJoCo's `data.time`, not the wall clock. The reference
trajectory is a function of that stamp, so the reference can never drift ahead of
the physics when the simulator runs slower than real time.

## Licence

Apache-2.0
