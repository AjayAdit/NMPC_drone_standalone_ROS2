# quad_nmpc_ros

Trajectory-tracking NMPC for a quadrotor, as ROS 2 packages.

| package | language | what it is |
|---|---|---|
| `quad_nmpc_ros` | Python | simulator + controller. The reference implementation. |
| `quad_nmpc_ros_cpp` | C++ | the controller and its ROS wrapper -- everything that would run on the aircraft. Ships a copy of the Python simulator as a test fixture. |

Both speak the same two topics, so either controller drives either simulator.
Measured over 60 s against the same simulator, they agree to three decimals:
**0.223 cm mean tracking error from both.**

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
export ACADOS_SOURCE_DIR=$HOME/acados          # needed by the C++ package
export LD_LIBRARY_PATH=$ACADOS_SOURCE_DIR/lib:$LD_LIBRARY_PATH
python -m colcon build --symlink-install
source install/setup.bash
```

`python -m colcon`, not bare `colcon`. The generated node launchers inherit the
shebang of whichever interpreter runs colcon -- run the system one and the nodes
start under a Python that cannot import mujoco.

Build **everything** at least once. `--packages-select` does not refresh the
top-level `install/setup.bash`, so a newly added package stays invisible to
`ros2 run` until a full build regenerates it.

## Run

Every terminal needs the same four lines first:

```bash
conda activate All_Drones                      # whichever env owns rclpy + mujoco
export ACADOS_SOURCE_DIR=$HOME/acados
export LD_LIBRARY_PATH=$ACADOS_SOURCE_DIR/lib:$LD_LIBRARY_PATH
source ~/drone_ws/install/setup.bash
```

`conda activate` BEFORE `source`, never after.

### Python controller

```bash
# terminal 1 — wait for "NMPC ready" before starting the sim
ros2 run quad_nmpc_ros nmpc_node

# terminal 2
ros2 run quad_nmpc_ros sim_node
```

### C++ controller

```bash
# terminal 1 — ready in milliseconds; the solver is already compiled
ros2 run quad_nmpc_ros_cpp nmpc_node

# terminal 2
ros2 run quad_nmpc_ros_cpp sim_node
```

### Mixed, which is the interesting one

Same topics, so the C++ controller flies the Python package's simulator:

```bash
ros2 run quad_nmpc_ros_cpp nmpc_node      # terminal 1
ros2 run quad_nmpc_ros sim_node           # terminal 2
```

Stop after a fixed number of SIM seconds instead of Ctrl-C:

```bash
ros2 run quad_nmpc_ros_cpp nmpc_node --ros-args -p duration:=60.0
ros2 run quad_nmpc_ros     sim_node  --ros-args -p duration:=64.0
```

Before starting anything, check nothing is already publishing motor commands.
Two controllers on `/motor_cmd` fight over four motors and the drone flies
badly for no visible reason:

```bash
pgrep -af nmpc_node        # must print nothing
```

A MuJoCo window opens and the drone traces a circle, leaving a green trail, with
its nose pointing along the direction of travel. Terminal 1 prints the tracking
error once a second. The flight is logged to `~/.quad_nmpc_ros/tracking_log.csv`
as it flies; render it any time with `ros2 run quad_nmpc_ros plot_log`.

> The Python `nmpc_node` spends ~30 s on its first run compiling the acados
> solver to C, cached in `~/.cache/quad_nmpc`. The C++ one never does: its
> solver is compiled into the executable at build time.

## Regenerating the C++ solver

`quad_nmpc_ros_cpp/acados_generated/` holds 528 KB of C emitted by acados, and
it is **committed on purpose**. acados's code generator is `acados_template`,
which is Python-only, so the C++ package can link the generated solver but
cannot produce it. Committing it is what lets a clean clone build without
running the Python node first.

The cost is that it goes stale the moment the model or the cost weights change
in `nmpc_controller.py`. After any such change, run exactly this:

```bash
cd ~/drone_ws/src/quad_nmpc_ros/quad_nmpc_ros_cpp
./scripts/regenerate.sh
cd ~/drone_ws && python -m colcon build --symlink-install
git add quad_nmpc_ros_cpp/acados_generated && git commit -m "Regenerate the acados solver"
```

Nothing checks this for you. Change the Python cost and skip the regenerate and
the two controllers will quietly disagree.

Inspect the graph while it flies:

```bash
ros2 topic list
ros2 topic hz /odom      # ~100
rqt_graph
```

## Porting notes

Three things the Python interface does silently that the C API does not. Each
one costs a day if you have to find it twice.

**The solver's online parameters are not zeroed.** The model carries 3 of them
(a target-acceleration feedforward this build never uses). `acados_create`
leaves them uninitialised, so the dynamics integrate garbage and every solve
returns `ACADOS_MINSTEP`. `quad_13state_drone_acados_update_params` for every
stage, once at construction.

**The horizon is not seeded.** `nlp_out` starts at all zeros, and an all-zero
quaternion is not a rotation. Seed every node's `x` with the identity pose
(zeros, `qw = 1`).

**Seeding `x` alone is not enough.** With the inputs left at zero the predicted
trajectory is a free fall, which runs straight into the `z >= -0.4` state bound
and hands HPIPM an infeasible QP -- status 4, with `u = 0` coming back out.
Seed `u` at hover for nodes `0 .. N-1` as well. The Python does both, at
`nmpc_controller.py` lines 93-96; it is two loops that are easy to skim past.

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
