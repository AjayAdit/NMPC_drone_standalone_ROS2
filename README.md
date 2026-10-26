# quad_nmpc_ros

Trajectory-tracking NMPC for a quadrotor, as ROS 2 packages.

| package | language | what it is |
|---|---|---|
| `quad_nmpc_ros` | Python | simulator + controller. The reference implementation. |
| `quad_nmpc_ros_cpp` | C++ | the controller and its ROS wrapper -- everything that would run on the aircraft. Ships a copy of the Python simulator as a test fixture. |
| `quadrotor_msgs` | msgs | `TRPYCommand` (thrust + body rates) and `PositionCommand`. |
| `mujoco_ros_utils` | C++ | MuJoCo plugins, vendored from [MujocoRosUtils](https://github.com/isri-aist/MujocoRosUtils). Provides **AcroMode**, the body-rate loop the C++ pipeline flies through. |

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

Exact versions of everything below, with install commands and porting notes:
**[DEPENDENCIES.md](DEPENDENCIES.md)**.

- ROS 2 Humble, with `nav_msgs` and `actuator_msgs`
- Python 3.10 with `mujoco`, `casadi`, `numpy`, `acados_template`
- [acados](https://docs.acados.org) built, with `ACADOS_SOURCE_DIR` set and
  `$ACADOS_SOURCE_DIR/lib` on `LD_LIBRARY_PATH`
- `matplotlib` (only needed by `plot_log`, which renders the flight log afterwards)
- For `mujoco_ros_utils` (the AcroMode plugin):
  - the **pip** `mujoco` package (tested with 3.6.0). It ships the C headers and
    `libmujoco.so.X.Y.Z`; the plugin is built against that exact copy, so it must
    be the same `mujoco` the simulator imports
  - GLFW 3.3: `sudo apt install libglfw3-dev`
  - ROS 2 `std_srvs`, `tf2_ros`, `visualization_msgs`
  - in a conda env, Humble's message generators need `pip install "empy==3.3.4" lark`.
    empy 4 fails with `module 'em' has no attribute 'BUFFERED_OPT'`

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
# the AcroMode plugin builds against the pip MuJoCo the simulator imports
MUJOCO_ROOT_DIR=$(python -c "import mujoco, os; print(os.path.dirname(mujoco.__file__))")
python -m colcon build --symlink-install \
  --cmake-args -DMUJOCO_ROOT_DIR=$MUJOCO_ROOT_DIR -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

`-DMUJOCO_ROOT_DIR` is required by `mujoco_ros_utils` only; the other packages
ignore it (CMake prints an "unused variable" note). Leave out
`-DCMAKE_BUILD_TYPE` and the install step prints a harmless
`CMake Error: Invalid value used with --config` from `cmake/uninstall.cmake`.

Check that the plugin links to the same MuJoCo the simulator imports:

```bash
ldd install/mujoco_ros_utils/lib/libMujocoRosUtilsPlugin.so | grep libmujoco
# libmujoco.so.3.6.0 => .../site-packages/mujoco/libmujoco.so.3.6.0
```

Two different copies means AcroMode registers in the wrong one and the world
fails with `unrecognized plugin 'MujocoRosUtils::AcroMode'`.

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

### C++ controller, through AcroMode (default)

The NMPC's output goes through a body-rate loop, like a real flight controller
in acro mode:

```
nmpc_node ─/motor_cmd + /nmpc/body_rate_ref─> trpy_converter_node ─/quadrotor/trpy_cmd─> AcroMode (in sim_node) ─> thrust + 3 torques
```

```bash
# terminal 1 — ready in milliseconds; the solver is already compiled
ros2 run quad_nmpc_ros_cpp nmpc_node

# terminal 2 — u0..u3 + body rates -> TRPYCommand
ros2 run quad_nmpc_ros_cpp trpy_converter_node

# terminal 3 — loads the AcroMode plugin and worlds/bare_world_acro.xml
ros2 run quad_nmpc_ros_cpp sim_node
```

Or all 3 in one terminal (sim starts 4 s later; `duration` > 0 ends the whole launch):

```bash
ros2 launch quad_nmpc_ros_cpp acro.launch.py
ros2 launch quad_nmpc_ros_cpp acro.launch.py duration:=60.0 log_file:=/tmp/log.csv
```

Without the converter nothing reaches AcroMode: the drone holds m·g and level
attitude, but drifts with no position control.

### C++ controller, motors direct (old pipeline)

```bash
ros2 run quad_nmpc_ros_cpp nmpc_node                                  # terminal 1
ros2 run quad_nmpc_ros_cpp sim_node --ros-args -p acro_mode:=false    # terminal 2
```

Both pipelines reach the same 1.4 mm steady error on the circle; see
`quad_nmpc_ros_cpp/post_analysis/comparison.md`.

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

## Run on ROS 2 Kilted (Docker)

The same code, built from GitHub on ROS 2 Kilted / Ubuntu 24.04. Needs Docker
and, for the viewer on an NVIDIA GPU, `nvidia-container-toolkit`.

Build the image once (about 5 min), and again after every push you want to test:

```bash
cd ~/"ACP DR/drone_ws/src/quad_nmpc_ros"
docker build -t nmpc-kilted --build-arg REPO_REF=$(git rev-parse --short HEAD) docker/kilted
```

1. Open the container, from the repo folder:
   ```bash
   cd ~/"ACP DR/drone_ws/src/quad_nmpc_ros"
   docker/kilted/run.sh
   ```
2. Source the workspace (inside the container):
   ```bash
   source /opt/ros/kilted/setup.bash
   source /ws/install/setup.bash
   ```
3. Fly the C++ NMPC through AcroMode for 60 s; the launch ends by itself:
   ```bash
   mkdir -p /results/acro
   ros2 launch quad_nmpc_ros_cpp acro.launch.py duration:=60.0 log_file:=/results/acro/log.csv
   ```
4. Plot expected vs actual, with the max and mean error:
   ```bash
   python3 /ws/src/quad_nmpc_ros/quad_nmpc_ros_cpp/scripts/plot_track.py /results/acro/log.csv
   ```

`/results` in the container is `./kilted_results` on the host, so the log and
`log_track.png` are in `kilted_results/acro/`. The container itself is deleted
on `exit`. The files belong to root; `sudo chown -R $USER kilted_results` once
to edit them on the host.

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


## Licence

Apache-2.0
