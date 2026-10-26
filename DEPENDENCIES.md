# Dependencies

Everything this repo needs that is NOT in the repo, with the exact versions it
was built and flown with. Porting to another Ubuntu or ROS 2 release: find the
nearest equivalent of each row, then read "Version-sensitive parts" below.

Recorded on 2026-10-25 from the machine that produced
`quad_nmpc_ros_cpp/post_analysis/comparison.md`.

## Tested platform

| Component | Version |
|---|---|
| OS | Ubuntu 22.04.5 LTS (jammy), x86_64, kernel 6.8 |
| Compiler | gcc / g++ 11.4.0 |
| CMake | 3.22.1 |
| ROS 2 | Humble (`ros-humble-desktop` 0.10.0) |
| Python | 3.10.20, conda env (conda 25.7.0) |

## System packages (apt)

| Package | Version | Needed by |
|---|---|---|
| `ros-humble-desktop` | 0.10.0 | everything (`ros-base` 0.10.0 is enough without rviz) |
| `ros-humble-rclcpp` | 16.0.19 | C++ nodes, AcroMode plugin |
| `ros-humble-rclpy` | 3.3.21 | `sim_node`, Python NMPC (`scripts/quad_nmpc_ros`) |
| `ros-humble-actuator-msgs` | 0.0.1 | `/motor_cmd` (not in desktop: install it) |
| `ros-humble-nav-msgs` | 4.9.1 | `/odom` |
| `ros-humble-geometry-msgs` | 4.9.1 | `/nmpc/body_rate_ref`, msgs |
| `ros-humble-std-msgs` | 4.9.1 | msgs |
| `ros-humble-std-srvs` | 4.9.1 | AcroMode payload service, converter |
| `ros-humble-sensor-msgs` | 4.9.1 | `mujoco_ros_utils` |
| `ros-humble-tf2-ros` | 0.25.20 | `mujoco_ros_utils` |
| `ros-humble-visualization-msgs` | 4.9.1 | `mujoco_ros_utils` |
| `ros-humble-builtin-interfaces` | 1.2.3 | msgs |
| `ros-humble-rosidl-default-generators` / `-runtime` | 1.2.1 | `quadrotor_msgs`, `mujoco_ros_utils` msgs |
| `ros-humble-ament-cmake` | 1.3.14 | all C++ packages |
| `libglfw3-dev` | 3.3.6 | `mujoco_ros_utils` (`find_package(glfw3 3.3)`) |
| `libeigen3-dev` | 3.4.0 | AcroMode |
| `python3-colcon-common-extensions` | 0.3.0 | build (or the pip one below) |

```bash
sudo apt install ros-humble-desktop ros-humble-actuator-msgs ros-humble-std-srvs \
  ros-humble-tf2-ros ros-humble-visualization-msgs libglfw3-dev libeigen3-dev
```

## acados (built from source)

| Item | Value |
|---|---|
| Repo | https://github.com/acados/acados |
| Commit | `53eaec3d762b36a9cd2f106273ceafdfeba78c8a` (2026-03-04) |
| `acados_template` (Python) | 0.5.1, installed editable from `<acados>/interfaces/acados_template` |
| CMake options | `CMAKE_BUILD_TYPE=Release`, `ACADOS_WITH_QPOASES=ON`, `BLASFEO_TARGET=X64_AUTOMATIC`, `HPIPM_TARGET=GENERIC` |
| `t_renderer` | present in `<acados>/bin` (acados_template downloads it on first use) |
| Environment | `ACADOS_SOURCE_DIR=<acados>`, `$ACADOS_SOURCE_DIR/lib` on `LD_LIBRARY_PATH` |

```bash
git clone https://github.com/acados/acados.git ~/acados && cd ~/acados
git checkout 53eaec3d762b36a9cd2f106273ceafdfeba78c8a
git submodule update --recursive --init
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DACADOS_WITH_QPOASES=ON
make install -j4
pip install -e ~/acados/interfaces/acados_template
```

## Python environment (one interpreter for everything)

| Package | Version | Needed by |
|---|---|---|
| python | 3.10.20 | must match the ROS 2 distro's Python (Humble: 3.10) |
| `mujoco` (pip) | 3.6.0 | simulator AND the AcroMode plugin build (headers + `libmujoco.so.3.6.0`) |
| `casadi` | 3.7.2 | Python NMPC, solver generation |
| `numpy` | 2.2.6 | everything Python |
| `acados_template` | 0.5.1 | Python NMPC, `scripts/regenerate.sh` |
| `empy` | **3.3.4** | Humble message generation. empy 4.x fails: `module 'em' has no attribute 'BUFFERED_OPT'` |
| `lark` | 1.3.1 | Humble message generation (`rosidl_parser`) |
| `catkin-pkg` | 1.1.1 | colcon |
| `colcon-common-extensions` | 0.3.0 (`colcon-core` 0.21.3) | build, if built from the conda env |
| `setuptools` | 79.0.1 | colcon (the Python NMPC is installed by `ament_cmake_python`) |
| `matplotlib` | 3.10.6 | `plot_log`, `post_analysis/scripts/pa.py` |
| `PyYAML` | 6.0.3 | `post_analysis/scripts/pa.py` |

```bash
conda create -n All_Drones python=3.10 && conda activate All_Drones
pip install "mujoco==3.6.0" "casadi==3.7.2" "numpy==2.2.6" "empy==3.3.4" "lark==1.3.1" \
  "catkin-pkg==1.1.1" "colcon-common-extensions==0.3.0" "matplotlib==3.10.6" "PyYAML==6.0.3"
pip install -e ~/acados/interfaces/acados_template
```

## Version-sensitive parts (check these first when porting)

| Part | Why it is sensitive | What to do on another version |
|---|---|---|
| `quad_nmpc_ros_cpp/acados_generated/` | C code emitted by acados at the commit above; links against that acados's C API | Different acados: run `quad_nmpc_ros_cpp/scripts/regenerate.sh`, rebuild |
| AcroMode plugin vs `mujoco` | Built against the headers of the pip `mujoco` the simulator imports; plugin API changes between MuJoCo releases. CMake requires >= 2.3.5 (`mujoco_ros_utils/CMakeLists.txt:28`). Only 3.6.0 is tested | Always build with `-DMUJOCO_ROOT_DIR=$(python -c "import mujoco, os; print(os.path.dirname(mujoco.__file__))")`; then `ldd` check from the README |
| `empy` / `lark` | Humble's rosidl templates use the empy 3 API | Other distros may differ: use the empy version that distro's rosidl expects (verify) |
| Python version | `rclpy` is built for the distro's Python (Humble 3.10, Jazzy 3.12) | Create the conda env with that exact Python version |
| `ament_target_dependencies` | Used in every C++ `CMakeLists.txt`; deprecated in newer ROS 2 releases | Works on Humble; on newer distros expect warnings, or switch to `target_link_libraries(... <pkg>::<target>)` |
| `actuator_msgs` | Separate package, not part of desktop | Check it exists for the target distro (`apt search ros-<distro>-actuator-msgs`) |
| numpy 2.x | Humble's `rosidl_generator_py` was built against numpy 1.x; 2.2.6 worked here for C++ users of the msgs | If a Python node fails to import generated msgs, try numpy 1.26 |
