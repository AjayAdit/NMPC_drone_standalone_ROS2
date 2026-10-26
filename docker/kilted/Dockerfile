# ROS 2 Kilted (Ubuntu 24.04) image for this repo: clones it and builds it,
# so a passing build here means a fresh clone builds on Kilted.
#
#   docker build -t nmpc-kilted docker/kilted                      # branch main
#   docker build -t nmpc-kilted --build-arg REPO_REF=<sha> docker/kilted
#   docker/kilted/run.sh                                          # shell with GPU + X11
#
# Versions follow DEPENDENCIES.md where Kilted allows; the differences are noted inline.
FROM osrf/ros:kilted-desktop

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# --- apt: ROS extras + plugin build deps. empy 3.3.4 and lark come from apt, so they
#     match Kilted's rosidl (no pip pinning needed, unlike the Humble conda setup).
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-kilted-actuator-msgs ros-kilted-std-srvs ros-kilted-tf2-ros \
      ros-kilted-visualization-msgs \
      libglfw3-dev libeigen3-dev \
      python3-venv python3-pip git build-essential cmake \
 && rm -rf /var/lib/apt/lists/*

# --- acados, same commit as the Humble machine: the committed acados_generated/ C code
#     was emitted by it.
ARG ACADOS_COMMIT=53eaec3d762b36a9cd2f106273ceafdfeba78c8a
ENV ACADOS_SOURCE_DIR=/opt/acados
RUN git clone https://github.com/acados/acados.git ${ACADOS_SOURCE_DIR} \
 && cd ${ACADOS_SOURCE_DIR} && git checkout ${ACADOS_COMMIT} \
 && git submodule update --recursive --init \
 && mkdir -p build && cd build \
 && cmake .. -DCMAKE_BUILD_TYPE=Release -DACADOS_WITH_QPOASES=ON \
 && make install -j"$(nproc)"
ENV LD_LIBRARY_PATH=${ACADOS_SOURCE_DIR}/lib

# --- Python: a venv ON TOP of the system site-packages, so rclpy (built for the system
#     Python 3.12) and the apt numpy 1.26 stay visible. numpy<2 keeps pip from pulling a
#     second numpy that the ROS-generated message modules were not built against.
#     (Humble machine: conda, numpy 2.2.6.)
ENV VIRTUAL_ENV=/opt/venv
RUN python3 -m venv --system-site-packages ${VIRTUAL_ENV} \
 && ${VIRTUAL_ENV}/bin/pip install --no-cache-dir "numpy<2" \
      "mujoco==3.6.0" "casadi==3.7.2" matplotlib pyyaml \
 && ${VIRTUAL_ENV}/bin/pip install --no-cache-dir -e ${ACADOS_SOURCE_DIR}/interfaces/acados_template \
 && echo y | ${VIRTUAL_ENV}/bin/python -c \
      "from acados_template.utils import get_tera; print(get_tera())"
# ^ t_renderer (acados's code-gen template tool) into $ACADOS_SOURCE_DIR/bin now: the
#   Python NMPC needs it on first run, and acados would otherwise ask y/N on stdin.
ENV PATH=${VIRTUAL_ENV}/bin:${PATH}

# --- the repo, exactly as a stranger would get it
ARG REPO_URL=https://github.com/AjayAdit/NMPC_drone_standalone_ROS2.git
ARG REPO_REF=main
WORKDIR /ws
RUN git clone ${REPO_URL} src/quad_nmpc_ros \
 && cd src/quad_nmpc_ros && git checkout ${REPO_REF} && git log -1 --oneline

# python -m colcon (venv python), so the generated launchers use the venv interpreter.
RUN source /opt/ros/kilted/setup.bash \
 && MUJOCO_ROOT_DIR=$(python -c "import mujoco, os; print(os.path.dirname(mujoco.__file__))") \
 && python -m colcon build --symlink-install \
      --cmake-args -DMUJOCO_ROOT_DIR=${MUJOCO_ROOT_DIR} -DCMAKE_BUILD_TYPE=Release \
      2>&1 | tee /ws/build.log \
 && test "${PIPESTATUS[0]}" -eq 0

RUN echo 'source /opt/ros/kilted/setup.bash && source /ws/install/setup.bash' >> /root/.bashrc
CMD ["bash"]
