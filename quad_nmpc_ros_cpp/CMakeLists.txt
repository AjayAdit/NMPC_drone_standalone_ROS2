cmake_minimum_required(VERSION 3.8)
project(quad_nmpc_ros_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(actuator_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# ---------------------------------------------------------------------------
# acados
#
# ACADOS_SOURCE_DIR must point at an acados build tree. The same variable the
# Python NMPC (scripts/quad_nmpc_ros) needs, so if one runs, so does the other.
# ---------------------------------------------------------------------------
if(DEFINED ENV{ACADOS_SOURCE_DIR})
  set(ACADOS_DIR $ENV{ACADOS_SOURCE_DIR})
else()
  message(FATAL_ERROR
    "ACADOS_SOURCE_DIR is not set. Point it at your acados tree, e.g.\n"
    "  export ACADOS_SOURCE_DIR=$ENV{HOME}/acados")
endif()
if(NOT EXISTS ${ACADOS_DIR}/lib/libacados.so)
  message(FATAL_ERROR "no libacados.so under ${ACADOS_DIR}/lib -- is acados built?")
endif()

# ---------------------------------------------------------------------------
# The generated solver
#
# acados_generated/ is committed. It is emitted by acados_template, which is
# Python-only, so this package cannot regenerate it -- it only consumes it.
# Re-emit it with scripts/regenerate.sh after changing the model or the cost.
# ---------------------------------------------------------------------------
set(GEN ${CMAKE_CURRENT_SOURCE_DIR}/acados_generated)
if(NOT EXISTS ${GEN}/acados_solver_quad_13state_drone.h)
  message(FATAL_ERROR
    "acados_generated/ is empty. Regenerate it:\n"
    "  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/regenerate.sh")
endif()
file(GLOB_RECURSE ACADOS_GEN_SRC ${GEN}/*.c)

add_library(acados_generated_solver STATIC ${ACADOS_GEN_SRC})
target_include_directories(acados_generated_solver PUBLIC
  ${GEN}
  ${ACADOS_DIR}/include
  ${ACADOS_DIR}/include/blasfeo/include
  ${ACADOS_DIR}/include/hpipm/include
  ${ACADOS_DIR}/interfaces)
target_link_libraries(acados_generated_solver
  ${ACADOS_DIR}/lib/libacados.so
  ${ACADOS_DIR}/lib/libhpipm.so
  ${ACADOS_DIR}/lib/libblasfeo.so
  m)
# Generated C is machine-written; do not hold it to our warning settings.
target_compile_options(acados_generated_solver PRIVATE -w)
set_target_properties(acados_generated_solver PROPERTIES POSITION_INDEPENDENT_CODE ON)

# ---------------------------------------------------------------------------
# The node
# ---------------------------------------------------------------------------
add_executable(nmpc_node
  src/nmpc_node.cpp
  src/nmpc_controller.cpp)
target_include_directories(nmpc_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(nmpc_node acados_generated_solver)
ament_target_dependencies(nmpc_node rclcpp nav_msgs actuator_msgs geometry_msgs)

# ---------------------------------------------------------------------------
# The converter: /motor_cmd + /nmpc/body_rate_ref -> TRPYCommand for AcroMode.
# No acados here -- it only adds up k_tau * u^2 and copies the rates.
# ---------------------------------------------------------------------------
add_executable(trpy_converter_node src/trpy_converter_node.cpp)
ament_target_dependencies(trpy_converter_node
  rclcpp actuator_msgs geometry_msgs quadrotor_msgs std_srvs)

install(TARGETS nmpc_node trpy_converter_node DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/sim_node.py
        DESTINATION lib/${PROJECT_NAME}
        RENAME sim_node)

# ---------------------------------------------------------------------------
# Python: the NMPC library and nodes in scripts/quad_nmpc_ros, importable as
# `quad_nmpc_ros`, plus thin launchers. The Python NMPC is the acados code
# generator (scripts/regenerate.sh) and a second controller to compare with.
# ---------------------------------------------------------------------------
ament_python_install_package(quad_nmpc_ros PACKAGE_DIR scripts/quad_nmpc_ros)
install(PROGRAMS scripts/nmpc_node_py.py
        DESTINATION lib/${PROJECT_NAME}
        RENAME nmpc_node_py)
install(PROGRAMS scripts/plot_log.py
        DESTINATION lib/${PROJECT_NAME}
        RENAME plot_log)
install(DIRECTORY worlds config launch DESTINATION share/${PROJECT_NAME})

ament_package()
