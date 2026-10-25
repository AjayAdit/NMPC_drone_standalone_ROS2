add_library(
  MujocoRosUtils SHARED
  ClockPublisher.cpp PosePublisher.cpp ImagePublisher.cpp ActuatorCommand.cpp
  ExternalForce.cpp SensorPublisher.cpp WrenchToActuators.cpp OdometryPublisher.cpp ImuPublisher.cpp AcroMode.cpp)
target_include_directories(
  MujocoRosUtils PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                        $<INSTALL_INTERFACE:include>)
target_include_directories(MujocoRosUtils PRIVATE ${MUJOCO_INCLUDE_DIR})
target_include_directories(MujocoRosUtils PUBLIC ${ament_INCLUDE_DIRS})
ament_target_dependencies(
  MujocoRosUtils
  PUBLIC
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
  tf2_ros
  sensor_msgs
  nav_msgs
  builtin_interfaces
  Eigen3
  quadrotor_msgs
  std_srvs
  )
add_dependencies(MujocoRosUtils ${PROJECT_NAME})
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME}
                              "rosidl_typesupport_cpp")
target_link_libraries(MujocoRosUtils PUBLIC ${cpp_typesupport_target}
                                            ${ament_LIBRARIES})
target_link_libraries(MujocoRosUtils PUBLIC ${LIB_MUJOCO} glfw)

add_library(MujocoRosUtilsPlugin SHARED Plugin.cpp)
target_include_directories(MujocoRosUtilsPlugin PRIVATE ${MUJOCO_INCLUDE_DIR})
target_link_libraries(MujocoRosUtilsPlugin PUBLIC MujocoRosUtils)
# $ORIGIN: the plugin finds libMujocoRosUtils.so next to itself.
# MUJOCO_LIB_REAL_DIR: the SAME libmujoco the Python bindings load.
set_target_properties(
  MujocoRosUtils MujocoRosUtilsPlugin
  PROPERTIES INSTALL_RPATH "$ORIGIN;${MUJOCO_LIB_REAL_DIR}")

# Into this package's install/ tree, never into the MuJoCo (site-packages) dir.
install(
  TARGETS MujocoRosUtils MujocoRosUtilsPlugin
  EXPORT "${TARGETS_EXPORT_NAME}"
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")
