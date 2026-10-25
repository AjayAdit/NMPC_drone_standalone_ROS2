#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>
#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mjvisualize.h>

#include <string>

namespace MujocoRosUtils
{

/** \brief Plugin to publish nav_msgs::msg::Odometry of a body. */
class OdometryPublisher
{
public:
  /** \brief Register plugin. */
  static void RegisterPlugin();

  /** \brief Create an instance. */
  static OdometryPublisher* Create(const mjModel* m, mjData* d, int plugin_id);

public:
  /** \brief Move-only. */
  OdometryPublisher(OdometryPublisher&&) = default;
  OdometryPublisher(const OdometryPublisher&) = delete;
  OdometryPublisher& operator=(const OdometryPublisher&) = delete;

  /** \brief Reset. */
  void reset(const mjModel* m, int plugin_id);

  /** \brief Compute (runs at mjSTAGE_VEL). */
  void compute(const mjModel* m, mjData* d, int plugin_id);

protected:
  /** \brief Constructor. */
  OdometryPublisher(const mjModel* m,
                    mjData* d,
                    int sensor_id,
                    const std::string& frame_id,
                    const std::string& odom_topic_name,
                    mjtNum publish_rate,
                    const std::string& child_frame_id,
                    const std::string& topic_prefix); // Added prefix

  static Eigen::Matrix3d quatToRot(const Eigen::Vector4d& q_wxyz);

protected:
  int sensor_id_ = -1;
  int body_id_   = -1;

  rclcpp::Node::SharedPtr nh_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  std::string frame_id_;
  std::string odom_topic_name_;
  std::string child_frame_id_;
  std::string topic_prefix_; // Store prefix

  int publish_skip_ = 0;
  int sim_cnt_ = 0;
};

} // namespace MujocoRosUtils
