#include "OdometryPublisher.h"
#include <mujoco/mujoco.h>
#include <cstring>
#include <iostream>

namespace MujocoRosUtils
{

// Static helpers to prevent "nstate is null" engine errors
static int odom_nstate(const mjModel* m, int instance_id) { return 0; }
static int odom_nsensordata(const mjModel* m, int instance_id, int sensor_id) { return 0; }

// ---------------- Registration ----------------
void OdometryPublisher::RegisterPlugin()
{
  mjpPlugin plugin;
  mjp_defaultPlugin(&plugin);

  plugin.name = "MujocoRosUtils::OdometryPublisher";
  plugin.capabilityflags |= mjPLUGIN_SENSOR;
  plugin.needstage = mjSTAGE_VEL;

  // Added "topic_prefix" to the attributes list
  static const char* attributes[] = {
    "frame_id",
    "odom_topic_name",
    "publish_rate",
    "child_frame_id",
    "topic_prefix"
  };
  plugin.nattribute = int(sizeof(attributes)/sizeof(attributes[0]));
  plugin.attributes = attributes;

  // Use static functions instead of lambdas to ensure pointers are not null
  plugin.nstate = odom_nstate;
  plugin.nsensordata = odom_nsensordata;

  plugin.init = +[](const mjModel* m, mjData* d, int plugin_id)
  {
    auto* inst = OdometryPublisher::Create(m, d, plugin_id);
    if (!inst) return -1;
    d->plugin_data[plugin_id] = reinterpret_cast<uintptr_t>(inst);
    return 0;
  };

  plugin.destroy = +[](mjData* d, int plugin_id)
  {
    delete reinterpret_cast<OdometryPublisher*>(d->plugin_data[plugin_id]);
    d->plugin_data[plugin_id] = 0;
  };

  plugin.reset = +[](const mjModel* m, double*, void* plugin_data, int plugin_id)
  {
    reinterpret_cast<OdometryPublisher*>(plugin_data)->reset(m, plugin_id);
  };

  plugin.compute = +[](const mjModel* m, mjData* d, int plugin_id, int)
  {
    reinterpret_cast<OdometryPublisher*>(d->plugin_data[plugin_id])->compute(m, d, plugin_id);
  };

  mjp_registerPlugin(&plugin);
}

// ---------------- Factory ----------------
OdometryPublisher* OdometryPublisher::Create(const mjModel* m, mjData* d, int plugin_id)
{
  std::string frame_id = "";
  if (const char* c = mj_getPluginConfig(m, plugin_id, "frame_id"); c && std::strlen(c) > 0)
    frame_id = c;

  std::string odom_topic_name = "";
  if (const char* c = mj_getPluginConfig(m, plugin_id, "odom_topic_name"); c && std::strlen(c) > 0)
    odom_topic_name = c;

  std::string child_frame_id = "";
  if (const char* c = mj_getPluginConfig(m, plugin_id, "child_frame_id"); c && std::strlen(c) > 0)
    child_frame_id = c;

  // Extract the new topic_prefix
  std::string topic_prefix = "";
  if (const char* c = mj_getPluginConfig(m, plugin_id, "topic_prefix"); c && std::strlen(c) > 0)
    topic_prefix = c;

  mjtNum publish_rate = 30.0;
  if (const char* c = mj_getPluginConfig(m, plugin_id, "publish_rate"); c && std::strlen(c) > 0)
    publish_rate = strtod(c, nullptr);

  // Find sensor logic
  int sensor_id = 0;
  for (; sensor_id < m->nsensor; ++sensor_id) {
    if (m->sensor_type[sensor_id] == mjSENS_PLUGIN && m->sensor_plugin[sensor_id] == plugin_id) break;
  }

  return new OdometryPublisher(m, d, sensor_id, frame_id, odom_topic_name, publish_rate, child_frame_id, topic_prefix);
}

OdometryPublisher::OdometryPublisher(const mjModel* m,
                                     mjData*,
                                     int sensor_id,
                                     const std::string& frame_id,
                                     const std::string& odom_topic_name,
                                     mjtNum publish_rate,
                                     const std::string& child_frame_id,
                                     const std::string& topic_prefix)
: sensor_id_(sensor_id),
  body_id_(m->sensor_objid[sensor_id]),
  frame_id_(frame_id),
  odom_topic_name_(odom_topic_name),
  child_frame_id_(child_frame_id),
  topic_prefix_(topic_prefix)
{
  const char* raw_body_name = mj_id2name(m, mjOBJ_XBODY, body_id_);
  std::string body_name = (raw_body_name) ? std::string(raw_body_name) : "unnamed_body";

  // Logic: if prefix is provided, result is "prefix/body_name/odom"
  if (odom_topic_name_.empty()) {
    if (!topic_prefix_.empty()) {
        odom_topic_name_ = topic_prefix_ + "/" + body_name + "/odom";
    } else {
        odom_topic_name_ = body_name + "/odom";
    }
  }

  if (child_frame_id_.empty()) child_frame_id_ = body_name;
  if (frame_id_.empty()) frame_id_ = "world";

  publish_skip_ = std::max(static_cast<int>(1.0 / (publish_rate * m->opt.timestep)), 1);

  if (!rclcpp::ok()) { 
    int argc = 0; char** argv = nullptr; 
    rclcpp::init(argc, argv); 
  }
  
  // Create a unique node name to avoid collision in Multi-Drone scenes
  auto node_name = std::string("mujoco_odom_") + (topic_prefix_.empty() ? "" : topic_prefix_ + "_") + body_name;
  nh_ = rclcpp::Node::make_shared(node_name);
  odom_pub_ = nh_->create_publisher<nav_msgs::msg::Odometry>(odom_topic_name_, 10);
}

void OdometryPublisher::reset(const mjModel*, int) { sim_cnt_ = 0; }

void OdometryPublisher::compute(const mjModel* m, mjData* d, int)
{
  ++sim_cnt_;
  if (sim_cnt_ % publish_skip_ != 0) return;

  const rclcpp::Time stamp_now = nh_->get_clock()->now();
  const int bi = body_id_;

  // MuJoCo quaternion order is (w,x,y,z)
  Eigen::Vector4d q_curr;
  q_curr << d->xquat[4*bi + 0], d->xquat[4*bi + 1], d->xquat[4*bi + 2], d->xquat[4*bi + 3];
  Eigen::Matrix3d R = quatToRot(q_curr);

  mjtNum vel_local[6];
  mj_objectVelocity(m, d, mjOBJ_XBODY, bi, vel_local, 0);

  Eigen::Vector3d w_world(vel_local[0], vel_local[1], vel_local[2]);
  Eigen::Vector3d w_body = R.transpose() * w_world;

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = stamp_now;
  odom.header.frame_id = frame_id_;
  odom.child_frame_id = child_frame_id_;

  odom.pose.pose.position.x = d->xpos[3*bi + 0];
  odom.pose.pose.position.y = d->xpos[3*bi + 1];
  odom.pose.pose.position.z = d->xpos[3*bi + 2];
  odom.pose.pose.orientation.w = q_curr(0);
  odom.pose.pose.orientation.x = q_curr(1);
  odom.pose.pose.orientation.y = q_curr(2);
  odom.pose.pose.orientation.z = q_curr(3);

  odom.twist.twist.angular.x = w_body.x();
  odom.twist.twist.angular.y = w_body.y();
  odom.twist.twist.angular.z = w_body.z();
  odom.twist.twist.linear.x  = vel_local[3];
  odom.twist.twist.linear.y  = vel_local[4];
  odom.twist.twist.linear.z  = vel_local[5];

  odom_pub_->publish(odom);
}

Eigen::Matrix3d OdometryPublisher::quatToRot(const Eigen::Vector4d& q) {
    Eigen::Quaterniond eq(q(0), q(1), q(2), q(3));
    return eq.toRotationMatrix();
}

} // namespace MujocoRosUtils
