#pragma once

#include <rclcpp/rclcpp.hpp>
#include <quadrotor_msgs/msg/trpy_command.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>

#include <string>
#include <limits>
#include <memory>

namespace MujocoRosUtils
{

class AcroMode
{
public:
    static void RegisterPlugin();
    static AcroMode * Create(const mjModel * m, mjData * d, int plugin_id);

public:
    AcroMode(AcroMode &&) = default;
    AcroMode(const AcroMode &) = delete;
    AcroMode & operator=(const AcroMode &) = delete;

    void reset(const mjModel * m, int plugin_id);
    void compute(const mjModel * m, mjData * d, int plugin_id);

protected:
    AcroMode(const mjModel * m, mjData * d,
             int id_fz, int id_tx, int id_ty, int id_tz,
             std::string topic_name, double control_hz,
             int body_id, std::string body_name);

    // Callbacks
    void trpy_callback(const quadrotor_msgs::msg::TRPYCommand::SharedPtr msg);
    void trpy_payload_callback(const quadrotor_msgs::msg::TRPYCommand::SharedPtr msg);
    
    // Service Callback
    void activate_payload_callback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                  std::shared_ptr<std_srvs::srv::SetBool::Response> response);

    static Eigen::Matrix3d quatToRot(const Eigen::Vector4d& q_wxyz);

protected:
    // Actuator and Body IDs
    int id_fz_ = -1, id_tx_ = -1, id_ty_ = -1, id_tz_ = -1;
    int body_id_ = -1;
    std::string body_name_;

    // Controller State
    bool use_nmpc_payload_ = false;

    // Command Variables
    double commanded_thrust_ = 0.0;
    double hover_thrust_ = 0.0;   // m*g from the model; the command until the first TRPY arrives
    Eigen::Vector3d wd_ = Eigen::Vector3d::Zero(); // Desired angular velocity
    
    // Gains and Inertia
    double kw_x_{20.0};
    double kw_y_{35.0};
    double kw_z_{45.0};
    Eigen::Matrix3d kom_ = Eigen::Vector3d(kw_x_, kw_y_, kw_z_).asDiagonal();
    // Overwritten from the model in the constructor; this is only a fallback.
    Eigen::Matrix3d J_ = Eigen::Vector3d(0.00345398, 0.00179687, 0.00179676).asDiagonal();

    // Simulation Timing
    double ctrl_dt_ = 0.0;
    double next_ctrl_time_ = 0.0;

    // ROS 2
    rclcpp::Node::SharedPtr nh_;
    rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
    rclcpp::Subscription<quadrotor_msgs::msg::TRPYCommand>::SharedPtr sub_trpy_cmd_;
    rclcpp::Subscription<quadrotor_msgs::msg::TRPYCommand>::SharedPtr sub_trpy_payload_cmd_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_activate_payload_;
};

} // namespace MujocoRosUtils
