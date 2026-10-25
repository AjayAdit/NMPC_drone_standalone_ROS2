#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mjvisualize.h>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>

#include <limits>
#include <string>

namespace MujocoRosUtils
{

/** \brief Plugin to map a ROS2 Wrench to four MuJoCo actuators (Fz, Tx, Ty, Tz),
 *         with an internal controller that runs at a fixed sim-time rate. */
class WrenchToActuators
{
public:
  /** \brief Register plugin with MuJoCo. */
  static void RegisterPlugin();

  /** \brief Create an instance.
      \param m model
      \param d data
      \param plugin_id plugin ID
   */
  static WrenchToActuators * Create(const mjModel * m, mjData * d, int plugin_id);

public:
  /** \brief Move-only. */
  WrenchToActuators(WrenchToActuators &&) = default;
  WrenchToActuators(const WrenchToActuators &) = delete;
  WrenchToActuators & operator=(const WrenchToActuators &) = delete;

  /** \brief Reset hook. */
  void reset(const mjModel * m, int plugin_id);

  /** \brief Per-step compute hook (called at mjSTAGE_VEL). */
  void compute(const mjModel * m, mjData * d, int plugin_id);

protected:
  /** \brief Constructor. */
  WrenchToActuators(const mjModel * m, mjData * d,
                    int id_fz, int id_tx, int id_ty, int id_tz,
                    std::string topic_name, double control_hz,
                    int body_id, std::string body_name);
  
  static Eigen::Matrix3d quatToRot(const Eigen::Vector4d& q_wxyz);
  static Eigen::Vector3d vee(const Eigen::Matrix3d& S);

  /** \brief ROS callback: external wrench command (optional override). */
  void callback(const quadrotor_msgs::msg::PositionCommand msg);

protected:
  // -------- Actuator IDs --------
  int id_fz_ = -1, id_tx_ = -1, id_ty_ = -1, id_tz_ = -1;

  // -------- Last commanded wrench (hold-last from ROS; NaN = no external cmd) --------
  mjtNum Fx_ = std::numeric_limits<mjtNum>::quiet_NaN();
  mjtNum Fy_ = std::numeric_limits<mjtNum>::quiet_NaN();
  mjtNum Fz_ = std::numeric_limits<mjtNum>::quiet_NaN();
  mjtNum Tx_ = std::numeric_limits<mjtNum>::quiet_NaN();
  mjtNum Ty_ = std::numeric_limits<mjtNum>::quiet_NaN();
  mjtNum Tz_ = std::numeric_limits<mjtNum>::quiet_NaN();

  // -------- State access (like PosePublisher) --------
  int         body_id_   = -1;     // which body to read/control
  std::string body_name_;

  // -------- Fixed-rate scheduler (sim time) --------
  double ctrl_dt_        = 0.0;
  double next_ctrl_time_ = 0.0;

  // -------- Diagnostics (sim-time) --------
  double control_hz_       = 0.0;
  double next_report_time_ = 0.0;
  double report_period_    = 1.0;     // seconds
  double window_start_t_   = 0.0;
  unsigned long long window_ticks_ = 0;
  double max_abs_jitter_   = 0.0;

  double last_pos_[3]  = {0,0,0};         // world position
  double last_quat_[4] = {1,0,0,0};       // world orientation (w,x,y,z)
  double last_w_[3]    = {0,0,0};         // angular vel [wx,wy,wz] (world)
  double last_v_[3]    = {0,0,0};         // linear  vel [vx,vy,vz] (world)

  // variables system
  Eigen::Matrix<double,3,1> xd_ =
    (Eigen::Matrix<double,3,1>() <<
         0.0, 0.0, 0.0             // desired position  x y z
    ).finished();

  Eigen::Matrix<double,3,1> vd_ =
    (Eigen::Matrix<double,3,1>() <<
         0.0, 0.0, 0.0             // desired velocity  vx vy vz
    ).finished();

  Eigen::Matrix<double,3,1> ad_ =
    (Eigen::Matrix<double,3,1>() <<
         0.0, 0.0, 0.0             // desired acceleration  vx vy vz
    ).finished();

  Eigen::Matrix<double,4,1> qd_ =
    (Eigen::Matrix<double,4,1>() <<
        1.0, 0.0, 0.0, 0.0             // desired quaternion  qw qx qy qz
    ).finished();

  Eigen::Matrix<double,3,1> wd_ =
    (Eigen::Matrix<double,3,1>() <<
         0.0, 0.0, 0.0             // desired angular velocity  wx wy wz
    ).finished();

  Eigen::Matrix<double,3,1> gravityVector_ =
    (Eigen::Matrix<double,3,1>() <<
         0.0, 0.0, g_             // Gravity Vector
    ).finished();

  Eigen::Matrix<double,3,1> ez_ =
    (Eigen::Matrix<double,3,1>() <<
         0.0, 0.0, 1.0             // Unit Vector z
    ).finished();

  Eigen::Matrix<double,3,1> x_ =
    (Eigen::Matrix<double,3,1>() <<
         0.0, 0.0, 0.0             // position  x y z
    ).finished();

  Eigen::Matrix<double,4,1> q_ =
    (Eigen::Matrix<double,4,1>() <<
        1.0, 0.0, 0.0, 0.0             // quaternion  qw qx qy qz
    ).finished();

  Eigen::Matrix<double,3,1> v_ =
    (Eigen::Matrix<double,3,1>() <<
         0.0, 0.0, 0.0             //velocity  vx vy vz
    ).finished();

  Eigen::Matrix<double,3,1> w_ =
    (Eigen::Matrix<double,3,1>() <<
         0.0, 0.0, 0.0             // angular velocity  wx wy wz
    ).finished();

  // Define gains of the controller
  double kp_x_{10.0};
  double kp_y_{10.0};
  double kp_z_{10.0};
  Eigen::Matrix3d KP_ = Eigen::Vector3d(kp_x_, kp_y_, kp_z_).asDiagonal();

  double kv_x_{5.0};
  double kv_y_{5.0};
  double kv_z_{5.0};
  Eigen::Matrix3d KV_ = Eigen::Vector3d(kv_x_, kv_y_, kv_z_).asDiagonal();

  double kw_x_{20.0};
  double kw_y_{20.0};
  double kw_z_{20.0};
  Eigen::Matrix3d KW_ = Eigen::Vector3d(kw_x_, kw_y_, kw_z_).asDiagonal();

  double kq_x_{250.0};
  double kq_y_{250.0};
  double kq_z_{40.0};
  Eigen::Matrix3d KQ_ = Eigen::Vector3d(kq_x_, kq_y_, kq_z_).asDiagonal();

  // Mass of the Quadrotor
  double mass_{1.0499999999999998};
  double psid_{1.57};
  double g_{9.81};

  Eigen::Matrix3d J_ = Eigen::Vector3d(0.00345398, 0.00179687, 0.00179676).asDiagonal();

  // clamp helper
  inline double clamp_to_model(const mjModel* m, int act_id, double v) const {
    const double lo = m->actuator_ctrlrange[2*act_id + 0];
    const double hi = m->actuator_ctrlrange[2*act_id + 1];
    return std::min(std::max(v, lo), hi);
  }

  // -------- ROS --------
  rclcpp::Node::SharedPtr nh_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  //rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr sub_;
  rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr sub_position_cmd_;
};

} // namespace MujocoRosUtils
