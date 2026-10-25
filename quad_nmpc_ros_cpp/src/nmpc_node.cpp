// Subscribes to the drone's state, publishes motor commands.
//
// A direct port of quad_nmpc_ros/nmpc_node.py. Same topics, same reference,
// same double-cover handling, same solver guard -- so either controller can
// drive either simulator, and a behaviour difference between them is a bug in
// one of the two rather than a difference in what they were asked to do.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "actuator_msgs/msg/actuators.hpp"
 #include "geometry_msgs/msg/vector3_stamped.hpp"

#include "quad_nmpc_ros_cpp/nmpc_controller.hpp"

namespace
{
constexpr double kPi = 3.14159265358979323846;
}

namespace quad_nmpc_ros_cpp
{

class NmpcNode : public rclcpp::Node
{
public:
  NmpcNode()
  : rclcpp::Node("nmpc_node")
  {
    radius_ = declare_parameter<double>("radius", 5.0);
    altitude_ = declare_parameter<double>("altitude", 1.5);
    omega_ = declare_parameter<double>("omega", 0.5);
    duration_ = declare_parameter<double>("duration", 0.0);

    const double mass = declare_parameter<double>("mass", 0.8);
    const double gravity = declare_parameter<double>("gravity", 9.81);
    const double k_tau = declare_parameter<double>("k_tau", 12.0);
    const double dt = declare_parameter<double>("dt", 0.05);

    const std::string log_file = declare_parameter<std::string>("log_file", "");

    RCLCPP_INFO(get_logger(), "creating the acados solver...");
    ctrl_ = std::make_unique<NmpcController>(mass, gravity, k_tau, dt);
    RCLCPP_INFO(
      get_logger(), "NMPC ready  (N=%d, dt=%.3f s, u_hover=%.4f)",
      ctrl_->horizon(), ctrl_->dt(), ctrl_->hoverInput());

    const std::size_t n = static_cast<std::size_t>(ctrl_->horizon()) + 1;
    positions_.resize(n);
    velocities_.resize(n);
    yaws_.resize(n);

    openLog(log_file);

    cmd_pub_ = create_publisher<actuator_msgs::msg::Actuators>("/motor_cmd", 10);
    rate_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("/nmpc/body_rate_ref", 10);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {onOdom(*msg);});
  }

  ~NmpcNode() override
  {
    closeLog();
  }

private:
  // Where the drone should be at time t, how fast it should be going, and which
  // way it should face. The heading is the path tangent, which for this circle
  // is omega*t + pi/2 -- and it climbs without bound on purpose. See onOdom().
  void reference(double t, NmpcController::Vec3 & pos, NmpcController::Vec3 & vel, double & yaw) const
  {
    pos = {radius_ * std::cos(omega_ * t), radius_ * std::sin(omega_ * t), altitude_};
    vel = {-radius_ * omega_ * std::sin(omega_ * t),
      radius_ * omega_ * std::cos(omega_ * t), 0.0};
    yaw = omega_ * t + kPi / 2.0;
  }

  void onOdom(const nav_msgs::msg::Odometry & msg)
  {
    // Sim time, phase-zeroed on the first message we ever see.
    double t = static_cast<double>(msg.header.stamp.sec) +
      static_cast<double>(msg.header.stamp.nanosec) * 1e-9;
    if (!t0_) {t0_ = t;}
    t -= *t0_;

    const auto & p = msg.pose.pose.position;
    const auto & q = msg.pose.pose.orientation;
    const auto & v = msg.twist.twist.linear;
    const auto & w = msg.twist.twist.angular;

    // ROS is scalar-last (x,y,z,w); the controller is scalar-first (w,x,y,z).
    const NmpcController::State x_meas {
      p.x, p.y, p.z,
      v.x, v.y, v.z,
      q.w, q.x, q.y, q.z,
      w.x, w.y, w.z};

    // Lock the double-cover offset ONCE. reference()'s heading climbs forever
    // and so does MuJoCo's quaternion, so a constant offset keeps the two in
    // step for good. Recomputing it per tick needs a round(), and round() steps
    // once per lap -- which shifts the reference by 2*pi, which NEGATES the
    // reference quaternion, which the solver reads as a 180 degree error.
    if (!turns_) {
      const double psi_meas = std::atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z));
      NmpcController::Vec3 dummy_p, dummy_v;
      double psi0 = 0.0;
      reference(t, dummy_p, dummy_v, psi0);
      turns_ = 2.0 * kPi * std::round((psi_meas - psi0) / (2.0 * kPi));
    }

    const int n = ctrl_->horizon();
    const double dt = ctrl_->dt();
    for (int k = 0; k <= n; ++k) {
      double psi = 0.0;
      reference(t + k * dt, positions_[k], velocities_[k], psi);
      yaws_[k] = psi + *turns_;
    }

    ctrl_->setTargetTrajectory(positions_, velocities_, yaws_);
    auto u = ctrl_->computeAction(x_meas);

    // A non-zero status means the QP did not converge and computeAction
    // returned whatever was left in node 0. Flying on the last good command
    // beats flying on garbage; in Python, publishing the garbage once put the
    // drone on the floor in under a second.
    if (ctrl_->status() != 0) {
      ++bad_solves_;
      if (!last_good_u_) {
        return;                             // nothing safe to send yet
      }
      if (bad_solves_ % 10 == 1) {
        RCLCPP_WARN(
          get_logger(), "solver status %d, holding last command (%d consecutive)",
          ctrl_->status(), bad_solves_);
      }
      u = *last_good_u_;
    } else {
      last_good_u_ = u;
      bad_solves_ = 0;
    }

    actuator_msgs::msg::Actuators cmd;
    cmd.header.stamp = msg.header.stamp;
    cmd.normalized.assign(u.begin(), u.end());
    cmd_pub_->publish(cmd);
    geometry_msgs::msg::Vector3Stamped rate ;
    rate.header.stamp =msg.header.stamp;
    rate.header.frame_id ="base_link";
    const auto w_ref =ctrl_->predictedBodyRates();
    rate.vector.x = w_ref[0];
    rate.vector.y = w_ref[1];
    rate.vector.z = w_ref[2];
    rate_pub_->publish(rate);

    const auto & ref = positions_[0];
    const double err = std::hypot(std::hypot(ref[0] - p.x, ref[1] - p.y), ref[2] - p.z);
    logRow(t, p.x, p.y, p.z, ref, err * 100.0);

    if (t >= next_report_) {
      RCLCPP_INFO(
        get_logger(), "t=%5.1fs  err=%6.1f cm  pos=(%+.2f,%+.2f,%+.2f)",
        t, err * 100.0, p.x, p.y, p.z);
      next_report_ += 1.0;
    }

    if (duration_ > 0.0 && t >= duration_) {
      RCLCPP_INFO(get_logger(), "reached duration %.1f s, stopping", duration_);
      closeLog();
      rclcpp::shutdown();
    }
  }

  // --- logging ------------------------------------------------------------
  // Written as we fly, one row per odometry message. Nothing about it depends
  // on a clean shutdown: whatever is on disk when the node dies is valid data,
  // and quad_nmpc_ros's plot_log renders it unchanged.
  void openLog(const std::string & requested)
  {
    log_path_ = requested;
    if (log_path_.empty()) {
      const char * home = std::getenv("HOME");
      log_path_ = std::string(home ? home : ".") + "/.quad_nmpc_ros/tracking_log.csv";
    }
    const auto slash = log_path_.find_last_of('/');
    if (slash != std::string::npos) {
      std::string dir = log_path_.substr(0, slash);
      std::string cmd = "mkdir -p '" + dir + "'";
      if (std::system(cmd.c_str()) != 0) {
        RCLCPP_WARN(get_logger(), "could not create %s", dir.c_str());
      }
    }
    log_ = std::fopen(log_path_.c_str(), "w");
    if (log_ == nullptr) {
      RCLCPP_WARN(get_logger(), "could not open %s; flying without a log", log_path_.c_str());
      return;
    }
    std::fprintf(log_, "t,x,y,z,ref_x,ref_y,ref_z,err_cm\n");
    RCLCPP_INFO(get_logger(), "logging to %s", log_path_.c_str());
  }

  void logRow(
    double t, double x, double y, double z,
    const NmpcController::Vec3 & ref, double err_cm)
  {
    if (log_ == nullptr) {return;}
    std::fprintf(
      log_, "%.4f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.4f\n",
      t, x, y, z, ref[0], ref[1], ref[2], err_cm);
    if (++rows_ % 50 == 0) {std::fflush(log_);}   // ~twice a second, costs nothing
  }

  void closeLog()
  {
    if (log_ == nullptr) {return;}
    std::fflush(log_);
    std::fclose(log_);
    log_ = nullptr;
    std::printf("[nmpc_node] flight log: %s\n", log_path_.c_str());
    std::printf("[nmpc_node] plot it with:  ros2 run quad_nmpc_ros plot_log\n");
    std::fflush(stdout);
  }

  std::unique_ptr<NmpcController> ctrl_;
  rclcpp::Publisher<actuator_msgs::msg::Actuators>::SharedPtr cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr rate_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  std::vector<NmpcController::Vec3> positions_, velocities_;
  std::vector<double> yaws_;

  double radius_ {}, altitude_ {}, omega_ {}, duration_ {};
  std::optional<double> t0_;
  std::optional<double> turns_;                       // double-cover offset
  std::optional<NmpcController::Input> last_good_u_;
  int bad_solves_ {0};
  double next_report_ {0.0};

  std::FILE * log_ {nullptr};
  std::string log_path_;
  long rows_ {0};
};

}  // namespace quad_nmpc_ros_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<quad_nmpc_ros_cpp::NmpcNode>());
  if (rclcpp::ok()) {rclcpp::shutdown();}
  return 0;
}
