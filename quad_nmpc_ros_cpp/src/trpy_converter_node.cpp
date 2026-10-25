// Joins the NMPC's two outputs into the one command AcroMode takes.
//
// /motor_cmd (u0..u3) and /nmpc/body_rate_ref (omega at node 1) come from the
// same solve and carry the same stamp. Pair them on that stamp, turn the four
// motor commands into one collective thrust, publish a TRPYCommand.

#include <memory>
#include <optional>
#include <string>
#include <vector>
#include <cmath>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "actuator_msgs/msg/actuators.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "quadrotor_msgs/msg/trpy_command.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace quad_nmpc_ros_cpp
{

class TrpyConverterNode : public rclcpp::Node
{
public:
TrpyConverterNode()
: rclcpp::Node("trpy_converter_node")
{
  k_tau_ = declare_parameter<double>("k_tau", 12.0);
  output_topic_ = declare_parameter<std::string>("output_topic", "/quadrotor/trpy_cmd");
  call_payload_service_ = declare_parameter<bool>("call_payload_service", false);

  trpy_pub_ = create_publisher<quadrotor_msgs::msg::TRPYCommand>(output_topic_, 10);

  cmd_sub_ = create_subscription<actuator_msgs::msg::Actuators>(
    "/motor_cmd", 10,
    [this](actuator_msgs::msg::Actuators::ConstSharedPtr msg) {onMotorCmd(*msg);});
  rate_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
    "/nmpc/body_rate_ref", 10,
    [this](geometry_msgs::msg::Vector3Stamped::ConstSharedPtr msg) {onRateRef(*msg);});

  if (call_payload_service_) {activatePayloadMode();}

  RCLCPP_INFO(
    get_logger(), "k_tau=%.2f  publishing TRPYCommand on %s",
    k_tau_, output_topic_.c_str());
}

private:
  void onMotorCmd(const actuator_msgs::msg::Actuators & msg)
  {
    if (msg.normalized.size()!=4) {
        RCLCPP_WARN_THROTTLE(
            get_logger(),*get_clock(),1000,
            "/motor_cmd has %zu entries, expected 4; dropping it",msg.normalized.size());
        return;
    }
    last_cmd_ =msg;
    tryPublish();
  }

  void onRateRef(const geometry_msgs::msg::Vector3Stamped & msg)
  {
    last_rate_ = msg;
    tryPublish();
  }

  // Publishes only when both halves of the same solve are here.
  void tryPublish()
  {
    if (!last_cmd_ || !last_rate_) {return;}         // other half not here yet

    // Different stamps: one message was lost. Do nothing -- the next arrival
    // overwrites the older half, so the pair re-forms on its own.
    if (last_cmd_->header.stamp != last_rate_->header.stamp) {return;}

    quadrotor_msgs::msg::TRPYCommand trpy;
    trpy.header.stamp = last_cmd_->header.stamp;
    trpy.header.frame_id = "base_link";              // thrust and rates are body frame
    trpy.thrust = thrustFrom(last_cmd_->normalized);
    trpy.angular_velocity = last_rate_->vector;
    trpy_pub_->publish(trpy);

    // Each half is used once; a stale u must never pair with a newer omega.
    last_cmd_.reset();
    last_rate_.reset();
  }

  // Collective thrust in N: sum of k_tau * u_i^2 over the four motors.
  double thrustFrom(const std::vector<double> & u) const
  {
    const double u0 = u[0];
    const double u1 = u[1];
    const double u2 = u[2];
    const double u3 = u[3];
    const double T = k_tau_ * u0*u0 + k_tau_ *u1*u1 + k_tau_ *u2*u2 + k_tau_ * u3*u3;
    return T;
  }

  void activatePayloadMode()
  {
    payload_client_ = create_client<std_srvs::srv::SetBool>(
      "/quadrotor/activate_payload_nmpc_controller");

    // AcroMode lives inside the simulator, which may start after us: retry
    // once a second until the service shows up, then send exactly once.
    payload_timer_ = create_wall_timer(
      std::chrono::seconds(1), [this]() {
        if (!payload_client_->service_is_ready()) {
          RCLCPP_INFO(get_logger(), "waiting for the AcroMode payload service...");
          return;
        }
        payload_timer_->cancel();

        auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
        req->data = true;
        payload_client_->async_send_request(
          req, [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture f) {
            RCLCPP_INFO(get_logger(), "AcroMode replied: %s", f.get()->message.c_str());
          });
      });
  }
  double k_tau_ {};
  std::string output_topic_;
  bool call_payload_service_ {false};
  rclcpp::TimerBase::SharedPtr payload_timer_;


  std::optional<actuator_msgs::msg::Actuators> last_cmd_;
  std::optional<geometry_msgs::msg::Vector3Stamped> last_rate_;

  rclcpp::Subscription<actuator_msgs::msg::Actuators>::SharedPtr cmd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr rate_sub_;
  rclcpp::Publisher<quadrotor_msgs::msg::TRPYCommand>::SharedPtr trpy_pub_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr payload_client_;
};

}  // namespace quad_nmpc_ros_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<quad_nmpc_ros_cpp::TrpyConverterNode>());
  if (rclcpp::ok()) {rclcpp::shutdown();}
  return 0;
}
