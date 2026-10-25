// NMPC controller: a thin C++ wrapper over the acados solver generated from
// nmpc_controller.py. Deliberately the same two-call shape as the Python class
// -- set_target_trajectory() then compute_action() -- so the two are readable
// side by side and a behaviour difference is easy to localise.
//
// The acados headers stay out of this file. A caller needs ROS and this header,
// nothing else.

#ifndef QUAD_NMPC_ROS_CPP__NMPC_CONTROLLER_HPP_
#define QUAD_NMPC_ROS_CPP__NMPC_CONTROLLER_HPP_

#include <array>
#include <memory>
#include <vector>

namespace quad_nmpc_ros_cpp
{

class NmpcController
{
public:
  // Fixed by the generated solver. Changing any of these means regenerating:
  // see scripts/regenerate.sh.
  static constexpr int kNx = 13;   // [pos(3), vel(3), quat(4), omega(3)]
  static constexpr int kNu = 4;    // normalised motor commands, [0, 1]
  static constexpr int kNy = 18;   // nx + nu + 1, the trailing slot is v_y_body
  static constexpr int kNyN = 13;  // terminal cost has no input slot
  static constexpr int kN = 20;    // horizon nodes

  using State = std::array<double, kNx>;
  using Input = std::array<double, kNu>;
  using Vec3 = std::array<double, 3>;

  // mass, gravity and k_tau set the hover input the cost is referenced to.
  // They must match the airframe the solver was generated for -- the dynamics
  // themselves are baked into the generated C, not read from here.
  NmpcController(double mass, double gravity, double k_tau, double dt);
  ~NmpcController();

  NmpcController(const NmpcController &) = delete;
  NmpcController & operator=(const NmpcController &) = delete;

  // Reference across the whole horizon. All three must hold kN + 1 entries.
  // yaw is in radians and is NOT wrapped: see the double-cover note in the
  // README before changing that.
  void setTargetTrajectory(
    const std::vector<Vec3> & positions,
    const std::vector<Vec3> & velocities,
    const std::vector<double> & yaws);

  // Pins the initial state, solves, returns node 0's input clipped to [0, 1].
  // On a failed solve the return value is whatever the solver left behind --
  // check status() before publishing it.
  Input computeAction(const State & x_meas);

  // acados status from the last computeAction(). 0 means converged.
  int status() const {return status_;}

    // omega (body frame, rad/s) at node 1 of the last CONVERGED solve.
  // Zero until the first good solve.
  Vec3 predictedBodyRates() const;

  int horizon() const {return kN;}
  double dt() const {return dt_;}
  double hoverInput() const {return u_hover_;}

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  double dt_;
  double u_hover_;
  int status_ {0};
  Vec3 omega_ref_{};
};

}  // namespace quad_nmpc_ros_cpp

#endif  // QUAD_NMPC_ROS_CPP__NMPC_CONTROLLER_HPP_
