#include "quad_nmpc_ros_cpp/nmpc_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

extern "C" {
#include "acados_c/ocp_nlp_interface.h"
#include "acados_solver_quad_13state_drone.h"
}

namespace quad_nmpc_ros_cpp
{

// Everything acados lives here, so the public header stays free of C headers.
struct NmpcController::Impl
{
  quad_13state_drone_solver_capsule * capsule {nullptr};
  ocp_nlp_config * config {nullptr};
  ocp_nlp_dims * dims {nullptr};
  ocp_nlp_in * in {nullptr};
  ocp_nlp_out * out {nullptr};

  // Baseline reference, rebuilt per node by setTargetTrajectory. The input
  // slots hold the hover command so that simply hovering costs nothing; the
  // trailing slot is the orbiting term (body-frame lateral velocity), whose
  // reference is zero -- that is what asks the nose to follow the velocity.
  std::array<double, kNy> yref {};
  std::array<double, kNyN> yref_e {};
};

NmpcController::NmpcController(double mass, double gravity, double k_tau, double dt)
: impl_(std::make_unique<Impl>()), dt_(dt)
{
  impl_->capsule = quad_13state_drone_acados_create_capsule();
  if (impl_->capsule == nullptr) {
    throw std::runtime_error("acados: could not allocate the solver capsule");
  }
  const int rc = quad_13state_drone_acados_create(impl_->capsule);
  if (rc != 0) {
    throw std::runtime_error(
            "acados: solver creation failed with status " + std::to_string(rc));
  }

  impl_->config = quad_13state_drone_acados_get_nlp_config(impl_->capsule);
  impl_->dims = quad_13state_drone_acados_get_nlp_dims(impl_->capsule);
  impl_->in = quad_13state_drone_acados_get_nlp_in(impl_->capsule);
  impl_->out = quad_13state_drone_acados_get_nlp_out(impl_->capsule);

  // Same expression as the Python: the per-motor normalised command that
  // exactly cancels weight, since thrust = k_tau * u^2 per motor.
  u_hover_ = std::sqrt(mass * gravity / (4.0 * k_tau));

  // The model carries 3 online parameters: the target's acceleration
  // feedforward, which this trajectory-tracking build never uses. acados does
  // NOT zero them, so leaving them unset feeds uninitialised memory straight
  // into the dynamics and every solve returns ACADOS_MINSTEP. The Python sets
  // them on every call; they never change, so once at construction is enough.
  double p_zero[3] = {0.0, 0.0, 0.0};
  for (int k = 0; k <= kN; ++k) {
    quad_13state_drone_acados_update_params(impl_->capsule, k, p_zero, 3);
  }

  // Seed the horizon with a valid pose before the first solve. acados leaves
  // nlp_out at zeros, and an all-zero quaternion is not a rotation -- every
  // solve then returns ACADOS_MINSTEP, from the first tick onward. x0 is the
  // origin, level, stationary: zeros with qw = 1.
  State x0 {};
  x0[6] = 1.0;
  for (int k = 0; k <= kN; ++k) {
    ocp_nlp_out_set(impl_->config, impl_->dims, impl_->out, impl_->in, k, "x", x0.data());
  }

  // ...and seed the inputs at hover. Leaving them at zero means the predicted
  // trajectory is a free fall, which runs straight into the z >= -0.4 state
  // bound and hands HPIPM an infeasible QP -- status 4 on every tick, with
  // u = 0 coming back out. There is no input at the terminal node.
  Input u_init;
  u_init.fill(u_hover_);
  for (int k = 0; k < kN; ++k) {
    ocp_nlp_out_set(impl_->config, impl_->dims, impl_->out, impl_->in, k, "u", u_init.data());
  }

  impl_->yref.fill(0.0);
  impl_->yref_e.fill(0.0);
  impl_->yref[6] = 1.0;        // qw of the identity quaternion
  impl_->yref_e[6] = 1.0;
  for (int i = 0; i < kNu; ++i) {
    impl_->yref[kNx + i] = u_hover_;
  }
}

NmpcController::~NmpcController()
{
  if (impl_ && impl_->capsule != nullptr) {
    quad_13state_drone_acados_free(impl_->capsule);
    quad_13state_drone_acados_free_capsule(impl_->capsule);
  }
}

void NmpcController::setTargetTrajectory(
  const std::vector<Vec3> & positions,
  const std::vector<Vec3> & velocities,
  const std::vector<double> & yaws)
{
  const std::size_t want = static_cast<std::size_t>(kN) + 1;
  if (positions.size() != want || velocities.size() != want || yaws.size() != want) {
    throw std::invalid_argument(
            "setTargetTrajectory needs exactly " + std::to_string(want) + " entries");
  }

  for (int k = 0; k <= kN; ++k) {
    // Yaw-only attitude: a rotation of psi about world z is (cos(psi/2), 0, 0,
    // sin(psi/2)). psi is NOT wrapped here on purpose -- psi and psi + 2*pi are
    // the same heading but opposite quaternions, and the cost is quadratic on
    // the components, so a wrap would read as a 180 degree error. The caller
    // keeps psi continuous.
    const double qw = std::cos(yaws[k] / 2.0);
    const double qz = std::sin(yaws[k] / 2.0);

    if (k < kN) {
      auto ref = impl_->yref;                 // copy keeps the hover inputs
      std::copy(positions[k].begin(), positions[k].end(), ref.begin());
      std::copy(velocities[k].begin(), velocities[k].end(), ref.begin() + 3);
      ref[6] = qw;
      ref[9] = qz;
      ocp_nlp_cost_model_set(impl_->config, impl_->dims, impl_->in, k, "yref", ref.data());
    } else {
      auto ref_e = impl_->yref_e;             // 13 long: no input slots
      std::copy(positions[k].begin(), positions[k].end(), ref_e.begin());
      std::copy(velocities[k].begin(), velocities[k].end(), ref_e.begin() + 3);
      ref_e[6] = qw;
      ref_e[9] = qz;
      ocp_nlp_cost_model_set(impl_->config, impl_->dims, impl_->in, kN, "yref", ref_e.data());
    }
  }
}

NmpcController::Input NmpcController::computeAction(const State & x_meas)
{
  // Pin the initial state by collapsing its box constraint onto a point.
  // acados has no separate "set x0" call; lbx == ubx at stage 0 is the idiom.
  State x = x_meas;
  ocp_nlp_constraints_model_set(
    impl_->config, impl_->dims, impl_->in, impl_->out, 0, "lbx", x.data());
  ocp_nlp_constraints_model_set(
    impl_->config, impl_->dims, impl_->in, impl_->out, 0, "ubx", x.data());

  status_ = quad_13state_drone_acados_solve(impl_->capsule);

  Input u {};
  ocp_nlp_out_get(impl_->config, impl_->dims, impl_->out, 0, "u", u.data());
  for (auto & ui : u) {
    ui = std::clamp(ui, 0.0, 1.0);
  }
  return u;
}

}  // namespace quad_nmpc_ros_cpp
