#include "fdt/aero.hpp"

#include <cmath>

namespace fdt {

AeroModel::AeroModel(const Aero& config) : config_(config) {}

Wrench AeroModel::wrench(const Eigen::Vector3d& velocity_body, const Eigen::Vector3d& angular_velocity,
                         double total_thrust) const {
  const Eigen::Vector3d& lin = config_.linear_drag.value;
  const Eigen::Vector3d& quad = config_.quadratic_drag.value;

  Wrench w;
  for (int i = 0; i < 3; ++i) {
    const double v = velocity_body[i];
    // v*|v| rather than v^2 so the force reverses with the flow.
    w.force[i] = -(lin[i] * v + quad[i] * v * std::abs(v));
  }

  // Induced drag: the rotor disc tilts its thrust vector back as it is flown
  // through the air, costing in-plane force in proportion to thrust and
  // in-plane airspeed. This is why a real quad sinks in fast forward flight.
  const double k = config_.induced_drag_k.value;
  if (k > 0.0 && total_thrust > 0.0) {
    w.force.x() -= k * total_thrust * velocity_body.x();
    w.force.y() -= k * total_thrust * velocity_body.y();
  }

  w.torque = -(config_.angular_damping.value.array() * angular_velocity.array()).matrix();
  return w;
}

}  // namespace fdt
